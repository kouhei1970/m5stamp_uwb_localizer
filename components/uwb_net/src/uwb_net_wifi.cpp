/**
 * @file uwb_net_wifi.cpp
 * @brief Wi-Fi 起動・NVS の認証情報・mDNS・"wifi" コンソールコマンド
 *        (uwb_net_internal.hpp の分担: uwb_net_wifi.cpp)。
 *        Wi-Fi bring-up, NVS-backed credentials, mDNS, and the "wifi" console command.
 *
 * 状態機械（scratchpad/NET_SPEC.md §3）:
 *   mode=auto（既定）: NVS に ssid があれば STA で繋ぎに行く。20 秒たっても
 *     IP が付かない（または ssid 未設定）なら SoftAP へ切り替え、以後は
 *     再起動するまで SoftAP のまま（auto の中で STA へ戻ることはしない）。
 *   mode=sta: 常に STA。繋がらなければ 2 秒間隔で永久にリトライする。
 *   mode=ap : 常に SoftAP。
 *
 * 実装方針: Wi-Fi の状態遷移は WIFI_EVENT/IP_EVENT のイベントハンドラと
 * 2 本の esp_timer（20 秒の AP フォールバック用・2 秒の再接続用）だけで
 * 駆動する。専用タスクは持たない（イベント駆動で十分軽い処理のため）。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "esp_wifi_default.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"

#if CONFIG_UWB_NET_MDNS
#include "mdns.h"
#endif

namespace uwb::net::internal {

namespace {

constexpr const char* kTag          = "uwb_net_wifi";
constexpr const char* kNvsNamespace = "uwb_net";
constexpr const char* kKeyMode      = "mode";
constexpr const char* kKeySsid      = "ssid";
constexpr const char* kKeyPass      = "pass";

constexpr uint8_t kModeAuto = 0;
constexpr uint8_t kModeSta  = 1;
constexpr uint8_t kModeAp   = 2;

/** auto モードで STA を諦めて SoftAP へ切り替えるまでの猶予 [us]。 */
constexpr int64_t kApFallbackUs = 20LL * 1000 * 1000;
/** STA 切断後、次の再接続を試みるまでの間隔 [ms]。 */
constexpr uint32_t kReconnectIntervalMs = 2000;
/** auto モードでAPへ落ちた後、保存済みSSIDが（例えばタグの起動が遅れて）
 *  後から現れていないか定期的に探しに行く間隔 [us]（コーディネータ指示の
 *  実機不具合対応: タグ・アンカーが同時起動すると、アンカーの20秒STA待ち
 *  がタグ自身のSoftAP開設より先に切れてしまい、二度と合流できなかった）。 */
constexpr int64_t kRescanIntervalUs = 30LL * 1000 * 1000;

enum class ActiveMode : uint8_t { Off, Sta, Ap };

/* ---- 共有状態。g_mutex は文字列/フラグ類の読み書きだけを短時間守る
 * （Wi-Fi ドライバ自体のAPI呼び出しはESP-IDF側で別途スレッド安全）。 ---- */
SemaphoreHandle_t g_mutex   = nullptr;
esp_netif_t* g_staNetif      = nullptr;
esp_netif_t* g_apNetif        = nullptr;

// g_savedModeRaw/g_savedSsid/g_savedPass は複数タスク（USBシリアルREPL・
// リモートWS/TCPコンソール・rescanタスク・イベントハンドラ）から読み書き
// されるため、以下すべての箇所で g_mutex（lock()/unlock()）を通す
// （レビュー指摘 2026-08-31）。
uint8_t g_savedModeRaw = kModeAuto; //!< NVS "mode" の値（0=auto/1=sta/2=ap）。g_mutexで保護
char g_savedSsid[33]    = "";        //!< NVS "ssid"。g_mutexで保護
char g_savedPass[65]    = "";        //!< NVS "pass"（画面には絶対に出さない）。g_mutexで保護

// g_activeMode/g_connected は単純な状態フラグの読み書きしかしないので、
// g_mutexではなく std::atomic にする（レビュー指摘 2026-08-31: 以前は
// lock()/unlock()で囲っていたが、読み書きが1語で完結するため過剰だった）。
std::atomic<ActiveMode> g_activeMode{ActiveMode::Off};
std::atomic<bool> g_connected{false}; //!< STA: GOT_IP 済み / AP: 起動済み
bool g_autoSwitchedToAp         = false; //!< auto モードで一度 AP へ切り替えたら true（再起動まで維持）

esp_timer_handle_t g_apFallbackTimer = nullptr;
esp_timer_handle_t g_reconnectTimer   = nullptr;
esp_timer_handle_t g_rescanTimer       = nullptr; //!< 30秒周期。auto+AP中にだけ動かす
TaskHandle_t g_rescanTaskHandle         = nullptr; //!< "uwb_net_wifi" タスク（実際のスキャンを行う）
bool g_mdnsStarted                     = false;

/** "wifi scan" 実行中は true。実機報告(2026-08-31): esp_wifi_connect() と
 *  esp_wifi_scan_start() が同時に走ると、能動スキャンが数百msで打ち切られ
 *  APが1件も見つからなかった。このフラグが立っている間は
 *  onWifiEvent(WIFI_EVENT_STA_START)/reconnectTimerCb() が esp_wifi_connect() を
 *  呼ばず、apFallbackTimerCb() もモードを切り替えない。複数タスク（ローカル
 *  USBシリアルREPLタスクとリモートWS/TCPコンソール）から同時に読み書き
 *  されうるので std::atomic にする。 */
std::atomic<bool> g_scanInProgress{false};

/** WIFI_EVENT_STA_DISCONNECTED のログを最大2秒に1行へ間引くための直近ログ時刻
 *  [us]（esp_timer基準）。2秒の再接続タイマーと同じ周期なので、これで
 *  「切断→再接続失敗」のたびに1行出れば十分（実機診断のコーディネータ指示）。 */
int64_t g_lastDisconnectLogUs = 0;

bool lock()
{
    return (g_mutex != nullptr) && (xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE);
}

void unlock()
{
    if (g_mutex != nullptr) {
        xSemaphoreGive(g_mutex);
    }
}

const char* modeRawName(uint8_t modeRaw)
{
    switch (modeRaw) {
    case kModeSta: return "sta";
    case kModeAp:  return "ap";
    default:       return "auto";
    }
}

/** WIFI_EVENT_STA_DISCONNECTED の reason（wifi_err_reason_t、
 *  esp_wifi_types_generic.h）を短い名前にする。実機診断用
 *  （2026-08-31 実機報告: SSID は保存できても20秒でAP切替えになり、
 *  disconnectの理由が全く分からなかった問題への対応）。
 *  値は同ヘッダから拾った実際の定義（コーディネータの指示にあった
 *  "208 BEACON_TIMEOUT" は誤りで、実際は 200。208 は
 *  ASSOC_COMEBACK_TIME_TOO_LONG）。未知の値は "other"。 */
const char* wifiReasonName(uint8_t reason)
{
    switch (reason) {
    case WIFI_REASON_UNSPECIFIED:                       return "UNSPECIFIED";
    case WIFI_REASON_AUTH_EXPIRE:                        return "AUTH_EXPIRE";
    case WIFI_REASON_AUTH_LEAVE:                         return "AUTH_LEAVE";
    case WIFI_REASON_DISASSOC_DUE_TO_INACTIVITY:        return "DISASSOC_DUE_TO_INACTIVITY";
    case WIFI_REASON_CLASS2_FRAME_FROM_NONAUTH_STA:     return "CLASS2_FRAME_FROM_NONAUTH_STA";
    case WIFI_REASON_CLASS3_FRAME_FROM_NONASSOC_STA:    return "CLASS3_FRAME_FROM_NONASSOC_STA";
    case WIFI_REASON_ASSOC_LEAVE:                        return "ASSOC_LEAVE";
    case WIFI_REASON_MIC_FAILURE:                        return "MIC_FAILURE";
    case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:            return "4WAY_HANDSHAKE_TIMEOUT";
    case WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT:          return "GROUP_KEY_UPDATE_TIMEOUT";
    case WIFI_REASON_802_1X_AUTH_FAILED:                return "802_1X_AUTH_FAILED";
    case WIFI_REASON_BEACON_TIMEOUT:                     return "BEACON_TIMEOUT";       // 200
    case WIFI_REASON_NO_AP_FOUND:                        return "NO_AP_FOUND";           // 201: APが見つからない(SSID間違い/2.4GHz圏外)
    case WIFI_REASON_AUTH_FAIL:                          return "AUTH_FAIL";             // 202: 主にパスワード間違い
    case WIFI_REASON_ASSOC_FAIL:                         return "ASSOC_FAIL";            // 203
    case WIFI_REASON_HANDSHAKE_TIMEOUT:                  return "HANDSHAKE_TIMEOUT";     // 204
    case WIFI_REASON_CONNECTION_FAIL:                    return "CONNECTION_FAIL";       // 205
    case WIFI_REASON_ASSOC_COMEBACK_TIME_TOO_LONG:      return "ASSOC_COMEBACK_TIME_TOO_LONG"; // 208
    case WIFI_REASON_NO_AP_FOUND_W_COMPATIBLE_SECURITY: return "NO_AP_FOUND_W_COMPATIBLE_SECURITY"; // 210
    case WIFI_REASON_NO_AP_FOUND_IN_AUTHMODE_THRESHOLD: return "NO_AP_FOUND_IN_AUTHMODE_THRESHOLD"; // 211
    case WIFI_REASON_NO_AP_FOUND_IN_RSSI_THRESHOLD:     return "NO_AP_FOUND_IN_RSSI_THRESHOLD";     // 212
    default:                                              return "other";
    }
}

/** wifi_auth_mode_t（esp_wifi_types_generic.h）を短い名前にする（"wifi scan" の表示用）。 */
const char* authModeNameForScan(wifi_auth_mode_t mode)
{
    switch (mode) {
    case WIFI_AUTH_OPEN:                 return "OPEN";
    case WIFI_AUTH_WEP:                   return "WEP";
    case WIFI_AUTH_WPA_PSK:              return "WPA";
    case WIFI_AUTH_WPA2_PSK:             return "WPA2";
    case WIFI_AUTH_WPA_WPA2_PSK:         return "WPA_WPA2";
    case WIFI_AUTH_WPA2_ENTERPRISE:      return "WPA2_ENT"; // == WIFI_AUTH_ENTERPRISE (同値のため1本化)
    case WIFI_AUTH_WPA3_PSK:             return "WPA3";
    case WIFI_AUTH_WPA2_WPA3_PSK:        return "WPA2_WPA3";
    case WIFI_AUTH_WAPI_PSK:             return "WAPI";
    case WIFI_AUTH_OWE:                   return "OWE";
    case WIFI_AUTH_WPA3_ENT_192:         return "WPA3_ENT_192";
    case WIFI_AUTH_DPP:                   return "DPP";
    case WIFI_AUTH_WPA3_ENTERPRISE:      return "WPA3_ENT";
    case WIFI_AUTH_WPA2_WPA3_ENTERPRISE: return "WPA2_WPA3_ENT";
    case WIFI_AUTH_WPA_ENTERPRISE:       return "WPA_ENT";
    default:                              return "?";
    }
}

/* ---------------------------------------------------------------- NVS I/O */

/** 起動時に1回、NVS "uwb_net" 名前空間から mode/ssid/pass を読む。
 *  無い・壊れているキーは既定値（auto・空文字列）のまま残す。 */
void loadFromNvs()
{
    nvs_handle_t h;
    const esp_err_t openErr = nvs_open(kNvsNamespace, NVS_READONLY, &h);
    if (openErr != ESP_OK) {
        // 初回起動（NVSにこの名前空間が無い）はよくあるので INFO に留める。
        ESP_LOGI(kTag, "NVS(%s) 未設定 (err=%s)。既定値 mode=auto, ssid未設定 で動きます",
                 kNvsNamespace, esp_err_to_name(openErr));
        return;
    }

    uint8_t modeRaw       = kModeAuto;
    const bool haveMode = (nvs_get_u8(h, kKeyMode, &modeRaw) == ESP_OK && modeRaw <= kModeAp);

    char ssidBuf[sizeof(g_savedSsid)] = "";
    size_t ssidLen                       = sizeof(ssidBuf);
    const bool haveSsid                = (nvs_get_str(h, kKeySsid, ssidBuf, &ssidLen) == ESP_OK);

    char passBuf[sizeof(g_savedPass)] = "";
    size_t passLen                       = sizeof(passBuf);
    const bool havePass                = (nvs_get_str(h, kKeyPass, passBuf, &passLen) == ESP_OK);
    nvs_close(h);

    // g_savedModeRaw/g_savedSsid/g_savedPass への反映は1回のロックでまとめて
    // 行う（個別に読み書きするとリモートコマンドとのTOCTOUが起きうるため。
    // レビュー指摘 2026-08-31）。
    if (lock()) {
        if (haveMode) {
            g_savedModeRaw = modeRaw;
        }
        std::snprintf(g_savedSsid, sizeof(g_savedSsid), "%s", haveSsid ? ssidBuf : "");
        std::snprintf(g_savedPass, sizeof(g_savedPass), "%s", havePass ? passBuf : "");
        unlock();
    }
}

/** ssid/pass/modeRaw のうち非nullptrのものだけを NVS へ書く。 */
esp_err_t saveToNvs(const char* ssid, const char* pass, const uint8_t* modeRaw)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    if (ssid != nullptr) {
        err = nvs_set_str(h, kKeySsid, ssid);
    }
    if (err == ESP_OK && pass != nullptr) {
        err = nvs_set_str(h, kKeyPass, pass);
    }
    if (err == ESP_OK && modeRaw != nullptr) {
        err = nvs_set_u8(h, kKeyMode, *modeRaw);
    }
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t eraseNvs()
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(kNvsNamespace, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return (err == ESP_ERR_NVS_NOT_FOUND) ? ESP_OK : err;
    }
    err = nvs_erase_all(h);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

/* --------------------------------------------------------------- mDNS --- */

void startMdnsIfNeeded()
{
#if CONFIG_UWB_NET_MDNS
    if (g_mdnsStarted) {
        return;
    }
    esp_err_t err = mdns_init();
    if (err != ESP_OK) {
        ESP_LOGW(kTag, "mdns_init() 失敗 (err=%s)。<name>.local は使えません", esp_err_to_name(err));
        return;
    }
    mdns_hostname_set(config().name);
    mdns_instance_name_set(config().name);
    mdns_service_add(nullptr, "_http", "_tcp", config().httpPort, nullptr, 0);
    mdns_service_add(nullptr, "_uwbcon", "_tcp", config().consolePort, nullptr, 0);
    g_mdnsStarted = true;
#endif
}

/* ---------------------------------------------------------- モード切替 --- */

/** dst（dstCap バイト、呼び出し側で既にゼロ初期化済み）へ src を最大 dstCap
 *  バイトまでコピーする。wifi_sta_config_t::ssid/password 等は「固定長・
 *  ちょうど埋まったときは NUL 終端が無くてもよい」フィールドなので、
 *  snprintf(...,"%s",...) だと（GCCから見て src がちょうど収まらない可能性を
 *  否定できず）-Werror=format-truncation で弾かれる。意図的に memcpy で
 *  書き、NUL終端は要求しない。 */
void copyBounded(uint8_t* dst, size_t dstCap, const char* src)
{
    const size_t srcLen = std::strlen(src);
    const size_t n        = (srcLen < dstCap) ? srcLen : dstCap;
    std::memcpy(dst, src, n);
}

void ensureNetifs()
{
    if (g_staNetif == nullptr) {
        g_staNetif = esp_netif_create_default_wifi_sta();
    }
    if (g_apNetif == nullptr) {
        g_apNetif = esp_netif_create_default_wifi_ap();
    }
}

/** "net: mode=... ip=... url=... mdns=..." の起動ログ（GOT_IP時・AP起動時の両方から呼ぶ）。 */
void logNetReady(const char* modeStr, const char* ip)
{
    char ssidSnapshot[sizeof(g_savedSsid)] = "";
    if (lock()) {
        std::snprintf(ssidSnapshot, sizeof(ssidSnapshot), "%s", g_savedSsid);
        unlock();
    }
    ESP_LOGI(kTag, "net: mode=%s ssid=%s ip=%s url=http://%s/ mdns=http://%s.local/", modeStr,
             (modeStr[0] == 's') ? ssidSnapshot : config().name, ip, ip, config().name);
}

/** STA で起動（または既に起動済みなら設定を入れ直して再接続）する。 */
esp_err_t startStaMode()
{
    ensureNetifs();
    // 既に他モードで動いていれば一旦止める。初回（未起動）の esp_wifi_stop() は
    // ESP_ERR_WIFI_NOT_STARTED を返すだけなので無視してよい。
    esp_wifi_stop();

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_STA);
    if (err != ESP_OK) {
        return err;
    }

    // g_savedSsid/g_savedPass/g_savedModeRaw をロックしてコピーしてから使う
    // （レビュー指摘 2026-08-31）。
    char ssidCopy[sizeof(g_savedSsid)] = "";
    char passCopy[sizeof(g_savedPass)] = "";
    uint8_t modeRawCopy                  = kModeAuto;
    if (lock()) {
        std::snprintf(ssidCopy, sizeof(ssidCopy), "%s", g_savedSsid);
        std::snprintf(passCopy, sizeof(passCopy), "%s", g_savedPass);
        modeRawCopy = g_savedModeRaw;
        unlock();
    }

    wifi_config_t wcfg = {};
    copyBounded(wcfg.sta.ssid, sizeof(wcfg.sta.ssid), ssidCopy);
    copyBounded(wcfg.sta.password, sizeof(wcfg.sta.password), passCopy);
    wcfg.sta.threshold.authmode = (passCopy[0] != '\0') ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;
    err                          = esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_set_hostname(g_staNetif, config().name);

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    g_activeMode.store(ActiveMode::Sta, std::memory_order_release);
    g_connected.store(false, std::memory_order_release);

    // auto モードでは常に「STAへ切り替えたら20秒フォールバックタイマーを
    // 起動し直す」（起動時の初回呼び出しと、後述のrescanタスクからの復帰の
    // 両方で同じ挙動にする。mode=sta（固定）のときはフォールバック自体が
    // 無い＝タイマーは動かさない＝ずっとSTAで再接続し続ける）。
    if (modeRawCopy == kModeAuto && g_apFallbackTimer != nullptr) {
        esp_timer_stop(g_apFallbackTimer); // 呼び出し元で既に止めている場合もあるが冪等なので無害
        esp_timer_start_once(g_apFallbackTimer, static_cast<uint64_t>(kApFallbackUs));
    }
    return ESP_OK;
}

/** SoftAP で起動する。ここでは即座に「起動できた」とみなし GOT_IP 相当のログを出す
 *  （AP自身にはIPが即座に付くため、STAのような非同期待ちが要らない）。 */
esp_err_t startApMode()
{
    ensureNetifs();
    esp_wifi_stop();

    esp_err_t err = esp_wifi_set_mode(WIFI_MODE_AP);
    if (err != ESP_OK) {
        return err;
    }

    wifi_config_t wcfg = {};
    const char* name    = config().name;
    copyBounded(wcfg.ap.ssid, sizeof(wcfg.ap.ssid), name);
    wcfg.ap.ssid_len = static_cast<uint8_t>(std::strlen(name));
    copyBounded(wcfg.ap.password, sizeof(wcfg.ap.password), CONFIG_UWB_NET_AP_PASSWORD);
    wcfg.ap.channel        = CONFIG_UWB_NET_AP_CHANNEL;
    wcfg.ap.max_connection = 4;
    // パスワードが短すぎるとWPA2としては不正なので、その場合だけオープンにする
    // （CONFIG_UWB_NET_AP_PASSWORD の既定値 "uwb-localizer" は12文字でWPA2条件を満たす）。
    wcfg.ap.authmode =
        (std::strlen(CONFIG_UWB_NET_AP_PASSWORD) >= 8) ? WIFI_AUTH_WPA2_PSK : WIFI_AUTH_OPEN;

    err = esp_wifi_set_config(WIFI_IF_AP, &wcfg);
    if (err != ESP_OK) {
        return err;
    }

    esp_netif_set_hostname(g_apNetif, name);

    err = esp_wifi_start();
    if (err != ESP_OK) {
        return err;
    }

    g_activeMode.store(ActiveMode::Ap, std::memory_order_release);
    g_connected.store(true, std::memory_order_release); // AP自身は「起動できた」ことを以て connected 扱いにする

    char ip[16] = "0.0.0.0";
    esp_netif_ip_info_t ipInfo;
    if (esp_netif_get_ip_info(g_apNetif, &ipInfo) == ESP_OK) {
        esp_ip4addr_ntoa(&ipInfo.ip, ip, sizeof(ip));
    }
    startMdnsIfNeeded();
    logNetReady("ap", ip);
    return ESP_OK;
}

/* ------------------------------------------------------------ タイマー --- */

/** 20秒たっても STA が IP を取れなかった（auto モードのみ armed される）。 */
void apFallbackTimerCb(void* /*arg*/)
{
    if (g_connected.load(std::memory_order_acquire)) {
        return; // タイマー発火の直前にGOT_IPが来ていた（レース対策）
    }
    if (g_scanInProgress.load(std::memory_order_acquire)) {
        // "wifi scan" の最中はモードを切り替えない。スキャンは数秒で終わる
        // ので、短い猶予をおいて改めて判定し直す（フォールバック自体を
        // 取りこぼさないようにする）。
        esp_timer_start_once(g_apFallbackTimer, 2LL * 1000 * 1000);
        return;
    }
    ESP_LOGW(kTag, "net: STA 接続が %lld 秒以内に確立しませんでした。SoftAP へ切り替えます",
             static_cast<long long>(kApFallbackUs / 1000000));
    startApMode();
    g_autoSwitchedToAp = true;

    // g_savedModeRaw/g_savedSsid をロックしてコピーしてから使う
    // （レビュー指摘 2026-08-31）。
    uint8_t modeRawCopy = kModeAuto;
    bool haveSsid          = false;
    if (lock()) {
        modeRawCopy = g_savedModeRaw;
        haveSsid      = (g_savedSsid[0] != '\0');
        unlock();
    }

    // auto モードでAPへ落ちたので、保存済みSSIDが後から現れていないか
    // 30秒おきに探しに行く（コーディネータ指示: タグ・アンカー同時起動で
    // 片方の20秒待ちがもう片方のSoftAP開設より先に切れ、二度と合流でき
    // なかった実機不具合への対応）。mode=sta（固定）にはこの機構は無い
    // （STAはずっと再接続し続けるだけで、ここには来ない）。
    if (modeRawCopy == kModeAuto && haveSsid && g_rescanTimer != nullptr) {
        esp_timer_stop(g_rescanTimer); // 念のため（多重armは無害だが冪等にしておく）
        esp_timer_start_periodic(g_rescanTimer, static_cast<uint64_t>(kRescanIntervalUs));
    }
}

/** STA切断後、一定時間おいて再接続を試みる（呼び出し元は必ずSTAモードであること）。 */
void reconnectTimerCb(void* /*arg*/)
{
    if (g_scanInProgress.load(std::memory_order_acquire)) {
        return; // "wifi scan" 実行中は接続を試みない（cmdWifiScan()が終了時に自分で再開する）
    }
    if (g_activeMode.load(std::memory_order_acquire) == ActiveMode::Sta &&
        !g_connected.load(std::memory_order_acquire)) {
        esp_wifi_connect(); // 失敗（多重接続中等）は無視してよい。次の切断イベントで再試行される
    }
}

void armReconnectTimer()
{
    if (g_reconnectTimer == nullptr) {
        return;
    }
    esp_timer_stop(g_reconnectTimer); // 未起動なら ESP_ERR_INVALID_STATE。無視してよい
    esp_timer_start_once(g_reconnectTimer, static_cast<uint64_t>(kReconnectIntervalMs) * 1000);
}

/** g_rescanTimer（周期タイマー）のコールバック。esp_timerタスク上で動くため
 *  ブロックしてはいけない（scratchpad/NET_SPEC.md・コーディネータ指示）。
 *  実際のスキャンは "uwb_net_wifi" タスク（rescanTask()）へ丸投げする。 */
void rescanTimerCb(void* /*arg*/)
{
    if (g_rescanTaskHandle != nullptr) {
        xTaskNotifyGive(g_rescanTaskHandle);
    }
}

/** 保存済みSSIDが見えるようになっていないか1回だけ探す（rescanTask()から
 *  呼ばれる。ブロッキングOK＝esp_timerタスクではなく専用タスク上）。
 *
 * 前提条件（mode=auto・現在AP・SSID設定済み）は呼ばれるたびに毎回
 * 確認し直す：タイマーが鳴った瞬間と実際にこのタスクが動く瞬間の間、
 * あるいは前回のスキャンから今回のスキャンまでの30秒の間に、ユーザーが
 * "wifi mode"/"wifi set" 等で状況を変えているかもしれないため。条件が
 * 崩れていたら（もう auto+AP ではない）周期タイマー自体を止めて終わる
 * （次に本当に auto モードでAPへ落ちたら apFallbackTimerCb() が改めて
 * armする）。 */
void doRescanOnce()
{
    // g_savedModeRaw/g_savedSsid をロックしてコピーしてから使う
    // （レビュー指摘 2026-08-31）。
    uint8_t modeRawCopy                  = kModeAuto;
    char ssidCopy[sizeof(g_savedSsid)] = "";
    if (lock()) {
        modeRawCopy = g_savedModeRaw;
        std::snprintf(ssidCopy, sizeof(ssidCopy), "%s", g_savedSsid);
        unlock();
    }

    if (modeRawCopy != kModeAuto || g_activeMode.load(std::memory_order_acquire) != ActiveMode::Ap ||
        ssidCopy[0] == '\0') {
        if (g_rescanTimer != nullptr) {
            esp_timer_stop(g_rescanTimer);
        }
        return;
    }

    // "wifi scan" コマンドと同じフラグを取り合う。busyなら今回はあきらめて
    // 次の周期（30秒後）に賭ける（コーディネータ指示: skip this round）。
    bool expected = false;
    if (!g_scanInProgress.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return;
    }

    // SoftAPは張ったまま（esp_wifi_stopは呼ばない）APSTAへ一時的に上げる。
    const esp_err_t modeErr = esp_wifi_set_mode(WIFI_MODE_APSTA);
    if (modeErr != ESP_OK) {
        g_scanInProgress.store(false, std::memory_order_release);
        return;
    }

    wifi_scan_config_t scanCfg   = {};
    scanCfg.ssid                    = reinterpret_cast<uint8_t*>(ssidCopy); // このSSIDだけを狙って探す
    scanCfg.show_hidden           = true;
    scanCfg.scan_type              = WIFI_SCAN_TYPE_ACTIVE;
    scanCfg.scan_time.active.min = 120; // ms/チャネル
    scanCfg.scan_time.active.max = 300; // ms/チャネル
    const esp_err_t scanErr = esp_wifi_scan_start(&scanCfg, /*block=*/true);

    bool found = false;
    if (scanErr == ESP_OK) {
        uint16_t num = 0;
        esp_wifi_scan_get_ap_num(&num);
        found = (num > 0); // SSID指定スキャンなので1件以上=見つかった
    }
    esp_wifi_clear_ap_list(); // 詳細は要らないので内部リストは常に解放する

    g_scanInProgress.store(false, std::memory_order_release);

    if (found) {
        ESP_LOGI(kTag, "net: saved SSID found, switching back to STA");
        startStaMode(); // g_savedModeRaw==kModeAuto なので20秒フォールバックタイマーも再始動する
        if (g_rescanTimer != nullptr) {
            esp_timer_stop(g_rescanTimer); // STA試行中は不要。ダメならapFallbackTimerCb()が再armする
        }
    } else {
        const esp_err_t restoreErr = esp_wifi_set_mode(WIFI_MODE_AP);
        if (restoreErr != ESP_OK) {
            ESP_LOGW(kTag, "net: 再スキャン後にSoftAPへ戻せませんでした (err=%s)", esp_err_to_name(restoreErr));
        }
        // 周期タイマーは継続（ESP-IDFのperiodicタイマーは自動的に次回へ再スケジュールされる）。
    }
}

void rescanTask(void* /*arg*/)
{
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        doRescanOnce();
    }
}

/* --------------------------------------------------------- イベント処理 --- */

void onWifiEvent(void* /*arg*/, esp_event_base_t base, int32_t id, void* data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START: {
            if (g_scanInProgress.load(std::memory_order_acquire)) {
                // "wifi scan" がAP-only状態からAPSTAへ上げた直後に出るSTA_START。
                // ここで接続を始めるとスキャンと衝突して数百msで打ち切られる
                // （実機報告）。cmdWifiScan()が終了時に必要なら自分で再開する。
                break;
            }
            // 実機診断用（コーディネータ報告: SSID保存後も"wifi:state:"すら出ず、
            // どの段階で詰まっているか分からなかった）。関連付け開始をここで明示する。
            char ssidSnapshot[sizeof(g_savedSsid)] = "";
            if (lock()) {
                std::snprintf(ssidSnapshot, sizeof(ssidSnapshot), "%s", g_savedSsid);
                unlock();
            }
            ESP_LOGI(kTag, "net: STA connecting to ssid=%s", ssidSnapshot);
            esp_wifi_connect();
            break;
        }
        case WIFI_EVENT_STA_CONNECTED:
            // 関連付け(802.11 associate)は済んだがDHCPはまだ、という段階を区別できるように。
            ESP_LOGI(kTag, "net: associated, waiting for DHCP");
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            const auto* evt = static_cast<const wifi_event_sta_disconnected_t*>(data);
            g_connected.store(false, std::memory_order_release);
            // 2秒の再接続タイマーと同じ周期でしか意味が増えないので、ログもそれに
            // 合わせて最大2秒に1行へ間引く（切断は本来もっと高頻度に起こりうる）。
            const int64_t nowUs = esp_timer_get_time();
            if ((nowUs - g_lastDisconnectLogUs) >= 2LL * 1000 * 1000) {
                g_lastDisconnectLogUs = nowUs;
                char ssidSnapshot[sizeof(g_savedSsid)] = "";
                if (lock()) {
                    std::snprintf(ssidSnapshot, sizeof(ssidSnapshot), "%s", g_savedSsid);
                    unlock();
                }
                ESP_LOGW(kTag, "net: STA disconnected ssid=%s reason=%u (%s)", ssidSnapshot,
                         static_cast<unsigned>(evt->reason), wifiReasonName(evt->reason));
            }
            if (g_activeMode.load(std::memory_order_acquire) == ActiveMode::Sta) {
                armReconnectTimer();
            }
            break;
        }
        default:
            break;
        }
        return;
    }

    if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const auto* evt = static_cast<const ip_event_got_ip_t*>(data);
        char ip[16];
        esp_ip4addr_ntoa(&evt->ip_info.ip, ip, sizeof(ip));

        g_connected.store(true, std::memory_order_release);
        if (g_apFallbackTimer != nullptr) {
            esp_timer_stop(g_apFallbackTimer); // STAで無事つながったのでAPフォールバックは不要
        }
        startMdnsIfNeeded();
        logNetReady("sta", ip);
    }
}

/* ------------------------------------------------------- "wifi" コマンド --- */

void printUsage()
{
    std::printf("使い方:\n");
    std::printf("  wifi                          状態を表示\n");
    std::printf("  wifi set <ssid> [password]    NVSへ保存し、STAで即接続を試みる\n");
    std::printf("  wifi mode auto|sta|ap         起動モードをNVSへ保存する\n");
    std::printf("  wifi scan                     周辺のAPをスキャンして一覧表示する\n");
    std::printf("  wifi clear                    Wi-Fi設定(ssid/pass/mode)を消去する\n");
}

int cmdWifiStatus()
{
    WifiStatus st;
    wifiStatus(st);

    uint8_t modeRawCopy = kModeAuto;
    bool havePass          = false;
    if (lock()) {
        modeRawCopy = g_savedModeRaw;
        havePass      = (g_savedPass[0] != '\0');
        unlock();
    }

    std::printf("mode         : %s (NVS 設定: %s)\n", st.mode, modeRawName(modeRawCopy));
    std::printf("ssid         : %s\n", (st.ssid[0] != '\0') ? st.ssid : "(none)");
    // Wi-Fi パスワード自体は絶対に表示しない (scratchpad/NET_SPEC.md §3)。
    std::printf("password     : %s\n", havePass ? "(set)" : "(none)");
    std::printf("state        : %s\n", st.connected ? "connected" : "connecting/disconnected");
    std::printf("ip           : %s\n", st.ip);
    if (std::strcmp(st.mode, "sta") == 0) {
        std::printf("rssi         : %d dBm\n", st.rssi);
    } else if (std::strcmp(st.mode, "ap") == 0) {
        std::printf("ap clients   : %u\n", static_cast<unsigned>(st.apClients));
    }
    std::printf("ws clients   : %d\n", httpWsClientCount());
    std::printf("tcp console  : %s\n", tcpConsoleClientConnected() ? "connected" : "idle");
    std::printf("udp tx/rx    : %lu / %lu\n", static_cast<unsigned long>(udpTxCount()),
                static_cast<unsigned long>(udpRxCount()));
    std::printf("line drops   : %lu\n", static_cast<unsigned long>(sinkDrops()));
    return 0;
}

/** 0x20未満の制御文字・0x7F(DEL)が含まれていないか調べる（"wifi set" の
 *  ssid/password 検証用。レビュー指摘 2026-08-31: 制御文字が混ざると
 *  "node" 行のJSON等に混入しうる。JSON側もエスケープするが、そもそも
 *  Wi-Fi認証情報としても制御文字を含むのは想定外の入力なので入口で弾く）。 */
bool hasControlByte(const char* s)
{
    for (const unsigned char* p = reinterpret_cast<const unsigned char*>(s); *p != '\0'; ++p) {
        if (*p < 0x20 || *p == 0x7F) {
            return true;
        }
    }
    return false;
}

int cmdWifiSet(int argc, char** argv)
{
    if (g_scanInProgress.load(std::memory_order_acquire)) {
        std::printf("scan in progress, try again\n");
        return 1;
    }
    if (argc < 1 || argc > 2) {
        std::printf("使い方: wifi set <ssid> [password]\n");
        return 1;
    }
    const char* ssid = argv[0];
    const char* pass = (argc == 2) ? argv[1] : "";

    if (ssid[0] == '\0' || std::strlen(ssid) >= sizeof(g_savedSsid)) {
        std::printf("ssid が不正です（1〜%u文字）\n", static_cast<unsigned>(sizeof(g_savedSsid) - 1));
        return 1;
    }
    if (std::strlen(pass) >= sizeof(g_savedPass)) {
        std::printf("password が長すぎます（最大%u文字）\n", static_cast<unsigned>(sizeof(g_savedPass) - 1));
        return 1;
    }
    if (pass[0] != '\0' && std::strlen(pass) < 8) {
        // esp_wifi_set_config はWPA2パスワードが8文字未満だと拒否するため、
        // ここで先に弾いて分かりやすいエラーメッセージにする。
        std::printf("password は 8 文字以上にしてください（オープンAPにするなら省略）\n");
        return 1;
    }
    if (hasControlByte(ssid) || hasControlByte(pass)) {
        std::printf("ssid/password に不正な文字（制御文字）が含まれています\n");
        return 1;
    }

    if (lock()) {
        std::snprintf(g_savedSsid, sizeof(g_savedSsid), "%s", ssid);
        std::snprintf(g_savedPass, sizeof(g_savedPass), "%s", pass);
        unlock();
    }
    // 保存/表示は検証済みのローカル ssid/pass をそのまま使う（直後に
    // g_savedSsid/g_savedPass を読み返すとTOCTOUの隙間が生まれるため）。
    const esp_err_t saveErr = saveToNvs(ssid, pass, nullptr);
    if (saveErr != ESP_OK) {
        std::printf("警告: NVS への保存に失敗しました (err=%s)。今回だけ接続を試みます\n",
                    esp_err_to_name(saveErr));
    }

    g_autoSwitchedToAp = false; // 明示指定なので auto の「AP固定」状態を解除する
    if (g_apFallbackTimer != nullptr) {
        esp_timer_stop(g_apFallbackTimer);
    }
    const esp_err_t staErr = startStaMode();
    if (staErr != ESP_OK) {
        std::printf("STA での起動に失敗しました (err=%s)\n", esp_err_to_name(staErr));
        return 1;
    }
    std::printf("ssid=%s を保存し、STA で接続を試みています\n", ssid);
    return 0;
}

int cmdWifiMode(int argc, char** argv)
{
    if (g_scanInProgress.load(std::memory_order_acquire)) {
        std::printf("scan in progress, try again\n");
        return 1;
    }
    if (argc != 1) {
        std::printf("使い方: wifi mode auto|sta|ap\n");
        return 1;
    }
    uint8_t modeRaw;
    if (std::strcmp(argv[0], "auto") == 0) {
        modeRaw = kModeAuto;
    } else if (std::strcmp(argv[0], "sta") == 0) {
        modeRaw = kModeSta;
    } else if (std::strcmp(argv[0], "ap") == 0) {
        modeRaw = kModeAp;
    } else {
        std::printf("不明なモードです: %s（auto|sta|ap のいずれか）\n", argv[0]);
        return 1;
    }

    const esp_err_t err = saveToNvs(nullptr, nullptr, &modeRaw);
    if (err != ESP_OK) {
        std::printf("NVS への保存に失敗しました (err=%s)\n", esp_err_to_name(err));
        return 1;
    }
    bool haveSsid = false;
    if (lock()) {
        g_savedModeRaw = modeRaw;
        haveSsid          = (g_savedSsid[0] != '\0');
        unlock();
    }
    std::printf("mode=%s を NVS へ保存しました\n", argv[0]);

    // 反映が単純な遷移だけその場で適用する。それ以外（例: ssid未設定でsta指定）は
    // 次回起動時にwifiStart()側の初期化ロジックで解決される。
    if (modeRaw == kModeSta && haveSsid) {
        g_autoSwitchedToAp = false;
        if (g_apFallbackTimer != nullptr) {
            esp_timer_stop(g_apFallbackTimer);
        }
        startStaMode();
        std::printf("STA へ即時切り替えました\n");
    } else if (modeRaw == kModeAp) {
        if (g_apFallbackTimer != nullptr) {
            esp_timer_stop(g_apFallbackTimer);
        }
        startApMode();
        g_autoSwitchedToAp = true;
        std::printf("SoftAP へ即時切り替えました\n");
    } else {
        std::printf("次回起動から反映されます\n");
    }
    return 0;
}

int cmdWifiClear()
{
    if (g_scanInProgress.load(std::memory_order_acquire)) {
        std::printf("scan in progress, try again\n");
        return 1;
    }
    const esp_err_t err = eraseNvs();
    if (err != ESP_OK) {
        std::printf("NVS の消去に失敗しました (err=%s)\n", esp_err_to_name(err));
        return 1;
    }
    if (lock()) {
        g_savedModeRaw = kModeAuto;
        g_savedSsid[0]  = '\0';
        g_savedPass[0]  = '\0';
        unlock();
    }
    std::printf("Wi-Fi 設定を消去しました（次回起動から mode=auto, ssid未設定 で動きます）\n");
    return 0;
}

/* -------------------------------------------------------------------- scan */

/** wifi_ap_record_t 配列を rssi 降順に並べ替える（最大20件想定の挿入ソートで十分）。 */
void sortByRssiDesc(wifi_ap_record_t* records, uint16_t n)
{
    for (uint16_t i = 1; i < n; ++i) {
        const wifi_ap_record_t key = records[i];
        int j                       = static_cast<int>(i) - 1;
        while (j >= 0 && records[j].rssi < key.rssi) {
            records[j + 1] = records[j];
            --j;
        }
        records[j + 1] = key;
    }
}

/** wifi_ap_record_t::ssid（uint8_t[33]、NUL終端は保証されない）を安全に
 *  printf へ渡せるよう、NUL終端した表示用文字列にする。 */
void ssidForDisplay(const uint8_t* ssid, size_t ssidCap, char* out, size_t outCap)
{
    const size_t len = strnlen(reinterpret_cast<const char*>(ssid), ssidCap);
    const size_t n     = (len < outCap) ? len : (outCap - 1);
    std::memcpy(out, ssid, n);
    out[n] = '\0';
}

/**
 * @brief 周辺APをスキャンして一覧表示する（実機診断用。コーディネータ指示:
 * "AP not found" と "wrong password" を切り分けたいが、家の中に複数の
 * 2.4GHz APがある環境ではSSID名だけでは判別しづらいため、チャネル/RSSI/
 * 認証方式を横に並べて見せる）。
 *
 * スキャンにはSTAインターフェースが要る。現在SoftAPのみ(WIFI_MODE_AP)の
 * ときは、SoftAPを張ったまま(esp_wifi_stopは呼ばない)WIFI_MODE_APSTAへ
 * 一時的に上げてスキャンし、終わったらWIFI_MODE_APへ戻す。
 * STAが接続試行中のときは esp_wifi_scan_start() が ESP_ERR_WIFI_STATE を
 * 返す仕様（esp_wifi.hのesp_wifi_connect()コメント参照）なので、その場合は
 * エラー名を出して素直に諦める（自動で待って再試行はしない）。
 */
int cmdWifiScan()
{
    bool expected = false;
    if (!g_scanInProgress.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        std::printf("scan in progress, try again\n");
        return 1;
    }

    // 元のモードを覚えておく。スキャンにはSTAインターフェースが要るので、
    // AP-onlyだった場合だけ一時的にAPSTAへ上げる（SoftAPは張ったまま。
    // esp_wifi_stopは呼ばない）。
    const ActiveMode modeBefore = g_activeMode.load(std::memory_order_acquire);
    bool switchedToApsta          = false;
    if (modeBefore == ActiveMode::Ap) {
        const esp_err_t modeErr = esp_wifi_set_mode(WIFI_MODE_APSTA);
        if (modeErr != ESP_OK) {
            std::printf("スキャン用にSTAを起こせませんでした (err=%s)\n", esp_err_to_name(modeErr));
            g_scanInProgress.store(false, std::memory_order_release);
            return 1;
        }
        switchedToApsta = true;
    }

    // ここに来た時点でSTAインターフェースは必ず動いている（元々STAだったか、
    // 直前にAPSTAへ上げたか）。進行中の関連付け試行を止めてからスキャンする。
    // 実機報告(2026-08-31): esp_wifi_connect()とesp_wifi_scan_start()が同時に
    // 走ると、能動スキャンが約100msで打ち切られAPが1件も見つからなかった
    // （13チャネルの能動スキャンは100msでは終わらない）。onWifiEvent /
    // reconnectTimerCb / apFallbackTimerCb はg_scanInProgress中は接続・モード
    // 切替えをしない（各関数の該当コメント参照）ので、ここで切った接続が
    // スキャン中に勝手に再開されることは無い。
    esp_wifi_disconnect(); // 未接続中のエラー(ESP_ERR_WIFI_NOT_STARTED等)は無視してよい
    vTaskDelay(pdMS_TO_TICKS(200));

    wifi_scan_config_t scanCfg   = {};
    scanCfg.show_hidden           = true;
    scanCfg.scan_type              = WIFI_SCAN_TYPE_ACTIVE;
    scanCfg.scan_time.active.min = 120; // ms/チャネル。弱いAPも拾えるよう既定より長めにする
    scanCfg.scan_time.active.max = 300; // ms/チャネル
    const esp_err_t scanErr = esp_wifi_scan_start(&scanCfg, /*block=*/true);
    if (scanErr != ESP_OK) {
        // STAが接続試行中だとESP_ERR_WIFI_STATEになりうる（esp_wifi_scan_start()の
        // ドキュメント参照）。上のdisconnect+待ちで基本的には避けられるはずだが、
        // 万一発生したらエラー名を出してそのまま諦める（自動リトライはしない）。
        std::printf("esp_wifi_scan_start に失敗しました (err=%s)\n", esp_err_to_name(scanErr));
    }

    uint16_t num = 0;
    if (scanErr == ESP_OK) {
        esp_wifi_scan_get_ap_num(&num);
    }
    std::printf("found %u APs\n", static_cast<unsigned>(num));

    constexpr uint16_t kMaxRecords = 20;
    const uint16_t fetchNum          = (num < kMaxRecords) ? num : kMaxRecords;
    wifi_ap_record_t* records         = nullptr;
    uint16_t got                       = 0;

    if (fetchNum > 0) {
        records = static_cast<wifi_ap_record_t*>(std::malloc(sizeof(wifi_ap_record_t) * fetchNum));
        if (records == nullptr) {
            std::printf("メモリ不足でスキャン結果を取得できませんでした\n");
        } else {
            got                       = fetchNum;
            const esp_err_t getErr = esp_wifi_scan_get_ap_records(&got, records);
            if (getErr != ESP_OK) {
                std::printf("スキャン結果の取得に失敗しました (err=%s)\n", esp_err_to_name(getErr));
                got = 0;
            }
        }
    }
    // ドライバ内部のスキャンリストは esp_wifi_scan_get_ap_records() が呼ばれれば
    // そこで解放される。呼ばれなかった経路（fetchNum==0・malloc失敗）はここで
    // 明示的に解放する。
    if (records == nullptr) {
        esp_wifi_clear_ap_list();
    }

    if (got > 0) {
        sortByRssiDesc(records, got);
        std::printf("%-33s %4s %5s %-12s\n", "ssid", "ch", "rssi", "auth");
        for (uint16_t i = 0; i < got; ++i) {
            char ssidBuf[33];
            ssidForDisplay(records[i].ssid, sizeof(records[i].ssid), ssidBuf, sizeof(ssidBuf));
            std::printf("%-33s %4u %5d %-12s\n", (ssidBuf[0] != '\0') ? ssidBuf : "(hidden)",
                        static_cast<unsigned>(records[i].primary), static_cast<int>(records[i].rssi),
                        authModeNameForScan(records[i].authmode));
        }
        if (num > fetchNum) {
            std::printf("（上限%u件で打ち切り。実際は%u件見つかりました）\n",
                        static_cast<unsigned>(kMaxRecords), static_cast<unsigned>(num));
        }
    }
    std::free(records);

    // 後始末: スキャン中に切ったSTA接続・一時的に上げたAPSTAを元に戻す。
    // 先にフラグを下ろしてから戻す（以後 onWifiEvent/reconnectTimerCb/
    // apFallbackTimerCb が通常どおり反応してよい状態にするため）。
    g_scanInProgress.store(false, std::memory_order_release);
    if (switchedToApsta) {
        const esp_err_t restoreErr = esp_wifi_set_mode(WIFI_MODE_AP);
        if (restoreErr != ESP_OK) {
            std::printf("警告: SoftAPへ戻せませんでした (err=%s)。'wifi mode ap' で復旧してください\n",
                        esp_err_to_name(restoreErr));
        }
    } else if (modeBefore == ActiveMode::Sta) {
        if (!g_connected.load(std::memory_order_acquire)) {
            esp_wifi_connect(); // スキャン前に切った接続を再開する
        }
    }

    return (scanErr == ESP_OK) ? 0 : 1;
}

int cmdWifi(int argc, char** argv)
{
    if (argc < 2) {
        return cmdWifiStatus();
    }
    const char* sub = argv[1];
    if (std::strcmp(sub, "set") == 0) {
        return cmdWifiSet(argc - 2, argv + 2);
    }
    if (std::strcmp(sub, "mode") == 0) {
        return cmdWifiMode(argc - 2, argv + 2);
    }
    if (std::strcmp(sub, "scan") == 0) {
        return cmdWifiScan();
    }
    if (std::strcmp(sub, "clear") == 0) {
        return cmdWifiClear();
    }
    std::printf("不明なサブコマンド: %s\n", sub);
    printUsage();
    return 1;
}

} // namespace

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された関数
 * ==================================================================== */

esp_err_t wifiStart()
{
    if (g_mutex == nullptr) {
        g_mutex = xSemaphoreCreateMutex();
        if (g_mutex == nullptr) {
            return ESP_ERR_NO_MEM;
        }
    }

    loadFromNvs();

    // esp_netif_init()/esp_event_loop_create_default() は「既に初期化済み」を
    // ESP_ERR_INVALID_STATE で返すだけの冪等API。uwb_net だけがWi-Fiスタックの
    // 所有者である前提だが、念のため許容しておく。
    esp_err_t err = esp_netif_init();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    wifi_init_config_t wifiInitCfg = WIFI_INIT_CONFIG_DEFAULT();
    err                              = esp_wifi_init(&wifiInitCfg);
    if (err != ESP_OK) {
        return err;
    }

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &onWifiEvent, nullptr,
                                                          nullptr));
    ESP_ERROR_CHECK(
        esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &onWifiEvent, nullptr, nullptr));

    // レイテンシと再送の確実性を優先する（scratchpad/NET_SPEC.md §3）。
    esp_wifi_set_ps(WIFI_PS_NONE);

    const esp_timer_create_args_t apFallbackArgs = {
        .callback             = &apFallbackTimerCb,
        .arg                    = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                   = "uwb_net_apfb",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&apFallbackArgs, &g_apFallbackTimer);

    const esp_timer_create_args_t reconnectArgs = {
        .callback             = &reconnectTimerCb,
        .arg                    = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                   = "uwb_net_recon",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&reconnectArgs, &g_reconnectTimer);

    const esp_timer_create_args_t rescanArgs = {
        .callback             = &rescanTimerCb,
        .arg                    = nullptr,
        .dispatch_method       = ESP_TIMER_TASK,
        .name                   = "uwb_net_rescan",
        .skip_unhandled_events = true,
    };
    esp_timer_create(&rescanArgs, &g_rescanTimer);

    // 実際のスキャン（esp_wifi_scan_start(block=true)）は数百ms〜数秒かかりうる
    // ため、esp_timerのコールバック（esp_timerタスク上で動く。ブロック厳禁）
    // からは直接やらず、この専用タスクへ通知するだけにする
    // （rescanTimerCb() -> xTaskNotifyGive() -> ここが起きて doRescanOnce()）。
    const BaseType_t rescanTaskOk = xTaskCreatePinnedToCore(&rescanTask, "uwb_net_wifi", 4096, nullptr, 4,
                                                             &g_rescanTaskHandle, 0);
    if (rescanTaskOk != pdPASS) {
        ESP_LOGE(kTag, "uwb_net_wifi タスクの起動に失敗しました（保存済みSSIDの再スキャンは動きません）");
        g_rescanTaskHandle = nullptr;
    }

    // g_savedModeRaw/g_savedSsid をロックしてコピーしてから分岐する
    // （startStaMode()/startApMode() 自身も内部で改めてロック付きで読むが、
    // ここでの分岐判断自体もレースの無い一貫したスナップショットで行う。
    // レビュー指摘 2026-08-31）。
    uint8_t modeRawCopy = kModeAuto;
    bool haveSsid          = false;
    if (lock()) {
        modeRawCopy = g_savedModeRaw;
        haveSsid      = (g_savedSsid[0] != '\0');
        unlock();
    }

    if (modeRawCopy == kModeSta) {
        err = startStaMode();
    } else if (modeRawCopy == kModeAp) {
        err = startApMode();
    } else { // auto
        if (haveSsid) {
            // 20秒フォールバックタイマーの起動は startStaMode() 自身が行う
            // （g_savedModeRaw==kModeAuto のとき。rescanタスクからの復帰時と
            // 挙動を揃えるため、このファイルの startStaMode() コメント参照）。
            err = startStaMode();
        } else {
            ESP_LOGI(kTag, "net: ssid未設定のため SoftAP から起動します（wifi set で設定できます）");
            err                  = startApMode();
            g_autoSwitchedToAp = true;
        }
    }
    return err;
}

void wifiStatus(WifiStatus& out)
{
    out = WifiStatus{};
    std::snprintf(out.ip, sizeof(out.ip), "0.0.0.0");

    const ActiveMode mode = g_activeMode.load(std::memory_order_acquire);
    const bool connected     = g_connected.load(std::memory_order_acquire);

    switch (mode) {
    case ActiveMode::Sta: std::snprintf(out.mode, sizeof(out.mode), "sta"); break;
    case ActiveMode::Ap:  std::snprintf(out.mode, sizeof(out.mode), "ap"); break;
    default:               std::snprintf(out.mode, sizeof(out.mode), "off"); break;
    }
    out.connected = connected;
    out.rssi        = 0;
    out.apClients   = 0;

    if (mode == ActiveMode::Sta) {
        if (lock()) {
            std::snprintf(out.ssid, sizeof(out.ssid), "%s", g_savedSsid);
            unlock();
        }
        wifi_ap_record_t rec;
        if (connected && esp_wifi_sta_get_ap_info(&rec) == ESP_OK) {
            out.rssi = rec.rssi;
        }
        esp_netif_ip_info_t ipInfo;
        if (g_staNetif != nullptr && esp_netif_get_ip_info(g_staNetif, &ipInfo) == ESP_OK &&
            ipInfo.ip.addr != 0) {
            esp_ip4addr_ntoa(&ipInfo.ip, out.ip, sizeof(out.ip));
        }
    } else if (mode == ActiveMode::Ap) {
        std::snprintf(out.ssid, sizeof(out.ssid), "%s", config().name);
        esp_netif_ip_info_t ipInfo;
        if (g_apNetif != nullptr && esp_netif_get_ip_info(g_apNetif, &ipInfo) == ESP_OK) {
            esp_ip4addr_ntoa(&ipInfo.ip, out.ip, sizeof(out.ip));
        }
        wifi_sta_list_t staList;
        if (esp_wifi_ap_get_sta_list(&staList) == ESP_OK) {
            out.apClients = static_cast<uint8_t>(staList.num);
        }
    }
}

esp_err_t wifiRegisterConsoleCommands()
{
    const esp_console_cmd_t cmd = {
        .command  = "wifi",
        .help     = "Wi-Fi の状態表示・設定（wifi / wifi set <ssid> [password] / wifi mode "
                    "auto|sta|ap / wifi scan / wifi clear）",
        .hint     = " [set <ssid> [password] | mode auto|sta|ap | scan | clear]",
        .func     = &cmdWifi,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    return esp_console_cmd_register(&cmd);
}

TaskHandle_t wifiRescanTaskHandle()
{
    return g_rescanTaskHandle;
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t wifiStart()
{
    return ESP_OK;
}

void wifiStatus(WifiStatus& out)
{
    out = WifiStatus{};
}

esp_err_t wifiRegisterConsoleCommands()
{
    return ESP_OK;
}

TaskHandle_t wifiRescanTaskHandle()
{
    return nullptr;
}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
