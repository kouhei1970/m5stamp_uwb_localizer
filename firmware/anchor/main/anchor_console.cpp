/**
 * @file anchor_console.cpp
 * @brief anchor_console.hpp の実装。ESP-IDF の console コンポーネント
 * （linenoise ベースの REPL）でシリアルコンソールを立てる。
 *
 * コマンド:
 *   addr                  現在のショートアドレスを表示
 *   addr set <hex>        変更する（例: addr set 0x0003）。反映は次の応答から
 *   save                  現在の設定を NVS へ保存
 *   reset-config          NVS を消して Kconfig の既定値へ戻す
 *   info                  Device ID / チップ名 / 現在の設定 / 測距統計
 *   reboot                再起動
 *
 * REPL のデバイスは USB-Serial/JTAG を既定とする（M5StampS3A / M5 AtomS3 は
 * USB-CDC で PC につながるため。sdkconfig.defaults で
 * CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y にしてある）。UART をコンソールに
 * 選んだ構成でもビルドが通るよう、下の #if で使う API を切り替える。
 *
 * 引数の解析は argtable3 ではなく argc/argv を直接見る形にしてある
 * （esp_console_cmd_t::argtable を nullptr にすると esp_console は
 * 行を分割した argc/argv をそのまま渡してくる）。argtable3 の位置引数解析は
 * 内部で getopt_long を通すため、**負の数が「不正なオプション」として
 * 弾かれる**（"-1.5" が -1 -. -5 の3つのオプション扱いになる。ホストで
 * 実際に再現を確認済み）。タグ側では座標やアンテナ遅延に負の値が普通に
 * 出てくるので、両ファームとも同じ流儀に揃えている。
 * linenoise による行編集・履歴・help コマンドは従来どおり console
 * コンポーネントのものを使う。
 */
#include "anchor_console.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "uwb_cfgstore.hpp"

namespace anchorapp {

namespace {

const char* kLogTag = "uwb_anchor_console";

/* ------------------------------------------------------------------ 共有状態 */

SemaphoreHandle_t g_mutex = nullptr;
StaticInfo g_info;
uint16_t g_addr       = 0;     //!< いま有効なショートアドレス
uint16_t g_savedAddr  = 0;     //!< 直近に NVS に入っている（はずの）値
bool g_savedValid     = false; //!< g_savedAddr が意味を持つか（起動時に NVS から読めた or save した）
Stats g_stats;                 //!< 測距ループが公開する統計（info コマンドの表示用）

/** ミューテックスを取る。初期化前でも安全に呼べるよう nullptr を許容する。 */
bool lock()
{
    if (g_mutex == nullptr) {
        return false;
    }
    return xSemaphoreTake(g_mutex, portMAX_DELAY) == pdTRUE;
}

void unlock()
{
    if (g_mutex != nullptr) {
        xSemaphoreGive(g_mutex);
    }
}

/** 現在のアドレスが NVS に保存済みか（ロック済みの状態で呼ぶこと）。
 *  既定値のまま起動した場合は NVS に何も無いので「未保存」になる。 */
bool savedLocked()
{
    return g_savedValid && (g_addr == g_savedAddr);
}

/* ------------------------------------------------------------------ 引数解析 */

/**
 * @brief 16 進（0x 付き）でも 10 進でも受け付けてショートアドレスにする。
 * @return 受け付けられたら true。0xFFFF はブロードキャストなので拒否する。
 */
bool parseShortAddr(const char* text, uint16_t* out)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    char* end        = nullptr;
    errno            = 0;
    const unsigned long v = std::strtoul(text, &end, 0);
    if (end == text || *end != '\0' || errno != 0 || v > 0xFFFFul) {
        return false;
    }
    const uint16_t addr = static_cast<uint16_t>(v);
    if (!uwb::cfg::isValidShortAddr(addr)) {
        return false;
    }
    *out = addr;
    return true;
}

/* -------------------------------------------------------------------- addr */

void printUsageAddr()
{
    std::printf("使い方:\n");
    std::printf("  addr            現在のショートアドレスを表示\n");
    std::printf("  addr set <hex>  変更する（例: addr set 0x0003）\n");
}

int cmdAddr(int argc, char** argv)
{
    // argv[0] はコマンド名 "addr"。
    const char* action = (argc >= 2) ? argv[1] : nullptr;
    const char* value  = (argc >= 3) ? argv[2] : nullptr;

    if (argc > 3) {
        std::printf("引数が多すぎます\n");
        printUsageAddr();
        return 1;
    }

    if (action == nullptr) {
        if (!lock()) {
            return 1;
        }
        const uint16_t addr = g_addr;
        const bool saved     = savedLocked();
        unlock();
        std::printf("short_addr = 0x%04X  (Kconfig 既定値 = 0x%04X, NVS = %s)\n",
                    static_cast<unsigned>(addr), static_cast<unsigned>(g_info.defaultAddr),
                    saved ? "保存済み" : "未保存（save が必要）");
        return 0;
    }

    if (std::strcmp(action, "set") != 0) {
        printUsageAddr();
        return 1;
    }
    if (value == nullptr) {
        std::printf("値がありません。例: addr set 0x0003\n");
        return 1;
    }

    uint16_t addr = 0;
    if (!parseShortAddr(value, &addr)) {
        std::printf("不正なアドレスです: %s（0x0000〜0xFFFE。0xFFFF はブロードキャストなので不可）\n",
                    value);
        return 1;
    }
    if (addr == g_info.tagAddr) {
        std::printf("タグと同じアドレス 0x%04X は使えません\n", static_cast<unsigned>(addr));
        return 1;
    }

    if (!lock()) {
        return 1;
    }
    g_addr = addr;
    unlock();

    std::printf("short_addr = 0x%04X に変更しました（次の応答から反映）。"
                "電源を切っても残すには save を実行してください\n",
                static_cast<unsigned>(addr));
    return 0;
}

/* -------------------------------------------------------------------- save */

int cmdSave(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    if (!lock()) {
        return 1;
    }
    const uint16_t addr = g_addr;
    unlock();

    const esp_err_t err = uwb::ConfigStore::saveAnchorAddr(addr);
    if (err != ESP_OK) {
        std::printf("保存に失敗しました (err=%s)\n", esp_err_to_name(err));
        return 1;
    }

    if (lock()) {
        g_savedAddr  = addr;
        g_savedValid = true;
        unlock();
    }
    std::printf("NVS へ保存しました: short_addr = 0x%04X\n", static_cast<unsigned>(addr));
    return 0;
}

/* ------------------------------------------------------------ reset-config */

int cmdResetConfig(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    const esp_err_t err = uwb::ConfigStore::eraseAll();
    if (err != ESP_OK) {
        std::printf("NVS の消去に失敗しました (err=%s)\n", esp_err_to_name(err));
        return 1;
    }

    if (lock()) {
        g_addr       = g_info.defaultAddr;
        g_savedValid = false;
        unlock();
    }
    std::printf("NVS を消去し、Kconfig の既定値 0x%04X に戻しました（次の応答から反映）\n",
                static_cast<unsigned>(g_info.defaultAddr));
    return 0;
}

/* -------------------------------------------------------------------- info */

int cmdInfo(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    if (!lock()) {
        return 1;
    }
    const uint16_t addr  = g_addr;
    const bool saved      = savedLocked();
    const Stats stats     = g_stats;
    unlock();

    const uint32_t total = stats.ok + stats.fail;
    const double rate    = (total == 0) ? 0.0 : (100.0 * static_cast<double>(stats.ok) / total);

    std::printf("=== uwb_anchor ===\n");
    std::printf("  board        : %s\n", g_info.boardName);
    std::printf("  method       : %s\n", g_info.methodName);
    std::printf("  device id    : 0x%08lX  chip: %s\n", static_cast<unsigned long>(g_info.deviceId),
                g_info.chipName);
    std::printf("  short_addr   : 0x%04X  (起動時の設定元: %s, Kconfig 既定値: 0x%04X, NVS: %s)\n",
                static_cast<unsigned>(addr), uwb::configSourceName(g_info.source),
                static_cast<unsigned>(g_info.defaultAddr), saved ? "保存済み" : "未保存（save が必要）");
    std::printf("  tag addr     : 0x%04X   pan id: 0x%04X\n", static_cast<unsigned>(g_info.tagAddr),
                static_cast<unsigned>(g_info.panId));
    std::printf("  nvs          : %s\n", uwb::ConfigStore::isReady() ? "使用可" : "使用不可（既定値のみ）");
    std::printf("  ranging      : ok=%lu fail=%lu rate=%.1f%%\n", static_cast<unsigned long>(stats.ok),
                static_cast<unsigned long>(stats.fail), rate);
    if (stats.samples > 0) {
        std::printf("  distance     : mean=%.1f mm  std=%.1f mm  n=%lu\n", stats.meanMm, stats.stdMm,
                    static_cast<unsigned long>(stats.samples));
    } else {
        std::printf("  distance     : (まだサンプルがありません)\n");
    }
    std::printf("  free heap    : %lu bytes\n", static_cast<unsigned long>(esp_get_free_heap_size()));
    return 0;
}

/* ------------------------------------------------------------------ reboot */

int cmdReboot(int argc, char** argv)
{
    (void)argc;
    (void)argv;
    std::printf("再起動します\n");
    std::fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(100)); // 出力が USB へ流れ切るのを待つ
    esp_restart();
    return 0; // 到達しない
}

/* --------------------------------------------------------------- 登録処理 */

void registerCommands()
{
    const esp_console_cmd_t addrCmd = {
        .command  = "addr",
        .help     = "ショートアドレスの表示・変更（addr / addr set 0x0003）",
        .hint     = " [set <hex>]",
        .func     = &cmdAddr,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&addrCmd));

    const esp_console_cmd_t saveCmd = {
        .command  = "save",
        .help     = "現在の設定を NVS へ保存する",
        .hint     = nullptr,
        .func     = &cmdSave,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&saveCmd));

    const esp_console_cmd_t resetCmd = {
        .command  = "reset-config",
        .help     = "NVS を消去して Kconfig の既定値に戻す",
        .hint     = nullptr,
        .func     = &cmdResetConfig,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&resetCmd));

    const esp_console_cmd_t infoCmd = {
        .command  = "info",
        .help     = "Device ID / チップ名 / 現在の設定 / 測距統計を表示する",
        .hint     = nullptr,
        .func     = &cmdInfo,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&infoCmd));

    const esp_console_cmd_t rebootCmd = {
        .command  = "reboot",
        .help     = "再起動する",
        .hint     = nullptr,
        .func     = &cmdReboot,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&rebootCmd));
}

} // namespace

/* ==================================================================== *
 * 公開 API
 * ==================================================================== */

bool sharedInit(uint16_t addr, const StaticInfo& info)
{
    if (g_mutex == nullptr) {
        g_mutex = xSemaphoreCreateMutex();
    }
    if (g_mutex == nullptr) {
        ESP_LOGE(kLogTag, "ミューテックスを作れませんでした");
        return false;
    }

    g_info       = info;
    g_addr       = addr;
    g_savedAddr  = addr;
    // 起動時に NVS から読めた場合だけ「保存済みの値を知っている」と言える。
    g_savedValid = (info.source == uwb::ConfigSource::Nvs);
    g_stats      = Stats{};
    return true;
}

uint16_t currentShortAddr()
{
    if (!lock()) {
        return g_addr;
    }
    const uint16_t addr = g_addr;
    unlock();
    return addr;
}

void publishStats(const Stats& stats)
{
    if (!lock()) {
        return;
    }
    g_stats = stats;
    unlock();
}

esp_err_t consoleStart()
{
    esp_console_repl_t* repl          = nullptr;
    esp_console_repl_config_t replCfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    replCfg.prompt                     = "uwb-anchor>";
    replCfg.max_cmdline_length         = 128;
    // 既定の 4096 から増やしてある。info / addr の表示に printf の浮動小数点
    // 変換（newlib の vfprintf）を通すため、余裕を見ておく。
    replCfg.task_stack_size            = 6144;

    esp_console_register_help_command();
    registerCommands();

#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
    esp_console_dev_usb_serial_jtag_config_t devCfg = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    const esp_err_t err = esp_console_new_repl_usb_serial_jtag(&devCfg, &replCfg, &repl);
#elif CONFIG_ESP_CONSOLE_USB_CDC
    esp_console_dev_usb_cdc_config_t devCfg = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    const esp_err_t err = esp_console_new_repl_usb_cdc(&devCfg, &replCfg, &repl);
#else
    esp_console_dev_uart_config_t devCfg = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    const esp_err_t err = esp_console_new_repl_uart(&devCfg, &replCfg, &repl);
#endif

    if (err != ESP_OK) {
        ESP_LOGE(kLogTag, "REPL を作れませんでした (err=%s)", esp_err_to_name(err));
        return err;
    }
    return esp_console_start_repl(repl);
}

} // namespace anchorapp
