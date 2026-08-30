/**
 * @file tag_console.cpp
 * @brief tag_console.hpp の実装。ESP-IDF の console コンポーネント
 * （linenoise ベースの REPL）でシリアルコンソールを立てる。
 *
 * コマンド:
 *   anchor list                               登録テーブルを表示
 *   anchor set <idx> <hex_addr> <x> <y> <z>   座標を入れる（例 anchor set 0 0x0002 0.0 0.0 2.4）
 *   anchor delay <idx> <meters>               アンテナ遅延オフセット
 *   anchor enable <idx> / anchor disable <idx>
 *   anchor count <n>                          有効件数
 *   output <on|off>                           JSON Lines 出力の一時停止
 *   save                                      NVS へ保存
 *   reset-config                              NVS を消して kAnchors[] の既定値へ戻す
 *   info                                      Device ID / チップ名 / 現在の設定 / 測位状況
 *   reboot                                    再起動
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
 * 実際に再現を確認済み）。座標もアンテナ遅延も負の値を普通に取るので、
 * ここでは argtable3 の解析を使わない。linenoise による行編集・履歴・
 * help コマンドは従来どおり console コンポーネントのものを使う。
 *
 * v2（uwb::RangingService 導入後）のロック方針は tag_console.hpp 冒頭の
 * コメント参照: 表の編集は service.lockTable() を取って直接
 * AnchorTable::update()/set() を呼ぶ。resetStats()/reinitEkf() は
 * unlockTable() で離してから呼ぶ（内部で取り直すため、二重取得は
 * デッドロックする）。
 *
 * **素の tableMutex() ではなく必ず lockTable()/unlockTable() を使う**
 * （uwb_ranging_service.hpp 冒頭「tableMutex() の優先度逆転」参照）。
 * 処理タスク（RangingService 内部、優先度18・連続実行）は1周期を終える
 * たびに素の tableMutex() を離してすぐ取り直すため、素のハンドルへ直接
 * xSemaphoreTake() すると、この優先度2のコンソールタスクは無期限に
 * ブロックし得る（実機で `info` / `anchor list` の固まりとして確認・
 * 修正済み）。lockTable()/unlockTable() は待ち手を処理タスクへ知らせ、
 * 処理タスク側が1ティック譲ることで飢餓を避ける。
 */
#include "tag_console.hpp"

#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "uwb_cfgstore.hpp"
#include "uwb_net.hpp"

namespace tagapp {

namespace {

const char* kLogTag = "uwb_tag_console";

/* ------------------------------------------------------------------ 共有状態 */

/** g_saved / g_jsonOutput / g_tableGeneration の保護用。アンカー表そのもの
 *  は service.lockTable()/unlockTable() が守るので、この mutex とは別物
 *  （tag_console.hpp 冒頭コメント参照）。 */
SemaphoreHandle_t g_mutex = nullptr;
StaticInfo g_info;

/** 生きたアンカー登録テーブルと測位サービス（sharedInit() が受け取った参照）。 */
uwb::AnchorTable* g_table      = nullptr;
uwb::RangingService* g_service = nullptr;

/** 現在の表の内容が NVS に保存済みか。
 *  起動時は「NVSから読めた場合のみ保存済み」とする（既定値で起動した場合は
 *  NVS には何も無いので未保存）。編集で false、save で true になる。 */
bool g_saved = false;

/** アンカー表の「世代」。tag_console.hpp の tableGeneration() 参照。 */
uint32_t g_tableGeneration = 0;

bool g_jsonOutput = true;

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

bool readSaved()
{
    bool saved = false;
    if (lock()) {
        saved = g_saved;
        unlock();
    }
    return saved;
}

void markUnsaved()
{
    if (lock()) {
        g_saved = false;
        unlock();
    }
}

void markSaved()
{
    if (lock()) {
        g_saved = true;
        unlock();
    }
}

void bumpTableGeneration()
{
    if (lock()) {
        ++g_tableGeneration;
        unlock();
    }
}

/**
 * @brief 表を編集したコマンドの共通の締めくくり。
 *
 * **呼び出し側は service.unlockTable() で既に離していること**
 * （resetStats()/reinitEkf() が内部で lockTable()/unlockTable() を
 * 使って取り直すため、離す前に呼ぶとデッドロックする）。
 */
void afterTableEdit()
{
    markUnsaved();
    if (g_service != nullptr) {
        g_service->resetStats();
        g_service->reinitEkf(); // enableEkf==false のときは no-op
    }
    bumpTableGeneration();
}

/* ------------------------------------------------------------------ 引数解析 */

/** 16 進（0x 付き）でも 10 進でも受け付けてショートアドレスにする。 */
bool parseShortAddr(const char* text, uint16_t* out)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    char* end             = nullptr;
    errno                  = 0;
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

/** 実数を読む。NaN / Inf / 絶対値が limit 超は拒否する。 */
bool parseReal(const char* text, float limit, float* out)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    char* end        = nullptr;
    errno            = 0;
    const double v   = std::strtod(text, &end);
    if (end == text || *end != '\0' || errno != 0) {
        return false;
    }
    if (!std::isfinite(v) || v < -static_cast<double>(limit) || v > static_cast<double>(limit)) {
        return false;
    }
    *out = static_cast<float>(v);
    return true;
}

/** 添字を読む。0 以上 limit 未満のみ受け付ける。 */
bool parseIndex(const char* text, size_t limit, size_t* out)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    char* end             = nullptr;
    errno                  = 0;
    const unsigned long v = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0' || errno != 0 || v >= limit) {
        return false;
    }
    *out = static_cast<size_t>(v);
    return true;
}

/* ------------------------------------------------------------ anchor コマンド */

void printUsageAnchor()
{
    std::printf("使い方:\n");
    std::printf("  anchor list                              登録テーブルを表示\n");
    std::printf("  anchor set <idx> <hex_addr> <x> <y> <z>  座標を入れる（例 anchor set 0 0x0002 0.0 0.0 2.4）\n");
    std::printf("  anchor delay <idx> <meters>              アンテナ遅延オフセット [m]\n");
    std::printf("  anchor enable <idx>                      測距・測位に使う\n");
    std::printf("  anchor disable <idx>                     測距・測位から外す\n");
    std::printf("  anchor count <n>                         有効件数（1〜%u）\n",
                static_cast<unsigned>(uwb::kMaxAnchors));
}

/** 呼び出し側が service.lockTable() を取った状態で呼ぶこと。 */
void printTableLocked(bool saved)
{
    std::printf("idx  addr       x[m]      y[m]      z[m]   delay[m]  enabled\n");
    const size_t count = g_table->size();
    for (size_t i = 0; i < count; ++i) {
        const uwb::AnchorEntry& e = g_table->entry(i);
        std::printf("%3u  0x%04X  %8.3f  %8.3f  %8.3f  %9.4f  %s\n", static_cast<unsigned>(i),
                    static_cast<unsigned>(e.short_addr), static_cast<double>(e.pos[0]),
                    static_cast<double>(e.pos[1]), static_cast<double>(e.pos[2]),
                    static_cast<double>(e.antenna_delay_m), e.enabled ? "yes" : "no");
    }
    size_t enabled = 0;
    for (size_t i = 0; i < count; ++i) {
        if (g_table->entry(i).enabled) {
            ++enabled;
        }
    }
    std::printf("件数 %u / 上限 %u（うち enabled %u）  NVS: %s\n", static_cast<unsigned>(count),
                static_cast<unsigned>(uwb::kMaxAnchors), static_cast<unsigned>(enabled),
                saved ? "保存済み" : "未保存（save が必要）");
    if (enabled < 4) {
        std::printf("注意: 3D 測位には有効測距が最低 4 件必要です\n");
    }
}

int cmdAnchor(int argc, char** argv)
{
    // argv[0] はコマンド名 "anchor"。argv[1] がサブコマンド、以降が引数。
    if (argc < 2) {
        printUsageAnchor();
        return 1;
    }
    const char* sub       = argv[1];
    const int nArgs       = argc - 2;
    const char* const* a  = argv + 2;

    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }
    SemaphoreHandle_t mtx = g_service->tableMutex();
    if (mtx == nullptr) {
        std::printf("内部エラー: 測位サービスが起動していません\n");
        return 1;
    }

    if (std::strcmp(sub, "list") == 0) {
        const bool saved = readSaved();
        g_service->lockTable();
        printTableLocked(saved);
        g_service->unlockTable();
        return 0;
    }

    if (std::strcmp(sub, "set") == 0) {
        if (nArgs != 5) {
            std::printf("引数の数が違います。例: anchor set 0 0x0002 0.0 0.0 2.4\n");
            return 1;
        }
        size_t idx    = 0;
        uint16_t addr = 0;
        float pos[3]  = {0, 0, 0};
        if (!parseIndex(a[0], uwb::kMaxAnchors, &idx)) {
            std::printf("idx が範囲外です（0〜%u）: %s\n", static_cast<unsigned>(uwb::kMaxAnchors - 1), a[0]);
            return 1;
        }
        if (!parseShortAddr(a[1], &addr)) {
            std::printf("アドレスが不正です（0x0000〜0xFFFE）: %s\n", a[1]);
            return 1;
        }
        if (addr == g_info.tagAddr) {
            std::printf("タグ自身のアドレス 0x%04X は使えません\n", static_cast<unsigned>(addr));
            return 1;
        }
        for (int i = 0; i < 3; ++i) {
            if (!parseReal(a[2 + i], uwb::cfg::kMaxCoordM, &pos[i])) {
                std::printf("座標が不正です: %s\n", a[2 + i]);
                return 1;
            }
        }

        g_service->lockTable();

        // 【修正6】docs/archive/REVIEW_2026-08-21.md app層M-3: 自タグの
        // アドレスとの一致（上のチェック）だけでは、既に登録済みの「他の
        // スロット」と同じアドレスを別スロットへ入れることを防げない。
        // 同一の物理アンカーを別座標の2観測として測位に混ぜてしまうため、
        // 自スロット(idx)以外に同じアドレスが無いか確認してから拒否する。
        const size_t count = g_table->size();
        for (size_t i = 0; i < count; ++i) {
            if ((i != idx) && (g_table->entry(i).short_addr == addr)) {
                g_service->unlockTable();
                std::printf("アドレス 0x%04X は既に anchor[%u] で使われています\n",
                            static_cast<unsigned>(addr), static_cast<unsigned>(i));
                return 1;
            }
        }

        // idx が現在の件数以上なら、そこまで件数を伸ばす。間のスロットは
        // 「未設定（enabled=false）」のプレースホルダになる。件数が変わる
        // 編集は AnchorTable::update() では表せないため set() で丸ごと
        // 差し替える。
        bool extended = false;
        size_t newCount = count;
        if (idx >= count) {
            uwb::AnchorEntry buf[uwb::kMaxAnchors];
            for (size_t i = 0; i < count; ++i) {
                buf[i] = g_table->entry(i);
            }
            for (size_t i = count; i <= idx; ++i) {
                buf[i] = uwb::AnchorEntry{};
            }
            buf[idx].short_addr = addr;
            buf[idx].pos[0]       = pos[0];
            buf[idx].pos[1]       = pos[1];
            buf[idx].pos[2]       = pos[2];
            buf[idx].enabled       = true;
            newCount                = idx + 1;
            if (!g_table->set(buf, newCount)) {
                g_service->unlockTable();
                std::printf("テーブルの更新に失敗しました\n");
                return 1;
            }
            extended = true;
        } else {
            uwb::AnchorEntry e = g_table->entry(idx);
            e.short_addr         = addr;
            e.pos[0]              = pos[0];
            e.pos[1]              = pos[1];
            e.pos[2]              = pos[2];
            // 座標を入れたということは使うつもりのはずなので有効化する。
            // antenna_delay_m は既存の値を保つ（anchor delay で別に設定する）。
            e.enabled = true;
            g_table->update(idx, e);
        }

        if (g_info.applyPlacementPolicy != nullptr) {
            g_info.applyPlacementPolicy(*g_table);
        }
        g_service->unlockTable();

        afterTableEdit();

        std::printf("anchor[%u] = 0x%04X (%.3f, %.3f, %.3f) enabled=yes\n", static_cast<unsigned>(idx),
                    static_cast<unsigned>(addr), static_cast<double>(pos[0]), static_cast<double>(pos[1]),
                    static_cast<double>(pos[2]));
        if (extended) {
            std::printf("件数を %u 件に広げました\n", static_cast<unsigned>(newCount));
        }
        std::printf("次の測位周期から反映されます。残すには save を実行してください\n");
        return 0;
    }

    if (std::strcmp(sub, "delay") == 0) {
        if (nArgs != 2) {
            std::printf("引数の数が違います。例: anchor delay 0 0.15\n");
            return 1;
        }
        g_service->lockTable();
        const size_t count = g_table->size();
        g_service->unlockTable();

        size_t idx  = 0;
        float delay = 0.0f;
        if (!parseIndex(a[0], count, &idx)) {
            std::printf("idx が範囲外です（0〜%u）: %s\n", static_cast<unsigned>((count == 0) ? 0 : count - 1),
                        a[0]);
            return 1;
        }
        if (!parseReal(a[1], uwb::cfg::kMaxAntennaDelayM, &delay)) {
            std::printf("値が不正です: %s\n", a[1]);
            return 1;
        }

        g_service->lockTable();
        uwb::AnchorEntry e   = g_table->entry(idx);
        e.antenna_delay_m     = delay;
        g_table->update(idx, e);
        g_service->unlockTable();

        afterTableEdit();
        std::printf("anchor[%u].antenna_delay_m = %.4f m（次の測位周期から反映）\n",
                    static_cast<unsigned>(idx), static_cast<double>(delay));
        return 0;
    }

    const bool isEnable  = (std::strcmp(sub, "enable") == 0);
    const bool isDisable = (std::strcmp(sub, "disable") == 0);
    if (isEnable || isDisable) {
        if (nArgs != 1) {
            std::printf("引数の数が違います。例: anchor %s 0\n", isEnable ? "enable" : "disable");
            return 1;
        }
        g_service->lockTable();
        const size_t count = g_table->size();
        g_service->unlockTable();

        size_t idx = 0;
        if (!parseIndex(a[0], count, &idx)) {
            std::printf("idx が範囲外です（0〜%u）: %s\n", static_cast<unsigned>((count == 0) ? 0 : count - 1),
                        a[0]);
            return 1;
        }

        g_service->lockTable();
        uwb::AnchorEntry e = g_table->entry(idx);
        e.enabled            = isEnable;
        g_table->update(idx, e);
        if (g_info.applyPlacementPolicy != nullptr) {
            g_info.applyPlacementPolicy(*g_table);
        }
        g_service->unlockTable();

        afterTableEdit();
        std::printf("anchor[%u].enabled = %s（次の測位周期から反映）\n", static_cast<unsigned>(idx),
                    isEnable ? "yes" : "no");
        return 0;
    }

    if (std::strcmp(sub, "count") == 0) {
        if (nArgs != 1) {
            std::printf("引数の数が違います。例: anchor count 5\n");
            return 1;
        }
        size_t n = 0;
        // 1〜kMaxAnchors を受け付けたいので、上限は kMaxAnchors+1 を渡す。
        if (!parseIndex(a[0], uwb::kMaxAnchors + 1, &n) || n == 0) {
            std::printf("件数が範囲外です（1〜%u）: %s\n", static_cast<unsigned>(uwb::kMaxAnchors), a[0]);
            return 1;
        }

        g_service->lockTable();
        // 増やしたぶんは「未設定（enabled=false）」で埋める。減らすぶんは
        // 単に buf に写さない（AnchorTable::set() が丸ごと差し替える）。
        uwb::AnchorEntry buf[uwb::kMaxAnchors];
        const size_t oldCount = g_table->size();
        for (size_t i = 0; i < n && i < oldCount; ++i) {
            buf[i] = g_table->entry(i);
        }
        for (size_t i = oldCount; i < n; ++i) {
            buf[i] = uwb::AnchorEntry{};
        }
        const bool ok = g_table->set(buf, n);
        if (ok && g_info.applyPlacementPolicy != nullptr) {
            g_info.applyPlacementPolicy(*g_table);
        }
        g_service->unlockTable();

        if (!ok) {
            std::printf("テーブルの更新に失敗しました\n");
            return 1;
        }

        afterTableEdit();
        std::printf("件数を %u 件にしました（次の測位周期から反映）。"
                    "新しいスロットは anchor set で設定してください\n",
                    static_cast<unsigned>(n));
        return 0;
    }

    std::printf("不明なサブコマンド: %s\n", sub);
    printUsageAnchor();
    return 1;
}

/* ------------------------------------------------------------ output コマンド */

int cmdOutput(int argc, char** argv)
{
    if (argc > 2) {
        std::printf("使い方: output [on|off]\n");
        return 1;
    }

    if (argc < 2) {
        bool on = true;
        if (lock()) {
            on = g_jsonOutput;
            unlock();
        }
        std::printf("json output = %s\n", on ? "on" : "off");
        return 0;
    }

    const char* mode = argv[1];
    bool on           = true;
    if (std::strcmp(mode, "on") == 0) {
        on = true;
    } else if (std::strcmp(mode, "off") == 0) {
        on = false;
    } else {
        std::printf("使い方: output [on|off]\n");
        return 1;
    }

    if (lock()) {
        g_jsonOutput = on;
        unlock();
    }
    if (on) {
        // 出力を止めている間に設定を変えたかどうかに関わらず、再開直後に
        // "anchors" 行を出し直したい（下流の JSON Lines 消費側は、途中から
        // 読み始めるとアンカー一覧を取り逃すため）。tableGeneration() を
        // 進めることで、それを監視している uwb_log タスクに再出力させる。
        bumpTableGeneration();
    }
    std::printf("json output = %s\n", on ? "on" : "off");
    return 0;
}

/* -------------------------------------------------------------------- save */

int cmdSave(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }

    uwb::AnchorEntry snapshot[uwb::kMaxAnchors];
    size_t count = 0;
    g_service->lockTable();
    count = g_table->size();
    for (size_t i = 0; i < count; ++i) {
        snapshot[i] = g_table->entry(i);
    }
    g_service->unlockTable();

    if (count == 0) {
        std::printf("テーブルが空です。anchor set / anchor count で設定してください\n");
        return 1;
    }

    const esp_err_t err = uwb::ConfigStore::saveAnchorTable(snapshot, count);
    if (err != ESP_OK) {
        std::printf("保存に失敗しました (err=%s)\n", esp_err_to_name(err));
        return 1;
    }

    markSaved();
    std::printf("NVS へ保存しました: アンカー %u 件\n", static_cast<unsigned>(count));
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

    size_t restored = 0;
    if (g_info.defaults != nullptr && g_table != nullptr && g_service != nullptr) {
        restored = (g_info.defaultCount < uwb::kMaxAnchors) ? g_info.defaultCount : uwb::kMaxAnchors;
        g_service->lockTable();
        const bool ok = g_table->set(g_info.defaults, restored);
        if (ok && g_info.applyPlacementPolicy != nullptr) {
            g_info.applyPlacementPolicy(*g_table);
        }
        g_service->unlockTable();
        if (!ok) {
            restored = 0;
        } else {
            afterTableEdit();
        }
    }
    std::printf("NVS を消去し、kAnchors[] の既定値 %u 件に戻しました（次の測位周期から反映）\n",
                static_cast<unsigned>(restored));
    return 0;
}

/* -------------------------------------------------------------------- info */

int cmdInfo(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }

    uwb::CycleResult result;
    const bool haveResult = g_service->getLatest(result);

    g_service->lockTable();
    const size_t count = g_table->size();
    size_t enabled       = 0;
    for (size_t i = 0; i < count; ++i) {
        if (g_table->entry(i).enabled) {
            ++enabled;
        }
    }
    g_service->unlockTable();

    const bool saved  = readSaved();
    bool jsonOn         = true;
    if (lock()) {
        jsonOn = g_jsonOutput;
        unlock();
    }

    std::printf("=== uwb_tag ===\n");
    std::printf("  board        : %s\n", g_info.boardName);
    std::printf("  method       : %s\n", g_info.methodName);
    std::printf("  device id    : 0x%08lX  chip: %s\n", static_cast<unsigned long>(g_info.deviceId),
                g_info.chipName);
    std::printf("  tag addr     : 0x%04X   pan id: 0x%04X\n", static_cast<unsigned>(g_info.tagAddr),
                static_cast<unsigned>(g_info.panId));
    std::printf("  anchors      : %u 件（enabled %u 件, 起動時の設定元: %s, NVS: %s）\n",
                static_cast<unsigned>(count), static_cast<unsigned>(enabled),
                uwb::configSourceName(g_info.source), saved ? "保存済み" : "未保存（save が必要）");
    std::printf("  nvs          : %s\n", uwb::ConfigStore::isReady() ? "使用可" : "使用不可（既定値のみ）");
    std::printf("  json output  : %s\n", jsonOn ? "on" : "off");
    // 【v2での変更】RangingService は「累積の epoch数/ok数」を公開していない
    // （CycleResult は直近1周期ぶんのスナップショットのみ。
    // docs/ARCHITECTURE_V2.md §3.1 の API にも累積カウンタは無い）。
    // 旧来の「epochs（ok率%）」表示は落とし、直近の seq（単調増加の周期番号）
    // と cycle_ms、直近の ok/ambiguous を表示する形に変えた。
    if (haveResult) {
        std::printf("  last cycle   : seq=%lu  cycle: %lu ms\n", static_cast<unsigned long>(result.seq),
                    static_cast<unsigned long>(result.cycleMs));
        if (result.lv2.ok) {
            std::printf("  last fix     : p=(%.3f, %.3f, %.3f) gdop=%.2f rms=%.3f n=%d/%d ambiguous=%s\n",
                        static_cast<double>(result.lv2.p[0]), static_cast<double>(result.lv2.p[1]),
                        static_cast<double>(result.lv2.p[2]), static_cast<double>(result.lv2.gdop),
                        static_cast<double>(result.lv2.residualRms), result.lv2.nUsed, result.lv2.nTotal,
                        result.lv2.ambiguous ? "yes" : "no");
        } else {
            std::printf("  last fix     : (未取得 / 直近は測位不能)\n");
        }
    } else {
        std::printf("  last cycle   : (測位サービスからまだ結果を受け取っていません)\n");
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
    const esp_console_cmd_t anchorCmd = {
        .command  = "anchor",
        .help     = "アンカー登録テーブルの表示・編集（anchor list / set / delay / enable / disable / count）",
        .hint     = " list | set <idx> <hex_addr> <x> <y> <z> | delay <idx> <m> | enable <idx> | "
                    "disable <idx> | count <n>",
        .func     = &cmdAnchor,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&anchorCmd));

    const esp_console_cmd_t outputCmd = {
        .command  = "output",
        .help     = "JSON Lines 出力を一時的に止める・再開する（設定作業中に使う）",
        .hint     = " [on|off]",
        .func     = &cmdOutput,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&outputCmd));

    const esp_console_cmd_t saveCmd = {
        .command  = "save",
        .help     = "現在のアンカー登録テーブルを NVS へ保存する",
        .hint     = nullptr,
        .func     = &cmdSave,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&saveCmd));

    const esp_console_cmd_t resetCmd = {
        .command  = "reset-config",
        .help     = "NVS を消去して kAnchors[] の既定値に戻す",
        .hint     = nullptr,
        .func     = &cmdResetConfig,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&resetCmd));

    const esp_console_cmd_t infoCmd = {
        .command  = "info",
        .help     = "Device ID / チップ名 / 現在の設定 / 測位状況を表示する",
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

bool sharedInit(uwb::AnchorTable& table, uwb::RangingService& service, const StaticInfo& info)
{
    if (info.defaults == nullptr) {
        ESP_LOGE(kLogTag, "sharedInit() に defaults=nullptr の StaticInfo が渡されました");
        return false;
    }
    if (g_mutex == nullptr) {
        g_mutex = xSemaphoreCreateMutex();
    }
    if (g_mutex == nullptr) {
        ESP_LOGE(kLogTag, "ミューテックスを作れませんでした");
        return false;
    }

    g_table   = &table;
    g_service = &service;
    g_info     = info;
    g_saved    = (info.source == uwb::ConfigSource::Nvs);
    g_jsonOutput       = true;
    g_tableGeneration = 0;
    return true;
}

uint32_t tableGeneration()
{
    uint32_t v = 0;
    if (lock()) {
        v = g_tableGeneration;
        unlock();
    }
    return v;
}

bool jsonOutputEnabled()
{
    if (!lock()) {
        return true;
    }
    const bool on = g_jsonOutput;
    unlock();
    return on;
}

esp_err_t consoleStart()
{
    esp_console_repl_t* repl          = nullptr;
    esp_console_repl_config_t replCfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    replCfg.prompt                     = "uwb-tag>";
    replCfg.max_cmdline_length         = 160;
    // 既定の 4096 から増やしてある。anchor list は 1 行あたり 4 個の浮動小数点を
    // printf するので、newlib の vfprintf のぶんの余裕を見ておく。
    replCfg.task_stack_size            = 6144;

    esp_console_register_help_command();
    registerCommands();
    // "wifi" コマンドをこの esp_console のコマンド一覧へ追加登録する
    // （uwb_net.hpp 冒頭コメント §1・scratchpad/NET_SPEC.md §8）。ここで
    // 登録した内容は USB シリアルの REPL だけでなく、uwb_net のブラウザ
    // WebSocket / TCP コンソール（同じ esp_console_run() 経由）からも
    // そのまま呼べる。CONFIG_UWB_NET_ENABLE=n のときは no-op。
    uwb::net::registerConsoleCommands();

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

} // namespace tagapp
