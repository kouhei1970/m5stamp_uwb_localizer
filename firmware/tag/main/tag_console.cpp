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
 *   mode                                      現在の測位モードと判定理由を表示
 *   mode auto|2d|3d                           測位モードを手動で切り替える（既定 auto）
 *   height                                    2D測位の固定高さ(z_fixed)を表示
 *   height <meters>                           2D測位の固定高さを設定（例 height -1.2、範囲 ±10m）
 *   survey dist <i> <j> <meters>              メジャーで測った距離を記録（RAMのみ）
 *   survey z <i> <meters>                     アンカーの高さを記録（既定0）
 *   survey show                               記録済みの入力一覧 + 計算可能なら座標プレビュー
 *   survey apply                              座標を計算してアンカー登録テーブルへ書き込む
 *   survey clear                              survey の入力を全消去
 *   ekf                                        EKF(拡張カルマンフィルタ)のQ/R/ゲートを表示
 *   ekf q <sigma_a>                            プロセス雑音Qの強さ [m/s^2]（範囲 0.01〜50）
 *   ekf r <sigma0> [per_m]                     観測雑音R=(sigma0+per_m*距離)^2 [m]
 *   ekf gate <k>                               イノベーションゲート [sigma]（範囲 0.5〜20）
 *   ekf model cv|ca                            運動モデル（cv=等速、ca=等加速度）
 *   output <on|off>                           JSON Lines 出力の一時停止
 *   save                                      NVS へ保存（アンカー登録テーブル + 測位モード設定）
 *   reset-config                              NVS を消して kAnchors[] の既定値・モードAutoへ戻す
 *   info                                      Device ID / チップ名 / 現在の設定 / 測位状況
 *   reboot                                    再起動
 *
 * REPL のデバイスは USB-Serial/JTAG を既定とする（M5StampS3A は
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
#include "nvs.h"

#include "uwb_cfgstore.hpp"
#include "uwb_net.hpp"
#include "uwb_port.h"
#include "uwb_survey_tape.h"

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

/** `survey dist`/`survey z` で記録中の入力（メジャー実測値からの閉形式
 *  座標計算 uwb_survey_tape_solve() 用。components/uwb_survey/include/
 *  uwb_survey_tape.h 参照）。NVSには保存しない（アンカー登録テーブルに
 *  書き込む前の作業用スクラッチ）。
 *
 *  g_mutex では守らない: コンソールは単一タスクの REPL で、この入力は
 *  他のタスク（測位ループ）からは参照されない（g_table/g_service 越しの
 *  アンカー登録テーブルとは別物）ため、他の共有状態と違って排他制御が
 *  要らない。`{}` で全ゼロ初期化されるので n=0（uwb_survey_tape_input_init
 *  と同じ意味）。実際の n は使わず、show/apply のたびに登録テーブルの
 *  台数で上書きする（台数は anchor set/count で変わりうるため）。 */
uwb_survey_tape_input g_surveyInput{};

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
 * @brief 設定（アンカー登録テーブル、または測位モード設定）を編集した
 * コマンドの共通の締めくくり。
 *
 * anchor set/delay/enable/disable/count だけでなく、mode/height コマンド
 * （AnchorTable::setModeOverride()/setZFixedM() を呼ぶ）、survey apply
 * （メジャー実測値から計算した座標を書き込む）の後にも共通して呼ぶ
 * （NVS未保存フラグ・統計リセット・EKF再構成・下流への再通知はどの編集
 * でも同じ後処理が必要なため）。
 *
 * **呼び出し側は service.unlockTable() で既に離していること**
 * （resetStats()/reinitEkf() が内部で lockTable()/unlockTable() を
 * 使って取り直すため、離す前に呼ぶとデッドロックする）。
 */
void afterConfigEdit()
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
    // 台数からの静的な注意書きではなく、実際に決まっている測位モード
    // （uwb::AnchorTable::evaluateMode()、有効台数・配置・手動オーバーライドの
    // 全部を反映済み）をそのまま出す。詳細理由は `mode` コマンドで見られる。
    const uwb::ModeDecision d = g_table->modeDecision();
    std::printf("測位モード: %s（%s）\n", uwb::positioningModeName(d.mode), uwb::modeReasonText(d.reason));
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

        if (g_info.applyModePolicy != nullptr) {
            g_info.applyModePolicy(*g_table);
        }
        g_service->unlockTable();

        afterConfigEdit();

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

        afterConfigEdit();
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
        if (g_info.applyModePolicy != nullptr) {
            g_info.applyModePolicy(*g_table);
        }
        g_service->unlockTable();

        afterConfigEdit();
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
        if (ok && g_info.applyModePolicy != nullptr) {
            g_info.applyModePolicy(*g_table);
        }
        g_service->unlockTable();

        if (!ok) {
            std::printf("テーブルの更新に失敗しました\n");
            return 1;
        }

        afterConfigEdit();
        std::printf("件数を %u 件にしました（次の測位周期から反映）。"
                    "新しいスロットは anchor set で設定してください\n",
                    static_cast<unsigned>(n));
        return 0;
    }

    std::printf("不明なサブコマンド: %s\n", sub);
    printUsageAnchor();
    return 1;
}

/* -------------------------------------------------------------- mode コマンド */

/** uwb::ModeOverride の表示名（"auto"/"2d"/"3d"）。console 表示専用
 *  （uwb::positioningModeName() は PositioningMode 用で Auto に対応する値が
 *  無いため別に用意する）。 */
const char* overrideName(uwb::ModeOverride o)
{
    switch (o) {
    case uwb::ModeOverride::Auto:
        return "auto";
    case uwb::ModeOverride::Force2D:
        return "2d";
    case uwb::ModeOverride::Force3D:
        return "3d";
    }
    return "unknown";
}

void printUsageMode()
{
    std::printf("使い方:\n");
    std::printf("  mode              現在の測位モードと判定理由を表示\n");
    std::printf("  mode auto|2d|3d   手動で切り替える（既定 auto）\n");
}

/** モード決定内容を表示する（呼び出し側が service.lockTable() を取った
 *  状態、または取らずに読んだコピーを渡す。d は値渡しなのでどちらでもよい）。 */
void printModeDecision(const uwb::ModeDecision& d)
{
    std::printf("mode     : %s（%s）\n", uwb::positioningModeName(d.mode), uwb::modeReasonText(d.reason));
    std::printf("override : %s\n", overrideName(d.override));
    std::printf("有効アンカー: %u台", static_cast<unsigned>(d.enabledCount));
    if (d.enabledCount >= 3) {
        std::printf("  配置: %s\n", d.coplanar ? "同一平面" : "立体");
    } else {
        std::printf("\n");
    }
    if (d.mode == uwb::PositioningMode::Mode2D) {
        std::printf("z_fixed  : %.3f m\n", static_cast<double>(d.zFixedM));
    }
    if (d.mode == uwb::PositioningMode::Mode3D && d.forcedCoplanarWarning) {
        std::printf("警告: 同一平面配置での3D強制です。ambiguousフラグを必ず確認してください\n");
    }
}

int cmdMode(int argc, char** argv)
{
    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }

    if (argc == 1) {
        g_service->lockTable();
        const uwb::ModeDecision d = g_table->modeDecision();
        g_service->unlockTable();
        printModeDecision(d);
        return 0;
    }

    if (argc != 2) {
        printUsageMode();
        return 1;
    }

    const char* arg = argv[1];
    uwb::ModeOverride newOverride;
    if (std::strcmp(arg, "auto") == 0) {
        newOverride = uwb::ModeOverride::Auto;
    } else if (std::strcmp(arg, "2d") == 0) {
        newOverride = uwb::ModeOverride::Force2D;
    } else if (std::strcmp(arg, "3d") == 0) {
        newOverride = uwb::ModeOverride::Force3D;
    } else {
        printUsageMode();
        return 1;
    }

    g_service->lockTable();
    g_table->setModeOverride(newOverride);
    if (g_info.applyModePolicy != nullptr) {
        g_info.applyModePolicy(*g_table);
    }
    const uwb::ModeDecision d = g_table->modeDecision();
    g_service->unlockTable();

    afterConfigEdit();

    printModeDecision(d);
    std::printf("次の測位周期から反映されます。残すには save を実行してください\n");
    return 0;
}

/* ------------------------------------------------------------ height コマンド */

int cmdHeight(int argc, char** argv)
{
    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }

    if (argc == 1) {
        g_service->lockTable();
        const float z = g_table->zFixedM();
        g_service->unlockTable();
        std::printf("height = %.3f m\n", static_cast<double>(z));
        return 0;
    }

    if (argc != 2) {
        std::printf("使い方: height [<メートル>]（例 height -1.2、範囲 ±%.0fm）\n",
                    static_cast<double>(uwb::cfg::kMaxZFixedM));
        return 1;
    }

    float z = 0.0f;
    if (!parseReal(argv[1], uwb::cfg::kMaxZFixedM, &z)) {
        std::printf("値が不正です（範囲 ±%.0fm）: %s\n", static_cast<double>(uwb::cfg::kMaxZFixedM), argv[1]);
        return 1;
    }

    g_service->lockTable();
    g_table->setZFixedM(z);
    if (g_info.applyModePolicy != nullptr) {
        g_info.applyModePolicy(*g_table);
    }
    const uwb::ModeDecision d = g_table->modeDecision();
    g_service->unlockTable();

    afterConfigEdit();

    std::printf("height = %.3f m（次の測位周期から反映。2D測位でないと今は使われません。残すには save）\n",
                static_cast<double>(z));
    if (d.mode == uwb::PositioningMode::Mode2D) {
        std::printf("現在2D測位中なので、この値がすぐ z_fixed として使われます\n");
    }
    return 0;
}

/* ------------------------------------------------------------ ekf コマンド */

/* EKF(拡張カルマンフィルタ、Lv3)のQ(プロセス雑音)/R(観測雑音)/イノベーション
 * ゲートを実行時に調整するコマンド。値は uwb::AnchorTable が保持する
 * uwb::EkfTuning（components/uwb_ranging/include/uwb_ranging_types.hpp）に
 * 書き込み、afterConfigEdit() 経由で g_service->reinitEkf() を呼んで
 * PositioningPipeline::initEkf() をQ/ゲート込みで組み直す（R はアンカー
 * ごとの uwb_anchor.sigma0/sigma_per_m へ AnchorTable::setEkfTuning() が
 * 直接反映する）。cmdHeight() と同じ lockTable()/unlockTable() ->
 * afterConfigEdit() の作法。 */

void printUsageEkf()
{
    std::printf("使い方:\n");
    std::printf("  ekf                      現在のQ/R/ゲートを表示\n");
    std::printf("  ekf q <sigma_a>          プロセス雑音Qの強さ [m/s^2]（範囲 0.01〜50、例 ekf q 0.8）\n");
    std::printf("  ekf r <sigma0> [per_m]   観測雑音 R=(sigma0+per_m*距離)^2 [m]"
                "（sigma0 範囲 0.001〜10、per_m 範囲 0〜1、例 ekf r 0.05 0.01）\n");
    std::printf("  ekf gate <k>             イノベーションゲート [sigma]（範囲 0.5〜20、例 ekf gate 4）\n");
    std::printf("  ekf model cv|ca          運動モデル（cv=等速、ca=等加速度、既定cv）\n");
}

/** CVモデル(状態=位置+速度)のQ対角成分[位置 m^2, 速度 (m/s)^2]を dt・sigma_a
 *  から計算する。components/uwb_loc/src/uwb_ekf.c transition() の k==2 分岐
 *  （q1[0]=dt^3/3*sigma_a^2, q1[3]=dt*sigma_a^2）と同じ式。transition() は
 *  static でここから呼べないため、表示専用にこの2項だけ複製してある
 *  （実際にEKFが使う値は uwb_ekf_predict() 経由の transition() が本体）。 */
void ekfQDiagCv(float sigmaA, float dt, double* qPos, double* qVel)
{
    const double s2 = static_cast<double>(sigmaA) * static_cast<double>(sigmaA);
    const double d1 = static_cast<double>(dt);
    const double d3 = d1 * d1 * d1;
    *qPos = d3 / 3.0 * s2;
    *qVel = d1 * s2;
}

/** 同上、CAモデル(状態=位置+速度+加速度)版。transition() の k==3 分岐
 *  （q1[0]=dt^5/20*sigma_a^2, q1[4]=dt^3/3*sigma_a^2, q1[8]=dt*sigma_a^2）。 */
void ekfQDiagCa(float sigmaA, float dt, double* qPos, double* qVel, double* qAcc)
{
    const double s2 = static_cast<double>(sigmaA) * static_cast<double>(sigmaA);
    const double d1 = static_cast<double>(dt);
    const double d3 = d1 * d1 * d1;
    const double d5 = d3 * d1 * d1;
    *qPos = d5 * 0.05 * s2;
    *qVel = d3 / 3.0 * s2;
    *qAcc = d1 * s2;
}

/** ekf コマンドの表示本体（サブコマンド無し、および各セッター実行後の
 *  確認表示の両方で使う）。 */
void printEkfStatus(const uwb::EkfTuning& t, bool saved)
{
    constexpr float kDisplayDtS = 0.025f; // Q表示用の代表周期 25ms（典型的な1周期）
    constexpr float kDisplayDM  = 2.0f;   // R表示用の代表距離 2m
    const bool isCa               = (t.model == 1);

    std::printf("model    : %s (%s)\n", isCa ? "ca" : "cv",
                isCa ? "等加速度モデル: 状態=位置+速度+加速度" : "等速モデル: 状態=位置+速度");

    if (isCa) {
        double qPos = 0.0, qVel = 0.0, qAcc = 0.0;
        ekfQDiagCa(t.sigmaA, kDisplayDtS, &qPos, &qVel, &qAcc);
        std::printf("q sigma_a: %.3f m/s^3  (加加速度の連続白色雑音。Q(dt=%.0fms) 位置 %.1e m^2, "
                    "速度 %.1e (m/s)^2, 加速度 %.1e (m/s^2)^2)\n",
                    static_cast<double>(t.sigmaA), static_cast<double>(kDisplayDtS) * 1000.0, qPos, qVel, qAcc);
    } else {
        double qPos = 0.0, qVel = 0.0;
        ekfQDiagCv(t.sigmaA, kDisplayDtS, &qPos, &qVel);
        std::printf("q sigma_a: %.3f m/s^2  (加速度の連続白色雑音。Q(dt=%.0fms) 位置 %.1e m^2, "
                    "速度 %.1e (m/s)^2)\n",
                    static_cast<double>(t.sigmaA), static_cast<double>(kDisplayDtS) * 1000.0, qPos, qVel);
    }

    const double rAtD = static_cast<double>(t.sigmaR0) +
                        static_cast<double>(t.sigmaRPerM) * static_cast<double>(kDisplayDM);
    std::printf("r sigma0 : %.3f m  per_m: %.3f  (R = (sigma0 + per_m*d)^2, d=%.0fm で %.1e m^2)\n",
                static_cast<double>(t.sigmaR0), static_cast<double>(t.sigmaRPerM),
                static_cast<double>(kDisplayDM), rAtD * rAtD);
    std::printf("gate     : %.1f sigma (イノベーション棄却しきい値)\n", static_cast<double>(t.gate));
    std::printf("NVS      : %s\n", saved ? "保存済み" : "未保存（save が必要）");
}

int cmdEkf(int argc, char** argv)
{
    if (g_table == nullptr || g_service == nullptr) {
        std::printf("内部エラー: コンソールが初期化されていません\n");
        return 1;
    }

    if (argc == 1) {
        g_service->lockTable();
        const uwb::EkfTuning t = g_table->ekfTuning();
        g_service->unlockTable();
        printEkfStatus(t, readSaved());
        return 0;
    }

    const char* sub = argv[1];

    if (std::strcmp(sub, "q") == 0) {
        if (argc != 3) {
            printUsageEkf();
            return 1;
        }
        float v = 0.0f;
        if (!parseReal(argv[2], 50.0f, &v) || v < 0.01f) {
            std::printf("値が不正です（範囲 0.01〜50 m/s^2）: %s\n", argv[2]);
            return 1;
        }
        g_service->lockTable();
        uwb::EkfTuning t = g_table->ekfTuning();
        t.sigmaA           = v;
        g_table->setEkfTuning(t);
        g_service->unlockTable();
        afterConfigEdit();
        printEkfStatus(t, readSaved());
        std::printf("次の測位周期から反映されます。残すには save を実行してください\n");
        return 0;
    }

    if (std::strcmp(sub, "r") == 0) {
        if (argc != 3 && argc != 4) {
            printUsageEkf();
            return 1;
        }
        float sigma0 = 0.0f;
        if (!parseReal(argv[2], 10.0f, &sigma0) || sigma0 < 0.001f) {
            std::printf("sigma0 が不正です（範囲 0.001〜10 m）: %s\n", argv[2]);
            return 1;
        }
        float perM       = 0.0f;
        const bool havePerM = (argc == 4);
        if (havePerM && (!parseReal(argv[3], 1.0f, &perM) || perM < 0.0f)) {
            std::printf("per_m が不正です（範囲 0〜1）: %s\n", argv[3]);
            return 1;
        }

        g_service->lockTable();
        uwb::EkfTuning t = g_table->ekfTuning();
        t.sigmaR0          = sigma0;
        if (havePerM) {
            t.sigmaRPerM = perM;
        }
        g_table->setEkfTuning(t);
        g_service->unlockTable();
        afterConfigEdit();
        printEkfStatus(t, readSaved());
        std::printf("次の測位周期から反映されます。残すには save を実行してください\n");
        return 0;
    }

    if (std::strcmp(sub, "gate") == 0) {
        if (argc != 3) {
            printUsageEkf();
            return 1;
        }
        float v = 0.0f;
        if (!parseReal(argv[2], 20.0f, &v) || v < 0.5f) {
            std::printf("値が不正です（範囲 0.5〜20 sigma）: %s\n", argv[2]);
            return 1;
        }
        g_service->lockTable();
        uwb::EkfTuning t = g_table->ekfTuning();
        t.gate             = v;
        g_table->setEkfTuning(t);
        g_service->unlockTable();
        afterConfigEdit();
        printEkfStatus(t, readSaved());
        std::printf("次の測位周期から反映されます。残すには save を実行してください\n");
        return 0;
    }

    if (std::strcmp(sub, "model") == 0) {
        if (argc != 3) {
            printUsageEkf();
            return 1;
        }
        uint8_t model = 0;
        if (std::strcmp(argv[2], "cv") == 0) {
            model = 0;
        } else if (std::strcmp(argv[2], "ca") == 0) {
            model = 1;
        } else {
            std::printf("運動モデルが不正です（cv または ca）: %s\n", argv[2]);
            return 1;
        }
        g_service->lockTable();
        uwb::EkfTuning t = g_table->ekfTuning();
        t.model             = model;
        g_table->setEkfTuning(t);
        g_service->unlockTable();
        afterConfigEdit();
        printEkfStatus(t, readSaved());
        std::printf("次の測位周期から反映されます（起動直後の1回だけ立ち上げ直します）。"
                    "残すには save を実行してください\n");
        return 0;
    }

    printUsageEkf();
    return 1;
}

/* ------------------------------------------------------------ survey コマンド */

/* メジャー実測値からの閉形式アンカー座標計算（components/uwb_survey/include/
 * uwb_survey_tape.h）。`survey dist`/`survey z` で少数の距離・高さを記録し、
 * `survey show` でプレビュー、`survey apply` でアンカー登録テーブルへ
 * 書き込む。座標系の規約（index0=原点、index1=+X軸、index2=+y側固定）は
 * アンカー登録テーブルの並び（`anchor list` と同じ番号）そのもの。
 *
 * ESP-NOW での総当たり自己測量（uwb_survey_solve()、docs/SURVEY_SPEC.md
 * S3〜S5、未実装）とは別の、より軽い経路。台数が少ない・巻尺だけで
 * 済ませたい場合に使う。 */

/** dist/z の添字として受け付ける上限。アンカー登録テーブルの上限
 *  (uwb::kMaxAnchors) と閉形式計算側の上限 (UWB_SURVEY_TAPE_MAX_NODES) の
 *  小さい方（既定はどちらも8なので通常は同じ値になる）。 */
size_t surveyIndexLimit()
{
    const size_t tapeMax = static_cast<size_t>(UWB_SURVEY_TAPE_MAX_NODES);
    return (uwb::kMaxAnchors < tapeMax) ? uwb::kMaxAnchors : tapeMax;
}

void printUsageSurvey()
{
    std::printf("使い方:\n");
    std::printf("  survey dist <i> <j> <meters>  メジャーで測った距離を記録（例 survey dist 0 2 4.83）\n");
    std::printf("  survey z <i> <meters>         アンカーの高さを記録（既定0、例 survey z 1 2.4）\n");
    std::printf("  survey show                   記録済みの入力一覧・座標プレビュー\n");
    std::printf("  survey apply                  座標を計算してアンカー登録テーブルへ書き込む\n");
    std::printf("  survey clear                  survey の入力を全消去\n");
    std::printf("  i, j は anchor list と同じ添字（index0=原点, index1=+X軸, index2=+y側固定）\n");
}

/** ペア(i,j)単位のビットマスク（missing/clamped_pairs）を "d(i,j)" の列で
 *  表示する（uwb_survey_link_index() と同じ添字規約）。何も立っていなければ
 *  何もしない。 */
void printPairMask(const char* label, unsigned long mask, int n)
{
    if (mask == 0UL) {
        return;
    }
    std::printf("%s:", label);
    for (int j = 1; j < n; ++j) {
        for (int i = 0; i < j; ++i) {
            const int bit = uwb_survey_link_index(i, j);
            if (bit >= 0 && ((mask >> bit) & 1UL) != 0UL) {
                std::printf(" d(%d,%d)", i, j);
            }
        }
    }
    std::printf("\n");
}

/** ノード単位のビットマスク（clamped_nodes/sign_unresolved）を表示する。 */
void printNodeMask(const char* label, unsigned long mask, int n)
{
    if (mask == 0UL) {
        return;
    }
    std::printf("%s:", label);
    for (int k = 0; k < n; ++k) {
        if (((mask >> k) & 1UL) != 0UL) {
            std::printf(" ノード%d", k);
        }
    }
    std::printf("\n");
}

/** 距離1本の入力状況を表示する（"d(i,j) = 4.830 m" または "d(i,j) = 未入力"）。
 *  i,j どちらに入れても拾う（uwb_survey_tape_solve() 側の流儀と揃えてある）。 */
void printDistEntry(int i, int j)
{
    if (g_surveyInput.have[i][j]) {
        std::printf("  d(%d,%d) = %.3f m\n", i, j, static_cast<double>(g_surveyInput.dist[i][j]));
    } else if (g_surveyInput.have[j][i]) {
        std::printf("  d(%d,%d) = %.3f m\n", i, j, static_cast<double>(g_surveyInput.dist[j][i]));
    } else {
        std::printf("  d(%d,%d) = 未入力\n", i, j);
    }
}

/** uwb_survey_tape_solve() が失敗した理由を日本語で表示する。 */
void printSurveyError(const uwb_survey_tape_result& r, int n)
{
    switch (r.status) {
    case UWB_SURVEY_TAPE_ERR_N_RANGE:
        if (n < UWB_SURVEY_TAPE_MIN_NODES) {
            std::printf("アンカー登録テーブルが%d台です（%d台以上必要）。"
                        "先に anchor set / anchor count で登録してください\n",
                        n, UWB_SURVEY_TAPE_MIN_NODES);
        } else {
            std::printf("アンカー登録テーブルが%d台です（この計算は%d台まで対応）\n", n,
                        UWB_SURVEY_TAPE_MAX_NODES);
        }
        break;
    case UWB_SURVEY_TAPE_ERR_MISSING:
        printPairMask("未入力", r.missing, n);
        break;
    case UWB_SURVEY_TAPE_ERR_BASELINE:
        std::printf("d(0,1) から水平距離を作れません（z[0]とz[1]の高さ差が d(0,1) 以上です）。"
                    "survey z 0 / survey z 1 か d(0,1) の測定値を確認してください\n");
        break;
    case UWB_SURVEY_TAPE_ERR_INCONSISTENT:
        if (r.err_i >= 0) {
            std::printf("d(%d,%d) が高さ差より短く、水平距離を計算できません"
                        "（測定値を確認してください）\n",
                        r.err_i, r.err_j);
        } else {
            std::printf("ノード%d の位置が測定値と矛盾しています（三角形 d(0,1)・d(0,%d)・"
                        "d(1,%d) を作れません。測定値を確認してください）\n",
                        r.err_j, r.err_j, r.err_j);
        }
        break;
    case UWB_SURVEY_TAPE_OK:
    default:
        break;
    }
}

/** 記録済みの入力から座標を計算する（登録テーブルへは書き込まない）。
 *  呼び出し側が n（登録テーブルの現在の台数）を渡す。 */
bool solveSurveyPreview(int n, uwb_survey_tape_result* out)
{
    uwb_survey_tape_input snapshot = g_surveyInput;
    snapshot.n                      = n;
    return uwb_survey_tape_solve(&snapshot, out) != 0;
}

/** 計算結果（座標・警告）を表示する（show のプレビューと apply の結果表示で共通）。 */
void printSurveyResult(const uwb_survey_tape_result& result, int n)
{
    for (int i = 0; i < n; ++i) {
        std::printf("  idx%d: (%.3f, %.3f, %.3f)\n", i, static_cast<double>(result.pos[i][0]),
                    static_cast<double>(result.pos[i][1]), static_cast<double>(result.pos[i][2]));
    }
    printNodeMask("警告: 左右(y符号)が未確定（d(2,k)が無いため+y側にしてあります）",
                  result.sign_unresolved, n);
    printPairMask("警告: 高さ差とほぼ同じ距離を0扱いに丸めました", result.clamped_pairs, n);
    printNodeMask("警告: 三角形の一辺をほぼ0として丸めました", result.clamped_nodes, n);
}

int cmdSurvey(int argc, char** argv)
{
    if (argc < 2) {
        printUsageSurvey();
        return 1;
    }
    const char* sub       = argv[1];
    const int nArgs       = argc - 2;
    const char* const* a  = argv + 2;

    if (std::strcmp(sub, "dist") == 0) {
        if (nArgs != 3) {
            std::printf("引数の数が違います。例: survey dist 0 2 4.83\n");
            return 1;
        }
        size_t idxI = 0, idxJ = 0;
        float  d    = 0.0f;
        const size_t limit = surveyIndexLimit();
        if (!parseIndex(a[0], limit, &idxI) || !parseIndex(a[1], limit, &idxJ)) {
            std::printf("添字が範囲外です（0〜%u）: %s, %s\n", static_cast<unsigned>(limit - 1), a[0], a[1]);
            return 1;
        }
        if (idxI == idxJ) {
            std::printf("同じ添字は指定できません: %u\n", static_cast<unsigned>(idxI));
            return 1;
        }
        if (!parseReal(a[2], uwb::cfg::kMaxCoordM, &d) || d <= 0.0f) {
            std::printf("距離が不正です（正の値にしてください）: %s\n", a[2]);
            return 1;
        }
        g_surveyInput.dist[idxI][idxJ] = static_cast<uwb_real>(d);
        g_surveyInput.dist[idxJ][idxI] = static_cast<uwb_real>(d);
        g_surveyInput.have[idxI][idxJ] = 1;
        g_surveyInput.have[idxJ][idxI] = 1;
        std::printf("d(%u,%u) = %.3f m を記録しました（RAMのみ。save では保存されません）\n",
                    static_cast<unsigned>(idxI), static_cast<unsigned>(idxJ), static_cast<double>(d));
        return 0;
    }

    if (std::strcmp(sub, "z") == 0) {
        if (nArgs != 2) {
            std::printf("引数の数が違います。例: survey z 1 2.4\n");
            return 1;
        }
        size_t idx = 0;
        float  z   = 0.0f;
        const size_t limit = surveyIndexLimit();
        if (!parseIndex(a[0], limit, &idx)) {
            std::printf("添字が範囲外です（0〜%u）: %s\n", static_cast<unsigned>(limit - 1), a[0]);
            return 1;
        }
        if (!parseReal(a[1], uwb::cfg::kMaxZFixedM, &z)) {
            std::printf("値が不正です（範囲 ±%.0fm）: %s\n", static_cast<double>(uwb::cfg::kMaxZFixedM), a[1]);
            return 1;
        }
        g_surveyInput.z[idx] = static_cast<uwb_real>(z);
        std::printf("z[%u] = %.3f m を記録しました（RAMのみ）\n", static_cast<unsigned>(idx),
                    static_cast<double>(z));
        return 0;
    }

    if (std::strcmp(sub, "clear") == 0) {
        uwb_survey_tape_input_init(&g_surveyInput, 0);
        std::printf("survey の入力を全消去しました\n");
        return 0;
    }

    if (std::strcmp(sub, "show") == 0) {
        if (g_table == nullptr || g_service == nullptr) {
            std::printf("内部エラー: コンソールが初期化されていません\n");
            return 1;
        }
        g_service->lockTable();
        const size_t count = g_table->size();
        g_service->unlockTable();
        const int n = static_cast<int>(count);

        std::printf("登録テーブルの台数: %u（survey はこの台数ぶんを対象にします）\n",
                    static_cast<unsigned>(count));
        if (count == 0) {
            std::printf("アンカーが登録されていません。先に anchor set で登録してください\n");
            return 0;
        }

        std::printf("高さ z[i]（既定0、単位m）:\n");
        for (int i = 0; i < n; ++i) {
            std::printf("  z[%d] = %.3f m\n", i, static_cast<double>(g_surveyInput.z[i]));
        }

        std::printf("距離（メジャー実測、必須）:\n");
        if (n >= 2) {
            printDistEntry(0, 1);
        }
        for (int k = 2; k < n; ++k) {
            printDistEntry(0, k);
            printDistEntry(1, k);
        }
        if (n > 3) {
            std::printf("距離（任意、左右(y符号)の判定用。無ければ+y側に決めて計算は続けます）:\n");
            for (int k = 3; k < n; ++k) {
                printDistEntry(2, k);
            }
        }

        if (n < UWB_SURVEY_TAPE_MIN_NODES) {
            std::printf("計算にはあと%d台ぶんの登録が必要です\n", UWB_SURVEY_TAPE_MIN_NODES - n);
            return 0;
        }

        uwb_survey_tape_result result;
        if (solveSurveyPreview(n, &result)) {
            std::printf("座標プレビュー（survey apply で登録テーブルへ書き込むまで反映されません）:\n");
            printSurveyResult(result, n);
        } else {
            std::printf("まだ計算できません: ");
            printSurveyError(result, n);
        }
        return 0;
    }

    if (std::strcmp(sub, "apply") == 0) {
        if (g_table == nullptr || g_service == nullptr) {
            std::printf("内部エラー: コンソールが初期化されていません\n");
            return 1;
        }
        g_service->lockTable();
        const size_t count = g_table->size();
        g_service->unlockTable();
        const int n = static_cast<int>(count);

        if (n < UWB_SURVEY_TAPE_MIN_NODES) {
            std::printf("アンカー登録テーブルが%u台です（%d台以上必要）。"
                        "先に anchor set / anchor count で登録してください\n",
                        static_cast<unsigned>(count), UWB_SURVEY_TAPE_MIN_NODES);
            return 1;
        }

        uwb_survey_tape_result result;
        if (!solveSurveyPreview(n, &result)) {
            std::printf("計算できません: ");
            printSurveyError(result, n);
            return 1;
        }

        g_service->lockTable();
        for (int i = 0; i < n; ++i) {
            uwb::AnchorEntry e = g_table->entry(static_cast<size_t>(i));
            e.pos[0]             = static_cast<float>(result.pos[i][0]);
            e.pos[1]             = static_cast<float>(result.pos[i][1]);
            e.pos[2]             = static_cast<float>(result.pos[i][2]);
            // enabled はここでは触らない: survey は座標だけを決める機能で、
            // 有効/無効の管理は anchor enable/disable に委ねる（anchor set
            // と違い、暗黙に enabled=true へ変えたりしない）。
            g_table->update(static_cast<size_t>(i), e);
        }
        if (g_info.applyModePolicy != nullptr) {
            g_info.applyModePolicy(*g_table);
        }
        g_service->unlockTable();

        afterConfigEdit();

        std::printf("%d 台の座標を計算してアンカー登録テーブルへ書き込みました:\n", n);
        printSurveyResult(result, n);
        std::printf("次の測位周期から反映されます。save で NVS へ永続化してください\n");
        return 0;
    }

    std::printf("不明なサブコマンド: %s\n", sub);
    printUsageSurvey();
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
    uwb::ModeOverride override;
    float zFixedM = 0.0f;
    uwb::EkfTuning ekfTuning;
    g_service->lockTable();
    count      = g_table->size();
    for (size_t i = 0; i < count; ++i) {
        snapshot[i] = g_table->entry(i);
    }
    override    = g_table->modeOverride();
    zFixedM      = g_table->zFixedM();
    ekfTuning    = g_table->ekfTuning();
    g_service->unlockTable();

    if (count == 0) {
        std::printf("テーブルが空です。anchor set / anchor count で設定してください\n");
        return 1;
    }

    const esp_err_t tableErr = uwb::ConfigStore::saveAnchorTable(snapshot, count);
    if (tableErr != ESP_OK) {
        std::printf("アンカー登録テーブルの保存に失敗しました (err=%s)\n", esp_err_to_name(tableErr));
        return 1;
    }

    // 測位モード設定（手動オーバーライド + 2D固定高さ）も同じ save コマンドで
    // 永続化する。テーブルは既に保存できているので、こちらが失敗しても
    // markSaved() は呼ばず「一部だけ保存できた」状態を素直に報告する。
    const esp_err_t modeErr = uwb::ConfigStore::savePositioningMode(override, zFixedM);
    if (modeErr != ESP_OK) {
        std::printf("アンカー登録テーブルは保存しましたが、測位モード設定の保存に失敗しました (err=%s)\n",
                    esp_err_to_name(modeErr));
        return 1;
    }

    // EKF(拡張カルマンフィルタ)チューニングも同じ save コマンドで永続化する。
    // 上2つは既に保存できているので、こちらが失敗しても markSaved() は呼ばず
    // 「一部だけ保存できた」状態を素直に報告する。
    const esp_err_t ekfErr = uwb::ConfigStore::saveEkfTuning(ekfTuning);
    if (ekfErr != ESP_OK) {
        std::printf("アンカー登録テーブル・測位モード設定は保存しましたが、"
                    "EKFチューニングの保存に失敗しました (err=%s)\n",
                    esp_err_to_name(ekfErr));
        return 1;
    }

    markSaved();
    std::printf("NVS へ保存しました: アンカー %u 件, mode=%s, height=%.3fm, "
                "ekf=%s/sigma_a=%.3f/sigma0=%.3f/per_m=%.3f/gate=%.1f\n",
                static_cast<unsigned>(count), overrideName(override), static_cast<double>(zFixedM),
                (ekfTuning.model == 1) ? "ca" : "cv", static_cast<double>(ekfTuning.sigmaA),
                static_cast<double>(ekfTuning.sigmaR0), static_cast<double>(ekfTuning.sigmaRPerM),
                static_cast<double>(ekfTuning.gate));
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
        // set() 内の rebuildStorage() がこの時点の ekfTuning() を使って
        // 各アンカーの sigma0/sigma_per_m を埋めるため、set() より先に
        // EKFチューニングを既定値（EkfTuning{} = sigma_a 0.5, sigma0 0.10m,
        // per_m 0, gate 3.0, model CV）へ戻しておく。
        g_table->setEkfTuning(uwb::EkfTuning{});
        const bool ok = g_table->set(g_info.defaults, restored);
        // 測位モード設定もコンパイル時の既定（override=Auto、高さ=Kconfig
        // UWB_TAG_FIXED_Z_MM）へ戻す。anchor set() が失敗した場合はテーブルが
        // 変わっていないので、こちらも変更しないでおく。
        if (ok) {
            g_table->setModeOverride(uwb::ModeOverride::Auto);
            g_table->setZFixedM(g_info.defaultZFixedM);
        }
        if (ok && g_info.applyModePolicy != nullptr) {
            g_info.applyModePolicy(*g_table);
        }
        g_service->unlockTable();
        if (!ok) {
            restored = 0;
        } else {
            afterConfigEdit();
        }
    }
    std::printf("NVS を消去し、kAnchors[] の既定値 %u 件・測位モード auto (height=%.3fm)・"
                "ekf既定値 (cv/sigma_a=0.5/sigma0=0.10/per_m=0/gate=3.0) に戻しました"
                "（次の測位周期から反映）\n",
                static_cast<unsigned>(restored), static_cast<double>(g_info.defaultZFixedM));
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
    const uwb::ModeDecision modeDecision = g_table->modeDecision();
    const uwb::EkfTuning ekfTuning         = g_table->ekfTuning();
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
    std::printf("  mode         : %s（%s）  override=%s", uwb::positioningModeName(modeDecision.mode),
                uwb::modeReasonText(modeDecision.reason), overrideName(modeDecision.override));
    if (modeDecision.mode == uwb::PositioningMode::Mode2D) {
        std::printf("  z_fixed=%.3fm", static_cast<double>(modeDecision.zFixedM));
    }
    std::printf("\n");
    std::printf("  ekf          : model=%s sigma_a=%.3f sigma0=%.3fm per_m=%.3f gate=%.1f"
                "（詳しくは ekf コマンド）\n",
                (ekfTuning.model == 1) ? "ca" : "cv", static_cast<double>(ekfTuning.sigmaA),
                static_cast<double>(ekfTuning.sigmaR0), static_cast<double>(ekfTuning.sigmaRPerM),
                static_cast<double>(ekfTuning.gate));
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

/* ------------------------------------------------------------ bootlog コマンド */

/* main.cpp の起動パンくず (bootBreadcrumb()) が NVS namespace "dbg" へ書く
 * レイアウトを、ここで独立して読む。main.cpp 側の BreadcrumbEntry と
 * キー名・構造体・容量を完全に一致させること（main.cpp 冒頭コメント参照。
 * 両ファイルは #include を共有していないため、layout の変更は手動で
 * 揃える必要がある）。 */
constexpr size_t kBootlogHistCapacity = 16;

struct BootlogHistEntry {
    uint32_t bootN;
    uint8_t stage;
} __attribute__((packed));

int cmdBootlog(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    nvs_handle_t h;
    if (nvs_open("dbg", NVS_READONLY, &h) != ESP_OK) {
        std::printf("起動パンくずの記録がありません（NVS namespace \"dbg\" を開けませんでした）\n");
        return 0;
    }

    uint32_t bootN = 0;
    uint8_t stage  = 0xFF;
    nvs_get_u32(h, "boot_n", &bootN);
    nvs_get_u8(h, "stage", &stage);

    uint32_t histN = 0;
    nvs_get_u32(h, "hist_n", &histN);

    BootlogHistEntry hist[kBootlogHistCapacity];
    std::memset(hist, 0, sizeof(hist));
    size_t histBytes = sizeof(hist);
    // main.cpp 側と同じ方針: 読み出し失敗（キー無し・サイズ不一致等）は
    // 空リングとして扱う。
    const bool haveHist = (nvs_get_blob(h, "hist", hist, &histBytes) == ESP_OK) && (histBytes == sizeof(hist));
    nvs_close(h);

    std::printf("=== 起動パンくず (bootlog) ===\n");
    std::printf("  今回      : boot_n=%lu stage=%u\n", static_cast<unsigned long>(bootN),
                static_cast<unsigned>(stage));
    std::printf("  凡例      : 0=entry 1=console 2=net_start 3=net_ok 4=got_ip\n");

    const size_t n =
        !haveHist ? 0 : ((histN > kBootlogHistCapacity) ? kBootlogHistCapacity : static_cast<size_t>(histN));
    std::printf("  履歴（新しい順、%u/%u件）:\n", static_cast<unsigned>(n),
                static_cast<unsigned>(kBootlogHistCapacity));
    for (size_t i = 0; i < n; ++i) {
        const size_t idx = static_cast<size_t>((histN - 1 - i) % kBootlogHistCapacity);
        std::printf("    boot_n=%lu stage=%u\n", static_cast<unsigned long>(hist[idx].bootN),
                    static_cast<unsigned>(hist[idx].stage));
    }
    if (n == 0) {
        std::printf("    (記録なし)\n");
    }
    return 0;
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

    const esp_console_cmd_t modeCmd = {
        .command  = "mode",
        .help     = "測位モードの表示・手動切替（有効アンカー台数・配置から自動決定。auto/2d/3dで強制も可）",
        .hint     = " [auto|2d|3d]",
        .func     = &cmdMode,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&modeCmd));

    const esp_console_cmd_t heightCmd = {
        .command  = "height",
        .help     = "2D測位の固定高さ(z_fixed)の表示・変更 [m]（範囲 ±10m）",
        .hint     = " [<meters>]",
        .func     = &cmdHeight,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&heightCmd));

    const esp_console_cmd_t ekfCmd = {
        .command  = "ekf",
        .help     = "EKF(拡張カルマンフィルタ)のQ(プロセス雑音)/R(観測雑音)/イノベーションゲートの表示・変更"
                    "（ekf q/r/gate/model）",
        .hint     = " [q <sigma_a> | r <sigma0> [per_m] | gate <k> | model cv|ca]",
        .func     = &cmdEkf,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&ekfCmd));

    const esp_console_cmd_t surveyCmd = {
        .command  = "survey",
        .help     = "メジャー実測値からのアンカー座標計算（survey dist/z/show/apply/clear）",
        .hint     = " dist <i> <j> <m> | z <i> <m> | show | apply | clear",
        .func     = &cmdSurvey,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&surveyCmd));

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

    const esp_console_cmd_t bootlogCmd = {
        .command  = "bootlog",
        .help     = "起動パンくず（起動回数・前回どの段階まで進んだか・直近16件の履歴）を表示する",
        .hint     = nullptr,
        .func     = &cmdBootlog,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    ESP_ERROR_CHECK(esp_console_cmd_register(&bootlogCmd));
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
    // read フックの差し替えは、必ず esp_console_start_repl() より【前】に行う。
    // ホスト不在時、start_repl が REPL タスク (優先度2) を起こした瞬間に
    // main タスク (優先度1、コア0固定) はコアを奪われ、素の read のままの
    // REPL が busy loop 化して main は二度と走れない (実機で boot_n=9 が
    // stage=1 で凍結、2026-08-31 充電器試験)。read 関数は new_repl 内の
    // linenoiseProbe() で確定済みなので、ここで差し替えれば上書きされない。
    // Install the read hook BEFORE esp_console_start_repl(): with no USB host,
    // the priority-2 REPL task preempts the core-0-pinned priority-1 main task
    // the moment it is started, and with the raw read() it spins forever, so
    // main would never reach an install placed after start_repl (observed as a
    // stage=1 freeze on charger power). linenoiseProbe() inside new_repl has
    // already finalized the read function, so nothing overwrites this hook.
    uwb_port_console_read_guard_install();
    return esp_console_start_repl(repl);
}

} // namespace tagapp
