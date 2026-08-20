/**
 * @file main.cpp
 * @brief Phase 4 Step 2 正式版タグファーム。
 *
 * アンカー登録テーブル（コンパイル時定数）を順にポーリングして測距し、
 * uwb::PositioningPipeline で位置を解いて、結果を JSON Lines で
 * シリアル(stdout)へ1行1エポックで出力する。
 *
 * JSON Lines の形式は third_party/uwb_localizer/docs/UWB_PROTOCOL.md の
 * 「方法B」（JsonLinesHal）に寄せてある:
 *   - 起動時に1回、"type":"anchors" 行を出す（Anchor.from_dict がそのまま読める形）
 *   - 毎エポック、"type":"meas" 行を出す（Measurement.from_dict がそのまま読める形。
 *     欠測したアンカーは単に含めない＝JsonLinesHal 側は「その周は測距が少なかった」
 *     として扱える）
 *   - 毎エポック、"type":"fix" 行も出す。これは本プロジェクト独自の診断情報
 *     （位置・ok/ambiguous・gdop・residual_rms・excludedビットマスク・
 *     1周の所要時間・各ソルバの計算時間等）で、JsonLinesHal の
 *     parse_line() は "anchors"/"meas" 以外の type を "other" として
 *     黙って読み捨てる仕様なので、既存のPython可視化を壊さずに追加情報を
 *     載せられる（third_party/uwb_localizer/uwb_loc/hal/jsonl.py 参照）。
 *
 * JSON行は printf 系ではなく必ず素の標準出力へ書く（ESP_LOGx はタイムスタンプ
 * 等のプレフィックスが付き先頭が '{' にならないため、JsonLinesHal 側で
 * 「JSONでない行」として読み捨てられるだけで実害は無いが、ここでは診断用の
 * 通常ログ(ESP_LOGx)とデータ行(printf)を意図的に分けている）。
 */
#include <cmath>
#include <cstdarg>
#include <cstdio>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_port.h"
#include "uwb_qm33120.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_pipeline.hpp"
#include "uwb_ranging_scheduler.hpp"
#include "uwb_ranging_types.hpp"

#if CONFIG_UWB_TAG_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "Stamp S3"
#endif

#if CONFIG_UWB_TAG_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_tag";

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- ネットワーク共通パラメータ（firmware/anchor と揃えること） --- */
static constexpr uint16_t PAN_ID          = 0xDECA;
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;

/**
 * アンカー登録テーブル（コンパイル時定数。NVS化は将来課題）。
 *
 * ============================================================
 * 暫定値。実配置の実測座標に置き換えること（boards 配下のヘッダと同様、
 * ここに書かれている座標は未確定のプレースホルダ）。
 * ============================================================
 *
 * short_addr は firmware/anchor の Kconfig (UWB_ANCHOR_SHORT_ADDR) で
 * 各アンカー実機に書き込む値と一致させる。docs/ANCHOR_PLACEMENT.md の
 * 配置ルールに従い、非同一平面（高さを最低1台は変える）にしてあり、かつ
 * アンカー平面がワールド原点(z=0)を通らないよう最低高さを 0.2m にしてある。
 */
static const uwb::AnchorEntry kAnchors[] = {
    {0x0002, {0.0f, 0.0f, 2.4f}, 0.0f, true},
    {0x0003, {5.0f, 0.0f, 0.2f}, 0.0f, true},
    {0x0004, {5.0f, 5.0f, 2.4f}, 0.0f, true},
    {0x0005, {0.0f, 5.0f, 0.2f}, 0.0f, true},
    {0x0006, {2.5f, 2.5f, 2.4f}, 0.0f, true},
};
static constexpr size_t kNumAnchors = sizeof(kAnchors) / sizeof(kAnchors[0]);

/**
 * @brief boards 配下のヘッダの BOARD_UWB_PORT_CONFIG を uwb::Config へコピーする。
 * firmware/twr, firmware/anchor と同一の変換。
 */
static uwb::Config makeConfigFromBoard()
{
    const uwb_port_config_t port = BOARD_UWB_PORT_CONFIG;

    uwb::Config cfg;
    cfg.spi_host     = port.spi_host;
    cfg.pin_sck      = port.pin_sck;
    cfg.pin_mosi     = port.pin_mosi;
    cfg.pin_miso     = port.pin_miso;
    cfg.pin_cs       = port.pin_cs;
    cfg.pin_rst      = port.pin_rst;
    cfg.pin_irq      = port.pin_irq;
    cfg.pin_wakeup   = port.pin_wakeup;
    cfg.pin_gp7      = port.pin_gp7;
    cfg.spi_slow_hz  = port.spi_slow_hz;
    cfg.spi_fast_hz  = port.spi_fast_hz;
    cfg.init_spi_bus = port.init_spi_bus;
    return cfg;
}

/* ==================================================================== *
 * JSON Lines 出力
 * ==================================================================== */

#define JSON_BUF_SIZE 2048

/**
 * @brief バッファ範囲を超えない vsnprintf ラッパ。*off を書き込み後の位置に進める。
 * 途中で切り詰められても（バッファが足りなくなっても）以降の呼び出しは
 * 安全に無視される（off が cap を超えないようにクランプする）。
 */
static void jsonAppend(char* buf, size_t cap, size_t* off, const char* fmt, ...)
    __attribute__((format(printf, 4, 5)));

static void jsonAppend(char* buf, size_t cap, size_t* off, const char* fmt, ...)
{
    if (*off >= cap) {
        return;
    }
    va_list args;
    va_start(args, fmt);
    const int n = vsnprintf(buf + *off, cap - *off, fmt, args);
    va_end(args);
    if (n > 0) {
        const size_t written   = static_cast<size_t>(n);
        const size_t remaining = cap - *off;
        *off += (written < remaining) ? written : (remaining - 1);
    }
}

/** アンカーの短縮ID文字列（"A0002"のような形）を作る。JSON側の"a"/"id"に使う。 */
static void formatAnchorId(uint16_t shortAddr, char* out, size_t outSize)
{
    std::snprintf(out, outSize, "A%04X", static_cast<unsigned>(shortAddr));
}

/** 起動時に1回だけ出す "type":"anchors" 行。 */
static void printAnchorsLine(const uwb::AnchorTable& table)
{
    char buf[JSON_BUF_SIZE];
    size_t off = 0;
    jsonAppend(buf, sizeof(buf), &off, "{\"v\":1,\"type\":\"anchors\",\"anchors\":[");
    for (size_t i = 0; i < table.size(); ++i) {
        const uwb::AnchorEntry& e = table.entry(i);
        char id[8];
        formatAnchorId(e.short_addr, id, sizeof(id));
        jsonAppend(buf, sizeof(buf), &off,
                   "%s{\"id\":\"%s\",\"p\":[%.4f,%.4f,%.4f],\"antenna_delay_m\":%.4f,\"enabled\":%s}",
                   (i == 0) ? "" : ",", id, static_cast<double>(e.pos[0]), static_cast<double>(e.pos[1]),
                   static_cast<double>(e.pos[2]), static_cast<double>(e.antenna_delay_m),
                   e.enabled ? "true" : "false");
    }
    jsonAppend(buf, sizeof(buf), &off, "]}\n");
    std::fputs(buf, stdout);
}

/** 毎エポック出す "type":"meas" 行（JsonLinesHal がそのまま読める標準形）。
 *  欠測したアンカーは含めない。 */
static void printMeasLine(double t, const uwb::AnchorTable& table, const uwb::RangingSample* samples,
                           size_t n)
{
    char buf[JSON_BUF_SIZE];
    size_t off = 0;
    jsonAppend(buf, sizeof(buf), &off, "{\"v\":1,\"type\":\"meas\",\"t\":%.3f,\"tag\":\"tag0\",\"meas\":[",
               t);
    bool first = true;
    for (size_t i = 0; i < n; ++i) {
        if (!samples[i].ok || samples[i].anchor_index >= table.size()) {
            continue;
        }
        char id[8];
        formatAnchorId(table.entry(samples[i].anchor_index).short_addr, id, sizeof(id));
        jsonAppend(buf, sizeof(buf), &off, "%s{\"a\":\"%s\",\"d\":%.4f}", first ? "" : ",", id,
                   static_cast<double>(samples[i].distance_m));
        first = false;
    }
    jsonAppend(buf, sizeof(buf), &off, "]}\n");
    std::fputs(buf, stdout);
}

/** PositionResult の中身を（波括弧なしで）キー:値の並びとして書く。
 *  トップレベルへ直接展開する場合と、"lv0":{...} のように入れ子にする場合の
 *  両方から呼ぶ共通部分。 */
static void appendResultBody(char* buf, size_t cap, size_t* off, const uwb::PositionResult& r)
{
    jsonAppend(buf, cap, off,
               "\"ok\":%s,\"ambiguous\":%s,\"solvable\":%s,"
               "\"p\":[%.4f,%.4f,%.4f],\"gdop\":%.4f,\"residual_rms\":%.4f,\"sigma\":%.4f,"
               "\"n_used\":%d,\"n_total\":%d,\"excluded\":\"0x%08lX\"",
               r.ok ? "true" : "false", r.ambiguous ? "true" : "false", r.solvable ? "true" : "false",
               static_cast<double>(r.p[0]), static_cast<double>(r.p[1]), static_cast<double>(r.p[2]),
               static_cast<double>(r.gdop), static_cast<double>(r.residualRms),
               static_cast<double>(r.sigma), r.nUsed, r.nTotal, r.excluded);
}

/** ソルバの計算時間 [us]（esp_timer_get_time() の差分）。 */
struct EpochTiming {
    int64_t solveLv0Us = 0;
    int64_t solveLv2Us = 0;
    int64_t solveLv3Us = 0; // haveLv3==false のときは未使用
};

/** 毎エポック出す "type":"fix" 行（本プロジェクト独自の診断情報。
 *  JsonLinesHal 側は type を認識せず読み捨てるだけなので安全に共存できる）。
 *  トップレベルの ok/ambiguous/p/... は Lv2（本番）の結果を表す。 */
static void printFixLine(double t, uint32_t cycleMs, const uwb::AnchorTable& table,
                          const uwb::RangingSample* samples, size_t n, const uwb::PositionResult& lv0,
                          const uwb::PositionResult& lv2, bool haveLv3, const uwb::PositionResult& lv3,
                          const EpochTiming& timing)
{
    char buf[JSON_BUF_SIZE];
    size_t off = 0;

    jsonAppend(buf, sizeof(buf), &off,
               "{\"v\":1,\"type\":\"fix\",\"t\":%.3f,\"tag\":\"tag0\",\"cycle_ms\":%lu,"
               "\"primary_level\":\"Lv2\",",
               t, static_cast<unsigned long>(cycleMs));

    appendResultBody(buf, sizeof(buf), &off, lv2);

    jsonAppend(buf, sizeof(buf), &off, ",\"solve_us_lv0\":%lld,\"solve_us_lv2\":%lld",
               static_cast<long long>(timing.solveLv0Us), static_cast<long long>(timing.solveLv2Us));
    if (haveLv3) {
        jsonAppend(buf, sizeof(buf), &off, ",\"solve_us_lv3\":%lld",
                   static_cast<long long>(timing.solveLv3Us));
    }

    jsonAppend(buf, sizeof(buf), &off, ",\"lv0\":{");
    appendResultBody(buf, sizeof(buf), &off, lv0);
    jsonAppend(buf, sizeof(buf), &off, "}");

    if (haveLv3) {
        jsonAppend(buf, sizeof(buf), &off, ",\"lv3\":{");
        appendResultBody(buf, sizeof(buf), &off, lv3);
        jsonAppend(buf, sizeof(buf), &off, "}");
    }

    jsonAppend(buf, sizeof(buf), &off, ",\"anchors\":[");
    for (size_t i = 0; i < n; ++i) {
        if (samples[i].anchor_index >= table.size()) {
            continue;
        }
        char id[8];
        formatAnchorId(table.entry(samples[i].anchor_index).short_addr, id, sizeof(id));
        if (samples[i].ok) {
            jsonAppend(buf, sizeof(buf), &off, "%s{\"a\":\"%s\",\"ok\":true,\"d\":%.4f,\"elapsed_ms\":%lu}",
                       (i == 0) ? "" : ",", id, static_cast<double>(samples[i].distance_m),
                       static_cast<unsigned long>(samples[i].elapsed_ms));
        } else {
            jsonAppend(buf, sizeof(buf), &off, "%s{\"a\":\"%s\",\"ok\":false}", (i == 0) ? "" : ",", id);
        }
    }
    jsonAppend(buf, sizeof(buf), &off, "]}\n");
    std::fputs(buf, stdout);
}

/* ==================================================================== *
 * app_main
 * ==================================================================== */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Phase 4 Step 2 uwb_tag firmware, board=%s method=%s anchors=%zu", BOARD_NAME,
             METHOD_NAME, kNumAnchors);

    uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    const uwb::PhyConfig phy; // defaults: ch9, preamble128, PAC8, 6.8Mbps

    if (!uwbDevice.begin(cfg, phy)) {
        ESP_LOGE(TAG, "begin() failed: error=%s", uwbDevice.lastErrorName());
        return;
    }
    ESP_LOGI(TAG, "deviceId=0x%08lX (expect 0x%08lX) chipName=%s isConnected=%d isInitialized=%d",
             (unsigned long)uwbDevice.deviceId(), (unsigned long)UWB_DEV_ID_EXPECTED, uwbDevice.chipName(),
             uwbDevice.isConnected(), uwbDevice.isInitialized());
    if (uwbDevice.deviceId() != UWB_DEV_ID_EXPECTED || !uwbDevice.isInitialized()) {
        ESP_LOGE(TAG, "UWB device not ready, aborting");
        return;
    }

    /* --- アンカー登録テーブルの構築 + 起動時チェック（必須） --- */
    uwb::AnchorTable table;
    if (!table.set(kAnchors, kNumAnchors)) {
        ESP_LOGE(TAG, "AnchorTable::set() failed (kNumAnchors=%zu, kMaxAnchors=%zu)", kNumAnchors,
                 uwb::kMaxAnchors);
        return;
    }

    const uwb::PlacementCheck placement = table.checkPlacement();
    if (placement.coplanar) {
        ESP_LOGW(TAG,
                 "登録済みアンカーが同一平面上にあります。3D測位のambiguousフラグを"
                 "必ず確認してください（normal=(%.3f,%.3f,%.3f) offset=%.3f）",
                 static_cast<double>(placement.normal[0]), static_cast<double>(placement.normal[1]),
                 static_cast<double>(placement.normal[2]), static_cast<double>(placement.offsetM));
        if (placement.originWarning) {
            ESP_LOGW(TAG, "アンカー平面が原点を通っています。ワールド原点をずらしてください"
                          "（この配置ではLv2が毎回ok=0になることが実測で判明しています）");
#if CONFIG_UWB_TAG_AUTO_2D_FALLBACK
            const float zFixedM = static_cast<float>(CONFIG_UWB_TAG_FIXED_Z_MM) / 1000.0f;
            table.setDimension2D(zFixedM);
            ESP_LOGW(TAG, "dim=2 (z_fixed=%.3fm) へフォールバックしました", static_cast<double>(zFixedM));
#endif
        }
    }

    uwb::PositioningConfig posCfg;
    posCfg.defaultLevel = uwb::SolverLevel::Lv2;
    uwb::PositioningPipeline pipeline(table, posCfg);
#if CONFIG_UWB_TAG_ENABLE_EKF
    pipeline.initEkf();
#endif

    uwb::SchedulerConfig schedCfg;
#if CONFIG_UWB_TAG_METHOD_DS
    schedCfg.method = uwb::RangingMethod::DS;
#else
    schedCfg.method = uwb::RangingMethod::SS;
#endif
    schedCfg.panId                = PAN_ID;
    schedCfg.tagShortAddr         = TAG_SHORT_ADDR;
    schedCfg.perAnchorIntervalMs = CONFIG_UWB_TAG_PER_ANCHOR_INTERVAL_MS;
    schedCfg.cycleIntervalMs      = CONFIG_UWB_TAG_CYCLE_INTERVAL_MS;
    uwb::RangingScheduler scheduler(uwbDevice, table, schedCfg);

    ESP_LOGI(TAG, "begin() + PHY config OK, starting ranging loop");

    /* JSON Lines: アンカー一覧を最初に1回出す */
    printAnchorsLine(table);

    const int64_t bootUs = esp_timer_get_time();
    uwb::RangingSample samples[uwb::kMaxAnchors];
    uint32_t failLogCounter = 0;

    while (true) {
        const size_t n = scheduler.runCycle(samples, uwb::kMaxAnchors);
        const double t = static_cast<double>(esp_timer_get_time() - bootUs) / 1e6;

        EpochTiming timing;
        const int64_t t0 = esp_timer_get_time();
        const uwb::PositionResult lv0 = pipeline.solve(samples, n, uwb::SolverLevel::Lv0);
        const int64_t t1 = esp_timer_get_time();
        timing.solveLv0Us              = t1 - t0;

        const uwb::PositionResult lv2 = pipeline.solve(samples, n, uwb::SolverLevel::Lv2);
        const int64_t t2 = esp_timer_get_time();
        timing.solveLv2Us              = t2 - t1;

        uwb::PositionResult lv3;
        bool haveLv3 = false;
#if CONFIG_UWB_TAG_ENABLE_EKF
        lv3 = pipeline.updateEkf(t, samples, n);
        const int64_t t3 = esp_timer_get_time();
        timing.solveLv3Us = t3 - t2;
        haveLv3             = true;
#endif

        printMeasLine(t, table, samples, n);
        printFixLine(t, scheduler.lastCycleMs(), table, samples, n, lv0, lv2, haveLv3, lv3, timing);

        if (!lv2.ok) {
            // 「測位不能」（有効測距不足）または解ソルバ失敗（同一平面配置の
            // 既知の縮退等）。JSON行そのものは毎回stdoutへ出るので、こちらは
            // 人間向けの要約ログを間引いて出すだけにする。
            if ((++failLogCounter % 20) == 1) {
                ESP_LOGW(TAG, "position unavailable: solvable=%d ok=%d n_total=%d anchors=%zu",
                         lv2.solvable, lv2.ok, lv2.nTotal, table.size());
            }
        }
    }
}
