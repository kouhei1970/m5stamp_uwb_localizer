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
 *
 * アンカー登録テーブルは **NVS に保存された値を優先し、無ければ下の
 * kAnchors[] を既定値として使う**（components/uwb_cfgstore）。NVSが空・
 * 未初期化・破損のいずれでも必ず kAnchors[] へフォールバックするので、
 * 購入者が何も設定しなければ従来どおり動く。実行時の変更は
 * USB-Serial/JTAG 上のシリアルコンソール（tag_console.cpp、
 * CONFIG_UWB_TAG_CONSOLE で無効化可）から anchor set / save で行う。
 * これにより「座標を数cm直すたびに再ビルド・再書き込み」が不要になる。
 * 測位ループとコンソールの同時実行の解き方は tag_console.hpp の
 * 冒頭コメントを参照。
 */
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "uwb_cfgstore.hpp"
#include "uwb_port.h"
#include "uwb_status_led.h"
#include "uwb_qm33120.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_pipeline.hpp"
#include "uwb_ranging_scheduler.hpp"
#include "uwb_ranging_types.hpp"

#if CONFIG_UWB_TAG_CONSOLE
#include "tag_console.hpp"
#endif

#if CONFIG_UWB_TAG_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_TAG_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPFLY_STATUS_LED_GPIO
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

#if CONFIG_UWB_TAG_METHOD_DS
#define METHOD_NAME "DS-TWR"
#else
#define METHOD_NAME "SS-TWR"
#endif

static const char* TAG = "uwb_tag";

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

/* --- タイミングプリセット(docs/TIMING_PRESETS.md、タスクD) ---
 * Kconfig の choice UWB_TIMING_PROFILE から uwb::TimingProfile を決める。
 * アンカー側(firmware/anchor)と必ず同じものを選ぶこと。 */
#if CONFIG_UWB_TIMING_PROFILE_ANCHOR_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::AnchorIrq;
#elif CONFIG_UWB_TIMING_PROFILE_BOTH_IRQ
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::BothIrq;
#else
static constexpr uwb::TimingProfile TIMING_PROFILE = uwb::TimingProfile::PollingBoth;
#endif

/* begin() 成功後に「実際に適用されたプリセット」で上書きする。
 * IRQ 線が死んでいると init() が PollingBoth へ降格するため、
 * コンパイル時の TIMING_PROFILE をそのまま使ってはいけない。
 * The profile init() actually applied - init() downgrades IRQ presets to
 * PollingBoth when the IRQ line turns out to be dead, so the compile-time
 * TIMING_PROFILE must not be used directly. */
static uwb::TimingProfile g_effectiveTimingProfile = TIMING_PROFILE;

/* --- ネットワーク共通パラメータ（firmware/anchor と揃えること） --- */
static constexpr uint16_t PAN_ID          = 0xDECA;
static constexpr uint16_t TAG_SHORT_ADDR = 0x0001;

/**
 * アンカー登録テーブルの **既定値**（NVSが空のときに使われる値）。
 *
 * ============================================================
 * 暫定値。**通常はここを書き換えず**、シリアルコンソールから
 *     anchor set 0 0x0002 0.0 0.0 2.4
 *     save
 * のように実測座標を入れること（docs/GETTING_STARTED.md §7）。
 * ここを直す方法は、コンソールを使わない場合の代替手段として残してある。
 * ============================================================
 *
 * 起動時は uwb::ConfigStore::loadAnchorTable() が NVS を読み、保存された
 * テーブルがあればそちらを採用する。NVSが空・未初期化・破損・値が異常の
 * いずれの場合もこの配列へフォールバックするので、何も設定しなくても
 * 従来どおり動く。
 *
 * short_addr は各アンカー実機のショートアドレス（firmware/anchor で
 * addr set したもの、または Kconfig UWB_ANCHOR_SHORT_ADDR の既定値）と
 * 一致させる。docs/ANCHOR_PLACEMENT.md の配置ルールに従い、非同一平面
 * （高さを最低1台は変える）にしてあり、かつアンカー平面がワールド原点
 * (z=0)を通らないよう最低高さを 0.2m にしてある。
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
    // CONFIG_UWB_ENABLE_IRQ が n のとき、ESP-IDF の Kconfig はこのマクロ自体を
    // 定義しない（0として定義されるわけではない）ので #ifdef ではなく
    // #if defined(...) && ... で判定する (docs/IRQ_POLICY.md, main/Kconfig.projbuild)。
    // pin_irq が UWB_PORT_PIN_UNUSED の場合や ISR 登録に失敗した場合は
    // Qm33120::init() が自動的にポーリングへフォールバックする。
#if defined(CONFIG_UWB_ENABLE_IRQ) && CONFIG_UWB_ENABLE_IRQ
    cfg.use_irq = true;
#else
    cfg.use_irq = false;
#endif
    // タイミングプリセット(docs/TIMING_PRESETS.md、タスクD)。相手(Anchor)へ
    // Poll フレームで伝わり、不一致検出に使われる
    // (uwb_qm33120_twr.cpp checkTimingTag()/logTimingMismatch())。
    cfg.timing_profile = TIMING_PROFILE;
    return cfg;
}

/* ==================================================================== *
 * JSON Lines 出力
 * ==================================================================== */

#define JSON_BUF_SIZE 2048

/**
 * @brief JSON 1行ぶんの組み立てバッファ。
 *
 * **測位ループのタスクからしか使わない**（コンソールのタスクは触らない）ので
 * static にしてよい。スタックではなく .bss に置くのは、app_main のタスク
 * スタック（CONFIG_ESP_MAIN_TASK_STACK_SIZE。ESP-IDF 既定は 3584 バイトだが、
 * `firmware/tag/sdkconfig.defaults` で 12288 バイトに引き上げ済み。理由は
 * 同ファイルのコメント参照 - EKF の `uwb_ekf_update()` → `uwb_ekf_predict()`
 * 連鎖が 9x9 double 行列を複数スタックに置き、配列だけで約 3.5KB、
 * Lv0/Lv2 経路も約 2KB 使うため、既定 3584 バイトでは溢れて panic リブート
 * する。docs/REVIEW_2026-08-21.md §0 #3 参照）に 2KB の配列を 2 つ積むと、
 * AnchorTable / PositioningPipeline（内部に uwb_ekf を持つ）と合わせて
 * 余裕が心もとなくなるため、引き続き .bss に逃がしている。コンソール追加とは
 * 独立した安全側の変更で、出力される JSON の内容は従来と同一。
 */
static char g_jsonBuf[JSON_BUF_SIZE];

/**
 * @brief JSON Lines を出力してよいか。
 *
 * コンソール有効時は output コマンドで一時的に止められる（毎周期2行のJSONが
 * 流れているとコンソールで設定作業ができないため）。無効時は常に true を返す
 * 定数関数になり、従来と全く同じ動作になる。
 */
static inline bool jsonOutputEnabled()
{
#if CONFIG_UWB_TAG_CONSOLE
    return tagapp::jsonOutputEnabled();
#else
    return true;
#endif
}

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

/**
 * @brief JSON 1行の末尾の閉じ（"]}\n"）を必ず書き込む。
 *
 * 【修正7】docs/archive/REVIEW_2026-08-21.md app層M-4: jsonAppend() は
 * バッファが逼迫すると超過分を無言で捨てて *off をそれ以上進めなくする
 * （*off は cap-1 付近で頭打ちになる）。これまでは printXxxLine() の
 * どれも最後に普通の jsonAppend(buf, JSON_BUF_SIZE, &off, "]}\n") を
 * 使っていたため、直前までの内容だけで容量を使い切っていると、この
 * 最後の閉じ自体も捨てられて改行の無いままバッファが終わる。
 * fputs() はその不完全な文字列をそのまま書き出すので、次に出力される
 * 行の先頭（"{"...）と連結してしまい、ホスト側の「1行=1JSON」前提の
 * パーサ（third_party/uwb_localizer/uwb_loc/hal/jsonl.py 等、readline()
 * で1行ずつ json.loads() する）がその周辺の行をまとめて壊す。
 *
 * この関数は「容量が足りなければ直前のバイトを巻き戻してでも」"]}\n"
 * を必ず書き切る。巻き戻しが発生した（＝この行のどこかで jsonAppend()
 * の切り詰めが既に起きていた）場合は、"]}\n" の直前にテキストの目印
 * "...TRUNCATED..." を残す。切り詰められた行は（配列要素の途中で
 * 切れている可能性があるため）そもそも有効なJSONである保証はできない
 * が、それは今回の要件ではない — 保証するのは「必ず改行で終わる」こと
 * だけであり、これにより巻き添えで壊れる行をこの1行だけに抑えられる。
 */
static void jsonAppendClose(char* buf, size_t cap, size_t* off)
{
    static const char kClose[]     = "]}\n";
    static const char kTruncated[] = "...TRUNCATED...]}\n";
    const size_t closeLen           = sizeof(kClose) - 1;
    const size_t truncatedLen       = sizeof(kTruncated) - 1;

    // *off が cap-1 付近まで来ているなら、手前で jsonAppend() の切り詰めが
    // 既に起きていたとみなす（通常の内容がたまたまこの数バイト差ちょうど
    // に収まる確率は無視できるほど低い）。
    const bool truncated = (*off + closeLen + 1) > cap;

    const char* closing;
    size_t len;
    if (!truncated) {
        closing = kClose;
        len      = closeLen;
    } else if (cap > truncatedLen) {
        // 目印込みでも収まる: 収まる位置まで手前のバイトを巻き戻す。
        *off    = cap - truncatedLen - 1;
        closing = kTruncated;
        len      = truncatedLen;
    } else if (cap > closeLen) {
        // 目印までは入らない: 最低限の閉じだけは必ず書く。
        *off    = cap - closeLen - 1;
        closing = kClose;
        len      = closeLen;
    } else {
        // cap が異常に小さい（現状の JSON_BUF_SIZE では起こらない）。
        // 書けるものが無いので、最低限バッファを空文字列にしておく。
        buf[0] = '\0';
        *off    = 0;
        return;
    }

    std::memcpy(buf + *off, closing, len);
    *off += len;
    buf[*off] = '\0';
}

/** アンカーの短縮ID文字列（"A0002"のような形）を作る。JSON側の"a"/"id"に使う。 */
static void formatAnchorId(uint16_t shortAddr, char* out, size_t outSize)
{
    std::snprintf(out, outSize, "A%04X", static_cast<unsigned>(shortAddr));
}

/** 起動時に1回だけ出す "type":"anchors" 行。 */
static void printAnchorsLine(const uwb::AnchorTable& table)
{
    if (!jsonOutputEnabled()) {
        return;
    }
    char* const buf = g_jsonBuf;
    size_t off      = 0;
    jsonAppend(buf, JSON_BUF_SIZE, &off, "{\"v\":1,\"type\":\"anchors\",\"anchors\":[");
    for (size_t i = 0; i < table.size(); ++i) {
        const uwb::AnchorEntry& e = table.entry(i);
        char id[8];
        formatAnchorId(e.short_addr, id, sizeof(id));
        jsonAppend(buf, JSON_BUF_SIZE, &off,
                   "%s{\"id\":\"%s\",\"p\":[%.4f,%.4f,%.4f],\"antenna_delay_m\":%.4f,\"enabled\":%s}",
                   (i == 0) ? "" : ",", id, static_cast<double>(e.pos[0]), static_cast<double>(e.pos[1]),
                   static_cast<double>(e.pos[2]), static_cast<double>(e.antenna_delay_m),
                   e.enabled ? "true" : "false");
    }
    jsonAppendClose(buf, JSON_BUF_SIZE, &off); // 【修正7】末尾の閉じは必ず書く（jsonAppendClose() 冒頭コメント参照）
    std::fputs(buf, stdout);
}

/** 毎エポック出す "type":"meas" 行（JsonLinesHal がそのまま読める標準形）。
 *  欠測したアンカーは含めない。 */
static void printMeasLine(double t, const uwb::AnchorTable& table, const uwb::RangingSample* samples,
                           size_t n)
{
    if (!jsonOutputEnabled()) {
        return;
    }
    char* const buf = g_jsonBuf;
    size_t off      = 0;
    jsonAppend(buf, JSON_BUF_SIZE, &off, "{\"v\":1,\"type\":\"meas\",\"t\":%.3f,\"tag\":\"tag0\",\"meas\":[",
               t);
    bool first = true;
    for (size_t i = 0; i < n; ++i) {
        if (!samples[i].ok || samples[i].anchor_index >= table.size()) {
            continue;
        }
        char id[8];
        formatAnchorId(table.entry(samples[i].anchor_index).short_addr, id, sizeof(id));
        jsonAppend(buf, JSON_BUF_SIZE, &off, "%s{\"a\":\"%s\",\"d\":%.4f}", first ? "" : ",", id,
                   static_cast<double>(samples[i].distance_m));
        first = false;
    }
    jsonAppendClose(buf, JSON_BUF_SIZE, &off); // 【修正7】末尾の閉じは必ず書く（jsonAppendClose() 冒頭コメント参照）
    std::fputs(buf, stdout);
}

/** PositionResult の中身を（波括弧なしで）キー:値の並びとして書く。
 *  トップレベルへ直接展開する場合と、"lv0":{...} のように入れ子にする場合の
 *  両方から呼ぶ共通部分。 */
static void appendResultBody(char* buf, size_t cap, size_t* off, const uwb::PositionResult& r)
{
    // 【修正4】"redundancy" (= n_used - (dim+1)) を追加。0 は「Lv2のロバスト
    // 外れ値棄却が自由度不足で原理的に効いていない解」を意味する
    // (docs/archive/REVIEW_2026-08-21.md app層M-1、
    // components/uwb_ranging/include/uwb_ranging_types.hpp の
    // PositionResult::redundancy 参照)。ホスト側で ok=true かつ
    // redundancy=0 の行を見分けられるよう、n_used/n_total と並べて出す。
    jsonAppend(buf, cap, off,
               "\"ok\":%s,\"ambiguous\":%s,\"solvable\":%s,"
               "\"p\":[%.4f,%.4f,%.4f],\"gdop\":%.4f,\"residual_rms\":%.4f,\"sigma\":%.4f,"
               "\"n_used\":%d,\"n_total\":%d,\"redundancy\":%d,\"excluded\":\"0x%08lX\"",
               r.ok ? "true" : "false", r.ambiguous ? "true" : "false", r.solvable ? "true" : "false",
               static_cast<double>(r.p[0]), static_cast<double>(r.p[1]), static_cast<double>(r.p[2]),
               static_cast<double>(r.gdop), static_cast<double>(r.residualRms),
               static_cast<double>(r.sigma), r.nUsed, r.nTotal, r.redundancy, r.excluded);
}

/** ソルバの計算時間 [us]（esp_timer_get_time() の差分）。 */
struct EpochTiming {
    int64_t solveLv0Us = 0;
    int64_t solveLv2Us = 0;
    int64_t solveLv3Us = 0; // haveLv3==false のときは未使用
};

/** 毎エポック出す "type":"fix" 行（本プロジェクト独自の診断情報。
 *  JsonLinesHal 側は type を認識せず読み捨てるだけなので安全に共存できる）。
 *  トップレベルの ok/ambiguous/p/... は Lv2（本番）の結果を表す。
 *  @param bootUs タスクF(docs/HANDOFF.md §5): anchors[].t を fix 行の
 *                 "t" と同じ基準（起動からの秒）に直すための引き算に使う。 */
static void printFixLine(double t, uint32_t cycleMs, const uwb::AnchorTable& table,
                          const uwb::RangingSample* samples, size_t n, const uwb::PositionResult& lv0,
                          const uwb::PositionResult& lv2, bool haveLv3, const uwb::PositionResult& lv3,
                          const EpochTiming& timing, int64_t bootUs)
{
    if (!jsonOutputEnabled()) {
        return;
    }
    char* const buf = g_jsonBuf;
    size_t off      = 0;

    jsonAppend(buf, JSON_BUF_SIZE, &off,
               "{\"v\":1,\"type\":\"fix\",\"t\":%.3f,\"tag\":\"tag0\",\"cycle_ms\":%lu,"
               "\"primary_level\":\"Lv2\",",
               t, static_cast<unsigned long>(cycleMs));

    appendResultBody(buf, JSON_BUF_SIZE, &off, lv2);

    jsonAppend(buf, JSON_BUF_SIZE, &off, ",\"solve_us_lv0\":%lld,\"solve_us_lv2\":%lld",
               static_cast<long long>(timing.solveLv0Us), static_cast<long long>(timing.solveLv2Us));
    if (haveLv3) {
        jsonAppend(buf, JSON_BUF_SIZE, &off, ",\"solve_us_lv3\":%lld",
                   static_cast<long long>(timing.solveLv3Us));
    }

    jsonAppend(buf, JSON_BUF_SIZE, &off, ",\"lv0\":{");
    appendResultBody(buf, JSON_BUF_SIZE, &off, lv0);
    jsonAppend(buf, JSON_BUF_SIZE, &off, "}");

    if (haveLv3) {
        jsonAppend(buf, JSON_BUF_SIZE, &off, ",\"lv3\":{");
        appendResultBody(buf, JSON_BUF_SIZE, &off, lv3);
        jsonAppend(buf, JSON_BUF_SIZE, &off, "}");
    }

    jsonAppend(buf, JSON_BUF_SIZE, &off, ",\"anchors\":[");
    for (size_t i = 0; i < n; ++i) {
        if (samples[i].anchor_index >= table.size()) {
            continue;
        }
        char id[8];
        formatAnchorId(table.entry(samples[i].anchor_index).short_addr, id, sizeof(id));
        // タスクF: t_us は rangeOne() が測距開始直前に必ず埋める（成功/失敗を
        // 問わない。uwb_ranging_scheduler.cpp 参照）ので、ok:false でも
        // 「いつ試みたか」は分かる。fix行トップレベルの "t" と比較できる
        // よう、生の t_us ではなく bootUs を引いた相対秒で出す。
        const double tAnchor = static_cast<double>(samples[i].t_us - bootUs) / 1e6;
        if (samples[i].ok) {
            jsonAppend(buf, JSON_BUF_SIZE, &off,
                       "%s{\"a\":\"%s\",\"ok\":true,\"d\":%.4f,\"elapsed_ms\":%lu,\"t\":%.4f}", (i == 0) ? "" : ",",
                       id, static_cast<double>(samples[i].distance_m), static_cast<unsigned long>(samples[i].elapsed_ms),
                       tAnchor);
        } else {
            jsonAppend(buf, JSON_BUF_SIZE, &off, "%s{\"a\":\"%s\",\"ok\":false,\"t\":%.4f}", (i == 0) ? "" : ",", id,
                       tAnchor);
        }
    }
    jsonAppendClose(buf, JSON_BUF_SIZE, &off); // 【修正7】末尾の閉じは必ず書く（jsonAppendClose() 冒頭コメント参照）
    std::fputs(buf, stdout);
}

/**
 * @brief 約1秒ごとに出す "type":"stats" 行（タスクE、docs/HANDOFF.md §5）。
 *
 * uwb::AnchorStats{attempts,successes,retries,rescued,successRate()}
 * （uwb_ranging_types.hpp）は実装済みだが接続先が無かったので、ここで
 * RangingScheduler::stats() を JSON Lines 出力へつなぐ。毎周期(約31Hz)出る
 * "fix" 行に足すとログが太るため、あえて別行・別間隔にする（決定事項。
 * 呼び出し側で CONFIG_UWB_TAG_STATS_INTERVAL_MS ごとに間引いて呼ぶこと）。
 * JsonLinesHal は未知の type を "other" として黙って読み捨てる仕様なので
 * （本ファイル冒頭コメント参照）、既存のPython可視化との互換性は保たれる。
 *
 * その周期に測ったものだけでなく、**登録されている全アンカー**
 * （無効化されているものも含む。stats_[i] は attempts=0 のまま）を出す。
 *
 * retries/rescued は再試行（CONFIG_UWB_TAG_RETRY_MAX/
 * CONFIG_UWB_TAG_RETRY_DELAY_MS、docs/HANDOFF.md §0-C「再試行の待ち時間」）
 * の内訳: retriesは無線試行としての再試行の総回数、rescuedは最初の試行が
 * 失敗し再試行のいずれかで成功したサイクル数（succの内数）。
 */
static void printStatsLine(double t, const uwb::AnchorTable& table, const uwb::RangingScheduler& scheduler,
                            uint32_t cycleMs)
{
    if (!jsonOutputEnabled()) {
        return;
    }
    char* const buf = g_jsonBuf;
    size_t off      = 0;
    jsonAppend(buf, JSON_BUF_SIZE, &off,
               "{\"v\":1,\"type\":\"stats\",\"t\":%.3f,\"tag\":\"tag0\",\"cycle_ms\":%lu,\"anchors\":[", t,
               static_cast<unsigned long>(cycleMs));
    for (size_t i = 0; i < table.size(); ++i) {
        const uwb::AnchorStats& s = scheduler.stats(i);
        char id[8];
        formatAnchorId(table.entry(i).short_addr, id, sizeof(id));
        jsonAppend(buf, JSON_BUF_SIZE, &off,
                   "%s{\"a\":\"%s\",\"att\":%lu,\"succ\":%lu,\"rate\":%.4f,\"retries\":%lu,\"rescued\":%lu}",
                   (i == 0) ? "" : ",", id, static_cast<unsigned long>(s.attempts),
                   static_cast<unsigned long>(s.successes), static_cast<double>(s.successRate()),
                   static_cast<unsigned long>(s.retries), static_cast<unsigned long>(s.rescued));
    }
    jsonAppendClose(buf, JSON_BUF_SIZE, &off); // 【修正7】末尾の閉じは必ず書く（jsonAppendClose() 冒頭コメント参照）
    std::fputs(buf, stdout);
}

/* ==================================================================== *
 * アンカー登録テーブルの適用
 * ==================================================================== */

/** 測位ループとコンソールの間で受け渡すテーブルの一時領域。
 *  測位ループのタスクからしか触らないので static でよい（app_main の
 *  スタックを節約する）。 */
static uwb::AnchorEntry g_workEntries[uwb::kMaxAnchors];

/**
 * @brief アンカー登録テーブルを差し替え、配置チェックと 2D フォールバック
 * 判定をやり直す。
 *
 * 起動時と、コンソールから設定が変わったときの両方から呼ぶ。**測距も測位も
 * 走っていない瞬間（1周期の先頭）にだけ呼ぶこと**（AnchorTable::set() は
 * uwb_config を作り直すため、ソルバから読まれている最中に呼んではいけない）。
 *
 * 前回 dim=2 へフォールバックしていた状態を引きずらないよう、判定前に
 * 必ず dim=3 へ戻してから checkPlacement() をやり直す。
 *
 * @return 差し替えに成功したら true。false のときテーブルは変更されない。
 */
static bool applyAnchorTable(uwb::AnchorTable& table, const uwb::AnchorEntry* entries, size_t count)
{
    if (!table.set(entries, count)) {
        ESP_LOGE(TAG, "AnchorTable::set() failed (count=%zu, kMaxAnchors=%zu)", count, uwb::kMaxAnchors);
        return false;
    }

    table.setDimension3D();

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
    return true;
}

/* ==================================================================== *
 * app_main
 * ==================================================================== */

extern "C" void app_main(void)
{
#ifdef BOARD_STATUS_LED_GPIO
/* Heartbeat colour tells the role apart once the boards are placed and no
 * serial monitor is attached: TAG = green, ANCHOR = red (components/
 * uwb_status_led). Only the M5StampS3A carries a WS2812 on a known GPIO -
 * the AtomS3 has an LCD instead - so the heartbeat compiles out there.
 * 設置後にシリアルを繋がない状態でも役割が分かるよう、ハートビートの色で
 * タグ(緑)とアンカー(赤)を見分ける。フルカラー LED を持つのは M5StampS3A
 * だけ(AtomS3 は LCD)なので、それ以外ではハートビートごと消える。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, UWB_STATUS_LED_ROLE_TAG);
#endif
    /* --- NVS からアンカー登録テーブルを読む（無ければ kAnchors[] 既定値） ---
     * ConfigStore::init() が失敗しても loadAnchorTable() は必ず既定値を返すので、
     * ここでは戻り値を見て中断しない。NVS破損で起動しなくなるのを避ける方針。 */
    uwb::ConfigStore::init();
    size_t entryCount = 0;
    const uwb::ConfigSource tableSource = uwb::ConfigStore::loadAnchorTable(
        kAnchors, kNumAnchors, g_workEntries, uwb::kMaxAnchors, &entryCount);

    ESP_LOGI(TAG, "Phase 4 Step 2 uwb_tag firmware, board=%s method=%s anchors=%zu (%s)", BOARD_NAME,
             METHOD_NAME, entryCount, uwb::configSourceName(tableSource));

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

    // init() may have downgraded the requested TIMING_PROFILE to PollingBoth
    // if the IRQ line turned out to be dead (Qm33120::verifyIrqLine()). Carry
    // the profile that was actually applied forward from here on.
    // IRQ 線が死んでいると init() が要求プリセットを PollingBoth へ降格させて
    // いることがあるため、以降は「実際に適用されたプリセット」を使う。
    g_effectiveTimingProfile = uwbDevice.config().timing_profile;

    /* --- アンカー登録テーブルの構築 + 起動時チェック（必須） --- */
    static uwb::AnchorTable table; // .bss へ置いて app_main のスタックを節約する
    if (!applyAnchorTable(table, g_workEntries, entryCount)) {
        return;
    }

    uwb::PositioningConfig posCfg;
    posCfg.defaultLevel = uwb::SolverLevel::Lv2;
    // pipeline は内部に uwb_ekf（1KB 程度）を持つので、こちらも .bss へ置く。
    static uwb::PositioningPipeline pipeline(table, posCfg);
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
    // 失敗したアンカーへの即時再試行（docs/HANDOFF.md §0-C「再試行の待ち時間」）。
    // DS-TWRはretryDelayMsに≥10msが必須（既定はKconfigのdefault 10 if
    // UWB_TAG_METHOD_DS）。根拠はKconfig.projbuildのUWB_TAG_RETRY_DELAY_MS
    // ヘルプとSchedulerConfig::retryDelayMsのフィールドコメント参照。
    // Retry a failed anchor immediately within the same cycle
    // (docs/HANDOFF.md §0-C "retry wait time"). DS-TWR needs
    // retryDelayMs >= 10ms (Kconfig defaults to 10 for UWB_TAG_METHOD_DS).
    // See Kconfig.projbuild's UWB_TAG_RETRY_DELAY_MS help and the
    // SchedulerConfig::retryDelayMs field comment for the rationale.
    schedCfg.retryMax      = CONFIG_UWB_TAG_RETRY_MAX;
    schedCfg.retryDelayMs = CONFIG_UWB_TAG_RETRY_DELAY_MS;
    // ssDefaults/dsDefaultsは構造体既定値(=PollingBothの値と
    // 完全一致。docs/TIMING_PRESETS.md §2)のまま個別上書きされていないので、
    // 実効プリセットがPollingBoth(明示選択時、またはIRQ線が死んでいて
    // init()が実行時に降格した場合)のときはここでのプリセット適用はno-op。
    // Kconfigの既定はBothIrqであり、その場合やIRQが実際に生きている場合は
    // ここで初めて値が反映される。
    uwb::applyTimingProfile(schedCfg.ssDefaults, g_effectiveTimingProfile);
    uwb::applyTimingProfile(schedCfg.dsDefaults, g_effectiveTimingProfile);
    static uwb::RangingScheduler scheduler(uwbDevice, table, schedCfg);

    ESP_LOGI(TAG, "begin() + PHY config OK, starting ranging loop");

    // --- IRQ（起床信号）の実際の有効状態をログに出す (docs/IRQ_POLICY.md) ---
    // Kconfigの設定値(CONFIG_UWB_ENABLE_IRQ)ではなく、Qm33120::irqActive()が
    // 返す「実際に有効化できたか」を出す。フォールバックが起きていないかを
    // 起動ログだけで判別できるようにするため。
    if (uwbDevice.irqActive()) {
        ESP_LOGI(TAG, "irq=active (pin=%d)", cfg.pin_irq);
    } else if (cfg.pin_irq == UWB_PORT_PIN_UNUSED) {
        ESP_LOGI(TAG, "irq=polling (pin_irq unwired)");
    } else if (!cfg.use_irq) {
        ESP_LOGI(TAG, "irq=polling (disabled by Kconfig)");
    } else {
        ESP_LOGW(TAG, "irq=polling (enable failed)");
    }

    // --- タイミングプリセット(docs/TIMING_PRESETS.md) ---
    // g_effectiveTimingProfile（init()が実際に適用した値）と、requested=
    // （コンパイル時のTIMING_PROFILE）の両方を出す。実行時のPollingBothへの
    // 降格が起きていないかを起動ログだけで判別できるように。
#if CONFIG_UWB_TAG_METHOD_DS
    {
        const uwb::TimingPresetDs ds = uwb::timingPresetDs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus) "
                 "final_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ds.responseTxDelayUus, ds.responseTxDelayUus * 1.02564,
                 (unsigned long)ds.finalTxDelayUus, ds.finalTxDelayUus * 1.02564);
    }
#else
    {
        const uwb::TimingPresetSs ss = uwb::timingPresetSs(g_effectiveTimingProfile);
        ESP_LOGI(TAG,
                 "timing profile=%s (version=%u, requested=%s) wait=%s response_tx_delay=%luuus(%.0fus)",
                 uwb::timingProfileName(g_effectiveTimingProfile), (unsigned)uwb::kTimingPresetVersion,
                 uwb::timingProfileName(TIMING_PROFILE), uwbDevice.irqActive() ? "irq" : "polling",
                 (unsigned long)ss.responseTxDelayUus, ss.responseTxDelayUus * 1.02564);
    }
#endif
    // 起動時チェック(docs/TIMING_PRESETS.md §4(b)): BothIrqはタグ側IRQも
    // 前提にしているのに、実際にはIRQが有効化されていない場合は目立つ
    // 警告を出す。**値は自動で変えない**(片側が勝手に変えると測距が壊れる)。
    if (uwb::timingProfileNeedsTagIrq(TIMING_PROFILE) && !uwbDevice.irqActive()) {
        ESP_LOGW(TAG,
                 "timing profile=%s はタグ側IRQも前提ですが irqActive()=false です"
                 "(pin_irq 未配線 / Kconfigで無効 / ISR登録失敗のいずれか)。"
                 "待ちはポーリングへフォールバックしましたが、折返しに最大1.2msかかるため"
                 "このプリセットの締切に間に合わない可能性が高いです。"
                 "値は自動で変えません — タグ・アンカー双方を PollingBoth で焼き直すこと"
                 "（docs/TIMING_PRESETS.md）。",
                 uwb::timingProfileName(TIMING_PROFILE));
    }

#if CONFIG_UWB_TAG_CONSOLE
    /* --- シリアルコンソール（別タスクで動く REPL）を起動する ---
     * コンソールは編集用のシャドウコピーだけを書き換え、この測位ループが
     * 1周期の先頭で takePendingTable() を呼んで取り込む
     *（方式と理由は tag_console.hpp の冒頭コメント）。
     * 起動に失敗しても測位そのものは継続する。 */
    bool consoleReady = false;
    {
        tagapp::StaticInfo consoleInfo;
        consoleInfo.boardName    = BOARD_NAME;
        consoleInfo.methodName   = METHOD_NAME;
        consoleInfo.chipName     = uwbDevice.chipName();
        consoleInfo.deviceId     = uwbDevice.deviceId();
        consoleInfo.tagAddr      = TAG_SHORT_ADDR;
        consoleInfo.panId        = PAN_ID;
        consoleInfo.source       = tableSource;
        consoleInfo.defaults     = kAnchors;
        consoleInfo.defaultCount = kNumAnchors;
        if (tagapp::sharedInit(g_workEntries, entryCount, consoleInfo)) {
            const esp_err_t consoleErr = tagapp::consoleStart();
            if (consoleErr == ESP_OK) {
                consoleReady = true;
                ESP_LOGI(TAG, "シリアルコンソールを起動しました（help でコマンド一覧。"
                              "JSON出力が邪魔なときは output off）");
            } else {
                ESP_LOGE(TAG, "シリアルコンソールを起動できませんでした (err=%s)。測位は継続します",
                         esp_err_to_name(consoleErr));
            }
        }
    }
#endif

    /* JSON Lines: アンカー一覧を最初に1回出す */
    printAnchorsLine(table);

    const int64_t bootUs = esp_timer_get_time();
    static uwb::RangingSample samples[uwb::kMaxAnchors];
    uint32_t failLogCounter = 0;
    uint32_t epochCount      = 0;
    uint32_t okFixCount      = 0;

    // タスクE: "type":"stats" 行の間引き用。起動直後ではなく、最初の
    // CONFIG_UWB_TAG_STATS_INTERVAL_MS が経ってから出す(0件のstatsを
    // 出すより、1周期ぶん貯まってからの方が意味がある)。
    int64_t lastStatsUs = bootUs;

    while (true) {
#if CONFIG_UWB_TAG_CONSOLE
        /* --- 1周期の先頭でだけ設定を取り込む ---
         * ここは測距も測位も走っていない唯一の地点。ロックはシャドウコピーの
         * コピー中だけで、測距・測位・JSON出力の間は一切保持しない。 */
        if (consoleReady) {
            size_t pendingCount = 0;
            if (tagapp::takePendingTable(g_workEntries, uwb::kMaxAnchors, &pendingCount)) {
                if (applyAnchorTable(table, g_workEntries, pendingCount)) {
                    ESP_LOGI(TAG, "アンカー登録テーブルを更新しました（%zu 件）", pendingCount);
                    // 【修正5】docs/archive/REVIEW_2026-08-21.md app層M-2:
                    // RangingScheduler::stats_[] はアンカーテーブルの添字で
                    // 引く成功率統計。テーブルを差し替えると台数・アドレス・
                    // 座標が変わりうるのに、reset せずに使い続けると
                    // 「新しいidx=2が指すアンカー」の成功率に「差し替え前の
                    // idx=2が指していた別アンカー」の統計が混ざってしまう
                    // （配列の添字が指す実体が変わっただけで、配列自体は
                    // クラッシュせず動き続けてしまうため気付きにくい）。
                    // テーブル差し替えが成功した直後に必ずリセットする。
                    scheduler.resetStats();
#if CONFIG_UWB_TAG_ENABLE_EKF
                    // 観測モデル（アンカー座標・台数）が変わったので EKF は組み直す。
                    pipeline.initEkf();
#endif
                    // 下流の可視化にも新しいテーブルを伝える。
                    printAnchorsLine(table);
                } else {
                    ESP_LOGE(TAG, "更新後のテーブルを適用できませんでした（前の設定のまま継続します）");
                }
            }
        }
#endif

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

        ++epochCount;
        if (lv2.ok) {
            ++okFixCount;
        }

        printMeasLine(t, table, samples, n);
        printFixLine(t, scheduler.lastCycleMs(), table, samples, n, lv0, lv2, haveLv3, lv3, timing, bootUs);

        // 【スタック監視】最初の測位周期の終了時に1回、app_mainのタスク
        // スタックのハイウォーターマーク（未使用の最小残量）を出す。
        // CONFIG_ESP_MAIN_TASK_STACK_SIZE（sdkconfig.defaultsで12288に
        // 設定済み。ここが3584バイトの既定のままだとEKF/Lv0/Lv2ソルバの
        // 呼び出し連鎖でスタック溢れpanicリブートする。docs/REVIEW_2026-08-21.md
        // §0 #3、main.cpp冒頭のg_jsonBufコメント参照）を実測で検証するため。
        // JSON行の形式は変えない（文書化された出力を壊さない）ので、
        // "type":"..."行としては出さず通常のESP_LOGIで出す。
        // 単位: ESP-IDF の uxTaskGetStackHighWaterMark() は vanilla FreeRTOS と
        // 違いワードではなく**バイト**で返す（~/esp/esp-idf/components/freertos/
        // FreeRTOS-Kernel/include/freertos/task.h:1496,1509「in bytes (as
        // opposed to words」）ので sizeof(StackType_t) を掛けてはいけない。
        if (epochCount == 1) {
            ESP_LOGI(TAG, "main task stack high-water mark: %u bytes free",
                     (unsigned)uxTaskGetStackHighWaterMark(NULL));
        }

        // タスクE: 約 CONFIG_UWB_TAG_STATS_INTERVAL_MS ごとに "type":"stats"
        // 行を出す。0 なら出さない（Kconfigのhelp参照）。
        if (CONFIG_UWB_TAG_STATS_INTERVAL_MS > 0) {
            const int64_t nowUs = esp_timer_get_time();
            if ((nowUs - lastStatsUs) >= static_cast<int64_t>(CONFIG_UWB_TAG_STATS_INTERVAL_MS) * 1000) {
                printStatsLine(t, table, scheduler, scheduler.lastCycleMs());
                lastStatsUs = nowUs;
                // 【スタック監視】stats行と同じタイミングでハイウォーター
                // マークも出す（起動直後の1回は上のepochCount==1で既出）。
                ESP_LOGI(TAG, "main task stack high-water mark: %u bytes free",
                         (unsigned)uxTaskGetStackHighWaterMark(NULL));
            }
        }

#if CONFIG_UWB_TAG_CONSOLE
        if (consoleReady) {
            tagapp::Status status;
            status.epochs      = epochCount;
            status.okFixes     = okFixCount;
            status.cycleMs     = scheduler.lastCycleMs();
            status.lastOk      = lv2.ok;
            status.p[0]        = lv2.p[0];
            status.p[1]        = lv2.p[1];
            status.p[2]        = lv2.p[2];
            status.gdop        = lv2.gdop;
            status.residualRms = lv2.residualRms;
            status.nUsed       = lv2.nUsed;
            status.nTotal      = lv2.nTotal;
            tagapp::publishStatus(status);
        }
#endif

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
