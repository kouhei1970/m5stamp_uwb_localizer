/**
 * @file main.cpp
 * @brief v2 タグファーム（docs/ARCHITECTURE_V2.md §3.2）。
 *
 * 測距・測位は uwb::RangingService（components/uwb_ranging）が専用タスクで
 * 回す。本ファイル（app_main）はセットアップだけを行い、
 *   - `uwb_ranging_svc` タスク（RangingService 内部。電波を触るのはこれだけ）
 *   - `uwb_log` タスク（本ファイルが起こす。JSON Lines の出力を担当）
 *   - コンソール REPL タスク（tag_console.cpp。CONFIG_UWB_TAG_CONSOLE 時のみ）
 * を起動したあと app_main 自身は return する（各タスクは動き続ける）。
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
 * 通常ログ(ESP_LOGx)とデータ行(printf)を意図的に分けている）。JSON整形・
 * 出力は **uwb_log タスクだけの仕事**にした（docs/ARCHITECTURE_V2.md §1
 * 「電波を触るタスクは1つ」の精神を測位側にも適用: uwb_ranging_svc タスクは
 * ホットループ中に ESP_LOG も printf も一切呼ばない）。
 *
 * アンカー登録テーブルは **NVS に保存された値を優先し、無ければ下の
 * kAnchors[] を既定値として使う**（components/uwb_cfgstore）。NVSが空・
 * 未初期化・破損のいずれでも必ず kAnchors[] へフォールバックするので、
 * 購入者が何も設定しなければ従来どおり動く。実行時の変更は
 * USB-Serial/JTAG 上のシリアルコンソール（tag_console.cpp、
 * CONFIG_UWB_TAG_CONSOLE で無効化可）から anchor set / save で行う。
 * これにより「座標を数cm直すたびに再ビルド・再書き込み」が不要になる。
 * コンソールとの同時実行の解き方は tag_console.hpp の冒頭コメントと
 * uwb_ranging_service.hpp の tableMutex() コメントを参照。
 */
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h" // IP_EVENT/IP_EVENT_STA_GOT_IP（起動パンくずのstage4用）
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "uwb_cfgstore.hpp"
#include "uwb_port.h"
#include "nvs.h"
#include "uwb_status_led.h"
#include "uwb_qm33120.hpp"
#include "uwb_qm33120_phy_kconfig.hpp" // docs/ARCHITECTURE_V2.md §4: phyConfigFromKconfig()
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_pipeline.hpp"
#include "uwb_ranging_scheduler.hpp"
#include "uwb_ranging_service.hpp"
#include "uwb_ranging_types.hpp"

#if CONFIG_UWB_TAG_CONSOLE
#include "tag_console.hpp"
#endif

// Wi-Fi + ブラウザダッシュボード + 無線コンソール（components/uwb_net）。
// CONFIG_UWB_NET_ENABLE=n でも公開関数は no-op になるだけなので、この
// #include と下の呼び出しは常に有効にしてよい（uwb_net.hpp 冒頭コメント）。
// Wi-Fi + browser dashboard + remote console. Safe to include/call
// unconditionally - every public function is a no-op when disabled.
#include "uwb_net.hpp"

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

/* ==================================================================== *
 * uwb_net（components/uwb_net）の "node" 行へ載せる role固有情報
 * ==================================================================== *
 * uwb::net::Config::statusFn はネットワークタスク（コア0）から1秒ごとに
 * 呼ばれるコールバックで、電波（UWBチップ）を一切触ってはいけない
 * （scratchpad/NET_SPEC.md §8）。そのため「phy:」起動ログと同じ値を、
 * 起動時に一度だけここの静的バッファへ整形しておき、statusFn はそれを
 * 読むだけにする。
 *
 * Qm33120::logPhy() が読む Impl::applied_phy への公開アクセサは無い
 * （uwb_qm33120.hpp logPhy() コメント参照）。本ファームは
 * uwb::phyConfigFromKconfig() が channel だけでなく全項目を明示指定するため、
 * begin() に渡した要求値（下の `phy` ローカル変数）と実際に適用された値は
 * 一致する（同コメントの「channelだけを変えた場合に限り異なる」の対象外）。 */
static char g_netPhyStr[40]       = "";
static char g_netPllCoarseStr[8] = "";

/** uwb_qm33120_phy_kconfig.cpp のファイル内 pacSizeCount() と同じ対応表
 *  （非公開のためログ表示用にここで複製する）。 */
static unsigned pacSizeCountForNet(uwb::PacSize pac)
{
    switch (pac) {
    case uwb::PacSize::Pac4:  return 4;
    case uwb::PacSize::Pac8:  return 8;
    case uwb::PacSize::Pac16: return 16;
    case uwb::PacSize::Pac32: return 32;
    }
    return 0;
}

/** uwb::net::Config::statusFn 本体。"node" 行へ
 *  `"phy":"...","pll_coarse":"...","method":"...","retry_max":N,"retry_delay_ms":N`
 *  を追記する（先頭カンマ・波括弧なし。uwb_net.hpp StatusJsonFn のコメント参照）。 */
static size_t tagNetStatus(char* buf, size_t cap, void* /*user*/)
{
    const int n = std::snprintf(
        buf, cap,
        "\"phy\":\"%s\",\"pll_coarse\":\"%s\",\"method\":\"%s\",\"retry_max\":%d,\"retry_delay_ms\":%d",
        g_netPhyStr, g_netPllCoarseStr,
#if CONFIG_UWB_TAG_METHOD_DS
        "DS",
#else
        "SS",
#endif
        static_cast<int>(CONFIG_UWB_TAG_RETRY_MAX), static_cast<int>(CONFIG_UWB_TAG_RETRY_DELAY_MS));
    return (n > 0) ? static_cast<size_t>(n) : 0;
}

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
 * JSON Lines 出力（uwb_log タスク専用。他タスクからは呼ばないこと）
 * ==================================================================== */

#define JSON_BUF_SIZE 2048

/**
 * @brief JSON 1行ぶんの組み立てバッファ。
 *
 * **uwb_log タスクからしか使わない**（v1 の main ループから v2 で移動した。
 * measurement/positioning は uwb_ranging_svc タスク側に移り、JSON整形は
 * uwb_log タスク側に一本化された）ので static でよい。スタックではなく
 * .bss に置くのは、uwb_log タスクのスタック（CONFIG_UWB_TAG_LOG_TASK_STACK）
 * を切り詰めるため。2KB の配列を積むとタスクスタックがすぐ逼迫する。
 */
static char g_jsonBuf[JSON_BUF_SIZE];

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
 * jsonAppend() はバッファが逼迫すると超過分を無言で捨てて *off をそれ以上
 * 進めなくする（*off は cap-1 付近で頭打ちになる）。この関数は「容量が
 * 足りなければ直前のバイトを巻き戻してでも」"]}\n" を必ず書き切る。
 * 巻き戻しが発生した（＝この行のどこかで jsonAppend() の切り詰めが既に
 * 起きていた）場合は、"]}\n" の直前にテキストの目印 "...TRUNCATED..." を
 * 残す。切り詰められた行は（配列要素の途中で切れている可能性があるため）
 * そもそも有効なJSONである保証はできないが、それは今回の要件ではない —
 * 保証するのは「必ず改行で終わる」ことだけであり、これにより巻き添えで
 * 壊れる行をこの1行だけに抑えられる。
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

/**
 * @brief JSON Lines を出力してよいか。
 *
 * コンソール有効時は output コマンドで一時的に止められる（毎周期複数行の
 * JSON が流れているとコンソールで設定作業ができないため）。無効時は常に
 * true を返す定数関数になり、tagapp 名前空間自体が存在しない
 * （CONFIG_UWB_TAG_CONSOLE=n だと tag_console.cpp/hpp ごとビルド対象から
 * 外れる）構成でもビルドが通る。
 */
static inline bool jsonOutputEnabled()
{
#if CONFIG_UWB_TAG_CONSOLE
    return tagapp::jsonOutputEnabled();
#else
    return true;
#endif
}

/** 起動時に1回、およびアンカー表が変わるたびに出す "type":"anchors" 行。
 *  呼び出し側が service.tableMutex() を保持した状態で呼ぶこと。 */
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
    jsonAppendClose(buf, JSON_BUF_SIZE, &off);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, off);
}

/** 毎エポック出す "type":"meas" 行（JsonLinesHal がそのまま読める標準形）。
 *  欠測したアンカーは含めない。呼び出し側が service.tableMutex() を
 *  保持した状態で呼ぶこと（table.entry() のIDルックアップのため）。 */
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
    jsonAppendClose(buf, JSON_BUF_SIZE, &off);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, off);
}

/** PositionResult の中身を（波括弧なしで）キー:値の並びとして書く。
 *  トップレベルへ直接展開する場合と、"lv0":{...} のように入れ子にする場合の
 *  両方から呼ぶ共通部分。 */
static void appendResultBody(char* buf, size_t cap, size_t* off, const uwb::PositionResult& r)
{
    jsonAppend(buf, cap, off,
               "\"ok\":%s,\"ambiguous\":%s,\"solvable\":%s,"
               "\"p\":[%.4f,%.4f,%.4f],\"gdop\":%.4f,\"residual_rms\":%.4f,\"sigma\":%.4f,"
               "\"n_used\":%d,\"n_total\":%d,\"redundancy\":%d,\"excluded\":\"0x%08lX\"",
               r.ok ? "true" : "false", r.ambiguous ? "true" : "false", r.solvable ? "true" : "false",
               static_cast<double>(r.p[0]), static_cast<double>(r.p[1]), static_cast<double>(r.p[2]),
               static_cast<double>(r.gdop), static_cast<double>(r.residualRms),
               static_cast<double>(r.sigma), r.nUsed, r.nTotal, r.redundancy, r.excluded);
}

/** ソルバの計算時間 [us]。CycleResult::solveUsLv0/Lv2/Lv3（uwb_ranging_svc
 *  タスクが esp_timer_get_time() の差分で埋める）をそのまま運ぶだけの入れ物。 */
struct EpochTiming {
    int64_t solveLv0Us = 0;
    int64_t solveLv2Us = 0;
    int64_t solveLv3Us = 0; // haveLv3==false のときは未使用
};

/** 毎エポック出す "type":"fix" 行（本プロジェクト独自の診断情報。
 *  JsonLinesHal 側は type を認識せず読み捨てるだけなので安全に共存できる）。
 *  トップレベルの ok/ambiguous/p/... は Lv2（本番）の結果を表す。
 *  呼び出し側が service.tableMutex() を保持した状態で呼ぶこと。
 *  @param bootUs anchors[].t を fix 行の "t" と同じ基準（起動からの秒）に
 *                 直すための引き算に使う。 */
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
        // t_us は rangeOne() が測距開始直前に必ず埋める（成功/失敗を問わない。
        // uwb_ranging_scheduler.cpp 参照）ので、ok:false でも「いつ試みたか」
        // は分かる。fix行トップレベルの "t" と比較できるよう、生の t_us では
        // なく bootUs を引いた相対秒で出す。
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
    jsonAppendClose(buf, JSON_BUF_SIZE, &off);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, off);
}

/**
 * @brief 約1秒ごとに出す "type":"stats" 行（アンカーごとの成功率）。
 *
 * その周期に測ったものだけでなく、**登録されている全アンカー**
 * （無効化されているものも含む。attempts=0 のまま）を出す。
 * retries/rescued は再試行の内訳: retriesは無線試行としての再試行の総回数、
 * rescuedは最初の試行が失敗し再試行のいずれかで成功したサイクル数
 * （succの内数）。
 *
 * 呼び出し側が service.tableMutex() を保持した状態で呼ぶこと（table参照
 * のため）。service.stats() 自体は内部で別の軽量ミューテックスを取るので、
 * ここで二重にブロックすることはない。
 */
static void printStatsLine(double t, const uwb::AnchorTable& table, const uwb::RangingService& service,
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
        const uwb::AnchorStats s = service.stats(i);
        char id[8];
        formatAnchorId(table.entry(i).short_addr, id, sizeof(id));
        jsonAppend(buf, JSON_BUF_SIZE, &off,
                   "%s{\"a\":\"%s\",\"att\":%lu,\"succ\":%lu,\"rate\":%.4f,\"retries\":%lu,\"rescued\":%lu}",
                   (i == 0) ? "" : ",", id, static_cast<unsigned long>(s.attempts),
                   static_cast<unsigned long>(s.successes), static_cast<double>(s.successRate()),
                   static_cast<unsigned long>(s.retries), static_cast<unsigned long>(s.rescued));
    }
    jsonAppendClose(buf, JSON_BUF_SIZE, &off);
    if (uwb_port_usb_host_connected()) {  // ホスト不在時は USB へ書かない (uwb_port.h 参照) / skip USB writes with no host
        std::fputs(buf, stdout);
    }
    uwb::net::publishLine(buf, off);
}

/* ==================================================================== *
 * アンカー登録テーブルの適用
 * ==================================================================== */

/** 測位開始前にコンソールから渡すテーブルの一時領域。
 *  app_main のセットアップ中にしか使わないので static でよい
 *  （app_main のスタックを節約する）。 */
static uwb::AnchorEntry g_workEntries[uwb::kMaxAnchors];

/**
 * @brief テーブル編集後の後処理（配置チェック + 2D自動フォールバック + ログ）。
 *
 * 起動時（applyAnchorTable() から）と、コンソールでの編集後の両方から呼ぶ
 * 共通関数（tag_console.hpp の PlacementPolicyFn 経由）。前回 dim=2 へ
 * フォールバックしていた状態を引きずらないよう、判定前に必ず dim=3 へ
 * 戻してから checkPlacement() をやり直す。
 *
 * 呼び出し側が service.tableMutex() を保持した状態で呼ぶこと
 * （table.setDimension2D/3D() はソルバが読んでいる最中に呼んではいけない
 * ため。tag_console.hpp 冒頭コメント参照）。
 */
static void applyPlacementPolicy(uwb::AnchorTable& table)
{
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
}

/**
 * @brief アンカー登録テーブルを丸ごと差し替え、applyPlacementPolicy() を
 * かける。起動時（NVS もしくは kAnchors[] の初期テーブル適用）専用
 * （service 起動前なので tableMutex 不要。以後の差し替えは
 * tag_console.cpp が table.set()/update() + applyPlacementPolicy() を
 * 直接呼ぶ）。
 *
 * @return 差し替えに成功したら true。false のときテーブルは変更されない。
 */
static bool applyAnchorTable(uwb::AnchorTable& table, const uwb::AnchorEntry* entries, size_t count)
{
    if (!table.set(entries, count)) {
        ESP_LOGE(TAG, "AnchorTable::set() failed (count=%zu, kMaxAnchors=%zu)", count, uwb::kMaxAnchors);
        return false;
    }
    applyPlacementPolicy(table);
    return true;
}

/* ==================================================================== *
 * uwb_log タスク（JSON Lines 出力・診断ログを一手に引き受ける）
 * ==================================================================== */

/** uwbLogTask() に渡す引数。タスクが動き続ける間ずっと参照されるため、
 *  app_main のローカル変数ではなく static 記憶域に置く。 */
struct LogTaskArgs {
    uwb::RangingService* service = nullptr;
    uwb::AnchorTable* table       = nullptr;
    int64_t bootUs                 = 0;
};
static LogTaskArgs g_logTaskArgs;

/** uwb_log タスクの優先度。RangingService タスク（UWB_TAG_SERVICE_TASK_PRIO、
 *  既定18）より必ず低くすること。Kconfig 化はしていない（この値自体は
 *  スタックサイズと違って実測で追い込む類のものではなく、「サービスより
 *  低ければ何でもよい」固定値のため）。 */
static constexpr UBaseType_t kLogTaskPrio = 10;

/**
 * @brief JSON Lines 出力・診断ログ用タスク（コア0、docs/ARCHITECTURE_V2.md §3.2）。
 *
 * uwb::RangingService::resultQueue() をブロッキング受信し、"meas"/"fix" 行を
 * 出す。CONFIG_UWB_TAG_STATS_INTERVAL_MS ごとに "stats" 行、コンソールが
 * アンカー表を書き換えるたび（tableGeneration() の変化を検知）に "anchors"
 * 行を出し直す。「測位不能」警告の間引きと、起動5秒後のスタック残量ログも
 * ここで行う。
 */
static void uwbLogTask(void* argRaw)
{
    LogTaskArgs* args               = static_cast<LogTaskArgs*>(argRaw);
    uwb::RangingService& service    = *args->service;
    uwb::AnchorTable& table          = *args->table;
    const int64_t bootUs             = args->bootUs;
    const SemaphoreHandle_t tableMtx = service.tableMutex();

    // 起動時に1回、アンカー一覧を出す。
    xSemaphoreTake(tableMtx, portMAX_DELAY);
    printAnchorsLine(table);
    xSemaphoreGive(tableMtx);

#if CONFIG_UWB_TAG_CONSOLE
    uint32_t lastTableGen = tagapp::tableGeneration();
#endif

    uint32_t failLogCounter    = 0;
    int64_t lastStatsUs         = esp_timer_get_time();
    const int64_t taskStartUs = esp_timer_get_time();
    bool stackLogged            = false;

    uwb::CycleResult result;
    while (true) {
        if (xQueueReceive(service.resultQueue(), &result, portMAX_DELAY) != pdTRUE) {
            continue;
        }

#if CONFIG_UWB_TAG_CONSOLE
        // コンソールがアンカー表を書き換えていたら "anchors" 行を出し直す
        // （下流の可視化に新しい表を伝える。tag_console.hpp 冒頭コメント参照）。
        const uint32_t gen = tagapp::tableGeneration();
        if (gen != lastTableGen) {
            lastTableGen = gen;
            xSemaphoreTake(tableMtx, portMAX_DELAY);
            printAnchorsLine(table);
            xSemaphoreGive(tableMtx);
        }
#endif

        const double t = static_cast<double>(result.tUs - bootUs) / 1e6;

        EpochTiming timing;
        timing.solveLv0Us = result.solveUsLv0;
        timing.solveLv2Us = result.solveUsLv2;
        timing.solveLv3Us = result.solveUsLv3;

        xSemaphoreTake(tableMtx, portMAX_DELAY);
        printMeasLine(t, table, result.samples, result.n);
        printFixLine(t, result.cycleMs, table, result.samples, result.n, result.lv0, result.lv2,
                     result.haveLv3, result.lv3, timing, bootUs);
        xSemaphoreGive(tableMtx);

        // 【スタック監視】起動5秒後に1回、uwb_ranging_svc タスクと
        // uwb_log タスク自身のスタックハイウォーターマーク（未使用の
        // 最小残量）を出す。単位はバイト（ESP-IDF の
        // uxTaskGetStackHighWaterMark() は vanilla FreeRTOS と違いワードでは
        // なく**バイト**を返す）。UWB_TAG_SERVICE_TASK_STACK /
        // UWB_TAG_LOG_TASK_STACK の既定値が妥当かをここで確認する。
        if (!stackLogged && (esp_timer_get_time() - taskStartUs) >= 5 * 1000 * 1000) {
            stackLogged = true;
            ESP_LOGI(TAG, "uwb_ranging_svc task stack high-water mark: %u bytes free",
                     (unsigned)uxTaskGetStackHighWaterMark(service.taskHandle()));
            ESP_LOGI(TAG, "uwb_log task stack high-water mark: %u bytes free",
                     (unsigned)uxTaskGetStackHighWaterMark(nullptr));
        }

        if (CONFIG_UWB_TAG_STATS_INTERVAL_MS > 0) {
            const int64_t nowUs = esp_timer_get_time();
            if ((nowUs - lastStatsUs) >= static_cast<int64_t>(CONFIG_UWB_TAG_STATS_INTERVAL_MS) * 1000) {
                xSemaphoreTake(tableMtx, portMAX_DELAY);
                printStatsLine(t, table, service, result.cycleMs);
                xSemaphoreGive(tableMtx);
                lastStatsUs = nowUs;
            }
        }

        if (!result.lv2.ok) {
            // 「測位不能」（有効測距不足）または解ソルバ失敗（同一平面配置の
            // 既知の縮退等）。JSON行そのものは毎回stdoutへ出るので、こちらは
            // 人間向けの要約ログを間引いて出すだけにする。
            if ((++failLogCounter % 20) == 1) {
                ESP_LOGW(TAG, "position unavailable: solvable=%d ok=%d n_total=%d anchors=%zu",
                         result.lv2.solvable, result.lv2.ok, result.lv2.nTotal, table.size());
            }
        }
    }
}

/* ==================================================================== *
 * app_main
 * ==================================================================== */


/* ==================================================================== *
 * 起動パンくず（給電のみ運用の凍結調査、2026-08-31。docs/HANDOFF.md 参照）
 * 電源断でも消えない NVS に「起動回数」と「前回どの段階まで進んだか」を
 * 記録する。PC なし給電で固まる問題の切り分け用（ブラウンアウトの再起動
 * ループなら起動回数が増え続け、凍結なら段階が途中で止まって残る）。
 * Boot breadcrumbs for diagnosing the standalone-power freeze: persist a
 * boot counter and the last reached stage in NVS so the previous run's
 * fate can be read back after a power cycle.
 *
 * 【2026-08-31 追記】以下を追加した:
 *   1. 履歴リング（NVS blob "hist"）: stage==0 のたびに「前回の記録
 *      (boot_n, stage)」を1件追記する。電源投入を繰り返すたびに積み上がる
 *      ので、bootlog コマンド（tag_console.cpp）で「毎回同じ段階で
 *      止まっていないか」を後から見返せる。
 *   2. +8秒の再表示: 挿し直し直後にログを取り逃した場合の対策として、
 *      stage==0 のときに一発 (one-shot) の esp_timer を仕掛け、8秒後に
 *      前回記録と現在stageを再度 ESP_LOGW で出す。タイマコールバックは
 *      esp_timer タスク上で動きブロッキング禁止なので、NVSは読まず
 *      static 変数だけを見る。
 *   3. stage 4 (got_ip): IP_EVENT_STA_GOT_IP のハンドラから呼ぶ
 *      （app_main、uwb::net::start() の後で登録）。
 * ==================================================================== */

namespace {

constexpr size_t kBreadcrumbHistCapacity = 16;

/** NVS blob "hist" の1要素。tag_console.cpp の bootlog コマンドも同じ
 *  レイアウト（キー名・この構造体・容量）を前提に独立して読むので、
 *  変更する場合は両方を揃えること。
 *  One element of the "hist" NVS blob. tag_console.cpp's `bootlog` command
 *  independently assumes this same layout (key names, this struct,
 *  capacity) - keep both in sync if this changes. */
struct BreadcrumbEntry {
    uint32_t bootN;
    uint8_t stage;
} __attribute__((packed));

/** 前回記録（stage==0で読んだ直後の値）と現在stageを、+8秒タイマの
 *  コールバックがNVSを読まずに使えるよう static に持つ。 */
uint32_t g_bcLastBootN        = 0;
uint8_t g_bcLastStage          = 0xFF; // 0xFF = 前回記録なし（初回起動）
uint8_t g_bcCurStage           = 0xFF;
esp_timer_handle_t g_bcTimer = nullptr;

/**
 * @brief 履歴リングへ1件追記する。呼び出し側が開いた handle をそのまま使う
 * （呼び出し側が nvs_commit() する）。
 *
 * 読み出し失敗（キー無し・サイズ不一致等）は空リング（全ゼロ）として扱い、
 * 書き込み失敗もログのみで続行する（起動を止めない）。
 */
void bootBreadcrumbPushHist(nvs_handle_t h, uint32_t bootN, uint8_t stage)
{
    BreadcrumbEntry hist[kBreadcrumbHistCapacity];
    std::memset(hist, 0, sizeof(hist));
    size_t histBytes = sizeof(hist);
    if (nvs_get_blob(h, "hist", hist, &histBytes) != ESP_OK || histBytes != sizeof(hist)) {
        std::memset(hist, 0, sizeof(hist)); // 読めなければ空リング扱い
    }
    uint32_t histN = 0;
    nvs_get_u32(h, "hist_n", &histN); // キー無しなら0のまま（＝空リング）

    const size_t slot = histN % kBreadcrumbHistCapacity;
    hist[slot].bootN    = bootN;
    hist[slot].stage     = stage;
    ++histN;

    if ((nvs_set_blob(h, "hist", hist, sizeof(hist)) != ESP_OK) || (nvs_set_u32(h, "hist_n", histN) != ESP_OK)) {
        ESP_LOGW(TAG, "breadcrumb: failed to persist boot history ring (non-fatal)");
    }
}

/**
 * @brief +8秒の再表示。挿し直し直後にログを取り逃した場合の対策。
 * esp_timer タスク上で動く（ブロッキング API 呼び出し厳禁）ため、NVSは
 * 読まず static 変数だけを見る。
 */
void bootBreadcrumbTimerCb(void* /*arg*/)
{
    ESP_LOGW(TAG,
             "breadcrumb +8s: previous run boot_n=%lu last_stage=%u, this run now at stage=%u "
             "(0=entry 1=console 2=net_start 3=net_ok 4=got_ip)",
             static_cast<unsigned long>(g_bcLastBootN), static_cast<unsigned>(g_bcLastStage),
             static_cast<unsigned>(g_bcCurStage));
}

} // namespace

static void bootBreadcrumb(uint8_t stage)
{
    g_bcCurStage = stage; // NVSの成否に関わらず先に更新する（+8秒タイマ用）

    nvs_handle_t h;
    if (nvs_open("dbg", NVS_READWRITE, &h) != ESP_OK) {
        return;
    }
    if (stage == 0) {  // 起動直後: 前回の記録を表示してから初期化 / at boot: report last run, then reset
        uint32_t bootN = 0;
        uint8_t last   = 0xFF;
        nvs_get_u32(h, "boot_n", &bootN);
        nvs_get_u8(h, "stage", &last);
        ESP_LOGW(TAG, "breadcrumb: previous run: boot_n=%lu last_stage=%u "
                      "(0=entry 1=console 2=net_start 3=net_ok 4=got_ip)",
                 static_cast<unsigned long>(bootN), static_cast<unsigned>(last));
        g_bcLastBootN = bootN;
        g_bcLastStage = last;

        if (last != 0xFF) { // 0xFF＝前回記録なし（初回起動）はリングへ積まない
            bootBreadcrumbPushHist(h, bootN, last);
        }

        nvs_set_u32(h, "boot_n", bootN + 1);

        // +8秒の再表示タイマ（一発、初回のみ生成）。登録失敗はログのみで続行。
        if (g_bcTimer == nullptr) {
            const esp_timer_create_args_t timerArgs = {
                .callback             = &bootBreadcrumbTimerCb,
                .arg                    = nullptr,
                .dispatch_method       = ESP_TIMER_TASK,
                .name                   = "uwb_bc_8s",
                .skip_unhandled_events = true,
            };
            if (esp_timer_create(&timerArgs, &g_bcTimer) != ESP_OK) {
                ESP_LOGW(TAG, "breadcrumb: esp_timer_create failed (8s re-log disabled, non-fatal)");
                g_bcTimer = nullptr;
            } else if (esp_timer_start_once(g_bcTimer, 8ULL * 1000ULL * 1000ULL) != ESP_OK) {
                ESP_LOGW(TAG, "breadcrumb: esp_timer_start_once failed (8s re-log disabled, non-fatal)");
            }
        }
    }
    nvs_set_u8(h, "stage", stage);
    nvs_commit(h);
    nvs_close(h);
}

/**
 * @brief stage 4 (got_ip) を記録する IP_EVENT_STA_GOT_IP ハンドラ。
 * app_main が uwb::net::start() の後で登録する（イベントループは
 * uwb_net が作成済み。CONFIG_UWB_NET_ENABLE=n のときは未作成なので登録
 * 自体が失敗するが、呼び出し側でログのみ出して続行する）。
 */
static void bootBreadcrumbGotIpHandler(void* /*arg*/, esp_event_base_t /*base*/, int32_t /*id*/, void* /*data*/)
{
    bootBreadcrumb(4);
}

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
    bootBreadcrumb(0);
    size_t entryCount = 0;
    const uwb::ConfigSource tableSource = uwb::ConfigStore::loadAnchorTable(
        kAnchors, kNumAnchors, g_workEntries, uwb::kMaxAnchors, &entryCount);

    ESP_LOGI(TAG, "uwb_tag firmware (v2/RangingService), board=%s method=%s anchors=%zu (%s)", BOARD_NAME,
             METHOD_NAME, entryCount, uwb::configSourceName(tableSource));

    // static にする理由: RangingService::start() はこのオブジェクトへの
    // 生ポインタを保持し、専用タスクが動き続ける間ずっと参照する。
    // app_main はセットアップ後に return する（v1と違い無限ループでは
    // 居座らない）ので、app_main のスタック上（ローカル変数）に置くと
    // return 後にダングリングポインタになる。
    // Must be static: RangingService::start() keeps a raw pointer to this
    // object for as long as its task runs. app_main now returns after
    // setup (unlike v1's infinite loop), so a plain local here would dangle
    // once app_main's stack frame goes away.
    static uwb::Qm33120 uwbDevice;
    const uwb::Config cfg = makeConfigFromBoard();
    // docs/ARCHITECTURE_V2.md §4: PHY を共通 Kconfig (UWB_PHY_*,
    // components/uwb_qm33120/Kconfig) から選ぶ。firmware/anchor と同じ関数。
    const uwb::PhyConfig phy = uwb::phyConfigFromKconfig();

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

    // docs/ARCHITECTURE_V2.md §4: 実際に適用された PHY 設定を起動ログへ出す
    // （firmware/anchor / firmware/twr と同じ書式。スクリプトが grep する）。
    uwbDevice.logPhy(TAG);
    // 校正結果（PLL粗調整コード・OTP値等）を、強制する前の状態としてまず出す。
    uwbDevice.logCal(TAG);

    // 2026-08-30 実機結果: 本番既定(850kbps/256, PollingBoth, IRQ) + PLL
    // 自動校正まかせは初回試行成功率p=50.1%だったが、粗調整コードを0x23へ
    // 固定すると90%だった（docs/HANDOFF.md §0-B、components/uwb_qm33120/
    // Kconfig の UWB_PHY_PLL_COARSE_MODE ヘルプ参照）。0x23は基板固有の値
    // なので、既定は個体ごとのOTPから読む OTP モード。
    // firmware/anchor/main/main.cpp と同一のブロック（コピー、共有関数化は
    // していない - 両ファームともこの数行だけの単純な呼び出しであるため）。
#if CONFIG_UWB_PHY_PLL_COARSE_MODE_OTP
    {
        const bool pllOk = uwbDevice.forcePllCoarseFromOtp();
        if (!pllOk) {
            ESP_LOGW(TAG,
                     "forcePllCoarseFromOtp(): OTP field looked unprogrammed/invalid; left the chip's own "
                     "calibration in place");
        }
        uwbDevice.logCal(TAG);
    }
#elif CONFIG_UWB_PHY_PLL_COARSE_MODE_FIXED
    {
        const bool pllOk = uwbDevice.forcePllCoarse(static_cast<uint8_t>(CONFIG_UWB_PHY_PLL_COARSE_CH9));
        if (!pllOk) {
            ESP_LOGW(TAG,
                     "forcePllCoarse(0x%02X) could not lock the forced code; radio fell back to its normal "
                     "calibration path (see the DIAG_PLL_COARSE(*) lines above)",
                     (unsigned)CONFIG_UWB_PHY_PLL_COARSE_CH9);
        }
        uwbDevice.logCal(TAG);
    }
#endif
    // UWB_PHY_PLL_COARSE_MODE=Auto: no forcing call - the logCal() line
    // above already shows the chip's own auto-calibration result.

    // uwb_net の "node" 行用に、起動時に決まったPHY/PLL粗調整の要約を
    // 一度だけ静的バッファへ書いておく（このファイル冒頭の tagNetStatus()
    // コメント参照。ネットワークタスクからは電波を一切触らない）。
    std::snprintf(g_netPhyStr, sizeof(g_netPhyStr), "%s/pre%u/pac%u/ch%u",
                  (phy.dataRate == uwb::DataRate::Rate850K) ? "850k" : "6m8",
                  static_cast<unsigned>(phy.preambleLength), pacSizeCountForNet(phy.pacSize),
                  static_cast<unsigned>(phy.channel));
#if CONFIG_UWB_PHY_PLL_COARSE_MODE_FIXED
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "0x%02X",
                  static_cast<unsigned>(CONFIG_UWB_PHY_PLL_COARSE_CH9));
#elif CONFIG_UWB_PHY_PLL_COARSE_MODE_OTP
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "otp");
#else
    std::snprintf(g_netPllCoarseStr, sizeof(g_netPllCoarseStr), "auto");
#endif

    // init() may have downgraded the requested TIMING_PROFILE to PollingBoth
    // if the IRQ line turned out to be dead (Qm33120::verifyIrqLine()). Carry
    // the profile that was actually applied forward from here on.
    // IRQ 線が死んでいると init() が要求プリセットを PollingBoth へ降格させて
    // いることがあるため、以降は「実際に適用されたプリセット」を使う。
    g_effectiveTimingProfile = uwbDevice.config().timing_profile;

    // 起動からの経過秒（JSON行の "t"）の基準時刻。以後は uwb_log タスクが
    // CycleResult::tUs から引き算して使う。
    const int64_t bootUs = esp_timer_get_time();

    /* --- アンカー登録テーブルの構築 + 起動時チェック（必須） ---
     * service 起動前なので tableMutex は不要（他タスクがまだ触らない）。 */
    static uwb::AnchorTable table; // .bss へ置いて app_main のスタックを節約する
    if (!applyAnchorTable(table, g_workEntries, entryCount)) {
        return;
    }

    /* --- RangingService の設定を Kconfig から組み立てる --- */
    uwb::ServiceConfig svcCfg;
#if CONFIG_UWB_TAG_METHOD_DS
    svcCfg.scheduler.method = uwb::RangingMethod::DS;
#else
    svcCfg.scheduler.method = uwb::RangingMethod::SS;
#endif
    svcCfg.scheduler.panId                = PAN_ID;
    svcCfg.scheduler.tagShortAddr         = TAG_SHORT_ADDR;
    svcCfg.scheduler.perAnchorIntervalMs = CONFIG_UWB_TAG_PER_ANCHOR_INTERVAL_MS;
    // 周期の刻みは ServiceConfig::cycleIntervalMs 側だけで行う（二重ペーシング
    // 回避。uwb_ranging_service.hpp の ServiceConfig::cycleIntervalMs コメント
    // 参照）。scheduler 側は明示的に 0 のままにする。
    svcCfg.scheduler.cycleIntervalMs = 0;
    // 失敗したアンカーへの即時再試行（docs/HANDOFF.md §0-C「再試行の待ち時間」）。
    // DS-TWRはretryDelayMsに≥10msが必須（既定はKconfigのdefault 10 if
    // UWB_TAG_METHOD_DS）。根拠はKconfig.projbuildのUWB_TAG_RETRY_DELAY_MS
    // ヘルプとSchedulerConfig::retryDelayMsのフィールドコメント参照。
    svcCfg.scheduler.retryMax      = CONFIG_UWB_TAG_RETRY_MAX;
    svcCfg.scheduler.retryDelayMs = CONFIG_UWB_TAG_RETRY_DELAY_MS;
    // ssDefaults/dsDefaultsは構造体既定値(=PollingBothの値と完全一致。
    // docs/TIMING_PRESETS.md §2)のまま個別上書きされていないので、実効
    // プリセットがPollingBoth(明示選択時、またはIRQ線が死んでいて init()が
    // 実行時に降格した場合)のときはここでのプリセット適用はno-op。Kconfig
    // の既定はBothIrqであり、その場合やIRQが実際に生きている場合はここで
    // 初めて値が反映される。
    uwb::applyTimingProfile(svcCfg.scheduler.ssDefaults, g_effectiveTimingProfile);
    uwb::applyTimingProfile(svcCfg.scheduler.dsDefaults, g_effectiveTimingProfile);

    svcCfg.pipeline.defaultLevel = uwb::SolverLevel::Lv2;
#if CONFIG_UWB_TAG_ENABLE_EKF
    svcCfg.enableEkf = true;
#else
    svcCfg.enableEkf = false;
#endif
    svcCfg.cycleIntervalMs = static_cast<uint32_t>(CONFIG_UWB_TAG_CYCLE_INTERVAL_MS);
    svcCfg.core             = CONFIG_UWB_TAG_SERVICE_TASK_CORE;
    svcCfg.prio              = CONFIG_UWB_TAG_SERVICE_TASK_PRIO;
    svcCfg.stack             = static_cast<uint32_t>(CONFIG_UWB_TAG_SERVICE_TASK_STACK);

    static uwb::RangingService service;
    if (!service.start(uwbDevice, table, svcCfg)) {
        ESP_LOGE(TAG, "RangingService::start() failed, aborting");
        return;
    }

    ESP_LOGI(TAG, "begin() + PHY config OK, RangingService started (core=%d prio=%d stack=%lu)", svcCfg.core,
             svcCfg.prio, (unsigned long)svcCfg.stack);

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
     * コンソールは生きた table を service.tableMutex() 保持中に直接編集する
     * （方式と理由は tag_console.hpp の冒頭コメント）。
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
        consoleInfo.applyPlacementPolicy = &applyPlacementPolicy;
        if (tagapp::sharedInit(table, service, consoleInfo)) {
            bootBreadcrumb(1);
    const esp_err_t consoleErr = tagapp::consoleStart();
    // REPL が USJ ドライバを入れた後に、ホスト不在時のログ抑止を有効化する
    // (給電専用アダプタで送信バッファ満杯 → 全タスク停止を防ぐ)。
    // After the REPL installed the USJ driver, arm the no-host log guard.
    uwb_port_console_guard_init();
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
    (void)consoleReady; // 現状は起動ログ以外に使わない（将来の分岐用に残す）
#endif

    /* --- uwb_log タスクを起動する（コア0、docs/ARCHITECTURE_V2.md §3.2） --- */
    g_logTaskArgs.service = &service;
    g_logTaskArgs.table    = &table;
    g_logTaskArgs.bootUs   = bootUs;

    const BaseType_t logTaskOk =
        xTaskCreatePinnedToCore(&uwbLogTask, "uwb_log", static_cast<uint32_t>(CONFIG_UWB_TAG_LOG_TASK_STACK),
                                 &g_logTaskArgs, kLogTaskPrio, nullptr, 0);
    if (logTaskOk != pdPASS) {
        ESP_LOGE(TAG, "uwb_log タスクの起動に失敗しました。JSON Lines / 診断ログは出ません");
    }

    /* --- uwb_net（Wi-Fi + ブラウザダッシュボード + 無線コンソール）を起動する ---
     * UWBの初期化・測位開始が済んだ後に呼ぶ（uwb_net.hpp 冒頭コメント）。
     * 失敗してもUSBシリアルでの運用・測位そのものは継続するので、ログだけ
     * 出して先へ進む。 */
    {
        uwb::net::Config netCfg;
        netCfg.role                    = uwb::net::Role::Tag;
        netCfg.name                     = "uwb-tag";
        netCfg.addr                     = "tag0";
        netCfg.aggregate                = true;  // アンカーからのUDPを自分の配信へ混ぜる
        netCfg.forward                   = false; // タグ自身はUDPで送らない
        netCfg.httpPort                 = static_cast<uint16_t>(CONFIG_UWB_NET_HTTP_PORT);
        netCfg.consolePort              = static_cast<uint16_t>(CONFIG_UWB_NET_CONSOLE_PORT);
        netCfg.udpPort                   = static_cast<uint16_t>(CONFIG_UWB_NET_UDP_PORT);
        netCfg.highRateMinIntervalMs = static_cast<uint32_t>(CONFIG_UWB_NET_HIGHRATE_MIN_INTERVAL_MS);
        netCfg.statusFn                 = &tagNetStatus;
        netCfg.statusUser               = nullptr;
        bootBreadcrumb(2);
        const esp_err_t netErr = uwb::net::start(netCfg);
        if (netErr == ESP_OK) {
            bootBreadcrumb(3);
        }
        if (netErr != ESP_OK) {
            ESP_LOGE(TAG, "uwb_net の起動に失敗しました (err=%s)。JSON出力・測位は継続します",
                     esp_err_to_name(netErr));
        }

        // stage 4 (got_ip): IP_EVENT_STA_GOT_IP。イベントループは
        // uwb::net::start() が（成功時は）作成済み。CONFIG_UWB_NET_ENABLE=n
        // やWi-Fi起動失敗でイベントループが無い場合は登録自体が失敗するが、
        // ログのみで続行する（測位・他の起動処理は止めない）。
        const esp_err_t ipHandlerErr =
            esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &bootBreadcrumbGotIpHandler, nullptr);
        if (ipHandlerErr != ESP_OK) {
            ESP_LOGW(TAG, "breadcrumb: IP_EVENT_STA_GOT_IP ハンドラの登録に失敗しました (err=%s、続行します)",
                     esp_err_to_name(ipHandlerErr));
        }
    }

    // セットアップはここまで。測距・測位（uwb_ranging_svc タスク）・
    // JSON出力（uwb_log タスク）・コンソール（REPLタスク）・uwb_net の
    // 各タスクは、それぞれ自前のタスクで動き続けるので、app_main は
    // return してよい（ESP-IDF は app_main のタスクスタックを回収する）。
    ESP_LOGI(TAG, "setup complete: uwb_ranging_svc + uwb_log tasks running");
}
