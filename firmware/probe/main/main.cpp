/**
 * @file main.cpp
 * @brief Bring-up acceptance test suite for the Qorvo QM33120W/DW3720
 *        (M5Stamp UWB Module) over components/uwb_port and components/
 *        uwb_qm33120.
 *
 * Originally (main.c) this covered only L1 (raw SPI DEV_ID read) and L2
 * (dwt_probe()+dwt_readdevid()), both at the slow 2 MHz SPI rate, and never
 * called uwb::Qm33120::begin(). That left docs/HANDOFF.md SS1's "not yet
 * verified on real hardware" list - RSTn(G6) / IRQ(G7) / WAKEUP(G8)
 * continuity, and the 16 MHz fast SPI rate - completely untested. This file
 * extends the same binary into a full board bring-up checklist (L1-L11),
 * each logging an explicit PASS / FAIL / SKIP, ending in one grep-able
 * summary line.
 *
 * Board selection: Kconfig choice UWB_PROBE_BOARD (see
 * main/Kconfig.projbuild), default M5StampS3A. Pin definitions live in
 * boards/stamps3.h / boards/atoms3.h / boards/stampfly.h at the repo root.
 *
 * --- Test order and why (L1 -> L11 in the order they run, NOT in numeric
 *     order - L11 runs before L10 because L10 puts the chip to sleep and
 *     L11 needs to read the post-begin() state first; see the L11 comment
 *     and its call site in app_main() below) ---
 *   L1  raw SPI DEV_ID read (slow SPI, no driver state needed)
 *   L2  dwt_probe() + dwt_readdevid() (slow SPI; this is what makes the
 *       static `dw` pointer in deca_compat.c valid - see the L3 comment)
 *   L3  low-speed SPI write/readback consistency
 *   L4  RSTn functional check (chip must still answer afterwards)
 *   L5  uwb::Qm33120::begin() - full init, PHY config, PLL cal, IRQ self-test
 *   L6  16 MHz (fast) SPI stability, 1000 reads
 *   L7  part/lot ID + calibration dump (record only)
 *   L8  TX smoke test
 *   L9  3s ambient RX scan (record only)
 *   L11 DGC (RX gain calibration) OTP-vs-SW path dump (record only) - runs
 *       here, right after L9, for the reason given above
 *   L10 WAKEUP pin check (opt-in, Kconfig UWB_PROBE_TEST_WAKEUP)
 * L3/L4 call the vendored SDK's raw register accessors directly (same way
 * L1/L2 already did before this file existed), which only work once L2's
 * dwt_probe() has assigned a live driver to the SDK's internal `dw` pointer
 * (components/qm33120w_sdk/deca_compat.c). If L2 fails, L3/L4 are SKIPped
 * rather than attempted, to avoid dereferencing a null driver table (see
 * the guard around their call sites in app_main() below). L5 is always
 * attempted regardless of L2's outcome: uwb::Qm33120::begin()/probe() check
 * dwt_probe()'s return value themselves before touching driver state, so
 * calling it again is safe even if L2 already failed.
 *
 * 元は main.c で、L1（生 SPI での DEV_ID 読み）と L2（dwt_probe() +
 * dwt_readdevid()）のみを低速 2MHz SPI で行い、uwb::Qm33120::begin() は
 * 一度も呼んでいなかった。そのため docs/HANDOFF.md §1 が「未確認」と挙げる
 * RSTn(G6) / IRQ(G7) / WAKEUP(G8) の導通と 16MHz SPI が一切検証できて
 * いなかった。本ファイルは同じバイナリを基板立ち上げ時の機能チェック一式
 * （L1〜L11）へ拡張する。各検査は PASS / FAIL / SKIP を明示し、最後に
 * grep しやすい1行のサマリへまとめる。
 */
#include <cstdio>
#include <cstring>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "deca_device_api.h"
#include "deca_interface.h"
#include "uwb_port.h"
#include "uwb_qm33120.hpp"
#include "uwb_status_led.h"

/* L11 (DGC dump) needs raw register access below deca_device_api.h's public
 * surface: dwt_readfromdevice() (declared in deca_private.h, defined in
 * components/qm33120w_sdk/deca_compat.c) and the DGC_* / OTP_CFG_ID register
 * IDs plus the DWT_DGC_CFG0/1/2 and E0_CH5/CH9_DGC_LUT_* hardcoded-table
 * constants (both in components/qm33120w_sdk/dw3720/dw3720_deca_regs.h,
 * which itself pulls in dw3720_deca_vals.h). Both headers already wrap their
 * contents in `extern "C" { ... }` guards, so including them directly from
 * this .cpp compiles cleanly with no local re-declaration needed.
 * L11（DGC ダンプ）は deca_device_api.h の公開 API より下、
 * dwt_readfromdevice()（deca_private.h 宣言・deca_compat.c 定義）と
 * DGC_* / OTP_CFG_ID のレジスタ ID、DWT_DGC_CFG0/1/2 および
 * E0_CH5/CH9_DGC_LUT_* のハードコード表定数
 * （どちらも dw3720_deca_regs.h、内部で dw3720_deca_vals.h を取り込む）に
 * 生でアクセスする必要がある。両ヘッダとも extern "C" で囲まれているため、
 * この .cpp から直接 include しても再宣言なしに問題なくコンパイルできる。 */
#include "deca_private.h"
#include "dw3720_deca_regs.h"

/* Matches the Arduino reference's own top-of-file extern declaration
 * (M5Stamp_UWB.cpp) and components/uwb_qm33120/src/uwb_qm33120.cpp:
 * USE_DRV_DW3720 is not defined in this build so the extern in
 * deca_device_api.h is compiled out and we must declare it ourselves. In
 * C++ this symbol is defined by a C translation unit
 * (components/qm33120w_sdk), so it needs extern "C" linkage here or the
 * linker will look for a mangled C++ name and fail.
 * Arduino版リファレンスおよびuwb_qm33120.cppと同じ宣言。USE_DRV_DW3720が
 * このビルドでは定義されないため deca_device_api.h 内の extern が
 * コンパイルから外れ、ここで自前に宣言する必要がある。定義側はCの翻訳単位
 * (components/qm33120w_sdk)にあるため、C++からはextern "C"でリンケージを
 * 合わせないとリンカがマングルされた名前を探しに行って失敗する。 */
extern "C" {
extern const struct dwt_driver_s dw3720_driver;
}

#if CONFIG_UWB_PROBE_BOARD_ATOMS3
#include "boards/atoms3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_ATOMS3_UWB_PORT_CONFIG
#define BOARD_NAME            "AtomS3(pinout " BOARD_ATOMS3_PINOUT_NAME ")"
#elif CONFIG_UWB_PROBE_BOARD_STAMPFLY
#include "boards/stampfly.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPFLY_UWB_PORT_CONFIG
#define BOARD_NAME            "StampFly"
#else
#include "boards/stamps3.h"
#define BOARD_UWB_PORT_CONFIG BOARD_STAMPS3_UWB_PORT_CONFIG
#define BOARD_NAME            "M5StampS3A"
/* Only the M5StampS3A carries a WS2812 on a known GPIO; the AtomS3 has an
 * LCD instead, so the heartbeat is compiled out there.
 * 内蔵フルカラー LED を持つのは M5StampS3A のみ（AtomS3 は LCD）。
 * それ以外のボードではハートビート表示ごとコンパイルから外れる。 */
#define BOARD_STATUS_LED_GPIO BOARD_STAMPS3_STATUS_LED_GPIO
#endif

static const char *TAG = "uwb_probe";

#define UWB_DEV_ID_EXPECTED 0xDECA0314UL

#define PROBE_RETRY_COUNT    5
#define PROBE_RETRY_DELAY_MS 20

/* ===========================================================================
 * L1: raw SPI DEV_ID read (unchanged from the original main.c)
 * =========================================================================== */

/**
 * @brief L1: raw SPI DEV_ID read, mirrors readRawDeviceId() in the Arduino
 * reference. Header byte 0x00 selects register file 0 (DEV_ID), no
 * sub-index. Retries up to PROBE_RETRY_COUNT times if the value comes back
 * as 0x00000000 or 0xFFFFFFFF (both mean "no response" / "bus idle", not
 * necessarily a real mismatch).
 *
 * FAIL の意味: SPI 4本（SCK/MOSI/MISO/CS）のどれかが未接続・断線している
 * か、この最初の生読みの時点でチップが電源すら入っていない。dwt_probe()
 * などのSDK側状態に一切依存しないので、L2以降が全滅していてもL1だけは
 * 独立に切り分けに使える。
 */
static bool run_l1_raw_spi_check(struct dwt_spi_s *spi, uint32_t *out_dev_id)
{
    uint32_t raw_dev_id = 0;

    for (int attempt = 1; attempt <= PROBE_RETRY_COUNT; ++attempt) {
        uint8_t header = 0x00;
        uint8_t buf[4]  = {0};

        int32_t rc = spi->readfromspi(1, &header, sizeof(buf), buf);
        if (rc != DWT_SUCCESS) {
            ESP_LOGW(TAG, "L1: readfromspi failed (attempt %d/%d)", attempt, PROBE_RETRY_COUNT);
            vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
            continue;
        }

        raw_dev_id = ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];

        if ((raw_dev_id != 0x00000000UL) && (raw_dev_id != 0xFFFFFFFFUL)) {
            break;
        }

        ESP_LOGW(TAG, "L1: no response (0x%08lX) on attempt %d/%d, retrying", (unsigned long)raw_dev_id, attempt,
                 PROBE_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
    }

    *out_dev_id = raw_dev_id;

    bool ok = (raw_dev_id == UWB_DEV_ID_EXPECTED);
    if (ok) {
        ESP_LOGI(TAG, "L1: raw DEV_ID = 0x%08lX (expect 0x%08lX) -> PASS", (unsigned long)raw_dev_id,
                 (unsigned long)UWB_DEV_ID_EXPECTED);
    } else {
        ESP_LOGE(TAG, "L1: raw DEV_ID = 0x%08lX (expect 0x%08lX) -> FAIL", (unsigned long)raw_dev_id,
                 (unsigned long)UWB_DEV_ID_EXPECTED);
    }
    return ok;
}

/* ===========================================================================
 * L2: dwt_probe() + dwt_readdevid() (unchanged from the original main.c)
 * =========================================================================== */

/**
 * @brief L2: dwt_probe() + dwt_readdevid(), mirrors probe() in the Arduino
 * reference. On success this is also what assigns a live driver to the
 * SDK's internal static `dw` pointer (components/qm33120w_sdk/deca_compat.c)
 * - every later test that calls an SDK function directly (L3, L4) depends
 * on that having happened, which is why app_main() SKIPs them when this
 * returns false rather than calling them anyway.
 *
 * FAIL の意味: L1 は通ったがチップが応答プロトコル（Device ID の
 * ドライバ照合）に乗ってこない。SPI自体は疎通しているが極性/モード設定や
 * タイミングを疑う。
 */
static bool run_l2_dwt_probe_check(struct dwt_spi_s *spi, uint32_t *out_dev_id)
{
    struct dwt_driver_s *drivers[] = {(struct dwt_driver_s *)&dw3720_driver};
    struct dwt_probe_s probe_interface = {};
    probe_interface.dw                    = NULL;
    probe_interface.spi                   = spi;
    probe_interface.wakeup_device_with_io = uwb_port_wakeup_device_with_io;
    probe_interface.driver_list           = drivers;
    probe_interface.dw_driver_num         = sizeof(drivers) / sizeof(drivers[0]);

    for (int attempt = 1; attempt <= PROBE_RETRY_COUNT; ++attempt) {
        if (dwt_probe(&probe_interface) == DWT_SUCCESS) {
            uint32_t dev_id = dwt_readdevid();
            *out_dev_id     = dev_id;

            bool ok = (dev_id == UWB_DEV_ID_EXPECTED);
            if (ok) {
                ESP_LOGI(TAG, "L2: dwt_probe + dwt_readdevid = 0x%08lX (expect 0x%08lX) -> PASS",
                         (unsigned long)dev_id, (unsigned long)UWB_DEV_ID_EXPECTED);
            } else {
                ESP_LOGE(TAG, "L2: dwt_probe + dwt_readdevid = 0x%08lX (expect 0x%08lX) -> FAIL",
                         (unsigned long)dev_id, (unsigned long)UWB_DEV_ID_EXPECTED);
            }
            return ok;
        }
        ESP_LOGW(TAG, "L2: dwt_probe failed (attempt %d/%d)", attempt, PROBE_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
    }

    *out_dev_id = 0;
    ESP_LOGE(TAG, "L2: dwt_probe failed after %d attempts -> FAIL", PROBE_RETRY_COUNT);
    return false;
}

/**
 * @brief Re-run dwt_probe() (no retries beyond PROBE_RETRY_COUNT) and check
 * DEV_ID, for use after a hard reset (L4) or a sleep/wake cycle (L10) where
 * we already know the chip answered once and just need to confirm it still
 * does. Kept separate from run_l2_dwt_probe_check() so its log lines can
 * carry the L4/L10 prefix instead of "L2:".
 */
static bool reprobe_dev_id(const char *prefix, uint32_t *out_dev_id)
{
    struct dwt_driver_s *drivers[]     = {(struct dwt_driver_s *)&dw3720_driver};
    struct dwt_probe_s probe_interface = {};
    probe_interface.dw                    = NULL;
    probe_interface.spi                   = uwb_port_spi();
    probe_interface.wakeup_device_with_io = uwb_port_wakeup_device_with_io;
    probe_interface.driver_list           = drivers;
    probe_interface.dw_driver_num         = sizeof(drivers) / sizeof(drivers[0]);

    for (int attempt = 1; attempt <= PROBE_RETRY_COUNT; ++attempt) {
        if (dwt_probe(&probe_interface) == DWT_SUCCESS) {
            const uint32_t dev_id = dwt_readdevid();
            *out_dev_id            = dev_id;
            if (dev_id == UWB_DEV_ID_EXPECTED) {
                return true;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(PROBE_RETRY_DELAY_MS));
    }
    ESP_LOGE(TAG, "%s: re-probe failed after %d attempts (dev_id=0x%08lX)", prefix, PROBE_RETRY_COUNT,
             (unsigned long)*out_dev_id);
    return false;
}

/* ===========================================================================
 * L3 / L4: register write/readback, and the RSTn functional check
 * ===========================================================================
 *
 * 【どのレジスタで殴るかの選定・2箇所共通】
 * この2つの検査は「SPI 経由でチップのレジスタに書いた値が、読み戻して
 * そのまま返ってくるか」を土台にしている。したがって使うレジスタは
 *   (1) 書きも読みも本当に SPI トランザクションを発行すること
 *   (2) 書いてもチップの外側（ピン）に影響が出ないこと
 * の両方を満たす必要がある。
 *
 * ■ dwt_setxtaltrim() / dwt_getxtaltrim() は使えない（(1) を満たさない）
 *   dwt_setxtaltrim() 側は SPI へ書く（dw3720_device.c の ull_setxtaltrim()
 *   は LOCAL_DATA(dw)->init_xtrim を更新したうえで
 *   dwt_write8bitoffsetreg(XTAL_ID, ...) を発行する）が、
 *   **dwt_getxtaltrim() が読むのはチップではなくホスト側 RAM** である
 *   （ull_getxtaltrim() は `return LOCAL_DATA(dw)->init_xtrim;` の1行のみ）。
 *   deca_device_api.h の doc comment も「The value returned by this function
 *   is the initial value only!」と書いている。
 *   さらに deca_compat.c の dwt_probe() は静的な static_dw を毎回使い回すので、
 *   この RAM 値は L4 の RSTn 後の再プローブをまたいでも保持される。
 *   結果:
 *     - L3 に使うと「RAM に書いて同じ RAM を読む」往復になり、SPI の書き込み
 *       経路が死んでいても常に PASS する自明な検査になる（確証バイアスそのもの）。
 *     - L4 に使うと、RSTn が正しく配線されていても T2 == T1 のままになり
 *       **常に FAIL** する偽陰性になる。実機を疑わせて時間を失わせるだけの罠。
 *
 * ■ dwt_setgpiodir() / dwt_getgpiodir() も使わない（(2) を満たさない）
 *   こちらは (1) は満たす（GPIO_DIR_ID を SPI で読み書きする）が、
 *   **GPIO の入出力方向を変える＝チップのピンを出力に切り替える**。
 *   出力データレジスタの既定は 0 なので、そのピンが基板側で何かに駆動されて
 *   いると Low へ引きに行って喧嘩する。本ファームは「配線が分からない基板を
 *   最初に挿して素性を調べる」道具なので、素性の分からないピンを能動的に
 *   駆動する検査を既定で走らせるべきではない。
 *
 * ■ 採用: dwt_setrxantennadelay() / dwt_getrxantennadelay()（CIA_CONF_ID）
 *   (1) を満たす: ull_setrxantennadelay() は
 *       dwt_write16bitoffsetreg(CIA_CONF_ID, 0, v)、
 *       ull_getrxantennadelay() は dwt_read16bitoffsetreg(CIA_CONF_ID, 0)。
 *       どちらもホスト側キャッシュを経由せず実際に SPI を叩く。
 *   (2) を満たす: CIA_CONF はタイムスタンプ補正に使う純粋な内部レジスタで、
 *       ピンを一切駆動しない。値を変えても受信タイムスタンプの補正量が
 *       ずれるだけで、この時点では測距もしていない。しかも L5 の begin() が
 *       改めて dwt_setrxantennadelay(16385) を書き直すので、取りこぼしても
 *       実害が残らない。
 *   L4 に使える理由: チップ内レジスタそのものなので、RSTn が本当に効けば
 *       ハードウェアリセット既定値へ戻る。ホスト RAM は介在しない。
 *
 * いずれの関数も deca_device_api.h に実在する
 * （dwt_setrxantennadelay: :2018 / dwt_getrxantennadelay: :2025。
 *  後者の doc comment に "16-bit RX antenna delay value which is currently
 *  programmed in CIA_CONF_ID register" とある）。
 * ===========================================================================
 */

/**
 * @brief L3: low-speed SPI write/readback consistency, using CIA_CONF_ID (RX antenna delay)
 * (see the block comment above for why this register was chosen instead of
 * XTAL trim). Sequence: read T0, write T1 = T0 ^ 0x0F, read back and compare
 * to T1, then always restore T0 and verify that too.
 *
 * FAIL の意味: SPI の書き込み経路が壊れている（読みだけ通って書きが通らない
 * 配線・タイミング）。
 */
static bool run_l3_spi_writeback_check()
{
    const uint16_t t0 = dwt_getrxantennadelay();
    const uint16_t t1 = static_cast<uint16_t>(t0 ^ 0x0FFFU);

    dwt_setrxantennadelay(t1);
    const uint16_t readback = dwt_getrxantennadelay();
    const bool write_ok     = (readback == t1);

    /* 検査後は必ず元へ戻す。 Always restore, regardless of write_ok. */
    dwt_setrxantennadelay(t0);
    const uint16_t restored = dwt_getrxantennadelay();
    const bool restore_ok   = (restored == t0);

    if (write_ok && restore_ok) {
        ESP_LOGI(TAG, "L3: SPI write/readback OK (rx_antd T0=0x%04X T1=0x%04X readback=0x%04X restored=0x%04X) -> PASS",
                 (unsigned)t0, (unsigned)t1, (unsigned)readback, (unsigned)restored);
        return true;
    }
    if (!write_ok) {
        ESP_LOGE(TAG, "L3: readback mismatch (wrote 0x%04X, read back 0x%04X) -> FAIL (SPI write path)",
                 (unsigned)t1, (unsigned)readback);
    }
    if (!restore_ok) {
        ESP_LOGE(TAG, "L3: FAILED TO RESTORE rx_antd to 0x%04X (now reads 0x%04X) -> retry or power-cycle before continuing",
                 (unsigned)t0, (unsigned)restored);
    }
    return false;
}

/**
 * @brief L4: RSTn functional check (docs/HANDOFF.md's "not yet verified"
 * item). Writes a non-default CIA_CONF (RX antenna delay) value, pulses RSTn via
 * uwb_port_hard_reset(), re-probes, and checks whether the write survived.
 *
 * 判定:
 *  - T2 != T1 -> PASS（リセットでレジスタが既定へ戻った＝RSTn は実際に
 *    効いている）
 *  - T2 == T1 -> FAIL（書いた値が生き残った＝RSTn が繋がっていない。POR
 *    直後しかリセットできない）
 *
 * FAIL の意味: MCU だけ再起動したとき（再書き込み時など）チップの内部状態が
 * 残り、dwt_configure() が CONFIG_FAILED になる既知の症状
 * (uwb_qm33120_types.hpp の soft_reset_on_begin のコメントにある GOROman氏
 * の報告) を踏む。
 *
 * L4 のあとチップはリセット直後の状態（PHY 未設定・OTP 未読み込み）になる。
 * これは意図した状態であり、続く L5 の uwb::Qm33120::begin() が
 * hard_reset_on_begin=true によって改めて RSTn からやり直し、dwt_probe() /
 * dwt_initialise() / dwt_configure() を一から実行する。
 */
static bool run_l4_rstn_check(const uwb_port_config_t &port, uint32_t *out_dev_id_after)
{
    if (port.pin_rst == UWB_PORT_PIN_UNUSED) {
        ESP_LOGW(TAG, "L4: SKIP (pin_rst unwired)");
        return false;
    }

    /* この時点で CIA_CONF はまだ誰も書いていない（L3 は書いた値を戻している）
     * ので、t0 はハードウェアリセット既定値そのものである。したがって RSTn が
     * 効けばリセット後は t0 へ戻るはずで、「t1 でない」より強い
     * 「t0 に戻った」まで判定できる。
     * Nothing has written CIA_CONF yet (L3 restores what it wrote), so t0 IS
     * the hardware reset default. A working RSTn must therefore bring it back
     * to exactly t0, which is a stronger check than merely "not t1". */
    const uint16_t t0 = dwt_getrxantennadelay();
    const uint16_t t1 = static_cast<uint16_t>(t0 ^ 0x0FFFU);
    dwt_setrxantennadelay(t1);

    ESP_LOGI(TAG, "L4: wrote rx_antd T1=0x%04X, pulsing RSTn (pin=%d)...", (unsigned)t1, port.pin_rst);
    uwb_port_hard_reset(5, 100);

    if (!reprobe_dev_id("L4", out_dev_id_after)) {
        ESP_LOGE(TAG, "L4: chip did not answer after the RSTn pulse -> FAIL (cannot judge RSTn)");
        /* レジスタは書き戻せないかもしれないが試みる。
         * The register write may be unrecoverable, but try anyway. */
        dwt_setrxantennadelay(t0);
        return false;
    }

    const uint16_t t2 = dwt_getrxantennadelay();
    /* 判定後、レジスタを元へ戻しておく。L5 の begin() が RSTn からやり直す
     * ため厳密には不要だが、直後に何か失敗しても安全側に倒しておく。
     * Not strictly needed since L5's begin() resets from RSTn again, but
     * restoring anyway is the safe default if something goes wrong first. */
    dwt_setrxantennadelay(t0);

    const bool rstn_effective = (t2 == t0);
    if (rstn_effective) {
        ESP_LOGI(TAG, "L4: RSTn OK (wrote T1=0x%04X, after reset reads T2=0x%04X == T0) -> PASS", (unsigned)t1,
                 (unsigned)t2);
    } else if (t2 != t1) {
        /* t1 でも t0 でもない: リセットは効いたが既定値の想定が違う、または
         * 読みが化けている。断定せず WARN 止まりにして値を出す。
         * Neither t1 nor t0: the reset did something, but not what was
         * predicted. Report the values instead of asserting a verdict. */
        ESP_LOGW(TAG,
                 "L4: inconclusive (T0=0x%04X T1=0x%04X T2=0x%04X). リセットで値は変わったが T0 へは戻っていない",
                 (unsigned)t0, (unsigned)t1, (unsigned)t2);
    } else {
        ESP_LOGE(TAG,
                 "L4: RSTn FAILED (T2=0x%04X == T1=0x%04X, the write survived the reset pulse) -> pin_rst likely "
                 "unconnected; only a power cycle actually resets internal state",
                 (unsigned)t2, (unsigned)t1);
    }
    return rstn_effective;
}

/* ===========================================================================
 * L6: 16 MHz (fast) SPI stability
 * =========================================================================== */

/**
 * @brief L6: read dwt_readdevid() 1000 times at the fast SPI rate
 * (uwb::Qm33120::init() switches to Config::spi_fast_hz right after
 * dwt_configure() succeeds - see uwb_qm33120.cpp) and count mismatches.
 *
 * FAIL の意味: FPC + DIP変換 + ジャンパの配線長で 16MHz が持たない。
 * CONFIG_UWB_SPI_FAST_HZ（本ファームの main/Kconfig.projbuild）で速度を
 * 下げて再試験できる。
 *
 * 参考・データシート assets/QM33120W Data Sheet.pdf Rev.D p.25/p.26:
 * チップ側の上限は 32 MHz、t2(SCLK→出力) Max 9.5ns、t3(セットアップ)
 * Min 2.5ns。
 */
static bool run_l6_fast_spi_check(uint32_t iterations, uint32_t *out_bad_count)
{
    uint32_t bad = 0;
    for (uint32_t i = 0; i < iterations; ++i) {
        if (dwt_readdevid() != UWB_DEV_ID_EXPECTED) {
            bad++;
        }
    }
    *out_bad_count = bad;

    const bool ok = (bad == 0);
    if (ok) {
        ESP_LOGI(TAG, "L6: fast SPI (%lu Hz) stable over %lu reads, 0 mismatches -> PASS",
                 (unsigned long)uwb_port_spi_active_hz(), (unsigned long)iterations);
    } else {
        ESP_LOGE(TAG,
                 "L6: fast SPI (%lu Hz) UNSTABLE: %lu/%lu reads mismatched the expected DEV_ID -> FAIL. Lower "
                 "CONFIG_UWB_SPI_FAST_HZ and re-test.",
                 (unsigned long)uwb_port_spi_active_hz(), (unsigned long)bad, (unsigned long)iterations);
    }
    return ok;
}

/* ===========================================================================
 * L7: individual/calibration dump (record only, not a pass/fail judgement)
 * =========================================================================== */

/**
 * @brief L7: dump part ID / lot ID / xtal trim / die temp / vbat. This is a
 * record, not a judgement - only an obviously-broken xtal trim (0 or 0x7F,
 * suggesting the OTP could not be read) gets a WARN. Intended so that when
 * bringing up 2+ boards, these lines can be placed side by side to compare
 * individual units.
 *
 * 2 台以上を立ち上げるとき、この行を並べて比べるためのもの。
 */
static void run_l7_calibration_dump()
{
    const uint32_t partId = dwt_getpartid();
    const uint64_t lotId  = dwt_getlotid();
    const uint8_t xtal    = dwt_getxtaltrim();
    const uint16_t raw    = dwt_readtempvbat();
    const float tempC     = dwt_convertrawtemperature(static_cast<uint8_t>(raw >> 8));
    const float vbat      = dwt_convertrawvoltage(static_cast<uint8_t>(raw & 0xFFU));

    ESP_LOGI(TAG, "L7: partId=0x%08lX lotId=0x%010llX xtal_trim=%u(0x%02X) die_temp=%.1fC vbat=%.2fV -> rec",
             (unsigned long)partId, (unsigned long long)lotId, (unsigned)xtal, (unsigned)xtal, tempC, vbat);
    if ((xtal == 0U) || (xtal == 0x7FU)) {
        ESP_LOGW(TAG, "L7: xtal_trim=%u looks like an OTP read failure (0 or 0x7F) - treat calibration as suspect",
                 (unsigned)xtal);
    }
}

/* ===========================================================================
 * L8: TX smoke test
 * =========================================================================== */

/**
 * @brief L8: send a ~16 byte dummy frame with an immediate TX and check that
 * DWT_INT_TXFRS_BIT_MASK (frame sent) is observed within 20ms. No peer
 * needed - this only proves the local TX chain (PLL/TX/antenna switch)
 * runs, not that anything was received.
 *
 * FAIL の意味: 送信経路（PLL / TX / アンテナ切替）が動いていない。
 */
static bool run_l8_tx_smoke_test()
{
    static uint8_t txBuf[16] = {0xDE, 0xCA, 'p', 'r', 'o', 'b', 'e', '-', 'L', '8', '-', 't', 'x', 0x00, 0x00, 0x00};

    dwt_writetxdata(sizeof(txBuf), txBuf, 0);
    dwt_writetxfctrl(sizeof(txBuf), 0, 0);

    if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
        ESP_LOGE(TAG, "L8: dwt_starttx() returned DWT_ERROR -> FAIL");
        return false;
    }

    bool sent               = false;
    const int64_t deadline_us = esp_timer_get_time() + (20 * 1000);
    while (esp_timer_get_time() < deadline_us) {
        if ((dwt_readsysstatuslo() & (uint32_t)DWT_INT_TXFRS_BIT_MASK) != 0U) {
            sent = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    dwt_writesysstatuslo((uint32_t)DWT_INT_TXFRS_BIT_MASK);

    if (sent) {
        ESP_LOGI(TAG, "L8: TXFRS observed within 20ms -> PASS");
    } else {
        ESP_LOGE(TAG, "L8: TXFRS NOT observed within 20ms -> FAIL (TX chain not working)");
    }
    return sent;
}

/* ===========================================================================
 * L9: 3s ambient RX scan (record only)
 * =========================================================================== */

/**
 * @brief L9: open the receiver with both timeouts disabled and watch
 * dwt_readsysstatuslo() every 10ms for 3 seconds, OR-ing every status word
 * seen. Whenever a terminal RX event fires (good frame, timeout, or error),
 * clear it and re-arm dwt_rxenable() so the window keeps listening for the
 * rest of the 3 seconds instead of sitting idle after the first event.
 * Always ends with dwt_forcetrxoff() + a status clear.
 *
 * 判定はしない（記録のみ、常に PASS 扱いで値を出す）。相手機がいないのに
 * RXPRD が頻繁に立つなら、その帯域に何かいるか受信機が誤検出している。
 */
static void run_l9_rx_ambient_scan()
{
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    uint32_t observed = 0;
    (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);

    const int64_t end_us = esp_timer_get_time() + (3 * 1000 * 1000);
    while (esp_timer_get_time() < end_us) {
        const uint32_t st = dwt_readsysstatuslo();
        observed |= st;
        if ((st & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | (uint32_t)DWT_INT_RXFCG_BIT_MASK)) != 0U) {
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);
            (void)dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    dwt_forcetrxoff();
    dwt_writesysstatuslo(SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_RX_GOOD);

    static const struct {
        uint32_t mask;
        const char *name;
    } kBits[] = {
        {(uint32_t)DWT_INT_RXPRD_BIT_MASK, "RXPRD"},   // preamble detected / プリアンブル検出
        {(uint32_t)DWT_INT_RXSFDD_BIT_MASK, "RXSFDD"}, // SFD detected
        {(uint32_t)DWT_INT_RXPHE_BIT_MASK, "RXPHE"},   // PHY header error
        {(uint32_t)DWT_INT_RXSTO_BIT_MASK, "RXSTO"},   // SFD timeout
        {(uint32_t)DWT_INT_RXFCG_BIT_MASK, "RXFCG"},   // frame CRC good
    };
    char buf[96];
    size_t n = 0;
    buf[0]   = '\0';
    for (const auto &b : kBits) {
        if (((observed & b.mask) != 0U) && ((n + 8) < sizeof(buf))) {
            n += static_cast<size_t>(snprintf(buf + n, sizeof(buf) - n, "%s%s", (n != 0) ? "|" : "", b.name));
        }
    }
    if (buf[0] == '\0') {
        snprintf(buf, sizeof(buf), "none");
    }
    ESP_LOGI(TAG, "L9: 3s ambient RX scan observed=[%s] raw=0x%08lX -> rec (record only, no pass/fail)", buf,
             (unsigned long)observed);
}

/* ===========================================================================
 * L11: DGC (RX gain calibration) OTP-vs-SW path dump (record only)
 * ===========================================================================
 * Numbered L11 because it was added after L10, but it actually RUNS right
 * after L9 in app_main() - before L10, which puts the chip to sleep. L11
 * must read the post-begin() DGC state while it is still live, so it has to
 * come first; see the call site comment in app_main() below.
 * L10 の後に追加したため番号は L11 だが、実行順は L9 の直後（L10 より前）。
 * L10 はチップをスリープさせるため、begin() 直後の DGC 状態が生きている
 * うちに L11 で読んでおく必要がある。詳細は app_main() 内の呼び出し箇所の
 * コメントを参照。
 * =========================================================================== */

/**
 * @brief Read a 32-bit little-endian register via dwt_readfromdevice() (same
 * byte order as the SDK's own dwt_read32bitoffsetreg() - see
 * dw3720_device.c). Local to L11 only.
 */
static uint32_t l11_read_reg32(uint32_t regFileID)
{
    uint8_t buf[4] = {0, 0, 0, 0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
}

/**
 * @brief Read a 16-bit little-endian register via dwt_readfromdevice().
 * Local to L11 only (OTP_CFG_ID is a 32-bit register, but the bits L11
 * cares about - OTP_CFG_DGC_KICK_BIT_MASK=0x100 - all live in the low 16
 * bits, so 16 bits is enough per the task spec).
 */
static uint16_t l11_read_reg16(uint32_t regFileID)
{
    uint8_t buf[2] = {0, 0};
    dwt_readfromdevice(regFileID, 0, sizeof(buf), buf);
    return (uint16_t)(((uint16_t)buf[1] << 8) | (uint16_t)buf[0]);
}

/**
 * @brief L11: dump whether DGC (Digital Gain Control = the receiver's gain
 * calibration table) is running from per-unit OTP calibration or from a
 * generic hardcoded table, plus the raw register contents either way
 * (record only, no PASS/FAIL - like L7/L9). Background: TWR ranging
 * currently succeeds only ~22% of the time at 540mm, implying effective RX
 * sensitivity is ~30dB worse than spec (see docs/HANDOFF.md); the DGC path
 * was an unverified suspect and nobody had checked which path real hardware
 * takes.
 *
 * During dwt_initialise() the SDK (dw3720_device.c, ~line 967) reads OTP
 * word 0x20 and, ONLY if it equals DWT_DGC_CFG0 (0x00000240), trusts the
 * per-unit calibration baked into OTP at manufacture time ("path=OTP").
 * Otherwise it falls back to a hardcoded generic table for whichever
 * channel is active ("path=SW" - see ull_configmrxlut() in the same file).
 *
 * 読み方 / how to read the output:
 *  - path=OTP: この個体は OTP に工場較正された DGC 値を持ち、それを使って
 *    いる。 This unit has a factory DGC calibration in OTP and is using it.
 *  - path=SW: この個体には OTP 上の個体別 DGC 較正が **無く**、汎用
 *    ハードコード表（ch9 または ch5）のまま動作している。他個体のログと
 *    比較すること。 If path=SW, the chip has no per-unit DGC calibration in
 *    OTP and runs on generic table values instead; compare with the other
 *    unit.
 *  - WARN 行は内部矛盾（例: path=SW なのに LUT がハードコード表と違う、
 *    RX_TUNE イネーブルが落ちている等）を示す。RX_TUNE が落ちていれば、
 *    それだけで感度不足を直接説明できる。 The WARN lines flag internal
 *    inconsistencies; a clear RX_TUNE enable bit alone would directly
 *    explain poor sensitivity.
 *  - OTP_CFG の kick ビット (LDO/BIAS/DGC) は「1 を書くと OTP から
 *    レジスタへ転送し、その後自動で 0 に戻る」自己クリア型。初期化後に
 *    0 と読めるのは正常で、矛盾ではない。その裏付けとして LDO/BIAS の
 *    OTP 語も併記する: これらが非ゼロなら init が LDO/BIAS kick (0x600)
 *    を書いたはずで、それでも OTP_CFG が 0 なら自己クリアの実証になる。
 *    The OTP_CFG kick bits (LDO/BIAS/DGC) are self-clearing: writing 1
 *    transfers OTP data into the registers and the bit reads back 0
 *    afterwards. Reading 0 after init is normal, not an inconsistency.
 *    The LDO/BIAS OTP words are printed as supporting evidence: if they
 *    are non-zero, init must have written the LDO/BIAS kicks (0x600), and
 *    OTP_CFG still reading 0 demonstrates the self-clearing behaviour.
 *
 * @return true if this unit is on the OTP calibration path, false if it
 * fell back to the SW hardcoded table. Used only to annotate the summary
 * line token (L11=rec(dgc=OTP|SW)) - this is data, not a verdict.
 */
static bool run_l11_dgc_check()
{
    uint32_t otp[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    dwt_otpread(0x20U, otp, 8U);

    char otp_line[192];
    size_t n    = 0;
    otp_line[0] = '\0';
    for (int i = 0; i < 8; ++i) {
        n += static_cast<size_t>(snprintf(otp_line + n, sizeof(otp_line) - n, "%s0x%02X=0x%08lX", (i != 0) ? " " : "",
                                           (unsigned)(0x20 + i), (unsigned long)otp[i]));
    }
    ESP_LOGI(TAG, "L11: OTP[0x20..0x27] = %s", otp_line);

    /* Supporting OTP words: LDO tune lo/hi (0x04/0x05), bias tune (0x0A),
     * xtal trim (0x1E). Addresses as in dw3720_device.c (LDOTUNELO_ADDRESS,
     * LDOTUNEHI_ADDRESS, BIAS_TUNE_ADDRESS, XTRIM_ADDRESS).
     * 補助の OTP 語: LDO tune lo/hi (0x04/0x05)、bias tune (0x0A)、
     * 水晶トリム (0x1E)。番地は dw3720_device.c の定義と同じ。 */
    uint32_t otp_ldo_lo = 0, otp_ldo_hi = 0, otp_bias = 0, otp_xtrim = 0;
    dwt_otpread(0x04U, &otp_ldo_lo, 1U);
    dwt_otpread(0x05U, &otp_ldo_hi, 1U);
    dwt_otpread(0x0AU, &otp_bias, 1U);
    dwt_otpread(0x1EU, &otp_xtrim, 1U);
    ESP_LOGI(TAG, "L11: OTP ldo_tune_lo(0x04)=0x%08lX ldo_tune_hi(0x05)=0x%08lX bias_tune(0x0A)=0x%08lX xtrim(0x1E)=0x%08lX",
             (unsigned long)otp_ldo_lo, (unsigned long)otp_ldo_hi, (unsigned long)otp_bias, (unsigned long)otp_xtrim);

    const bool otp_path = (otp[0] == DWT_DGC_CFG0);
    ESP_LOGI(TAG, "L11: dgc_otp_tune=0x%08lX (expect 0x00000240 for OTP path) -> path=%s", (unsigned long)otp[0],
             otp_path ? "OTP" : "SW");

    const uint32_t dgc_cfg  = l11_read_reg32(DGC_CFG_ID);
    const uint32_t dgc_cfg0 = l11_read_reg32(DGC_CFG0_ID);
    const uint32_t dgc_cfg1 = l11_read_reg32(DGC_CFG1_ID);
    const uint32_t dgc_cfg2 = l11_read_reg32(DGC_CFG2_ID);
    const uint32_t lut0     = l11_read_reg32(DGC_LUT_0_CFG_ID);
    const uint32_t lut1     = l11_read_reg32(DGC_LUT_1_CFG_ID);
    const uint32_t lut2     = l11_read_reg32(DGC_LUT_2_CFG_ID);
    const uint32_t lut3     = l11_read_reg32(DGC_LUT_3_CFG_ID);
    const uint32_t lut4     = l11_read_reg32(DGC_LUT_4_CFG_ID);
    const uint32_t lut5     = l11_read_reg32(DGC_LUT_5_CFG_ID);
    const uint32_t lut6     = l11_read_reg32(DGC_LUT_6_CFG_ID);
    const uint32_t dgc_dbg  = l11_read_reg32(DGC_DBG_ID);
    const uint16_t otp_cfg  = l11_read_reg16(OTP_CFG_ID);

    ESP_LOGI(TAG, "L11: DGC_CFG=0x%08lX DGC_CFG0=0x%08lX DGC_CFG1=0x%08lX DGC_CFG2=0x%08lX", (unsigned long)dgc_cfg,
             (unsigned long)dgc_cfg0, (unsigned long)dgc_cfg1, (unsigned long)dgc_cfg2);
    ESP_LOGI(TAG, "L11: DGC_LUT_0..6 = 0x%05lX 0x%05lX 0x%05lX 0x%05lX 0x%05lX 0x%05lX 0x%05lX", (unsigned long)lut0,
             (unsigned long)lut1, (unsigned long)lut2, (unsigned long)lut3, (unsigned long)lut4, (unsigned long)lut5,
             (unsigned long)lut6);
    ESP_LOGI(TAG, "L11: DGC_DBG=0x%08lX OTP_CFG=0x%04X", (unsigned long)dgc_dbg, (unsigned)otp_cfg);

    const uint8_t dgc_decision =
        (uint8_t)((dgc_dbg & DGC_DBG_DGC_DECISION_BIT_MASK) >> DGC_DBG_DGC_DECISION_BIT_OFFSET);
    const bool rx_tune_en      = ((dgc_cfg & DGC_CFG_RX_TUNE_EN_BIT_MASK) != 0U);
    const bool otp_cfg_dgc_kick = ((otp_cfg & OTP_CFG_DGC_KICK_BIT_MASK) != 0U);
    ESP_LOGI(TAG, "L11: dgc_decision=%u rx_tune_en=%d otp_cfg_dgc_kick=%d", (unsigned)dgc_decision, rx_tune_en ? 1 : 0,
             otp_cfg_dgc_kick ? 1 : 0);

    const bool cfg_matches_hardcoded =
        (dgc_cfg0 == DWT_DGC_CFG0) && (dgc_cfg1 == DWT_DGC_CFG1) && (dgc_cfg2 == DWT_DGC_CFG2);

    /* begin() configures the default Channel9 (see uwb_qm33120.cpp), so the
     * expected SW-path table is the ch9 one; ch5 match is reported too since
     * seeing it would itself be a symptom of a channel mis-configuration.
     * begin() は既定で Channel9 を使う（uwb_qm33120.cpp 参照）ため、SW 側で
     * 期待されるのは ch9 表。ch5 に一致した場合もチャネル設定の不整合の
     * 兆候として報告する。 */
    const bool lut_matches_ch9 = (lut0 == (uint32_t)E0_CH9_DGC_LUT_0) && (lut1 == (uint32_t)E0_CH9_DGC_LUT_1) &&
                                  (lut2 == (uint32_t)E0_CH9_DGC_LUT_2) && (lut3 == (uint32_t)E0_CH9_DGC_LUT_3) &&
                                  (lut4 == (uint32_t)E0_CH9_DGC_LUT_4) && (lut5 == (uint32_t)E0_CH9_DGC_LUT_5) &&
                                  (lut6 == (uint32_t)E0_CH9_DGC_LUT_6);
    const bool lut_matches_ch5 = (lut0 == (uint32_t)E0_CH5_DGC_LUT_0) && (lut1 == (uint32_t)E0_CH5_DGC_LUT_1) &&
                                  (lut2 == (uint32_t)E0_CH5_DGC_LUT_2) && (lut3 == (uint32_t)E0_CH5_DGC_LUT_3) &&
                                  (lut4 == (uint32_t)E0_CH5_DGC_LUT_4) && (lut5 == (uint32_t)E0_CH5_DGC_LUT_5) &&
                                  (lut6 == (uint32_t)E0_CH5_DGC_LUT_6);
    const char *lut_class = lut_matches_ch9 ? "ch9-hardcoded" : (lut_matches_ch5 ? "ch5-hardcoded" : "other(OTP?)");

    ESP_LOGI(TAG, "L11: cfg0/1/2==DWT_DGC_CFG0/1/2(hardcoded)? %s, lut=%s", cfg_matches_hardcoded ? "yes" : "no",
             lut_class);

    /* The kick bits are self-clearing (see the section comment), so a clear
     * bit after init is the expected state on either path. Only a bit that
     * is still set is anomalous (a kick that never completed).
     * kick ビットは自己クリア型なので、初期化後に 0 なのはどちらの経路でも
     * 正常。立ったままなら異常（転送が完了していない）。 */
    if (otp_path && !otp_cfg_dgc_kick) {
        const bool ldo_bias_present = (otp_ldo_lo != 0UL) && (otp_ldo_hi != 0UL) && (otp_bias != 0UL);
        ESP_LOGI(TAG, "L11: path=OTP, DGC kick bit reads 0 - expected (self-clearing)%s",
                 ldo_bias_present ? "; LDO/BIAS kicks (0x600) were also written at init and read back 0, "
                                    "confirming self-clearing"
                                  : "");
    }
    if (otp_cfg_dgc_kick) {
        ESP_LOGW(TAG, "L11: OTP_CFG DGC kick bit is still SET after init - the OTP->register transfer may not have "
                      "completed");
    }
    if (!otp_path && !lut_matches_ch9) {
        ESP_LOGW(TAG, "L11: path=SW but LUTs do not match the hardcoded ch9 table (classified as %s) - unexpected",
                 lut_class);
    }
    if (!rx_tune_en) {
        ESP_LOGW(TAG, "L11: DGC_CFG RX_TUNE_EN bit is CLEAR - DGC is disabled, which would directly explain poor "
                      "RX sensitivity");
    }
    bool otp_all_zero = true;
    bool otp_all_ff    = true;
    for (int i = 0; i < 8; ++i) {
        if (otp[i] != 0x00000000UL) {
            otp_all_zero = false;
        }
        if (otp[i] != 0xFFFFFFFFUL) {
            otp_all_ff = false;
        }
    }
    if (otp_all_zero || otp_all_ff) {
        ESP_LOGW(TAG, "L11: all 8 OTP words are %s - looks like an OTP read failure",
                 otp_all_zero ? "0x00000000" : "0xFFFFFFFF");
    }

    ESP_LOGI(TAG, "L11: dgc path=%s lut=%s -> rec", otp_path ? "OTP" : "SW", lut_class);
    return otp_path;
}

/* ===========================================================================
 * L10: WAKEUP pin check (opt-in, default off)
 * =========================================================================== */

#if defined(CONFIG_UWB_PROBE_TEST_WAKEUP) && CONFIG_UWB_PROBE_TEST_WAKEUP
/**
 * @brief L10: put the chip to sleep, pulse WAKEUP for 1ms, wait 100ms, then
 * check dwt_readdevid(). The sleep is configured to wake ONLY on the
 * WAKEUP pin (DWT_WAKE_WUP), deliberately NOT on chip-select
 * (DWT_WAKE_CSN) - dwt_readdevid() itself drives CS, so including
 * DWT_WAKE_CSN would make a successful read ambiguous about which signal
 * actually woke the chip.
 *
 * FAIL 時は必ず uwb_port_hard_reset() -> dwt_probe() で復帰させる。復帰
 * できなければ ERROR。
 */
static bool run_l10_wakeup_check(const uwb_port_config_t &port)
{
    if (port.pin_wakeup == UWB_PORT_PIN_UNUSED) {
        ESP_LOGW(TAG, "L10: SKIP (pin_wakeup unwired)");
        return false;
    }

    ESP_LOGI(TAG, "L10: entering sleep (wake source = WAKEUP pin only, pin=%d)...", port.pin_wakeup);
    dwt_configuresleep(DWT_CONFIG, DWT_WAKE_WUP | DWT_SLP_EN);
    dwt_entersleep(DWT_DW_IDLE_RC);

    uwb_port_set_wakeup(true);
    vTaskDelay(pdMS_TO_TICKS(1));
    uwb_port_set_wakeup(false);
    vTaskDelay(pdMS_TO_TICKS(100));

    const uint32_t dev_id = dwt_readdevid();
    if (dev_id == UWB_DEV_ID_EXPECTED) {
        ESP_LOGI(TAG, "L10: dev_id=0x%08lX after WAKEUP pulse -> PASS", (unsigned long)dev_id);
        return true;
    }

    ESP_LOGE(TAG, "L10: dev_id=0x%08lX after WAKEUP pulse (expect 0x%08lX) -> FAIL, recovering via hard reset...",
             (unsigned long)dev_id, (unsigned long)UWB_DEV_ID_EXPECTED);

    uwb_port_hard_reset(5, 100);
    uint32_t recovered_dev_id = 0;
    if (reprobe_dev_id("L10", &recovered_dev_id)) {
        ESP_LOGW(TAG, "L10: recovered via uwb_port_hard_reset() + dwt_probe() (dev_id=0x%08lX)",
                 (unsigned long)recovered_dev_id);
    } else {
        ESP_LOGE(TAG, "L10: RECOVERY FAILED - device did not respond even after a hard reset. Power-cycle required.");
    }
    return false;
}
#endif

/* ===========================================================================
 * app_main
 * =========================================================================== */

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "UWB probe / board bring-up acceptance test, board=%s", BOARD_NAME);

    /* Start the heartbeat before anything else can fail, and note that the
     * blink runs in its own FreeRTOS task: it keeps going even when
     * app_main() returns early on an error below. A blinking LED therefore
     * means "the board booted and is running", not "the probe passed" - the
     * PASS/FAIL/SKIP verdicts are only in the log and the summary line.
     * 何かが失敗するより先に点滅を始める。点滅は専用の FreeRTOS タスクで
     * 走るので、下でエラー復帰して app_main() を抜けても点滅は続く。
     * つまり点滅は「起動して動作中」の意味であり、「疎通 OK」ではない。
     * 判定はログとサマリ行にしか出ない。 */
#ifdef BOARD_STATUS_LED_GPIO
    /* probe has no TAG/ANCHOR role, so it keeps the amber heartbeat.
     * probe は役割を持たないので琥珀色のまま。 */
    (void)uwb_status_led_start_role_heartbeat(BOARD_STATUS_LED_GPIO, UWB_STATUS_LED_ROLE_NONE);
#endif

    uwb_port_config_t port_cfg = BOARD_UWB_PORT_CONFIG;

    /* Kconfig 経由の低速化オプション（切り分け用）。既定 0 なら boards 配下の
     * 各ヘッダの値（16MHz）をそのまま使う。Overrides cfg.spi_fast_hz only when the
     * Kconfig value is non-zero; the default, 0, leaves it untouched. */
#if CONFIG_UWB_SPI_FAST_HZ > 0
    port_cfg.spi_fast_hz = CONFIG_UWB_SPI_FAST_HZ;
#endif
    ESP_LOGI(TAG, "spi_slow=%lu spi_fast=%lu", (unsigned long)port_cfg.spi_slow_hz,
             (unsigned long)port_cfg.spi_fast_hz);

    esp_err_t err = uwb_port_init(&port_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "uwb_port_init failed: %s", esp_err_to_name(err));
        return;
    }

    uwb_port_hard_reset(5, 100);

    /* Arduino 版リファレンス M5Stamp_UWB::begin() は hardReset() の直後に
     * wakeupDeviceWithIoImpl() を1回無条件に呼んでから ID 読み出しリトライに入る。
     * ハードリセット済みならスリープ状態にはいないはずだが、リファレンスの
     * シーケンスに合わせておく（L1 が不安定な場合の切り分けを減らすため）。 */
    uwb_port_wakeup_device_with_io();

    struct dwt_spi_s *spi = uwb_port_spi();

    /* --- L1 / L2 ------------------------------------------------------- */
    uint32_t l1_dev_id = 0;
    uint32_t l2_dev_id = 0;
    const bool l1_ok = run_l1_raw_spi_check(spi, &l1_dev_id);
    const bool l2_ok = run_l2_dwt_probe_check(spi, &l2_dev_id);

    /* --- L3 / L4 ---------------------------------------------------------
     * L2 が失敗した状態で L3/L4 を呼ぶと、SDK 内部の静的ドライバテーブル
     * (components/qm33120w_sdk/deca_compat.c の `dw`) が未確定のまま
     * dwt_setrxantennadelay() 等の関数ポインタ経由呼び出しへ入り、NULL 参照で
     * クラッシュ（再起動）しうる。L2 失敗時は SKIP して安全側に倒す。
     * Skip L3/L4 rather than risk a null-driver crash when L2 already
     * failed - see the SDK's static `dw` pointer note above L3/L4. */
    bool l3_ok = false;
    bool l4_ok = false;
    bool l4_skip = false;
    uint32_t l4_dev_id_after = 0;
    if (l2_ok) {
        l3_ok = run_l3_spi_writeback_check();
        l4_ok = run_l4_rstn_check(port_cfg, &l4_dev_id_after);
        l4_skip = (port_cfg.pin_rst == UWB_PORT_PIN_UNUSED);
    } else {
        ESP_LOGW(TAG, "L3/L4: SKIP (L2 dwt_probe failed - SDK driver state is not valid)");
    }

    /* --- L5: uwb::Qm33120::begin() ---------------------------------------
     * L4 の直後、チップは RSTn 直後の未初期化状態。begin() は
     * hard_reset_on_begin=true（既定）により自前で RSTn からやり直し、
     * dwt_probe() / dwt_initialise() / dwt_configure() を一から実行する。
     * port_already_initialized=true にして、上で既に呼んだ uwb_port_init()
     * を二重に呼ばないようにする。begin()/probe() は dwt_probe() の戻り値を
     * 自分で確認してから続けるので、L2 が失敗していても安全に呼べる。 */
    uwb::Qm33120 uwbDevice;
    uwb::Config cfg;
    cfg.spi_host          = port_cfg.spi_host;
    cfg.pin_sck            = port_cfg.pin_sck;
    cfg.pin_mosi           = port_cfg.pin_mosi;
    cfg.pin_miso           = port_cfg.pin_miso;
    cfg.pin_cs              = port_cfg.pin_cs;
    cfg.pin_rst             = port_cfg.pin_rst;
    cfg.pin_irq             = port_cfg.pin_irq;
    cfg.pin_wakeup          = port_cfg.pin_wakeup;
    cfg.pin_gp7             = port_cfg.pin_gp7;
    cfg.spi_slow_hz         = port_cfg.spi_slow_hz;
    cfg.spi_fast_hz         = port_cfg.spi_fast_hz;
    cfg.init_spi_bus        = port_cfg.init_spi_bus;
    cfg.port_already_initialized = true; // uwb_port_init() は上で呼び済み
    // cfg.use_irq は Config の構造体既定 (true) のまま使う。

    const bool l5_ok = uwbDevice.begin(cfg);

    char l5_note[32] = "";
    if (l5_ok) {
        ESP_LOGI(TAG, "L5: deviceId=0x%08lX chipName=%s isInitialized=%d irqActive=%d -> PASS",
                 (unsigned long)uwbDevice.deviceId(), uwbDevice.chipName(), uwbDevice.isInitialized(),
                 uwbDevice.irqActive());
        snprintf(l5_note, sizeof(l5_note), "(irq=%s)", uwbDevice.irqActive() ? "active" : "polling");
    } else {
        ESP_LOGE(TAG, "L5: begin() FAILED, error=%s -> FAIL. L6-L11 will be SKIPped.", uwbDevice.lastErrorName());
        snprintf(l5_note, sizeof(l5_note), "(%s)", uwbDevice.lastErrorName());
    }

    /* --- L6-L11: only meaningful once begin() succeeded ------------------ */
    bool l6_ok = false;
    uint32_t l6_bad = 0;
    bool l8_ok = false;
    bool l11_dgc_otp_path = false;
    bool l10_ok = false;
    bool l10_skip = false;
#if defined(CONFIG_UWB_PROBE_TEST_WAKEUP) && CONFIG_UWB_PROBE_TEST_WAKEUP
    const bool l10_enabled = true;
#else
    const bool l10_enabled = false;
#endif

    if (l5_ok) {
        l6_ok = run_l6_fast_spi_check(1000, &l6_bad);
        run_l7_calibration_dump();
        l8_ok = run_l8_tx_smoke_test();
        run_l9_rx_ambient_scan();
        /* L11 must run here, before L10: L10 puts the chip to sleep, and L11
         * reads the live post-begin() DGC state (see the L11 section comment
         * above for the full reasoning). Numbered L11 only because it was
         * added after L10 already existed.
         * L11 はここ、L10 より前に置く必要がある: L10 はチップをスリープ
         * させるため、L11 は begin() 直後の生きた DGC 状態を読む必要がある
         * （詳細は上の L11 セクションのコメント）。番号が L11 なのは単に
         * L10 が先に存在していたため。 */
        l11_dgc_otp_path = run_l11_dgc_check();
#if defined(CONFIG_UWB_PROBE_TEST_WAKEUP) && CONFIG_UWB_PROBE_TEST_WAKEUP
        l10_skip = (port_cfg.pin_wakeup == UWB_PORT_PIN_UNUSED);
        l10_ok    = run_l10_wakeup_check(port_cfg);
#else
        ESP_LOGI(TAG, "L10: SKIP (CONFIG_UWB_PROBE_TEST_WAKEUP disabled)");
#endif
    } else {
        ESP_LOGW(TAG, "L6-L11: SKIP (L5 begin() failed)");
    }

    /* --- summary ----------------------------------------------------------
     * grep しやすい1行。各トークンは PASS / FAIL / SKIP のいずれかで始まる。 */
    char l3_tok[8]  = "SKIP";
    char l4_tok[24] = "SKIP";
    char l5_tok[40];
    char l6_tok[24] = "SKIP";
    char l8_tok[8]  = "SKIP";
    char l10_tok[24] = "SKIP";
    char l11_tok[24] = "SKIP";

    if (l2_ok) {
        snprintf(l3_tok, sizeof(l3_tok), "%s", l3_ok ? "PASS" : "FAIL");
        if (l4_skip) {
            snprintf(l4_tok, sizeof(l4_tok), "SKIP");
        } else {
            snprintf(l4_tok, sizeof(l4_tok), "%s(RSTn %s)", l4_ok ? "PASS" : "FAIL", l4_ok ? "ok" : "broken");
        }
    }
    snprintf(l5_tok, sizeof(l5_tok), "%s%s", l5_ok ? "PASS" : "FAIL", l5_note);
    if (l5_ok) {
        snprintf(l6_tok, sizeof(l6_tok), "%s(%lu/1000 bad)", l6_ok ? "PASS" : "FAIL", (unsigned long)l6_bad);
        snprintf(l8_tok, sizeof(l8_tok), "%s", l8_ok ? "PASS" : "FAIL");
        snprintf(l11_tok, sizeof(l11_tok), "rec(dgc=%s)", l11_dgc_otp_path ? "OTP" : "SW");
        if (l10_enabled) {
            snprintf(l10_tok, sizeof(l10_tok), "%s", l10_skip ? "SKIP" : (l10_ok ? "PASS" : "FAIL"));
        }
    }

    ESP_LOGI(TAG,
             "=== PROBE SUMMARY L1=%s L2=%s L3=%s L4=%s L5=%s L6=%s L7=rec L8=%s L9=rec L10=%s L11=%s ===",
             l1_ok ? "PASS" : "FAIL", l2_ok ? "PASS" : "FAIL", l3_tok, l4_tok, l5_tok, l6_tok, l8_tok, l10_tok,
             l11_tok);

    /* --- periodic loop -----------------------------------------------------
     * Ongoing stability observation. If L5 succeeded, SPI is now at the
     * effective fast rate (spi_fast_hz, default 16MHz) - read dwt_readdevid()
     * once a second and keep a running mismatch count. If L5 never
     * succeeded (no live driver / still at the slow rate), fall back to the
     * original raw-SPI L1 read instead so this loop never dereferences a
     * driver that was never established.
     * begin() が成功していれば実効レート（既定16MHz）で、そうでなければ
     * 元の生SPI読みで、1秒ごとに安定性を見続ける。累積不一致件数も出す。 */
    uint32_t periodic_count = 0;
    uint32_t periodic_bad   = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        periodic_count++;

        if (l5_ok) {
            const uint32_t dev_id = dwt_readdevid();
            if (dev_id != UWB_DEV_ID_EXPECTED) {
                periodic_bad++;
            }
            ESP_LOGI(TAG, "periodic (fast, %lu Hz): DEV_ID = 0x%08lX, cumulative bad=%lu/%lu",
                     (unsigned long)uwb_port_spi_active_hz(), (unsigned long)dev_id, (unsigned long)periodic_bad,
                     (unsigned long)periodic_count);
            continue;
        }

        uint8_t header = 0x00;
        uint8_t buf[4]  = {0};
        int32_t rc      = spi->readfromspi(1, &header, sizeof(buf), buf);
        if (rc != DWT_SUCCESS) {
            periodic_bad++;
            ESP_LOGE(TAG, "periodic (raw, slow): readfromspi failed, cumulative bad=%lu/%lu",
                     (unsigned long)periodic_bad, (unsigned long)periodic_count);
            continue;
        }
        const uint32_t dev_id =
            ((uint32_t)buf[3] << 24) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[0];
        if (dev_id != UWB_DEV_ID_EXPECTED) {
            periodic_bad++;
        }
        ESP_LOGI(TAG, "periodic (raw, slow): DEV_ID = 0x%08lX, cumulative bad=%lu/%lu", (unsigned long)dev_id,
                 (unsigned long)periodic_bad, (unsigned long)periodic_count);
    }
}
