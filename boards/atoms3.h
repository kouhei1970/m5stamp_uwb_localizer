/**
 * @file atoms3.h
 * @brief Board pin definition for M5 AtomS3 (ESP32-S3FN8, no PSRAM) as the
 *        host board driving a Qorvo QM33120W/DW3720 (M5Stamp UWB Module)
 *        module via components/uwb_port.
 *
 * ============================================================
 * 暫定値。実配線に合わせて要変更。
 * ここに書かれているピン番号は未確定であり、実機の配線・シルク印刷・
 * テスターでの導通確認等で必ず検証してから使用すること。
 * ============================================================
 */
#pragma once

#include "uwb_port.h"

/*
 * M5 AtomS3 はオンボード周辺機能（LCD: G21/G17/G15/G33/G34/G16、
 * IMU(MPU6886) I2C: G38/G39、ボタン: G41）でほとんどの GPIO を消費しており、
 * 完全に空いている GPIO は実質 6 本のみ。内訳は 2 箇所に分かれている:
 *   - 底面 6 ピンヘッダ: G5 / G6 / G7 / G8
 *   - Grove ポート (HY2.0-4P): G1 / G2 (本来は I2C の SDA/SCL)
 * SPI (SCK/MOSI/MISO/CS) はどちらの構成でも共通してこの底面 6 ピンヘッダの
 * うち G5/G6/G7/G8 を使う。
 * 出典: https://docs.m5stack.com/en/core/AtomS3
 * ※ この内訳は実物で要確認（docs/SOLDER_PADS.md の未確認事項リスト参照）
 *
 * WAKEUP と GP7 はどちらの構成でも UWB_PORT_PIN_UNUSED（未配線）とする。
 * 機能面での支障は無い: 本モジュールの最小配線要件は
 * GND/VCC/CLK/MOSI(CDI)/MISO(CDO)/CS のみであり、本ドライバはポーリング
 * 方式で動作するため IRQ 自体も必須ではない。
 * WAKEUP 未配線時は uwb_port_wakeup_device_with_io() が CS パルスによる
 * フォールバック経路を使う（components/uwb_port/src/uwb_port.c 実装済み）。
 *
 * ============================================================
 * ■ ピン構成 A / B の切替（docs/IRQ_POLICY.md 22〜27行の表、2026-08-21 確定）
 * ============================================================
 * RST/IRQ と ToF(高さ自動計測用 I2C, docs/SURVEY_SPEC.md 参照) の割当を
 * 入れ替えた2通りの構成を Kconfig (CONFIG_UWB_BOARD_ATOMS3_PINOUT_B、
 * 各アプリの main/Kconfig.projbuild の choice UWB_BOARD_ATOMS3_PINOUT) で
 * 選べるようにしてある。マクロ BOARD_ATOMS3_UWB_PORT_CONFIG の中身は
 * このビルド設定で切り替わるが、呼び出し側（各 firmware アプリの
 * main.c(pp)）はマクロ名を変えずに済む。
 *
 * |        | SPI   | RST / IRQ    | ToF (I2C) | ToF の接続        | 推奨ボード   |
 * |--------|-------|--------------|-----------|-------------------|--------------|
 * | 構成A  | G5-G8 | Grove G1/G2  | G38/G39   | 底面へ手配線      | 無印 AtomS3  |
 * | 構成B  | G5-G8 | G38/G39      | Grove G1/G2 | Grove に挿すだけ | AtomS3R      |
 *
 * - **構成A（既定）**: RST/IRQ に Grove ポート (G1/G2、本来は I2C SDA/SCL)
 *   を転用し、ToF は底面ヘッダの G38/G39 へ手配線する。無印 AtomS3 では
 *   G38/G39 が IMU(MPU6886) の I2C と共用になるため、この構成でないと
 *   ToF が IMU と同じバスに相乗りしてしまう（本リポジトリは IMU を
 *   使わないため実害は無いが、念のため使わない前提のボード=無印を推奨）。
 * - **構成B**: RST/IRQ に底面ヘッダの G38/G39 を使い、ToF は Grove ポート
 *   (G1/G2) に挿すだけで済む。AtomS3R は IMU(BMI270)/地磁気(BMM150) が
 *   G0/G45 に移っているため G38/G39 が完全に専有できる
 *   （下記「無印 AtomS3 と AtomS3R の違い」参照）。
 *
 * 構成B の pin_rst=38 / pin_irq=39 は「底面ヘッダに G38/G39 が出ている」
 * 「AtomS3R では IMU が G0/G45 に移っているので G38/G39 を専有できる」の
 * 2点を根拠に割り当てた組み合わせであり、**G38 と G39 のどちらが RST で
 * どちらが IRQ かは実機のシルク印刷・導通確認による確認をまだ行っておらず
 * 未確認**。実物到着時に入れ替えが必要になる可能性がある。
 *
 * ■ 無印 AtomS3 と AtomS3R の違い（2026-08-21 ユーザ情報）
 * **AtomS3R が現行版。無印は在庫限り。**
 * | | 無印 AtomS3 | AtomS3R |
 * |---|---|---|
 * | IMU | MPU6886 @ **G38/G39** | BMI270 @ **G0/G45** |
 * | 地磁気 | なし | BMM150 @ **G0/G45** |
 * | G38/G39 | **IMU と共用**（I2C バスに相乗りする形になる） | **専用**（IMU は繋がっていない） |
 * | G0/G45 | — | **拡張ピンに出ていない** |
 *
 * 本リポジトリは **IMU も地磁気も使わない**ので、
 * **無印と R は互換と推定される**（G38/G39 が I2C として使える点は共通。
 * R ではむしろバスを専有できるので条件が良い）。
 * ただし **これは推定であり実機未検証**。実物到着時に
 * 底面ヘッダのピン配置を必ず確認すること。
 *
 * 想定: G38 = SDA, G39 = SCL（**実物で要確認**）
 */
#if CONFIG_UWB_BOARD_ATOMS3_PINOUT_B
/* 構成B: RST/IRQ = G38/G39（底面ヘッダ）、ToF は Grove(G1/G2) に挿すだけ。
 * AtomS3R 向け（無印では G38/G39 が MPU6886 と共用のため構成A を推奨）。 */
#define BOARD_ATOMS3_PINOUT_NAME "B"
#define BOARD_ATOMS3_UWB_PORT_CONFIG                                     \
    {                                                                    \
        .spi_host     = SPI2_HOST,                                      \
        .pin_sck      = 7,  /* G7: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_mosi     = 6,  /* G6: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_miso     = 5,  /* G5: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_cs       = 8,  /* G8: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_rst      = 38, /* G38: bottom header, AtomS3R では IMU が G0/G45 に移り専有できる。RST/IRQのどちらがG38かは実機未確認 */ \
        .pin_irq      = 39, /* G39: bottom header, 同上。RST/IRQのどちらがG39かは実機未確認 */ \
        .pin_wakeup   = UWB_PORT_PIN_UNUSED, /* no free GPIO left; see comment above (CS-pulse wakeup fallback) */ \
        .pin_gp7      = UWB_PORT_PIN_UNUSED, /* no free GPIO left; see comment above */ \
        .spi_slow_hz  = 2000000,                                        \
        .spi_fast_hz  = 16000000,                                       \
        .init_spi_bus = true,                                           \
    }
#else
/* 構成A（既定）: RST/IRQ = Grove(G1/G2)、ToF は底面ヘッダ G38/G39 へ手配線。
 * 無印 AtomS3 向け（G38/G39 が MPU6886 の I2C と共用のため、AtomS3R での
 * 構成B ほど綺麗ではないが、この構成なら無印でも動く）。 */
#define BOARD_ATOMS3_PINOUT_NAME "A"
#define BOARD_ATOMS3_UWB_PORT_CONFIG                                     \
    {                                                                    \
        .spi_host     = SPI2_HOST,                                      \
        .pin_sck      = 7,  /* G7: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_mosi     = 6,  /* G6: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_miso     = 5,  /* G5: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_cs       = 8,  /* G8: bottom 6-pin header, free GPIO, docs.m5stack.com/en/core/AtomS3 */ \
        .pin_rst      = 1,  /* G1: Grove port SDA repurposed as RST (sacrifices Grove I2C), docs.m5stack.com/en/core/AtomS3 */ \
        .pin_irq      = 2,  /* G2: Grove port SCL repurposed as IRQ (sacrifices Grove I2C), docs.m5stack.com/en/core/AtomS3 */ \
        .pin_wakeup   = UWB_PORT_PIN_UNUSED, /* no free GPIO left; see comment above (CS-pulse wakeup fallback) */ \
        .pin_gp7      = UWB_PORT_PIN_UNUSED, /* no free GPIO left; see comment above */ \
        .spi_slow_hz  = 2000000,                                        \
        .spi_fast_hz  = 16000000,                                       \
        .init_spi_bus = true,                                           \
    }
#endif
