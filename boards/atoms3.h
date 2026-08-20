/**
 * @file atoms3.h
 * @brief Board pin definition for M5 AtomS3 (ESP32-S3FN8, no PSRAM) as the
 *        host board driving a Qorvo QM33120W/DW3720 (M5Stack Stamp UWB-F)
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
 * この 6 本を SCK/MOSI/MISO/CS/RST/IRQ に全て割り当てる。
 * 出典: https://docs.m5stack.com/en/core/AtomS3
 * ※ この内訳は実物で要確認（docs/SOLDER_PADS.md の未確認事項リスト参照）
 *
 * - SCK/MOSI/MISO/CS: 底面 6 ピンヘッダの空き GPIO
 * - RST/IRQ: 本来 Grove ポートの I2C (SDA/SCL) に使われる G1/G2 を転用。
 *   これにより Grove I2C は使用不可になる（トレードオフとして明記）。
 *
 * WAKEUP と GP7 は UWB_PORT_PIN_UNUSED（未配線）とする。理由:
 * AtomS3 のクリーンな空き GPIO 6 本は SCK/MOSI/MISO/CS/RST/IRQ で使い切って
 * おり、残る物理ピンは G38/G39 のみだが、これらはオンボード IMU(MPU6886)
 * の I2C バスと共用（かつ底面ヘッダにも同じ信号が出ている）ため転用は
 * 安全でない。機能面での支障は無い: 本モジュールの最小配線要件は
 * GND/VCC/CLK/MOSI(CDI)/MISO(CDO)/CS のみであり、本ドライバはポーリング
 * 方式で動作するため IRQ 自体も必須ではない（AtomS3 では一応 IRQ は G2 に
 * 配線している）。WAKEUP 未配線時は uwb_port_wakeup_device_with_io() が
 * CS パルスによるフォールバック経路を使う
 * （components/uwb_port/src/uwb_port.c 実装済み、追加対応不要）。
 */
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
