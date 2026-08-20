/**
 * @file stamps3.h
 * @brief Board pin definition for M5Stamp S3 (ESP32-S3FN8, no PSRAM) as the
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
 * M5Stamp S3 は 23 本の GPIO がヘッダ/パッドに露出しており、オンボード
 * 周辺機能との競合がほぼないため余裕を持ったピン配置にしている。
 *
 * SCK/MOSI/MISO/CS は ESP32-S3 の FSPI (SPI2_HOST) ネイティブ IO_MUX ピン
 * を使用（GPIO マトリクス経由より信号品質が良い）。出典:
 * https://docs.m5stack.com/en/core/StampS3
 *
 * RST/IRQ/WAKEUP/GP7 はオンボード周辺機能を持たない空きヘッダ GPIO。
 */
#define BOARD_STAMPS3_UWB_PORT_CONFIG                                    \
    {                                                                    \
        .spi_host     = SPI2_HOST,                                      \
        .pin_sck      = 12, /* G12: FSPICLK (native FSPI SCLK), docs.m5stack.com/en/core/StampS3 */ \
        .pin_mosi     = 11, /* G11: FSPID   (native FSPI MOSI), docs.m5stack.com/en/core/StampS3 */ \
        .pin_miso     = 13, /* G13: FSPIQ   (native FSPI MISO), docs.m5stack.com/en/core/StampS3 */ \
        /* G10: FSPICS0 相当のピンだが、本ドライバは CS を素の GPIO として
         * ソフトウェア制御する（理由は uwb_port.c の uwb_spi_xfer 周辺コメント参照）。
         * したがってこのピンが FSPI ネイティブ CS であることには依存していない。 */ \
        .pin_cs       = 10, /* G10, docs.m5stack.com/en/core/StampS3 */ \
        .pin_rst      = 6,  /* G6: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/StampS3 */ \
        .pin_irq      = 7,  /* G7: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/StampS3 */ \
        .pin_wakeup   = 8,  /* G8: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/StampS3 */ \
        .pin_gp7      = 9,  /* G9: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/StampS3 */ \
        .spi_slow_hz  = 2000000,                                        \
        .spi_fast_hz  = 16000000,                                       \
        .init_spi_bus = true,                                           \
    }
