/**
 * @file stamps3.h
 * @brief Board pin definition for M5StampS3A (ESP32-S3FN8, no PSRAM) as the
 *        host board driving a Qorvo QM33120W/DW3720 (M5Stamp UWB Module)
 *        module via components/uwb_port.
 *
 * ============================================================
 * 暫定値。実配線に合わせて要変更。
 * ここに書かれているピン番号は未確定であり、実機の配線・シルク印刷・
 * テスターでの導通確認等で必ず検証してから使用すること。
 * ============================================================
 *
 * ■ 標準構成 (2026-08-22 確定)
 * **M5StampS3A を StampS3 BreakOut に載せ、UWB モジュール (S017-F) を
 * 0.5mm 12P FPC ケーブル + FPC→DIP 変換基板でヘッダに繋ぐ。**
 * タグ(据置)・アンカーとも同じ構成で、モジュール側の半田付けは不要になる。
 * 配線手順は docs/WIRING.md。
 *
 * BreakOut は M5StampS3A の 1.27mm ピンを 2.54mm へ変換するだけの基板で、
 * 露出 IO は 23 本 (G0-G15, G39-G44, G46)。下記の SPI 4 本と
 * RST/IRQ/WAKEUP はすべてこの中に含まれるので、**BreakOut を使っても
 * 使わなくてもピン定義は同一**である。
 * BreakOut には G0 と EN のタクトスイッチが載っているので、書き込みモード
 * への入り方が確実になる。
 * 出典: https://docs.m5stack.com/en/stamp/StampS3BreakOut
 *
 * ■ 注意: BreakOut の Grove ポートは G13/G15 に繋がっている
 * **G13 は下記の MISO と衝突する。** ToF センサ (docs/SURVEY_SPEC.md の
 * 高さ自動計測、オプション) を増設する場合、Grove に挿すことはできない。
 * 空いている G1-G5 あたりへ手配線すること。
 */
#pragma once

#include "uwb_port.h"

/*
 * 対象ボード: M5StampS3A（電源とアンテナが改良された現行版）。
 * **旧 M5StampS3（A 無し）と完全互換**なので、旧版でもこのピン定義がそのまま使える。
 *
 * M5StampS3A は 23 本の GPIO がヘッダ/パッドに露出しており、オンボード
 * 周辺機能との競合がほぼないため余裕を持ったピン配置にしている。
 *
 * SCK/MOSI/MISO/CS は ESP32-S3 の FSPI (SPI2_HOST) ネイティブ IO_MUX ピン
 * を使用（GPIO マトリクス経由より信号品質が良い）。出典:
 * https://docs.m5stack.com/en/core/M5StampS3A
 *
 * RST/IRQ/WAKEUP/GP7 はオンボード周辺機能を持たない空きヘッダ GPIO。
 */
#define BOARD_STAMPS3_UWB_PORT_CONFIG                                    \
    {                                                                    \
        .spi_host     = SPI2_HOST,                                      \
        .pin_sck      = 12, /* G12: FSPICLK (native FSPI SCLK), docs.m5stack.com/en/core/M5StampS3A */ \
        .pin_mosi     = 11, /* G11: FSPID   (native FSPI MOSI), docs.m5stack.com/en/core/M5StampS3A */ \
        .pin_miso     = 13, /* G13: FSPIQ   (native FSPI MISO), docs.m5stack.com/en/core/M5StampS3A */ \
        /* G10: FSPICS0 相当のピンだが、本ドライバは CS を素の GPIO として
         * ソフトウェア制御する（理由は uwb_port.c の uwb_spi_xfer 周辺コメント参照）。
         * したがってこのピンが FSPI ネイティブ CS であることには依存していない。 */ \
        .pin_cs       = 10, /* G10, docs.m5stack.com/en/core/M5StampS3A */ \
        .pin_rst      = 6,  /* G6: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/M5StampS3A */ \
        .pin_irq      = 7,  /* G7: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/M5StampS3A */ \
        .pin_wakeup   = 8,  /* G8: free header GPIO, no onboard-peripheral conflict, docs.m5stack.com/en/core/M5StampS3A */ \
        /* GP7 (モジュール pin 5) は本ドライバのどのファームからも読まない
         * (uwb_port_read_gp7() の呼び出し箇所が無い)。配線しないなら
         * UWB_PORT_PIN_UNUSED にしておく。GPIO 番号を入れたまま未配線だと
         * 入力が浮く。配線する場合のみ 9 に戻すこと (G9)。 */ \
        .pin_gp7      = UWB_PORT_PIN_UNUSED, \
        .spi_slow_hz  = 2000000,                                        \
        .spi_fast_hz  = 16000000,                                       \
        .init_spi_bus = true,                                           \
    }
