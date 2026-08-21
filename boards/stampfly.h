/**
 * @file stampfly.h
 * @brief Board pin definition for StampFly (M5StampS3A を搭載したドローン
 *        機体) を **タグ(initiator)** として Qorvo QM33120W/DW3720
 *        (M5Stamp UWB Module) を載せる構成。components/uwb_port 経由で使用。
 *
 * 本リポジトリは StampFly から独立している（`docs/PLAN.md` §1）。
 * このファイルはその独立とは別に、**タグを StampFly へ移植するときの
 * 互換性を保つための基準配線**として位置づけられている。したがって
 * **このファイルが定める制約（GROVE 4本のみ、IRQ/RST 無し）が、
 * タグ側実装全体（`uwb_port` / `uwb_qm33120` / `firmware/tag`）が
 * 前提にしてよいハードウェアの上限を決めている**。GROVE 4本を超える
 * 配線や IRQ/RST を必要とする実装をタグ側の第一級経路に入れないこと
 * （出典: `docs/PLAN.md` §1）。
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
 * 対象ボード: StampFly（M5StampS3A を搭載したドローン機体本体。
 * third_party/stampfly_ecosystem の飛行制御ファーム(vehicle/, sf_board 等)
 * が同じ M5StampS3A 上で動いている点が boards/stamps3.h（単体の
 * M5StampS3A、飛行制御ファーム無し）との決定的な違い）。
 *
 * ■ StampFly の GPIO 使用状況（実コード確認済み、docs/STAMPFLY_INTEGRATION.md §5.2）
 * 出典: third_party/stampfly_ecosystem/firmware/vehicle/main/config.hpp:45-73
 *
 *   G0            ボタン
 *   G3 / G4       I2C SDA/SCL (INA3221, BMM150, BMP280, VL53L3)
 *   G5            モータ M4 (FL, CW)
 *   G7            ToF XSHUT（底面）
 *   G9            ToF XSHUT（前方）
 *   G10           モータ M3 (RL, CCW)
 *   G12           PMW3901 CS2
 *   G14 / G43 / G44  SPI MOSI / MISO / SCK（BMI270 + PMW3901 共用）
 *   G21           M5StampS3A 内蔵 LED
 *   G39           機体 LED
 *   G40           ブザー
 *   G41 / G42     モータ M2 (RR, CW) / M1 (FR, CCW)
 *   G46           BMI270 CS
 *
 * → **外部に取り出せる信号線は GROVE 2系統・計4本 (G13/G15/G1/G2) だけ**。
 *    これ以外の GPIO はすべて飛行に必須で流用できない。
 *
 * ■ 配線 (HW-1 案。docs/STAMPFLY_INTEGRATION.md §5.3)
 *   GROVE (RED  / I2C ): G13 = SDA        → SCK
 *                        G15 = SCL        → MOSI
 *   GROVE (BLACK/ UART): G1  = GROVE I(RX) → MISO
 *                        G2  = GROVE O(TX) → CS
 * 出典:
 * third_party/stampfly_ecosystem/firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md:120-130,
 * docs/STAMPFLY_INTEGRATION.md:758, docs/PLAN.md:10
 *
 * SPI 4本で GROVE を使い切るため RST/IRQ/WAKEUP/GP7 は配線しない
 * (docs/IRQ_POLICY.md「タグは IRQ 不使用。ポーリングで成立させる」)。
 * pin_wakeup 未配線時は CS パルスによるフォールバックが uwb_port.c に
 * 実装済みなので機能面での支障は無い。
 *
 * spi_host が SPI3_HOST である理由: 飛行系(BMI270/PMW3901)が SPI2_HOST を
 * G14/G43/G44/G46 で使用中のため(config.hpp:45-48 参照)。SPI3 が空いている
 * (docs/STAMPFLY_INTEGRATION.md:765)。**ESP32-S3 の SPI3(GPSPI3) は
 * IO_MUX 直結ピンを持たず必ず GPIO マトリクス経由になるが、16MHz では
 * 問題にならない**(docs/STAMPFLY_INTEGRATION.md:766)。
 *
 * init_spi_bus は true: SPI3_HOST は飛行系ファーム(sf_board)が初期化しない
 * バスなので、本ドライバが自前で spi_bus_initialize() する。
 * ※ 将来、下記「IRQ の別配線候補」で述べる HW-2 (SPI2_HOST へ相乗り)
 * 構成へ移行する場合は init_spi_bus=false + Config::port_already_initialized
 * = true にして、sf_board が起動時に1回だけ実行する spi_bus_initialize()
 * を使い回すこと(components/uwb_qm33120/include/uwb_qm33120_types.hpp:93、
 * docs/STAMPFLY_INTEGRATION.md §5.3 HW-2 表「バス初期化」行)。
 *
 * ■ pin_rst 未配線の影響
 * hard_reset_on_begin (components/uwb_qm33120/include/uwb_qm33120_types.hpp:83
 * 付近の Config::hard_reset_on_begin) は使えない。dwt_softreset だけで
 * 復旧できるかは **実機で要検証**
 * (docs/STAMPFLY_INTEGRATION.md §5.3 HW-1 表「RSTn」行:
 * 「取れない → hard_reset_on_begin が使えない。ソフトリセット
 * (dwt_softreset) だけで復旧できるか要検証」)。
 *
 * ■ 電源について（重要）
 * **GROVE のレールから電源を直接取ってはいけない。** StampFly の GROVE は
 * 電池電圧そのもの（満充電でも約4.35V。プロジェクト設計者による実機確認、
 * 2026-08-21）で、パッド2(VCC_3V3)はQM33120Wの電源レールに直結しているため
 * (docs/SOLDER_PADS.md §5.4)、絶対最大定格4.0Vを超えて壊れる恐れがある。
 * LDOによる降圧、または基板上の3.3Vレールからの給電が必要
 * (docs/STAMPFLY_INTEGRATION.md §5.3 HW-1 表「電源」行、docs/SOLDER_PADS.md §5.4)。
 *
 * ■ ケーブルについて
 * GROVE(RED) と GROVE(BLACK) は StampFly 基板上の別位置にあるため、市販の
 * 標準 GROVE ケーブル1本では配線できない。4本をまとめる**カスタムケーブルが
 * 必須**(docs/STAMPFLY_INTEGRATION.md §5.3 HW-1 表「配線」行)。
 *
 * ■ IRQ の別配線候補について（重要・誤読注意）
 * docs/HANDOFF.md には「別配線候補 G6/G8/G11」とあるが、この3本は
 * **StampFly 基板上で既に他の IC へ配線済みでコネクタに出ていない**ため、
 * そのままでは IRQ に転用できない:
 *   G6  = 底面 ToF (VL53L3CX) の INT
 *   G8  = 前方 ToF (VL53L3CX) の INT
 *   G11 = BMI270 の INT1
 * 出典:
 * third_party/stampfly_ecosystem/firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md:112-140,
 * docs/STAMPFLY_INTEGRATION.md:740-741
 * 「G6/G8 × VL53L3CX IC へ配線済み。コネクタに出ていない」
 * 「G11 × BMI270 へ配線済み」
 * **「現行ファームが GPIO を使っていない」ことと「基板上でどこにも
 * 繋がっていない」ことは別問題であり、前者だけを根拠に空きピン扱いしては
 * いけない**(docs/SOURCE_POLICY.md の運用ルール「フラグ・メタデータより、
 * 直接観測できる事実を優先する」と同じ誤りパターンなので注意)。
 *
 * タグで IRQ を取る現実的な経路は、docs/STAMPFLY_INTEGRATION.md §5.3 の
 * HW-2: SPI 3線を M5StampS3A のキャステレーション/基板上の SPI2 配線
 * (G44=SCK / G14=MOSI / G43=MISO) から分岐させ、空いた GROVE 4本を
 * CS / IRQ / RSTn / WAKEUP に回す構成である。値の例（未検証。あくまで
 * 参考として下にコメントアウトで示す。有効化する場合は spi_sck 等は
 * 上記の分岐元ピンを直接指すのではなく、実配線・実配線先の GPIO 番号を
 * 確認してから書くこと）:
 *
 *   // #define BOARD_STAMPFLY_HW2_UWB_PORT_CONFIG                       \
 *   //     {                                                            \
 *   //         .spi_host     = SPI2_HOST, // 飛行系(BMI270等)と共有     \
 *   //         .pin_sck      = 44, // G44 から分岐（sf_board 側と共用） \
 *   //         .pin_mosi     = 14, // G14 から分岐                      \
 *   //         .pin_miso     = 43, // G43 から分岐                      \
 *   //         .pin_cs       = 13, // G13 = GROVE(RED) SDA              \
 *   //         .pin_irq      = 15, // G15 = GROVE(RED) SCL              \
 *   //         .pin_rst      = 1,  // G1  = GROVE(BLACK) RX             \
 *   //         .pin_wakeup   = 2,  // G2  = GROVE(BLACK) TX             \
 *   //         .pin_gp7      = UWB_PORT_PIN_UNUSED,                     \
 *   //         .spi_slow_hz  = 2000000,                                 \
 *   //         .spi_fast_hz  = 16000000,                                \
 *   //         .init_spi_bus = false, // sf_board が既に初期化済みのバスに相乗り \
 *   //     }
 *
 * ただし **飛行制御の生命線である IMU(BMI270) と SPI バスを共有する
 * リスクがある**（UWB ドライバがバスをハングさせると IMU も止まる）ことに
 * 注意(docs/STAMPFLY_INTEGRATION.md §5.3 HW-2 表「リスク」行)。また
 * M5StampS3A のパッドへ物理的に半田付けアクセスできるかどうかも未確認
 * (同表「実現性」行)。**地上検証フェーズでは HW-1（本ファイルの既定値）を
 * 使い、HW-2 は飛行統合以降の検討課題とする**
 * (docs/STAMPFLY_INTEGRATION.md §5.3「推奨」表)。
 */
#define BOARD_STAMPFLY_UWB_PORT_CONFIG                                   \
    {                                                                    \
        .spi_host     = SPI3_HOST,                                      \
        .pin_sck      = 13, /* G13: GROVE(RED) SDA を SCK に転用. third_party/stampfly_ecosystem/firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md:120-130 */ \
        .pin_mosi     = 15, /* G15: GROVE(RED) SCL を MOSI に転用. 同上 */ \
        .pin_miso     = 1,  /* G1: GROVE(BLACK) GROVE I(RX) を MISO に転用. 同上 */ \
        .pin_cs       = 2,  /* G2: GROVE(BLACK) GROVE O(TX) を CS に転用. 同上 */ \
        .pin_rst      = UWB_PORT_PIN_UNUSED, /* GROVE 4本を SPI で使い切るため配線なし。上記「pin_rst 未配線の影響」参照 */ \
        .pin_irq      = UWB_PORT_PIN_UNUSED, /* 既定。docs/IRQ_POLICY.md「タグは IRQ 不使用」。上記「IRQ の別配線候補について」参照 */ \
        .pin_wakeup   = UWB_PORT_PIN_UNUSED, /* 同上。未配線時は CS パルスによるフォールバックが uwb_port.c に実装済み */ \
        .pin_gp7      = UWB_PORT_PIN_UNUSED, /* 同上。どのファームからも読まない(uwb_port.h の uwb_port_read_gp7() コメント参照) */ \
        .spi_slow_hz  = 2000000,                                        \
        .spi_fast_hz  = 16000000,                                       \
        .init_spi_bus = true, /* SPI3_HOST は sf_board が初期化しないため本ドライバが初期化する */ \
    }
