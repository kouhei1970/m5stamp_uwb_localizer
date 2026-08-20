# 調査3: M5Stamp UWB Module ハードウェア仕様 (2026-08-19)

## 結論（計画の要）
**M5Stamp UWB Module は SPI 直結専用。UART/AT コマンド方式は存在しない。**
コネクタは **HY2.0-4P GROVE ではなく 0.5mm ピッチ 12P FPC**。

## チップ
- **Qorvo QM33120W**（QM33120WTR13, WLCSP52, 3.1x3.5mm）
- IEEE 802.15.4-2020 / 802.15.4z-2020 (BPRF)、ch5/ch9 対応
  （M5Stamp UWB Module は **ch9 固定運用**、中心周波数 7987.2MHz）
- 公式ドライバソースに `dw3720_device.c` / `dw3720_deca_regs.h` があり、
  デバイスID は `0xDECA0314` を返す → **DW3720 系レジスタ体系を継承**
  （Qorvo は Decawave を買収。既存 DW3000 系の知見がそのまま効く）
- 回路図の一部シンボルに「DW3120」表記が残るが、実装部品は QM33120WTR13

## 通信インタフェース: SPI 一択（根拠4点）
1. 公式製品ページ "Communication Interface: SPI"
2. 回路図上、12P FPC に出ている信号は SPI/制御線と電源/GND のみ。**UART TX/RX が無い**
3. 公式 Arduino ライブラリのソースは `SPI.h` の低レベル通信のみ。
   `Serial` で AT コマンドを送受信するコードは**一切存在しない**
4. Qorvo データシート "SPI interface to host MCU supports rates up to 32 MHz"

### SPI 諸元
- チップ上限 32MHz
- 公式ライブラリ既定: **初期化時 2MHz → 通常動作 16MHz**（変更可）
- **SPI_MODE0 / MSBFIRST**

### 注意: 「Unit UWB」との混同厳禁
M5Stack の**別製品**「Unit UWB」(SKU U100) は UART+AT コマンド方式
（STM32F103 + Ai-Thinker BU01 = DW1000ベース、115200bps、`AT+switchdis=` 等）。
**M5Stamp UWB Module とは別チップ・別I/F。** uwb_localizer が対応する RYUW122 も AT 型。
→ 本件では **AT コマンド資産は一切使えない**。

## ピンアサイン

![公式ピンマップ](../assets/S017_Stamp_UWB_pinmap.jpg)

**重要: 半田パッドと FPC ケーブルでは信号の並びが違う。**
公式ピンマップの「側面ラベル」がパッド、「下部の番号付きの帯」が FPC ケーブルを表す。
混同すると配線を間違えるので、必ず使う側の表を見ること。

### (A) 半田パッド（キャステレーションホール）— **本プロジェクトはこちらを使う**

KiCad フットプリント（`docs/refs/m5_hardware/Stamp_UWB.kicad_mod`）の座標と
公式ピンマップの側面ラベルの両方で確認済み。

| Pin | 信号 | 用途 |
|---|---|---|
| 1 | GND | |
| 2 | 3V3 | **電源 3.3V 単一** |
| 3 | DW_WAKEUP | ウェイクアップ制御 |
| 4 | DW_IRQ | 割り込み出力 |
| 5 | DW_GP7 | チップGPIO7/SYNC 相当（予備） |
| 6 | DW_RSTn | リセット（負論理） |
| 7 | DW_CDO | SPI MISO |
| 8 | GND | |
| 9 | DW_CDI | SPI MOSI |
| 10 | DW_CSn | SPI CS |
| 11 | DW_CLK | SPI SCK |
| 12 | GND | |

**物理配置（番号は反時計回り）**
```
左列 上→下 = 1 2 3 4 5 6        右列 上→下 = 12 11 10 9 8 7
つまり pin 7 は pin 1 の対面ではなく pin 6 の対面
```
KiCad の実測値: pad1 (-5.9, -2.95) / pad6 (-5.9, 3.4) / pad7 (6.1, 3.4) / pad12 (6.1, -2.95)
（KiCad は +y が下向き）。ピッチ 1.27mm、列間 12.0mm。

### (B) FPC ケーブル（0.5mm 12P）— パッドとは並びが違う

公式ピンマップ下部の番号付きの帯。緑の矢印が「12 ← 1」の向きを示す。

| Pin | 信号 |
|---|---|
| 1 | 3V3 |
| 2 | 3V3 |
| 3 | GP7 |
| 4 | IRQ |
| 5 | WAKEUP |
| 6 | RSTn |
| 7 | GND |
| 8 | MISO |
| 9 | MOSI |
| 10 | CSn |
| 11 | GND |
| 12 | CLK |

**パッドは GND×3 / 3V3×1、FPC は GND×2 / 3V3×2** と本数も異なる。

**ホスト側に必要な信号線: 最低 4本 (SCK/MOSI/MISO/CS)。
推奨 +2本 (IRQ/RSTn)、省電力運用なら +1本 (WAKEUP)。**

### 公式配線例（ホスト = Stamp C5 の場合。ホスト側GPIOは基板依存で自由）
GP7=G23, IRQ=G0, WAKEUP=G24, RSTn=G25, MISO=G26, MOSI=G27, CSn=G11, SCK=G12
（ライブラリの `M5Stamp_UWBConfig` でピン番号は自由に変更可能）

## 電源
- **DC 3.3V 単一供給**（モジュール内に 3.3V→1.8V PMIC `JW5712` 搭載）。**5V 供給は不可/不要**
- 消費電流: スリープ 75.9uA / アンカー動作 5.23mA / **タグ動作 58.0mA** (@3.3V)
- TX 単独の瞬時ピーク値はドキュメント未記載

## 測距
- チップ仕様: SS-TWR / DS-TWR / TDoA / AoA 対応
- **ただし公式 Arduino ライブラリの実装・サンプルは SS-TWR と DS-TWR のみ**
  （examples は `SS_TWR_ANCHOR/TAG`, `DS_TWR_ANCHOR/TAG` の4つ。TDoA用APIは無い）
- 役割はソフトウェア的（`requestRange` 側=イニシエータ/タグ、`respondRange` 側=レスポンダ/アンカー）
- 個体識別: PAN ID + 16bit ショートアドレス
- 精度: DS-TWR で約 0.14m（M5Stack 自社測定）。チップ仕様は約10cm
- 最大距離 55m（見通し）、データレート 850kbps / 6.81Mbps
- 最大アンカー数・公式更新レート上限は**未公開**
  （サンプルの `RANGE_INTERVAL_MS = 200`(5Hz) はデモ設定であり仕様値ではない）

## 公式ライブラリ
- https://github.com/m5stack/M5Stamp-UWB （MIT。内部 Qorvo SDK 部のみ `LicenseRef-QORVO-2`）
- 既定ホストは Stamp C5 (ESP32-C5)。**Arduino 実装**
- 主API: `begin(config, phy)` / `hardReset()` / `deviceId()` / `chipName()` / `isConnected()`
  / `sendFrame()` / `receiveFrame()` / `requestRange()` / `respondRange()`
  / `requestDSRange()` / `respondDSRange()`
- `M5Stamp_UWBPHYConfig` でチャネル・プリアンブル長・PAC・SFD・データレート・STS・PDoA
  ・TX電力・**アンテナ遅延**を設定可能

## M5Stamp UWB Module (無印 S017) との違い
コネクタ実装の有無のみ。電気仕様・チップ・ピン配置・ライブラリは同一。
- S017: FPC 12P コネクタ未実装（パッドのみ）、$23.50
- **S017-F: FPC コネクタ実装済み + FPCケーブル同梱、$24.50**

## 出典
- https://docs.m5stack.com/en/stamp/Stamp_UWB_F
- https://docs.m5stack.com/en/arduino/projects/stamp/stamp_uwb
- https://github.com/m5stack/M5Stamp-UWB
- https://docs.m5stack.com/en/unit/uwb （別製品・混同注意）
- 回路図PDF: m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1262/SCH_UWB_MODULE_SCH_main_V0.2_*.pdf
- Qorvo QM33120W Product Brief: m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1262/QM33120W_Product_Brief.pdf
- https://github.com/m5stack/M5_Hardware/tree/master/Products/S017_Stamp_UWB/Footprint

## 本件への示唆（重大）
1. **uwb_localizer には QM33120W/DW3000 系のチップドライバも測距シーケンスも無い。**
   → レンジング層は M5Stack 公式 Arduino ライブラリ (+Qorvo SDK) を
     **ESP-IDF へ移植**するのが現実解。ゼロから書く必要はない。
2. 成果物の構造は「**M5Stamp-UWB 由来のレンジング層** + **uwb_localizer c/ 由来の測位層**」
3. **GROVE 1系統(2本)では SPI は張れない**（最低4本必要）。StampFly 接続は要再設計。
4. **電源は 3.3V。GROVE が 5V 供給なら降圧が必要。**
