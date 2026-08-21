# 調査5: M5Stamp-UWB のライセンスと移植量 (2026-08-19)

対象: https://github.com/m5stack/M5Stamp-UWB @ `71d35e5` (2026-07-17)
クローン先: `third_party/M5Stamp-UWB`

## R1: ライセンス判定 → **クリア（vendoring 可）**

デュアル構成:
- ルート `LICENSE` = **MIT** (Copyright (c) 2026 M5Stack Technology CO LTD)
- `src/qm33120w_sdk/` 配下のみ **`LicenseRef-QORVO-2`**
  （条文全文: `src/qm33120w_sdk/LICENSES/LicenseRef-QORVO-2.txt`、Copyright (c) 2024 Qorvo US, Inc.、BSDライク6条）

| 論点 | 判定 |
|---|---|
| ソース再配布（改変版含む） | **許可**「Redistribution and use in source and binary forms, with or without modification, are permitted」＋条件1 |
| **Qorvo製IC限定条件** | **あり（重要）** 条件3「You may only use this software ... with an integrated circuit developed by Qorvo US, Inc. ... or any module that contains such integrated circuit」→ **別チップへ転用すると抵触** |
| 帰属表示の保持義務 | あり（条件1: ソース配布時、条件2: バイナリ配布時） |
| コピーレフト性 | **なし**（同一ライセンスでの再公開要求は無い） |
| 公開リポジトリへの vendoring | **条文上は可能**。M5Stack 自身が同じ形（SPDXヘッダ + `LICENSES/` + `THIRD_PARTY_LICENSES.md`）で公開している |

その他条件: 条件4=バイナリのリバースエンジニアリング禁止、条件5="Qorvo" 商標・名称の無断使用禁止（派生製品名含む）、条件6=改訂権は Qorvo のみ。

**遵守事項（本リポジトリの必須ルール）**
1. `src/qm33120w_sdk/` 由来の全15ファイル冒頭の SPDX ヘッダを**削除しない**
   ```
   SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.
   SPDX-License-Identifier: LicenseRef-QORVO-2
   ```
2. `LICENSES/LicenseRef-QORVO-2.txt` を同梱し続ける
3. `THIRD_PARTY_LICENSES.md` 相当でデュアル構成を明示する
4. **QM33120W/DW3720 以外のチップ向けに流用しない**

※法的助言ではなく条文読解。公開時に厳密を期すなら Qorvo への確認を推奨。

## 移植量（実測）
| 区分 | ファイル数 | 行数 | Arduino依存 |
|---|---|---|---|
| M5Stack wrapper (`src/` 直下, MIT) | 3 | **1,946** | **あり（全部ここに集約）** |
| Qorvo SDK (`src/qm33120w_sdk/`) | 15 | **21,630** | **ゼロ（grep実証）** |
| examples (MIT) | 4 | 637 | あり |
| 合計 | 22 | 24,213 | |

SDK内訳: `dw3720_device.c` 10,756 / `deca_device_api.h` 4,028 / `dw3720_deca_regs.h` 3,658 /
`deca_compat.c` 1,343 / `deca_interface.h` 700 / `dw3720_deca_vals.h` 350 / 他

**→ 行数の9割は Arduino 非依存でそのままコピー可能。実作業は wrapper 1,946行のみ。**

## Arduino 依存箇所と ESP-IDF 置換対応
すべて `src/M5Stamp_UWB.cpp` (1,558行) と `src/M5Stamp_UWB_Types.h` (258行) に集約。

| 依存API | 箇所 | ESP-IDF 置換 |
|---|---|---|
| `#include <Arduino.h>` | Types.h:10 | 削除 |
| `SPI.h`/`SPIClass*`/`SPISettings` | Types.h:11,28,41 / cpp:45,454,457,1489-1529 | `driver/spi_master.h` |
| `spi->transfer()` バイトループ | cpp:1492,1495,1513,1516,1519 | `spi_device_polling_transmit` で1トランザクション化（**現状より高速化余地あり**） |
| `pinMode()` ×7 | cpp:434,438,441,445,448,595,598 | `gpio_config()` |
| `digitalWrite()` ×15 | cpp:422,425,435,442,506,509,596,1490,... | `gpio_set_level()` |
| `delay()` ×約29 | cpp 各所 | `vTaskDelay(pdMS_TO_TICKS())` |
| `delayMicroseconds()` | cpp:27 | `esp_rom_delay_us()` |
| `millis()` ×約46 | タイムアウト計測 | `esp_timer_get_time()/1000` |
| `portENTER/EXIT_CRITICAL` | cpp:17,33,39 | **変更不要**（Arduino-ESP32 は ESP-IDF の FreeRTOS をそのまま公開） |
| `Serial.printf` | examples のみ | `ESP_LOGI` |
| `attachInterrupt`/`digitalRead` | **不使用** | 対応不要 |

## 移植の要: Qorvo SDK が要求する抽象化I/F
### (a) SPI 抽象（`deca_interface.h:474-543`）
```c
struct dwt_spi_s {
    int32_t (*readfromspi)(uint16_t headerLength, uint8_t *headerBuffer,
                           uint16_t readlength, uint8_t *readBuffer);
    int32_t (*writetospi)(uint16_t headerLength, const uint8_t *headerBuffer,
                          uint16_t bodyLength, const uint8_t *bodyBuffer);
    int32_t (*writetospiwithcrc)(uint16_t headerLength, const uint8_t *headerBuffer,
                                 uint16_t bodyLength, const uint8_t *bodyBuffer, uint8_t crc8);
    void (*setslowrate)(void);
    void (*setfastrate)(void);
};
```
### (b) グローバル関数（`extern "C"`、`dw3720_device.c` から直接呼ばれる）
```c
void deca_sleep(unsigned int time_ms);
void deca_usleep(unsigned long time_us);
decaIrqStatus_t decamutexon(void);
void decamutexoff(decaIrqStatus_t);
```
### (c) プローブ（`dwt_probe_s`）
`spi` ポインタ + `wakeup_device_with_io` 関数ポインタ + `driver_list` を渡して `dwt_probe()`

**→ uwb_port が実装すべきものはこれで確定。数百行規模。**

## 【重要】ライブラリは完全ポーリング方式
`attachInterrupt`/`detachInterrupt` はソース中 0件。`pin_irq` は `pinMode(INPUT)` のみで
`digitalRead` すら呼ばれない。`dwt_readsysstatuslo()` をループ監視する実装。
SDK の `dwt_isr`/`dwt_setcallbacks` もこの範囲では未使用。

**→ StampFly の GROVE 2系統案（IRQ 線が取れない）が、そのまま成立する。**

## 公開 API（`M5Stamp_UWB.h`）
```cpp
bool begin(const M5Stamp_UWBConfig& = {}, const M5Stamp_UWBPHYConfig& = {});
void end();  bool init(const M5Stamp_UWBPHYConfig& = {});
void hardReset(uint32_t reset_low_ms = 5, uint32_t startup_ms = 100);
uint32_t deviceId() const;  uint32_t readRawDeviceId();  const char* chipName() const;
M5Stamp_UWBTxResult sendFrame(...);  M5Stamp_UWBRxResult receiveFrame(...);
M5Stamp_UWBRangeResult       requestRange(const M5Stamp_UWBRangeConfig& = {});   // SS-TWR initiator
M5Stamp_UWBResponderResult   respondRange(const M5Stamp_UWBRangeConfig& = {});   // SS-TWR responder
M5Stamp_UWBDSRangeResult     requestDSRange(const M5Stamp_UWBDSRangeConfig& = {}); // DS-TWR initiator
M5Stamp_UWBDSResponderResult respondDSRange(const M5Stamp_UWBDSRangeConfig& = {});// DS-TWR responder
bool isConnected() const;  bool isInitialized() const;
M5Stamp_UWBError lastError() const;  const char* lastErrorName() const;
```
実装は PImpl（`struct Impl`）。SPI I/O・遅延・排他は `private static` メソッドで
Qorvo SDK にバインドされている。

### `M5Stamp_UWBConfig`（ピン/SPI。既定は Stamp C5 配線）
`spi`, `pin_cs=11`, `pin_sck=12`, `pin_miso=26`, `pin_mosi=27`, `pin_rst=25`,
`pin_irq=0`(未使用), `pin_wakeup=24`, `pin_gp7=23`,
`spi_slow_hz=2M`, `spi_fast_hz=16M`, `probe_retry_count=5`, `probe_retry_delay_ms=20`,
`begin_spi=true`, `hard_reset_on_begin=true`

### `M5Stamp_UWBPHYConfig`
`channel=Ch9`, `preambleLength=Len128`, `pacSize=Pac8`, `tx/rxPreambleCode=9`,
`sfdType=DW8`, `dataRate=Rate6M8`, `stsMode=Off`, `stsLength=64`, `pdoaMode=Off`,
`sfdTimeout=129`, `phrMode=0`, `phrRate=0`, `pgDelay=0x34`, `txPower=0xfefefefe`(自動),
**`txAntennaDelay=16385`, `rxAntennaDelay=16385`**, `enableLnaPa=true`

## TWR 実装（すべて `src/M5Stamp_UWB.cpp` 1ファイル）
| 方式 | 関数 | 行 |
|---|---|---|
| SS-TWR Initiator | `requestRange()` | 785–889 |
| SS-TWR Responder | `respondRange()` | 891–1009 |
| DS-TWR Initiator | `requestDSRange()` | 1010–1161 |
| **DS-TWR Responder（距離計算はここ）** | `respondDSRange()` | 1163–1377 |

Qorvo SDK 側に TWR 固有の計算関数は無く、**ToF 計算は全部 wrapper 層**。

### SS-TWR（cpp:848-863）
```c
pollTxTs = dwt_readtxtimestamplo32();      respRxTs = dwt_readrxtimestamplo32();
pollRxTs = get32le(&rawFrame[12]);         respTxTs = get32le(&rawFrame[16]);  // 相手が埋め込んだ値
rtdInit = respRxTs - pollTxTs;             rtdResp = respTxTs - pollRxTs;
tof = ((rtdInit - rtdResp) / 2.0) * DWT_TIME_UNITS;
distanceM = tof * speedOfLight;
```
### DS-TWR（cpp:1294-1319、非対称DS-TWR標準式）
```c
ra = respRxTs32 - pollTxTs32;   rb = finalRxTs32 - respTxTs32;
da = finalTxTs32 - respRxTs32;  db = respTxTs32  - pollRxTs32;
tofDtu = (ra*rb - da*db) / (ra + rb + da + db);
tof = tofDtu * DWT_TIME_UNITS;   distanceM = tof * speedOfLight;
```
**計算は Responder(Anchor) 側で実施**し、結果を "DWD" フレーム（距離mm を 4byte LE、
`resultRepeatCount` 回リピート送信）で Initiator(Tag) へ返す。
`requestDSRange()` は受信値をそのまま採用し再計算しない。

### 定数
- `DWT_TIME_UNITS = 1.0/499.2e6/128.0` ≈ 15.65 ps（`deca_device_api.h:59`）
- `speedOfLight = 299702547.0` m/s（真空中ではなく実効値。cpp:787, 1012, 1165 で**3箇所に重複定義**）

### タイムスタンプ
DW3720 のHWタイムスタンプは 40bit(5byte)。
`readTxTimestamp64()`/`readRxTimestamp64()` (cpp:298,305) が `dwt_read{tx,rx}timestamp(uint8_t[5])`
→ `get40le()` で 64bit 化。SS-TWR は差分のみで足りるため 32bit 下位版を使用。
DS-TWR は 40bit 取得後 uint32_t にキャストして差分（巻き戻り演算で正しく出る）。

### アンテナ遅延
- `init()` 内で `dwt_setrxantennadelay()`/`dwt_settxantennadelay()` によりレジスタへ書き込み、
  **タイムスタンプ読み出し時にハードウェアが自動補正**（cpp:550-551）
- 加えて wrapper が `_impl->tx_antenna_delay` をキャッシュし、**遅延送信の起動時刻から
  実際の TX タイムスタンプを算出する際に手動加算**（cpp:946, 1087, 1226）
  ※これは「遅延送信のトリガ時刻」と「アンテナから電波が出る時刻」の差を埋めるもので、
    HW 自動補正と二重計上ではない（DW3000 系の定石）
- **自動キャリブレーション機能は無い。既定値 16385 を使うだけ**
  → 既知距離での実測→手動調整が前提。Phase 3 で自前実装する

## examples 4本
全例共通の `initUwb()`（配線定義→Config→`begin()`→Device ID 確認）を持ち、
`setup()` は `Serial.begin(115200)`→1秒待機→`initUwb()`、`loop()` はロール別処理のみ。
**FreeRTOS タスクは使わずシングルスレッド。** ピン配置は4例とも Stamp C5 想定で同一。

| Example | ロール | loop() |
|---|---|---|
| SS_TWR_TAG | Initiator | 200ms間隔で `requestRange()` |
| SS_TWR_ANCHOR | Responder | 毎ループ `respondRange()` をブロッキング呼び出し |
| DS_TWR_TAG | Initiator | 200ms間隔で `requestDSRange()`、Anchor 計算済み距離を受信 |
| DS_TWR_ANCHOR | Responder | 毎ループ `respondDSRange()`、自ら距離計算して Tag へ返送 |

## 未解決の小さな謎
`dw3720/CMakeLists.txt` が参照する `uwb_driver` ターゲットはリポジトリ内に存在しない
（Qorvo 社内ビルドの残骸と推測。ESP-IDF 移植では無視してよい）
