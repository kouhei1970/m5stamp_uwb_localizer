# StampFly UWB ドライバ開発 進捗記録

対象:
- UWBモジュール: M5Stamp UWB Module (https://docs.m5stack.com/en/stamp/Stamp_UWB_F)
- 既存ドライバ資産: https://github.com/kouhei1970/uwb_localizer
- 最終ターゲット: https://github.com/M5Fly-kanazawa/stampfly_ecosystem
- 接続構想: StampFly の GROVE 端子を SPI に設定して接続

作業ディレクトリ: /Users/kouhei/tmp/github/m5stack_uwb

---

## 2026-08-19

### Phase 0: 事前調査（実施中）
- [x] 作業ディレクトリ確認（空ディレクトリ）
- [x] uwb_localizer をローカルにクローン
- [x] 周辺リポジトリのローカル存在確認
      - /Users/kouhei/tmp/github/stampfly_ecosystem (M5Fly-kanazawa)
      - /Users/kouhei/tmp/github/stampfly_hal (kouhei1970)
      - /Users/kouhei/tmp/github/M5StampFly (M5Fly-kanazawa)
- [x] 調査1: uwb_localizer のアーキテクチャ / 移植性 → docs/SURVEY_uwb_localizer.md
- [x] 調査2: stampfly_ecosystem のドライバ追加作法 / SPIバス構成 → docs/SURVEY_stampfly_ecosystem.md
- [x] 調査3: M5Stamp UWB Module ハード仕様（チップ・I/F・ピン・電源）→ docs/SURVEY_m5stamp_uwb_module.md
- [x] 調査4: StampFly GROVE 端子のGPIO割当と SPI 転用可否 → docs/SURVEY_stampfly_grove.md

### Phase 0 完了 (2026-08-19)
4件の調査すべて完了。結果は docs/SURVEY_*.md、計画は **docs/PLAN.md** に策定済み。

**判定: 実現可能。ただし当初構想からの修正が3点。**
1. uwb_localizer にチップドライバ/測距シーケンスは無い（測位層のみ）
   → レンジング層は M5Stack 公式 Arduino ライブラリを ESP-IDF へ移植して新規作成
2. M5Stamp UWB Module のコネクタは GROVE ではなく 0.5mm 12P FPC、電源は 3.3V 単一
   → FPC変換 + 5V→3.3V 降圧の小基板が必要
3. StampFly の GROVE 1系統(2本)では SPI 不可 → 2系統併用(4本)で IRQ 無しポーリング、
   または空きGPIO(G5/G10/G41/G42)へ直付け

### 追加調査（同日・完了）
- [x] 調査5: M5Stamp-UWB のライセンスと移植量 → docs/SURVEY_m5stamp_uwb_port.md
      - `third_party/M5Stamp-UWB` にクローン済み (@71d35e5)
      - **R1 クリア**: LicenseRef-QORVO-2 は改変込みソース再配布を許可。
        条件 = 著作権/SPDX表示の保持 + **Qorvo製IC限定使用**。vendoring 可
      - 移植量: 全24,213行のうち **21,630行(Qorvo SDK)は Arduino依存ゼロでそのままコピー可**。
        実作業は wrapper 1,946行のみ
      - **公式ライブラリは完全ポーリング方式（attachInterrupt 不使用）**
        → StampFly の GROVE 2系統案（IRQ線なし）がそのまま成立する

### 確定した前提（ユーザ回答 2026-08-19）
- 保有ハード: **M5Stamp UWB Module ×6 を購入済み**（2026-08-19 ユーザ指示で確定）
- **アンカー台数は固定しない。4台以上の任意台数に対応する**（同日ユーザ指示）
  - アンカー登録テーブル（アドレス + 3D座標 + 有効フラグ）駆動。**台数はテーブル長で決まる**
  - 下限は 3D で4台、2D（高さ既知）で3台
  - **1周で全アンカーから応答が返る前提にしない。有効測距が4件以上なら解く**という作りにする。
    欠測・タイムアウトは常態として扱い、4件未満の周期は「測位不能」を返して
    EKF(Lv3)側で予測のみ進める
  - 上限は `uwb_loc` の `UWB_MAX_ANCHORS`（既定16。RAM 節約で8へ下げる余地あり）
  - 同一平面配置は3Dが縮退する（鏡像解）。`uwb_anchors_coplanar()` で必ず検出する
  - 台数が多いほど冗長性・GDOP は改善するが、**1周の所要時間は台数に比例して伸びる**
    （4台ちょうどだと1台弾いた時点で解けないので、実質 Huber/χ²ゲートを効かせられない。
      5台以上あって初めて外れ値棄却が意味を持つ）
- 当面の検証構成は**タグ1 + アンカー5**（手持ち6台）
- ホストボードが6台あるかは未確認。4台あれば受入条件の下限は検証できる
- **懸念**: DS-TWR を N アンカー逐次で回すと1周が長い。原本サンプルの 200ms 間隔だと
  5台で1周1秒＝1Hz にしかならず飛行制御には不足。
  **Phase 2 完了時点で1リンクの所要時間を実測する**
- StampFly 統合先: **`firmware/vehicle`**
  - ※2026-08-19 ユーザ指摘により訂正。リモートを確認したところ
    旧 `vehicle_new` → 現 **`vehicle`**（統合対象）、旧 `vehicle` → `vehicle_old` にリネーム済み。
    ローカルクローンが 2026-05-09 時点で古く、調査2 が旧名を拾っていた。
    アーキテクチャに関する調査内容自体は現 `vehicle` にそのまま当てはまる。
    **Phase 6 着手前にローカルクローンの更新が必要**

### 方針改訂
- 当初「コアは C99」→ **層ごとに言語を使い分け**（PLAN.md §2 設計原則）。
  qm33120w_sdk=C原本 / uwb_port=C / uwb_qm33120,uwb_twr=C++ / uwb_loc=C99原本 / app=C++
  移植性は言語ではなく「ハード依存が uwb_port 1枚に閉じている」ことで担保する

---

## Phase 1: 基盤とSPI疎通（着手）
- [x] リポジトリ雛形（README, LICENSE, .gitignore, THIRD_PARTY_LICENSES.md）
- [x] ESP-IDF プロジェクト骨格（target esp32s3）
- [x] `components/qm33120w_sdk/` へ Qorvo SDK を SPDX ヘッダ保持のままコピー
- [x] `components/uwb_port/` I/F 設計と ESP-IDF 実装
      （dwt_spi_s の5関数 / deca_sleep / deca_usleep / decamutexon / decamutexoff / GPIO）
- [x] `boards/stamps3.h`, `boards/atoms3.h`
- [x] 立ち上げ手順を docs/BRINGUP.md に整備（配線・書き込み・切り分け表）
- [ ] **受入条件: M5StampS3A で Device ID 0xDECA0314 を読み出せる** ← **実機作業待ち（ブロック中）**
- [ ] 同じことを AtomS3 でもピン定義の差し替えだけで確認 ← 実機作業待ち

### Phase 1 実装完了 (2026-08-19) — ビルド通過
- ESP-IDF v5.5.2 でクリアビルド成功。`uwb_probe.bin` 0x40630 bytes、**警告0・エラー0**（自分で再現確認済み）
- Qorvo SDK 14ファイルを SPDX ヘッダ保持のまま `components/qm33120w_sdk/` へ vendoring、
  `LICENSES/LicenseRef-QORVO-2.txt` 同梱、Qorvo社内ビルド用 `dw3720/CMakeLists.txt` は削除して
  ESP-IDF 用 `CMakeLists.txt` を新規作成。警告抑制は不要だった
- `components/uwb_port/` 実装（432行）。SPI転送のバッファ境界は Arduino 版
  `readFromSpiImpl`/`writeToSpiImpl` (M5Stamp_UWB.cpp:1482-1524) と一致することを突き合わせ確認済み
  （readBuffer にはヘッダを除いたペイロードのみを返す）
- `firmware/probe/` : L1(生SPIでDEV_ID) / L2(dwt_probe→dwt_readdevid) の2段階で確認するアプリ

#### 検収で発見したバグ（**修正済み**）
**同一 CS ピンで `spi_bus_add_device()` を2回呼んでいた**（slow用/fast用）。
ESP-IDF の `spicommon_cs_initialize()`
(`esp_driver_spi/src/gpspi/spi_common.c`) は
`esp_rom_gpio_connect_out_signal(cs_io_num, spics_out[cs_num], ...)` を呼ぶため、
1つのGPIOには1つの出力信号しか繋がらない。2つ目のデバイス追加でピンが `spics_out[1]` に
張り替わり、**slow側(cs_num=0)のトランザクションでは CS が一度も Low にならない**。
slow レートは `dwt_probe()`/初期化で使う経路そのものなので、実機で必ず失敗する。
→ **CS をソフトウェア制御に変更**（Arduino 版リファレンスと同じ方式）。
   `spics_io_num = -1` にし、`spi_device_acquire_bus()` でバスを押さえてから
   `gpio_set_level()` で CS を叩く。バス共有時（StampFly の SPI2_HOST は BMI270/PMW3901 と共有）
   の安全性もこれで確保される。
   → 修正完了。`uwb_spi_xfer()` ヘルパに CS 制御を集約し、
     `readfromspi`/`write_impl` の両方がこれを通る。エラー経路でも CS は必ず High に戻る。
     WAKEUP ピン未配線時の「CSパルスで起こす」フォールバックも、
     ダミーSPIトランザクションによる代用をやめて素直な GPIO パルスに書き直した。

#### 検収で追加対応した点
- `firmware/probe/main/main.c`: Arduino 版 `begin()` は hardReset() 直後に
  `wakeupDeviceWithIo()` を1回無条件に呼ぶが、移植版は L1 の前に呼んでいなかった
  （L2 では `dwt_probe()` 経由で間接的に呼ばれる）。リファレンスのシーケンスに揃えた
- `boards/stamps3.h`: CS を「FSPI ネイティブ CS0」と説明するコメントが、
  ソフトウェアCS化後は誤解を招くため修正

#### 検収後の最終ビルド
クリアビルド成功、**警告0・エラー0**、`uwb_probe.bin` 0x40790 bytes（自分で再現確認済み）

#### ピン割当（暫定・実配線で要検証）
| | SCK | MOSI | MISO | CS | RST | IRQ | WAKEUP | GP7 |
|---|---|---|---|---|---|---|---|---|
| M5StampS3A | G12 | G11 | G13 | G10 | G6 | G7 | G8 | G9 |
| AtomS3 | G7 | G6 | G5 | G8 | G1 | G2 | 未配線 | 未配線 |

- M5StampS3A の SCK/MOSI/MISO/CS は ESP32-S3 の FSPI ネイティブ IO_MUX ピン（G12/G11/G13/G10）
- AtomS3 は空きGPIOが6本しかなく、RST/IRQ は Grove 端子(G1/G2)を転用。
  WAKEUP/GP7 は割り当て先が無く未配線（最小配線は GND/VCC/CLK/MOSI/MISO/CS のみで足り、
  ドライバはポーリング方式なので機能上は問題ない）
- 両ボードとも ESP32-S3FN8（PSRAM 非搭載）

### 2026-08-19 進捗
- `git init`（未 add/commit）、`.gitignore`、`LICENSE`（MIT, Kouhei Ito）、
  `THIRD_PARTY_LICENSES.md`、`README.md` を作成
- `components/qm33120w_sdk/` へ Qorvo SDK 14ファイルを `third_party/M5Stamp-UWB` から
  SPDX ヘッダ保持のまま `cp`（byte-identical 確認済み）。`dw3720/CMakeLists.txt`（Qorvo内製ビルド用）
  は除外し、ESP-IDF 用の新規 `CMakeLists.txt` を作成（`REQUIRES` なし、5 .c ファイルのみ）
- `components/uwb_port/` を実装（SPI/GPIO/時刻/排他の ESP-IDF 抽象層）。ビルド検証済み
- `boards/stamps3.h`, `boards/atoms3.h` を作成（ピン番号は暫定値、実機配線での検証が必要）
- `firmware/probe/main.c` に Phase 1 受入テスト本体を実装
  （L1: 生SPIでの DEV_ID 読み出し / L2: dwt_probe()+dwt_readdevid()、
  いずれも 5 回リトライ・20ms 間隔、以降 1 秒周期で L1 を再実行し安定性を確認）。
  ボード選択は Kconfig choice（`UWB_PROBE_BOARD_STAMPS3` 既定 / `UWB_PROBE_BOARD_ATOMS3`）
- `idf.py set-target esp32s3` → クリーン `idf.py build`（M5StampS3A 既定設定、AtomS3 選択の
  両方）で警告ゼロのビルド成功を確認
- **ビルド検証まで完了。実機での Device ID 読み出し確認は未実施
  （実機書き込み不可のため、次のセッションでの宿題）**

---

## 追加要件（2026-08-19 ユーザ指示）

**本リポジトリ (m5stack_uwb) は StampFly とは独立に、UWB による測位が完結して
できるものに仕上げる。** stampfly_ecosystem への移植は「本リポジトリの成果物を
利用する下流の統合作業」と位置づける。

### 設計上の含意
- コア（レンジング処理・測位演算・プロトコル）は StampFly 依存を一切持たない
- ハード依存はプラットフォーム抽象層（SPI/UART/時刻/ログ）に隔離し、差し替え可能にする
- 単体で動く成果物を用意する:
  - タグ側ファームウェア（M5Stamp UWB Module + 汎用 ESP32-S3 ボードで測位が動く）
  - アンカー側の設定 or ファームウェア
  - 測位結果を確認する手段（シリアル出力 / ホスト側ツール）
- stampfly_ecosystem へは「ESP-IDF コンポーネントとして取り込める形」で提供する
  （components/ 配下にそのまま置ける、または git submodule / managed component）

### 追加要件2（2026-08-19 ユーザ指示）
単体動作の想定ホストボードは **M5StampS3A / M5 AtomS3**（いずれも ESP32-S3）。
汎用DevKitではなく、この2種で動くことを前提に設計・検証する。
- M5StampS3A: Stamp 形状。GPIO はピンヘッダ/パッドに露出、GROVE端子あり
- AtomS3: GROVE(HY2.0-4P) 1系統 + 内部でLCD/ボタン等が GPIO を消費
- → 使えるGPIO本数の制約が両ボードで異なるため、**SPI(4線+IRQ+RST) が
  引けるか / UART(2線) で足りるか**が構成選択に直結する


---

## Phase 2: TWR 移植（着手 2026-08-19）
実機未着のため Phase 1 の実機確認はブロック中。ハード非依存で書ける Phase 2 を先行させる。
（TWR 層は `dwt_spi_s` 経由でしかハードに触らないので、L1/L2 の結果に関わらず成果は無駄にならない）

### 方針の細部（PLAN.md からの実装レベルの具体化）
- M5Stamp_UWB.cpp (1,558行) は1クラスに全機能が入っている。**クラス構造は保ったまま**
  移植し（移植リスク最小化）、ファイルだけ責務で分割する:
  - `components/uwb_qm33120/src/uwb_qm33120.cpp` … デバイス管理・PHY設定・フレーム送受信
  - `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` … SS/DS-TWR の4メソッド（同一クラス・別TU）
- 独立コンポーネント `uwb_twr` は当面作らない。マルチアンカーのスケジューリング層（Phase 4）を
  上位に置く形にする
- Phase 2 の検証ファームは `firmware/twr`（役割 tag/anchor × 方式 SS/DS を Kconfig で選択）。
  正式な `firmware/tag` / `firmware/anchor`（D2/D3）は Phase 4 で作る

### Step 1: デバイス層の移植（完了 2026-08-19）
- [x] `components/uwb_qm33120/` 新規。Arduino 型を排した設定/結果構造体
      (`uwb::Config`/`uwb::PhyConfig`/`uwb::FrameConfig`/`uwb::TxResult`/`uwb::RxResult`/`uwb::Error`)
- [x] begin / init / hardReset / deviceId / chipName / isConnected / PHY設定
- [x] sendFrame (2オーバーロード) / receiveFrame
- [x] ビルド通過（警告0・エラー0、`firmware/devtest` を stamps3/atoms3 × sender/receiver の
      全組み合わせでクリアビルド確認済み）
- [x] `firmware/probe` が壊れていないことを確認（stamps3/atoms3 ともクリアビルドで
      警告0・エラー0、`uwb_probe.bin` サイズも Phase 1 時点と同一の 0x40790 bytes）
- [x] Step 2 (TWR) 用ヘルパを前倒しで移植: `components/uwb_qm33120/src/uwb_qm33120_internal.hpp`
      に get16le/get32le/get40le/set16le/set32le, readTxTimestamp64/readRxTimestamp64,
      buildShortAddressFrame/parseShortAddressFrame/payloadMatches, rxStatusToError,
      stopRadioAndClear{Status,RxStatus,TxStatus,IoStatus}, kSpeedOfLightMPerS/kUusToDwtTime を収録
      （`static inline` として2TU間で共有可能な形にしてあるので、Step 2 は
      `src/uwb_qm33120_twr.cpp` を追加して `Qm33120::requestRange()` 等を生やすだけで済む）

主な意図的な変更点（詳細はコード中コメント参照）:
- `uwb::Config` は `SPIClass*` の代わりに `uwb_port_config_t` 相当のフィールドを直接持ち、
  `begin()` が `uwb_port_init()` を呼ぶ。`port_already_initialized` フラグで
  「既に他所が uwb_port を初期化済みなら触らない」を選べる（StampFly統合向け）
- 原本の per-instance SPI トランポリン(`readFromSpi`/`writeToSpi`/`setSlowRate`/`setFastRate`/
  `wakeupDeviceWithIo` 等の static メンバ)は無し。`uwb_port` が元からシングルトンで
  `struct dwt_spi_s` とwakeupパルスを実装済みのため不要
- `millis()` は `esp_timer_get_time()/1000` ベースの `nowMs()` に置換
  （`(nowMs()-start) < timeout` の unsigned wraparound-safe idiom は維持）

#### Step 1 の検収（メイン）
- ビルドを自分で再現確認（devtest 0x44f20 / probe 0x40790、いずれも警告0・エラー0）
- `begin()` の順序が原本と**行番号レベルで一致**していることを確認
  （GPIO/SPI初期化 → slowレート → hardReset → wakeup → 生SPIでID読みリトライ → probe → init）
- SPI の slow/fast 切替は原本でも `dwt_spi_s` の関数ポインタ経由で **Qorvo SDK 自身が呼ぶ**
  だけなので、`uwb_port_init()` が slow レートで返す現状で等価。
  移植版に明示呼び出しが無いのは正しい（原本 cpp:461 相当）

#### 検収でメインが入れた修正: **FreeRTOS tick 100Hz → 1000Hz**
移植元の Arduino-ESP32 は FreeRTOS tick が既定 1000Hz で、受信待ちループは `delay(1)` = 1ms 刻み。
ESP-IDF 既定の 100Hz のままだと `vTaskDelay(pdMS_TO_TICKS(1))` が **10ms 刻み**になり挙動が変わる。
特に TWR の遅延送信（`dwt_setdelayedtrxtime()` で指定した時刻までに送信を起動する必要がある）では
10ms もタスクが止まると送信ウィンドウを取り逃す。
→ `firmware/*/sdkconfig.defaults` に `CONFIG_FREERTOS_HZ=1000` を理由コメント付きで追加。
   probe / devtest とも再ビルドして反映確認済み（`CONFIG_FREERTOS_HZ=1000`）。
   Step 2 の `firmware/twr` にも引き継ぐこと。

### Step 2: TWR 層の移植（完了 2026-08-19）
- [x] `uwb::RangeConfig/DSRangeConfig/RangeResult/DSRangeResult/ResponderResult/DSResponderResult`
- [x] requestRange / respondRange（SS-TWR）
- [x] requestDSRange / respondDSRange（DS-TWR）
- [x] `firmware/twr` 検証ファーム（ボード × ロール(TAG/ANCHOR) × 方式(SS/DS) の3軸 Kconfig）
- [x] ビルド通過（警告0、4通り全部、AtomS3でも確認、devtest/probe も無事）

#### Step 2 の移植で死守すること（委譲指示に明記済み）
- ToF 計算式を変えない
  - SS: `rtdInit = respRxTs - pollTxTs`, `rtdResp = respTxTs - pollRxTs`,
    `tof = ((rtdInit - rtdResp) / 2.0) * DWT_TIME_UNITS`  (cpp:848-863)
  - DS: `ra=respRxTs32-pollTxTs32, rb=finalRxTs32-respTxTs32, da=finalTxTs32-respRxTs32,
    db=respTxTs32-pollRxTs32`, `tofDtu=(ra*rb-da*db)/(ra+rb+da+db)`  (cpp:1294-1319)
- 32bit/40bit の使い分け、符号付き/符号なしの別を厳密に守る。`double` を `float` にしない
- **アンテナ遅延の手動加算 (cpp:946, 1087, 1226) を落とさない**
  （遅延送信の起動時刻→実TXタイムスタンプの換算。チップ側の自動補正とは役割が別で二重計上ではない）
- フレームのバイト位置とエンディアン、遅延送信のタイミング計算（UUS→DTU）を変えない
- `speedOfLight = 299702547.0` は3箇所の重複を1本化するだけ。**値は変えない**
  （真空中の光速ではないので 299792458 に「直さない」）

#### Step 2 実装完了 (2026-08-19)
- `components/uwb_qm33120/include/uwb_qm33120_types.hpp`: `uwb::RangeConfig` /
  `DSRangeConfig` / `RangeResult` / `DSRangeResult` / `ResponderResult` /
  `DSResponderResult` を追加（約90行）。既定値は
  `M5Stamp_UWB_Types.h:184-258` の値をそのまま踏襲
- `components/uwb_qm33120/include/uwb_qm33120.hpp`: `requestRange`/
  `respondRange`/`requestDSRange`/`respondDSRange` の4メソッド宣言を追加
- `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` を新規作成（約470行）。
  4メソッドを `M5Stamp_UWB.cpp:785-1377` と1行ずつ突き合わせて移植。
  ToF計算式・32/40bit と符号の使い分け・アンテナ遅延の手動加算・フレームの
  バイト位置とエンディアン・遅延送信のタイミング計算は完全に同一。
  重複していた `speedOfLight`/`uusToDwtTime` は Step 1 で用意済みの
  `detail::kSpeedOfLightMPerS`/`detail::kUusToDwtTime` に一本化（値は不変）
- `components/uwb_qm33120/src/uwb_qm33120_impl.hpp` を新規作成。
  `struct Qm33120::Impl` の定義を `uwb_qm33120.cpp` から切り出して両TUで共有
  （下記「原本から意図的に変えた点」参照。Step2でuwb_qm33120.cppに加えた唯一の変更）
- `components/uwb_qm33120/CMakeLists.txt`: SRCS に `uwb_qm33120_twr.cpp` を追加
- `firmware/twr/`: `firmware/devtest` と同じ作りで新規作成
  （CMakeLists.txt, main/CMakeLists.txt, main/Kconfig.projbuild(3軸選択),
  main/main.cpp(約400行), sdkconfig.defaults(devtestからコピー)）。
  TAG は200ms間隔でレンジングし10回に1回、ANCHOR は毎ループブロッキング
  呼び出しで20回成功するごとに、成功回数/試行回数と距離の平均・標準偏差
  （Welfordのオンラインアルゴリズム）をログ出力する

##### ビルド結果
- `firmware/twr`: TAG/ANCHOR × SS/DS の4通り全部を `rm -rf build sdkconfig` →
  `set-target esp32s3` → `build` のクリアビルドで検証、**全て警告0・エラー0**。
  AtomS3設定（ANCHOR+SS, TAG+SS）でもクリアビルド成功、警告0
- `firmware/devtest`: M5StampS3A(SENDER)/AtomS3(RECEIVER) ともクリアビルド成功、警告0
  （uwb_qm33120.cppの変更を含めても壊れていないことを確認）
- `firmware/probe`: M5StampS3A/AtomS3 ともクリアビルド成功、警告0

##### 原本から意図的に変えた点
- **`uwb_qm33120.cpp` への唯一の変更**: `struct Qm33120::Impl` の定義
  （旧cpp:296-305相当、原本のcpp:43-54相当）を新規ファイル
  `uwb_qm33120_impl.hpp` に切り出した。原本は単一ファイルなので
  `struct M5Stamp_UWB::Impl` を呼び出し側と同じファイル内に直接書けたが、
  本移植はStep1/Step2を別TUに分割する設計（PROGRESS.md記載の当初方針通り）
  のため、Step2のTWRメソッドが `_impl->tx_sequence`/`_impl->tx_antenna_delay`/
  `_impl->initialized` に原本同様アクセスするには、Implの完全な定義を
  両TUから見える場所に置く必要があった。フィールド構成・既定値・コメントは
  一切変更していない（単純な置き場所の移動）
- `speedOfLight`/`uusToDwtTime` の3箇所重複を `uwb_qm33120_internal.hpp` の
  `detail::kSpeedOfLightMPerS`/`detail::kUusToDwtTime` に一本化（値は不変、
  委譲指示で明示許可されていた整理）
- millis()→`detail::nowMs()`, delay(N)→`vTaskDelay(pdMS_TO_TICKS(N))`
  （Step1と同じ wraparound-safe idiom。`CONFIG_FREERTOS_HZ=1000` を
  `firmware/twr/sdkconfig.defaults` にも設定済みなので粒度も同一）
- `firmware/twr/main/main.cpp` のコメントで `boards/*.h` や `examples/*.ino`
  のようなワイルドカード表記を使うと `-Werror=comment`
  （コメント内に偶然 `/*` という並びができる）でビルドが落ちるため、
  「boards 配下の」のような日本語の言い回しに変更した（実装ロジックとは無関係、
  コメント文言のみの回避）

##### 移植中に気づいた原本の設計上の引っかかり
- **DS-TWR は距離計算が Responder(Anchor) 側にあり、Initiator(Tag) は
  "DWD" フレームで送られてきた値をそのまま受け取るだけ**
  (`requestDSRange()` は再計算しない、cpp:1152-1153 / 移植先も同じ)。
  Phase 4 のマルチアンカー測位では、Tag 側で複数 Anchor との距離を集約して
  位置を解くのが自然な構成だが、DS-TWR ではその「距離」自体が Anchor 側の
  計算結果に依存する。つまり:
  - Anchor が距離を計算 → Result フレームで Tag に返す、という往復が
    Anchor の数だけ発生し、Tag 側のレンジング周期がその分律速される
    （SS-TWRならTag側で完結するのでこの往復が要らない）
  - Anchor 側にバグや個体差（アンテナ遅延未校正など）があっても、Tag 側は
    それを検証する術がない（送られてきた distanceMm を信じるしかない）。
    複数Anchorの結果を突き合わせて外れ値検出をするような仕組みを
    Phase 4で入れるなら、Anchor側の生タイムスタンプ（ra/rb/da/db相当）も
    一緒に返す拡張を検討する価値がある
  - 反対にSS-TWRはTag側で完結する（cpp:848-863、本移植も同じ）ため、
    マルチアンカー測位でもTag側の負荷が均一で、Anchor側の実装ミスが
    Tag側の距離計算そのものには波及しない（ただしSS-TWRはDS-TWRより
    クロックドリフトの影響を受けやすく精度は劣る、という一般的なトレードオフは残る）
- `respondRange()`(SS)は `readRxTimestamp64()` ヘルパを使わずタイムスタンプ
  読み出しを毎回インライン展開しているのに対し、`requestDSRange()`/
  `respondDSRange()`(DS)は同じ処理に `readTxTimestamp64()`/`readRxTimestamp64()`
  ヘルパを使っている、という原本内の不統一があった。移植では原本の
  この不統一もそのまま踏襲した（詳細はコード中コメント参照）。動作に影響はない

##### 実機が来たときに真っ先に確認すべき項目
1. `firmware/twr` を TAG/ANCHOR の2台に書き込み、SS-TWR から疎通確認
   （距離が読めること、ログの `SS_RANGE_STAT`/`SS_RESP_STAT` で成功率が
   出ること）。次に DS-TWR で同様に確認し、SS/DSで距離値が近いか比較する
2. 既知距離（例えば1m/2m/5m）で `mean_mm` を見て、アンテナ遅延の既定値
   16385 からのオフセットを実測 → Phase 3 の校正の初期値にする
3. `hostTimeoutMs=100`, `rxTimeoutUus=3000` 等のタイムアウト値がボードの
   実際のRTT（特にDS-TWRの4フレーム往復）に対して十分か、`elapsed_ms`
   ログで確認する（不足なら Kconfig で変えずに makeRangeConfig() の値を
   実測ベースで調整する運用になる想定）
4. ANCHOR側respondDSRange()の"DWD"結果フレームは3回リピート送信するが、
   Tag側は最初の1つを受け取れば成立を判定する実装（cpp:1141同等）。
   電波環境が悪い場所で resultRepeatCount/resultRepeatGapMs のチューニングが
   要るか確認する
5. M5StampS3A / AtomS3 それぞれのピン配置（`boards/stamps3.h`/`boards/atoms3.h`）
   がまだ実配線未検証（Phase 1 のブロック中の受入条件）なので、TWR以前に
   Device ID 読み出し自体をまず確認すること


---

## Phase 2 完了 (2026-08-19) — メイン検収済み

### 成果物
| ファイル | 行数 |
|---|---|
| `components/uwb_qm33120/include/uwb_qm33120_types.hpp` | 315 |
| `components/uwb_qm33120/include/uwb_qm33120.hpp` | 134 |
| `components/uwb_qm33120/src/uwb_qm33120.cpp` | 788 |
| `components/uwb_qm33120/src/uwb_qm33120_impl.hpp` | 52 |
| `components/uwb_qm33120/src/uwb_qm33120_internal.hpp` | 194 |
| `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` | 643 |
| `firmware/twr/main/main.cpp` | 434 |

`firmware/twr` は **ボード(stamps3/atoms3) × ロール(TAG/ANCHOR) × 方式(SS/DS)** の3軸 Kconfig。

### ビルド（メインが再現確認）
`firmware/twr` クリアビルド **exit=0 / 警告0・エラー0**、`uwb_twr.bin` 0x42220 bytes、
`CONFIG_FREERTOS_HZ=1000` が効いていることを確認。`firmware/devtest` / `firmware/probe` も無傷。

### 検収: 移植の忠実性（原本と1行突き合わせ）
| 項目 | 結果 |
|---|---|
| SS-TWR ToF 計算 (cpp:848-863) | **完全一致**。`rtdInit/rtdResp` の int32 キャスト、`<= 0` ガード、`/2.0`、`DWT_TIME_UNITS` すべて同一 |
| DS-TWR ToF 計算 (cpp:1294-1319) | **完全一致**。`ra/rb/da/db` の uint32 巻き戻し差分→double、`denominator <= 0` ガード、`(ra*rb-da*db)/denominator` すべて同一 |
| **アンテナ遅延の手動加算** | **3箇所すべて保持**（原本 cpp:946/1087/1226 → 移植版 twr.cpp:216/360/501）。式も `((respTxTime & 0xFFFFFFFE) << 8) + tx_antenna_delay` で同一 |
| `speedOfLight` | `299702547.0` のまま。3箇所の重複を `detail::kSpeedOfLightMPerS` に一本化しただけ |
| `uusToDwtTime` | `65536ULL` のまま（原本は4箇所に重複） |
| distanceMm の丸め | 原本と同じ `+0.5f/-0.5f` の符号付き丸め |
| "DWD" 結果フレーム | 距離mm を 4byte LE で埋める形式を維持 |
| double→float 化 | **していない**（ToF計算は double のまま。距離のみ float） |

### 移植で意図的に変えた点
- `struct Qm33120::Impl` を `uwb_qm33120.cpp` 内から `src/uwb_qm33120_impl.hpp` へ移動。
  TWR メソッドが別 TU になったため、`_impl->tx_sequence` / `tx_antenna_delay` / `initialized`
  へ原本と同じようにアクセスするために必要（定義内容は無変更）
- 定数の重複解消（上記）
- `millis()`→`detail::nowMs()`、`delay(N)`→`vTaskDelay(pdMS_TO_TICKS(N))`（Step 1 と同じ）

### Phase 4 に向けた申し送り（重要）
- **DS-TWR は距離計算が Anchor 側だけで行われ、Tag は "DWD" フレームの値を中継するだけ。**
  Tag 側で生タイムスタンプ (ra/rb/da/db) を検算できない。
  → マルチアンカー測位で外れ値検出をやりたいなら、
    結果フレームに生の ra/rb/da/db を載せる拡張を検討する余地がある
  → 各アンカーとの往復レイテンシがそのまま1周の時間に乗る（R6 に直結）
- 原本自体が SS-TWR と DS-TWR で `readRxTimestamp64()` ヘルパを使う/インライン展開する で
  不統一。移植版もそのまま保持している

### 実機が来たら真っ先にやること
1. Phase 1 の受入（Device ID `0xDECA0314`）— **ピン配線が未検証なのでここが最初の関門**
2. SS-TWR で2台リンク → 次に DS-TWR
3. 既知距離での `mean_mm` を取り、Phase 3 のアンテナ遅延キャリブレーションの種にする
4. `elapsed_ms` を `hostTimeoutMs` / `rxTimeoutUus` と突き合わせる
5. **1リンクあたりの所要時間を実測**（R6: N アンカー逐次の1周時間を決める根拠になる）


---

## Phase 4 地ならし（着手 2026-08-19）
実機未着のため、ハード非依存で進められる Phase 4 を先行させる。
Phase 3（アンテナ遅延キャリブレーション）は既知距離の実測が必須なので実機待ち。

### 事前整理
- `uwb_localizer` のクローンを リポジトリ直下 → `third_party/uwb_localizer/` へ移動
  （`.gitignore` が `third_party/` を除外しているため、入れ子 git リポジトリが
    本リポジトリの履歴に混入するのを防ぐ）

### Step 1: 測位ソルバの vendoring（実行中）
- [ ] `components/uwb_loc/` へ取り込み（**ロジック無改造**、CMakeLists 追加のみ）
- [ ] `UWB_MAX_ANCHORS` の既定を **16 → 8** に（Kconfig で変更可）。RAM 節約
- [ ] `UWB_USE_FLOAT` は **既定 OFF（double のまま）**
      取り込み元 README が「Beck法の二分法と共分散で桁落ちしやすいので、
      まず double で検証してから float へ」と明記しているため
- [ ] ホスト側テスト 53件を third_party 版・components 版の両方で通す（`UWB_MAX_ANCHORS=8` でも）
- [ ] `make strict`（`-Wall -Wextra -Werror -pedantic`）
- [ ] `firmware/soltest/` … **実機ハード不要**の合成データ検証アプリ
      - アンカー4台の既知配置＋理論距離 → Lv0/Lv2 で解いて真値との誤差確認
      - **アンカー5台で1台を外れ値にしたとき Lv2 が弾いて解が残ること**
      - `uwb_anchors_coplanar()` で同一平面配置（全アンカー z=0）が検出されること
      - `uwb_gdop_at()` の表示
      - **各ソルバの実行時間を `esp_timer_get_time()` で計測**（更新レート設計の根拠）
- [ ] `THIRD_PARTY_LICENSES.md` に uwb_localizer 由来のエントリ追加（コミットハッシュも記録）

### Step 1 完了 (2026-08-19) — メイン検収済み
- [x] `components/uwb_loc/` へ取り込み。**8ファイルすべて third_party と byte 一致を diff で確認**
      （ロジック無改造。追加したのは `CMakeLists.txt` 34行 と `Kconfig` 60行のみ）
- [x] `Kconfig`: `UWB_LOC_MAX_ANCHORS`=8 / `UWB_LOC_MAX_MEAS`=8 / `UWB_LOC_ID_LEN`=16 /
      `UWB_LOC_USE_FLOAT`=n。`target_compile_definitions` で `-D` 上書きする方式
      （ヘッダ本体は書き換えていない）
- [x] ホストテスト: third_party 版・components 版とも **53件すべて通過**
      （`UWB_MAX_ANCHORS=8/MAX_MEAS=8` でも、取り込み元既定 16/32 でも通る）
      → `tools/test_uwb_loc/Makefile` でいつでも回せる。メインも実行して再現確認
- [x] `firmware/soltest/` (main.c 415行) クリアビルド **警告0・エラー0**、
      `uwb_soltest.bin` 0x36720 bytes。`CONFIG_UWB_LOC_MAX_ANCHORS=8` 反映確認
- [x] `THIRD_PARTY_LICENSES.md` に uwb_localizer エントリ追加
      （取り込み元コミット `8d0edc057ed05cf6b4af91df329999fe2343f515`）
- [x] 既存 firmware (probe/devtest/twr) が無傷であることを確認

#### `UWB_MAX_ANCHORS=8` の影響 → **実害なし**
`UWB_MAX_ANCHORS` はライブラリ内部の配列サイズに一切使われていない
（使用箇所は `examples/03_replay.c` のみ）。スタック使用量に効くのは **`UWB_MAX_MEAS`** 側
（`uwb_nls.c`/`uwb_closed_form.c`/`uwb_ekf.c` の残差・ヤコビアン・pending バッファ）。
`uwb_config.anchors` はポインタ借用なのでアンカー配列は呼び出し側が確保する。

#### 【上流への報告事項】uwb_localizer の `make strict` が通らない
`third_party/uwb_localizer/c` で `make strict` すると **`src/uwb_nls.c:342`** で失敗する。
```
src/uwb_nls.c:342:18: error: variable 'wsum' set but not used [-Werror,-Wunused-but-set-variable]
```
- Lv1/Lv2 側 (138-153行) は `wsum += w[i]; num += w[i]*resid^2;` → `rms = sqrt(num/wsum)` で
  **重み付き RMS** を計算しており `wsum` を使っている
- Lv0 側 (342-352行) は `wsum += w[i];` するが、365行で `rms = sqrt(num/set.n)` と
  **重み無し RMS** を計算しており `wsum` を読まない
  （コード中のコメントに「Lv0 の残差 RMS は Python 版と同じく重み無しの RMS」と明記あり）
- → **機能的なバグではなく、Lv1/Lv2 のブロックからコピペした際の消し忘れ（デッドコード）**。
  Lv0 ブロックの `wsum` 宣言と `wsum +=` の2行を消せば `make strict` が通る。
  README が「`make strict` で無警告」と謳っているので、上流で直す価値がある

#### 【重要な設計発見】アンカー配置が測位の成否を直接左右する
メインが `components/uwb_loc` を直接叩いて実測し、`docs/ANCHOR_PLACEMENT.md` に記録:
- 同一平面配置だと **Lv0 は高さを一切復元できない**（平面の z によらず z=0 を返す）
- **Lv2 は鏡像解のどちらかを返す**。z=0.001 の配置では真値 +0.900 に対し **-0.898** を返した
  → **`ambiguous` フラグを必ず見ること**
- **アンカー平面がワールド原点を通る（z=0）と Lv2 は `ok=0` で完全に失敗する**
  → 「アンカーを床に並べる」は最もやりがちで**最悪の配置**（同一平面 かつ 原点を通る）
- → Phase 4 Step 2 では、起動時に `uwb_anchors_coplanar()` を必ず呼び、
  真なら警告＋`dim=2`/`z_fixed` に倒す設計にする

#### ソルバ実行時間の見積もり（実測は実機待ち）
- Lv0: 反復なし。O(n) で組んで 3x3 を1回解くだけ。最軽量
- Lv2: Beck法の二分法（最大 60+200+200 回、各回 4x4 LU）＋ Gauss-Newton（既定 max_iter=30、
  各回 O(n) + 3x3 ソルブ）。**Lv0 よりかなり重い**
- EKF更新: イノベーションがスカラーなので**逆行列を一切計算しない**。1測距あたり O(nx²)、
  予測 O(nx³)（nx≤9）。Lv2 より軽いはず
- **ESP32-S3 は単精度 FPU のみ。`double` はソフトウェアエミュレーション。**
  既定を double にしている以上、実測はその分を強く反映する。
  → 精度検証後に `UWB_USE_FLOAT` を試せば**大きな高速化が期待できる**
- `firmware/soltest` に `esp_timer_get_time()` による Lv0/Lv2/EKF の計測を実装済み
  （各50回・EKFは40エポック平均、min/avg/max 表示）。実機で数値を取る

#### Step 2 に向けた API 上の注意（実コードで確認済み）
- **`uwb_config.anchors` は借用**（`uwb_config_init` はポインタ代入のみ、コピーしない）。
  アンカー登録テーブルは安定した記憶域に置き、内容変更時は同じ配列を書き換えるか
  `uwb_config_init` をやり直す
- **`uwb_ekf` はスレッド安全でない**（内部に `x`/`P`/`pending[]` の可変状態）。
  Lv0〜Lv2 は無状態の純関数なので `const uwb_config*` の同時読みは安全
- `uwb_fix.excluded` / `n_used` / `n_total` によるゲート結果の可視化は実際に機能する
  （5台中1台に +3.0m バイアスを入れて Lv2 が正しく除外・解を維持することを確認済み）
- libm は追加の `REQUIRES` なしで ESP-IDF がリンクする（実ビルドで確認）

### Step 2: 測距スケジューラ + 測位パイプライン（完了 2026-08-20） — メイン検収済み

#### 成果物
| ファイル | 行数 |
|---|---|
| `components/uwb_ranging/include/uwb_ranging_types.hpp` | 133 |
| `components/uwb_ranging/include/uwb_ranging_anchor_table.hpp` | 123 |
| `components/uwb_ranging/include/uwb_ranging_pipeline.hpp` | 99 |
| `components/uwb_ranging/include/uwb_ranging_scheduler.hpp` | 106 |
| `components/uwb_ranging/src/uwb_ranging_anchor_table.cpp` | 143 |
| `components/uwb_ranging/src/uwb_ranging_pipeline.cpp` | 146 |
| `components/uwb_ranging/src/uwb_ranging_scheduler.cpp` | 144 |
| `components/uwb_ranging/CMakeLists.txt` | 29 |
| `firmware/anchor/`（CMakeLists×2, Kconfig.projbuild, main.cpp, sdkconfig.defaults） | 379 |
| `firmware/tag/`（同上） | 508 |
| `tools/test_pipeline/`（Makefile + test_pipeline.cpp） | 381 |
| **合計** | **2191** |

`components/uwb_ranging` は `namespace uwb`。設計の肝は**ハード非依存部分と
ハード依存部分をファイル単位で分離**したこと:
- ハード非依存（`uwb_loc.h` のみに依存）: `AnchorTable`（アンカー登録テーブル。
  `uwb_config.anchors` の借用ポインタ問題に対応するため安定記憶域を自前で持ち、
  テーブル差し替え時に `uwb_config_init()` をやり直す）、`PositioningPipeline`
  （測距結果→`uwb_meas[]`→Lv0/Lv2/Lv3→`PositionResult` のパイプライン）
- ハード依存（`uwb_qm33120`/ESP-IDF/FreeRTOS）: `RangingScheduler`
  （アンカー登録テーブルを順にポーリングし、1周の所要時間とアンカーごとの
  成功率を記録する）

この分離により `tools/test_pipeline` はハード非依存の2ファイルだけを
`components/uwb_loc` と一緒にホストで直接ビルドして検証できる
（`components/uwb_ranging/src/uwb_ranging_scheduler.cpp` はビルド対象に含めない）。

#### ホストシミュレーションテスト（`tools/test_pipeline`、`make test`）
6シナリオ・34チェック **全件成功**、`make strict`（`-Wall -Wextra -Wshadow -Werror`、
自前実装分のみ）も通過:
1. アンカー4台・非同一平面・ノイズなし → Lv0/Lv2 とも真値と誤差1e-3m未満で一致
2. アンカー5台・1台に+3.0mの外れ値 → Lv2が `excluded` にそのアンカーを立てて弾き、解を維持
3. アンカー5台のうち2台欠測（有効3件）→ `solvable=0` で「測位不能」を返す
4. アンカー5台のうち1台欠測（有効4件）→ 解ける（真値と誤差1e-3m未満で一致）
5. 同一平面配置（天井4隅、原点は通らない）→ `coplanar=1` を検出、Lv2は解けるが `ambiguous=1`
6. アンカー平面が原点を通る配置（z=0）→ `coplanar=1` かつ `originWarning=1` を検出し、
   Lv2が実際に `ok=0` で失敗することを確認。`dim=2`フォールバック
   （`AnchorTable::setDimension2D()`）で回復できることも確認

#### `firmware/anchor`（正式版アンカー、D3）
常時レスポンダとして待ち受けるだけの専用ファーム。役割選択なし
（`firmware/twr` と異なり常にANCHOR固定）。`UWB_ANCHOR_SHORT_ADDR`（hex、既定
`0x0002`）をKconfigで個体ごとに設定でき、実機5台にそれぞれ異なる値を焼き込む
運用を想定。方式（SS/DS、既定DS-TWR）・ボード（M5StampS3A/AtomS3）選択、
成功率(%)込みの統計ログを実装。**NVS保存・シリアルコンソールからの変更は
未実装（将来課題）**。

#### `firmware/tag`（正式版タグ、D2）
アンカー登録テーブル（コンパイル時定数。NVS化は将来課題）を
`uwb::RangingScheduler` で順次ポーリングし、`uwb::PositioningPipeline` で
Lv0（比較用）とLv2（本番）を毎周期解き、`UWB_TAG_ENABLE_EKF`（既定OFF）で
Lv3(EKF)も同一タスクから呼べる。起動時に `AnchorTable::checkPlacement()` を
必ず呼び、同一平面配置の警告、および「アンカー平面が原点を通っています。
ワールド原点をずらしてください」という具体的な警告を出す。
`UWB_TAG_AUTO_2D_FALLBACK`（既定ON）でその条件を検出したら `dim=2` へ自動フォールバック。

出力はJSON Lines（1行1エポック、stdout直書き）。起動時に1回 `"type":"anchors"`、
毎周期 `"type":"meas"`（`third_party/uwb_localizer/uwb_loc/hal/jsonl.py` の
`JsonLinesHal` がそのまま読める標準形。欠測アンカーは単に含めない）と
`"type":"fix"`（本プロジェクト独自の診断行。位置・ok/ambiguous/gdop/
residual_rms/excluded・1周の所要時間・各レベルのソルバ計算時間・アンカーごとの
成否を含む。`JsonLinesHal.parse_line()` は未知のtypeを`"other"`として黙って
読み捨てる仕様なので、既存のPython可視化を壊さず共存できる）を出す。

#### ビルド検証（メインが実施・再現確認）
- `firmware/tag`: `rm -rf build sdkconfig` → `set-target esp32s3` → `build` の
  クリアビルドを **M5StampS3A/DS-TWR(既定)**、**AtomS3/SS-TWR/EKF有効**、
  **AtomS3/DS-TWR**、**dim=2自動フォールバックOFF** の4通りで実施、
  **全て警告0・エラー0**。最終的に既定設定（M5StampS3A/DS-TWR、
  `uwb_tag.bin` 0x4a6c0 bytes）へ復元
- `firmware/anchor`: サブエージェントが M5StampS3A/AtomS3 × SS-TWR/DS-TWR の
  4通りをクリアビルドで警告0・エラー0を確認。メインが既定設定
  （M5StampS3A/DS-TWR、`UWB_ANCHOR_SHORT_ADDR=0x0002`）で再現確認
  （`uwb_anchor.bin` 0x42730 bytes）
- 既存 `firmware/probe`/`firmware/devtest`/`firmware/twr`/`firmware/soltest` を
  メインが全てクリアビルドで再検証、**全て警告0・エラー0**（無傷）

#### 設計上の判断（迷った点）
- `uwb_anchor.sigma0`/`sigma_per_m` は `AnchorEntry` に対応フィールドが無いため、
  `AnchorTable` は常に `sigma0=0` を書き込み、`uwb_model.c` の
  「0以下なら0.1mにフォールバック」に委ねる設計にした（個別アンカーの
  ノイズモデルを外から調整したくなったらAnchorEntryの拡張が必要）
- 「アンカー平面が原点を通る」判定のしきい値（`originPlaneEpsM`、既定0.05m）は
  実測2点（z=0.000で失敗／z=0.001で鏡像解）からの外挿であり、正確な境界は
  未特定。安全側に倒した暫定値として明記した
- `uwb_fix.excluded`（ソルバに渡した観測配列の添字基準）を
  `PositionResult.excluded`（アンカー登録テーブルの添字基準）へ変換する処理を
  パイプライン側に持たせた（呼び出し側がアンカー添字で直接扱えるようにするため）
- `RangingScheduler::lastCycleMs()` は `cycleIntervalMs` による意図的な待ち時間を
  含めない「純粋なポーリング所要時間」とした（R6実測の素の値を汚さないため。
  実効更新レートを知りたい場合は呼び出し側で `runCycle()` の呼び出し時刻の
  差分を取る運用とした）

#### 実機が来たら真っ先に確認すべきこと（特にR6）
1. `firmware/anchor` を5台に別々の `UWB_ANCHOR_SHORT_ADDR` を焼いて書き込み、
   `firmware/tag` の `kAnchors` テーブル（座標は暫定値なので実測に置き換える）と
   アドレスを一致させて疎通確認
2. `"type":"fix"` 行の `cycle_ms` を見て**1周の所要時間を実測**する。
   DS-TWRとSS-TWRで比較し、`UWB_TAG_PER_ANCHOR_INTERVAL_MS`/
   `UWB_TAG_CYCLE_INTERVAL_MS` を0のまま（最速）で何Hz出るか確認するのが最初の一歩。
   アンカーごとの成功率は `RangingScheduler::stats()` から取れる
   （現状はfirmware/tagのログに未接続。実測後に必要ならログ出力を追加すること）
3. `"anchors"` ごとの `elapsed_ms` を見て、往復に特に時間がかかっているアンカーが
   いないか（配置・電波環境の問題の切り分け）
4. 実配置でのアンカー座標を `kAnchors`（`firmware/tag/main/main.cpp`）へ反映し、
   `docs/ANCHOR_PLACEMENT.md` の配置ルール（非同一平面・原点を通らない）を
   満たしているか `checkPlacement()` の起動ログで確認
5. アンテナ遅延未校正の状態で `ambiguous`/`ok=0` が想定より多く出ないか
   （Phase 3のキャリブレーション未実施の影響切り分け）

#### 将来課題
- `firmware/anchor`/`firmware/tag` ともアドレス・アンカーテーブルの
  NVS化・シリアルコンソール対応（現状はKconfig/コンパイル時定数のみ）
- `RangingScheduler::stats()`（アンカーごとの成功率）を `firmware/tag` の
  JSON出力またはログへ接続する
- `UWB_TAG_ENABLE_EKF` 有効時のEKF出力を可視化パイプラインへどう繋ぐか
  （現状は`"fix"`行の`"lv3"`セクションに載るのみ）


---

## 【横道】上流 uwb_localizer の最適化 (2026-08-19〜20) — 完了

ユーザ判断「組込みプログラムは1回でも早くするのが正しい」により、
`components/uwb_loc/` を手当てするのではなく **上流 kouhei1970/uwb_localizer を直す**方針に。

作業場所: `/Users/kouhei/tmp/github/uwb_localizer`、ブランチ **`perf/exploit-structure`**
**未コミット・未 push。ユーザのレビュー待ち。**

### 実施内容（4件）
| # | 内容 | ファイル |
|---|---|---|
| 0 | `wsum` デッドコード削除 → `make strict` が通るように | `c/src/uwb_nls.c` |
| A | EKF update の Joseph 形式を O(nx³)→O(nx²)。作業行列を全廃 | `c/src/uwb_ekf.c` |
| B | EKF predict の `F P Fᵀ` を O(nx³)→O(nx²)。F の Kronecker 構造を利用 | `c/src/uwb_ekf.c` |
| C | Beck 二分法の 4x4 LU をスカラー演算に。固有ベクトル対応 `uwb_sym_eig()` 新設 | `c/src/uwb_linalg.{c,h}`, `c/src/uwb_closed_form.c` |
| D | Beck の根探索を安全策付きニュートン法(rtsafe型)に。反復 56→3〜5回 | `c/src/uwb_closed_form.c` |
| E | 収束判定を丸め誤差の床で打ち切り。**潜在バグ(y の復元点)も発見・修正** | `c/src/uwb_closed_form.c`, `c/src/uwb_internal.h` |
| F | Jacobi 要素スキップ閾値 → **効果ゼロにつき差し戻し** | （差し戻し済み） |
| + | ベンチマークハーネス新設、固有ベクトル検証テスト追加 | `c/bench/`, `c/tests/test_uwb.c`, `c/Makefile` |

`git diff --stat`: 8ファイル、+313/-121

### 結果
- `uwb_ekf_update` **5.7倍**、`uwb_ekf_predict` **4.3倍**、`uwb_beck_gtrs` **3.0倍**、
  `uwb_solve_lv2` **2.6倍**、`uwb_solve_lv1` **2.5倍**（`uwb_solve_lv0` は対象外で不変）
- **スタック約3.1KB削減**（double版）
- **数値は完全に維持**: crossval worst 8.36e-11 でベースラインと同一。
  C テスト 77件（元53+新24）、strict / float とも通過、pytest 177 passed/1 failed（既存の1件のみ）
- 詳細は `docs/PERF_ANALYSIS.md` の追記節

### 上流へのフィードバック事項（ユーザ判断待ち）
1. `make strict` が通らなかった（`uwb_nls.c:342` の `wsum`）。
   **既存テスト `test_c_port.py::test_c_library_builds_without_warnings` が正しく検出していた**
2. `tests/test_calibration.py::test_self_survey_with_noise_and_missing_links[1]` が失敗する。
   C実装と無関係で、Python 側自己測量の**同一平面→鏡像**問題。
   ライブラリ自身が `align_to_reference: 既知点がほぼ同一平面...` と警告を出しており、
   `docs/ANCHOR_PLACEMENT.md` に記録した現象と同一。
   **テストの基準アンカーに高さの違う点を1つ足せば解消する性質**

### 本リポジトリへの影響（TODO）
**上流がマージされたら `components/uwb_loc/` を再 vendoring すること。**
現在は vendoring 時点（`8d0edc057ed05cf6b4af91df329999fe2343f515`）と byte 一致だが、
最適化が入ると差分が生じる。`THIRD_PARTY_LICENSES.md` のコミットハッシュも更新。

### 教訓（記録）
当初 Beck は「2〜3倍で A/B ほど劇的でない」と優先度を低く見ていたが、
A/B 完了後に測り直すと **Beck が Lv2 の86%を占める**構図になり、絶対値では最大の削減余地だった。
→ **段階ごとに測り直さないと優先順位を誤る。**


---

## Phase 4 Step 2 完了 (2026-08-20) — メイン検収済み

### 成果物（計 2,191行 / 21ファイル）
| | 行数 | 依存 |
|---|---|---|
| `components/uwb_ranging/` | 923 | |
| ├ `uwb_ranging_types.hpp` | 133 | **ハード非依存**（uwb_loc のみ） |
| ├ `uwb_ranging_anchor_table.{hpp,cpp}` | 123+143 | 同上 |
| ├ `uwb_ranging_pipeline.{hpp,cpp}` | 99+146 | 同上 |
| └ `uwb_ranging_scheduler.{hpp,cpp}` | 106+144 | ハード依存（uwb_qm33120/ESP-IDF） |
| `firmware/anchor/` | 379 | 常時レスポンダ。`UWB_ANCHOR_SHORT_ADDR` で個体別アドレス |
| `firmware/tag/` | 508 | テーブル→スケジューラ→パイプライン→JSON Lines |
| `tools/test_pipeline/` | 381 | ホストシミュレーションテスト |

**ハード依存を `scheduler` だけに隔離**できたので、パイプラインはホストで検証できる。

### ビルド（メインが再現確認）
- `firmware/tag` **警告0・エラー0**、`uwb_tag.bin` 0x4a6c0 bytes
- `firmware/anchor` **警告0・エラー0**、`uwb_anchor.bin` 0x42730 bytes
- 既存4本（probe/devtest/twr/soltest）も無傷

### ホストシミュレーションテスト（メインが実行、**34件中0件失敗**（当時。現在は53チェック））
| # | シナリオ | 結果 |
|---|---|---|
| 1 | 4台・非同一平面・ノイズなし | 真値と誤差 1e-3m 未満 |
| 2 | 5台・1台に外れ値 | `excluded` ビットが立ち解が維持される |
| 3 | **5台中2台欠測（有効3件）** | **`solvable=0` で「測位不能」** |
| 4 | 5台中1台欠測（有効4件） | 正しく解ける |
| 5 | 同一平面（天井、原点は通らない） | `coplanar=1` 検出、Lv2 は解けるが `ambiguous=1` |
| 6 | **アンカー平面が原点を通る (z=0)** | `coplanar=1` + `originWarning=1`、Lv2 が実際に `ok=0`、パイプラインが「測位不能」扱い。`dim=2` フォールバックでの回復も確認 |

**docs/ANCHOR_PLACEMENT.md の知見がすべてテストで守られている。**

### JSON Lines は uwb_localizer の Python 可視化と互換にできた
- 起動時 `{"type":"anchors"}`、毎周期 `{"type":"meas"}` を
  `third_party/uwb_localizer/uwb_loc/hal/jsonl.py` の `JsonLinesHal` がそのまま読める
  標準スキーマで出力（`Anchor.from_dict` / `Measurement.from_dict` と1:1対応）
- 追加情報（ok/ambiguous/gdop/residual_rms/excluded/周期時間/ソルバ計算時間）は
  独自の `{"type":"fix"}` 行に載せた。`parse_line()` は未知 type を `"other"` として
  読み捨てる仕様なので**既存の可視化を壊さず共存できる**
→ **ホスト側ツールを新規に書かずに済む見込み**

### 設計上の判断
- `AnchorEntry` に sigma 系フィールドが無いため `sigma0=0` 固定（uwb_loc 側の 0.1m フォールバックに委ねる）
- 「原点を通る」判定閾値 0.05m は実測2点からの外挿。正確な境界は未特定
- `excluded` ビットマスクをソルバの観測配列添字→アンカーテーブル添字へ変換する処理を
  パイプライン側に持たせた

### 将来課題
- アンカーアドレス/座標テーブルの NVS 化・シリアルコンソール対応（anchor/tag 両方）
- `RangingScheduler::stats()`（アンカー別成功率）が firmware/tag のログ/JSON に未接続
- EKF(Lv3) 出力の可視化パイプラインへの接続方法は未検討

### 実機到着時の手順
1. アンカー5台に別アドレスを焼き、`firmware/tag` の `kAnchors`（暫定座標）を実測値に置換
2. `"fix"` 行の `cycle_ms` で **1周の所要時間を実測**（SS/DS 比較、間隔0での最速値）← **R6**
3. `"anchors"` 各エントリの `elapsed_ms` で遅いアンカーを特定
4. `checkPlacement()` の起動ログで実配置が ANCHOR_PLACEMENT.md のルールを満たすか確認
5. アンテナ遅延未校正時の `ambiguous` / `ok=0` の頻度を切り分け

---

## スコープ変更 (2026-08-20 ユーザ指示)

**「本リポジトリは m5stack の UWB を ESP32-S3 で使う専用なので、そちらに最適化してよい」**

### 切り分け
| 対象 | 方針 |
|---|---|
| `m5stack_uwb` | **ESP32-S3 専用最適化 OK** |
| `uwb_localizer`（上流） | **移植性維持**（他でも使うライブラリ） |
| StampFly | 引き続き**非依存**（利用者の一つ） |

「StampFly 非依存」と「プラットフォーム非依存」は別物。前者は維持、後者は放棄。

### 即座に適用したこと
全ファーム（probe/devtest/twr/tag/anchor。soltest はベンチ作業中のため別途）の
`sdkconfig.defaults` に追加:
```
CONFIG_COMPILER_OPTIMIZATION_PERF=y      # -O2。既定は -Og だった
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y    # 既定は 160MHz
```

再ビルド結果（全て警告0・エラー0、`-O2` と 240MHz の反映を sdkconfig と
compile_commands.json で確認）:
| ファーム | 変更前 | 変更後 |
|---|---|---|
| tag | 0x4a6c0 | **0x48430** |
| anchor | 0x42730 | **0x3fbf0** |
| twr | 0x42220 | **0x3f650** |
| devtest | 0x44f20 | **0x42300** |
| probe | 0x40790 | **0x3de60** |

**-Og → -O2 でバイナリはむしろ縮んだ。**

### 背景
`docs/PLATFORM_TUNING.md` の調査で判明した見落とし3件:
1. ファームが `-Og` でビルドされていた（ホストのベンチは -O2。実機測定が無意味になるところだった）
2. CPU が 160MHz（240MHz にできる = 1.5倍）
3. **単精度 FMA `madd.s` がハードにある**（GCC が `a + b*c` から自動生成。`madd.d` は無い）
   → `UWB_USE_FLOAT` の価値が想定より高い

### 今後の評価項目（実機測定後）
- `UWB_USE_FLOAT`（精度検証が先）
- ホット関数の IRAM 配置
- esp-dsp（3x3/4x4 float 行列積のアセンブリ実装あり。ただし最適化後のホットパスは
  行列積ではないので期待値は低い）


---

## 実機用マイクロベンチを追加 (2026-08-20) — メイン検収済み

`firmware/soltest/main/bench.c` (776行) + `bench.h` (46行)。
実機が届いた瞬間に、以降の最適化判断の土台となる数字が全部取れる状態にした。

### 計測内容
- **基本演算のサイクル数**: float / double の 加算・減算・乗算・**積和(FMA)**・除算・sqrt、参考に int32
  - **レイテンシ**（依存チェーン20万回）と**スループット**（独立8本）を分けて測る
- **メモリアクセス**: 9x9 配列（EKF の P 相当）の連続 / ストライド(72バイト) read-modify-write。float/double 両方
- 既存のソルバ計測（Lv0/Lv2/EKF）と、**float ビルド時の精度**（合成データの真値との誤差）

### 計測ロジック（検収でコードを確認）
- `esp_cpu_get_cycle_count()` を使用。CPU周波数は `esp_rom_get_cpu_ticks_per_us()` から取得しログ先頭に出す
- 初期値・係数を**実行時 seed**（サイクルカウンタ下位ビット）から作り、コンパイル時定数化を防ぐ
- 結果は**ループを抜けた後に一度だけ** volatile シンクへ（ループ内 volatile は計測を歪めるため）
- 空ループのオーバーヘッドを測って差し引く
- **メモリベンチだけ配列を `volatile` にする**（「メモリアクセスを実際に発生させる」のが目的で、
  スカラー演算ベンチとは逆の判断。コメントに明記）
- **サイクルカウンタはコアごとに独立**なので `xTaskCreatePinnedToCore(..., 0)` で CPU0 に固定。
  各計測後に `vTaskDelay(1)` を挟んで Task Watchdog を回避

### 逆アセンブルによる裏取り（メインが再現確認）
`bench.c.obj` を objdump した結果:

| 命令 | 出現数 | 意味 |
|---|---|---|
| **`madd.s`** | **65** | **float FMA はハード1命令**（`madd.d` は存在せず） |
| `add.s` / `sub.s` / `mul.s` | 62 / 9 / 9 | float 加減乗はハード |
| `__divsf3` / `sqrtf` | 18 / 18 | **float の除算・sqrt はソフト** |
| `__adddf3` / `__muldf3` / `__divdf3` / `__subdf3` / `sqrt` | 260 / 166 / 52 / 30 / 18 | **double は全てソフト**。double FMA は `__muldf3`→`__adddf3` の対で出る |

`docs/PLATFORM_TUNING.md` の記述を機械語レベルで裏付けた。

### ビルド（メインが再現確認）
| 設定 | 結果 |
|---|---|
| double（既定）・-O2・240MHz | 警告0・エラー0、`uwb_soltest.bin` 0x38ec0 |
| **float** (`CONFIG_UWB_LOC_USE_FLOAT=y`) | 警告0・エラー0、**0x371c0**（-7KB）。`-DUWB_USE_FLOAT=1` 反映確認 |
| probe/devtest/twr/tag/anchor | 全て無傷 |

リポジトリは double 既定の状態に戻してある。

### 未対応（申し送り）
- `CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE` は**変更していない**（安全性設定なので判断保留）
- int32 の除算・乗算がハードかソフトか未確認（参考値として断定せずに出力）
- メモリベンチのオーバーヘッド差し引きは、フラットループの値をネストループへ流用する近似

---

## 実行時設定（NVS + シリアルコンソール）を追加 (2026-08-20)

`docs/GETTING_STARTED.md` を書いた結果、**購入者が実機で最も繰り返す作業が
ビルド時定数になっていて手順が煩雑**と判明したため、実機到着前に潰した。

| 対象 | 変更前 | 変更後 |
|---|---|---|
| アンカーのアドレス | Kconfig。5台に焼くのに menuconfig→build→flash を5回 | `addr set 0x0003` / `save` / `reboot` |
| アンカー座標テーブル | `kAnchors[]` 定数。数cm直すたびに再ビルド | `anchor set 0 0x0002 0.0 0.0 2.4` / `save` |

### 成果物
- `components/uwb_cfgstore/`（962行）
  - `uwb_cfgstore_blob.{hpp,cpp}`（541行）… **ESP-IDF 非依存**のバイト列形式。ホストで検証可能
  - `uwb_cfgstore.{hpp,cpp}`（392行）… NVS 層とフォールバック判断
- `firmware/anchor/main/anchor_console.{hpp,cpp}`（495行）
- `firmware/tag/main/tag_console.{hpp,cpp}`（834行）
- `firmware/{anchor,tag}/partitions.csv`（配置は ESP-IDF 既定と同一。nvs 必須を明示するため）

### 後方互換
- **NVS が空・未初期化・破損のいずれでも既定値にフォールバックして起動を継続**
- Kconfig の `UWB_ANCHOR_SHORT_ADDR` と `kAnchors[]` は「NVS が空のときの初期値」として存置
- コンソールを無効化する Kconfig あり（ただし NVS 分の +10〜11KB は残る）

### 測位ループとコンソールの同時実行
**編集用シャドウコピー + 1周期境界での差し替え**（片方向の二重バッファ）を採用。
- コンソールは編集用コピーだけを書き換えて `pending` を立てる
- 測位ループは**周期の先頭**で `takePendingTable()` を呼び、変更があればそこで `AnchorTable::set()`
- ミューテックスが守るのは数百バイトの memcpy のみ。測距・測位・JSON 出力中はロックを持たない
- 理由: `uwb_config.anchors` は内部記憶域を指すので、`set()` 中にソルバから読まれると
  `n_anchors` と実体が食い違う。「測位が走っていない瞬間にしか差し替えない」を構造で保証した

### 検証
- 全6ファーム クリアビルド **警告0・エラー0**
- ホストテスト **132件**（従来53 + シリアライズ検算79）。`make strict` も通過
  - 往復、境界値、**破損データ**（全0/全0xFF の未初期化フラッシュ、CRC 1ビット化け、
    版違い、件数改ざん、NaN/Inf、桁違い座標）
- バイナリ: tag 0x59020 (364KB) / anchor 0x52520 (337KB)。1M パーティションに対し 33〜35%

### 実装中に潰した既存バグ
- **`app_main` のスタック枯渇リスク**。`CONFIG_ESP_MAIN_TASK_STACK_SIZE` が 3584 なのに
  JSON バッファ 2KB×2 + `PositioningPipeline`（内部 `uwb_ekf` だけで約1KB）+ `AnchorTable`
  が同一フレームにあった。JSON バッファを単一 static に、他を `.bss` へ移動（出力は不変）

### 判断のログ
- **argtable3 は使わなかった。** 位置引数の解析が内部で `getopt_long` を通るため、
  `-1.5` が `-1 -. -5` の不正オプション扱いになる（ホストで再現確認済み）。
  座標もアンテナ遅延も負値を取るので、`argtable=nullptr` にして argc/argv を直接見る形にした。
  linenoise の行編集・履歴・`help`・dispatch は console コンポーネントのまま使用
- **`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` に変更。** 既定（UART0 が一次）だと
  `esp_console_new_repl_usb_serial_jtag()` の宣言が出ずキー入力を受け取れない。
  UART0(GPIO43/44) への出力は失われるが、これらのボードでは何も繋がっていない
- タグの JSON 出力が毎周期流れると設定作業ができないので `output on|off` を追加

### 実機が無いため未検証
- NVS への実際の読み書き（ホストで検証したのは「NVS に置くバイト列」の層まで）
- USB-Serial/JTAG 上の REPL が実機で動くか（linenoise の端末制御、`idf.py monitor` との相性）
- 設定変更中に測距・測位が途切れないか

---

## ハードウェア構成の確定 (2026-08-20 ユーザ指示)

| 役割 | ボード | 台数 | UWB |
|---|---|---|---|
| **タグ（移動体）** | **M5StampS3A** | 1 | M5Stamp UWB Module |
| **アンカー（固定局）** | **M5 AtomS3** | 5 | M5Stamp UWB Module |

- `firmware/tag` の既定ボード = M5StampS3A（変更なし）
- **`firmware/anchor` の既定ボードを M5StampS3A → AtomS3 に変更**（ビルド確認済み）

### この割り当てで注意すべき点
- **AtomS3 は空きGPIOが6本しかない**（`boards/atoms3.h` 参照）。
  SCK=G7 / MOSI=G6 / MISO=G5 / CS=G8 / RST=G1 / IRQ=G2 で、
  **WAKEUP と GP7 は未配線**。最小配線 + RST + IRQ は確保できている
- **IRQ が取れているのは重要**。R6（IRQ駆動化）で効くのは
  **レスポンダ側の折り返し時間**（`POLL_RX_TO_RESP_TX_DLY`）であり、
  レスポンダ＝アンカー＝AtomS3 だから。ここが IRQ 化できないと R5 の遅延短縮が頭打ちになる
- M5StampS3A は GPIO に余裕があり、8本フル配線できる

### 接続方法の変更（同日ユーザ要望）
FPC（0.5mm 12P）ではなく、**モジュールに出ている半田付け用パッドを使う**。
実験段階では FPC より扱いやすいため。→ `docs/SOLDER_PADS.md`（調査中）

---

## 設計上の確定事項: タグは IRQ 非依存 (2026-08-21)

**タグ側は UWB の割り込みを使わない。ポーリングで成立させることを仕様とする。**

理由: 最終ターゲットが StampFly であり、StampFly で外部に出ているのは
GROVE の4本（G13/G15/G1/G2）のみ。SPI 4線で使い切るため IRQ 線が残らない。
（当初「空きGPIO G5/G10/G41/G42」としていたのは誤りで、この4本はモータPWM）

| 役割 | ボード | IRQ | 方針 |
|---|---|---|---|
| **タグ** | StampFly (StampS3) | **取れない** | **IRQ 非依存。ポーリング経路を必ず残す** |
| タグ | 単体 M5StampS3A | 取れる (G7) | 使ってもよいが**依存しない** |
| **アンカー** | AtomS3 / AtomS3R | **取れる (G2)** | **R6 の対象はここだけ** |

### 現状の確認
`dwt_setinterrupt` / `dwt_setcallbacks` / `dwt_isr` / `gpio_isr_handler_add` は
**ソース中で一度も呼ばれていない**（ヒットするのはビルド生成物の .map のみ）。
`uwb_port` は `gpio_set_direction(pin_irq, GPIO_MODE_INPUT)` するだけ。
**タグ・アンカーとも完全ポーリング**であり、タグ側は既に仕様を満たしている。

### R5（遅延値の追い込み）への影響
DS-TWR では**両側に遅延送信の締切がある**:
- アンカー（レスポンダ）: `responseTxDelayUus` → **IRQ 化で詰められる**
- **タグ（イニシエータ）: `finalTxDelayUus` → ポーリングのままなので詰められない**

| 構成 | アンカー | タグ | 1周 | レート |
|---|---:|---:|---:|---:|
| 現状 | 3077µs | 1846µs | 31.9ms | 31.3 Hz |
| **R5（アンカーのみ IRQ）＝ StampFly の現実解** | **900µs** | **1400µs** | **16.8ms** | **59.4 Hz** |
| （参考）両側 IRQ | 900µs | 700µs | 11.1ms | 90.2 Hz |

**59 Hz は位置制御の実効帯域（約0.064 Hz）に対して十分すぎる**ので、
StampFly 用途ではここで打ち止めてよい。

### IRQ 方針の確定 (2026-08-21 ユーザ決定)
> アンカーは IRQ を積極的に使用し、タグは使用しない。
> ただし StampFly が別配線で IRQ 接続できるかもしれない可能性を残し、
> その対応も準備しておく。

→ `docs/IRQ_POLICY.md` に確定版をまとめた。要点:
- **IRQ はポーリングを「置き換える」のではなく「選択肢として足す」。**
  ポーリング経路は常に第一級の実装として残す
- `pin_irq == UWB_PORT_PIN_UNUSED` なら設定に関わらずポーリングへフォールバック
- **遅延値は役割ごと・IRQ の有無ごとのプリセットにする**（ハードコードしない）
- **【重要】遅延値はタグとアンカーで一致していないと測距が成立しない。**
  プリセットにバージョン番号を持たせ、不一致を検出して警告する
- `boards/stampfly.h`（未作成）は `pin_irq` を持たせ既定 UNUSED。
  別配線に備えて G6/G8/G11 を候補としてコメントに残す

### アンカーのピン予算（8本ちょうど）
| 構成 | SPI | RST/IRQ | ToF(I2C) | ToF の接続 |
|---|---|---|---|---|
| A（無印 AtomS3 向け） | G5-G8 | Grove G1/G2 | G38/G39 | 底面へ手配線 |
| B（**AtomS3R 向け**） | G5-G8 | G38/G39 | **Grove G1/G2** | **Grove に挿すだけ** |

AtomS3R は G38/G39 に何も繋がっていないので構成Bが綺麗。
無印は G38/G39 が MPU6886 の I2C なので構成A。

### StampFly で未使用の GPIO（要実機確認）
現行ファームが使っていないのは **G1, G2, G6, G8, G11, G13, G15 の7本**。
Grove の4本（G1/G2/G13/G15）以外に **G6 / G8 / G11** が未使用。
**基板上でアクセスできればタグでも IRQ が取れる**が、
「ファームが使っていない」と「基板に出ている」は別問題。実機で要確認。
（この区別を誤って G5/G10/G41/G42 を「空き」と誤報した前科がある）
