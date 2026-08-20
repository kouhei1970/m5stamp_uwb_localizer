# 調査2: stampfly_ecosystem の構造 (2026-08-19)

> **【重要な訂正 2026-08-19】ディレクトリ名について**
> 本調査はローカルクローン（2026-05-09 時点、古い）を対象に行ったため、
> `vehicle_new` という名前で記述している箇所がある。**リモートでは既にリネーム済み**:
>
> | 本文中の表記 | 現在の実際の名前 | 中身 |
> |---|---|---|
> | `firmware/vehicle_new` | **`firmware/vehicle`** ← **今回の統合対象** | sf_board / sf_core / sf_estimator など |
> | `firmware/vehicle`（旧世代） | `firmware/vehicle_old` | sf_algo_* / sf_svc_* 命名 |
>
> 以下の本文で `vehicle_new` とあるものは `vehicle` と読み替えること。
> アーキテクチャに関する記述（バス集約・Pub-Sub・タスク構成）はそのまま有効。
>
> リモートの `firmware/vehicle/components`（2026-08-19 時点、ローカルより増えている）:
> `sf_actuator sf_api sf_autotune sf_board sf_calibration sf_comm sf_command sf_controller
> sf_controller_pid sf_core sf_estimator sf_estimator_complementary sf_estimator_eskf
> sf_failsafe sf_hal_bmi270 sf_hal_bmm150 sf_hal_bmp280 sf_hal_button sf_hal_buzzer
> sf_hal_led sf_hal_motor sf_hal_pmw3901 sf_hal_power sf_hal_vl53l3cx sf_logger sf_math
> sf_notify sf_state sf_takeoff_landing sf_telemetry`
>
> **ローカルクローンは古いので、Phase 6 着手前に更新が必要。**

## ファームウェアの所在（世代並存に注意）
| ディレクトリ | 位置づけ | 状態 |
|---|---|---|
| `firmware/vehicle/` | 現行実機ファーム (ACRO skeleton) | 実装済み・ビルド可 |
| `firmware/vehicle_new/` | 次世代。BSP層 `sf_board` 新設、Pub-Sub 再設計 | 開発中（直近更新が最新） |
| `firmware/controller/` | 送信機(プロポ)側、ESP-NOW | — |
| `firmware/common/` | vehicle/controller 共有 | — |

- リポジトリ全体を `uwb` で grep → **0件**。UWB は完全新規追加。
- `vehicle` の既知負債: extern グローバル乱立、I2Cバス所有権が中途半端
  → これを解消する形で `vehicle_new` が設計されている

## ビルド系
- **ESP-IDF のみ**（PlatformIO なし）。ターゲット **esp32s3**（M5StampS3A）
- IDF バージョン: `firmware/vehicle/dependencies.lock` に `idf: 5.5.2`
- component 命名規則: **`sf_<layer>_<name>`**（例 `sf_hal_bmi270`）
- 外部取得は Component Manager 経由（`managed_components/espressif__led_strip` のみ）

## 既存ドライバ（チップ型番ベース）
`sf_hal_bmi270`(IMU/SPI), `sf_hal_bmm150`(磁気/I2C), `sf_hal_bmp280`(気圧/I2C),
`sf_hal_vl53l3cx`(ToF/I2C ×2), `sf_hal_pmw3901`(フロー/SPI), `sf_hal_power`(INA3221/I2C),
`sf_hal_motor/led/buzzer/button`

### 典型構造（sf_hal_bmp280）
```
CMakeLists.txt   # idf_component_register(SRCS "bmp280.cpp" INCLUDE_DIRS "include" REQUIRES driver esp_timer)
bmp280.cpp
include/bmp280.hpp
```
- C++ クラス、`namespace stampfly`、クラス名=チップ名大文字
- API: `esp_err_t init(const Config&)` / `esp_err_t read(BaroData&)`
- 全公開関数が `esp_err_t` を返す。コンポーネント内で `ESP_ERROR_CHECK` は**使わない**
- ログ TAG = クラス名。読み取り失敗ログは約5秒スロットリング
- init は `app_main` ではなく**各センサタスクの setup フェーズ**で呼ぶ。
  非致命センサの初期化失敗は `vTaskDelete(NULL)` でタスクのみ終了

`sf_hal_vl53l3cx` はベンダSDK取り込みのため `src/ docs/ Kconfig README.md` を持つ複雑版
→ UWB のようにドライバコードが大きい場合はこちらに近い形になる

## SPI バス（重要）
- SPI デバイスは **BMI270 と PMW3901 の2つのみ**
- **単一ホスト `SPI2_HOST` を共有。`SPI3_HOST` は未使用（空き）**
- vehicle_new ではバス初期化を `sf_board` の `init_spi_bus()` に集約
  （`spi_bus_initialize(SPI2_HOST, ...)` を1回、MOSI/MISO/SCLK のみ。CSは各ドライバが `spi_bus_add_device()`）
- 各ドライバも内部で `spi_bus_initialize()` を呼び `ESP_ERR_INVALID_STATE` を許容
  → **単独利用と sf_board 配下利用の両対応**（この二重初期化耐性は本件でも踏襲すべき）

### 確定ピンマップ（出典: sf_hal_bmi270/docs/M5StamFly_spec_ja.md 4.2節）
| GPIO | BMI270 | PMW3901 |
|---|---|---|
| G14 | MOSI | MOSI |
| G44 | SCK | SCK |
| G43 | MISO | MISO |
| G46 | CS | - |
| G12 | - | CS2 |
| G11 | INT1 | - |

BMI270 = 10MHz/mode0、PMW3901 = mode3

## I2C バスと GROVE 端子（重要）
- メイン I2C: **I2C_NUM_0 のみ、SDA=GPIO3 / SCL=GPIO4**（PCB固定）。`sf_board` で初期化
  接続: BMP280, BMM150, VL53L3CX×2 (XSHUT でアドレス切替), INA3221
- I2C_NUM_1 は未使用
- **GROVE 端子は HW仕様書に記載ありだがコード上は未実装（GPIO番号を grep して 0ヒット）**
  - Grove (赤/I2C): **SDA=G13, SCL=G15** ← メインI2Cとは別系統
  - Grove (黒/UART): **G1, G2**
  → StampFly には Grove が2系統あり、信号線は合計4本使える可能性がある

## タスク/データ連携
- `xTaskCreatePinnedToCore`。優先度・スタックは `main/config.hpp` に集中定義
- コア0 = センサ/サービス系、コア1 = IMU/制御/状態
- 優先度帯: IMU=24 > Control=23 > State=22 > OptFlow=20 > Mag=18 > Baro=16
  > Comm=15 > ToF=14 > Telemetry=13 > Power=12 > Button=10 > Notify=8 > CLI/Log=5
- 周期実行は必ず `vTaskDelayUntil()`（`vTaskDelay` 単体は不使用）
- 1タスク=1ファイル: `tasks/<name>_task.cpp`、宣言は `tasks/tasks.hpp` に一括
- vehicle_new のデータ受け渡しは `sf_core` の軽量 Pub-Sub (`topic.hpp`)
  - `Topic<T, Latest, 1>` : mutex保護の最新値
  - `Topic<T, RingBuffer, N>` : ロックフリーSPSC、ISR安全（高頻度）
  - `Topic<T, Queue, N>` : FreeRTOSキュー（低レートセンサ）← UWB はこれが近い
  - 全トピックは `sf_core/include/topics.hpp` に集約宣言

## 新規コンポーネント追加で触るファイル
1. `components/sf_hal_<new>/CMakeLists.txt` (新規)
2. `components/sf_hal_<new>/include/<new>.hpp`, `<new>.cpp`
3. `components/sf_hal_<new>/Kconfig`（GPIO 既定値。`sf_hal_vl53l3cx/Kconfig` が参考）
4. `main/CMakeLists.txt`（REQUIRES 追加、タスクファイルを SRCS へ）
5. `main/config.hpp`（GPIO・優先度・周期の定数）
6. `tasks/tasks.hpp` + `tasks/<new>_task.cpp`
7. (vehicle_new) `components/sf_core/include/topics.hpp`
8. (共有バス使用時) `components/sf_board/board.cpp`
9. 推定へ組み込むなら `sf_estimator` / `sf_algo_fusion`
- トップ `CMakeLists.txt` は `EXTRA_COMPONENT_DIRS` で自動走査のため通常編集不要

## 本件への示唆
- 統合先は **`firmware/vehicle`**（＝本文中の `vehicle_new`。バス集約・Pub-Sub 済み）で確定。
  `firmware/vehicle_old` は旧世代なので対象外
- コンポーネント名は型番込みが流儀 → `sf_hal_uwb_<chip>`
- SPI 接続なら **空いている `SPI3_HOST` を専用に使う案が有力**（測距はレイテンシに敏感、
  かつ BMI270 の 10MHz/mode0 バス占有と競合しない）
- ただし **CS/IRQ/RST 用の空き GPIO 精査が必要**（G46/G12/G11 は使用済み）
