# 割り込み（IRQ）の使用方針 (2026-08-21 確定)

## 仕様

| 役割 | ボード | IRQ | 方針 |
|---|---|---|---|
| **アンカー** | AtomS3 / AtomS3R | **あり（G2 または G38/G39）** | **積極的に使用する** |
| **タグ** | StampFly (StampS3) | 標準では**無し** | **使用しない。ポーリングで成立させる** |
| タグ | StampFly（**別配線した場合**） | あり得る | **対応を準備しておく。使えるなら使う** |
| タグ | 単体 M5StampS3A | あり（G7） | 使ってよい |

**原則: IRQ（Interrupt ReQuest、割り込み要求）はポーリングを「置き換える」のではなく「選択肢として足す」。
ポーリング経路は常に第一級の実装として残す。**

## 根拠

### アンカーで IRQ が取れる理由
AtomS3 のピン予算は **8本**（底面 G5/G6/G7/G8/G38/G39 + Grove G1/G2）。
必要なのは SPI（Serial Peripheral Interface）4本 + RST + IRQ + I2C（Inter-Integrated Circuit）2本(ToF 距離センサ（Time-of-Flight 方式の測距センサ）) = **8本ちょうど**で収まる。

| | SPI | RST / IRQ | ToF (I2C) | ToF の接続 |
|---|---|---|---|---|
| **構成A** | G5-G8 | Grove G1/G2 | G38/G39 | 底面へ手配線 |
| **構成B** | G5-G8 | G38/G39 | Grove G1/G2 | **Grove に挿すだけ** |

- **AtomS3R** は G38/G39 に何も繋がっていないので **構成B が綺麗**
- **無印 AtomS3** は G38/G39 が MPU6886 の I2C なので **構成A** を推奨

### タグで IRQ が取れない理由

**前提となる方針**: タグのハードは StampFly 互換を維持する、という制約を意図的に課している
（`docs/PLAN.md` §1、2026-08-21 ユーザ指示）。だからタグ単体構成で IRQ が取れる場合でも、
**IRQ を前提にした実装にはしない**。

StampFly で外部に出ているのは **GROVE の4本（G13/G15/G1/G2）のみ**。
SPI 4線で使い切るため IRQ 線が残らない。
（かつて「空きGPIO G5/G10/G41/G42」としていたのは誤り。この4本はモータPWM）

> **略語**: GPIO（General Purpose Input/Output、汎用入出力）／PWM（Pulse Width Modulation、パルス幅変調）

### ただし可能性は残す
現行ファームが使っていない GPIO は **G1, G2, G6, G8, G11, G13, G15 の7本**。
Grove の4本以外に **G6 / G8 / G11** が未使用である。

#### 【訂正 2026-08-21】G6 / G8 / G11 は IRQ の候補にならない
「ファームが使っていない」と「基板上でどこにも繋がっていない」は**別の話**であり、
この3本は**いずれも StampFly 基板上で IC に配線済み**である。

| GPIO | 基板上の接続先 |
|---|---|
| **G6** | 底面 ToF (VL53L3CX) の **INT** |
| **G8** | 前方 ToF (VL53L3CX) の **INT** |
| **G11** | BMI270 の **INT1** |

出典: `third_party/stampfly_ecosystem/firmware/vehicle/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md:112-140`
（`docs/STAMPFLY_INTEGRATION.md:738-745` に同じ結論が既に書かれていた）。
**コネクタには出ていないので、そのままでは転用できない。**

#### タグで IRQ を取る現実的な経路は HW-2
`docs/STAMPFLY_INTEGRATION.md` §5.3 の **HW-2**: SPI 3線を M5StampS3A の
キャステレーション/基板上の SPI2 配線（G44=SCK / G14=MOSI / G43=MISO）から分岐させ、
**空いた GROVE 4本を CS / IRQ / RSTn / WAKEUP に回す**。
半田付けが要り、飛行制御の生命線である IMU と SPI バスを共有するリスクがある。
**実機で M5StampS3A のパッドに物理的にアクセスできるかは未確認。**

具体的な値は `boards/stampfly.h` にコメントアウトした形で置いてある。

## 実装要件

### 1. 両経路を第一級で持つ
- IRQ 駆動とポーリングの**両方**を実装し、切り替え可能にする
- **`pin_irq == UWB_PORT_PIN_UNUSED` なら、設定に関わらずポーリングへフォールバック**する
- ポーリング経路は「劣化版」ではなく、正規の動作モードとして保守する

### 2. 遅延値は IRQ の有無に紐づける
DS-TWR（Double-Sided TWR、両側二方向測距）は**両側に遅延送信の締切がある**:

| | 誰が守る | 設定 | IRQ で詰められるか |
|---|---|---|---|
| Poll → Response | **アンカー** | `responseTxDelayUus` | **できる** |
| Response → Final | **タグ** | `finalTxDelayUus` | タグに IRQ があれば |

したがって遅延値は**役割ごと・IRQ の有無ごとのプリセット**にする。
ハードコードしない。

| 構成 | アンカー | タグ | 1周(5台) | レート |
|---|---:|---:|---:|---:|
| 現状（両側ポーリング） | 3077µs | 1846µs | 31.9ms | 31.3 Hz |
| **アンカーのみ IRQ**（標準構成） | **900µs** | **1400µs** | **16.8ms** | **59.4 Hz** |
| 両側 IRQ（StampFly で別配線できた場合） | 900µs | 700µs | 11.1ms | 90.2 Hz |

> **⚠ この表の数値は実マイクロ秒である。** `RangeConfig` / `DSRangeConfig` の `*Uus` フィールドは **UUS（UWB microsecond）**
> （1 UUS = 1.02564 µs）なので、**この表の値をそのまま代入してはいけない**。換算済みの実際の設定値は
> `docs/TIMING_PRESETS.md` §2 の表を使うこと。単位の詳細は `docs/UNITS.md`。

### 3. 【重要】遅延値はタグとアンカーで一致していなければならない
`responseTxDelayUus` はアンカーが「いつ送るか」であり、
タグは `responseRxAfterTxDelayUus` / `rxTimeoutUus` で「いつ待つか」を決める。
**片方だけ書き換えると測距が成立しなくなる。**

対策:
- 遅延プリセットに**バージョン番号**を持たせる
- 起動時のログとフレームに載せ、**不一致を検出したら警告する**
- `GETTING_STARTED.md` に「アンカーとタグは必ず同じプリセットで焼くこと」を明記

### 4. `boards/stampfly.h`（作成済み）
- **`pin_irq` を持たせ、既定は `UWB_PORT_PIN_UNUSED`**
- 別配線できた場合に備えた値をコメントで示す。**ただし G6 / G8 / G11 ではない**
  （上の【訂正 2026-08-21】のとおり、この3本は基板上で IC に配線済みでコネクタに出ていない）。
  示すのは **HW-2**（SPI を M5StampS3A のパッドへ逃がして GROVE 4本を空ける）の値
- 併せて `pin_rst` も同様（現状は取れない）

## 実装状況（2026-08-21 時点）

**IRQ を「起床信号」として使う経路を実装済み。** 設計どおり、ステータス
レジスタの判読・フレーム照合・エラー処理はポーリング経路と完全に同一の
コードを通り、IRQ は各待ちループの `vTaskDelay(pdMS_TO_TICKS(1))` を
置き換える起床トリガとしてのみ使われる。

- **`components/uwb_port`**（`uwb_port.h` / `uwb_port.c`）に
  `uwb_port_irq_enable()` / `uwb_port_irq_disable()` /
  `uwb_port_irq_available()` / `uwb_port_irq_clear_pending()` /
  `uwb_port_irq_wait()` を追加した。ISR (`IRAM_ATTR`) は
  `xSemaphoreGiveFromISR()` のみを行い、SPI・ログ呼び出しは一切しない。
  `gpio_config()` は `intr_type = GPIO_INTR_POSEDGE`（アクティブHIGH前提。
  極性は実機未検証）、`pull_down_en = GPIO_PULLDOWN_ENABLE`（未配線時の
  フロート対策）。`gpio_install_isr_service()` が既にインストール済み
  （`ESP_ERR_INVALID_STATE`）の場合は成功として扱う。
- **`components/uwb_qm33120`**: `Config::use_irq`（既定 `false`）を追加。
  `Qm33120::init()` が PHY 設定の直後に `uwb_port_irq_enable()` を呼び、
  成功したら `dwt_setinterrupt(DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO
  | SYS_STATUS_ALL_RX_ERR, 0, DWT_ENABLE_INT)` で3種のイベント
  （RX成功・RXタイムアウト・RXエラー）に対して割り込みを有効化する。
  失敗した場合は `ESP_LOGW` を出してポーリングのまま継続する（`init()`
  自体は失敗させない）。実際に有効化できたかは `Qm33120::irqActive()` で
  取得できる。`end()` は `uwb_port_irq_disable()` を必ず呼ぶ（
  `port_already_initialized==true` で `uwb_port` 自体の所有権が無い場合も、
  IRQ の有効/無効はこのクラスが責任を持つ）。
- **`dwt_setcallbacks()` / `dwt_isr()` は意図的に使っていない。**
  復号経路（ステータス判読・フレーム取得・エラー処理）をポーリングと
  IRQ とで1本に保つため（本ファイル冒頭「ポーリング経路は劣化版ではなく
  正規の動作モードとして保守する」）。SDKコールバック経路を使うと
  復号ロジックが2本立てになり、検証コストが倍になる。
- **`components/uwb_qm33120/src/uwb_qm33120_twr.cpp`** の4関数
  （`requestRange` / `respondRange` / `requestDSRange` / `respondDSRange`）
  すべての受信待ちループで `vTaskDelay(pdMS_TO_TICKS(1))` を
  `(void)uwb_port_irq_wait(1)` に置き換え、各ループの直前で
  `uwb_port_irq_clear_pending()` を呼ぶ。自分の送信完了
  （`DWT_INT_TXFRS_BIT_MASK`）を待つループ（`respondRange()` の
  Response送信後、`respondDSRange()` のDWD結果送信後）や結果フレーム
  再送間隔（`resultRepeatGapMs`）の `vTaskDelay` は対象外
  （`dwt_setinterrupt()` で有効化しているのはRX系のみで、TXFRSでは
  IRQが来ないため）。`Qm33120::receiveFrame()` の受信待ちループも同じ
  方針で置き換えた。
- **`uwb_port_irq_wait()` は IRQ が無効なとき
  `vTaskDelay(pdMS_TO_TICKS(timeout_ms))` と完全に等価に振る舞う。**
  したがって `use_irq==false`（既定）、`pin_irq` 未配線、ISR登録失敗の
  いずれでも、待ちループの挙動はIRQ対応前とビット単位で同一になる。
- **Kconfig**: `firmware/anchor` / `firmware/tag` / `firmware/twr` の
  `main/Kconfig.projbuild` に共通シンボル `UWB_ENABLE_IRQ`（既定 `n`）を
  追加した。`main.cpp` は `cfg.use_irq = CONFIG_UWB_ENABLE_IRQ` 相当の値を
  設定し、起動ログには**設定値ではなく `Qm33120::irqActive()` が返す
  実際の有効状態**（`irq=active (pin=N)` / `irq=polling (pin_irq unwired)`
  / `irq=polling (disabled by Kconfig)` / `irq=polling (enable failed)`）
  を出す。
- **既定は全ファームで無効（ポーリング）のまま。** 実機での極性検証
  （アクティブHIGH前提が正しいか）がまだ済んでいないため。既定を上げる
  かどうかは Phase 1〜2 の実機検証後に判断する。
- タグ側（StampFly / `boards/stampfly.h`）は `pin_irq` が
  `UWB_PORT_PIN_UNUSED` のままなので、`UWB_ENABLE_IRQ=y` でビルドしても
  自動的にポーリングへフォールバックする（仕様どおり）。
