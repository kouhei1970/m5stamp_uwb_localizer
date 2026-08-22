# 割り込み（IRQ）の使用方針 (2026-08-21 確定、2026-08-22 タグ側 IRQ 対応を反映)

## 仕様

| 役割 | ボード | IRQ | 方針 |
|---|---|---|---|
| **アンカー**（既定） | M5StampS3A + StampS3 BreakOut（既定 Kconfig `UWB_ANCHOR_BOARD_STAMPS3`） | **あり（G7）** | **積極的に使用する** |
| アンカー（代替） | AtomS3 / AtomS3R（構成A/B の Kconfig は現役のまま残る） | あり（G2 または G38/G39） | 積極的に使用してよい |
| **タグ** | StampFly（M5StampS3A を搭載したドローン機体。背面 12P FPC 経由） | **あり（G16）**（2026-08-22 確定、`boards/stampfly.h`） | 使えるが**既定はポーリング**（理由は下記） |
| タグ | 単体 M5StampS3A | あり（G7） | 同上 |

**原則: IRQ（Interrupt ReQuest、割り込み要求）はポーリングを「置き換える」のではなく「選択肢として足す」。
ポーリング経路は常に第一級の実装として残す。**

> **2026-08-22 更新: この原則を維持する理由が変わった。**
> 旧: 「StampFly はハード的に IRQ 線を取れないので、そもそもポーリングしか選べない」。
> 新: 「アンカー・タグとも IRQ 線自体は取れるようになったが、**IRQ の極性（アクティブHIGH前提）
> が実機で一度も検証されていない**ため」（`components/uwb_port/src/uwb_port.c`
> `uwb_port_irq_enable()` の `GPIO_INTR_POSEDGE` 周辺コメント。DW3720 のアクティブHIGH根拠と
> 「この極性は実機でまだ検証していない」という留保が書かれている）。
> 誤っていた場合の帰結は「常にタイムアウト待ちになる」または「即座に IRQ で起床し続ける」
> （後者も実害は `vTaskDelay(1)` 相当に留まる想定、とはいえ検証はしていない）。
> **結論は変わらない: 既定のプリセットは `PollingBoth` のままで、ポーリング経路を第一級の実装として保守する。**
> 実機で Phase 1〜2 の検証が済んでから `AnchorIrq` → `BothIrq` の順に既定を上げる方針
> （`docs/EXPERIMENT_PLAN.md`）も変わらない。

## 根拠

### アンカーで IRQ が取れる理由（既定構成: M5StampS3A + StampS3 BreakOut）
既定のアンカー構成は M5StampS3A を StampS3 BreakOut に載せたもの（`boards/stamps3.h`、
既定 Kconfig `UWB_ANCHOR_BOARD_STAMPS3`）。BreakOut は M5StampS3A の 1.27mm ピンを 2.54mm へ
変換するだけの基板で、露出 GPIO は 23 本（G0-G15, G39-G44, G46）あり、そのうち
**G7 がオンボード周辺機能と競合しない空きヘッダ GPIO として IRQ に使える**
（`boards/stamps3.h` 冒頭コメント。RST=G6, IRQ=G7, WAKEUP=G8 も同様に空き）。
据置きボードで配線に余裕があるため IRQ を積極的に使ってよい。

#### 代替: AtomS3 の場合
AtomS3 / AtomS3R は削除されておらず、Kconfig の構成A/B も引き続き代替ボードとして残っている。
ピン予算は **8本**（底面 G5/G6/G7/G8/G38/G39 + Grove G1/G2）。
必要なのは SPI（Serial Peripheral Interface）4本 + RST + IRQ + I2C（Inter-Integrated Circuit）2本(ToF 距離センサ（Time-of-Flight 方式の測距センサ）) = **8本ちょうど**で収まる。

| | SPI | RST / IRQ | ToF (I2C) | ToF の接続 |
|---|---|---|---|---|
| **構成A** | G5-G8 | Grove G1/G2 | G38/G39 | 底面へ手配線 |
| **構成B** | G5-G8 | G38/G39 | Grove G1/G2 | **Grove に挿すだけ** |

- **AtomS3R** は G38/G39 に何も繋がっていないので **構成B が綺麗**
- **無印 AtomS3** は G38/G39 が MPU6886 の I2C なので **構成A** を推奨

### タグの IRQ 事情（2026-08-22 更新: 背面 FPC 経由に決定）

**前提が変わった。** 旧方針は「タグのハードは StampFly 互換（GROVE 2系統4本のみ）を維持する」
という制約の下で書かれており、その制約下では GROVE の4本を SPI で使い切るため IRQ 線が
残らず、「タグでは IRQ が取れない」がそのまま事実だった。

**2026-08-22 のユーザ決定により、StampFly 搭載タグの接続経路は M5StampS3A 背面の 12P FPC
（`boards/stampfly.h`）に変更され、旧 GROVE 4本構成は廃案になった。** 背面 FPC 経由なら
SPI3_HOST の4本に加えて **G33=RSTn / G16=IRQ / G17=WAKEUP** が取れる（`boards/stampfly.h`
冒頭コメント）。**これにより「タグでは IRQ が取れない」という前提はもう成り立たない。**

以下は廃案になった旧 GROVE 構成を検討していた際の経緯であり、現在の結論（背面 FPC を使う）
には影響しないが、記録として残す（`docs/README.md` 約束ごと規則6）:

#### 【訂正 2026-08-21】G6 / G8 / G11 は IRQ の候補にならない（経緯）
GROVE 構成を検討していた当時、現行ファームが使っていない GPIO として G1, G2, G6, G8, G11,
G13, G15 の7本を挙げ、Grove の4本以外に G6 / G8 / G11 が未使用と考えたが、これは誤りだった。
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

#### HW-2（飛行系 SPI2 への相乗り）の議論（経緯、いまは不要）
`docs/STAMPFLY_INTEGRATION.md` §5.3 の **HW-2**: SPI 3線を M5StampS3A の
キャステレーション/基板上の SPI2 配線（G44=SCK / G14=MOSI / G43=MISO）から分岐させ、
空いた GROVE 4本を CS / IRQ / RSTn / WAKEUP に回すという案。半田付けが要り、飛行制御の生命線
である IMU と SPI バスを共有するリスクがあった。実機で M5StampS3A のパッドに物理的に
アクセスできるかも未確認のままだった。

**背面 FPC 経路（`boards/stampfly.h`）は StampFly が使っていない専有線
（G16/G17/G18/G33-G37）だけで完結し、飛行系(SPI2_HOST)とバスを一切共有しない。
GROVE 4本の制約も、HW-2 が抱えていた「飛行制御の生命線と同じバスを共有する」リスクも
両方とも不要になった。** HW-2 の具体値は `boards/stampfly.h` にはもう残していない。

## 実装要件

### 1. 両経路を第一級で持つ
- IRQ 駆動とポーリングの**両方**を実装し、切り替え可能にする
- **`pin_irq == UWB_PORT_PIN_UNUSED` なら、設定に関わらずポーリングへフォールバック**する
- ポーリング経路は「劣化版」ではなく、正規の動作モードとして保守する
- **既定でポーリングを選ぶ理由は「IRQ 線が無いから」ではない**（アンカー・タグとも今は
  IRQ 線自体は取れる）。**「IRQ の極性が実機で未検証だから」**である
  （`components/uwb_port/src/uwb_port.c` `uwb_port_irq_enable()` の `GPIO_INTR_POSEDGE`
  周辺コメント。詳細は本文書冒頭のコールアウト参照）。既定を上げるかどうかは Phase 1〜2 の
  実機検証後に判断する。

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
| 両側 IRQ（StampFly 背面 FPC 経由。`docs/TIMING_PRESETS.md` `BothIrq`） | 900µs | 700µs | 11.1ms | 90.2 Hz |

> **⚠ この表の数値は実マイクロ秒である。** `RangeConfig` / `DSRangeConfig` の `*Uus` フィールドは **UUS（UWB microsecond）**
> （1 UUS = 1.02564 µs）なので、**この表の値をそのまま代入してはいけない**。換算済みの実際の設定値は
> `docs/TIMING_PRESETS.md` §2 の表を使うこと。単位の詳細は `docs/GLOSSARY.md`。

### 3. 【重要】遅延値はタグとアンカーで一致していなければならない
`responseTxDelayUus` はアンカーが「いつ送るか」であり、
タグは `responseRxAfterTxDelayUus` / `rxTimeoutUus` で「いつ待つか」を決める。
**片方だけ書き換えると測距が成立しなくなる。**

対策:
- 遅延プリセットに**バージョン番号**を持たせる
- 起動時のログとフレームに載せ、**不一致を検出したら警告する**
- `GETTING_STARTED.md` に「アンカーとタグは必ず同じプリセットで焼くこと」を明記

### 4. `boards/stampfly.h`（2026-08-22 更新: 背面 FPC 経由）
- `pin_irq` = G16、`pin_rst` = G33、`pin_wakeup` = G17 を**実際の GPIO 番号として**持つ
  （以前のような `UWB_PORT_PIN_UNUSED` 既定や、HW-2 用のコメントアウトされた仮の値ではない）。
  背面 12P FPC のみで完結するため、GROVE 経由の値や HW-2 の値はもう残していない。
- ただし `use_irq`（Kconfig `UWB_ENABLE_IRQ`）の既定は他ボードと同じく `n`。**IRQ 線が
  取れることと、既定で有効にするかは別の話**（上記「実装要件 1」の極性未検証の理由による）。

## 実装状況（2026-08-21 実装、2026-08-22 タグ側ピン定義を反映）

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
  極性は実機未検証）、`pull_down_en = GPIO_PULLDOWN_DISABLE`（モジュール上の
  外付けプルアップに対抗できないため、詳細は下記「IRQ 外付けプルアップ」節）。
  `gpio_install_isr_service()` が既にインストール済み
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
- **2026-08-22 更新**: タグ側（StampFly / `boards/stampfly.h`）は背面 FPC 決定により
  `pin_irq` が `UWB_PORT_PIN_UNUSED` ではなく実際の GPIO（G16）を指すようになった。
  ただし Kconfig `UWB_ENABLE_IRQ` の既定は `n` のままなので、明示的に有効化しない限り
  引き続きポーリングで動く（仕様どおり）。CI には `UWB_ENABLE_IRQ=y` +
  `UWB_TIMING_PROFILE_BOTH_IRQ=y` を組み合わせた `anchor-stamps3-ds-bothirq` /
  `tag-stampfly-ds-bothirq` のビルドが追加されている（`.github/workflows/build.yml`、
  `docs/TIMING_PRESETS.md`）。両者は必ずペアで焼くこと（片側だけだと測距が成立しない）。

## 【重要】モジュール上の IRQ 外付けプルアップについて（2026-08-21、公式回路図で判明）

公式回路図（`assets/SCH_UWB_MODULE_SCH_main_V0.2_...pdf`。詳細は
`docs/WIRING.md` §5.5(4)）を確認したところ、**M5Stamp UWB Module 上に
DW_IRQ を VCC_3V3 へプルアップする抵抗 R2（10kΩ）が実装されている**ことが分かった。

- **動作中は問題にならない**: QM33120W の IRQ は既定でアクティブ HIGH の
  push-pull 出力なので、チップ自身がピンを能動的に駆動している間はこの
  外付けプルアップの影響を受けない。上記「実装状況」の `GPIO_INTR_POSEDGE`
  前提はそのまま有効。
- **SLEEP / DEEPSLEEP 中は IRQ ピンが H に張り付く**: チップが低消費電力状態に
  入って IRQ 出力が Hi-Z（または非駆動）になると、モジュール上の 10kΩ
  プルアップにより IRQ ピンは High に固定される。QM33120W データシートには
  「sleep するには IRQ が low/inactive でなければならない」旨の記載があり、
  **この外付けプルアップはその条件と逆方向に働く部品**である。
- **`uwb_port.c` の内部プルダウンは実質無意味**: 従来 `gpio_config()` に
  `pull_down_en = GPIO_PULLDOWN_ENABLE`（未配線時のフロート対策）を設定して
  いたが、内部プルダウン（ESP32-S3 で概算 45kΩ 程度）はモジュール上の
  10kΩ プルアップに負けるため、**IRQ ピンを Low 側へ倒す効果はほぼ無く、
  3.3V を (10k+45k) で分圧した約 60µA のリーク電流を常時流すだけ**になる。
  2026-08-21 に `GPIO_PULLDOWN_DISABLE` へ修正した
  （`components/uwb_port/src/uwb_port.c`）。
- **`GPIO_INTR_POSEDGE`（アクティブ HIGH）前提は「動作中のみ」有効**という
  留保が付く。現状の実装（起床信号としてのみ IRQ を使い、チップを能動的に
  SLEEP させる運用はしていない）では実害は無いが、**将来 SLEEP/WAKEUP を
  積極的に使う設計にする場合は、この 10kΩ プルアップの影響
  （SLEEP 復帰直後に IRQ が誤って High のまま読める可能性、Wakeup 直後の
  ノイズ耐性など）を要再検討**。
