# 割り込み（IRQ）の使用方針

## 仕様

**全ボードで IRQ が取れる。したがって IRQ を既定とし、遅延プリセットの既定も
`BothIrq`（約 90 Hz）にする。**

| 役割 | ボード | IRQ ピン |
|---|---|---|
| **アンカー**（標準） | M5StampS3A + StampS3 BreakOut | **G7** |
| アンカー（代替） | AtomS3 / AtomS3R | G2（構成A）/ G38・G39（構成B） |
| **タグ**（StampFly 搭載） | M5StampS3A 背面 12P FPC | **G16** |
| タグ（単体） | M5StampS3A | **G7** |

ピン定義は `boards/stamps3.h` / `boards/stampfly.h` / `boards/atoms3.h`、
配線は `docs/WIRING.md`。

**ポーリング経路は残すが、位置づけは「フォールバック」である。**
消さない理由は3つ:

1. IRQ の極性が実機で未検証（下記）
2. `pin_irq` を配線しない個体・構成があり得る
3. ISR 登録に失敗しても測距だけは続けられるようにしておきたい

`uwb_port_irq_wait()` は IRQ が無効なとき `vTaskDelay()` と完全に等価に
振る舞うので、**待ちループのコードは1本**である。IRQ の有無で復号経路が
分岐することはない。

---

## 【重要】既定を IRQ にしたことの risk

**IRQ の極性は実機で一度も検証していない。** 実装は `GPIO_INTR_POSEDGE`
（アクティブ HIGH 前提）で、根拠は Qorvo SDK の記述
（`components/qm33120w_sdk/deca_device_api.h`「The IRQ line has to be
low/inactive (i.e., no pending events) otherwise device will not enter
sleep」＝ active は HIGH）だが、実機で確かめてはいない。

さらに **遅延プリセットは IRQ の実際の有効／無効に追従しない**
（`docs/TIMING_PRESETS.md`）。`BothIrq` は折返し時間を詰めた値なので、
**IRQ が期待どおり動かないまま `BothIrq` の遅延で走ると測距が全く成立しない。**

### 測距が出ないときの切り分け

起動ログの `irq=` 行で実際の状態が分かる（`firmware/anchor/main/main.cpp`、
`firmware/tag/main/main.cpp`）:

| ログ | 意味 |
|---|---|
| `irq=active (pin=N)` | IRQ が有効。想定どおり |
| `irq=polling (pin_irq unwired)` | `boards/*.h` の `pin_irq` が `UWB_PORT_PIN_UNUSED` |
| `irq=polling (disabled by Kconfig)` | `CONFIG_UWB_ENABLE_IRQ=n` |
| `irq=polling (enable failed)` | ISR 登録に失敗 |

**`irq=polling` なのに遅延プリセットが `BothIrq` のままなら、その組み合わせが原因。**
配布バイナリなら `*-polling` 版に差し替える
（`anchor-stamps3-ds-polling` + `tag-stampfly-ds-polling`。**必ずペアで**）。
自分でビルドするなら `CONFIG_UWB_ENABLE_IRQ=n` +
`CONFIG_UWB_TIMING_PROFILE_POLLING_BOTH=y`。

---

## 実装要件

### 1. 復号経路は1本に保つ
ステータスレジスタの判読・フレーム照合・エラー処理は、IRQ の有無に関わらず
同一のコードを通す。IRQ は各待ちループの起床トリガとしてのみ使う。
`dwt_setcallbacks()` / `dwt_isr()` は意図的に使わない（使うと復号ロジックが
2本立てになり検証コストが倍になる）。

### 2. 遅延値は IRQ の有無に紐づける
折返し時間が変わるので、プリセットは IRQ の有無とセットで選ぶ
（`docs/TIMING_PRESETS.md`）。

| プリセット | 想定 | レート |
|---|---|---|
| `BothIrq` | **既定。**タグ・アンカーとも IRQ | 約 90 Hz |
| `AnchorIrq` | アンカーのみ IRQ | 約 59 Hz |
| `PollingBoth` | どちらもポーリング | 約 31 Hz |

### 3. 【重要】遅延値はタグとアンカーで一致していなければならない
片側だけ変えると測距が成立しない。しかも症状は「距離が出ない」だけで
どちらが悪いのかログからは分からないため、フレームに版番号と種別を載せて
相手と比較し、不一致なら警告する仕組みが入っている
（測距自体は続行する。`docs/TIMING_PRESETS.md`）。

---

## 実装状況

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
- **`components/uwb_qm33120`**: `Config::use_irq` を追加（`main.cpp` が Kconfig の値を渡す）。
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
  したがって `use_irq==false`、`pin_irq` 未配線、ISR登録失敗の
  いずれでも、待ちループの挙動はIRQ対応前とビット単位で同一になる。
- **Kconfig**: `firmware/anchor` / `firmware/tag` / `firmware/twr` の
  `main/Kconfig.projbuild` に共通シンボル `UWB_ENABLE_IRQ`（既定 `y`）を
  追加した。`main.cpp` は `cfg.use_irq = CONFIG_UWB_ENABLE_IRQ` 相当の値を
  設定し、起動ログには**設定値ではなく `Qm33120::irqActive()` が返す
  実際の有効状態**（`irq=active (pin=N)` / `irq=polling (pin_irq unwired)`
  / `irq=polling (disabled by Kconfig)` / `irq=polling (enable failed)`）
  を出す。
- **既定は全ファームで有効。** 遅延プリセットの既定も `BothIrq`。
  極性の実機検証は済んでいないので、駄目だったときの切り分け手順は
  上の「既定を IRQ にしたことの risk」を参照。

## 【重要】モジュール上の IRQ 外付けプルアップ

**M5Stamp UWB Module 上に DW_IRQ を VCC_3V3 へプルアップする抵抗 R2（10kΩ）が
実装されている**（出典: 公式回路図 `assets/SCH_UWB_MODULE_SCH_main_V0.2_...pdf`。
詳細は `docs/WIRING.md` §7.3）。

- **動作中は問題にならない**: QM33120W の IRQ は既定でアクティブ HIGH の
  push-pull 出力なので、チップ自身がピンを能動的に駆動している間はこの
  外付けプルアップの影響を受けない。上記「実装状況」の `GPIO_INTR_POSEDGE`
  前提はそのまま有効。
- **SLEEP / DEEPSLEEP 中は IRQ ピンが H に張り付く**: チップが低消費電力状態に
  入って IRQ 出力が Hi-Z（または非駆動）になると、モジュール上の 10kΩ
  プルアップにより IRQ ピンは High に固定される。QM33120W データシートには
  「sleep するには IRQ が low/inactive でなければならない」旨の記載があり、
  **この外付けプルアップはその条件と逆方向に働く部品**である。
- **内部プルダウンは効かない**: ESP32-S3 の内部プルダウン（概算 45kΩ 程度）は
  モジュール上の
  10kΩ プルアップに負けるため、**IRQ ピンを Low 側へ倒す効果はほぼ無く、
  3.3V を (10k+45k) で分圧した約 60µA のリーク電流を常時流すだけ**になる。
  そのため `GPIO_PULLDOWN_DISABLE` にしてある
  （`components/uwb_port/src/uwb_port.c`）。
- **`GPIO_INTR_POSEDGE`（アクティブ HIGH）前提は「動作中のみ」有効**という
  留保が付く。現状の実装（起床信号としてのみ IRQ を使い、チップを能動的に
  SLEEP させる運用はしていない）では実害は無いが、**将来 SLEEP/WAKEUP を
  積極的に使う設計にする場合は、この 10kΩ プルアップの影響
  （SLEEP 復帰直後に IRQ が誤って High のまま読める可能性、Wakeup 直後の
  ノイズ耐性など）を要再検討**。
