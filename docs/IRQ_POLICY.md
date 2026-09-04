# 割り込み（IRQ）の使用方針

## 仕様

**全ボードで IRQ が取れる。したがって IRQ（受信待ちの起床信号としての使用）は
既定で有効にする**（`UWB_ENABLE_IRQ` default y。2026-08-29〜30 に実機で通電確認済み。
下記「既定を IRQ にしたことの risk」）。

遅延プリセットの既定は当初 `BothIrq`（約 90 Hz）としていたが、実機で `BothIrq` の
素の成功率が極端に低い（6.8Mbps 構成で 1.0%）ことが判明したため、**現在の既定は
`PollingBoth`** である（`firmware/anchor` / `firmware/tag` の Kconfig と
`Config::timing_profile`。評価用 `firmware/twr` のみ `BothIrq` のまま。経緯は
`docs/HANDOFF.md` §0-C、プリセットの正本は `docs/TIMING_PRESETS.md`）。

| 役割 | ボード | IRQ ピン |
|---|---|---|
| **アンカー**（標準） | M5StampS3A + StampS3 BreakOut | **G7** |
| **タグ**（StampFly 搭載） | M5StampS3A 背面 12P FPC | **G16** |
| タグ（単体） | M5StampS3A | **G7** |

ピン定義は `boards/stamps3.h` / `boards/stampfly.h`、配線は `docs/WIRING.md`。

**ポーリング経路は残すが、位置づけは「フォールバック」である。**
消さない理由:

1. `pin_irq` を配線しない個体・構成があり得る
2. ISR 登録に失敗しても測距だけは続けられるようにしておきたい
3. （2026-08-28 追加）ISR 登録に成功しても IRQ 線が実際に動作しているとは限らない。
   `verifyIrqLine()` の実測に落ちても測距だけは続けられるようにしておきたい
   （詳細は下記「実装要件 1」）

（当初は「IRQ 極性の実機通電確認がまだ」も理由の一つだったが、2026-08-29〜30 に
実機確認済みとなり解消した。）

`uwb_port_irq_wait()` は IRQ が無効なとき `vTaskDelay()` と完全に等価に
振る舞うので、**待ちループのコードは1本**である。IRQ の有無で復号経路が
分岐することはない。

---

## 【重要】既定を IRQ にしたことの risk

**IRQ の極性は一次資料（データシート）と実機の両方で裏付けが取れている**
（実機: 2026-08-29 の拡張 probe で 2 台とも `L5=PASS(irq=active)`〈IRQ 自己診断込み〉、
2026-08-30 に IRQ 有効構成での測距成立と、ポーリング比較で同等の成功率。
`docs/HANDOFF.md`）。実装は `GPIO_INTR_POSEDGE`（アクティブ HIGH 前提）。従来この
節は根拠を Qorvo SDK のコメント（`components/qm33120w_sdk/deca_device_api.h`
「The IRQ line has to be low/inactive (i.e., no pending events) otherwise
device will not enter sleep」から active は HIGH と類推）だけに置いていたが、
これは状況証拠であって直接の裏付けではなかった。**2026-08-28、Qorvo 公式
データシートの本文で直接裏付けが取れた:**

> `assets/QM33120W Data Sheet.pdf` (Rev. D, 2025-10) p.6, Section 2 "Pin
> Configuration and Descriptions", Table 1, ball B1 (IRQ/GPIO8):
> "Interrupt Request output from the QM33120W to the host processor. By
> default, IRQ is an active-high output but may be configured to be
> active-low if required. For correct operation in SLEEP and DEEPSLEEP
> modes, it should be configured for active-high operation. This pin will
> float in SLEEP and DEEPSLEEP states and may cause spurious interrupts
> unless pulled low."

**IRQ は既定でアクティブ HIGH の出力**であり、`GPIO_INTR_POSEDGE` の想定と
一致している。「一次資料でチップの既定仕様を確認した」ことと「この個体で
実際にその極性で動作していることを通電して確認した」ことは別だが、
**後者も 2026-08-29〜30 の実機セッションで確認できた**（上記）。

**遅延プリセットは、2026-08-28 から既定で IRQ の実際の有効／無効に追従する
ようになった**（`docs/TIMING_PRESETS.md` §4(b)、
`Config::downgrade_timing_profile_when_polling` 既定 `true`）。待ちが
ポーリングへ落ちたときは、`AnchorIrq`/`BothIrq` を要求していても `init()` が
自動的に `PollingBoth` へ降格するため、既定のままなら「IRQ が期待どおり
動かないまま `BothIrq` の遅延で動作する」状態にはならない。この既定を `false`
にして旧方針（自動で変えない・警告のみ）へ戻した場合は、従来どおり
**IRQ が期待どおり動かないまま `BothIrq` の遅延で動作すると測距が全く成立
しない**ままなので注意（`docs/TIMING_PRESETS.md` §4(b)）。

### 測距が出ないときの切り分け

起動ログの `irq=` 行で実際の状態が分かる（`firmware/anchor/main/main.cpp`、
`firmware/tag/main/main.cpp`）:

| ログ | 意味 |
|---|---|
| `irq=active (pin=N)` | IRQ が有効。想定どおり |
| `irq=polling (pin_irq unwired)` | `boards/*.h` の `pin_irq` が `UWB_PORT_PIN_UNUSED` |
| `irq=polling (disabled by Kconfig)` | `CONFIG_UWB_ENABLE_IRQ=n` |
| `irq=polling (enable failed)` | ISR 登録に失敗、**または（2026-08-28〜）`verifyIrqLine()` の実測に失敗**。ログの文言だけではどちらか区別できない（`Qm33120::irqActive()` はどちらの場合も false を返す）。切り分けるには `Qm33120` タグの `irq self-test: ...` 行（`ESP_LOGW`/`ESP_LOGI`）を別途見る（`docs/TIMING_PRESETS.md` §4(a)） |

**2026-08-28 以降、既定（`downgrade_timing_profile_when_polling=true`）では
`irq=polling` になると遅延プリセットも自動で `PollingBoth` へ降格するため、
「`irq=polling` なのに遅延プリセットが `BothIrq` のまま」という組み合わせは
基本的に起きない。** 起動ログの `timing profile=` 行（`docs/TIMING_PRESETS.md`
§5）で実際に適用された値を確認すること。この既定を `false` にしている場合や
旧バージョンのファームでは、従来どおりこの組み合わせが原因になりうる。
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

**IRQ が使えないときはポーリングへ自動的にフォールバックする。**
`pin_irq == UWB_PORT_PIN_UNUSED`、ISR（Interrupt Service Routine、割り込みが
起きたときに呼ばれる処理）の登録失敗に加えて、**（2026-08-28 追加）
`Qm33120::verifyIrqLine()` による実測で IRQ 線が動作していないと判定された場合**
も、設定に関わらずポーリングへ落ちる。判定条件の全体像・2段階の実測手順の
詳細は `docs/TIMING_PRESETS.md` §4(a) を参照。

追加が必要だった理由: **「ISR を登録できた」ことは「IRQ が使える」ことを意味
しない。** `uwb_port_irq_enable()` の成功は GPIO ドライバへ割り込みハンドラを
登録できたことしか確認しておらず、QM33120W の IRQ 出力ピン（DW_IRQ）がホスト
側の `pin_irq` に物理的に配線されているか、極性（アクティブ HIGH の想定）が
合っているかまでは検出できない。従来はこの状態でも `irqActive()` が true を
返し、起動ログには `irq=active` と出ていたが、実際には全ての待ちが 1 ms タイム
アウトへ何の兆候もなく劣化していた（症状を検知する手段が無かった）。`verifyIrqLine()` は
`init()` の直後に実際にエッジが届くかを能動的に測ることで、この検出漏れを
塞ぐ。**2026-08-29 の拡張 probe 実行で、2 台とも自己診断が通ること
（`L5=PASS(irq=active)`）を実機確認済み。**

### 2. 遅延値は IRQ の有無に紐づける
折返し時間が変わるので、プリセットは IRQ の有無とセットで選ぶ
（`docs/TIMING_PRESETS.md`）。

| プリセット | 想定 | レート |
|---|---|---|
| `BothIrq` | タグ・アンカーとも IRQ（**旧既定**。実機での成功率が極端に低く既定から外した。`docs/HANDOFF.md` §0-C） | 約 90 Hz |
| `AnchorIrq` | アンカーのみ IRQ | 約 59 Hz |
| `PollingBoth` | **現在の既定。**どちらもポーリング相当の遅延（IRQ 有効時もこの遅延で安全に使える） | 約 31 Hz |

### 3. 【重要】遅延値はタグとアンカーで一致していなければならない
片側だけ変えると測距が成立しない。しかも症状は「距離が出ない」だけで
どちらが悪いのかログからは分からないため、フレームに版番号と種別を載せて
相手局の値と比較し、不一致なら警告する仕組みが入っている
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
  一次資料での裏付けは済み・実機の通電確認は未実施。上記「既定を IRQ に
  したことの risk」参照）、`pull_down_en = GPIO_PULLDOWN_DISABLE`（モジュール上の
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
  IRQ の有効/無効はこのクラスが責任を持つ）。**（2026-08-28 追加）** 割り込み
  有効化に成功した直後、`Qm33120::verifyIrqLine()` で実際にエッジが届くかを
  実測し、失敗すればここで割り込みを無効化して `irqActive()` を false に戻す
  （判定条件・2段階の実測手順は `docs/TIMING_PRESETS.md` §4(a)）。
- **`dwt_setcallbacks()` / `dwt_isr()` は意図的に使っていない。**
  復号経路（ステータス判読・フレーム取得・エラー処理）をポーリングと
  IRQ とで1本に保つため（本ファイル冒頭「ポーリング経路は劣化版ではなく
  正規の動作モードとして保守する」）。SDKコールバック経路を使うと
  復号ロジックが2本立てになり、検証コストが倍になる。
- **`components/uwb_qm33120/src/uwb_qm33120_twr.cpp`** の4関数
  （`requestRange` / `respondRange` / `requestDSRange` / `respondDSRange`）
  すべての受信待ちループで `vTaskDelay(pdMS_TO_TICKS(1))` を
  `(void)uwb_port_irq_wait(1)` に置き換え、各ループの直前で
  `uwb_port_irq_clear_pending()` を呼ぶ。自分自身の送信完了
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
- **IRQ の既定は全ファームで有効。** 遅延プリセットの既定は `PollingBoth`
  （本番 `firmware/anchor` / `firmware/tag`。評価用 `firmware/twr` のみ `BothIrq` のまま）。
  極性は一次資料（データシート）と実機（2026-08-29〜30）の両方で確認済み。
  駄目だったときの切り分け手順は上の「既定を IRQ にしたことの risk」を参照。
  **（2026-08-28 追加）** 待ちがポーリングへ落ちたときは、既定で遅延プリセット
  も自動的に `PollingBoth` へ降格する（`Config::downgrade_timing_profile_
  when_polling`、`docs/TIMING_PRESETS.md` §4(b)）。

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
- **（2026-08-28 追記）DW3720 側にも内部プルがあり、向きは逆**。データシート
  で確認できた:

  > `assets/QM33120W Data Sheet.pdf` (Rev. D, 2025-10) p.29, Section 5.13:
  > "All the GPIO pins have a software controllable internal pull up/down
  > resistor to ensure safe operation when input pins are not driven. This
  > defaults to enabled and pull-down except for the SPICSn pin which defaults
  > to pull-up. The value of the pull-up/down will vary with the VDD1 supply
  > voltage over a range from 10 kΩ to 30 kΩ."

  つまり **DW3720 の IRQ には既定で 10〜30kΩ の内部プルダウンが有効**であり、
  モジュール上の 10kΩ 外付けプルアップと引っ張り合う。ピンが駆動されていない
  状態（SLEEP/DEEPSLEEP、電源投入直後）では、10k:10k なら約 1.65V、
  10k:30k なら約 2.48V という**中間電位になりうる**。ただし IRQ は
  4〜6mA で能動駆動される出力なので（同 p.11 Table 5 "Digital Output Drive
  Current" の `GPIOx, IRQ` 行）、**駆動中はどちらのプル抵抗も勝てず、
  動作中の判定には影響しない**。本ファームは `dwt_configuresleep()` を
  呼んでおらずスリープに入らないため、現状この中間電位の条件には入らない。
- **`GPIO_INTR_POSEDGE`（アクティブ HIGH）前提は「動作中のみ」有効**という
  留保が付く。現状の実装（起床信号としてのみ IRQ を使い、チップを能動的に
  SLEEP させる運用はしていない）では実害は無いが、**将来 SLEEP/WAKEUP を
  積極的に使う設計にする場合は、この 10kΩ プルアップの影響
  （SLEEP 復帰直後に IRQ が誤って High のまま読める可能性、Wakeup 直後の
  ノイズ耐性など）を要再検討**。
