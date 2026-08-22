# 遅延プリセットとバージョン不一致検出（設計、2026-08-21）

`docs/IRQ_POLICY.md` §実装要件 2 / 3 の実装仕様。**実装前の設計ノートであり、
実機検証は済んでいない**（本リポジトリの他の数値と同じ扱い）。

> 略語は `docs/GLOSSARY.md` を参照。

---

## 0. なぜプリセットが要るか

DS-TWR（Double-Sided TWR、両側二方向測距）は**両側に遅延送信の締切がある**。

| 区間 | 締切を守る側 | 設定フィールド | 相手側が対応して持つ設定 |
|---|---|---|---|
| Poll → Response | **アンカー** | `responseTxDelayUus` | タグの `responseRxAfterTxDelayUus` / `rxTimeoutUus` |
| Response → Final | **タグ** | `finalTxDelayUus` | アンカーの `finalRxAfterResponseTxDelayUus` / `rxTimeoutUus` |
| Final → Result(DWD) | アンカー（即時送信） | なし | タグの `resultRxAfterFinalTxDelayUus` |

**片側だけ値を変えると測距が成立しない。** しかも症状は「距離が出ない」だけで、
どちらが悪いのかログからは分からない。だから

1. 値は**役割 × IRQ（Interrupt ReQuest、割り込み要求）有無のプリセット**として1か所で定義し、
2. **プリセットの版番号と種別をフレームに載せて、受信側が不一致を検出する。**

## 1. 締切がどこから来るか（数値の根拠）

すべての `*Uus` は UUS（UWB microsecond）単位（1 UUS = 512/499.2 µs = 1.02564 µs）。**UUS とは何か、なぜ実 µs と
違うのか、Qorvo 公式の `*_UUS` 定数がなぜそのまま使えないのかは `docs/GLOSSARY.md` を必ず先に読むこと。**
以下の導出は**実 µs** で行い、最後に UUS へ直す。

### 1.1 フレームの長さ

> **略語**: SHR（Synchronization Header、同期ヘッダ）／SFD（Start-of-Frame Delimiter、フレーム開始区切り）／
> PHR（PHY Header、物理層ヘッダ）／FCS（Frame Check Sequence、フレーム検査系列）

- SHR（preamble 128 + SFD 8 = 136 シンボル × 1017.63 ns）= **138.4 µs**
- 短いフレーム全長（SHR + PHR + データ + FCS）≈ **179 µs**
  （出典: `docs/archive/CRITICAL_REVIEW.md`「フレーム air time の実測算」。
  SHR は `docs/refs/DW3000_Datasheet_wayback.txt` Table 18 から独立に再検算済み）
- したがって **RMARKER（Ranging Marker、＝タイムスタンプの基準）はフレーム先頭から 138.4 µs、
  フレーム末尾の 41 µs 手前**にある。

### 1.2 受信窓を開く締切
`dwt_setrxaftertxdelay(D)` は**自分の送信が終わってから** D 後に RX を開く。
一方 `responseTxDelayUus` (= R) は**相手の RX タイムスタンプ（RMARKER）から** R 後に送る。

相手のフレームの**プリアンブル先頭**が自分に届くのは、自分の送信終了を基準にして

```
R − (frame_len − SHR) − SHR  =  R − frame_len  ≈  R − 179 µs
```

**⇒ `responseRxAfterTxDelayUus` は実 µs で `R − 179 µs` より小さくなければならない。**
（同じ式が `finalRxAfterResponseTxDelayUus` と `finalTxDelayUus` の関係にも当てはまる）

### 1.3 折返しに要る時間（＝ R / F の下限）
送信側が「相手のフレームを受信したと気づいてから、遅延送信を予約し終える」までの時間。

| 待ち方 | 気づくまでの遅れ | SPI 数回 + 計算 | 合計 |
|---|---:|---:|---:|
| **ポーリング**（`vTaskDelay(1ms)`、tick=1000Hz） | **最大 1000 µs** | 100〜200 µs | **〜1.2 ms** |
| **IRQ**（GPIO 割り込み → タスク通知） | 数十 µs | 100〜200 µs | **〜0.3 ms** |

ポーリング経路が `vTaskDelay(pdMS_TO_TICKS(1))` で回っていることは
`components/uwb_qm33120/src/uwb_qm33120_twr.cpp`（`respondDSRange()` の Poll 待ちループ）
で確認済み。tick 1000Hz は `firmware/*/sdkconfig.defaults` の `CONFIG_FREERTOS_HZ=1000`。

これが「両側ポーリングなら 3077 µs / 1846 µs、アンカーだけ IRQ なら 900 µs / 1400 µs」
（`docs/IRQ_POLICY.md` の表）の出どころである。

### 1.3.1 Qorvo 公式サンプルとの照合

`docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/` を `grep -a` で確認した実値
（**Qorvo の `*_UUS` 定数は実マイクロ秒**。→ `docs/GLOSSARY.md` §3）:

| 定数 | 値（実 µs） | 出典 |
|---|---:|---|
| `POLL_RX_TO_RESP_TX_DLY_UUS` | **900** | `ex_05b_ds_twr_resp/ds_twr_responder.c:85` |
| `RESP_TX_TO_FINAL_RX_DLY_UUS` | **500** | 同 :87 |
| `FINAL_RX_TIMEOUT_UUS` | 220 | 同 :89 |
| `POLL_TX_TO_RESP_RX_DLY_UUS` | 300 + CPU | `ex_05a_ds_twr_init/ds_twr_initiator.c:85` |
| `RESP_RX_TO_FINAL_TX_DLY_UUS` | 300 + CPU | 同 :88 |

**本プリセットの `AnchorIrq` の 900 µs は、Qorvo 公式 DS-TWR responder と完全に同じ値**
である（偶然ではなく、IRQ 駆動の折返し時間が同じ桁だから）。

一方 **タグ側の `finalTxDelayUus` は Qorvo の 300 µs に対して 1400 µs と大きい**。
これは Qorvo のサンプルが `while` でステータスレジスタを読み続ける（ビジーウェイト）のに対し、
本実装は `vTaskDelay(1ms)` で他タスクへ CPU を譲るため。タグ側にも IRQ がある `BothIrq`
では 700 µs まで詰められるが、Qorvo と同じ 300 µs には**しない**
（本実装は FreeRTOS のタスク切替を挟むため）。

### 1.4 【落とし穴】DWD（結果フレーム）は IRQ があると**早く来る**
DWD はアンカーが Final を受信した直後に `DWT_START_TX_IMMEDIATE` で即時送信する
（`respondDSRange()`）。つまり**アンカーの折返しが速くなるほど DWD は早く着く**。

タグから見て DWD のプリアンブル先頭が届くのは、自分の Final 送信終了を基準に

```
τ(アンカーの折返し) + ToF − 41 µs
```

| | τ | DWD 到達（Final 送信終了基準） | `resultRxAfterFinalTxDelayUus` |
|---|---:|---:|---:|
| ポーリング | 〜1.2 ms | 〜1.16 ms 後 | 200 UUS (205 µs) で間に合う |
| **IRQ** | 〜0.3 ms | **〜0.26 ms 後、速ければ 60 µs 後** | **200 UUS では開くのが遅すぎる → 0 にする** |

**`resultRxAfterFinalTxDelayUus` は IRQ プリセットでは 0（送信直後に RX を開く）にする。**
値を小さくしすぎて困ることは無い（受信機が少し長く ON になるだけ）が、
大きすぎると**プリアンブルの先頭を取り逃して丸ごと失敗する**。非対称なので 0 側に倒す。

---

## 2. プリセット表（確定値）

単位はすべて **UUS**。`hostTimeoutMs` は全プリセットで 10（`docs/archive/REIMPL_PLAN.md` R9）。

### 2.1 `RangeConfig`（SS-TWR）

| フィールド | `PollingBoth` | `AnchorIrq` | `BothIrq` |
|---|---:|---:|---:|
| `responseTxDelayUus` | 3000 | **878** | **878** |
| `responseRxAfterTxDelayUus` | 500 | **400** | **400** |
| `rxTimeoutUus` | 4500 | **1200** | **1200** |

### 2.2 `DSRangeConfig`（DS-TWR、本番運用）

| フィールド | `PollingBoth` | `AnchorIrq` | `BothIrq` |
|---|---:|---:|---:|
| `responseTxDelayUus` | 3000 | **878** | **878** |
| `responseRxAfterTxDelayUus` | 1500 | **400** | **400** |
| `finalTxDelayUus` | 1800 | **1365** | **683** |
| `finalRxAfterResponseTxDelayUus` | 500 | 500 | **200** |
| `resultRxAfterFinalTxDelayUus` | 200 | **0** | **0** |
| `rxTimeoutUus` | 3000 | **1200** | **1200** |
| `resultRepeatCount` | 1 | 1 | 1 |

**`PollingBoth` の列は現在の既定値と完全に一致する**（`uwb_qm33120_types.hpp` の
メンバ初期化子）。したがってこのプリセットを既定にする限り**挙動は一切変わらない**。

### 2.3 導出の検算

878 UUS = 900.5 µs、1365 UUS = 1400.0 µs、683 UUS = 700.5 µs、1200 UUS = 1230.8 µs（実測換算値）。

| 窓 | プリセット | 開く時刻 | プリアンブル到達 | フレーム終了 | 窓の終わり | 判定 |
|---|---|---:|---:|---:|---:|---|
| タグの Response 待ち | AnchorIrq | 410 µs | 900−179 = 721 µs | 900 µs | 410+1231 = 1641 µs | ✅ |
| アンカーの Final 待ち | AnchorIrq | 513 µs | 1400−179 = 1221 µs | 1400 µs | 513+1231 = 1744 µs | ✅ |
| タグの Response 待ち | BothIrq | 410 µs | 721 µs | 900 µs | 1641 µs | ✅ |
| アンカーの Final 待ち | BothIrq | 205 µs | 700−179 = 521 µs | 700 µs | 205+1231 = 1436 µs | ✅ |
| タグの DWD 待ち | IRQ 両方 | 0 µs | 60〜260 µs | 〜440 µs | 1231 µs | ✅ |

`BothIrq` で `finalRxAfterResponseTxDelayUus` を 500 UUS(513 µs) のまま使うと
プリアンブル到達 521 µs に対して余裕が 8 µs しか無い。**だから 200 UUS に下げる。**

**注意（PRETOC）**: 上の表はフレーム待ちタイムアウト（`rxTimeoutUus` /
`dwt_setrxtimeout()`）だけを検算したものであり、プリアンブル検出タイムアウト
（PRETOC、`dwt_setpreambledetecttimeout()`）は含んでいない。PRETOC は「RX が
開放された時刻」を起点に走るタイマーで、上の表の「開く時刻」から「プリアンブル
到達」までの時間より短いと必ず RXPTO で失敗する。本プリセット（DS-TWR）では
PRETOC は無効（0）にしてある（`uwb_qm33120_twr.cpp:531, 841`。
`docs/REVIEW_2026-08-21.md` §0 #1、`docs/archive/REIMPL_PLAN.md` R9）。有効にする場合は、
公式サンプルの値をそのまま持ち込むのではなく、上表の「開く時刻」から
「プリアンブル到達」までの時間を PAC 単位に換算して設定する必要がある。

1周（5アンカー、DS-TWR）の概算: poll 0.18 + R 0.9 + resp 0.18 + F 1.4 + final 0.18
+ 折返し 0.3 + DWD 0.18 ≈ **3.3 ms/台 → 5台で 16.6 ms ≈ 60 Hz**。
`docs/IRQ_POLICY.md` の「アンカーのみ IRQ = 16.8 ms / 59.4 Hz」と一致する。

---

## 3. バージョンと種別をフレームに載せる

### 3.1 何を送るか
```
kTimingPresetVersion : uint8_t   // プリセット表の版。表の数値を1つでも変えたら上げる
TimingProfile        : uint8_t   // 0=PollingBoth, 1=AnchorIrq, 2=BothIrq
```

**送るのは「Kconfig で設定した種別」ではなく「実際に適用した種別」**
（§4 のフォールバックを経た後の値）。設定値を送ると、フォールバックが起きたときに
「合っているはずなのに測距できない」状態を検出できなくなる。

### 3.2 どのフレームに載せるか
**Poll と Response の両方**。

| フレーム | 現在のペイロード | 変更後 | 検出する側 |
|---|---|---|---|
| `"TWP"` / `"DWP"`（Poll） | 3 バイト | **5 バイト**（+version +profile） | **アンカー** |
| `"TWR"` / `"DWR"`（Response） | 11 バイト | **13 バイト**（末尾に +version +profile） | **タグ** |
| `"DWF"`（Final） | 15 バイト | 変更なし | — |
| `"DWD"`（結果） | 7 バイト | 変更なし | — |

Poll に載せる理由: **交換の最初のフレームであり、遅延の影響を受けずに必ず届く**。
Response にも載せる理由: 運用中に人が見ているのは**タグの JSON Lines 出力**なので、
タグ側でも警告が出せる方が実用的。

追加は 2 バイト（air time にして約 2.4 µs）で、§2.3 の余裕に対して無視できる。

### 3.3 受信側の扱い（重要）
`detail::payloadMatches()` はペイソード長を**厳密一致**で見る。そのまま長さを増やすと
**旧版の相手からのフレームは「不一致」として黙って捨てられ、警告すら出ない**
（症状は「応答が無い」だけ）。それでは不一致検出にならない。

**⇒ 受信側は新旧どちらの長さも受理する:**
- Poll: ペイロード長 **3 または 5** を受理。3 なら「版情報を持たない旧ファーム」として警告。
- Response: ペイロード長 **11 または 13** を受理。11 なら同様に警告。

不一致を検出しても**測距は続行する**（`docs/IRQ_POLICY.md`「不一致を検出したら警告する」）。
拒否すると切り分けが余計に難しくなるため。

警告は**同じ相手について 1 回だけ**出す（毎周期 5 台分の警告でログが埋まるのを防ぐ）。

---

## 4. フォールバックの規則（2つを混同しない）

| | 対象 | 規則 |
|---|---|---|
| **(a) 待ち方のフォールバック** | IRQ 待ちか、ポーリング待ちか | **`pin_irq == UWB_PORT_PIN_UNUSED` または ISR 登録失敗なら、設定に関わらずポーリング。** ローカルに閉じた話なので自動で落として良い（`docs/IRQ_POLICY.md` 実装要件 1） |
| **(b) 遅延プリセット** | §2 の数値 | **自動で変えてはいけない。** 相手と一致していることが要件なので、片側が勝手に変えたら §0 の破綻そのものになる |

(b) が要件を満たせないとき（例: `AnchorIrq` を選んだのに `pin_irq` が未配線）は、
**起動ログで目立つ警告を出す**が、値は変えない。

```
W (1234) uwb_anchor: timing profile=AnchorIrq は IRQ 前提だが pin_irq が未配線。
                     待ちはポーリングへフォールバックした。折返しが 1.2ms かかるため
                     responseTxDelayUus=878 (900us) の締切に間に合わない可能性が高い。
                     タグ・アンカー双方を PollingBoth で焼き直すこと。
```

## 5. 設定の入口

`Kconfig`（5アプリ共通のシンボル名。`boards/atoms3.h` のピン構成と同じ方式）:

```
choice UWB_TIMING_PROFILE
    prompt "TWR 遅延プリセット"
    default UWB_TIMING_PROFILE_POLLING_BOTH
    config UWB_TIMING_PROFILE_POLLING_BOTH   bool "両側ポーリング（31 Hz、実機未検証の既定）"
    config UWB_TIMING_PROFILE_ANCHOR_IRQ     bool "アンカーのみ IRQ（59 Hz、標準構成）"
    config UWB_TIMING_PROFILE_BOTH_IRQ       bool "両側 IRQ（90 Hz、タグ側にも IRQ が必要）"
endchoice
```

**2026-08-22 更新**: `BothIrq` はタグ側にも IRQ が必要な構成で、以前は「タグを別配線できた
場合」という条件付きの扱いだった。プリセット値・版不一致検出・Kconfig の切り替え自体は
`docs/TIMING_PRESETS.md` 初版の時点から全部入っており、**単に対応するハードが存在しなかった
だけ**である。StampFly 搭載タグの接続経路が M5StampS3A 背面の 12P FPC 経由に変更され、G16 の
IRQ が実際に取れるようになったことで（`boards/stampfly.h`、`docs/IRQ_POLICY.md`）、`BothIrq`
は条件付きの選択肢ではなく実際に成立する構成になった。

**既定は `PollingBoth`**。現在の既定値と同一で挙動が変わらないため。
実機で Phase 1〜2 が通ってから `AnchorIrq` を既定に上げる。`BothIrq` を試す場合も同様に
実機検証を経てから選ぶこと（本ドキュメントの数値はいずれも実機未検証）。

`docs/GETTING_STARTED.md` に「**アンカーとタグは必ず同じプリセットで焼くこと**」を明記する。

### 5.1 CI ビルド（`.github/workflows/build.yml`）
CI の firmware マトリクスには `AnchorIrq`（`-fast`）と同様に `BothIrq` のペアも追加されている:

| ペア | 用途 |
|---|---|
| `anchor-stamps3-ds-fast` / `tag-stamps3-ds-fast` | `AnchorIrq`（59 Hz） |
| `anchor-stamps3-ds-bothirq` / `tag-stampfly-ds-bothirq` | `BothIrq`（90 Hz、タグは StampFly 背面 FPC 前提） |

`-fast` のペアと同じく、**`-bothirq` のペアも必ずセットで焼くこと**。片側だけ `BothIrq` の
遅延値を焼くと、相手側は別のプリセットの締切で待つことになり測距が成立しない（§0）。

---

## 6. 実装の置き場所

| 追加/変更 | 場所 |
|---|---|
| `TimingProfile` enum、`kTimingPresetVersion`、プリセット表、`applyTimingProfile()` | **新規** `components/uwb_qm33120/include/uwb_qm33120_timing.hpp` |
| プリセットの数値が §2 の表と一致することのテスト、§1.2 の締切式の検算 | `tests/host/pipeline` |
| Poll/Response のペイロード生成・照合 | `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` |
| 長さ 3/5・11/13 の受理 | `components/uwb_qm33120/src/uwb_qm33120_internal.hpp` の `payloadMatches()` 周辺 |
| Kconfig と起動ログ | `firmware/{twr,tag,anchor}/main/` |

`uwb_qm33120_timing.hpp` は `uwb_qm33120_units.hpp` / `uwb_qm33120_frame_match.hpp` と同じ方針で
**ESP-IDF / Qorvo SDK ヘッダに依存させない**（`<cstdint>` のみ）。
ホスト側 `tests/host/pipeline` からそのまま include して検算できるようにするため。

---

## 7. 実装状況（2026-08-21 実装）

本ドキュメントの設計は**実装済み**。実際のシンボル名は以下の通り。

| 設計上の要素 | 実際のシンボル | 場所 |
|---|---|---|
| `TimingProfile` enum、`kTimingPresetVersion`、プリセット表 | `uwb::TimingProfile`、`uwb::kTimingPresetVersion`、`uwb::timingPresetSs()`/`timingPresetDs()`、`uwb::timingProfileName()`/`timingProfileNeedsAnchorIrq()`/`timingProfileNeedsTagIrq()`/`timingProfileValid()` | `components/uwb_qm33120/include/uwb_qm33120_timing.hpp`（新規、ESP-IDF非依存） |
| `applyTimingProfile()`、`Config::timing_profile` | `uwb::applyTimingProfile(RangeConfig&, TimingProfile)` / `uwb::applyTimingProfile(DSRangeConfig&, TimingProfile)`、`uwb::Config::timing_profile`（既定 `PollingBoth`） | `components/uwb_qm33120/include/uwb_qm33120_types.hpp` |
| 長さ3/5・11/13の受理 | `uwb::detail::payloadMatchesEither()`、`uwb::detail::readTimingTag()`（既存の `payloadMatches()` は無変更） | `components/uwb_qm33120/src/uwb_qm33120_internal.hpp` |
| Poll/Response への版/種別付与、送受信、不一致検出・警告 | `requestRange()`/`respondRange()`/`requestDSRange()`/`respondDSRange()` 内、`checkTimingTagAndWarn()`（同ファイル内の無名namespace） | `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` |
| 不一致を1回だけ警告するための状態 | `Qm33120::Impl::warned_peers[8]` / `warned_count` | `components/uwb_qm33120/src/uwb_qm33120_impl.hpp` |
| Kconfig（3アプリ共通） | `choice UWB_TIMING_PROFILE`（`UWB_TIMING_PROFILE_POLLING_BOTH` / `_ANCHOR_IRQ` / `_BOTH_IRQ`） | `firmware/{anchor,tag,twr}/main/Kconfig.projbuild` |
| 起動時ログ・§4(b)の起動時チェック | `timingProfileName()`/`timingProfileNeedsAnchorIrq()`/`timingProfileNeedsTagIrq()` を使った `ESP_LOGI`/`ESP_LOGW` | `firmware/{anchor,tag,twr}/main/main.cpp` の `app_main()` |
| プリセット表・締切式・往復のホスト検算 | シナリオ13〜17（188件中の一部。旧132件から+56） | `tests/host/pipeline/test_pipeline.cpp` |

**既知の制約（実機未検証）**: 本ドキュメントの数値導出と同じく、実機での検証はまだ済んでいない。
`PollingBoth` は現在の既定値と数値上完全に一致するため実装前後で挙動は変わらないが、
`AnchorIrq`/`BothIrq` は実機で Phase 1〜2 の検証が済むまで既定にしない方針は変わらない。

**ホスト検算の限界**: `RangeConfig`/`DSRangeConfig`（`uwb_qm33120_types.hpp`）と
`payloadMatchesEither()`/`readTimingTag()`（`uwb_qm33120_internal.hpp`）は、既存の `payloadMatches()`
と同じ理由で ESP-IDF ヘッダ（`uwb_port.h` 経由の `driver/spi_master.h`、`esp_timer.h`、`deca_device_api.h`）
に依存しており、ホストから直接 include できない。`tests/host/pipeline` のシナリオ14/16/17は、
実体のロジックを模した局所ミラー実装（値・アルゴリズムを1行ずつ突き合わせてコメントで出典を明記）で検算している。
シナリオ13/15（`uwb_qm33120_timing.hpp` 自体の検算）はこの制約を受けず、実体の関数を直接呼んでいる。
