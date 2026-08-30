# 遅延プリセットとバージョン不一致検出（設計）

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

### 1.3.2 実機での検算: `txMarginUs`（2026-08-28 追加）

上の §1.3 が置いた折返し時間（ポーリング 約1.2ms / IRQ 約0.3ms）はいずれも
**見積もりであって実測ではなかった**。**遅延送信の締切までの残り時間を、
チップ自身の時計で直接測る**診断値 `txMarginUs` を `ResponderResult` /
`DSRangeResult` / `DSResponderResult`（`uwb_qm33120_types.hpp`）に追加した。

計算は `detail::delayedTxMarginUs(dxTime)`
（`components/uwb_qm33120/src/uwb_qm33120_internal.hpp`）が行う。`dxTime` は
`dwt_setdelayedtrxtime()` に渡した値（40bit システム時刻の上位32bit = DX_TIME
レジスタと同じ単位・同じ原点）で、これを `dwt_readsystimestamphi32()`
（SYS_TIME レジスタの上位32bit）から符号付き32bitで引く。1単位 = 256 DTU
（Device Time Unit、§1 冒頭で触れた単位系リファレンス `docs/GLOSSARY.md` の
DTU）= 256 × DWT_TIME_UNITS ≈ 4.0064 ns なので、µs に換算して返す。符号付き
32bit で引くことで、約17.2秒周期のシステム時計の巻き戻りをまたいでも正しい
残り時間になる（標準的な wraparound-safe な実装方法）。戻り値が負なら、その
時点で既に締切を過ぎている——すなわち §4(b) 2. で述べた「SDK が送信を取り
消す」状態にある。

測っている箇所は3つ:

| 関数 | 測る遅延送信 | 対応する締切 |
|---|---|---|
| `respondRange()` | Response の遅延送信（SS-TWR アンカー） | `responseTxDelayUus` |
| `requestDSRange()` | Final の遅延送信（DS-TWR タグ） | `finalTxDelayUus` |
| `respondDSRange()` | Response の遅延送信（DS-TWR アンカー） | `responseTxDelayUus` |

`firmware/twr` の SS-TWR ANCHOR ログ（`SS_RESP_STAT`）は、成功・失敗いずれの
行にも `tx_margin_us=` を出す。失敗時に `error=TxStartFailed` かつ
`tx_margin_us` が負であれば、「Poll は聞こえていたのに折返しが締切に間に合わ
ず送信が取り消された」ことの直接の証拠になる。

**これにより、§1.3 が見積もりで置いた折返し時間（ポーリング 約1.2ms / IRQ
約0.3ms）を、実機で `tx_margin_us` と `responseTxDelayUus` の実µs値の差
（＝ `responseTxDelayUus`（実µs）− `tx_margin_us` = 実際にかかった折返し時
間）として検算できるようになった。**

（実装: `components/uwb_qm33120/src/uwb_qm33120_internal.hpp` の
`detail::delayedTxMarginUs()`、`components/uwb_qm33120/src/uwb_qm33120_twr.cpp`
の `respondRange()` / `requestDSRange()` / `respondDSRange()`、
`components/uwb_qm33120/include/uwb_qm33120_types.hpp` の `txMarginUs` メンバ、
`firmware/twr/main/main.cpp` の `SS_RESP_STAT` ログ。**本節の記述はビルド確認
のみで実機未検証。** §1.3 の見積もりと実測が実際にどの程度一致するか、締切
超過時に本当に「RXFTO のみ」の失敗として観測されるかは、実機での確認が済ん
でいない。）

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

## 2. プリセット表（確定値、版2）

単位はすべて **UUS**。`hostTimeoutMs` は SS-TWR (`RangeConfig`) で全プリセット 10
（`docs/archive/REIMPL_PLAN.md` R9）、DS-TWR (`DSRangeConfig`) で全プリセット **20**
（2026-08-29 DS-TWR原因特定で10→20、下記§2.4参照。`DSRangeConfig::hostTimeoutMs`
のフィールドコメント参照）。

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
| `finalTxDelayUus` | **3000** | **1365** | **683** |
| `finalRxAfterResponseTxDelayUus` | **1500** | 500 | **200** |
| `resultRxAfterFinalTxDelayUus` | 200 | **0** | **0** |
| `rxTimeoutUus` | 3000 | **1200** | **1200** |
| `resultRepeatCount` | 1 | 1 | 1 |

**`PollingBoth` の列は現在の既定値と完全に一致する**（`uwb_qm33120_twr_config.hpp` の
メンバ初期化子）。したがってこのプリセットを既定にする限り**挙動は一切変わらない**。

`finalTxDelayUus`（1800→**3000**）・`finalRxAfterResponseTxDelayUus`
（500→**1500**）は2026-08-29に変更した（`kTimingPresetVersion` 1→2）。**`AnchorIrq`・
`BothIrq` の DS 列は未検証のまま**（6.8 Mbps・preamble 128 前提で導出されたもので、
850 kbps・preamble 256 での再検証は行っていない。§2.4 参照）。

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

### 2.4 【2026-08-29 追記】850 kbps / preamble 256 での再導出（`finalTxDelayUus` 1800→3000）

本番機の実運用 PHY（850 kbps・preamble 256、`firmware/twr` の `anc_850_p256` 等）
は §1〜§2.3 の導出が前提にしていた 6.8 Mbps・preamble 128 とはフレーム長が
大きく異なる。DS-TWR 実測 0〜23%（SS-TWR は同条件で 99.95%）の机上解析
（`docs/HANDOFF.md` §0-C(2)）で、DS PollingBoth の `finalTxDelayUus`（旧 1800
UUS）だけが 850 kbps/256 向けに再検算されていなかったことが分かったので、
Response 側（`responseTxDelayUus`=3000 UUS、実証済み）と同じ手順で導き直す。

**フレーム長（850 kbps / preamble 256）**:

- SHR = preamble 256 + SFD 8 = 264 シンボル × 1.0176 µs/シンボル ≈ **269 µs**
  （シンボル長は §1.1 の preamble 128 の場合と同じ 1017.63 ns/シンボルを使用。
  preamble 長だけが変わる）。
- PHR ≈ **21.3 µs**（850 kbps の PHY ヘッダ長）。
- データ部（850 kbps） = (8×バイト数 + 48) × 1.0256 µs。
- RMARKER から見た「フレーム末尾までの時間」= PHR + データ部
  （RMARKER は SHR の終端＝PHR の先頭に立つ。§1.1 参照）:

  | フレーム | 総バイト数（ヘッダ+ペイロード+FCS） | RMARKER 後 |
  |---|---:|---:|
  | Poll ("DWP") | 16 B | ≈ **0.20 ms** |
  | Response ("DWR") | 24 B | ≈ **0.27 ms** |
  | Final ("DWF") | 26 B | ≈ **0.28 ms** |
  | DWD ("DWD") | 18 B | ≈ **0.22 ms** |

**折返しに要る時間（ポーリング、§1.3 と同じ内訳）**: 気づくまでの遅れ ≤ 1 ms
（`vTaskDelay(pdMS_TO_TICKS(1))` 粒度）+ SPI/計算 ≈ 0.25 ms。

**DW3000 UM §9.4.1 のリード時間規則**: 遅延送信の予約時刻（DX_TIME）は
「現在時刻 + プリアンブル長 + SFD 長 + 20 µs」以上先でなければならない。
すなわち最低リード時間 ≈ SHR（269 µs）+ 20 µs ≈ **289 µs**。これを下回ると
HPDWARN すら立たず、`dwt_starttx()` は DWT_SUCCESS を返すのに実際には
送信されない（`uwb_qm33120_internal.hpp` の `detail::delayedTxWedged()`
コメント参照）。

**旧 `finalTxDelayUus`=1800 UUS（≈1846.2 µs 実 µs）の余裕**: DX_TIME は
Response の RMARKER（タグが受信した `respRxTs`）から 1846.2 µs 後。タグが
実際に `dwt_starttx()` を呼ぶまでの折返し時間は上記の内訳から最大 ≈1.25 ms
程度まで伸びうるため、リード時間の余裕（DX_TIME − 289 µs − 実際の折返し
時間）は数十 µs 〜 1 ms 程度まで薄くなる（詳細な実測ベースの区間は
`docs/HANDOFF.md` §0-C(2) 参照）。850 kbps/256 では Final フレーム自体も
preamble 128/6.8 Mbps 時より長くなっている分、この薄い余裕がジッタで
容易に負へ振れうる。

**新 `finalTxDelayUus`=3000 UUS（≈3076.9 µs 実 µs）**: Response 側と同じ値
にすることで、Response 側で既に実証済みの余裕（1.3〜2.3 ms）をそのまま
Final 側にも与える。

**アンカーの Final 受信窓（`finalRxAfterResponseTxDelayUus`=1500 UUS ≈1538.5 µs、
`rxTimeoutUus`=3000 UUS ≈3076.9 µs）が Final フレーム全体を覆うことの確認**
（Response 送信終了時刻を基準 T0 とする。Response の RMARKER 後の残り
0.27 ms は上表より）:

| | 時刻（T0基準） |
|---|---:|
| 窓が開く | T0 + 1538.5 µs ≈ **T0+1.54 ms** |
| Final のプリアンブル先頭到達（タグ視点のDX_TIMEはR+3076.9µs、Rからここまでの折返し・伝搬を無視した近似） | ≈ T0+2.5〜2.8 ms 程度（タグ側の折返し時間に依存） |
| Final フレーム終端（プリアンブル先頭 + SHR269 + RMARKER後0.28ms） | 上記 + ≈0.55 ms |
| 窓が閉じる（RXFTO） | T0 + 1538.5 + 3076.9 ≈ **T0+4.62 ms** |

窓は Final のプリアンブル到達より確実に早く開き、フレーム終端よりも
確実に遅く閉じる（RX_FWTO はフレーム受信中も数え続け、途中で打ち切り
うる - UM 記載 - ので、窓はフレーム全体を覆わなければならない。§2.3の
「注意（PRETOC）」と同じ理由）。

**`AnchorIrq`・`BothIrq` の DS 列（`finalTxDelayUus`=1365/683 等）はこの
再検証の対象外**で、6.8 Mbps・preamble 128 前提の値のまま未検証（本項の
850 kbps/256 では使わない組み合わせのため）。IRQ 運用でこの PHY を使う
場合は本項と同じ手順で再導出すること。

`kTimingPresetVersion` を 1→2 に、`DSRangeConfig::hostTimeoutMs` を
10→20 に変更した（`respondDSRange()` の Final 待ちが上表のハードウェア
RXFTO（T0+4.62 ms、Poll 受信からは概算で+7.5 ms 程度）より先にホスト側
タイムアウトで打ち切られないための余裕。フィールドコメント参照）。

#### 受信窓の基準点（2026-08-30 判明）

**上の「アンカーの Final 受信窓」の検算（T0 = Response 送信終了時刻）は正しいが、
2026-08-29 時点では見落としていた前提が 1 つある: `finalRxAfterResponseTxDelayUus`
と `finalTxDelayUus` は基準点が異なる。** これを取り違えたまま `finalTxDelayUus` だけ
旧値 1800 に戻すテストをすると、受信窓側は新値 1500 のままなので Final を毎回取り
こぼす（実機 e30、`docs/HANDOFF.md` §0-C「e30: もう一つの設計ミス」参照、周期成功率
0.0%）。

**基準点の違い**:
- `finalRxAfterResponseTxDelayUus` は `dwt_setrxaftertxdelay()` に渡る。Qorvo
  DW3XXX Software API Guide（`assets/DW3_QM33_SDK_1.1.1/Drivers/` 配下）4p12 §5.3.24
  はこの関数を「delay ... after a frame transmission has completed」（フレーム
  送信が**完了した後**の遅延）と説明しており、基準は**自分（アンカー）の Response
  送信完了時刻**である。
- `finalTxDelayUus` は `dwt_setdelayedtrxtime()` に渡り（× `kUusToDwtTime` で
  DTU 化）、同ガイド §5.3.4 の記載どおり、遅延送信の予約時刻は**RMARKER**（フレームの
  基準時刻、タイムスタンプが打たれる瞬間で同期ヘッダの終わりに立つ）で解釈される。
  つまり基準は**Response の RMARKER**（タグが読み取った受信タイムスタンプ）である。

両者の基準点は「Response の RMARKER」から「Response の送信完了」までの**残り伝送
時間**分だけずれている。850 kbps・24 バイトの Response で再導出すると:

$$\text{PHR} \approx 21.3\,\mu s,\quad
\text{データ部} = (8\times24+48)\times1.0256\,\mu s \approx 246.1\,\mu s$$

$$21.3 + 246.1 \approx \mathbf{267\,\mu s}$$

（§1.1 の「フレーム末尾の 41 µs 手前」は 6.8 Mbps・preamble 128 の短いフレームの値で、
850 kbps・256 では PHR がそのままでデータ部の伝送時間が大きく伸びるため大きく異なる。
Response(24B) の場合の値が本項の 267 µs）。

**Response の RMARKER を基準点 R として再検算した表**（e25〜e30 の実測と一致することを
`docs/HANDOFF.md` §0-C で確認済み）:

| | 1800/500（旧プリセット） | 1800/1500（e30、不成立） | 3000/1500（新プリセット） |
|---|---:|---:|---:|
| アンカーの Final 受信窓が開く（R 基準 = R+267+`finalRxAfterResponseTxDelayUus`） | R+780 µs | R+1806 µs | R+1806 µs |
| Final のプリアンブル先頭到達（R 基準 = R+`finalTxDelayUus`(実µs)−SHR269µs） | R+1577 µs | R+1577 µs | R+2808 µs |
| 差（窓が開く − プリアンブル到達） | −797 µs | **+228 µs** | −1002 µs |
| 判定 | ✅ | ❌（残る SHR は 41 µs のみ→毎回取りこぼす） | ✅ |

**設計則**: アンカーの受信窓が開く時刻は、必ずタグの Final プリアンブル先頭到達より
十分早くなければならない。式で書くと、

$$t_{\text{window open}} = t_{\text{RMARKER,Resp}} + T_{\text{rest,Resp}} + \text{finalRxAfterResponseTxDelayUus}$$

が

$$t_{\text{RMARKER,Final}} - T_{\text{SHR}}$$

より、余裕（目安 0.5 ms 以上）を持って早くなければならない。ここで
$T_{\text{rest,Resp}}$ は Response フレームの RMARKER 後の残り伝送時間（850 kbps・24 バイトで
上記の ≈267 µs）、$t_{\text{RMARKER,Final}}$ はタグが `finalTxDelayUus`（実 µs 換算）で
予約する Final の RMARKER 時刻（= DX_TIME）、$T_{\text{SHR}}$ は Final の SHR 長（850 kbps・
preamble 256 で 269 µs）である。「$T_{\text{rest,Resp}}$ を無視して `finalRxAfterResponseTxDelayUus`
の数値だけを `finalTxDelayUus` と比較する」（旧 Kconfig ヘルプ文の誤り、
`firmware/twr/main/Kconfig.projbuild` で訂正済み）は、この 267 µs 分だけずれた誤った
判定になる。

**一次資料**: Qorvo DW3XXX Software API Guide（`assets/DW3_QM33_SDK_1.1.1/Drivers/`
配下）4p12 §5.3.24（`dwt_setrxaftertxdelay()`: "delay ... after a frame transmission
has completed"）および §5.3.4（delayed TX の予約時刻は RMARKER で解釈される）。

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
| **(a) 待ち方のフォールバック** | IRQ 待ちか、ポーリング待ちか | **次のいずれかに該当すれば、設定に関わらずポーリング。** ローカルに閉じた話なので自動で落として良い（`docs/IRQ_POLICY.md` 実装要件 1） |
| **(b) 遅延プリセット** | §2 の数値 | **2026-08-28 に既定を変更した（詳細は下記）。** 待ちがポーリングへ落ちたときは、既定で IRQ 前提のプリセットも自動的に `PollingBoth` へ降格する |

### (a) 待ち方のフォールバックの判定条件

`init()` が IRQ 待ちを諦めてポーリングへ落とすのは、次の**いずれか**に該当するときである。

1. `pin_irq == UWB_PORT_PIN_UNUSED`（そのボードで IRQ ピンが定義されていない）
2. ISR（Interrupt Service Routine、割り込みが起きたときに呼ばれる処理）の登録に失敗（`uwb_port_irq_enable()` が失敗）
3. **（2026-08-28 追加）`Qm33120::verifyIrqLine()` による実測で IRQ 線が生きていないと判定された**

1・2 は従来からの条件。3 が新設された理由は、1・2 をどちらもクリアしても
「DW_IRQ（QM33120W の IRQ 出力ピン）が本当にホスト側の `pin_irq` に配線されて
いるか」「極性（アクティブ HIGH）の想定が正しいか」までは分からないため。
`uwb_port_irq_enable()` の成功は「ISR を GPIO ドライバへ登録できた」ことしか
意味せず、線が未配線・フローティングのままでも成功する。**従来はこの状態でも
`irqActive()` が true を返し、起動ログには `irq=active` と出ていた**
（実際には全ての待ちが 1ms タイムアウトへ黙って劣化していた）。

`verifyIrqLine()` は `init()` が `uwb_port_irq_enable()` + `dwt_setinterrupt()`
に成功した直後に呼ばれ、2段階で実際にエッジが届くかを測る。「そうでなければ
何がどう見えるか」を先に決めてから測る、という組み方になっている:

- **段階0（前処理）**: `dwt_forcetrxoff()` + `dwt_writesysstatuslo(ALL_RX_TO |
  ALL_RX_ERR | ALL_RX_GOOD | TXFRS)` で SYS_STATUS（チップの状態を示すレジス
  タ）の該当ビットを掃除し、IRQ 線を必ず非アクティブ（Low）へ落とす。これを
  せずに段階1へ入ると、既に立っているビットで線が High に張り付いたまま
  「エッジが来ない」を判定することになり、線が生きていても偽陰性になる。
- **段階1（静穏確認、5 ms）**: 何も事象を起こさずに 5 ms 待つ。この間にエッジ
  が来たら「未配線でフロートしている」または「極性が逆」と判定し、false を
  返す。
- **段階2（事象確認、最大 50 ms）**: `dwt_setrxtimeout(2000 UUS)`（約 2.05 ms）
  を設定して `dwt_rxenable()` で RX を開き、**必ず RXFTO（RX Frame Timeout、
  受信タイムアウト）を起こす**。RXFTO は `init()` が有効化済みの割り込み種別
  （`DWT_INT_RXFCG_BIT_MASK | SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR`）
  に含まれるので、線が生きていれば必ずエッジが1回来るはずである。SYS_STATUS
  に事象（RXFTO・RX エラー・RX 成功のいずれか）が立っているのにエッジが来
  なければ、線は死んでいると判定して false を返す。事象自体が立たなかった
  場合は判定不能として安全側（false・ポーリング）に倒す。

false と判定された場合は `dwt_setinterrupt(..., DWT_DISABLE_INT)` と
`uwb_port_irq_disable()` を呼び戻し、`Qm33120::irqActive()` が false を返す
状態にする。

（実装: `components/uwb_qm33120/src/uwb_qm33120.cpp` の `Qm33120::verifyIrqLine()`。
**本節の記述はビルド確認のみで実機未検証。** IRQ 線が実際に生きている個体で
段階1・2が誤って false を返さないこと、未配線の個体で正しく false へ落ちる
ことは、いずれも実機での確認が済んでいない。）

### (b) 遅延プリセットのフォールバック（2026-08-28 方針変更）

**旧方針（〜2026-08-27）**: 自動で変えない。§2 の数値は据え置き、起動ログで
警告するだけ。根拠は「相手と一致していることが要件なので、片側が勝手に変えた
ら §0 の破綻そのものになる」。警告の例:

```
W (1234) uwb_anchor: timing profile=AnchorIrq は IRQ 前提だが pin_irq が未配線。
                     待ちはポーリングへフォールバックした。折返しが 1.2ms かかるため
                     responseTxDelayUus=878 (900us) の締切に間に合わない可能性が高い。
                     タグ・アンカー双方を PollingBoth で焼き直すこと。
```

**新方針（2026-08-28〜、既定）**: (a) の判定でポーリングへ落ちたとき、要求
プリセットが `AnchorIrq`/`BothIrq` であれば、`init()` が
`Config::timing_profile` を `PollingBoth` へ自動的に書き換える。方針を変更
した理由は次の4点:

1. **据え置きは必ず失敗する。** IRQ プリセットの `responseTxDelayUus = 878
   UUS`（約 900 µs）は §1.3 が見積もった「IRQ 駆動の折返し 約0.3 ms」を前提
   に置いた値である。ところが同じ §1.3 は、ポーリングの折返しを「最大
   1000 µs ＋ SPI 100〜200 µs ≈ 約 1.2 ms」と見積もっている。待ちがポーリング
   へ落ちたのにプリセットだけ `AnchorIrq`/`BothIrq` のままだと、締切を構造的
   に割る。
2. **締切を割った遅延送信は SDK が送信ごと取り消す。**
   `components/qm33120w_sdk/dw3720/dw3720_device.c` の `ull_starttx()` は
   HPDWARN（Half Period Delay Warning、締切超過を示す SYS_STATUS のビット）
   を見て `CMD_TXRXOFF` を発行し、`DWT_ERROR` を返す。フレームは一切送信さ
   れない。相手からは「RXFTO のみが立ち、RXPRD（Preamble Detected、プリアン
   ブル検出）も RXSFDD（SFD Detected、SFD 検出）も立たない」という、電波が
   来ていないのと区別できない失敗に見える。
3. **降格は両機が同じ挙動をする限り一致したままである。** アンカーとタグは
   同じ基板・同じ配線・同じファームであり、(a) の判定条件は各個体のローカル
   なハードウェア状態にしか依存しない。したがって両機が同じ理由で降格する限
   り、降格後も §0 の「両側で値が一致している」という要件は保たれる。
4. **万一片側だけ降格しても、§3 の不一致検出が働く。** 実際に適用した種別
   （降格後の値）は Poll/Response フレームに載って相手へ伝わるので、片側だけ
   降格した場合は §3 の仕組みが不一致を検出して警告する。

降格が起きたときの起動時の警告ログ（ログタグ `Qm33120`、
`components/uwb_qm33120/src/uwb_qm33120.cpp`）:

```
W (1234) Qm33120: timing profile AnchorIrq requires IRQ but the wait is polling;
                  using PollingBoth instead. 対向機も同じ有効プリセットで動いている
                  ことをログで確認すること (片側だけ落ちると測距は成立しない)。
```

**旧方針へ戻す方法**: `Config::downgrade_timing_profile_when_polling`（既定
`true`）を `false` にすると、(b) は旧方針（自動で変えない・警告のみ）に戻る。
アプリ側の起動時警告（`firmware/{anchor,tag,twr}/main/main.cpp`）は降格の有
無に関わらず出るので、どちらの設定でもログから状況は分かる。

（実装: `components/uwb_qm33120/include/uwb_qm33120_types.hpp` の
`Config::downgrade_timing_profile_when_polling`、
`components/uwb_qm33120/src/uwb_qm33120.cpp` の `init()`。**本節の記述は
ビルド確認のみで実機未検証。** 降格が実際に測距成功率を改善するか、締切超過
が実機でも「RXFTO のみ」の失敗として観測されるかは、実機での確認が済んで
いない。）

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

`BothIrq` はタグ側にも IRQ が必要な構成である。StampFly 搭載タグの接続経路が M5StampS3A
背面の 12P FPC 経由になり、G16 の IRQ が実際に取れるようになったことで
（`boards/stampfly.h`、`docs/IRQ_POLICY.md`）、`BothIrq` が実際に成立する構成になっている。

**既定は `BothIrq`**（全ボードで IRQ が取れるため。`docs/IRQ_POLICY.md`）。
**本ドキュメントの数値はいずれも実機未検証**である。IRQ の極性は
データシートで既定値（アクティブ HIGH）を確認済みだが、実機での通電確認は
まだ済んでいない（詳細は `docs/IRQ_POLICY.md`）ので、測距が成立しないときは
`PollingBoth` へ落として切り分けること。

`docs/GETTING_STARTED.md` に「**アンカーとタグは必ず同じプリセットで焼くこと**」を明記する。

### 5.1 CI ビルド（`.github/workflows/build.yml`）
CI の firmware マトリクスは**既定（`BothIrq` + IRQ 有効）で焼いた base variant** と、
IRQ が動かなかったとき用の**フォールバック**の2系統を配布している:

| variant | プリセット |
|---|---|
| `anchor-stamps3-ds` / `tag-stampfly-ds` ほか base | `BothIrq`（90 Hz、既定） |
| `anchor-stamps3-ds-polling` / `tag-stampfly-ds-polling` | `PollingBoth`（31 Hz、フォールバック） |

**どちらもタグとアンカーでセットを揃えること。** 片側だけ別のプリセットを焼くと、
相手側は違う締切で待つことになり測距が成立しない（§0）。
`AnchorIrq`（59 Hz）は CI では配布していないが、menuconfig で選べる。

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

## 7. 実装状況

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
| IRQ 線の起動時自己診断（§4(a) の条件3） | `Qm33120::verifyIrqLine()` | `components/uwb_qm33120/src/uwb_qm33120.cpp` |
| プリセットのポーリング降格（§4(b)、2026-08-28） | `Config::downgrade_timing_profile_when_polling`（既定 `true`） | `components/uwb_qm33120/include/uwb_qm33120_types.hpp`、`uwb_qm33120.cpp` の `init()` |
| 遅延送信の締切までの残り時間の実測（§1.3.2、2026-08-28） | `detail::delayedTxMarginUs()`、`ResponderResult`/`DSRangeResult`/`DSResponderResult::txMarginUs` | `components/uwb_qm33120/src/uwb_qm33120_internal.hpp`、`uwb_qm33120_twr.cpp`、`uwb_qm33120_types.hpp` |

**既知の制約（実機未検証）**: 本ドキュメントの数値導出と同じく、実機での検証はまだ済んでいない。
`PollingBoth` は現在の既定値と数値上完全に一致するため実装前後で挙動は変わらないが、
`AnchorIrq`/`BothIrq` は実機で Phase 1〜2 の検証が済むまで既定にしない方針は変わらない。

**ホスト検算の限界**: `RangeConfig`/`DSRangeConfig`（`uwb_qm33120_types.hpp`）と
`payloadMatchesEither()`/`readTimingTag()`（`uwb_qm33120_internal.hpp`）は、既存の `payloadMatches()`
と同じ理由で ESP-IDF ヘッダ（`uwb_port.h` 経由の `driver/spi_master.h`、`esp_timer.h`、`deca_device_api.h`）
に依存しており、ホストから直接 include できない。`tests/host/pipeline` のシナリオ14/16/17は、
実体のロジックを模した局所ミラー実装（値・アルゴリズムを1行ずつ突き合わせてコメントで出典を明記）で検算している。
シナリオ13/15（`uwb_qm33120_timing.hpp` 自体の検算）はこの制約を受けず、実体の関数を直接呼んでいる。
