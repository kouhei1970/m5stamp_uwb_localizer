# M5Stack ラッパ実装の批判的レビュー結果 (2026-08-20)

対象: `third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp` (1,558行) と、その移植版
判断根拠: `components/qm33120w_sdk/` の一次資料（`deca_device_api.h`, `dw3720_device.c`,
`dw3720_deca_regs.h`, `deca_compat.c`）

**前提**: SDK 部分は `diff` でバイト一致（無改変）。移植版は原本に対し論理変更なし。
したがって以下はすべて**原本 M5Stack ラッパへの指摘**であり、移植でそのまま継承されている。

---

## 総括: 欠陥の根本原因は1つ

> **IRQ ピンを配線し `pinMode(INPUT)` まで設定しておきながら一度も使わず、
> 全ステートマシンを `delay(1)` ポーリングで回している。**

`gpio_isr_handler_add` も `dwt_setinterrupt` / `dwt_setcallbacks` / `dwt_isr` も
**全リポジトリで一度も呼ばれていない**。

その結果の連鎖:
```
IRQ 不使用
  → ホスト応答遅延が 0〜1ms の一様分布
    → responseTxDelayUus を Decawave 標準 450 → 3000 (6.7倍) に膨張
      → SS-TWR のクロックオフセット誤差が 6.7倍
      → 1リンクが長くなる
        → "DWD" 3回リピートという場当たり的ハック
          → 整合の取れていない受信ウィンドウ
```

（**この「450」は誤り。§`responseTxDelayUus = 3000` の妥当性 の訂正ボックス参照**）

**重要な所見: 「写した部品」は正確だった。壊れているのは「どう繋いだか」。**
DS-TWR の式も、`DX_TIME` のビット操作も、タイムスタンプの読み出し順序も、
Decawave のリファレンスを正確に写している。単純な計算式の誤りは**1件も無い**。

---

## 確定した欠陥

### 【重大1】不一致フレームで測距シーケンスを即座に破棄する
**メインが実コードで確認済み** (`uwb_qm33120_twr.cpp:107-117` ほか計6箇所)

```cpp
if (!parseShortAddressFrame(...) || !payloadMatches(...) ||
    (parsed.sequence != pollSeq) || (parsed.src != range.responderAddress) || ...) {
    detail::stopRadioAndClearRxStatus();   // dwt_forcetrxoff()
    result.error = Error::RangeFrameMismatch;
    return result;                          // ← 受信を続けずに諦める
}
```

**アドレス不一致は「エラー」ではなく「他人宛のフレームを1枚拾っただけ」。**
本来はタイムアウトまで受信を継続すべき。

該当箇所: `requestRange` (107-117), `respondRange` (198-208),
`requestDSRange` (344-354, 414-424), `respondDSRange` (485-495, 556-567)

**影響**: 欠陥【重大2】（HWフィルタ無し）と掛け算になり、
アンカー5台構成では他ペアのフレーム1枚で測距が丸ごと失敗する。

### ~~【重大2】ハードウェアのフレームフィルタが未設定~~ → **【訂正: 欠陥ではない】**
**2026-08-20 訂正。** 一次資料（Qorvo 公式 API rev9p3）で確認したところ、
**公式 TWR サンプル4本すべてがフレームフィルタを使っていない**:

| サンプル | `configureframefilter`/`setpanid`/`setaddress16` |
|---|---|
| `ex_05a_ds_twr_init` | 0件 |
| `ex_05b_ds_twr_resp` | 0件 |
| `ex_06a_ss_twr_initiator` | 0件 |
| `ex_06b_ss_twr_responder` | 0件 |

（使用例は `ex_07b_ack_data_rx` と `ex_15_le_pend` のみ）

**M5Stack は Qorvo の流儀通りであり、一次資料からの逸脱ではない。**
「欠陥」という当初の判定は誤り。

**ただし問題の実体は残る**: Qorvo のサンプルは1対1。本プロジェクトは**5台 round-robin**。
他ペアのフレームが上がること自体は Qorvo でも同じだが、
**それを「エラー」として測距シーケンスごと破棄するのは M5Stack 独自**（→【重大1】）。

→ **正しい対処はフレームフィルタの有効化ではなく、不一致時に受信を継続すること。**
   一次資料の推奨も「TWR ではフレームフィルタを有効化しない
   （0x8841 の Decawave 独自フレームでは扱いが増えるだけ）」。

#### この誤判定から得た教訓
1. 「SDK に API があるのに使っていない」は**それ自体では欠陥の根拠にならない**。
   一次資料のリファレンス実装が使っているかを確認すること
2. 公式サンプルは**非UTF-8（latin-1）**で、`grep` が binary 扱いして黙って0件を返す。
   `grep -a` か `iconv` を通すこと。**この罠で一度誤った結論を出しかけた**

### 【重大3】"DWD" 3回リピートが次のリンクを能動的に破壊する
Poll RMARKER を t=0 とした DS-TWR タイムライン（ch9/preamble128/PAC8/6.8M/SFD DW8）:

| イベント | 時刻 |
|---|---|
| DWF (final) 終端 | +4,973 µs |
| **Tag の DWD 受信ウィンドウ** | **+5,486 〜 +8,563 µs** |
| DWD#1 送信開始 | ≈ +5,123 + L µs （L∈(0,1000] は `vTaskDelay(1)` 由来） |
| DWD#2 | ≈ +7,400〜8,400 + L + L2 µs |
| **DWD#3** | **≈ +10,500〜12,000 µs** ← ウィンドウ閉端の 2〜3.5ms 後 |

1. **DWD#1 は L < 363µs のとき Tag の RX がまだ開いておらず取りこぼす → 一様分布仮定で約36%**
2. DWD#2 はウィンドウ閉端にまたがる（`vTaskDelay(3ms)` は仕様上 (2,3] ms なので運次第）
3. **DWD#3 は物理的に絶対に受信できない**

`resultRepeatCount=3` / `gap=3ms` と `rxTimeoutUus=3000` (=3.077ms) は
**一度も整合が取られていない**。3ms 間隔で再送するなら RX ウィンドウは最低 8,000 UUS 必要。

**さらに悪いこと**: Tag のスケジューラは `perAnchorIntervalMs = 0`（既定）なので
DWD#1 を受けたら即座に次のアンカーへ poll を撃つ。
一方 Anchor i はまだ DWD#2/#3 を送っている。
**DWD#3 (+10.5〜12ms) が、次のアンカー i+1 の応答窓 (8.6〜11.7ms) のど真ん中に着弾する。**
Tag のチップはそれを受信 → ペイロード不一致 → **欠陥【重大1】で測距中断**。

**1リンク14msのうち9msを占めるこのリピートは、信頼性を上げるどころか次のリンクを壊している。**

### 【重大4】アンテナ遅延が未校正（値の出自は判明・妥当）
**2026-08-20 訂正**: `16385` の由来が一次資料で特定できた。
- **APS014 §2.2.1 Figure 4**: 「Set initial delays (**~513ns**)」
- APS014 §2.2.2 の実測値: 514.4747 / 514.5911 / 515.0413 ns（EVB1000 3台、7.914m）
- **検算: 16385 × 2 = 32,770 dtu × 15.65004 ps = 512.85 ns ≈ 513 ns。一致。**
- 公式サンプル NOTE 2 も「The sum of the values is the TX to RX antenna delay,
  experimentally determined by a calibration process. Here we use a hard coded typical value」

→ **「EVB からコピーした間違った値」ではなく、APS014 が定める典型的な初期値。**
  当初の characterization は不正確だった。
- **TWR では TX/RX の合計しか効かない**（APS014 Table 1）。
  16385/16385 の均等分割自体は無害。**校正すべきは合計値**
  （分割が必要なのは TDoA 無線同期のみ。その場合の比率は TX 44% / RX 56%、Table 4）
- TX/RX で同じ値を使うこと自体は妥当（対称 TWR では和だけが効く）
- **問題は絶対値**。Δ=1ns で **30cm の定常バイアス**。1 DTU = 15.65ps = 4.7mm
- M5Stamp UWB Module のアンテナ・RF 経路は EVB と別物なので、
  **数nsのずれ = 数十cm〜1m超の定常オフセットがほぼ確実に乗る**
- **定常バイアスなので校正すれば消える**（Qorvo APS014 手順）

### 【重大5】SS-TWR にクロックオフセット補正が無い（既報）
`dwt_readclockoffset()` (`deca_device_api.h:2801`, "divided by 2^26 to get ppm offset")
が存在するのに未使用。誤差 = c·e·T_reply/2。

`responseTxDelayUus = 3000` (=3,077µs) で **0.46 m/ppm**。
**450 UUS に落とすだけで 0.069 m/ppm（6.7分の1）になる。**
（**この「450」は誤り。§`responseTxDelayUus = 3000` の妥当性 の訂正ボックス参照**）

*緩和材料*: `dwt_initialise()` が OTP から crystal trim を読んで適用する
（`deca_device_api.h:1836` Note 2、`dw3720_device.c:1015-1021` で `XTRIM_ADDRESS` → `XTAL_ID` 確認）。
工場トリム済みなので最悪 20ppm にはならない。ただし温度ドリフトは残る。

---

## 中程度の指摘

### 【中1】`recommendedPHYProfile()` は完全な恒等関数（40行のデッドコード）
**メインが実コードで確認済み**。`phy.channel = channel` した後、
switch で `phy.channel = Channel9/Channel5` を再代入するだけ。入力と出力が完全一致。

**ヘッダコメントは嘘**: 「If only channel is changed, init() applies the built-in
recommended profile for that channel.」（`uwb_qm33120_types.hpp`）

**実害**: 既定値（`pgDelay=0x34`, `txPower=0xfefefefe`, preamble code 9）は
Qorvo の **ch9 用 `txconfig_options_ch9` と一致**する。
`Channel5` を選ぶと **ch5 に ch9 の TX 電力設定がそのまま適用される**
（Qorvo の ch5 既定は `0xfdfdfdfd`）。

### 【中2】`sfdTimeout` の既定値が自動計算式を殺している
**メインが実コードで確認済み**。既定 `129` が非ゼロなので `makeSfdTimeout()` の式は絶対に走らない。

129 = 128 + 1 + 8 − 8 は**既定 PHY に対しては正しい**。
しかし `preambleLength` を Len256 に変えると、正しくは 257 であるべきところが
**129 のまま**。プリアンブル取得中に打ち切られ受信率が激減する。
SDK はエラーを返さないので**発見が極めて困難な罠**。
→ 既定を `0`（自動計算）にすべき。

### 【中3】PG 帯域幅キャリブレーションが走らない
`pgDelay = 0x34` は有効値だが `PGcount = 0`。
`deca_device_api.h:1971`: 「PGcount != 0 なら PG キャリブレーション（帯域幅校正）が走り、
そうでなければ PGdelay 値をそのまま使う」
→ 個体差・温度による送信帯域のずれが未補正。

### 【中4】温度・電圧補償の API が全部あるのに一切使っていない
- `dwt_readtempvbat()` (3073) / `dwt_convertrawtemperature()` (3088)
- **`dwt_xtal_temperature_compensation()` (3960)** — 温度に応じて水晶トリムを設定する専用 API。
  ヘッダに水晶の温度-周波数曲線の ASCII 図まである
- `dwt_pgf_cal()` (3301) — 「受信の前に必要」と明記

呼ばれているのは `dwt_configure()` 内の `ull_pgf_cal(dw, 1)` **1回だけ**。以降再校正されない。

**本プロジェクトで特に効く**: アンカーは連続受信（RX 電流 数十mA）でダイ温度が
周囲＋20〜30℃ まで上がる。Tag（間欠）とアンカー（連続）で**温度が非対称に乖離**し、
**SS-TWR のクロックオフセット誤差がウォームアップに伴ってドリフトする**。

### 【中5】診断情報（NLOS判定材料）が全部揃っているのに未取得
- **`deca_rsl.c/.h`**: `rsl_calculate_signal_power()` (RSSI),
  `rsl_calculate_first_path_power()` (第一波電力)。Q8.8, 0.1dBm 精度。**ビルド対象にも入っている**
- `dwt_nlos_alldiag()` (3524): F1/F2/F3, cir_power, accumCount, DGC_DECISION
- `dwt_nlos_ipdiag()` (3534): 第一波インデックス, ピークパスインデックス

**アプリ・コンポーネントコードから一度も呼ばれていない**（grep 確認済み）。

`RSL − FP_RSL > 6dB` は Qorvo 標準の見通し外指標。
**5アンカー構成では「どのアンカーが壁越しか」を判定して `uwb_loc` の重み付けに渡せる**のに、
その情報がそのまま捨てられている。
→ **本プロジェクトにとって最も費用対効果の高い追加**。

### 【中6】`hostTimeoutMs = 100` がチップ側タイムアウトの20〜30倍
チップ側 3.08〜4.6ms に対しホスト側 100ms。
**5アンカー × 100ms = 500ms のストール**を招く。10ms 程度が妥当。

---

## 軽微

- **DS-TWR の "DWR" ペイロード8バイトが完全な死荷重**。`respondDSRange` が
  `pollRxTs`/`respTxTsPlan` を詰めるが `requestDSRange` は一切読まない
  （自分の値を使う）。SS-TWR 版からのコピペ残骸。8バイト ≈ 9.4µs の無駄
- `static_cast<dwt_ip_sts_segment_e>(0)` — `deca_compat.c:421-423` で `(void) segment`
  なので機能的に無害。ただし正しくは `DWT_COMPAT_NONE` (0xFF)
- `dwt_initialise(DWT_DW_INIT)` — `DWT_DW_INIT` は `dwt_setdwstate()` 用の enum で型の誤用。
  値が 0 で `DWT_READ_OTP_ALL`(0x00) と一致するため**結果的に最も完全な初期化**になっている
- 設定の二重管理が既に破綻: `firmware/anchor` は `rxTimeoutUus=3000` を明示指定するが、
  `firmware/tag` のスケジューラ経路はライブラリ既定の 4500 のまま
- リプレイ検出なし（同じ seq のフレームを何度でも受理）

---

## フレーム air time の実測算

設定: **ch9 / preamble 128 / PAC8 / 6.8Mbps / SFD DW8**
`txCode = rxCode = 9` → `deca_device_api.h:963` "9 -> PRF of 64 MHz" より **64MHz PRF 確定**
64MHz PRF のプリアンブルシンボル長 = 508 chip / 499.2 Mcps = **1017.63 ns**

| 区間 | 時間 |
|---|---|
| SHR (preamble 128 + SFD 8 = 136 sym) | **138.4 µs** |
| PHR (標準, 850kb/s) | 19.5〜21.5 µs |
| データ 6.8Mb/s (12B poll / 20B resp / 24B final / 16B DWD) | 16.4 / 25.8 / 30.5 / 21.1 µs |
| **フレーム全長** | **174 / 184 / 188 / 179 µs** |
| RMARKER→終端 | 36 / 45 / 50 / 41 µs |

### `responseTxDelayUus = 3000` の妥当性

> **【2026-08-21 訂正】以下の「450 UUS」は出典が確認できなかった。**
> `docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/` を `grep -a` で再確認したところ
> （`.c` は非UTF-8 なので `grep -a` 必須。`docs/HANDOFF.md` 運用ルール3）、Qorvo 公式サンプルの
> SS-TWR responder（`ex_06b_ss_twr_responder/ss_twr_responder.c:77`）は
> `POLL_RX_TO_RESP_TX_DLY_UUS 650`、DS-TWR responder
> （`ex_05b_ds_twr_resp/ds_twr_responder.c:85`）は `POLL_RX_TO_RESP_TX_DLY_UUS 900`
> （同 :87 の `RESP_TX_TO_FINAL_RX_DLY_UUS` は `500`）であり、450 ではない。
> `450` という数値自体は STS 版のサンプル（`ss_twr_responder_sts.c:75`）に
> `(450 + CPU_PROCESSING_TIME)` として存在するが、非 STS 版（本節が参照している構成）には無い。
>
> **さらにこれらは名前に反して実マイクロ秒であって UUS ではない**（`docs/UNITS.md` §3）。
> UUS に直すと `usToUus(650) = 634`、`usToUus(900) = 878`。
>
> したがって**本節の結論（3000 UUS は過剰）は変わらないが、比較対象の数値は 450 ではなく
> 634〜878 UUS 相当である**。実際に採用したプリセット値は `docs/TIMING_PRESETS.md` §2 を参照。

必要最小値 = フレーム尾部 36µs + ホスト折返し + SHR 138.4µs + マージン
- **IRQ駆動 + 16MHz SPI なら**: 折返し 60〜150µs → **≈ 300〜400 UUS**
  （Decawave 公式 `ss_twr_responder` の **450 UUS** とほぼ一致）
  （**この「450」は誤り。上の訂正ボックス参照**）
- **現行の 1ms ポーリングでも**: 最悪 1.15ms → **≈ 1,320 UUS**

**3,000 UUS は IRQ 実装に必要な値の 6.7倍、現行のポーリング実装に対してすら 2.3倍の過剰。**

---

## 疑ったが問題なかったもの（誤検知防止のため明記）

1. DS-TWR の 40→32bit 切り詰めと非対称式 — Decawave 標準と完全一致、ラップも安全
   （全区間 ≈3ms ≪ 2^32 DTU = 67.2ms）
2. `(x & 0xFFFFFFFE) << 8 + TX_ANT_DLY` — `DX_TIME` レジスタ定義
   （bit0無視、mask 0xfffffffe、`dw3720_deca_regs.h:265-267`）と厳密に一致。誤差ゼロ
3. RX タイムスタンプの取得元 — `RX_TIME_0` = Ipatov 補正済み TOA。STS Off の構成で正しい
4. `sfdTimeout = 129` — 既定 PHY に対しては正しい値
5. 遅延TX失敗後の復旧 — `dwt_forcetrxoff()` は DW3000 系の正しい作法。
   **`dwt_rxreset` は本 SDK に存在せず、それで正しい**
6. Responder が `respTxTs32` を DWD 送信より前に読む順序依存 — 正しい
7. アンテナ遅延の手動加算は二重計上ではない
8. **移植版の SPI 単一トランザクション化** — 原本の1バイトループを解消した明確な改善

---

## 「このまま実機に載せて動く確率」

既定構成（DS-TWR、ch9、5アンカー round-robin、`perAnchorIntervalMs=0`）:

| シナリオ | 確率 | 主な減点要因 |
|---|---|---|
| Device ID 読み出し → `dwt_configure()` 成功 | **90%** | 残るリスクは電源/wakeup タイミング |
| 1対1 で DS-TWR の距離値が出る | **60〜70%** | DWD#1 の取りこぼし ≈36% |
| その距離値が ±1m 以内 | **30%** | アンテナ遅延未校正。Δ=1ns で 30cm |
| **5アンカーで全アンカーから安定して取れる** | **15〜25%** | 【重大1】×【重大2】×【重大3】の三重苦 |
| SS-TWR の距離精度 | 実用外 | 0.46 m/ppm + 温度ドリフト |
| 公称精度 0.14m 相当 | **5%未満** | 無校正では原理的に到達不能 |
