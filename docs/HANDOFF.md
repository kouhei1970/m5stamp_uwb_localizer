# 次セッションへの申し送り (2026-08-21 時点 / §0-B は 2026-08-29 追記)

**このファイルを最初に読むこと。** 全体像・決定事項・落とし穴・次の一手をまとめてある。

**§1 以降は 2026-08-21 時点の記述で、2026-08-27〜29 の実機セッションより前のものである。**
測距が成立しない件の現在地は下の **§0-B** が正本。
**§0-A（2026-08-28 に立てた IRQ 前提の締切割れ説）は 2026-08-29 の実機測定で棄却済み。**
経緯の記録として残してあるだけなので、読む順序は §0-B → §0-A とすること。

---

## 0-B. 最優先: SS-TWR の成功率が上がらない件の現在地（2026-08-29 実機セッション）

2 台の M5StampS3A + M5Stamp UWB Module を距離 540 mm で向かい合わせ、
ch9 / preamble 128 / PAC8 / 6.8 Mbps / SS-TWR で測定した。

### 実機で確定した事実（すべて測定値。推測ではない）

1. **配線は健全。** `firmware/probe` の L1-L10 が 2 台とも全 PASS
   （RSTn / IRQ 自己診断 / 16 MHz SPI 1000 回読みで不一致 0 / TX スモークテスト）。
   partId は 0x1172CD26 と 0x1172C9B1、lotId は同一（同一ロット）。
2. **ソフトは純正ライブラリの忠実な移植である。**
   [`m5stack/M5Stamp-UWB`](https://github.com/m5stack/M5Stamp-UWB) と、PHY 既定値
   （ch9 / Len128 / Pac8 / code 9,9 / SFD DW8 / 6.8 Mbps / STS Off / phrMode 0 /
   pgDelay 0x34 / txPower 0xfefefefe / アンテナ遅延 16385 / LNA・PA 有効）、
   SPI クロック（slow 2 MHz / fast 16 MHz）、SS-TWR の遅延定数
   （responseRxAfterTxDelayUus=500 / responseTxDelayUus=3000 / rxTimeoutUus=4500 /
   hostTimeoutMs=100）、init 手順（`dwt_initialise(DWT_DW_INIT)` → `dwt_configure`
   → `dwt_configuretxrf` → アンテナ遅延 → `dwt_setlnapamode`）がすべて一致する。
   **`PollingBoth` 構成は純正構成そのものであり、それで 22〜39% しか出ていない。**
   唯一の差は `sfdTimeout` が純正は固定 129・本リポジトリは 0（自動計算）だが、
   既定条件では計算結果も 128+1+8-8 = 129 で同値。
3. **遅延送信の締切は一度も割っていない。** `tx_margin_us` は BothIrq で +615 µs、
   PollingBoth で +2785 µs。`TxStartFailed` は 0 件。折返しの実測は 901-615 ≒ 286 µs で、
   [`TIMING_PRESETS.md`](TIMING_PRESETS.md) §1.3 の「IRQ 駆動 0.3 ms」の見積りは正しかった。
4. **ソフトのフレーム照合は無罪。** 全失敗行で `rx_seen=0 / rx_rej=0`
   ＝ CRC を通ったフレームが 1 枚もソフトへ渡ってきていない。
   `frameMatchesExpectation()` が有効なフレームを捨てている事実はない。
5. **受信電力は正常。** −62〜−79 dBm。飽和域（−60 dBm 以上）ではない。
   **`CONFIG_UWB_TWR_DIAG_NO_LNA_PA=y` で外部 LNA/PA を止めても受信電力も成功率も
   変わらない（22.3% → 21.8%）。このモジュールに外部フロントエンドは無く、
   `dwt_setlnapamode()` は何もしていない。**
6. **温度は無関係。** アンカーのダイ温度が 39.7 → 52.3 ℃ まで上がっても、
   10 回ごとの瞬時成功率に下降傾向は出ない（平均 3.2/10 で定常）。課題 R10 は主因ではない。
7. **TWR の段取りも無罪。** `firmware/devtest`（SENDER = 即時送信 / RECEIVER = 素の受信、
   遅延送信も折返しも受信窓もなし）で 70 送信中 **36 受信 = 51%**、RX error 34 件。
   送信数と「受信 + エラー」の数が一致するので取りこぼしではなく復号失敗である。
8. **失敗の中身。** RXFSL（Reed-Solomon 復号失敗）と RXPHE（PHR 誤り）が主。
   いずれも RXPRD / RXSFDD は立っており、前置信号と SFD は掴めている。
   失敗時の `elapsed_ms=3` は応答の到達予定時刻ちょうど、RXFTO の `elapsed_ms=5` は
   受信窓の満了時刻ちょうど。**電波は予定どおり届いていて、復調だけが失敗している。**

### 核心の数字（同一配置での PHY 分離実験）

| 設定 | 成功率 | 既定からの改善 |
|---|---|---|
| 既定（preamble 128 / 6.8 Mbps） | 22.3% | — |
| プリアンブル 1024 のみ（`UWB_TWR_DIAG_PHY_PREAMBLE=1024`） | 29.1% | +6.8 pt |
| **850 kbps のみ**（`UWB_TWR_DIAG_PHY_850K=y`） | **50.0%** | **+27.7 pt** |
| 両方（`UWB_TWR_DIAG_ROBUST_PHY=y`） | 56.8% | +34.5 pt |

**効くのはデータ速度であってプリアンブル長ではない。** 失敗が PHR と本体の復号に
集中していることと整合する。

そして **DW3000 系の 850 kbps の受信感度は約 −102 dBm**（Qorvo フォーラムには
−103 dBm で 43 m という実測報告がある）。本リポジトリの構成は **−70 dBm で 5 割落として
いる。実効感度が仕様より約 30 dB 悪い。** これが解くべき問題の正体である。

なお「処理利得を 18 dB 足しても 100% にならないから雑音説は成立しない」という推論は
**誤りなので繰り返さないこと**。不足が 30 dB あるなら 18 dB 足しても道半ばであり、
22% → 57% はむしろ整合する。

### 残っている候補（次セッションはここから）

- **(a) 広帯域の妨害波 / 雑音の底が約 30 dB 高い。**
  `firmware/probe` の L9（3 秒の環境スキャン、両機とも送信停止）では
  `observed=[none]`＝プリアンブル検出も SFD タイムアウトも PHR 誤りも 0 件だった。
  UWB 的な干渉波は出ていないが、CW や短いバースト状の妨害はこの検査に映らない。
- **(b) 受信機の利得較正（DGC）が効いていない。** ← 調査が途中。下記参照。
- **(c) 信号の歪み（多重波）。** `rsl_dbm − fp_dbm` が場面によって 1 dB から 12 dB まで
  振れる。第一波に対して後続の多重波が大きいときがある。

**(b) の具体的な追い方（ここで中断した）:**
`components/qm33120w_sdk/dw3720/dw3720_device.c:967-974` で、OTP の `DGC_TUNE_ADDRESS` を
読んだ値が `DWT_DGC_CFG0` と一致したときだけ `dgc_otp_set = DWT_DGC_LOAD_FROM_OTP` になり、
一致しなければ `DWT_DGC_LOAD_FROM_SW`（`ull_configmrxlut()` のハードコード値）へ落ちる
（同ファイル 1790 行 / 9200 行で分岐）。**この判定がどちらに転んでいるかは実機未確認。**
`firmware/probe` に `dwt_otpreadword32(DGC_TUNE_ADDRESS)` の生値と `DGC_CFG0/1/2` の
実レジスタ値を出す検査（L11 相当）を足して読み出すこと。

### 配置への異常な敏感さ（実験手順として必ず守ること）

既定 PHY は配置に極端に敏感で、**置き場所を変えながらの A/B 比較は成立しない**。

| 配置 | 既定 PHY | robust PHY |
|---|---|---|
| 木のテーブル上（床から 72 cm、天板から 4 cm） | 38.9% → 別測定で 22.3% | 56.8% |
| 床から 28 cm（段ボール箱） | 5.0% | 64.1% |

robust PHY はほぼ位置に鈍感。**設定を比較するときは必ず配置を固定し、同一セッション内で
連続して測ること。** 日をまたぐ・置き直すと基準が動く。

なお 2 波モデル（直接波と床面反射波、行路差 Δ ≒ 2h²/d、合成振幅 2·|sin(πΔ/λ)|、
λ = 3.75 cm）では、d = 0.54 m・h = 0.04 m のとき Δ = 5.9 mm で自由空間比 −0.5 dB となり、
テーブル上の配置は本来ほぼ損失がない。**「低すぎるから打ち消している」という説明は
実測で棄却された**（持ち上げたら悪化した）。

### 計器の読み方の注意（重要）

CIA 診断レジスタは SYS_STATUS をクリアしても消えない。したがって
**RXFTO（何も受信していない）の行に出る `rsl_dbm` / `fp_dbm` / `accum` は、
前に受信した良品フレームの残りである。** `accum` が毎回同じ値になるのはこのため。
必ず `rx_status` と併せて読み、RXFTO の行の電力値は捨てること。

---

## 0-A. 【棄却済み】SS/DS-TWR がほぼ成立しない件（2026-08-28 に立てた説）

> **この節の結論は 2026-08-29 の実機測定で棄却された。** 上の §0-B が正本。
> `tx_margin_us` は常に正（+615 µs / +2785 µs）で締切は一度も割っておらず、
> IRQ 線も 2 台とも生きている（probe L5 が PASS）。
> 以下は経緯の記録として残す。

### 結論（コードと一次資料だけで確定。実機では未検証）

**IRQ（割り込み）を前提にしたタイミングプリセットが、実際にはポーリングで動いている
ファームに適用されていた。** IRQ プリセットの折返し締切（約 900 µs）は、ポーリングの
折返し時間（約 1.2 ms）では構造的に守れない。

連鎖はこうなっている。

1. **Kconfig の既定は `UWB_TIMING_PROFILE_BOTH_IRQ`**
   （`firmware/twr/main/Kconfig.projbuild`、`firmware/anchor`・`firmware/tag` も同じ。
   ビルド済み `firmware/twr/sdkconfig` も `CONFIG_UWB_TIMING_PROFILE_BOTH_IRQ=y`）。
   **ソース中の多数のコメントが「既定は PollingBoth」と書いていたが、これは事実と違った**
   （2026-08-28 に訂正済み）。過去のコミットメッセージが実測条件を「PollingBoth」と
   記録しているものも、この誤ったコメントに引きずられた可能性がある。
   **どのプリセットで焼いたかは、いまや起動ログに出る（下記「観測できるようになったもの」）。**

2. `BothIrq` / `AnchorIrq` の `responseTxDelayUus` は **878 UUS ≒ 900 µs**。
   この値は [`TIMING_PRESETS.md`](TIMING_PRESETS.md) §1.3 が「IRQ 駆動の折返し ≒ 0.3 ms」を
   前提に置いた数字である。同じ §1.3 が**ポーリングの折返しを「気づくまで最大 1000 µs ＋
   SPI と計算 100〜200 µs ＝ 約 1.2 ms」と見積もっている。** 締切のほうが短い。

3. **`uwb_port_irq_enable()` の成功は「ISR を登録できた」ことしか意味しない。**
   DW_IRQ が `pin_irq` に本当に繋がっているか、極性が合っているかは一切見ていなかった。
   線が死んでいても `irqActive()` は true を返し、起動ログは `irq=active` と出る。
   一方 `uwb_port_irq_wait(1)` は毎回 1 ms のタイムアウト待ちに落ちる ＝ 実質ポーリング。
   **本リポジトリは IRQ 線の導通を一度も確認していない**（§1「取れていないのは
   RSTn(G6) / IRQ(G7) / WAKEUP(G8)」）。

4. 締切を割った遅延送信は、SDK が黙って遅れて送るのではなく**送信ごと取り消す**。
   `components/qm33120w_sdk/dw3720/dw3720_device.c` の `ull_starttx()`:
   HPDWARN（Half Period Delay Warning）を見て `CMD_TXRXOFF` を発行し `DWT_ERROR` を返す。

5. 結果、アンカーは Poll を**聞いているのに** Response を一切出さない。タグ側からは
   **「RXFTO のみ・RXPRD も RXSFDD も立たない」**＝電波が来ていないのと区別できない
   失敗に見える。2026-08-28 に実測した `rx_status=0x000200F4` はまさにこの形である。

### 観測データとの整合（この説明だけが全部と矛盾しない）

- **距離・向き・役割の入替え・ダイ温度・PHY の頑健化（preamble 1024 / 850 kbps）・
  外部 LNA/PA のどれを変えても成功率が動かなかった**のは、損失が電波側ではなく
  ホストのスケジューリング側にあるなら当然である。
- **数十秒周期の「うなり」**は、2 台の 1 ms tick（FreeRTOS のタイマ刻み）の相対ドリフトで
  説明できる。水晶の相対誤差 25 ppm なら位相が 1 ms ぶんずれるのに
  $1\times10^{-3} / 25\times10^{-6} = 40\ \mathrm{s}$。記録にある
  「起動後 4.000 秒は 20/20 成功、その後 40 秒ほぼ無反応」と桁も形も合う。
- 成功率 18.1% は、1 ms の tick 位相のうち締切に間に合う割合が 18% だったと読める
  （＝折返しの実測が約 900 − 180 = 720 µs 程度）。
- **逆に `PollingBoth`（締切 3077 µs）ではこの周期性は原理的に出ない。** 余裕が
  1 ms tick より大きいので、位相がどこにあっても間に合う。
  **うなりが観測されたこと自体が、焼かれていたのが IRQ プリセットだった状況証拠になる。**

### 棄却した説

- **「アンカーの受信窓と Poll 周期のうなり」（`12c5dbc` で立てた説）は成立しない。**
  `respondRange()` は `dwt_setrxtimeout(0)`（フレーム待ちタイムアウト無効）で受信機を開き、
  窓が満了するまで開けたままにする。呼び直しの合間に閉じている時間は SPI 数回ぶん（µs）
  しかないので、**受信デューティは窓長によらず 99% 以上**あり、1〜18% を説明できない。
  該当箇所のコメントは訂正済み。
- **SPI 16 MHz が化けている説**も、データシート上は無理がない
  （`assets/QM33120W Data Sheet.pdf` Rev.D p.25/p.26: 最大 32 MHz、
  `t2`（SCLK→出力）Max 9.5 ns、`t3`（セットアップ）Min 2.5 ns。16 MHz の半周期 31.25 ns に対し
  十分な余裕がある）。**優先度は下げてよい**（ただし実測で否定したわけではない）。

### 入れた変更（すべて実機未検証。5 ファームのビルドとホストテストは通っている）

| 何を | どこ |
|---|---|
| **IRQ 線の起動時自己診断** `verifyIrqLine()`。①事象が無いのにエッジが来ないか（フロート/極性逆）②短い RX タイムアウトで必ず RXFTO を起こしエッジが来るか、の2段。落ちたら IRQ を諦め `irqActive()` を正直に false にする | `components/uwb_qm33120/src/uwb_qm33120.cpp` |
| **実効プリセットの一本化**。IRQ が使えないと決まったら `PollingBoth` へ降格し、フレームに載る種別・不一致警告・アプリが使う値が全部同じものを指すようにした | 同上 ＋ `firmware/{twr,anchor,tag}/main/main.cpp` |
| **締切までの残り時間の実測** `txMarginUs`（SS の Response、DS の Response と Final の3箇所） | `uwb_qm33120_internal.hpp` の `delayedTxMarginUs()`、`uwb_qm33120_twr.cpp` |
| 事実と違ったコメント（既定 PollingBoth、うなりの機構）の訂正 | `firmware/*/main/main.cpp` |

**記録済みの設計判断からの意図的な逸脱が1つある。**
[`TIMING_PRESETS.md`](TIMING_PRESETS.md) §4(b) は「遅延プリセットは自動で変えてはいけない
（片側だけ変わると破綻するから）」と定めていたが、据え置きは*必ず*失敗するのに対し
降格は両機が同じ挙動なら一致したままなので、既定を降格側にした。理由の全文は
`uwb::Config::downgrade_timing_profile_when_polling` のコメントと §4(b) にある。
`false` にすれば元の挙動に戻る。

### 観測できるようになったもの（起動ログ）

```
I ... irq self-test: OK (pin=7, status=0x........)        ← または FAILED / inconclusive
I ... irq=active (pin=7)                                  ← または irq=polling (...)
I ... timing profile=PollingBoth (version=1, requested=BothIrq) wait=polling response_tx_delay=3000uus(3077us)
```
アンカーの測距ログ:
```
I ... SS_RESP_STAT ok=.. fail=.. last=OK ... tx_margin_us=+1830 temp=..
W ... SS_RESP_STAT ok=.. fail=.. last=FAIL error=TxStartFailed ... tx_margin_us=-240
```

### 実機が来たら、この順で（反証の形で書いてある）

1. **【最優先・新ファーム不要】両機を `CONFIG_UWB_TIMING_PROFILE_POLLING_BOTH=y` で焼き、
   50 秒採る。**
   - 成功率が大きく上がれば（例: 50% 以上）→ 本説は支持される。
   - **変わらなければ本説は棄却。** その場合は下の 2・3 で別の切り分けに移ること。
2. **新ファームで起動ログの `irq self-test` を読む。**
   - `FAILED`（事象は起きたがエッジが来ない）→ IRQ 線が死んでいる。§0-A の前提どおり。
   - `OK` → 線は生きている。**その場合、本説の前提3が崩れる**ので、代わりに
     「IRQ が生きていても折返しが 900 µs に間に合っていない」を疑い、3 の `tx_margin_us` を見る。
   - `spurious edge` → 未配線でフロート、または極性が逆。
3. **アンカーの `tx_margin_us` を読む。これが今回の主計器。**
   - 負の値が出ていれば「Poll は聞こえていたが送信が取り消された」の**直接の証拠**。
     `error=TxStartFailed` の件数がタグの失敗数と同オーダーで並ぶはず。
   - 正で大きければ（例: `PollingBoth` で +1800 µs 前後）締切は原因ではない。**別を探すこと。**
   - `responseTxDelayUus` の実 µs 値からこの残りを引けば**折返しの実測値**が出る。
     [`TIMING_PRESETS.md`](TIMING_PRESETS.md) §1.3 の見積り（ポーリング約 1.2 ms /
     IRQ 約 0.3 ms）はこれで初めて検算できる。**実測が見積りより大きければ、
     プリセットの数値そのものを実測に合わせて引き直すこと。**
4. アンカーの失敗内訳（`error=` の分布）を見る。`TxStartFailed` ではなく
   `RxTimeout` ばかり（＝ログが出ない）なら、アンカーは Poll を**聞けていない**ので
   本説は棄却。受信側の切り分けへ戻る。

### まだ直していない・確認していないこと

- **`respondRange()` は回復可能な受信エラー（RXPHE / RXFCE / RXFSL / RXSTO / ARFE /
  CIAERR）1件で受信窓を放棄して return する。** Qorvo 公式の responder サンプル
  （`ex_06b_ss_twr_responder`）はエラーを消して `dwt_rxenable()` し、待ち続ける。
  今回は「アンカーの失敗計数の意味を変えると前後比較が読めなくなる」ため**あえて触っていない**。
  §0-A の主因が片付いたあとに直すこと。
- **`hostTimeoutMs = 10 ms`** は、チップ側 RX タイムアウト（`PollingBoth` で 4615 µs）1回ぶん
  しか吸収できない。不一致フレームを1枚拾って `dwt_rxenable()` で再アームされると
  10 ms を超え、`RxTimeout` が `rxStatus = 0` のまま返る（＝せっかくの診断が消える）。
  M5Stack 公式サンプルは 100 ms。**今回は A/B 比較を濁さないため据え置いた。**
- **アンカーの電源。** データシート p.11 Table 5 の連続受信ピーク電流は
  **ch9 で typ 82 mA / max 107 mA**（ch5 は typ 68 mA）。アンカーは連続受信なのでここに張り付く。
  外部給電で動かしていた回のデータは、給電が持っていたかを確認するまで採用しないこと。
- 上記の変更はすべて**実機未検証**。ビルド（5 ファーム）とホストテスト
  （`make -C tests all`）が通っただけである。

---

## 0. 次セッションの任務（2026-08-21 時点。§0-A より前の文脈）

> 実機投入前レビュー（[`archive/REVIEW_2026-08-21.md`](archive/REVIEW_2026-08-21.md)）の
> **Critical 1・High 6 は全件対応済み**。ただし**Medium 以下とテストの穴は未対応で残っている**
> （下記「レビューから引き継いだ未対応項目」）。

### レビューから引き継いだ項目は棚卸し・修正済み

実機投入前レビューの着手順 7〜10 を全件コードと突き合わせ、**実際に未対応だった
7 件を修正した**（5 件は後続コミットで既に解消済みだった）。

修正した内容:

| 層 | 直したもの |
|---|---|
| TWR | 折返し中の重い `ESP_LOGW` を、遅延送信の予約が済んだ後へ移した（予約が間に合わなくなるのを防ぐ） |
| TWR | アンカーの Poll 待ちだけタイムアウトを 200ms に分離（10ms ごとに受信機を落として Poll を取りこぼしていた） |
| port | `deca_sleep()` を `+1 tick` に。`CONFIG_FREERTOS_HZ < 1000` を `#error` で弾く |
| app | `PositionResult::redundancy` を追加し JSON に出す。`0` なら外れ値棄却が原理的に効かない |
| app | アンカーテーブル差し替え時に `resetStats()` を呼ぶ |
| app | 同一ショートアドレスの重複登録を拒否 |
| app | JSON の閉じを必ず書く（落ちると次行と連結して不正 JSON になっていた） |
| テスト | `test_pipeline` にノイズ付き外れ値ゲート試験（200試行）と、欠測+`excluded` の添字検証を追加 |
| ビルド | 4 つの Makefile に `-MMD -MP`。`sanitize` を全スイートに揃え、`tests/Makefile` に集約 |

**実機が要るのは H2（IRQ プリセットの `finalTxDelay`）だけ。** レビュー自身が
「実機の `dwt_starttx` 失敗回数を見て決める」としており、実機なしで数値を動かすと
検証の基準が変わるので手を付けていない。実験7・8 で失敗率を見てから判断すること。

### ブランチ `feat/stamps3-fpc-migration` での変更

前回 HANDOFF 時点（HEAD `dffcde5`、main）から本ブランチで6コミット
（`git log --oneline main..HEAD` で確認可能。一覧は §1）を積み、ハードウェア構成の
既定を切り替えた:

- **アンカー5台＋据置タグ1台を M5StampS3A + StampS3 BreakOut に統一**（`UWB_ANCHOR_BOARD_STAMPS3` /
  `UWB_TAG_BOARD_STAMPS3` が既定）。**AtomS3(R) は削除せず代替として残す。**
  UWB モジュールは S017-F を 0.5mm 12P FPC + FPC→DIP 変換基板で接続する（`docs/WIRING.md` 経路A）
- **StampFly 搭載タグは M5StampS3A 背面の 12P FPC 経由に切り替え**（`UWB_TAG_BOARD_STAMPFLY`、
  `boards/stampfly.h` 全面改訂）。**旧 GROVE 2系統4本構成は廃案**（RST/IRQ/WAKEUP が取れず、
  GROVE が電池電圧直結・満充電約4.35V で絶対最大定格4.0Vを超えるため LDO が必須だった。
  新経路は背面 FPC の VDD_3V3 に直結できるため LDO 不要）
- RST/IRQ/WAKEUP が全構成で取れるようになったので、**IRQ を既定にした**。
  遅延プリセットの既定も `PollingBoth` → **`BothIrq`（約90Hz）**。
  **IRQ の極性は実機未検証なので、測距が出ないときは `*-polling` 版へ落として切り分ける**
  （`docs/IRQ_POLICY.md`）
- ドキュメントを 22 本 → 17 本に整理。廃案・訂正・経緯は
  [`archive/DESIGN_HISTORY.md`](archive/DESIGN_HISTORY.md) へ退避し、現役文書には
  「いま正しいこと」だけを残した。対応表は同文書 §5

**このブランチは push 済み・`main` へは未マージ。** まず差分をレビューし、
問題なければ `main` へマージすること。**CI は feature ブランチへの push では走らない**
（トリガが `push: branches: [main]` と `pull_request`）ので、確認には PR が要る。

**実機が届いたらこの順で進める**（詳細・判断基準は
**[`docs/EXPERIMENT_PLAN.md`](EXPERIMENT_PLAN.md)**）:

1. `firmware/probe` — Device ID `0xDECA0314` が読めるか（最初の関門）
2. `firmware/twr` — **SS-TWR から**。DS-TWR は Critical バグ（PRETOC 誤設定）を修正済みだが
   実機では未検証なので後回しにする
3. `firmware/anchor` × 5 + `firmware/tag` × 1 — フル測位

**起動直後に必ず確認するログ3行**（`docs/EXPERIMENT_PLAN.md` §10 #12 にも記載。
出なければ即座に切り分けへ）:

1. `... spi: slow=... fast=... active=16000000` — SPI が 2MHz に張り付いていないか
   （レビュー H-2 の修正確認）
2. `... main task stack high-water mark: ... bytes` — メインタスクを 12288B に
   引き上げた効果の確認（レビュー H-1。残量が小さければ専用タスクへの分離を検討）
3. `INIT_FAILED` が **出ないこと**（`dwt_checkidlerc()` 待ちタイムアウト。レビュー M-2 の対策確認）

**実機到着前にできることはまだ残っている。** 旧版の「すぐ着手できる」項目 A〜F、レビュー #6
（外れ値棄却）・#7（ライセンス同梱）は前々回・前回セッションで完了・コミット・push 済み
（`dffcde5`、main、CI green）。その後 `feat/stamps3-fpc-migration` ブランチでハードウェア構成の
既定を切り替えたことで、**実機を待たずに進められる作業が新たに生まれている**:

- **FPC→DIP 変換基板の型番選定と接点面（同面／異面）の確認**（`docs/EXPERIMENT_PLAN.md` §10 #13）
- **M5StampS3A 背面 12P FPC コネクタの入手**（HDGC/0.5K-HX-12PWB。出荷時は未実装で後付けが要る）
- **StampS3 BreakOut の PinMap 確認**（3V3/GND のヘッダ位置。同 §10 #15）

配線図（`docs/WIRING.md`）とピン定義（`boards/stamps3.h` / `boards/stampfly.h`）はすでに
確定しているので、部材さえ揃えばこれらは実機（UWB モジュール本体）の到着を待たずに進められる。
それ以外に強いてやるなら以下の任意項目のみ:

- `uwb_math` の `uwb_sym3_solve_sphere` と Beck のλ探索の共通化（反復の形が同型。任意）
- `v0.2.0` を切るかどうかの判断（§1 Release 参照。全件コミット済みなので
  `git tag v0.2.0 && git push --tags` だけで CI が zip を作る）

---

## 1. 現在地

**Phase 1（SPI 疎通）は 2026-08-27 に実機で通った。** 測距・測位はまだ実機未検証。
製品自体が新しく、コミュニティの実績も無い（ユーザ談）。

### 実機の状況（2026-08-27。本リポジトリで実機の Device ID が読めた最初の記録）

| 項目 | 結果 |
|---|---|
| 構成 | **M5StampS3A + StampS3 BreakOut + FPC→DIP 変換基板 + `S017-F`**。＝ **本ブランチが標準に定めた経路A（FPC）そのものが実機で通った**。パッド直付け（経路B）は使っていない |
| 台数 | **1 台のみ**（残り 5 台は未着手。Phase 2 の TWR には 2 台要る） |
| `firmware/probe` | **L1 / L2 とも PASS**。`raw DEV_ID = 0xDECA0314`。60 秒 64 回すべて同値、`0x00000000` / `0xFFFFFFFF` は 0 件 |
| 機械的ストレス | 線を触る・軽く引っ張っても崩れない |
| ピン定義 | **`boards/stamps3.h` を一切変えずに通った** |
| SPI クロック | **2MHz (`spi_slow_hz`) のみ。16MHz は依然として未検証**（`firmware/probe` は `begin()` を呼ばない） |

**裏付けの範囲（広げて解釈しないこと）**
`docs/GETTING_STARTED.md` [4.2.1](GETTING_STARTED.md#probe-result) に詳細。要約すると
**取れた**のは FPC 接続そのもの・挿し方向・SPI 4 本（SCK=G12 / MOSI=G11 / MISO=G13 / CS=G10）。
**取れていない**のは RSTn(G6) / IRQ(G7) / WAKEUP(G8) と 16MHz SPI。
RSTn は未配線でも POR 直後なら probe が通り、WAKEUP は未配線だと CS パルス経路へ
フォールバックし、IRQ はポーリングのため読まれないため。

**到達までの経緯（原因未確定。断定しないこと）**
最初は L1/L2 とも FAIL で `raw DEV_ID` が `0x00000000` と `0xFFFFFFFF` の間をふらついていた
（= MISO が浮いている直接証拠）。3V3 は実測済みだったため配線側を疑い、**変換基板を
挿し直した**ところ PASS。半田付けはやり直していない。「差し込みが 1 ピンずれていた
可能性」が挙がっているが直接観測していないので未確定。機械的ストレスで再現しないため
接触不良の線は薄い。**1 ピンずれは電源逆接になり得るので、挿し位置の目印を推奨。**

| リポジトリ | 状態 |
|---|---|
| **m5stamp_uwb_localizer**（本体） | GitHub 公開済み `kouhei1970/m5stamp_uwb_localizer`（public）。`main` の HEAD は `dffcde5`。**現在の作業ブランチ `feat/stamps3-fpc-migration`（HEAD `9730d66`、main から6コミット、未マージ・未 push）で作業中**。ワークツリーには本ブランチの文書更新に伴う未コミットの差分がある |
| **uwb_localizer**（上流） | **2026-08-21 に凍結・独立**（`42daea9`。上流 `667551e` を取り込み）。以後 `components/uwb_loc/` は本リポジトリで独立して開発する。**上流は見ない** |
| stampfly_ecosystem | `third_party/stampfly_ecosystem` に読み取り専用クローンあり。**書き込み禁止** |
| 一次資料（PDF/公式API） | `docs/refs/`。**`.gitignore` 済み**（再配布禁止文書を含む） |

### 前回 HANDOFF（`d9a7dbc`）以降のコミット（HEAD `dffcde5`。`42daea9` のみ CI 失敗）

`42daea9` は float ビルドの loc テストが落ちた（gcc の FMA 縮約に関する事故。§3「float の検証」の由来）。
`98385fe` で修正済み。他はすべて CI 緑。

| コミット | 内容 |
|---|---|
| `bce1096` | 実機投入前の最終レビューと Critical 1・High 3 修正（DS-TWR の PRETOC、SPI 16MHz 切替、タグのメインタスクスタック、`decamutexon`） |
| `42daea9` | 上流 `uwb_localizer` を凍結（上流 `667551e`）・最終状態に同期・独立宣言（**CI 失敗**。float の loc テストが FMA 縮約事故で落ちた） |
| `4298c08` | `tests/host/` へホストテストを統合、`docs/archive/` を新設、線形代数コンポーネント `uwb_math` を新設 |
| `98385fe` | `uwb_loc` / `uwb_survey` を `uwb_math` ベースのスカラー展開に書き換え、一般 LU / Jacobi を全廃（`42daea9` の float ビルド失敗を修正） |
| `4de9c68` | 外部の実機報告（GOROman 氏 gist）を反映、`begin()` にソフトリセットを追加 |
| `4ab98e2` | 公式回路図で電源系統・FPC/パッド対応を確定、電源電圧の記述を裏付け直す |
| `dffcde5` | 測量の leave-one-out 外れ値棄却・キラリティ入力、`uwb_math` へ LDLᵀ / `solve_sphere` / `null_vector` 統合、Release zip へのライセンス同梱、`firmware/soltest/.cache/clangd/index/*.idx` の untrack |

### `dffcde5` でコミットした内容（前回 HANDOFF 時点では未コミットだったもの）

1. **Release zip へのライセンス同梱**（レビュー #7 対応）
   `.github/workflows/build.yml` / `README.md` / `THIRD_PARTY_LICENSES.md` / `docs/PREBUILT_BINARIES.md`。
   各 zip に `LICENSE` / `THIRD_PARTY_LICENSES.md` / `LICENSES/LicenseRef-QORVO-2.txt` を同梱し、
   Release 本文と `PREBUILT_BINARIES.md` に `LicenseRef-QORVO-2` 条件3（Qorvo 製 IC 限定）を明記した。
2. **測量の外れ値 leave-one-out 棄却（A-2）とキラリティ入力（A-4）**（レビュー #6 対応）
   `components/uwb_survey/{include,src}` / `docs/SURVEY_SPEC.md` / `tests/host/survey/test_survey.c`。
   `uwb_survey_input.chirality`、`uwb_survey_result.outlier_ambiguous` / `chirality_margin` を追加。
3. **`uwb_math` へ LDLᵀ・`solve_sphere`・`null_vector` 等を統合**
   `components/uwb_math/{include,src}` / `components/uwb_loc/src/{uwb_closed_form.c,uwb_internal.h,uwb_nls.c}` /
   `docs/archive/MATH_AUDIT_2026-08-21.md` / `tests/host/math/test_math.c`。
   `uwb_loc` 側・`uwb_survey` 側とも新 API への差し替えが完了している。

### `feat/stamps3-fpc-migration` ブランチのコミット（`main`=`dffcde5` から6コミット、HEAD `9730d66`。未 push）

| コミット | 内容 |
|---|---|
| `bc92494` | StampS3A 用アンカーバイナリを配布（CI に `anchor-stamps3-ds` / `-fast` を追加）、旧 `BRINGUP.md` の FPC 番号誤記を修正 |
| `5044f46` | アンカーを StampS3A 既定にし、StampFly を背面 12P FPC 接続へ切替（`boards/stampfly.h` 全面改訂、CI variant 14→18）。旧 GROVE 4線構成を廃案に |
| `2f9d8df` | M5StampS3A の公式回路図（`assets/Sch_StampS3_v0.3.3.pdf`）を追加。`boards/stampfly.h` の一次資料 |
| `6578065` | `BRINGUP.md` を `GETTING_STARTED.md` §3〜§4 へ統合 |
| `7f84e2d` | `PLATFORM_TUNING.md` / `UNITS.md` を統合、`MATH_AUDIT_2026-08-21.md` を `docs/archive/` へ移動 |
| `9730d66` | `SOLDER_PADS.md` を `WIRING.md` へ改組し配線の正本を1本化（経路A: FPC→DIP変換基板 / 経路B: 半田パッド直付け / 経路C: StampS3A背面12P FPC の新設を含む） |

詳細は各コミットメッセージおよび `docs/EXPERIMENT_PLAN.md` / `docs/STAMPFLY_INTEGRATION.md` /
`docs/WIRING.md` を参照。

### ホストテスト（`make -C tests all` = test / strict / float を再実行して確認。float ビルドの回帰は 591,184 件）

| スイート | 件数 | 失敗 |
|---|---:|---:|
| loc（`test_uwb.c`） | 77 | 0 |
| loc（`test_regress.c`、新旧比較回帰） | 591,418 | 0 |
| math | 10,781 | 0 |
| pipeline | 188 | 0 |
| survey | 561 | 0 |

（math は前回の 3,423〜3,439 件から `uwb_math` への LDLᵀ / `solve_sphere` / `null_vector` 追加で
10,781 件に増えた。survey は前回の 365 件から A-2 / A-4 のケース追加で 561 件に増えた。）

### ファームウェア（6種）

`firmware/{probe,devtest,twr,soltest,tag,anchor}`。

- 直近 CI（`dffcde5`, 3m6s, green）で 6 種ともカバー済み。
- `soltest` / `tag` / `anchor` は `uwb_loc` 経由で `uwb_math` を使うため、本セッションで
  `idf.py fullclean && idf.py build` により再確認した。**3種とも警告 0・エラー 0**
  （`soltest` は `uwb_survey` も直接リンクしており、A-2/A-4 込みで確認できている）。

### CI

直近 push（`dffcde5`）は green。未 push の差分は無い。

### Release

`v0.1.0`（タグは `9be9000`、2026-08-21 13:17 作成）は**古い**。以後 10 コミット
（レビューの Critical/High 修正、上流凍結、`uwb_math` 化、GOROman 報告反映、回路図確認、
ライセンス同梱を含む）が乗っていない。**特に DS-TWR の Critical バグ修正
（`bce1096`）より前なので、`v0.1.0` の DS-TWR ビルドは実機で確実に失敗する。**
`v0.2.0` を切るなら `git tag v0.2.0 && git push --tags` するだけで CI が
14 通りの zip をライセンス同梱込みで作る。

---

## 2. 何を作ったか

```
components/
  qm33120w_sdk/   Qorvo ドライバ (vendoring, SPDX 保持)              ← 一次資料。信頼してよい
  uwb_port/       dwt_spi_s / deca_sleep / mutex / GPIO              ← 自作
  uwb_qm33120/    デバイス層 + SS/DS-TWR                              ← M5Stack 由来を大幅に修正
  uwb_math/       対称3x3/2x2の閉形式・LDLᵀ・ブロックコレスキー      ← 新設。一般LU/Jacobiは置かない
  uwb_loc/        測位ソルバ Lv0-Lv3（uwb_math ベースに書き換え済み） ← 上流凍結後は本リポジトリで独立開発
  uwb_ranging/    アンカーテーブル + スケジューラ + パイプライン
  uwb_cfgstore/   NVS 永続化 + シリアルコンソール
  uwb_survey/     MDS(逐次三辺測量) + Gauss-Newton + ゲージ固定       ← uwb_math ベース。leave-one-out棄却・キラリティ入力を追加
firmware/         probe / devtest / twr / soltest / tag / anchor
tests/host/       loc（77+回帰59万） / math（10781） / pipeline（188） / survey（561）
tools/            bench_loc（測位ソルバのマイクロベンチ）
docs/archive/     経緯文書（PROGRESS / REIMPL_PLAN / CRITICAL_REVIEW / SURVEY_* など計8本）
```

**ハード依存は `uwb_port` と `uwb_ranging` のスケジューラ部分の2箇所だけ**に隔離。
測位パイプラインと測量計算はホストで検証できる。**線形代数はすべて `uwb_math` に集約**
されており、一般次元の LU 分解・Jacobi 法などは存在しない（設計根拠:
`docs/archive/MATH_AUDIT_2026-08-21.md`）。

---

## 3. 確定した仕様・決定事項

| # | 決定 | 文書 |
|---|---|---|
| 対象 | **ESP32-S3 + M5Stamp UWB Module 専用**。プラットフォーム最適化してよい。ただし StampFly には非依存 | `docs/PLAN.md` |
| **ハード方針** | **StampFly 非依存。ただしタグの配線だけは StampFly 互換を維持する**（GROVE 2系統4本で成立 ＝ IRQ/RST 不要）。想定利用者は本リポジトリを単体で試す人 | `docs/PLAN.md` §1 |
| 役割 | **タグ = M5StampS3A ×1（既定は据置＝StampS3 BreakOut 経由。StampFly 搭載時は機体の M5StampS3A を背面 12P FPC 経由で流用） / アンカー = M5StampS3A + StampS3 BreakOut ×5（既定、`UWB_ANCHOR_BOARD_STAMPS3`）。AtomS3(R) は代替として残る** | `docs/archive/PROGRESS.md`、本ブランチ `feat/stamps3-fpc-migration`（§0・§1） |
| 接続 | **標準は FPC**（経路A: `S017-F` + 0.5mm 12P FPC + FPC→DIP 変換基板 + StampS3 BreakOut。モジュール側の半田付け不要）。**2026-08-27 に実機で疎通確認済み**。半田パッド直付け（経路B）は代替として残すが今後は使わない方針。J1（FPC 用番号）と PINMAP（パッド用番号）は**別の番号体系**で両方正しい | `docs/WIRING.md` §7.3 |
| 電源 | パッド2（VCC_3V3）は QM33120W の VDD1/VDD2 に直結。動作上限 3.6V・絶対最大 4.0V。5V や StampFly GROVE（満充電 ~4.35V）は不可 | `docs/WIRING.md` §5.1 |
| **IRQ** | **アンカーは積極使用。タグは不使用。StampFly の別配線可能性は残す** | **`docs/IRQ_POLICY.md`** |
| 資料 | **一次資料 = Qorvo SDK/UM/APS。M5Stack ラッパは二次資料で信頼しない** | **`docs/PLAN.md` §5** |
| 測量 | 高さのみ実測(4点以上) + キラリティ1ビット入力（`uwb_survey_input.chirality`）。タグも6台目として参加。外れ値リンクは leave-one-out で棄却。計算は実機上（呼び出し元はまだ無い） | `docs/SURVEY_SPEC.md` |
| StampFly統合 | **案B-2 疎結合**: Lv2 で位置を出し `EskfCore::vectorUpdate3()` で POS_X/Y 観測 | `docs/STAMPFLY_INTEGRATION.md` |
| **数値計算方針** | **一般次元の行列計算（LU/Jacobi/一般コレスキー）は置かない。** 対称3x3/2x2の閉形式と nb≤8 のブロックコレスキーに集約し `uwb_math` に一本化する。ESP32-S3 特化（スカラー展開・単精度FPU前提の最適化）は OK | `docs/archive/MATH_AUDIT_2026-08-21.md` |
| **上流の扱い** | `uwb_localizer` は 2026-08-21 に凍結・独立（`42daea9`）。以後 `components/uwb_loc/` は本リポジトリで独立開発する。**上流は見ない**（孵化器は役目を終えた） | — |
| **float の検証** | clang に加え **gcc-16 と `-ffp-contract=off` でも**通すこと（FMA 縮約に助けられて clang だけ通っていた事故があった） | `tests/Makefile` |
| **ブラウザ自動化** | 使わない（§4 参照） | — |

---

## 4. 過去の誤りと運用ルール

**同じ失敗を繰り返さないために記録がある。**
パターンは1つに集約される:
**「二次的な指標を、直接証拠より優先した」「確認せずに断定した」。**

- **誤り 15 件の一覧と詳細** → [`archive/DESIGN_HISTORY.md` §3](archive/DESIGN_HISTORY.md)
- **調査の運用ルール**（直接証拠を優先／矛盾したら前提を疑う／`grep` 0 件はファイルを疑う／
  資料はハッシュ照合／**ブラウザ自動化は使わない**）→ [`PLAN.md` §5](PLAN.md)

## 5. 文書の地図

現役 22 本（+ 経緯を記録した archive 8 本）。索引は **[`docs/README.md`](README.md)**。

```
README.md                    購入者の入口（リポジトリ直下）
docs/README.md                ★ ドキュメント索引（ここから探す）
docs/HANDOFF.md               ← このファイル
docs/GLOSSARY.md              用語集（略語の正式名称と意味）。分からない略語はここ
docs/UWB_PRIMER.md            UWB 入門。なぜ電波で cm が測れるのか（最初に読む）
docs/UWB_ALGORITHMS.md        測位アルゴリズムの導出（上流からの移植・改訂版）
docs/EXPERIMENT_PLAN.md       実機到着後の実験計画とフラグ有効化の順序。実機でしか潰せない前提12件
docs/IRQ_POLICY.md            IRQ 方針（確定版）
docs/GLOSSARY.md                 UUS/DTU/実µs の単位リファレンス（遅延値を触る前に必読）
docs/TIMING_PRESETS.md        遅延プリセットとバージョン不一致検出（設計）
docs/SURVEY_SPEC.md           自動測量の仕様（外れ値 leave-one-out・キラリティ入力）
docs/STAMPFLY_INTEGRATION.md  StampFly 位置制御への統合（1127行）
docs/PERF_ANALYSIS.md         測位計算の性能分析と上流最適化の結果
docs/PERF_ANALYSIS.md       ESP32-S3 の浮動小数点・コンパイル設定
docs/archive/MATH_AUDIT_2026-08-21.md 行列計算の残存箇所の監査とスカラー化の設計根拠。uwb_math の仕様の元
docs/ANCHOR_PLACEMENT.md      アンカー配置ルール
docs/WIRING.md           パッド仕様と配線（公式回路図でのネットリスト確認込み）
docs/GETTING_STARTED.md               Phase 1 受入確認
docs/PLAN.md                  全体設計・フェーズ計画
docs/GETTING_STARTED.md       BOM から測位まで11章
docs/PREBUILT_BINARIES.md     ビルド済みバイナリの入手と書き込み。ライセンス条件込み
docs/archive/REVIEW_2026-08-21.md  実機投入前の最終レビュー。Critical/High は対応済み、Medium 以下は §0 へ引き継ぎ
docs/refs/README.md           一次資料の索引
docs/archive/                 経緯文書（現役8本 + archive自身のREADME.md）
  CRITICAL_REVIEW.md            M5Stack ラッパの批判的レビュー
  REIMPL_PLAN.md                R1-R12。R1/R2/R3-1/R4/R7/R8/R9 は実装済み
  PROGRESS.md                   開発進捗ログ
  SURVEY_*.md（5本）            設計当時の事前調査資料
```

---

## 6. 次にやること

### 実機が要る
| # | 内容 |
|---|---|
| **Phase 1 受入** | `firmware/probe` で Device ID `0xDECA0314`。**ピン配線が未検証なのでここが最初の関門** |
| R5 | 遅延値の追い込み（公式手順: `dwt_starttx()` が `DWT_ERROR` を返すまで下げる） |
| R6 | **アンカー側の** IRQ 駆動化 — **コードは実装済み**（`docs/IRQ_POLICY.md`「実装状況」節）。残るのは実機での極性検証（`gpio_config()`のアクティブHIGH前提が正しいか）だけ |
| R10-R12 | ch9 の PLL 再校正(20°Cごと)、アンテナ遅延校正、診断情報→測位重み |
| S3-S5 | ESP-NOW、測量モード、コーディネータ |
| S7 | I2C ToF（高さ自動計測） |
| **float 既定化** | 実機 `soltest` で double/float を比較し `CONFIG_UWB_LOC_USE_FLOAT` の既定値を判断する |

### 任意（実機不要、優先度低）
| # | 内容 |
|---|---|
| `uwb_math` の共通化 | `uwb_sym3_solve_sphere` と Beck のλ探索の共通化（反復の形が同型） |
| 測量 leave-one-out の計算量 | n=8 の外れ値入りで最大 ~5000 回のコレスキー/solve（S3 の double で数秒の可能性）。設置時1回として許容しているが、実機で時間を測るとなお良い |
| 測量の2本外れ値（同一ノード共有） | greedy leave-one-out の既知の限界（総当たりの leave-two-out は組合せ数が多すぎる）。記録のみで未対応 |
| SPDX ヘッダ | `components/qm33120w_sdk/` 以外に SPDX ヘッダ・著作権表示が無い。レビュー §5（Low-Medium） |

---

## 7. 落とし穴（踏むと時間を失う）

1. **Qorvo 公式サンプルの `.c` は latin-1。`grep -a` を使うこと**
2. **`UUS_TO_DWT_TIME` は DW1000=65536 / DW3000公式=63898。**
   Qorvo の `*_UUS` 定数は実は実マイクロ秒。そのまま流用すると2.5%ずれる
   → **`docs/GLOSSARY.md`**
3. **ESP-IDF ビルドとホスト `make strict` は警告設定が違う。**
   `uwb_survey.c` が `-Werror=maybe-uninitialized` で落ちた実例あり。
   **新規コンポーネントは必ず `idf.py build` も通すこと**
4. **シェルの cwd が持続する。** `cd firmware/anchor` した後に `cat >> PROGRESS.md` して
   迷子ファイルを作った実例あり。**絶対パスを使うこと**
5. **`sdkconfig` は `.gitignore` 済み。** `sed` で書き換える手順を書いてはいけない。
   `-D SDKCONFIG=build/xxx/sdkconfig` 方式を使う（`GETTING_STARTED.md` に検証済みの手順あり）
6. **`components/uwb_loc/` はもう「上流と byte 一致」ではない。**
   上流 `uwb_localizer` を凍結・最終取り込みした後は、`components/uwb_loc/` は本リポジトリで
   独立して開発する方針に転換済み（2026-08-21）。ESP32-S3 向けの最適化（`uwb_math` ベースの
   スカラー展開など）をソースに直接入れてよい
7. サブエージェントに Web 調査を任せるときは**取得手段を明示的に限定する**
8. **複数セッションが同じファイルを同時に編集していることがある。**
   本セッション中、`components/uwb_survey/src/uwb_survey.c` が別セッションの編集途中で
   一瞬ビルドエラーになる瞬間を観測した。ビルド失敗を見たら即座に「壊れた」と判断せず、
   数分おいて `git diff` / 再ビルドで再確認すること

---

## 8. 数字の要約（実機で最初に検証すべきもの）

| 項目 | 値 | 出典 |
|---|---|---|
| 測位レート（現状・5アンカー） | **31.3 Hz**（1周 31.9ms） | `STAMPFLY_INTEGRATION.md` |
| 同（アンカーのみ IRQ） | **59.4 Hz** | 同上 |
| StampFly 位置制御の実効帯域 | **約 0.064 Hz**（`pos.kp=0.4`） | `params.cpp:771` |
| アンカー座標の推定誤差（σ=5cm時） | RMS 5cm / 最悪 25cm | `test_survey` |
| 共通アンテナ遅延の推定誤差 | 平均 1.5cm | 同上 |
| 外れ値棄却の誤棄却率（σ=5cm ノイズのみ） | **≤0.35%**（2000試行実測、n=8） | `test_survey`（A-2） |
| **無校正のアンテナ遅延バイアス** | **Δ1ns = 30cm** | APS014 |
| 電源電圧の絶対最大定格 | **4.0V**（動作上限 3.6V。5V/StampFly GROVE 4.35V は不可） | `QM33120W_DS.txt` / 公式回路図 |
| ESP32-S3 の float | 加減乗と `madd.s` はハード、**除算と sqrt はソフト** | 逆アセンブルで確認 |

**StampFly の ESKF には POS_X/POS_Y を直接観測する update 関数が1つも無く、
水平位置は速度の積分だけ（実質デッドレコニング）。UWB が埋めるのはそこ。**
