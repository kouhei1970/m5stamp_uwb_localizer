# Phase 2R: TWR 層の再実装計画 (2026-08-20)

## 経緯
Phase 2 では M5Stack ラッパを「動作実績のある参照実装」とみなして忠実移植した。
その前提が崩れ（`docs/SOURCE_POLICY.md`）、批判的レビュー（`docs/CRITICAL_REVIEW.md`）で
重大な欠陥が複数見つかった。**一次資料（Qorvo 公式 API rev9p3 の TWR サンプル、
DW3000 UM、QM33120W データシート、APS011/APS014）が手元に揃った**ので、
それに基づいて再実装する。

## 基本判断: 捨てない。層ごとに扱いを変える

| 層 | 由来 | 判断 | 理由 |
|---|---|---|---|
| `qm33120w_sdk` | Qorvo（**一次**） | **そのまま** | 無改変。信頼してよい |
| `uwb_port` | 自作 | **そのまま** | SDK の要求通り。SPI 単一トランザクション化は原本より改善 |
| `uwb_qm33120` デバイス層 (`uwb_qm33120.cpp` 788行) | M5Stack（二次） | **検証して修正** | 構造は妥当。定数とデッドコードを直す |
| **`uwb_qm33120` TWR 層 (`uwb_qm33120_twr.cpp` 643行)** | M5Stack（二次） | **Qorvo 公式サンプルを基準に書き直す** | 制御フローが壊れている。**公式リファレンスが手に入った以上、それを基準にすべき** |
| `uwb_loc` | 自作・検証済み | そのまま | |
| `uwb_ranging` | 自作 | 一部調整 | `perAnchorIntervalMs` のガード等 |

### 重要: 「数式」は流用する
批判的レビューで**単純な計算式の誤りは1件も見つからなかった**。
以下は一次資料と厳密に一致しており、書き直しても同じものになる:
- DS-TWR 非対称式 `(ra·rb − da·db)/(ra+rb+da+db)`
- `DX_TIME` のビット操作 `(x & 0xFFFFFFFE) << 8` + `TX_ANT_DLY`（UM §3.3「低位9bitをゼロに」）
- 40bit→32bit 切り詰め（全区間 ≈3ms ≪ 2^32 dtu = 67.2ms）
- タイムスタンプの読み出し順序
- `speedOfLight = 299702547.0`（公式 `SPEED_OF_LIGHT` と同値）

**壊れているのは「部品をどう繋いだか」。数式ではなく制御フロー。**

---

## 作業項目（優先順）

### 【R1】単位の取り扱いを明文化する — 最初にやる
> **この節は経緯の記録。単位そのもののリファレンスは `docs/UNITS.md` にまとめた。**

**【2026-08-20 訂正】当初の計画（「`Uus`→`Us` にリネームし 63898 を使う」）は誤り。**
SDK のドキュメントで確認した結果、**M5Stack の単位系は正しく、内部的に一貫していた。**

#### 確定した事実（`deca_device_api.h` より）
| API | 単位 | SDK の記述 |
|---|---|---|
| `dwt_setrxaftertxdelay()` | **UUS** | 「The delay is in **UWB microseconds**, 20-bit value」(2681行) |
| `dwt_setrxtimeout()` | **UUS** | 「in **1.0256 us** (512/499.2MHz) units」(2360行) |

**6つの遅延設定のうち4つは、これらの API に直接渡っている**（変換なし）:
`responseRxAfterTxDelayUus`, `rxTimeoutUus`,
`resultRxAfterFinalTxDelayUus`, `finalRxAfterResponseTxDelayUus`
→ **`Uus` という命名は正しい。**

残る2つ（`responseTxDelayUus`, `finalTxDelayUus`）だけが
`× kUusToDwtTime` で DTU に変換され `dwt_setdelayedtrxtime()` 用に使われる。
- 1 UUS = 512/499.2 µs = 1.02564 µs、1 DTU = 1/(499.2e6×128) s = 15.65 ps
- UUS → DTU の係数 = 1.02564e-6 / 15.65e-12 = **65536** ✓ **M5Stack は正しい**

#### 本当の罠: Qorvo 自身の定数が誤称
| | 定数値 | 実際の単位 |
|---|---|---|
| **Qorvo 公式 `shared_defines.h:37`** | `UUS_TO_DWT_TIME 63898` | **実マイクロ秒**（1e-6/15.65e-12 = 63897.6） |
| M5Stack / 本プロジェクト | `kUusToDwtTime 65536` | **真の UUS**（1.02564 µs） |

**Qorvo の `POLL_RX_TO_RESP_TX_DLY_UUS 650` は「650 UUS」ではなく「650 実µs」。**
そのまま本プロジェクトの `responseTxDelayUus` に入れると 2.5% 長くなる。

#### 対処（リネームはしない）
1. **`kUusToDwtTime = 65536` と `Uus` 命名はそのまま維持する**（正しいので）
2. **`uwb_qm33120_types.hpp` に単位の定義を明記したコメントを置く**:
   - 1 UUS = 1.02564 µs であること
   - どのフィールドが SDK の UUS API に直接渡り、どれが DTU 変換されるか
   - **Qorvo 公式の `*_UUS` 定数は実µsであり、そのまま流用してはならないこと**
3. **実µs ↔ UUS の変換ヘルパを用意する**（Qorvo の値を採用するときに使う）:
   ```cpp
   // Qorvo 公式サンプルの *_UUS 定数は実マイクロ秒なので、本APIのUUSへ変換する
   constexpr uint32_t usToUus(uint32_t us) { return (us * 4992u + 2560u) / 5120u; }  // us / 1.02564
   ```
   例: Qorvo の 650 実µs → **634 UUS**
4. 各フィールドのコメントに**どの SDK API に渡るか**を書く

- 実機不要 / 影響: **R5（遅延値の追い込み）で Qorvo の値を採用する際の事故防止**

### 【R2】不一致フレームで測距シーケンスを破棄しない — 最大の効果
**問題**: 6箇所すべてで、期待外のフレームを1枚拾うと `dwt_forcetrxoff()` して `return`。
**アドレス不一致は「エラー」ではなく「他人宛のフレームを拾っただけ」。**

**対処**: `dwt_rxenable(DWT_START_RX_IMMEDIATE)` で受信を再開し、
**所定のタイムアウトまでループを継続**する。公式サンプルも「期待外のフレームは捨てて受信継続」。

- 実機不要（実装は） / 影響: **5台構成の成功率が劇的に改善**（推定 15-25% → 大幅改善）
- 該当: `requestRange`(107-117), `respondRange`(198-208),
  `requestDSRange`(344-354, 414-424), `respondDSRange`(485-495, 556-567)

### 【R3】"DWD" 3回リピートの廃止 / 再設計
**問題**: `resultRepeatCount=3` / `gap=3ms` と `rxTimeoutUus=3000`(=3.077ms) が
**一度も整合していない**。DWD#1 は約36%取りこぼし、**DWD#3 は物理的に受信不能**な上に
**次のアンカーの応答窓に着弾して測距を壊す**。1リンク14msのうち9msを占める。

**対処（段階的）**:
1. **`resultRepeatCount = 1`**（1行）— 破壊的挙動が消え、1リンク 14ms → 7ms
2. `resultRxAfterFinalTxDelayUs` を 500 → 200 以下（DWD#1 の取りこぼし解消）
3. そもそも結果を返す設計を見直す。**公式 DS-TWR は Responder 側で距離を出して終わり**で、
   Initiator へ返送しない。Tag 側で計算したいなら **final に必要なタイムスタンプを載せて
   Tag が計算する**方が素直（→ R3b）

**R3b（検討）**: DS-TWR の距離計算を Tag 側へ移す
- 現状は Anchor が計算して "DWD" で返送 → Tag は生タイムスタンプを検算できない
- Tag 側計算にすれば、外れ値検出に生の ra/rb/da/db が使える
- ただしフレーム設計の変更を伴う。**R3-1 を先に入れて効果を測ってから判断**

- 実機不要（実装は） / 影響: **1リンク時間が半減**、次リンクへの妨害が消える

### 【R4】SS-TWR にクロックオフセット補正を入れる
**一次資料（`ex_06a_ss_twr_initiator.c:193-210`）**:
```c
float clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
rtd_init = resp_rx_ts - poll_tx_ts;
rtd_resp = resp_tx_ts - poll_rx_ts;
tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
```
DW3000 UM §12.2 も同式を規定。APS011 §2.3: `Error = ½·t_reply·(e_A − e_B)`。

- 実機不要（実装は） / 影響: **SS-TWR が実用になる**（現状 0.46 m/ppm）
- DS-TWR には不要（原理的に免疫。UM「20ppm 水晶でも low picosecond range」）

### 【R5】遅延値を一次資料基準に下げる — **実機でのチューニングが必須**
| | 折り返し | ホスト予算 |
|---|---|---|
| **Qorvo SS-TWR 公式** | **650 µs** | 471.6 µs |
| Qorvo DS-TWR 公式 | 900 / 500 µs | — |
| Qorvo `CPU_PROCESSING_TIME` | **400 µs** | — |
| M5Stack | 3,077 µs | 2,898 µs（**7倍超**） |

**公式のチューニング法（`config_options.h:57`）**:
> `dwt_starttx()` が `DWT_ERROR` を返し始めるまで下げ、そこにマージンを積む

**公式 NOTE 4 の実測指針**: SPI 18MHz なら 400 µs、4.5MHz なら 550/600 µs。
「さらに下げるには**割り込みを使う**か、コード最適化か、より速い SPI」

**遅延送信の真の締切（UM §3.3）**: `DX_TIME` ではなく **`DX_TIME − (プリアンブル+SFD の air time)`**。
PLEN128/SFD8/64MHz PRF なら **138.4 µs 手前**。
HPDWARN は SYS_TIME 半周期(≈8.6秒)のラップ検出であり、**固定の下限は仕様上存在しない**。

- **実機必須**（下げすぎると測距が失敗する）
- 対処: Kconfig で振れるようにし、**成功率と `dwt_starttx()` 失敗回数を同時にログ**する

### 【R6】IRQ 駆動化 — **方針は `docs/IRQ_POLICY.md` に確定版がある**

> **要約**: アンカーは IRQ を積極的に使う。タグは使わない（ポーリングで成立させる）。
> ただし StampFly が別配線で IRQ を取れる可能性を残し、その対応を準備しておく。
> **IRQ はポーリングを置き換えるのではなく、選択肢として足す。**


**【重要な前提 2026-08-21】**

| 役割 | ボード | IRQ | 方針 |
|---|---|---|---|
| **タグ** | StampFly (StampS3) | **取れない** | **IRQ 非依存を仕様とする。ポーリング経路は必ず残す** |
| タグ | 単体 M5StampS3A | 取れる（G7） | 使ってもよいが**依存してはならない** |
| **アンカー** | AtomS3 / AtomS3R | **取れる（G2）** | **R6 の対象はここ** |

理由: StampFly で外部に出ているのは GROVE の4本（G13/G15/G1/G2）のみで、
SPI 4線で使い切るため IRQ 線が残らない。
（当初「空きGPIO G5/G10/G41/G42 がある」としていたのは誤りで、
　この4本はモータPWM。`docs/SURVEY_stampfly_grove.md` の訂正を参照）

**現状の実装は タグ・アンカーとも完全ポーリング**（`dwt_setinterrupt` /
`dwt_setcallbacks` / `dwt_isr` はソース中で一度も呼ばれていない）。
つまりタグ側は既に仕様を満たしている。R6 で変えるのは**アンカー側だけ**。

#### R5 への影響
DS-TWR では**両側に遅延送信の締切がある**:
- アンカー（レスポンダ）: `responseTxDelayUus` … **IRQ 化で詰められる**
- **タグ（イニシエータ）: `finalTxDelayUus` … ポーリングのままなので詰められない**

したがって R5 の到達点は非対称になる（`docs/STAMPFLY_INTEGRATION.md` の試算）:

| 構成 | アンカー側 | タグ側 | 1周 | レート |
|---|---:|---:|---:|---:|
| 現状 | 3077µs | 1846µs | 31.9ms | 31.3 Hz |
| **R5（アンカーのみ IRQ）** | **900µs** | **1400µs** | **16.8ms** | **59.4 Hz** |
| （参考）両側 IRQ | 900µs | 700µs | 11.1ms | 90.2 Hz |

**StampFly での現実解は「アンカーのみ IRQ」の 59 Hz。** 位置制御の実効帯域
（約0.064 Hz）に対して十分すぎるので、これで打ち止めてよい。

---

### 【R6 補足】元の記述（両側 IRQ を前提としていた）

**問題**: IRQ ピンを配線し `pinMode(INPUT)` まで設定しておきながら**一度も使わない**。
`dwt_setinterrupt` / `dwt_setcallbacks` / `dwt_isr` が全リポジトリで未使用。
全ステートマシンが `vTaskDelay(1)` ポーリング（1ms 粒度）。

**これが全ての性能問題の根源**。ホスト応答遅延 0〜1ms → 遅延値を膨らませる → …

**対処**: `dwt_setinterrupt()` + GPIO ISR + タスク通知（`xTaskNotifyFromISR`）で
イベント駆動にする。SDK の `dwt_isr()` を使う。

- **実機での検証必須**
- 影響: **R5 で 650 µs 台が狙える → 1リンク時間が更に短縮**
- 注意: M5StampS3A では IRQ 配線済み（G7）。**StampFly の GROVE 2系統案では IRQ 線が無い**
  → StampFly では IRQ 無しのポーリング経路も残す必要がある（**両対応にする**）

### 【R7】デッドコードと嘘コメントの除去
- `recommendedPHYProfile()` は**完全な恒等関数**（40行）。
  ヘッダコメント「チャネル別の推奨プロファイルを適用する」は**嘘**
- **実害**: 既定値は Qorvo の **ch9 用 `txconfig_options_ch9` と一致**。
  `Channel5` を選ぶと **ch5 に ch9 の TX 電力が適用される**（ch5 の公式既定は `0xfdfdfdfd`）
- **対処**: 本当にチャネル別プロファイルを持つ（ch5/ch9 の公式値を入れる）か、削除して正直に書く
  → M5Stamp UWB Module は **ch9 固定**なので、ch5 を捨てて ch9 専用にするのが最も正直

- 実機不要 / 影響: 保守性。ch5 を使うなら実害あり

### 【R8】`sfdTimeout` の既定を 0（自動計算）に
既定 `129` が非ゼロなので `makeSfdTimeout()` の式が絶対に走らない。
既定 PHY では 129 が正しいが、`preambleLength` を Len256 にすると
**257 であるべきが 129 のまま**。SDK はエラーを返さない。
- API Guide §5.2.1 の式: `PLEN + 1 + SFD_len − PAC`
- **注意**: 0 を SDK に渡すと既定 4161 に落ちるので、**ラッパ側で計算して渡す**
- 実機不要 / 影響: プリアンブル長を変えたときの事故防止

### 【R9】タイムアウトと保護の整備
- `hostTimeoutMs = 100` はチップ側(3.08〜4.6ms)の**20〜30倍**。
  5アンカー × 100ms = **500ms のストール**。→ **10ms 程度**に
- ~~**`dwt_setpreambledetecttimeout(5)`（5 PAC）を設定** — 公式 DS-TWR 例の既定。
  応答が来ないときの RX 電力浪費を防ぐ~~ →
  **これは移植ミスだった（`docs/REVIEW_2026-08-21.md` §0 #1 / Critical で確認）。**
  PRETOC は「RX が開放された時刻」を起点に走るタイマーであり（SDK
  `deca_device_api.h:2368-2372`「X ≥ 1 sets a timeout equal to (X+1)*PAC」、
  UM `docs/refs/DW3000_Family_User_Manual_wayback.txt:8152-8158`「The
  preamble detection timeout starts running as soon as the receiver is
  enabled to hunt for preamble」）、本実装は RX を相手プリアンブル到達の
  0.3〜1.4ms 前に開ける設計（`PollingBoth`: タグは Poll 送信後 1500 UUS
  で RX 開始、アンカーの Response は 3000 UUS 後）なので、5 PAC
  （6 PAC ≈ 49µs @PAC8）の PRETOC は必ず RXPTO を起こし DS-TWR が
  常に失敗する。公式 `ex_05a/ex_05b` が 5 PAC で動くのは RX 開放直後
  （数µs後）にプリアンブルが来るよう遅延を調律しているためで、本実装の
  遅延設計にはそのまま移植できない。`uwb_qm33120_twr.cpp:531, 841` は
  `dwt_setpreambledetecttimeout(0)`（無効）に修正済み。PRETOC を有効化
  したい場合は「公式に倣って5」ではなく、RX 開放時刻から相手プリアンブル
  先頭到達までの時間を PAC 単位で計算して設定すること。
- **HPDWARN (SYS_STATUS bit 27) を監視**し、立ったら `CMD_TXRXOFF` で中断（UM §8.2）。
  `EVC_HPW` カウンタ（要 `EVC_EN`）で遅延超過の頻度を定量化
- **`dwt_starttx()` の戻り値で失敗したら交換を放棄して RX 待ちに戻る**（公式 NOTE 10/13）。
  放置すると来ない TXFRS を待って固まる
- 実機不要（実装は） / 影響: 堅牢性、ストール防止

### 【R10】ch9 の PLL 再校正 — **M5Stamp UWB Module は ch9 固定なので必須**
**UM §10.4**: **ch9 では温度が 20°C 変化したら PLL 再校正が必要**（6ステップ）。**ch5 では不要。**
アンカーは連続受信でダイ温度が周囲+20〜30°C まで上がるため、**確実に該当する**。
- 対処: `dwt_readtempvbat()` を定期的に読み、±20°C 変化で再校正
- 実機での検証必須 / 影響: 長時間運用時の受信性能

### 【R11】アンテナ遅延の校正 — Phase 3
- **TWR では TX/RX の合計だけ校正すればよい**（APS014 Table 1）
- 現在値 16385×2 = 512.85 ns は APS014 の典型初期値（~513ns）。**出自は妥当**
- **Δ=1ns で 30cm の定常バイアス**。1 dtu = 15.65ps = 4.7mm
- APS014 実績: 校正前 3σ ≈ 30cm → 校正後 ≈ 4.5cm
- **推奨校正距離: 5.0 m**（APS011 Table 3、ch5/64MHz PRF）
- 手順: 3台以上で EDM を作り、アンテナ遅延0で TWR → ノルム最小化（APS014 §2.2）
- **温度依存 2.15 mm/°C (7.17 ps/°C)** — 校正時の温度を記録すること
- **【要確認】OTP 0x0B = "Antenna Delay – RFLoop"（Prod Test が書き込み）。
  工場校正値が入っている可能性。`dwt_otpread(0x0B, &v, 1)` で実機確認**
- 実機必須

### 【R12】診断情報の取得と `uwb_loc` への接続 — 費用対効果が最も高い追加
SDK に揃っているのに一度も呼ばれていない:
- **`deca_rsl.c/.h`**（ビルド対象に入っている）: `rsl_calculate_signal_power()`,
  `rsl_calculate_first_path_power()`。Q8.8, 0.1dBm 精度
- `dwt_nlos_alldiag()` / `dwt_nlos_ipdiag()`: 第一波/ピークのインデックスと電力

**`RSL − FP_RSL > 6dB` は Qorvo 標準の見通し外指標。**
**5アンカー構成では「どのアンカーが壁越しか」を判定して `uwb_meas.sigma` / `quality` に
反映できる。** `uwb_loc` の Lv2 は Huber/χ²ゲートを持っているので、
**この情報を入れれば NLOS 環境の測位精度が直接改善する。**

さらに APS011 §3: **測距バイアスは温度でなく受信信号強度(RSL)依存**。Table 2 に補正表。
- 実機での検証必須 / 影響: **NLOS 環境での測位精度**

---

## 実機の有無による切り分け

### 実機なしで進められる（今すぐ着手可能）
R1（単位）, R2（受信継続）, R3-1（リピート廃止）, R4（クロックオフセット）,
R7（デッドコード）, R8（sfdTimeout）, R9（タイムアウト整備）

**これだけで「5台で安定して取れる確率 15-25%」が大きく改善するはず。**

### 実機が要る
R5（遅延値の追い込み）, R6（IRQ 駆動の検証）, R10（PLL 再校正）,
R11（アンテナ遅延校正）, R12（診断情報の検証）

---

## 検収基準の変更

| | 旧 | 新 |
|---|---|---|
| 基準 | 原本 M5Stack と一致しているか | **一次資料に照らして正しいか** |
| 根拠 | `M5Stamp_UWB.cpp:NNN` | **Qorvo 公式サンプル / UM / API Guide / APS の該当箇所** |
| 検証 | ビルドが通る | ビルド + **実機で測れる形になっている** |

**コード中のコメントには、一次資料の出典（文書名・章番号）を書くこと。**
「M5Stack 版ではこうだった」ではなく「UM §3.3 より」と書く。

---

## 注意事項（作業者向け）

1. **公式サンプルは非UTF-8（latin-1）**。`grep` が binary 扱いして黙って0件を返す。
   **`grep -a` か `iconv` を通すこと。この罠で一度誤った結論を出しかけた**
2. 「SDK に API があるのに使っていない」は**それ自体では欠陥の根拠にならない**。
   公式サンプルが使っているかを確認すること（フレームフィルタで実際に誤判定した）
3. `UUS_TO_DWT_TIME` の世代差（65536 vs 63898）に注意。定数を移植するとき必ず確認
4. QM33120W の **SPI 上限は 32 MHz**（DW3000 の 36MHz、API Guide の「up to 38MHz」ではない）
