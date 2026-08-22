# 用語集（UWB 測距・測位）

**このリポジトリの文書で使う略語と専門用語の一覧。**
分からない略語が出てきたらここを引くこと。

## 目次

この文書は**「用語」と「単位」の2部構成**。

- **用語**（§1〜§9）: UWB・測距・測位・ハードウェアの略語と専門用語
  - §1 まずこの3つ / §2 測距（レンジング） / §3 時間の単位 / §4 無線フレームと PHY
  - §5 測位（ローカライゼーション） / §6 ハードウェアと ESP-IDF / §7 製品名・型番
  - §8 資料の略号 / §9 ⚠ 紛らわしい語
- **単位**（「単位リファレンス」節、旧 `docs/GLOSSARY.md`）: UUS / DTU / 実マイクロ秒の
  換算式・API ごとの単位・早見表

## 本リポジトリの表記ルール

> **略語は、各文書の中で初めて出てきたときに必ず
> 「正式名称（英語）＝日本語の意味」を添える。**
> 例: 「DS-TWR（Double-Sided Two-Way Ranging、両側二方向測距）」
>
> 2回目以降は略語のみでよい。文書をまたぐときは各文書で改めて展開する
> （読者はどの文書から読み始めるか分からないため）。

---

## 1. まずこの3つ

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **UWB** | **U**ltra-**W**ide**b**and（超広帯域無線） | 500 MHz 以上の広い帯域に極短パルスを撒く無線方式。帯域が広い＝時間分解能が高いので、**電波の飛行時間から距離を cm 級で測れる**。本プロジェクトが使う理由がこれ |
| **タグ** (tag) | — | **位置を知りたい側**。動く。本プロジェクトでは 1 台（M5StampS3A / StampFly）。測距シーケンスを開始する側なので **initiator（イニシエータ）** とも呼ぶ |
| **アンカー** (anchor) | — | **座標が既知の基準局**。動かさない。本プロジェクトでは 5 台（**既定は M5StampS3A + StampS3 BreakOut**。`UWB_ANCHOR_BOARD_STAMPS3` が既定。AtomS3(R) は代替として選択可）。タグからの問い合わせに応える側なので **responder（レスポンダ）** とも呼ぶ |

3次元の位置を解くにはアンカーが最低4台要る（`docs/ANCHOR_PLACEMENT.md`）。

---

## 2. 測距（レンジング）

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **TWR** | **T**wo-**W**ay **R**anging（双方向測距） | 電波を往復させ、往復時間から距離を出す方式。**時計を同期しなくてよい**のが利点 |
| **SS-TWR** | **S**ingle-**S**ided TWR（片側二方向測距） | Poll → Response の1往復だけ。速いが、**両機のクロック誤差がそのまま距離誤差になる** |
| **DS-TWR** | **D**ouble-**S**ided TWR（両側二方向測距） | さらに Final を足した2往復。**クロック誤差が一次の項で相殺される**ので精度が高い。**本プロジェクトの本番方式** |
| **ToF** | **T**ime **o**f **F**light（飛行時間） | 電波が片道を飛ぶ時間。距離 = ToF × 光速。**⚠ 同じ略語が距離センサの意味でも使われる。§9 参照** |
| **RTD** | **R**ound-**T**rip **D**elay（往復遅延） | 送信してから応答が返るまでの実測時間。ToF = (RTD − 相手の折返し時間) / 2 |
| **initiator / responder** | — | 測距を開始する側 / 応える側。本プロジェクトではタグ = initiator、アンカー = responder |
| **アンテナ遅延** | antenna delay | 信号がチップ内部とアンテナの間を通るのにかかる時間。**タイムスタンプに丸ごと乗る**ので、引かないと距離が一定量長く出る。**1 ns のずれ = 30 cm の誤差**（Qorvo APS014）。校正手順は `docs/GETTING_STARTED.md` §9 |
| **クロックオフセット** | clock offset / CFO (Carrier Frequency Offset) | 2機の発振器のずれ（ppm）。SS-TWR では距離誤差に直結するため、`dwt_readclockoffset()` で補正する（`RangeConfig::enableClockOffsetCorrection`） |
| **LOS / NLOS** | **L**ine **o**f **S**ight / **N**on-LOS（見通し内 / 見通し外） | 直線上に遮蔽物が無い / ある。NLOS では電波が回り込むので**距離が長めに出る**（外れ値の主因） |

### 測距フレームの呼び名（本プロジェクト実装）

`components/uwb_qm33120/src/uwb_qm33120_twr.cpp` が送受信する4種類のフレーム。
ペイロード先頭3バイトが識別子になっている。

| 略号 | 名前 | 送る側 | 内容 |
|---|---|---|---|
| `"TWP"` / `"DWP"` | **Poll**（ポール） | タグ | 測距の開始要求。SS-TWR は `TWP`、DS-TWR は `DWP` |
| `"TWR"` / `"DWR"` | **Response**（レスポンス） | アンカー | Poll への応答。タイムスタンプを載せる |
| `"DWF"` | **Final**（ファイナル） | タグ | DS-TWR の2往復目。3つのタイムスタンプを載せる |
| `"DWD"` | **Result**（結果） | アンカー | DS-TWR ではアンカー側が距離を計算するので、その結果をタグへ返す |

---

## 3. 時間の単位

**詳細は `docs/GLOSSARY.md`（遅延値を触る前に必読）。**

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **UUS** | **U**WB micro**s**econd（UWB マイクロ秒） | **1 UUS = 512/499.2 µs = 1.02564… µs。実マイクロ秒ではない** |
| **DTU** | **D**evice **T**ime **U**nit（デバイス時間単位） | チップ内のタイムスタンプカウンタの刻み。**1 DTU ≒ 15.65 ps**。1 UUS = 65536 DTU |
| **ppm** | **p**arts **p**er **m**illion（百万分率） | 発振器の誤差の単位。1 ppm = 0.0001% |

---

## 4. 無線フレームと PHY（物理層）

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **PHY** | **PHY**sical layer（物理層） | 変調・チャネル・データレートなど、電波そのものの設定。本実装では `PhyConfig`（`uwb_qm33120_types.hpp`） |
| **SHR** | **S**ynchronization **H**eader（同期ヘッダ） | フレーム先頭のプリアンブル + SFD。既定設定で **138.4 µs** と長く、遅延設計の主要因になる |
| **プリアンブル** | preamble | SHR の前半。受信機が信号を捕まえるための既知パターン。既定は 128 シンボル |
| **SFD** | **S**tart-of-**F**rame **D**elimiter（フレーム開始区切り） | プリアンブルの終わりを示すパターン。**この終端が RMARKER** |
| **RMARKER** | **R**anging **Marker**（測距基準点） | **タイムスタンプが打たれる時刻の基準点**。フレーム先頭ではなく SHR の終わりにある（≒ 先頭から 138.4 µs 後）。遅延計算がややこしいのはこれが原因 |
| **PHR** | **PHY H**eader（物理層ヘッダ） | フレーム長やデータレートを載せるヘッダ |
| **FCS** | **F**rame **C**heck **S**equence | 末尾のチェックサム（CRC）。2バイト |
| **PAC** | **P**reamble **A**cquisition **C**hunk | 受信機がプリアンブルを何シンボルまとめて相関させるかの単位。既定 PAC8 |
| **PRF** | **P**ulse **R**epetition **F**requency（パルス繰返し周波数） | 単位時間あたりのパルス数 |
| **STS** | **S**crambled **T**imestamp **S**equence | なりすまし対策の暗号化タイムスタンプ列。本実装の既定は `StsMode::Off` |
| **CIR** | **C**hannel **I**mpulse **R**esponse（チャネルインパルス応答） | 受信波形そのもの。NLOS 判定や品質評価に使える診断情報 |
| **PAN ID** | **P**ersonal **A**rea **N**etwork ID | IEEE 802.15.4 のネットワーク識別子。本実装の既定は `0xDECA` |
| **フレームフィルタ** | frame filtering | 宛先が自分でないフレームをチップが自動で捨てる機能。**Qorvo 公式 TWR サンプルも使っていない**ため、本実装でも使わない（`docs/HANDOFF.md` 誤り 1） |
| **PLL** | **P**hase-**L**ocked **L**oop（位相同期回路） | 搬送波を作る回路。ch9 では温度変化で再校正が要る（R10） |

---

## 5. 測位（ローカライゼーション）

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **三辺測量** | trilateration | 3点以上からの**距離**だけで位置を求める方法。角度は使わない（それは三角測量 = triangulation） |
| **ToA / TDoA** | **T**ime **o**f **A**rrival / **T**ime **D**ifference **o**f **A**rrival | 到達時刻 / 到達時刻差を使う測位方式。本プロジェクトは TWR で距離を出すので、そこから先は三辺測量 |
| **Lv0 / Lv2 / Lv3** | — | 測位ソルバの段階（`components/uwb_ranging/include/uwb_ranging_types.hpp:40-42`）。**Lv0** = 閉形式の三辺測量（初期値・配線確認用）、**Lv2** = Beck 厳密解 + Huber ロバスト化（**屋内運用の本番**）、**Lv3** = 密結合 EKF（移動体向け）。※ Lv1 は上流 uwb_localizer に存在するが本リポジトリの `SolverLevel` には出していない |
| **Beck 厳密解** | Beck's exact solution | 三辺測量を反復なしで解く代数解法。局所最小に落ちない |
| **Huber ロバスト化** | Huber loss | 外れ値の影響を抑える損失関数。NLOS で伸びた測距値に引きずられにくくする |
| **GN** | **G**auss-**N**ewton（ガウス・ニュートン法） | 非線形最小二乗の反復解法。自動測量（`uwb_survey`）で使う |
| **EKF / ESKF** | **E**xtended **K**alman **F**ilter / **E**rror-**S**tate KF（拡張カルマンフィルタ / 誤差状態 KF） | 動きのモデルと観測を融合して状態を推定する。Lv3 が EKF、StampFly 側の姿勢・位置推定が ESKF |
| **GDOP** | **G**eometric **D**ilution **o**f **P**recision（幾何学的精度低下率） | **アンカーの配置の良し悪しを表す倍率**。測距誤差が位置誤差に何倍に拡大されるか。小さいほど良い。配置が一直線・同一平面だと悪化する |
| **残差 RMS** | residual **R**oot **M**ean **S**quare | 測距値と推定位置から計算した距離の食い違いの二乗平均平方根。大きければ座標入力ミスかアンテナ遅延未校正を疑う |
| **MDS** | **M**ulti**d**imensional **S**caling（多次元尺度構成法） | 距離行列だけから座標を復元する古典的手法。自動測量の初期値作りに使う（`docs/SURVEY_SPEC.md`） |
| **ゲージ固定** | gauge fixing | MDS が返す座標には**回転・並進・鏡像の自由度が残る**ので、実測値でそれを固定する作業 |
| **キラリティ** | chirality（掌性） | 鏡像かどうかの1ビット。**高さの実測では原理的に決まらない**（線形汎関数だから）ので、人が cw/ccw で答える（`docs/SURVEY_SPEC.md`、`docs/HANDOFF.md` 誤り 7） |
| **ambiguous** | — | 本実装の出力フラグ。**`true` なら高さ(z)を信用してはいけない**（鏡像解の可能性）。`ok` と両方見ること |

---

## 6. ハードウェアと ESP-IDF

| 用語 | 正式名称 | 意味 |
|---|---|---|
| **SPI** | **S**erial **P**eripheral **I**nterface | 4線（SCK/MOSI/MISO/CS）の同期シリアル通信。**UWB モジュールとの通信はこれしかない** |
| **SCK / MOSI / MISO / CS** | Serial Clock / Master Out Slave In / Master In Slave Out / Chip Select | SPI の4本。順にクロック / ホスト→デバイス / デバイス→ホスト / デバイス選択 |
| **I2C** | **I**nter-**I**ntegrated **C**ircuit | 2線（SDA/SCL）のシリアル通信。本プロジェクトでは高さ自動計測の ToF センサ用 |
| **IRQ** | **I**nterrupt **R**e**Q**uest（割り込み要求） | デバイスがホストへ「用事ができた」と知らせる信号線。**ポーリング（定期的に聞きに行く）より応答が速い**。方針は `docs/IRQ_POLICY.md` |
| **ポーリング** | polling | 割り込みを使わず、ホストが定期的に状態レジスタを読みに行く方式。本実装は 1 ms 周期 |
| **GPIO** | **G**eneral **P**urpose **I**nput/**O**utput（汎用入出力） | マイコンの汎用ピン。本文書では `G13` のように書く |
| **IO_MUX** | **I**nput/**O**utput **Mu**ltiple**x**er | ESP32-S3 で周辺機能をピンに直結する経路。GPIO マトリクス経由より信号品質が良い |
| **DMA** | **D**irect **M**emory **A**ccess | CPU を介さずメモリ転送する仕組み。SPI 転送に使う |
| **NVS** | **N**on-**V**olatile **S**torage | ESP-IDF の不揮発設定ストレージ。アンカー座標の保存に使う（`components/uwb_cfgstore`） |
| **Kconfig / menuconfig** | — | ESP-IDF のビルド設定機構。`idf.py menuconfig` で対話的に設定する |
| **FPC** | **F**lexible **P**rinted **C**ircuit（フレキシブル基板） | 薄いフィルム状の配線。M5Stamp UWB Module の一部品種はこのコネクタを持つ |
| **キャステレーション** | castellation | 基板の端の半円形の半田パッド。M5StampS3A の側面にある |
| **FPC→DIP 変換基板** | — | 0.5mm ピッチの FPC ケーブルを 2.54mm/DIP ヘッダへ変換する基板。据置機の標準経路（`docs/WIRING.md` 経路A）で、M5Stamp UWB Module with FPC (S017-F) を StampS3 BreakOut に繋ぐのに使う。**接点面（同面／異面）が合わないと 1 番と 12 番が入れ替わる**ので実物で要確認（`docs/WIRING.md` §4.0） |
| **ロードスイッチ** | load switch | 電源レールを半導体で ON/OFF する IC。M5StampS3A では **AW35122FDR (U2)** が背面 FPC の **BL_3V3** 系統（バックライト・オンボード RGB LED 兼用）を切り替えており、EN が **G38 (DISP_BL)** に配線されている（`boards/stampfly.h`） |
| **BL_3V3 / VDD_3V3 の区別** | — | M5StampS3A 背面 12P FPC にある2系統の3.3V。**BL_3V3**（位置5）はロードスイッチの出力で **G38 が Low になると落ちる**。**VDD_3V3**（位置11）はロードスイッチの**上流**で常時給電。**UWB モジュールの電源は必ず VDD_3V3 から取る**（`boards/stampfly.h`「■ 電源（重要）」、`docs/WIRING.md` §3.5） |
| **LDO** | **L**ow **D**rop**o**ut regulator | 低損失の降圧レギュレータ。M5Stack 一般の GROVE は 5V 出力が多いが、**StampFly の GROVE は電池電圧（満充電 ~4.35V、チップの絶対最大 4.0V 超）**であり、いずれも 3.3V を作るのに要る。**StampFly 搭載タグの標準経路（背面 12P FPC の VDD_3V3 給電、旧 GROVE 4線構成は廃案）では LDO は不要**（`boards/stampfly.h`） |
| **XSHUT** | — | ToF 距離センサのシャットダウン端子。同一 I2C アドレスの複数個を個別に初期化するのに使う |
| **PWM** | **P**ulse **W**idth **M**odulation（パルス幅変調） | StampFly のモータ駆動信号 |

---

## 7. 製品名・型番

| 名前 | 何か |
|---|---|
| **M5Stamp UWB Module** | M5Stack の UWB モジュール（**通称 M5Stamp UWB**）。中身は Qorvo QM33120W。**本プロジェクトの対象**。FPC コネクタ付きの品種（**M5Stamp UWB Module with FPC**）もある |
| **QM33120W** | Qorvo の UWB トランシーバ IC。**DW3720 系**。Device ID = `0xDECA0314` |
| **DW3000 / DW3720 / DW1000** | Decawave（現 Qorvo）の UWB IC 系列。DW1000 が旧世代。**単位系の定数が世代で違うので混同注意**（`docs/GLOSSARY.md`） |
| **M5StampS3A** | ESP32-S3 の小型ボード。**既定のホスト（タグ・アンカーとも）**。据置機は **StampS3 BreakOut** に載せて使う（`UWB_*_BOARD_STAMPS3` が既定）。StampFly 搭載タグは背面 12P FPC 経由（`UWB_TAG_BOARD_STAMPFLY`）。旧 M5StampS3 と互換 |
| **AtomS3 / AtomS3R** | ESP32-S3 の小型ボード（LCD 付き）。**アンカーホストの代替**（`UWB_ANCHOR_BOARD_ATOMS3`。既定は M5StampS3A + StampS3 BreakOut に切り替わったが、削除はされていない）。R が現行版で、IMU が G0/G45 に移っている分 G38/G39 が空く |
| **StampS3 BreakOut** | M5StampS3A の 1.27mm ピンを 2.54mm へ変換する M5Stack 純正の拡張基板。露出 IO 23本（G0-G15, G39-G44, G46）は M5StampS3A 単体と同じで、BreakOut を使っても使わなくてもピン定義は同一。Grove ポートは **G13/G15** に配線されており、**G13 は UWB の SPI MISO と衝突する**ので ToF 増設等は Grove に挿すだけでは済まない（`boards/stamps3.h`、`docs/SURVEY_SPEC.md` §3.5）。G0/EN のタクトスイッチ付きで書き込みモードに入りやすい |
| **StampFly** | M5StampS3A を積んだマルチコプター機体。**本リポジトリは非依存だが、タグの配線だけ互換を保つ**（`docs/PLAN.md` §1） |
| **ESP-IDF** | Espressif **I**o**T** **D**evelopment **F**ramework。ESP32 系の公式 SDK。本プロジェクトのビルド環境 |
| **uwb_localizer** | 測位ソルバの上流リポジトリ（本プロジェクト作者のもの）。`components/uwb_loc/` として無改造で取り込んでいる |

---

## 8. 資料の略号

| 略号 | 意味 |
|---|---|
| **APS** | Qorvo の **A**pplication **N**ote（例: **APS014** = アンテナ遅延校正の手順書） |
| **UM** | **U**ser **M**anual（DW3000 ファミリのユーザマニュアル） |
| **一次資料** | Qorvo の SDK / UM / APS。**信頼してよい**。M5Stack のラッパは二次資料として扱う（`docs/PLAN.md` §5） |
| **R1〜R12 / S1〜S7** | 本プロジェクトの作業項目番号。R = 再実装項目（`docs/archive/REIMPL_PLAN.md`）、S = 自動測量関連（`docs/SURVEY_SPEC.md`） |

---

## 9. ⚠ 紛らわしい語

### 「ToF」が2つの意味で使われている
| 文脈 | 意味 |
|---|---|
| **測距の話**（TWR、タイムスタンプ、距離計算） | **T**ime **o**f **F**light = 電波の飛行時間 |
| **センサの話**（高さ自動計測、I2C、VL53L3CX、XSHUT） | **ToF 距離センサ** = 赤外線の飛行時間で距離を測る部品 |

本リポジトリでは**両方出てくる**（自動測量で高さを実測するのに ToF センサを使うため）。
文脈で判断すること。

### 「遅延（delay）」も2つある
| 用語 | 意味 |
|---|---|
| **アンテナ遅延** | 校正で求めて**距離から引く**もの。単位はメートルまたは DTU |
| **`*Uus` の遅延** | TWR の**送信タイミングの設計値**。単位は UUS（`docs/TIMING_PRESETS.md`） |

### 「UUS」は実マイクロ秒ではない
1 UUS = 1.02564 µs。**Qorvo 公式サンプルの `*_UUS` という名前の定数は、名前に反して実マイクロ秒**。
→ `docs/GLOSSARY.md` §3

### 「Lv1」が飛んでいる
`SolverLevel` は Lv0 / Lv2 / Lv3 の3つ。Lv1 は上流 uwb_localizer 側に存在する段階で、
本リポジトリの enum には出していない（`docs/PERF_ANALYSIS.md` には Lv1 の記述が残っている）。

---

## 関連文書
- `docs/UWB_PRIMER.md` — UWB 入門（原理。用語の背景が分かる）
- `docs/UWB_ALGORITHMS.md` — 測位アルゴリズムの導出
- `docs/GLOSSARY.md` — UUS / DTU / 実 µs の単位リファレンス
- `docs/TIMING_PRESETS.md` — 遅延プリセットとその導出
- `docs/PLAN.md` §5 — 資料の格付けと実装方針
- `docs/GETTING_STARTED.md` — BOM から測位までの手順

---

## 単位リファレンス（旧 UNITS.md）

この節は 2026-08-22 に `docs/GLOSSARY.md` を統合したもの。

**TWR の遅延値を読み書きする前に必ずここを読むこと。**
この単位を取り違えると測距が成立しない。しかも症状は「距離が出ない」だけで、
どこがずれているのかログからは分からない。

---

### 1. 三つの単位

| 単位 | 定義 | 値 |
|---|---|---|
| **DTU** | `1 / (499.2 MHz × 128)` | **≒ 15.65 ps** |
| **UUS** | 499.2 MHz の **512 周期** | **512/499.2 µs = 1.02564… µs** |
| 実マイクロ秒 | 普通の µs | 1 µs |

499.2 MHz は DW3000 の UWB 基本クロック（UWB チャネルの中心周波数もこの整数倍）。
チップ内のタイムスタンプ用カウンタ（40bit）は DTU で刻まれている。

#### なぜ「1 µs ちょうど」にしなかったのか

```
1 UUS = 512 × 128 = 65536 DTU = 2^16 DTU
```

**UUS は DTU のちょうど 16bit シフト**である。ハードのカウンタから見て
桁合わせがシフト1回で済む単位を選んだ結果、実 µs から 2.56% ずれた。

これが `components/uwb_qm33120/src/uwb_qm33120_internal.hpp:192` の

```cpp
static constexpr uint64_t kUusToDwtTime = 65536ULL;
```

の正体である。

参考: 実マイクロ秒 → DTU は `1e-6 / 15.65e-12 = 63897.6 ≒ 63898`。
**65536 / 63898 = 1.02564** ＝ 1 UUS の実 µs 換算そのもの。

---

### 2. どの API がどの単位を取るか

| API | 単位 | 出典 |
|---|---|---|
| `dwt_setrxaftertxdelay()` | **UUS** | `components/qm33120w_sdk/deca_device_api.h:2681`「The delay is in **UWB microseconds**, 20-bit value」 |
| `dwt_setrxtimeout()` | **UUS** | 同 :2360「in **1.0256 us** (512/499.2MHz) units」 |
| `dwt_setdelayedtrxtime()` | **DTU >> 8**（40bit カウンタの上位 32bit） | ラッパ側で `× kUusToDwtTime` してから渡す |

#### 本プロジェクトのフィールドの内訳

`RangeConfig` / `DSRangeConfig`（`components/uwb_qm33120/include/uwb_qm33120_types.hpp`）の
`*Uus` フィールドは**すべて UUS**。ただし SDK へ渡るまでの経路が 2 通りある。

| フィールド | 経路 |
|---|---|
| `responseRxAfterTxDelayUus` | UUS のまま `dwt_setrxaftertxdelay()` へ |
| `finalRxAfterResponseTxDelayUus` | 同上 |
| `resultRxAfterFinalTxDelayUus` | 同上 |
| `rxTimeoutUus` | UUS のまま `dwt_setrxtimeout()` へ |
| **`responseTxDelayUus`** | **× 65536 して DTU 化 → `dwt_setdelayedtrxtime()`** |
| **`finalTxDelayUus`** | 同上 |

`hostTimeoutMs` だけは **ms**（ホスト側ポーリングループの上限。SDK API には渡らない）。

---

### 3. 【罠】Qorvo 公式サンプルの `*_UUS` 定数は実マイクロ秒

`docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/shared_data/shared_defines.h:37`

```c
#define UUS_TO_DWT_TIME 63898
```

63898 は **「実 µs → DTU」の係数**である。つまり Qorvo 公式サンプルが
`*_UUS` という名前で定義している定数は、**名前に反して実マイクロ秒**で書かれている。

| | 係数 | 意味 |
|---|---|---|
| Qorvo 公式 `shared_defines.h:37` | `UUS_TO_DWT_TIME 63898` | **実マイクロ秒** → DTU |
| M5Stack / 本プロジェクト | `kUusToDwtTime 65536` | **真の UUS** → DTU |

**⇒ Qorvoの値を `*Uus` フィールドへそのまま代入すると、意図した遅延より約 2.5% 長くなる。**

> **命名が誤っているのは Qorvo 側であり、M5Stack 由来の `Uus` という命名は正しい。**

#### 変換ヘルパを必ず通すこと

`components/uwb_qm33120/include/uwb_qm33120_units.hpp`

```cpp
constexpr uint32_t usToUus(uint32_t us);   // 実µs -> UUS。四捨五入
```

ESP-IDF / Qorvo SDK のヘッダに依存しないので、ホスト側テスト
（`tests/host/pipeline`）からそのまま include して検算できる。

---

### 4. 実例: Qorvo `ex_05b_ds_twr_resp` の値を持ってくる

Qorvo 公式の DS-TWR responder サンプル（本プロジェクトのアンカーに相当）:

```c
/* docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/ex_05b_ds_twr_resp/ds_twr_responder.c */
#define POLL_RX_TO_RESP_TX_DLY_UUS  900   /* :85 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500   /* :87 */
#define FINAL_RX_TIMEOUT_UUS        220   /* :89 */
```

この 900 は**実 900 µs** であって 900 UUS ではない。本プロジェクトへ持ってくるなら

```
usToUus(900) = 878   ->  DSRangeConfig::responseTxDelayUus = 878
usToUus(500) = 488   ->  DSRangeConfig::finalRxAfterResponseTxDelayUus = 488
```

`900` と書いてしまうと 923 µs、つまり **23 µs（2.56%）長くなる**。

`docs/TIMING_PRESETS.md` の `AnchorIrq` プリセットが `responseTxDelayUus = 878` に
なっているのはこの換算の結果であり、**Qorvo 公式の DS-TWR responder と同じ実時間**である。

> **⚠ `.c` ファイルは非 UTF-8（latin-1）。`grep` が 0 件を返したらまずこれを疑うこと。
> `grep -a` か `iconv` を使う**（`docs/HANDOFF.md` 運用ルール 3）。

---

### 5. 早見表

| 実 µs | UUS | 用途 |
|---:|---:|---|
| 205 | 200 | `resultRxAfterFinalTxDelayUus`（PollingBoth） |
| 513 | 500 | `finalRxAfterResponseTxDelayUus` |
| 700 | 683 | `finalTxDelayUus`（BothIrq） |
| 900 | 878 | `responseTxDelayUus`（IRQ プリセット。Qorvo 公式と同値） |
| 1231 | 1200 | `rxTimeoutUus`（IRQ プリセット） |
| 1400 | 1365 | `finalTxDelayUus`（AnchorIrq） |
| 1846 | 1800 | `finalTxDelayUus`（PollingBoth、現行既定） |
| 3077 | 3000 | `responseTxDelayUus`（PollingBoth、現行既定） |

```
UUS -> 実µs :  x * 1.02564        （x * 512 / 499.2）
実µs -> UUS :  x * 0.975          （x * 4992 / 5120、usToUus() が四捨五入で実装）
UUS -> DTU  :  x * 65536
```

### 関連文書
- `docs/GLOSSARY.md` — 略語の一覧（UUS 以外の用語はこちら）
- `docs/TIMING_PRESETS.md` — 遅延プリセットの実際の値と、その導出
- `docs/archive/REIMPL_PLAN.md` R1 — この単位問題を発見・訂正した経緯
- `docs/PLAN.md` §5 — 「一次資料 = Qorvo SDK/UM/APS」の格付け
