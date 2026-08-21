# 用語集（UWB 測距・測位）

**このリポジトリの文書で使う略語と専門用語の一覧。**
分からない略語が出てきたらここを引くこと。

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
| **アンカー** (anchor) | — | **座標が既知の基準局**。動かさない。本プロジェクトでは 5 台（AtomS3(R)）。タグからの問い合わせに応える側なので **responder（レスポンダ）** とも呼ぶ |

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

**詳細は `docs/UNITS.md`（遅延値を触る前に必読）。**

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
| **LDO** | **L**ow **D**rop**o**ut regulator | 低損失の降圧レギュレータ。M5Stack 一般の GROVE は 5V 出力が多いが、**StampFly の GROVE は電池電圧（満充電 ~4.35V、チップの絶対最大 4.0V 超）**であり、いずれも 3.3V を作るのに要る |
| **XSHUT** | — | ToF 距離センサのシャットダウン端子。同一 I2C アドレスの複数個を個別に初期化するのに使う |
| **PWM** | **P**ulse **W**idth **M**odulation（パルス幅変調） | StampFly のモータ駆動信号 |

---

## 7. 製品名・型番

| 名前 | 何か |
|---|---|
| **M5Stamp UWB Module** | M5Stack の UWB モジュール（**通称 M5Stamp UWB**）。中身は Qorvo QM33120W。**本プロジェクトの対象**。FPC コネクタ付きの品種（**M5Stamp UWB Module with FPC**）もある |
| **QM33120W** | Qorvo の UWB トランシーバ IC。**DW3720 系**。Device ID = `0xDECA0314` |
| **DW3000 / DW3720 / DW1000** | Decawave（現 Qorvo）の UWB IC 系列。DW1000 が旧世代。**単位系の定数が世代で違うので混同注意**（`docs/UNITS.md`） |
| **M5StampS3A** | ESP32-S3 の小型ボード。**タグのホスト**。旧 M5StampS3 と互換 |
| **AtomS3 / AtomS3R** | ESP32-S3 の小型ボード（LCD 付き）。**アンカーのホスト**。R が現行版で、IMU が G0/G45 に移っている分 G38/G39 が空く |
| **StampFly** | M5StampS3A を積んだマルチコプター機体。**本リポジトリは非依存だが、タグの配線だけ互換を保つ**（`docs/PLAN.md` §1） |
| **ESP-IDF** | Espressif **I**o**T** **D**evelopment **F**ramework。ESP32 系の公式 SDK。本プロジェクトのビルド環境 |
| **uwb_localizer** | 測位ソルバの上流リポジトリ（本プロジェクト作者のもの）。`components/uwb_loc/` として無改造で取り込んでいる |

---

## 8. 資料の略号

| 略号 | 意味 |
|---|---|
| **APS** | Qorvo の **A**pplication **N**ote（例: **APS014** = アンテナ遅延校正の手順書） |
| **UM** | **U**ser **M**anual（DW3000 ファミリのユーザマニュアル） |
| **一次資料** | Qorvo の SDK / UM / APS。**信頼してよい**。M5Stack のラッパは二次資料として扱う（`docs/SOURCE_POLICY.md`） |
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
→ `docs/UNITS.md` §3

### 「Lv1」が飛んでいる
`SolverLevel` は Lv0 / Lv2 / Lv3 の3つ。Lv1 は上流 uwb_localizer 側に存在する段階で、
本リポジトリの enum には出していない（`docs/PERF_ANALYSIS.md` には Lv1 の記述が残っている）。

---

## 関連文書
- `docs/UWB_PRIMER.md` — UWB 入門（原理。用語の背景が分かる）
- `docs/UWB_ALGORITHMS.md` — 測位アルゴリズムの導出
- `docs/UNITS.md` — UUS / DTU / 実 µs の単位リファレンス
- `docs/TIMING_PRESETS.md` — 遅延プリセットとその導出
- `docs/SOURCE_POLICY.md` — 資料の格付けと、過去の誤りの記録
- `docs/GETTING_STARTED.md` — BOM から測位までの手順
