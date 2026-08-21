# UWB 測位アルゴリズム — 導出と実装の詳細

> **この文書について（本リポジトリ向け移植版）**
>
> 原典は [kouhei1970/uwb_localizer](https://github.com/kouhei1970/uwb_localizer) の
> `docs/UWB_ALGORITHMS.md`（MIT License, Copyright (c) 2026 Kouhei Ito）であり、
> 本文書は本リポジトリ（m5stamp_uwb_localizer）向けに改訂した移植版である。
>
> 本リポジトリの測位ソルバ `components/uwb_loc/` は、上流 uwb_localizer の C 版ソルバを
> **無改造で vendoring** したもの（取り込み元コミット `8d0edc057ed05cf6b4af91df329999fe2343f515`。
> 詳細は [`THIRD_PARTY_LICENSES.md`](../THIRD_PARTY_LICENSES.md)）である。
> **したがって、この文書の数式・導出はそのまま本リポジトリの実装の説明になる。**
> 改訂したのは「本リポジトリのどこに実装があるか」「本リポジトリで何が有効か」を示す注記の追加だけであり、
> 原典の数式・導出・記号の定義・章立ての骨格は一切変更していない。
>
> **読む順路**: [`docs/UWB_PRIMER.md`](UWB_PRIMER.md)（物理と測距の原理。**準備中** — 別途作成中のためリンクのみ先行して張ってある）
> → 本書（測位の数理）→ [`docs/ANCHOR_PLACEMENT.md`](ANCHOR_PLACEMENT.md)（アンカー配置の指針）。
>
> 原典が参照する周辺文書（なぜその手法を選んだかの経緯 = `DESIGN.md`、Python 版の使い方 = `TUTORIAL.md`、
> 原典のドキュメント一覧 = `README.md`）は本リポジトリの `docs/` には移植していない。読みたい場合は
> vendoring 元の [`third_party/uwb_localizer/docs/`](https://github.com/kouhei1970/uwb_localizer/tree/HEAD/docs)
> （[DESIGN.md](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/docs/DESIGN.md) /
> [TUTORIAL.md](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/docs/TUTORIAL.md)）を直接参照すること。
>
> 略語の表記は本リポジトリの用語集 [`docs/GLOSSARY.md`](GLOSSARY.md) の表記ルールとも整合させてある。
>
> **数値例・実測値について**: 本文中の「実測」は、断りが無い限り**上流 uwb_localizer 側の検証
> （ホスト PC 上のテスト・シミュレーション。§11 の比較表は乱数シード違いの合成観測列を使っている）
> によるもの**であり、本リポジトリ（m5stamp_uwb_localizer）の実機で測ったものではない。本リポジトリはまだ
> 実機での測距・測位を一度も確認していない（[`README.md`](../README.md) の「現状」参照）。
> 数値そのものは原典のまま変更していない。

## この文書と本リポジトリの実装の対応

| 節 | 内容 | 本リポジトリでの実装 |
|---|---|---|
| [§1.1](#11-距離-twr) 観測モデル（距離・TWR） | 残差とヤコビアン | `components/uwb_loc/src/uwb_model.c`（`uwb_evaluate()`）。測距そのものは `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` の `requestRange()`（SS-TWR）/ `requestDSRange()`（DS-TWR） |
| [§1.2](#12-距離差-tdoa) 距離差（TDoA） | 背景知識。**本リポジトリでは未使用** | — |
| [§1.3](#13-方位角・仰角-aoa--pdoa) 方位角・仰角（AoA/PDoA） | 背景知識。**本リポジトリでは未使用** | — |
| [§2](#2-lv0--線形最小二乗-lls) Lv0 LLS | 線形最小二乗 | `components/uwb_loc/src/uwb_closed_form.c`（`uwb_lls_trilateration()`） |
| [§3](#3-beck-の厳密解-gtrs--lv2-の初期解) Beck 厳密解（GTRS） | Lv2 の初期解 | `components/uwb_loc/src/uwb_closed_form.c`（`uwb_beck_gtrs()`） |
| [§4](#4-lv1--重み付き非線形最小二乗-wnls) Lv1 WNLS | 重み付き非線形最小二乗 | `components/uwb_loc/src/uwb_nls.c`（`uwb_solve_lv1()`。**ただし本リポジトリの `SolverLevel` enum には非公開**。詳細は §4 冒頭の注記） |
| [§5](#5-lv2--ロバスト化) Lv2 ロバスト化 | Huber + 片側損失 | `components/uwb_loc/src/uwb_nls.c`（`uwb_solve_lv2()`） |
| [§6](#6-lv3--密結合-ekf) Lv3 密結合 EKF | カルマンフィルタ | `components/uwb_loc/src/uwb_ekf.c`（`uwb_ekf_init()` / `uwb_ekf_predict()` / `uwb_ekf_update()`） |
| [§7](#7-tdoa-の-chan-法) TDoA の Chan 法 | 背景知識。**本リポジトリでは未実装**（TDoA 自体を使わないため） | — |
| [§8](#8-gdop-と-crlb--配置を数字で評価する) GDOP・CRLB | 配置評価 | `components/uwb_loc/src/uwb_model.c`（`uwb_gdop_at()` / `uwb_crlb_at()` / `uwb_anchors_coplanar()`） |
| [§9](#9-アンカーの自己測量) 自己測量（MDS） | アンカー座標の自動復元 | `components/uwb_loc/` には**対応する関数が無い**（上流 Python 版 `calibration.py:self_survey` の C 移植は無い）。本リポジトリでは同じ考え方（古典的 MDS + Gauss-Newton 精密化）を `components/uwb_survey/` が ESP-NOW 前提で独自に実装している（`docs/SURVEY_SPEC.md`）。コードの系統はこの文書の §9 とは別 |
| [§10](#10-アンテナ遅延の推定) アンテナ遅延の推定 | 遅延の分離推定 | `components/uwb_loc/` には**対応する関数が無い**。本リポジトリでは手動校正手順（`docs/GETTING_STARTED.md` §9「アンテナ遅延の校正」）と、`components/uwb_survey/` の自己測量が推定する共通遅延 δ（`docs/SURVEY_SPEC.md`）の 2 経路がある |

（各節冒頭の「実装:」注記は原典 (Python 版) のファイル参照のまま残してある。本リポジトリでの対応は上表を見ること。）

---

## この文書の読み方

**学部 2〜3 年の数学で読めるように書いてある。** 前提にしているのは

- 線形代数 — 行列とベクトル、転置、逆行列、固有値、対称行列
- 微分積分 — 偏微分、勾配、テイラー展開の 1 次まで
- 確率統計 — 平均・分散、正規分布、最小二乗法

これ以外 (最尤推定、カルマンフィルタ、M 推定など) は出てくるところで説明する。
**式の変形は途中を飛ばさない。** 飛ばしているように見えたら、それはこちらの
書き漏らしなので指摘してほしい。

節はこの順で読むと積み上がる。

| 節 | 内容 | 前の節への依存 |
|---|---|---|
| [1](#1-観測モデル--すべての土台) | 観測モデル (残差とヤコビアン) | — |
| [2](#2-lv0--線形最小二乗-lls) | Lv0 線形最小二乗 | 1 |
| [3](#3-beck-の厳密解-gtrs--lv2-の初期解) | Beck の厳密解 | 2 |
| [4](#4-lv1--重み付き非線形最小二乗-wnls) | Lv1 非線形最小二乗 | 1 |
| [5](#5-lv2--ロバスト化) | Lv2 ロバスト化 | 4 |
| [6](#6-lv3--密結合-ekf) | Lv3 カルマンフィルタ | 1, 4 |
| [7](#7-tdoa-の-chan-法)〜[10](#10-アンテナ遅延の推定) | TDoA・配置評価・自己測量・校正 | 1, 4 |

---

## 略語

**初出で必ず展開する**が、まとめて置いておく。本リポジトリの用語集
[`docs/GLOSSARY.md`](GLOSSARY.md) の表記ルール（「正式名称（英語）＝日本語の意味」を初出で添える）とも揃えてある。

| 略語 | 英語 | 意味 |
|---|---|---|
| **UWB** | Ultra-WideBand | 超広帯域無線。帯域が広い = 時間分解能が高いので、電波の到達時刻を精密に測れる |
| **LOS** | Line-Of-Sight | 見通し。送受信の間に遮蔽物が無い状態 |
| **NLOS** | Non-Line-Of-Sight | 見通し外。遮蔽されて電波が回り込むため、**距離が実際より長く出る** |
| **ToF** | Time-of-Flight | 電波の飛行時間。距離 = 光速 × 飛行時間 |
| **TWR** | Two-Way Ranging | 双方向測距。往復の時間を測って距離を求める方式。**時計を合わせなくてよい**のが利点 |
| **TDoA** | Time Difference of Arrival | 到達時間差。2 つのアンカーへの**距離の差**を測る方式。アンカー間の時計同期が要る |
| **AoA** | Angle of Arrival | 到来角。電波が来た方向 |
| **PDoA** | Phase Difference of Arrival | 位相差。複数アンテナの位相差から AoA を求める方式 |
| **LLS** | Linear Least Squares | 線形最小二乗法 |
| **WLS** | Weighted Least Squares | 重み付き最小二乗法 |
| **NLS** | Nonlinear Least Squares | 非線形最小二乗法 |
| **WNLS** | Weighted Nonlinear Least Squares | 重み付き非線形最小二乗法 |
| **MLE** | Maximum Likelihood Estimation | 最尤推定。「観測がいちばん起こりやすい」パラメータを選ぶ |
| **GN** | Gauss-Newton | ガウス・ニュートン法。非線形最小二乗の反復解法 |
| **LM** | Levenberg-Marquardt | レーベンバーグ・マルカート法。GN に減衰を入れて安定化したもの |
| **GTRS** | Generalized Trust Region Subproblem | 一般化信頼領域部分問題。「2 次関数を 2 次拘束のもとで最小化する」問題 |
| **IRLS** | Iteratively Reweighted Least Squares | 反復再重み付き最小二乗。M 推定を最小二乗の繰り返しで解く手法 |
| **RANSAC** | RANdom SAmple Consensus | 少数の観測から仮の解を作り、多数決で外れ値を見つける手法 |
| **EKF** | Extended Kalman Filter | 拡張カルマンフィルタ。非線形な観測を線形化して扱うカルマンフィルタ |
| **CV / CA** | Constant Velocity / Constant Acceleration | 等速度モデル / 等加速度モデル |
| **MDS** | MultiDimensional Scaling | 多次元尺度構成法。距離の表から座標を復元する手法 |
| **GDOP** | Geometric Dilution Of Precision | 幾何精度劣化係数。アンカーの置き方による誤差の増幅率 |
| **CRLB** | Cramér-Rao Lower Bound | クラメール・ラオ下限。不偏推定量の分散の理論的な下限 |
| **RMS** | Root Mean Square | 二乗平均平方根。値の大きさの代表値 |
| **RMSE** | Root Mean Square Error | 二乗平均平方根誤差。RMS を誤差に適用したもの |
| **CEP** | Circular Error Probable | 誤差がある半径以内に収まる確率。CEP95 なら 95% が入る半径 |
| **HAL** | Hardware Abstraction Layer | ハードウェア抽象化層。チップごとの差を吸収する層 |
| **GPS** | Global Positioning System | 全地球測位システム。衛星からの距離で測位する。原理は UWB と同じ |
| **LU** | Lower-Upper (decomposition) | LU 分解。行列を下三角 × 上三角に分けて連立方程式を解く方法 |

---

## 記号

**この文書で使う記号を全部挙げる。** 節ごとに追加される記号はその節の冒頭にも書く。

### 基本 (全節で共通)

| 記号 | 意味 | 備考 |
|---|---|---|
| $d$ | 空間の次元 | 2 か 3 |
| $n$ | 1 エポックで使える観測の本数 | 「エポック」= ほぼ同時刻に取れた観測のひとまとまり |
| $p \in \mathbb{R}^d$ | **タグの位置** (求めたいもの) | $p = (p_x, p_y, p_z)^\top$ |
| $\hat{p}$ | $p$ の推定値 | ハットは「推定値」の意味 |
| $a_i \in \mathbb{R}^d$ | アンカー $i$ の座標 (既知) | $i = 1,\dots,n$ |
| $d_i = \lVert p - a_i \rVert$ | タグとアンカー $i$ の**真の距離** | $\lVert\cdot\rVert$ はユークリッドノルム |
| $r_i$ | 測距値 (距離の観測値) | $r_i = d_i + (\text{誤差})$ |
| $z_i$ | 観測値一般 | 距離なら $z_i = r_i$、角度なら角度 |
| $h_i(p)$ | **観測モデル**。位置 $p$ なら観測 $i$ はいくつになるか | 距離観測なら $h_i(p) = \lVert p - a_i\rVert$ |
| $e_i = z_i - h_i(p)$ | **残差**。観測と予測のずれ | 符号の約束は [§1.4](#14-残差の符号の約束) |
| $n_i$ | 観測 $i$ の誤差 | $z_i = h_i(p^\text{真}) + n_i$ |
| $\sigma_i$ | 観測 $i$ の標準偏差 | 「この観測はどれくらいばらつくか」 |
| $w_i = 1/\sigma_i^2$ | 観測 $i$ の**重み** | ばらつく観測ほど軽く扱う |
| $u_i = (p - a_i)/d_i$ | アンカー $i$ から**タグへ向かう単位ベクトル** | $\lVert u_i\rVert = 1$ |
| $e \in \mathbb{R}^n$ | 残差を縦に並べたベクトル | $e = (e_1,\dots,e_n)^\top$ |
| $J \in \mathbb{R}^{n \times d}$ | **ヤコビアン行列**。第 $i$ 行が $\partial h_i/\partial p$ | 「位置を少し動かすと観測がどれだけ変わるか」 |
| $W = \mathrm{diag}(w_1,\dots,w_n)$ | 重み行列 (対角行列) | |
| $H$ | **幾何行列**。第 $i$ 行が $u_i^\top$ | 距離観測では $J = H$ |
| $I_k$ | $k \times k$ の単位行列 | 添字を省くこともある |
| $\mathbf{1}$ | 全成分が 1 のベクトル | |

### 記号の重複について

$u$ という文字を 2 通りに使う。紛らわしいので**ここで宣言しておく**。

| 記号 | 意味 | 出てくる節 |
|---|---|---|
| $u_i$ (添字が観測番号) | アンカー $i$ からタグへの**単位ベクトル** ($d$ 次元) | 1, 3, 4, 6, 8, 9 |
| $t_i = e_i/\sigma_i$ | **標準化残差** (スカラー)。残差を標準偏差で割ったもの | 5 |

標準化残差はロバスト統計の文献では $u$ と書くのが普通だが、この文書では
単位ベクトルと衝突するので **$t_i$** を使う。

### 行列に関する記号

| 記号 | 読み | 意味 |
|---|---|---|
| $A^\top$ | エー転置 | 転置行列 |
| $A^{-1}$ | エー逆 | 逆行列 |
| $\mathrm{tr}(A)$ | トレース | 対角成分の和 |
| $A \succ 0$ | 正定値 | 任意の $x \ne 0$ で $x^\top A x > 0$ |
| $A \succeq B$ | | $A - B$ が半正定値 ($x^\top(A-B)x \ge 0$) |
| $A \otimes B$ | クロネッカー積 | $A$ の各成分を $B$ 倍した block を並べた行列 |
| $\mathrm{diag}(\cdot)$ | | 対角行列を作る |
| $\lVert x \rVert$ | ノルム | $\sqrt{x^\top x}$ |

---

## 1. 観測モデル — すべての土台

実装: `uwb_loc/model.py`

> **注記（本リポジトリでの実装について）**: 以下、各節冒頭の「実装:」は原典
> （上流 uwb_localizer の Python パッケージ `uwb_loc`）でのファイル参照であり、
> **本リポジトリには Python API は含まれない**（本リポジトリはファームウェアであり、C/C++ のみ）。
> 数式の説明に必要な範囲でこの後もこの形の参照を残すが、擬似コードとして読むこと。
> 本リポジトリでの実装（C 版）は、この文書冒頭の「この文書と本リポジトリの実装の対応」表を見ること。

**このライブラリの設計の要**は、どの観測種別も

$$
(\text{残差 } e_i,\quad \text{ヤコビアン } \partial h_i/\partial p,\quad \text{標準偏差 } \sigma_i)
$$

という 3 つ組に落とすこと。下流のソルバ (最小二乗もカルマンフィルタも) は
「これが距離なのか角度なのか」を一切知らない。だから観測種別を増やしても
ソルバを直さなくてよい。

### 1.1 距離 (TWR)

> **本リポジトリでの位置づけ**: TWR による距離観測が**本リポジトリの本線**（唯一実装している測距方式。
> §1.2/§1.3 の TDoA・AoA/PDoA は未使用）。測距そのものを行っているのは
> `components/uwb_qm33120/src/uwb_qm33120_twr.cpp` の `requestRange()`（SS-TWR）/
> `requestDSRange()`（DS-TWR）。以下の観測モデルとヤコビアンは、そこで得た測距値を
> 測位ソルバ（`components/uwb_loc/`）に渡す際にそのまま使われる。

**TWR** (Two-Way Ranging, 双方向測距) は、電波を往復させてその時間から
距離を求める方式。片道の時刻を測る方式と違い、**送信側と受信側の時計が
合っている必要が無い**のが利点で、UWB モジュールの多くがこれを使う。

観測モデルは素直に「タグとアンカーの距離」。

$$
h_i(p) = \lVert p - a_i \rVert
$$

ヤコビアンを求める。まず成分で書くと

$$
h_i(p) = \sqrt{\sum_{k=1}^{d} (p_k - a_{i,k})^2}
$$

第 $k$ 成分で偏微分する。合成関数の微分で、外側 $\sqrt{\cdot}$ の微分が
$\frac{1}{2\sqrt{\cdot}}$、内側 $(p_k - a_{i,k})^2$ の微分が $2(p_k - a_{i,k})$ なので

$$
\frac{\partial h_i}{\partial p_k}
= \frac{1}{2\sqrt{\sum_l (p_l - a_{i,l})^2}} \cdot 2(p_k - a_{i,k})
= \frac{p_k - a_{i,k}}{\lVert p - a_i\rVert}
$$

これを $k = 1,\dots,d$ で並べれば

$$
\frac{\partial h_i}{\partial p} = \frac{p - a_i}{\lVert p - a_i\rVert} = u_i
$$

**ヤコビアンがちょうど単位ベクトルになる。** これがこの問題の性質を決めている。
なぜなら、ヤコビアンを縦に並べた幾何行列

$$
H = \begin{bmatrix} u_1^\top \\ \vdots \\ u_n^\top \end{bmatrix} \in \mathbb{R}^{n \times d}
$$

が**距離を含まず、方向だけで決まる**からである。後で出てくる精度の幾何依存性
(GDOP、 [§8](#8-gdop-と-crlb--配置を数字で評価する)) が「アンカーがどの方向から
見えるか」だけの関数になるのは、この事実の帰結。

### 1.2 距離差 (TDoA)

> **本リポジトリでは未使用**（`components/uwb_qm33120` は TWR のみを実装しており、
> TDoA に要るアンカー間の時計同期は無い）。背景知識として原典のまま残す。

**TDoA** (Time Difference of Arrival, 到達時間差) は、基準アンカー $\text{ref}$ との
**距離の差**を観測する方式。タグは電波を出すだけでよいが、アンカー同士の
時計が合っている必要がある。

$$
h_i(p) = \lVert p - a_i \rVert - \lVert p - a_\text{ref} \rVert
$$

ヤコビアンは §1.1 の結果を 2 回使って、差の微分は微分の差なので

$$
\frac{\partial h_i}{\partial p} = u_i - u_\text{ref}
$$

ここから**TDoA の弱点**が読める。 $\lVert u_i - u_\text{ref}\rVert$ は
三角不等式より高々 2 だが、タグがアンカー群から遠ざかると 2 つの方向が
揃って $u_i \to u_\text{ref}$ となり、**ヤコビアンが 0 に近づく**。

ヤコビアンが小さい = 「位置を動かしても観測がほとんど変わらない」なので、
逆に「観測から位置を決められない」。TDoA がアンカーの外側で急激に悪化するのは
この幾何が原因で、**アルゴリズムを変えても直らない**。

### 1.3 方位角・仰角 (AoA / PDoA)

> **本リポジトリでは未使用**（`components/uwb_qm33120` は TWR による距離測定のみを実装しており、
> AoA/PDoA に要る複数アンテナ・位相差の測定機能は無い）。背景知識として原典のまま残す。

**AoA** (Angle of Arrival, 到来角) は電波が来た方向を測る。複数アンテナの
**位相差**から求める方式を **PDoA** (Phase Difference of Arrival) と呼ぶ。

記号を置く。

$$
\Delta = p - a_i, \qquad \rho = \sqrt{\Delta_x^2 + \Delta_y^2}, \qquad r^2 = \rho^2 + \Delta_z^2
$$

$\rho$ は水平距離、 $r$ は 3 次元距離。

**方位角** (水平面内の角度) は

$$
h(p) = \mathrm{atan2}(\Delta_y, \Delta_x)
$$

微分する。 $\theta = \arctan(\Delta_y/\Delta_x)$ として、 $\arctan$ の微分
$\frac{d}{dx}\arctan x = \frac{1}{1+x^2}$ を使う。

$$
\frac{\partial \theta}{\partial \Delta_x}
= \frac{1}{1 + (\Delta_y/\Delta_x)^2} \cdot \left(-\frac{\Delta_y}{\Delta_x^2}\right)
= \frac{\Delta_x^2}{\Delta_x^2 + \Delta_y^2} \cdot \left(-\frac{\Delta_y}{\Delta_x^2}\right)
= -\frac{\Delta_y}{\rho^2}
$$

$$
\frac{\partial \theta}{\partial \Delta_y}
= \frac{1}{1 + (\Delta_y/\Delta_x)^2} \cdot \frac{1}{\Delta_x}
= \frac{\Delta_x^2}{\rho^2} \cdot \frac{1}{\Delta_x}
= \frac{\Delta_x}{\rho^2}
$$

$\Delta_z$ には依存しないので 0。 $\partial \Delta/\partial p = I$ だから

$$
\frac{\partial h}{\partial p} = \left(-\frac{\Delta_y}{\rho^2},\ \frac{\Delta_x}{\rho^2},\ 0\right)
$$

**仰角** (水平面からの角度) は同様に

$$
h(p) = \mathrm{atan2}(\Delta_z, \rho), \qquad
\frac{\partial h}{\partial p} = \left(-\frac{\Delta_z \Delta_x}{r^2\rho},\ -\frac{\Delta_z \Delta_y}{r^2\rho},\ \frac{\rho}{r^2}\right)
$$

方位角のヤコビアンの大きさを計算すると

$$
\left\lVert \frac{\partial h}{\partial p} \right\rVert
= \sqrt{\frac{\Delta_y^2 + \Delta_x^2}{\rho^4}} = \frac{\rho}{\rho^2} = \frac{1}{\rho}
$$

つまり**水平距離に反比例**する。角度の測定精度 $\sigma_\theta$ が同じでも、
位置の誤差は $\rho\,\sigma_\theta$ に比例して増える (遠いほど不利)。
重み $w = 1/\sigma_\theta^2$ を掛けるだけで、この「遠いアンカーの角度情報は
効きにくい」という正しい振る舞いが自動的に出る。

角度の残差は周期性があるので $(-\pi, \pi]$ に畳む (例: $359° - 1° = -2°$ とする)。

### 1.4 残差の符号の約束

残差の定義には $z - h$ と $h - z$ の 2 通りがあり、どちらでも最小二乗の答えは
同じ。しかしこのライブラリでは **$e = z - h$ に固定**している。

理由は NLOS の物理。**NLOS** (見通し外) では電波が遮蔽物を回り込むため、
経路が実際の直線距離より長くなる。つまり測距値は**必ず伸びる側にずれる**。

$$
z_i = h_i(p^\text{真}) + n_i + \underbrace{b_i}_{\ge 0\ \text{(NLOS バイアス)}}
$$

$e = z - h$ と定義しておけば、NLOS の残差は必ず $e_i > 0$ になる。
この符号の一貫性があるから、[§5.3](#53-片側損失--nlos-の物理を重みに入れる) の
片側損失が「正の残差だけ厳しく見る」と素直に書ける。

### 1.5 品質値の使い方

HAL が「この測距はどれくらい信用できるか」を $q \in [0,1]$ で返せる場合
(1 が見通し良好)、標準偏差を膨らませて反映する。

$$
\sigma_\text{eff} = \sigma \cdot \bigl(1 + 3(1-q)\bigr)
$$

$q=1$ なら等倍、 $q=0$ なら 4 倍。重みは $w = 1/\sigma^2$ なので $1/16$ に落ちる。
係数 3 に理論的な根拠は無く、**現場で調整する前提**の粗い写像。

---

## 2. Lv0 — 線形最小二乗 (LLS)

実装: `uwb_loc/solvers/closed_form.py:lls_trilateration`

**LLS** (Linear Least Squares, 線形最小二乗法) は、非線形な測距方程式を
**無理やり線形にして** 1 回の行列計算で解く方法。反復も初期値も要らない。

### 2.1 なぜ線形にできるのか

測距方程式は

$$
\lVert p - a_i \rVert = r_i
$$

両辺を 2 乗する。

$$
\lVert p - a_i \rVert^2 = r_i^2
$$

左辺を展開する。 $\lVert x - y\rVert^2 = (x-y)^\top(x-y) = x^\top x - 2x^\top y + y^\top y$ なので

$$
\lVert p \rVert^2 - 2a_i^\top p + \lVert a_i \rVert^2 = r_i^2 \qquad (2.1)
$$

ここで**重要な観察**をする。この式で $p$ について非線形なのは
$\lVert p \rVert^2$ の項**だけ**で、しかもその項は**添字 $i$ を含まない**。
つまり全部の式に同じものが乗っている。

だから、基準となるアンカー $j$ の式

$$
\lVert p \rVert^2 - 2a_j^\top p + \lVert a_j \rVert^2 = r_j^2 \qquad (2.2)
$$

を式 (2.1) から引けば、 $\lVert p \rVert^2$ が消える。

$$
(- 2a_i^\top p + \lVert a_i \rVert^2) - (- 2a_j^\top p + \lVert a_j \rVert^2) = r_i^2 - r_j^2
$$

$$
2(a_j - a_i)^\top p = r_i^2 - r_j^2 - \lVert a_i\rVert^2 + \lVert a_j\rVert^2 \qquad (2.3)
$$

**左辺は $p$ について線形**になった。これを $i \ne j$ について並べれば

$$
M p = c, \qquad
M = \begin{bmatrix} 2(a_j - a_{i_1})^\top \\ \vdots \end{bmatrix},\quad
c = \begin{bmatrix} r_{i_1}^2 - r_j^2 - \lVert a_{i_1}\rVert^2 + \lVert a_j\rVert^2 \\ \vdots \end{bmatrix}
$$

$n-1$ 本の式が立つので、 $n - 1 \ge d$ なら最小二乗で解ける。

$$
\hat{p} = (M^\top M)^{-1} M^\top c
$$

反復も初期値も要らず、行列 1 個の逆行列で終わる。

### 2.2 なぜこれで満足してはいけないか

3 つの問題がある。**どれも「2 乗した」ことの副作用**。

**(1) 2 乗による偏り (バイアス)。**
測距値を $r_i = d_i + n_i$ ( $n_i$ は平均 0、分散 $\sigma_i^2$ の誤差) と書くと

$$
r_i^2 = (d_i + n_i)^2 = d_i^2 + 2 d_i n_i + n_i^2
$$

期待値を取ると、 $\mathbb{E}[n_i] = 0$ だが $\mathbb{E}[n_i^2] = \sigma_i^2 \neq 0$ なので

$$
\mathbb{E}[r_i^2] = d_i^2 + \sigma_i^2
$$

**真の値より $\sigma_i^2$ だけ大きい。** これは平均を取っても消えない**系統誤差**。
式 (2.3) で差を取っても、 $\sigma_i^2 - \sigma_j^2$ が残るので完全には消えない。

**(2) 重みが正しくない。**
式 (2.3) の右辺の誤差は主に $2 d_i n_i$ の項から来る (小さい $n_i^2$ を無視)。
その分散は

$$
\mathrm{Var}[2 d_i n_i] = 4 d_i^2 \sigma_i^2
$$

つまり**実効的な分散が距離の 2 乗に比例して膨らむ**。
素直に $w_i = 1/\sigma_i^2$ を使うと、遠いアンカーを過大評価してしまう。

**(3) 行が相関する。**
全ての行が基準アンカーの $r_j$ (したがって誤差 $n_j$) を共有している。
最小二乗法は「各式の誤差が独立」を前提にしているので、この前提が崩れる。
**基準アンカーの選び方で答えが変わる**のもこれが理由。

### 2.3 実装での妥協

- 基準アンカーは**測距値が最小のもの**を選ぶ (通常いちばん S/N が良い)
- 差分行の重みは 2 本の観測の重みの**調和平均** $\sqrt{w_i w_j/(w_i + w_j)}$ で近似

それでも §2.2 の 3 つは残るので、**動作確認と初期値供給専用**と割り切っている。
実測でも Lv0 は Lv1/Lv2 に一貫して劣る ([§11](#11-実測での比較))。

---

## 3. Beck の厳密解 (GTRS) — Lv2 の初期解

実装: `uwb_loc/solvers/closed_form.py:beck_gtrs`

この節で追加される記号:

| 記号 | 意味 |
|---|---|
| $g = \lVert p\rVert^2$ | 補助変数 (位置ベクトルの長さの 2 乗) |
| $y = [p; g] \in \mathbb{R}^{d+1}$ | 拡張した未知数 |
| $A, b$ | 線形化した方程式 $Ay = b$ の係数 |
| $D, f$ | 拘束 $y^\top D y + 2f^\top y = 0$ の係数 |
| $G = A^\top W A$ | 正規方程式の行列 |
| $\lambda$ | ラグランジュ乗数 |
| $\varphi(\lambda)$ | 永年方程式 (これの根を探す) |
| $\gamma$ | 一般化固有値 |

### 3.1 発想 — 消すのではなく昇格させる

LLS は $\lVert p\rVert^2$ を差分で**消した**。その代償が §2.2 の 3 つだった。

Beck 法は逆をやる。 $\lVert p\rVert^2$ を**新しい変数に昇格させる**。

$$
g = \lVert p \rVert^2, \qquad y = \begin{bmatrix} p \\ g\end{bmatrix} \in \mathbb{R}^{d+1}
$$

すると式 (2.1) は

$$
g - 2a_i^\top p + \lVert a_i \rVert^2 = r_i^2
\quad\Longleftrightarrow\quad
\begin{bmatrix} -2a_i^\top & 1\end{bmatrix}
\begin{bmatrix} p \\ g \end{bmatrix}
= r_i^2 - \lVert a_i\rVert^2
$$

**$y$ について完全に線形**になった。全 $i$ について並べて

$$
A y = b, \qquad
A = \begin{bmatrix} -2a_1^\top & 1 \\ \vdots & \vdots \\ -2a_n^\top & 1 \end{bmatrix},\quad
b = \begin{bmatrix} r_1^2 - \lVert a_1\rVert^2 \\ \vdots \\ r_n^2 - \lVert a_n\rVert^2 \end{bmatrix}
$$

差分を取っていないので、**基準アンカーの選択も行間の相関も入らない**。

### 3.2 ただし y は自由ではない

$p$ と $g$ は独立ではなく、 $g = \lVert p\rVert^2$ という関係がある。
これを $y$ の 2 次形式で書く。

$$
y^\top D y + 2 f^\top y = 0, \qquad
D = \begin{bmatrix} I_d & 0 \\ 0 & 0\end{bmatrix},\quad
f = \begin{bmatrix} 0 \\ -1/2\end{bmatrix}
$$

**検算しておく。** $y = [p; g]$ を代入すると

$$
y^\top D y = \begin{bmatrix} p^\top & g\end{bmatrix}
\begin{bmatrix} I_d & 0 \\ 0 & 0\end{bmatrix}
\begin{bmatrix} p \\ g\end{bmatrix} = p^\top p = \lVert p\rVert^2
$$

$$
2 f^\top y = 2 \cdot \left(0^\top p + \left(-\tfrac{1}{2}\right) g\right) = -g
$$

足すと $\lVert p\rVert^2 - g$ で、これが 0 = 拘束そのもの。

### 3.3 解くべき問題

以上をまとめると

$$
\min_y\ (Ay - b)^\top W (Ay - b) \quad \text{s.t.}\quad y^\top Dy + 2f^\top y = 0
$$

**2 次の目的関数 + 2 次の拘束 1 本**。これを **GTRS**
(Generalized Trust Region Subproblem, 一般化信頼領域部分問題) と呼ぶ。

一般に 2 次拘束付きの 2 次計画は非凸で難しい。しかし GTRS には
**双対ギャップが無い**ことが知られており (Moré 1993 など)、
**大域最適解がラグランジュ乗数 1 個だけで書ける**。ここが Beck 法の要点。

### 3.4 ラグランジュ関数から解の形を出す

ラグランジュ関数を作る。

$$
\mathcal{L}(y,\lambda) = (Ay-b)^\top W (Ay-b) + \lambda\,(y^\top Dy + 2f^\top y)
$$

$y$ で微分する。ベクトルの微分公式
$\frac{\partial}{\partial y}(y^\top S y) = 2Sy$ ( $S$ 対称) と
$\frac{\partial}{\partial y}(c^\top y) = c$ を使う。第 1 項を展開すると

$$
(Ay-b)^\top W (Ay-b) = y^\top A^\top W A y - 2 b^\top W A y + b^\top W b
$$

なので

$$
\frac{\partial \mathcal{L}}{\partial y}
= 2 A^\top W A y - 2 A^\top W b + \lambda\,(2Dy + 2f) = 0
$$

両辺を 2 で割って整理すると

$$
(A^\top W A + \lambda D)\,y = A^\top W b - \lambda f
$$

よって、 $\lambda$ を決めれば $y$ が決まる。

$$
\boxed{\ y(\lambda) = (A^\top W A + \lambda D)^{-1}(A^\top W b - \lambda f)\ } \qquad (3.1)
$$

あとは**拘束を満たすような $\lambda$ を 1 つ見つけるだけ**。その条件が

$$
\varphi(\lambda) \equiv y(\lambda)^\top D\,y(\lambda) + 2f^\top y(\lambda) = 0 \qquad (3.2)
$$

この $\varphi$ を**永年方程式** (secular equation) と呼ぶ。
未知数が $\lambda$ 1 個の**スカラー方程式**になったのが嬉しいところ。

### 3.5 なぜ二分法で必ず解けるのか

$G = A^\top W A$ と置く。式 (3.1) が意味を持つには $G + \lambda D$ が正則で
なければならず、さらに解が最小点であるためには $G + \lambda D \succ 0$
(正定値) が要る。この範囲を調べる。

一般化固有値問題 $D v = \gamma\,G v$ を考える。 $G \succ 0$ とすると

$$
G + \lambda D \succ 0
\iff v^\top (G + \lambda D) v > 0 \quad (\forall v \ne 0)
$$

$v$ を一般化固有ベクトルに取れば $D v = \gamma G v$ より
$v^\top D v = \gamma\, v^\top G v$ なので

$$
v^\top G v + \lambda \gamma\, v^\top G v = (1 + \lambda\gamma)\, v^\top G v > 0
$$

$v^\top G v > 0$ だから、条件は

$$
1 + \lambda \gamma > 0 \quad (\text{全ての一般化固有値 } \gamma \text{ について})
$$

ここで $D = \mathrm{diag}(1,\dots,1,0)$ は**半正定値**なので $\gamma \ge 0$。
したがって $\gamma = 0$ の分は常に満たされ、条件を決めるのは
**最大固有値 $\gamma_1$** だけ。

$$
1 + \lambda\gamma_1 > 0 \iff \lambda > -\frac{1}{\gamma_1}
$$

この開区間

$$
I = \left(-\frac{1}{\gamma_1},\ \infty\right)
$$

の上で、次の 3 つが成り立つことが知られている。

- $G + \lambda D \succ 0$ ⟺ **2 階の最小条件が満たされる** (最大点や鞍点ではない)
- $\varphi(\lambda)$ は**狭義単調減少**
- $\lambda \to (-1/\gamma_1)^{+}$ のとき $\varphi(\lambda) \to +\infty$

単調減少で、左端で $+\infty$、右へ行けば負になる。したがって
**根はこの区間に唯一存在し、二分法で必ず捕まえられる**。
初期値の善し悪しも、反復の発散も無い。

### 3.6 実装

`beck_gtrs` の手順。

1. $\gamma_1 = \max \mathrm{eig}(G^{-1}D)$ を求め、 $\lambda_\text{lo} = -1/\gamma_1$ とする
2. $\lambda_\text{lo}$ のわずかに内側から始めて $\varphi > 0$ を確認する
3. $\varphi < 0$ になるまで上限を倍々に広げる (根を挟む区間を作る)
4. 200 回の二分法で根を詰める

行列は最大 $4\times4$ なので、これ全部で数十マイクロ秒。

> **C 版での工夫**: $G^{-1}D$ は非対称なので、一般の固有値計算が要るように見える。
> しかし $G = LL^\top$ (コレスキー分解) とすると $L^{-1} D L^{-\top}$ は**対称**で、
> $G^{-1}D$ と同じ固有値を持つ。対称行列なら Jacobi 法で簡単に解けるので、
> C 版はこの経路を使っている ([third_party/uwb_localizer/c/README.md](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/c/README.md))。

#### 3.6.1 本リポジトリの C 版: Schur 補元で 3×3 に落とす

上流の C 版は 4×4 の $G = LL^\top$ と $L^{-1}DL^{-\top}$ の Jacobi 固有分解を
使っていたが、本リポジトリ (`components/uwb_loc/src/uwb_closed_form.c`) は
$\alpha = \lVert p\rVert^2$ に当たる 4 番目の未知数を**先に消して** 3×3 で解く。

$y = [p;\ \alpha]$、 $G = \begin{bmatrix} G_{11} & g \\ g^\top & G_{33}\end{bmatrix}$
( $G_{11} = 4\sum w_i a_i a_i^\top$、 $g = -2\sum w_i a_i$、 $G_{33} = \sum w_i$ )、
$h = A^\top W b = [h_1;\ h_4]$ と分けると、式 (3.1) の 2 行目から
$\alpha = (h_4 + \lambda/2 - g^\top p)/G_{33}$。これを 1 行目に戻すと

$$
\bigl(\underbrace{G_{11} - g g^\top / G_{33}}_{S_c} + \lambda I\bigr)\,p
= h_1 - g\,(h_4 + \lambda/2)/G_{33}
$$

$S_c$ は Schur 補元で、 $S_c = 4\sum w_i (a_i - \bar a)(a_i - \bar a)^\top$
(重み付き重心 $\bar a$ まわりの散布行列の 4 倍)。さらに座標を $\bar a$ に
移す ( $d_i = a_i - \bar a$、 $q = p - \bar a$、 $b_i' = r_i^2 - \lVert d_i\rVert^2$ ) と
$g = 0$ になって式が最も簡単になる:

$$
(S_c + \lambda I)\,q = r_0,\quad r_0 = -2\sum w_i b_i' d_i,\qquad
\alpha(\lambda) = \bar b + \frac{\lambda}{2W},\qquad
\varphi(\lambda) = \lVert q(\lambda)\rVert^2 - \alpha(\lambda)
$$

( $W = \sum w_i$、 $\bar b = \sum w_i b_i'/W$。) 右辺 $r_0$ は $\lambda$ に
よらないので、 $\varphi'(\lambda) = -2\,q^\top (S_c+\lambda I)^{-1} q - 1/(2W) < 0$
が閉じた形で出る。目的関数 $\sum w_i(\lVert p - a_i\rVert^2 - r_i^2)^2$ は
平行移動不変なので、 $p = \bar a + q$ は元の座標で解いたものと同じ
( $\lambda$ の値だけが変わる)。 $G + \lambda D \succ 0 \iff S_c + \lambda I \succ 0$
だから、 $\lambda_\text{lo} = -\sigma_{\min}(S_c)$ で、 $\sigma_{\min}$ は 3×3 対称行列の
**閉形式固有値** (`uwb_sym3_eigvals`) から取る。固有ベクトルは使わない
(正方形や立方体の配置では縮退して不良条件になる)。ニュートン反復 1 回は
3×3 の $LDL^\top$ 分解 1 個 (除算 3 回、平方根なし) と代入 2 回で済み、
4×4 のコレスキー・Jacobi・ $L^{-1}$ は要らない。余因子逆行列 (除算 1 回) は
後退安定でなく、float では $\kappa = \sigma_{\max}/\sigma_{\min} \approx 200$ の
配置で解が $2\times10^{-4}$ m 狂った (4×4 の固有分解版は $3\times10^{-7}$ m)
ので使わない。 $LDL^\top$ なら同じ配置で $4\times10^{-6}$ m。

単精度で堅くなった点: 旧実装は $\lambda_\text{lo} + 10^{-9}|\lambda_\text{lo}|$ から
探索を始めていたが、 $10^{-9}$ は float の $\varepsilon$ ( $1.2\times10^{-7}$ ) より
小さいので `lo == lam_lo` になり、極 ( $1+\lambda\mu = 0$ ) を踏んで一歩も
動けずに失敗していた (FMA 縮約で偶然非ゼロになる環境でだけ通っていた)。
新実装は刻みを $64\,\varepsilon\,\sigma_{\max}$ と $\varepsilon$ に比例させ、極を
踏んだ (行列式が丸め誤差に埋もれた) ときは $\varphi = +\infty$ 側として
区間を詰める。収束判定の床も $\varepsilon$ 比例にしてある。

### 3.7 LLS との違い

| | LLS (§2) | Beck (§3) |
|---|---|---|
| $\lVert p\rVert^2$ | 差分で消す | 変数に昇格 |
| 基準アンカー | 要る (選び方で答えが変わる) | **要らない** |
| 行間の相関 | **入る** | 入らない |
| 2 乗バイアス | 残る | 残る |
| 最適性 | 保証なし | **大域最適** |

2 乗バイアスは Beck でも残るので**厳密な最尤推定ではない**。
しかし**大域最適であることが保証される**ので、Gauss-Newton ([§4](#4-lv1--重み付き非線形最小二乗-wnls))
の初期値として理想的。「初期値が悪くて局所解に落ちる」が原理的に起きない。

テスト `test_beck_is_global_and_beats_lls_with_noise` で、
ランダム 200 点の平均誤差が LLS より小さいことを確認している。

---

## 4. Lv1 — 重み付き非線形最小二乗 (WNLS)

実装: `uwb_loc/solvers/nls.py:solve_nls`

> **本リポジトリの `SolverLevel` には Lv1 が無い**（`components/uwb_ranging/include/uwb_ranging_types.hpp:40-42`
> の `enum class SolverLevel` は `Lv0` / `Lv2` / `Lv3` の 3 つのみで、`Lv1` を含まない）。
> Lv1（本節の重み付き非線形最小二乗）は Lv2（[§5](#5-lv2--ロバスト化)）の内側で使われる構成要素であり、
> 単体では公開していない。C 実装では `components/uwb_loc/src/uwb_nls.c` の `uwb_solve_lv1()` と
> `uwb_solve_lv2()` はどちらも共通の内部関数 `solve_iterative()`（ロバスト化の有無を切り替えるフラグ違い）を
> 呼ぶだけの薄いラッパになっており、`uwb_solve_lv1()` 自体は `uwb_loc.h` が宣言していて C API としては
> 呼べる。ただし本リポジトリのファームウェア（`uwb_ranging` 層の `PositioningPipeline::solve()`）は
> `SolverLevel` しか受け取らないため、Lv1 を単独では呼び出せない。**Lv2 は Lv1 の上に立っている**ので、
> この節は削除せずそのまま残す。`docs/PERF_ANALYSIS.md` には Lv1（`uwb_solve_lv1`）の性能実測値が残っている。
> `docs/GLOSSARY.md` の「Lv0 / Lv2 / Lv3」の項も参照。

### 4.1 何を最小化するのか

§2・§3 は「2 乗して線形にする」ことで近似解を得た。ここでは
**近似せず、元の非線形な式のまま**解く。

測距誤差が平均 0、分散 $\sigma_i^2$ の正規分布に従い、各観測が独立なら、
観測が得られる確率 (尤度) は

$$
L(p) = \prod_{i=1}^{n} \frac{1}{\sqrt{2\pi}\sigma_i}
\exp\left(-\frac{(z_i - h_i(p))^2}{2\sigma_i^2}\right)
$$

対数を取ると積が和になる。

$$
\log L(p) = \text{(定数)} - \frac{1}{2}\sum_{i=1}^{n} \frac{(z_i - h_i(p))^2}{\sigma_i^2}
$$

**尤度を最大にする** = **右辺の和を最小にする**。つまり

$$
\hat{p} = \arg\min_p\ S(p), \qquad
S(p) = \sum_{i=1}^{n} w_i \bigl(z_i - h_i(p)\bigr)^2, \quad w_i = \frac{1}{\sigma_i^2} \qquad (4.1)
$$

**重み付き最小二乗が最尤推定 (MLE) そのものになる。** これが
「重みを $1/\sigma^2$ にする」ことの理由で、恣意的な選択ではない。

### 4.2 Gauss-Newton 法 — 導出

$S(p)$ は $p$ について非線形なので、解析的には解けない。
そこで**現在の推定値のまわりで 1 次近似して、少しずつ動かす**。

残差ベクトルを $e(p) = z - h(p) \in \mathbb{R}^n$ と書く。
$p$ から $p + \Delta$ に動かしたときの残差をテイラー展開して 1 次で止める。

$$
h(p + \Delta) \approx h(p) + J\Delta, \qquad J_{ik} = \frac{\partial h_i}{\partial p_k}
$$

$$
\therefore\quad e(p + \Delta) = z - h(p+\Delta) \approx e(p) - J\Delta
$$

これを目的関数 (4.1) に代入する。 $S = e^\top W e$ と行列で書けるので

$$
S(p + \Delta) \approx (e - J\Delta)^\top W (e - J\Delta)
$$

展開する。 $W$ は対称なので $\Delta^\top J^\top W e = e^\top W J \Delta$ (スカラー同士) に注意して

$$
S(p+\Delta) \approx e^\top W e - 2\,\Delta^\top J^\top W e + \Delta^\top J^\top W J \Delta
$$

**これは $\Delta$ の 2 次関数**なので、微分して 0 と置けば最小点が求まる。

$$
\frac{\partial S}{\partial \Delta} = -2 J^\top W e + 2 J^\top W J \Delta = 0
$$

$$
\boxed{\ (J^\top W J)\,\Delta = J^\top W e, \qquad p \leftarrow p + \Delta\ } \qquad (4.2)
$$

これが**正規方程式**で、 $d \times d$ (3 次元なら $3\times3$) の連立 1 次方程式。
これを解いて $p$ を更新し、収束するまで繰り返す。

$J$ の第 $i$ 行は §1 で求めたヤコビアン ( $u_i^\top$ など)、
$W = \mathrm{diag}(w_1,\dots,w_n)$。

### 4.3 Levenberg-Marquardt 減衰 — なぜ要るか

式 (4.2) は $J^\top W J$ が正則であることを前提にしている。
しかし実際に**特異に近づく状況がある**。

- アンカーがほぼ一直線に並んでいる ( $u_i$ が全部似た方向を向く)
- タグがちょうどアンカー平面上にいる (3 次元で解いているとき)
- 見えているアンカーが $d+1$ 本ぎりぎり

このとき $(J^\top W J)^{-1}$ が巨大になり、 $\Delta$ が発散する。
そこで**対角に少し足して**無理やり正則にする。

$$
\left(J^\top W J + \lambda\,\frac{\mathrm{tr}(J^\top W J)}{d} I\right)\Delta = J^\top W e \qquad (4.3)
$$

$\mathrm{tr}(\cdot)/d$ で割っているのは**スケールを合わせる**ため
(単位系が変わっても $\lambda$ の意味が変わらないようにする)。

$\lambda$ の調整は次のとおり。

- コストが下がった → 更新を採用し、 $\lambda \leftarrow 0.3\lambda$ (Gauss-Newton 寄りに)
- コストが上がった → 更新を**破棄**し、 $\lambda \leftarrow 4\lambda$ (最急降下寄りに)

なぜこれで安定するのか。 $\lambda$ が大きいと式 (4.3) は
$\Delta \approx \frac{1}{\lambda \cdot \text{const}} J^\top W e$ となり、
これは**勾配方向への小さな一歩** (最急降下法) である。最急降下は遅いが
必ずコストを下げられるので、**解が退化していても止まることが保証される**。

### 4.4 共分散 — 「どれくらい信じてよいか」

推定値の不確かさも返したい。最尤推定の一般論から、最適点における
**フィッシャー情報行列**は $J^\top W J$ で与えられ、その逆行列が
推定量の共分散の近似になる。

$$
\mathrm{Cov}(\hat{p}) \approx (J^\top W J)^{-1} \qquad (4.4)
$$

(なぜそうなるかは [§8.1](#81-フィッシャー情報行列) で改めて見る。)

これを `Fix.covariance` として返し、その対角和の平方根
$\sigma = \sqrt{\mathrm{tr}(\mathrm{Cov})}$ を `Fix.sigma` としている。
**位置だけ返すライブラリは現場で困る** — 「今の値を信じてよいか」が
判断できないため。

### 4.5 次元拘束 (2 次元で解く場合)

$d=2$ のとき (高さが既知) は、 $J$ の $z$ 列を落として $2\times2$ の系にする。
$z$ は定数として扱うので、共分散も $2\times2$ 部分だけが埋まる。

同一平面配置で高さが決まらない場合の逃げ道としても使う
([§8.3](#83-同一平面の検出と鏡像解))。

---

## 5. Lv2 — ロバスト化

実装: `uwb_loc/solvers/robust.py`, `uwb_loc/solvers/nls.py`

この節で追加される記号:

| 記号 | 意味 |
|---|---|
| $t_i = e_i/\sigma_i$ | **標準化残差**。残差を標準偏差で割った無次元量 |
| $\rho(t)$ | 損失関数。「残差 $t$ にどれだけ罰を与えるか」 |
| $\psi(t) = \rho'(t)$ | 影響関数。「その観測が解をどれだけ引っぱるか」 |
| $\mathcal{W}(t)$ | 重み倍率 (IRLS で使う) |
| $k$ | ロバスト損失のしきい値 (標準化残差の単位) |

### 5.0 なぜロバスト化が要るのか

§4 の最小二乗は「誤差が正規分布」を前提にしていた。しかし屋内 UWB では
**NLOS** (見通し外) が起きる。人が間に立つ、壁で反射する — このとき
測距値は数十 cm から数 m 伸びる。

正規分布の裾は非常に薄い ( $4\sigma$ を超える確率は $6\times10^{-5}$ ) ので、
最小二乗は「そんな大きな残差はありえない」と考え、**その 1 本に合わせようとして
解を大きく引っぱられる**。損失が $e^2$ なので、残差が 10 倍なら罰は 100 倍。
外れ値 1 本が他の全部を圧倒する。

**屋内 UWB の精度を決めるのは測距ノイズではなく NLOS。** 順に効く手を重ねる。

### 5.1 物理ゲート (ほぼ無料)

位置を解く**前**にできるチェック。計算量がほぼゼロなのに効く。

- 負の距離、到達不能な距離 (`max_range` 超え)
- **三角不等式**: アンカー間の距離 $D_{ij} = \lVert a_i - a_j\rVert$ は既知なので、
  タグがどこにいても次が成り立たなければならない

$$
|r_i - r_j| \le D_{ij} \le r_i + r_j
$$

  (三角形の 2 辺の和は他の 1 辺より長く、差は短い。)
  これを誤差の余裕 (slack) 込みで破る組み合わせは**物理的にありえない**。

ただし「 $i$ と $j$ が矛盾する」と分かっても、**どちらが悪いかは決まらない**。
そこで各観測が他の何本と矛盾するかを数え、**過半と矛盾するものだけ**落とす。
1 本ずつの小競り合いで健全な観測を巻き添えにしないため。

### 5.2 M 推定 (Huber) と IRLS

**M 推定** (Maximum-likelihood-type estimation) は、二乗損失を
「外れ値に鈍い損失」に取り替える手法。最小化するのは

$$
S(p) = \sum_i \rho(t_i), \qquad t_i = \frac{e_i(p)}{\sigma_i}
$$

**フーバー損失** (Huber loss) は、小さい残差には二乗、大きい残差には
絶対値を使う。

$$
\rho(t) = \begin{cases}
\tfrac{1}{2}t^2 & |t| \le k \\
k|t| - \tfrac{1}{2}k^2 & |t| > k
\end{cases}
$$

( $|t| = k$ で値も傾きも連続になるように $-\tfrac{1}{2}k^2$ が付いている。)

**なぜこれで外れ値に強いのか。** 最小化の条件は $\sum_i \psi(t_i)\,\partial t_i/\partial p = 0$
で、 $\psi = \rho'$ は「その観測が解を引っぱる力」を表す (**影響関数**)。

$$
\psi(t) = \rho'(t) = \begin{cases}
t & |t| \le k \\
k \cdot \mathrm{sign}(t) & |t| > k
\end{cases}
$$

二乗損失なら $\psi(t) = t$ で**残差に比例して無限に強くなる**。
Huber では $|t| > k$ で $\psi$ が $\pm k$ に**頭打ち**になる。
「影響関数が有界」— これがロバストであることの定義そのもの。

**IRLS で最小二乗に帰着させる。** $\psi(t)$ を無理やり「重み × $t$」の形に書く。

$$
\psi(t) = \mathcal{W}(t)\cdot t
\quad\Longrightarrow\quad
\mathcal{W}(t) = \frac{\psi(t)}{t} = \begin{cases}
1 & |t| \le k \\
k/|t| & |t| > k
\end{cases}
$$

すると最小化の条件が

$$
\sum_i \mathcal{W}(t_i)\, t_i \,\frac{\partial t_i}{\partial p} = 0
$$

となり、これは**重み $\mathcal{W}(t_i)$ 付きの普通の最小二乗の条件と同じ形**。
だから「残差から重みを作り直しては §4 の最小二乗を解く」を繰り返せばよい。
これが **IRLS** (Iteratively Reweighted Least Squares, 反復再重み付き最小二乗)。

実装では反復のたびに

$$
w_i \leftarrow \frac{\mathcal{W}(t_i)}{\sigma_i^2}
$$

を作り直して §4 の Gauss-Newton に渡している。**ソルバ本体は変えていない。**

しきい値 $k = 1.345$ は、誤差が純粋な正規分布のときに
最小二乗の効率を 95% 保つ標準的な値 (残り 5% がロバスト性の対価)。

**Tukey の biweight** ( $\mathcal{W} = (1-(t/k)^2)^2$、 $|t|>k$ で 0) も用意して
あるが既定にしていない。 $\psi$ が**再下降する** ( $|t|$ が大きいと 0 に戻る) ので、
初期値が悪いと正しい観測まで完全に切って別の解に落ち着く。
実測でも Huber より悪かった ([§11](#11-実測での比較))。

### 5.3 片側損失 — NLOS の物理を重みに入れる

ここが**このライブラリで最も効いている工夫**。

§1.4 で見たとおり、NLOS は距離を**伸ばす側にしか出ない**。
$e = z - h$ の符号なら $e > 0$ 側。一方 $e < 0$ 側 (測距が短く出た) は
単なる測定ノイズで、疑う理由が無い。

そこで**しきい値を残差の符号で変える**。

$$
k_i = \begin{cases}
0.6\,k & e_i > 0 \quad (\text{NLOS の疑いあり — 厳しく見る}) \\
k & e_i \le 0 \quad (\text{単なる雑音 — 通常どおり})
\end{cases}
$$

「短すぎる側」の効率を落とさずに「長すぎる側」だけ抑えられる。
実測で RMSE 0.350 → 0.297 m ([§11](#11-実測での比較))。
**Huber を入れるより片側にする方が効いた。**

### 5.4 χ² ゲート

収束後の標準化残差 $t_i = e_i/\sigma_i$ は、モデルが正しければおおむね
標準正規分布 $N(0,1)$ に従う。

$|t_i| > 4$ となる確率は約 $6\times10^{-5}$ なので、これを超える観測は
「モデルが間違っている = 外れ値」とみなして落とし、**1 度だけ**解き直す。

残る本数が $d+1$ を下回るなら実行しない (解けなくなるため)。
しきい値は Lv1 で 3.5、Lv2 で 4.0。Lv2 の方が緩いのは、
ロバスト損失が先に効いているのでゲートで切りすぎない方が良いため。

### 5.5 RANSAC — 常時ではなく保険として

**RANSAC** (RANdom SAmple Consensus) は、最小構成 ( $d+1$ 本) から仮の解を作り、
その解と整合する観測 (インライア) が最多になる組を採る手法。

**ここが実装で最も試行錯誤した箇所。** 当初の設計どおり常時走らせたら、
ロバスト化しない Lv1 より**悪化した** (RMSE 0.297 → 0.401 m)。

理由は明快で、3 次元の最小構成は 4 本、**そこから作る仮解自体が数十 cm の
誤差を持つ**。その誤差だらけの仮解を基準に $3\sigma$ でインライア判定すると、
健全な観測が大量に「外れ値」に見えてしまう。

対策は 3 つ。

1. **起動条件をつける** — 標準化残差の RMS $\sqrt{\overline{t^2}}$ が 3 を超えたときだけ走らせる
   (モデルが正しければ 1 前後になるはずの量)
2. **しきい値を広げる** ( $3\sigma \to 4\sigma$ ) — 仮解自身の誤差を見込む
3. **改善したときだけ採用する** — RANSAC 後の標準化残差が元より小さくならなければ元の解を使う

結果として、まともな条件では RANSAC は**一度も起動しない**。それでよい。
これは精度を上げる部品ではなく、M 推定が引きずられるほど壊れた状況
(NLOS が過半を占める、アンカーが 1 台動かされた) のための**保険**である。

> C 版には RANSAC が入っていない。組込みには重く、上のとおり通常は
> 起動しないため ([third_party/uwb_localizer/c/README.md](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/c/README.md))。

### 5.6 温め直さない (warm start しない)

Lv2 は前回の推定値を初期値に使わない。ロバスト重みは残差から作るので、
前回値に引きずられると**外れ値に固着**しうる (間違った位置では、
正しい観測の方が「外れ値」に見える)。

毎回 Beck の閉形式 (§3) から解き直しても、それが**大域最適解**なので
収束性は落ちない。§3 の「初期値依存が無い」性質がここで効いている。

---

## 6. Lv3 — 密結合 EKF

実装: `uwb_loc/solvers/ekf.py`

この節で追加される記号:

| 記号 | 意味 |
|---|---|
| $x$ | **状態ベクトル**。位置だけでなく速度なども含む |
| $n_x$ | 状態の次元 |
| $P$ | 状態の**共分散行列**。「今どれくらい自信が無いか」 |
| $F$ | 状態遷移行列。「 $\Delta t$ 秒後に状態はどう変わるか」 |
| $Q$ | プロセスノイズ共分散。「運動モデルの当てにならなさ」 |
| $K$ | **カルマンゲイン**。「観測をどれだけ信じて state を動かすか」 |
| $\nu = z - h(\hat{x})$ | **イノベーション**。予測と観測のずれ |
| $S$ | イノベーションの分散 (この実装ではスカラー) |
| $\sigma_a$ | 加速度の白色雑音の強さ (CV のとき) |
| $\Delta t$ | 前回の更新からの経過時間 |

### 6.0 カルマンフィルタとは何か (最短の説明)

§2〜§5 は、**1 エポックの観測だけ**から位置を求めていた
(スナップショット測位)。過去の情報は捨てている。

タグが動いている場合、これはもったいない。1 秒前に $(4, 3)$ にいて
毎秒 1 m で動いていたなら、今は $(5, 3)$ の近くにいるはずで、
**その予想も情報**である。

カルマンフィルタは「**予測**」と「**更新**」を交互に行う。

1. **予測** — 運動モデルで「今どこにいるはず」を計算する。
   同時に「その予想はどれくらい当てにならないか」 ( $P$ ) も増やす
2. **更新** — 観測が来たら、予測と観測を**それぞれの不確かさで重み付けして**混ぜる。
   $P$ は減る

観測モデルが非線形 (距離は $\lVert p - a\rVert$ なので非線形) の場合、
線形化して使うものを **EKF** (Extended Kalman Filter, 拡張カルマンフィルタ) と呼ぶ。
線形化に使うのは §1 で求めたヤコビアンそのもの。

### 6.1 なぜ「密結合」か

2 つの作り方がある。

- **疎結合** (loosely coupled): まず §5 で位置を出し、**その位置**をフィルタに入れる
- **密結合** (tightly coupled): **測距値そのもの**を観測としてフィルタに入れる ← こちら

密結合の利点は 3 つ。

**(1) アンカーが 3 本未満でも更新できる。**
2 本では三辺測量は不可能 (位置が決まらない) だが、各測距は依然として
**1 次元の拘束**である。「アンカー A から 3.2 m」という情報は、
その方向の共分散を縮める。残りの方向は運動モデルからの予測が埋める。
疎結合ならこのエポックは丸ごと捨てになる。

**(2) 観測が非同期でよい。**
TWR はアンカーを順にポーリングするので、測距は**もともとバラバラに届く**。
密結合なら「1 スキャン = 1 エポック」に束ねる必要がない。

**(3) 幾何が悪い方向の情報だけを部分的に取り込める。**

テスト `test_ekf_tracks_with_fewer_than_three_anchors` がこれを検証している。

### 6.2 状態と予測

状態は**軸ごとの積分器の連鎖**にする。

$$
\text{CV (等速)}: x = \begin{bmatrix} p \\ v \end{bmatrix}, \qquad
\text{CA (等加速度)}: x = \begin{bmatrix} p \\ v \\ a \end{bmatrix}
$$

**状態遷移行列を導く。** 1 軸だけ、CV の場合を考える。等速なら

$$
p(t + \Delta t) = p(t) + v(t)\,\Delta t, \qquad v(t+\Delta t) = v(t)
$$

行列で書くと

$$
\begin{bmatrix} p \\ v\end{bmatrix}_{t+\Delta t}
= \begin{bmatrix} 1 & \Delta t \\ 0 & 1\end{bmatrix}
\begin{bmatrix} p \\ v\end{bmatrix}_t
$$

CA なら $p(t+\Delta t) = p + v\Delta t + \tfrac{1}{2}a\Delta t^2$ が加わる。
一般に、 $k$ 段の積分器の連鎖はテイラー展開そのものなので

$$
F_1[i][j] = \frac{\Delta t^{\,j-i}}{(j-i)!} \quad (j \ge i), \qquad F_1[i][j] = 0 \quad (j < i)
$$

( $j-i = 0$ なら 1、 $=1$ なら $\Delta t$、 $=2$ なら $\Delta t^2/2$。)

これは 1 軸分。 $d$ 軸に広げるにはクロネッカー積を使う。

$$
F = F_1 \otimes I_d
$$

**軸は独立と仮定**している ( $x$ 方向の運動が $y$ に影響しない)。

**プロセスノイズ $Q$ を導く。** 運動モデルは完璧ではない。
「等速」といっても実際には加速する。そこで
**最上位の微分に連続時間の白色雑音が乗る**と考える
(CV なら加速度が、CA なら加加速度がランダムに揺れる)。

CV の場合、雑音は加速度に入るので $G = [0, 1]^\top$。時刻 $\tau$ に入った雑音が
$\Delta t$ 後の状態に与える影響は $F(\Delta t - \tau) G$ で、これを積分すると

$$
Q_1 = \int_0^{\Delta t} F(\tau)\,G\,\sigma_a^2\,G^\top F(\tau)^\top\,d\tau
$$

中身を計算する。

$$
F(\tau) G = \begin{bmatrix} 1 & \tau \\ 0 & 1\end{bmatrix}\begin{bmatrix} 0 \\ 1\end{bmatrix} = \begin{bmatrix} \tau \\ 1\end{bmatrix}
$$

なので

$$
F(\tau)G\,G^\top F(\tau)^\top = \begin{bmatrix} \tau \\ 1\end{bmatrix}\begin{bmatrix} \tau & 1\end{bmatrix}
= \begin{bmatrix} \tau^2 & \tau \\ \tau & 1\end{bmatrix}
$$

各成分を $0$ から $\Delta t$ まで積分すると

$$
\int_0^{\Delta t}\tau^2 d\tau = \frac{\Delta t^3}{3},\qquad
\int_0^{\Delta t}\tau\, d\tau = \frac{\Delta t^2}{2},\qquad
\int_0^{\Delta t} d\tau = \Delta t
$$

$$
\therefore\quad
Q_1^{\text{CV}} = \sigma_a^2\begin{bmatrix} \Delta t^3/3 & \Delta t^2/2 \\ \Delta t^2/2 & \Delta t\end{bmatrix}
$$

CA も同様に計算して

$$
Q_1^{\text{CA}} = \sigma_j^2\begin{bmatrix}
\Delta t^5/20 & \Delta t^4/8 & \Delta t^3/6 \\
\Delta t^4/8 & \Delta t^3/3 & \Delta t^2/2 \\
\Delta t^3/6 & \Delta t^2/2 & \Delta t\end{bmatrix}
$$

軸への展開は $F$ と同じく $Q = Q_1 \otimes I_d$。

予測ステップはこれで

$$
\hat{x} \leftarrow F\hat{x}, \qquad P \leftarrow FPF^\top + Q
$$

> **注意**: 離散版 (1 ステップ中は加速度が一定と見なす $\Gamma\Gamma^\top$ 形、
> $\Delta t^4/4$ が出るもの) と**混ぜないこと**。どちらも「正しい」モデルだが、
> `sigma_a` の意味が変わってしまう。このライブラリは上の連続時間版で統一している。

### 6.3 更新 — 1 本ずつスカラーで

測距の観測は**位置にしか依存しない**ので、状態に対するヤコビアンは

$$
H = \begin{bmatrix} \partial h/\partial p & 0 & \cdots & 0 \end{bmatrix} \in \mathbb{R}^{1 \times n_x}
$$

(速度や加速度の成分は 0。) 左端の $\partial h/\partial p$ は §1 の $u_i^\top$ そのもの。

標準的なカルマンフィルタの更新式は

$$
\nu = z - h(\hat{x}) \qquad (\text{イノベーション})
$$

$$
S = HPH^\top + \sigma^2 \qquad (\text{イノベーションの分散})
$$

$$
K = PH^\top S^{-1} \qquad (\text{カルマンゲイン})
$$

$$
\hat{x} \leftarrow \hat{x} + K\nu
$$

**ここで重要なのは、観測を 1 本ずつ処理していること。** すると $H$ が
$1 \times n_x$ の**行ベクトル**になり、 $S$ が**スカラー**になる。

$$
K = \frac{PH^\top}{S}
$$

**行列の逆行列が一切要らない** (スカラーの割り算だけ)。
これは C への移植で大きく効く ([§12](#12-c-への移植))。

**共分散は Joseph 形式で更新する。**

$$
P \leftarrow (I - KH)P(I - KH)^\top + K\sigma^2K^\top
$$

最適な $K$ に対しては、よく見る $P \leftarrow (I-KH)P$ と代数的に等価。
しかし Joseph 形式は**右辺が明らかに対称かつ半正定値**の形をしているので、
丸め誤差が乗っても $P$ の性質が壊れにくい。スカラー更新を 1 エポックに
何本も繰り返すこの実装では、この性質が要る。
さらに毎回 $P \leftarrow (P + P^\top)/2$ で対称化する。

#### 6.3.1 本リポジトリの C 版: 対称 rank-1 ダウンデート

観測 1 本 ( $H = h^\top$ ) のとき $u = Ph$、 $S = h^\top u + \sigma^2$、 $K = u/S$ と
置いて Joseph 形式を展開すると

$$
P - K u^\top - u K^\top + K(h^\top P h)K^\top + \sigma^2 K K^\top
= P - K u^\top - u K^\top + S\,K K^\top
= P - \frac{u u^\top}{S}
$$

( $h^\top P h + \sigma^2 = S$ と $S K K^\top = u K^\top$ を使った。) つまり
**$P$ が対称なら Joseph 形式は $P \leftarrow P - u u^\top/S$ に厳密に潰れる**。
本リポジトリの C 版 (`uwb_ekf.c`) はこれを `uwb_symn_rank1_downdate` で
上三角だけ計算して下三角へ鏡映するので、結果は構成的に対称になり、
後段の $(P+P^\top)/2$ も不要。 $n_x \times n_x$ の 4 項の積和が 1 項になり、
除算は $1/S$ の 1 回、ゲート $|\nu| > \gamma\sqrt{S}$ は $\nu^2 > \gamma^2 S$ で
平方根も取らない。Joseph 形式の利点 (対称・半正定値が保たれる) は
rank-1 の形でも同じ: $P - uu^\top/S = P^{1/2}(I - vv^\top/(v^\top v + \sigma^2))P^{1/2}$
( $v = P^{1/2}h$ ) で、括弧の中の固有値は $[\sigma^2/S,\,1]$ にある。

**副次効果**: 1 本ずつ順に処理すると、2 本目は**1 本目で更新済みの位置**で
線形化される。反復 EKF に近い効果がただで手に入る。

### 6.4 イノベーションゲートと「棺桶化」対策

$|\nu| > \gamma\sqrt{S}$ ( $\gamma$ は既定 3) の観測は、そのエポックで使わない。
**これが NLOS 対策の本体。**

$S$ が「予測の不確かさ $HPH^\top$ + 観測の不確かさ $\sigma^2$」を正しく含むので、
**フィルタが自信を持っているときほど厳しく**判定される。
逆に立ち上げ直後で $P$ が大きいときは緩い。この自動調整が嬉しいところ。

ただし**既知の失敗モード**がある。何かの拍子に状態が間違った位置に行くと、
**正しい観測がすべて外れ値に見えて全部弾かれ、二度と直らない**。
フィルタが自分の誤りを守り続ける状態で、「棺桶問題」と呼ばれる。

対策として、全観測を弾いたエポックを数え、`max_rejects` 回連続したら
**状態を捨てて Lv2 のスナップショット測位からやり直す**。
テスト `test_ekf_recovers_after_teleport` で検証している。

そのほかの実装上の手当て。

- **立ち上げ** (bootstrap) は Lv2 の解と共分散を使う。速度は不明なので大きな事前分散から始める。
  ただし**測距 1 本では位置が決まらない** (球面 1 枚) ので、 $d+2$ 本たまるまで待つ
- 観測が `max_dt` 以上途切れたら予測が意味を失うので再初期化
- 時刻が巻き戻った観測 (遅れて届いたもの) は捨てる

---

## 7. TDoA の Chan 法

実装: `uwb_loc/solvers/closed_form.py:chan_tdoa`

> **本リポジトリでは未実装**（[§1.2](#12-距離差-tdoa) の注記のとおり、TDoA 自体を使わない構成のため）。
> `components/uwb_loc/` にも対応する C 関数は無い。背景知識として原典のまま残す。

TDoA (§1.2) の閉形式解。GPS 由来の手法で、**基準アンカーまでの距離
$d_\text{ref}$ を補助未知数に加える**と線形になる、という発想。

観測は距離の差 $r_i = d_i - d_\text{ref}$ なので、

$$
d_i = d_\text{ref} + r_i
$$

これを式 (2.1) の形 ( $K_i = \lVert a_i\rVert^2$ と略記) に代入する。

$$
\lVert p\rVert^2 - 2a_i^\top p + K_i = d_i^2 = (d_\text{ref} + r_i)^2
= d_\text{ref}^2 + 2 d_\text{ref} r_i + r_i^2
$$

基準アンカーについては $r_\text{ref} = 0$ なので

$$
\lVert p\rVert^2 - 2a_\text{ref}^\top p + K_\text{ref} = d_\text{ref}^2
$$

**引き算する。** 左辺の $\lVert p\rVert^2$ と右辺の $d_\text{ref}^2$ が**両方消える**。

$$
-2a_i^\top p + 2a_\text{ref}^\top p + K_i - K_\text{ref} = 2 d_\text{ref} r_i + r_i^2
$$

整理して

$$
\boxed{\ 2(a_i - a_\text{ref})^\top p + 2r_i\,d_\text{ref} = K_i - K_\text{ref} - r_i^2\ }
$$

未知数 $[p;\ d_\text{ref}] \in \mathbb{R}^{d+1}$ について**線形**。
$n-1 \ge d+1$ 本あれば WLS (重み付き最小二乗) で解ける。

Chan 法の 2 段目 ( $d_\text{ref} = \lVert p - a_\text{ref}\rVert$ という
拘束を使った補正) は入れていない。初期値として使う分には 1 段目で十分で、
最終精度は後段の WNLS (§4) が担保するため。

---

## 8. GDOP と CRLB — 配置を数字で評価する

実装: `uwb_loc/geometry.py`

### 8.1 フィッシャー情報行列

「観測から位置についてどれだけ情報が得られるか」を表す量が
**フィッシャー情報行列** $\mathcal{F}$ である。定義は対数尤度の 2 階微分の
期待値 (の符号反転)。

§4.1 の対数尤度

$$
\log L(p) = \text{const} - \frac{1}{2}\sum_i \frac{(z_i - h_i(p))^2}{\sigma_i^2}
$$

を $p$ で 2 回微分する。1 階微分は

$$
\frac{\partial \log L}{\partial p} = \sum_i \frac{(z_i - h_i(p))}{\sigma_i^2}\,\frac{\partial h_i}{\partial p}
$$

もう 1 回微分すると、 $h$ の 2 階微分の項と、1 階微分の積の項が出る。
前者には $(z_i - h_i)$ が掛かっており、真値では期待値が 0 になるので消える。残るのは

$$
\mathcal{F} = -\mathbb{E}\left[\frac{\partial^2 \log L}{\partial p\,\partial p^\top}\right]
= \sum_i \frac{1}{\sigma_i^2}\,\frac{\partial h_i}{\partial p}\frac{\partial h_i}{\partial p}^\top
= \sum_i \frac{1}{\sigma_i^2}\,u_i u_i^\top
$$

(最後は距離観測の場合。) 行列で書けば $\mathcal{F} = J^\top W J$ で、
§4.4 で共分散に使った量そのもの。

**クラメール・ラオ下限** (CRLB) は、任意の不偏推定量について

$$
\mathrm{Cov}(\hat{p}) \succeq \mathcal{F}^{-1}
$$

が成り立つ、という定理。`crlb_at` は $\sqrt{\mathrm{tr}(\mathcal{F}^{-1})}$ を返す。

**どんなアルゴリズムを使ってもこれより良くはならない**ので、
実測 RMSE と並べれば「アルゴリズムの改善余地」と「配置の限界」を切り分けられる。

ただしこれは**1 エポック分の観測で解く場合の下限**。Lv3 は時間方向の情報も
使うので下回りうる (実測で CRLB 0.178 m に対し Lv3 は 0.116 m)。

### 8.2 GDOP

CRLB は $\sigma_i$ に依存する。「測距の精度は同じとして、**置き方だけ**で
どれだけ損をするか」を見たいことがある。そこで全ての $\sigma_i$ を 1 と置く
(= $H$ の行を単位ベクトルに正規化する) と

$$
\text{GDOP} = \sqrt{\mathrm{tr}\bigl((H^\top H)^{-1}\bigr)}
$$

**GDOP** (Geometric Dilution Of Precision, 幾何精度劣化係数)。読み方は

$$
\sigma_p \approx \text{GDOP} \times \sigma_\text{range}
$$

つまり「測距誤差が何倍に増幅されて位置誤差になるか」。目安は
2 以下なら良い、2〜5 は使える、5 超なら配置を見直す。

実装では $H^\top H$ の階数を先に確認し、退化していれば $\infty$ を返す
(2 本しか見えないときなど、逆行列が数値的に暴れてトレースが負になることがある)。

### 8.3 同一平面の検出と鏡像解

アンカー座標を中心化して特異値分解し、第 3 特異値 $/\sqrt{n}$ を
「平面からの広がり」とみなす。これが最大特異値の 5% 未満なら**同一平面**と判定する。

> **本リポジトリの C 版** (`uwb_model.c:uwb_anchors_coplanar`) は特異値分解の
> 代わりに、中心化した散布行列 $C = \sum (a_i-\bar a)(a_i-\bar a)^\top$ (3×3 対称)
> の固有値を閉形式 (`uwb_sym3_eigvals`) で取り、
> $\lambda_{\min}/\lambda_{\max} < 0.05^2$ で判定する (特異値比 5% と同じ。
> 平方根は取らない)。法線は最小固有値の固有ベクトル (`uwb_sym3_min_eigvec`)。
> 判定はアンカーの (enabled, 座標) だけで決まるので、`uwb_config_init` /
> `uwb_config_refresh` で 1 回計算して `uwb_config.plane` に持ち、毎 fix の
> 鏡像処理 (`uwb_resolve_mirror`) では作成時に写した座標と今の配列を
> 比べて一致すればそれを使う。呼び出し側がアンカーを書き換えたあと
> refresh を忘れても、不一致を検出して計算し直すので結果は変わらない。

**3 次元測位では、アンカーが同一平面に並んでいないことが本質的に効く。**
理由は 2 つある。

**(1) 高さが観測しにくい。** 天井 4 隅だけの配置では $z$ の情報がほとんど入らず、
しかもその誤差が水平方向にも回り込む。実測で RMSE(z) 0.92 m、
`dim=2` で高さを固定すると水平 RMSE まで 0.374 → 0.271 m に改善した。

**(2) 鏡像解が生じる。** これは (1) より厄介で、精度の劣化ではなく
**問題そのものの多義性**である。

アンカーが平面 $n^\top x = c$ 上にあるとする ( $n$ は単位法線)。
任意の点 $p$ に対して、その平面に関する**鏡像**を

$$
p' = p - 2\,(n^\top p - c)\,n
$$

と定義する。このとき**すべてのアンカーからの距離が厳密に等しい**。

$$
\lVert p' - a_i\rVert = \lVert p - a_i\rVert \quad (\forall i)
$$

**証明。** $s = n^\top p - c$ (平面からの符号付き距離) と置くと $p' = p - 2sn$。
アンカー $a_i$ は平面上にあるので $n^\top a_i = c$、したがって
$n^\top(p - a_i) = n^\top p - c = s$。よって

$$
\lVert p' - a_i\rVert^2 = \lVert (p - a_i) - 2sn \rVert^2
= \lVert p-a_i\rVert^2 - 4s\,n^\top(p-a_i) + 4s^2\lVert n\rVert^2
$$

$n^\top(p-a_i) = s$ かつ $\lVert n\rVert = 1$ だから

$$
= \lVert p-a_i\rVert^2 - 4s^2 + 4s^2 = \lVert p-a_i\rVert^2 \qquad \blacksquare
$$

残差はどちらも完全に同じなので、**測距値だけからはどちらか選べない**。
どんなアルゴリズムを使っても解決しない。

実装 (`solvers/base.py:resolve_mirror`) は、距離以外の情報を強い順に使う。

1. **`SolveConfig(z_bounds=...)`** — 「タグは天井より下」のような事前知識。
   片方だけが範囲に入るならそれを採る。**これが本来の解決策**
2. **直前に採った側** — 平面のどちら側にいたかという**上下の二択だけ**を
   引き継ぐ。位置の初期値としては使わないので、Lv2 が前回値から
   温め直さない方針 (§5.6) と衝突しない
3. **どちらも無い** — 決めようがないので `Fix.ambiguous = True` を立てて返す

鏡映は直交変換なので、位置を移すときは共分散も $RCR^\top$
( $R = I - 2nn^\top$ ) で移す。残差 RMS と GDOP は鏡映で不変。
**移した先で解き直してはいけない** — 退化した幾何では最適点が平面をまたいで
戻ってしまい、せっかく寄せた側から逃げる。

これを入れる前は、Lv2 が毎エポック閉形式から解き直すために側が反転し、
300 エポック中 24 回天井の上に飛んで RMSE 0.89 m だった (同条件の Lv1 は
warm start が偶然側を保つので 0.28 m で、レベルの順序が逆転していた)。
側を固定した後は反転がゼロになる。

ただし **2 の手当ては「安定させる」だけで「正しくする」ものではない**。
最初のエポックで反対側を掴むと、その track はまるごと鏡像側に張り付く
(水平は正しく、高さだけ折り返る)。実測でも 3 seed 中 1 回それが起きた。
だから同一平面 + 3D + `z_bounds` 無しの構成では**構築時に警告を出す**。
黙って 1/2 の確率で高さが折り返るのがいちばん危ないため。

---

## 9. アンカーの自己測量

実装: `uwb_loc/calibration.py:self_survey`

アンカー同士で相互測距できるなら、巻き尺は要らない。
**距離の表から座標を復元する**手法が **MDS** (MultiDimensional Scaling, 多次元尺度構成法)。

この節で追加される記号:

| 記号 | 意味 |
|---|---|
| $x_i$ | アンカー $i$ の座標 (これを求める) |
| $X$ | 座標を縦に並べた行列 ( $n \times d$ ) |
| $D$ | 距離行列。 $D_{ij} = \lVert x_i - x_j\rVert$ |
| $B$ | グラム行列 (内積の表) |
| $J = I - \frac{1}{n}\mathbf{1}\mathbf{1}^\top$ | 中心化行列 |

### 9.1 古典的 MDS (Torgerson) — なぜ距離から座標が出るのか

距離が分かっていて座標を知りたい。まず**内積が分かれば座標が出る**ことを見る。

内積の表 (グラム行列) を $B_{ij} = x_i^\top x_j$ とすると、
座標を並べた行列 $X$ について $B = XX^\top$。 $B$ は対称半正定値なので
固有分解でき、

$$
B = U\Lambda U^\top \quad\Longrightarrow\quad X = U_d \Lambda_d^{1/2}
$$

( $\Lambda_d$ は大きい方から $d$ 個の固有値。) つまり**内積さえ分かれば座標が出る**。

問題は、我々が持っているのは内積ではなく**距離**であること。両者の関係は

$$
D_{ij}^2 = \lVert x_i - x_j\rVert^2 = \lVert x_i\rVert^2 - 2x_i^\top x_j + \lVert x_j\rVert^2
= B_{ii} - 2B_{ij} + B_{jj} \qquad (9.1)
$$

$B_{ij}$ について解くと

$$
B_{ij} = -\tfrac{1}{2}\left(D_{ij}^2 - B_{ii} - B_{jj}\right)
$$

しかし $B_{ii} = \lVert x_i\rVert^2$ は原点の取り方に依存する未知量。
**そこで「重心を原点に置く」と決めてしまう。** すると $\sum_i x_i = 0$ で、
式 (9.1) を $i$ について平均すると

$$
\frac{1}{n}\sum_i D_{ij}^2 = \frac{1}{n}\sum_i B_{ii} - \frac{2}{n}\underbrace{\sum_i B_{ij}}_{= (\sum_i x_i)^\top x_j = 0} + B_{jj}
= \overline{B} + B_{jj}
$$

( $\overline{B} = \frac{1}{n}\sum_i B_{ii}$ と置いた。) 同様に $j$ について平均、
両方について平均すると $B_{ii}, B_{jj}$ が全部書ける。代入して整理すると、
結局こう書ける。

$$
\boxed{\ B = -\tfrac{1}{2}\,J\,D^{\circ 2}\,J, \qquad J = I - \tfrac{1}{n}\mathbf{1}\mathbf{1}^\top\ }
$$

( $D^{\circ 2}$ は成分ごとの 2 乗、 $J$ は**中心化行列** — 左から掛けると
各列から列平均を引き、右から掛けると各行から行平均を引く。)
この操作を**二重中心化**と呼ぶ。式 (9.1) の
「 $\lVert x_i\rVert^2$ の項」と「 $\lVert x_j\rVert^2$ の項」が
行方向・列方向の平均を引くことでちょうど消え、内積の項だけが残る、というのが
この式の意味。

あとは $B$ を固有分解すれば座標が出る。

### 9.2 欠測リンクの埋め方 — 実装で見つかった落とし穴

現実には、遮蔽で測れないアンカー間の距離がある。ここに**平均値を入れると壊れる**。

MDS は**大域的なスペクトル法**なので、1 箇所の大きな誤りが埋め込み全体を歪める。
しかも後段の Gauss-Newton は欠測ペアを残差に含めないので、
**それを直す力が働かない**。

対策として、既知リンクを辿った**最短経路長** (Floyd–Warshall 法) で埋める。
「A-B が測れなくても、A-C と C-B が測れていれば A-B ≈ AC + CB」という近似。
凸に近い配置なら良い近似になり、MDS を正しい盆地に入れられる。

### 9.3 Gauss-Newton による仕上げ

MDS の解は 2 乗距離を使うので §2.2 と同じバイアスを持つ。
仕上げに §4 と同じ Gauss-Newton をかける。

$$
\min_X \sum_{(i,j)\in\text{既知}} w_{ij}\bigl(\lVert x_i - x_j\rVert - D_{ij}\bigr)^2
$$

ペア $(i,j)$ のヤコビアンは、§1.1 の結果をそのまま使って

$$
\frac{\partial}{\partial x_i}\lVert x_i - x_j\rVert = u_{ij}, \qquad
\frac{\partial}{\partial x_j}\lVert x_i - x_j\rVert = -u_{ij}
$$

( $u_{ij}$ は $x_j$ から $x_i$ への単位ベクトル。)

座標系の自由度 (3 次元で並進 3 + 回転 3 = 6) の分だけ $J^\top J$ は
**必ず特異になる** (座標系全体を動かしても距離は変わらないため)。
LM 減衰 (§4.3) がそれを吸収する。

### 9.4 座標系の固定 (ゲージ)

相互距離からは回転・並進・鏡像が決まらない。そこで点そのものから基底を作る。

1. 点 0 を原点
2. 点 1 の方向を第 1 軸
3. 第 1 軸への直交成分が**最大**の点で第 2 軸
4. 平面から十分離れた**最初の**点が $z>0$ 側になるよう鏡像を固定

3 で「最初の点」ではなく「最大の点」を使うのは、直交成分がほぼ 0 の点に
当たったとき雑音で向きが決まってしまうため。逆に 4 で「最大」ではなく
「最初」なのは、箱型のような対称配置では複数の点の $|z|$ が並び、
雑音で選ばれ方が変わって**鏡像が反転する**ため。
どちらも実装中に実際に踏んだ罠で、テストで固定してある。

なお第 2 軸を「適当な固定ベクトル」から作ってはいけない。第 1 軸まわりの
回転が残り、同じ配置でも入力座標系によって別の答えになる。

### 9.5 実座標への位置合わせ (Kabsch 法)

自己測量の座標系は任意なので、実測した点に重ねる。
2 組の点群 $X_\text{src}, X_\text{dst}$ (どちらも重心を原点に移したもの) に対し、

$$
\min_R\ \lVert X_\text{src}R - X_\text{dst}\rVert_F^2 \quad \text{s.t.}\ R^\top R = I
$$

を解く。展開すると

$$
\lVert X_\text{src}R\rVert_F^2 - 2\,\mathrm{tr}(R^\top X_\text{src}^\top X_\text{dst}) + \lVert X_\text{dst}\rVert_F^2
$$

第 1 項は $R$ が直交なので $R$ に依らない。したがって
**$\mathrm{tr}(R^\top M)$ を最大化**すればよい ( $M = X_\text{src}^\top X_\text{dst}$ )。
$M = U\Sigma V^\top$ と特異値分解すると、この最大化の解は

$$
R = VU^\top
$$

`align_to_reference` がこれ。ただし**鏡像の扱いに注意**が要る:
3 次元では**同一平面に乗らない 4 点以上**を基準にしないと、
裏返った解も同じだけよく合ってしまう (3 点は必ず同一平面上にある)。
点数不足や同一平面の基準点には警告を出す。

全台を巻き尺で測る必要はなく、高さの違う 4 台を実測すれば残りは自己測量に
任せられる (実測: 測距誤差 5 cm で座標誤差 平均 3.7 cm)。

---

## 10. アンテナ遅延の推定

実装: `uwb_loc/calibration.py:estimate_antenna_delays`

測距値には、アンテナと回路の遅延による**一定のオフセット**が乗る。

$$
r = d + \delta_\text{anchor} + \delta_\text{tag}
$$

既知距離での測定を集めれば線形最小二乗で分離できる…が、
**定数分の不定性がある**。全アンカーの遅延を $+c$ し、タグの遅延を $-c$ しても

$$
(\delta_\text{anchor} + c) + (\delta_\text{tag} - c) = \delta_\text{anchor} + \delta_\text{tag}
$$

で測距値は変わらない。つまり**この 2 つは原理的に分離できない**。

そこで拘束を 1 本足す。**アンカー遅延の平均を 0 に固定**する。

$$
\sum_i \delta_{\text{anchor},i} = 0
$$

残りはタグ側 ( $\delta_\text{tag}$ ) に寄る。個々の値はこの拘束の取り方で変わるが、
**和 $\delta_\text{anchor} + \delta_\text{tag}$ は一意に決まる**ので、
補正するときは両方を足して使えばよい。

未補正のアンテナ遅延は数十 cm の系統誤差になる。距離に比例する成分まで含めた
1 次モデル $d_\text{true} \approx \alpha\,r_\text{meas} + \beta$ は
`fit_range_bias` で当てられる。

---

## 11. 実測での比較

同一の観測列 (8 台立体配置、 $\sigma_0 = 8$ cm、10 Hz、8 の字軌道、5 seed 平均)。

| NLOS 率 | Lv0 | Lv1 | Lv2 | Lv3 |
|---|---|---|---|---|
| 0 % | 0.193 | 0.177 | 0.177 | **0.116** |
| 15 % | 0.607 | 0.376 | 0.311 | **0.172** |
| 35 % | 1.049 | 0.799 | 0.637 | **0.328** |

(RMSE 3D [m])

見通しが良ければ Lv1 と Lv2 に差はない。**ロバスト化が効くのは NLOS があるとき
だけ**で、しかも NLOS 率が上がるほど差が開く。

Lv2 の内訳 (NLOS 15%、4 seed 平均、RMSE 3D [m])。

| 構成 | RMSE | CEP95 |
|---|---|---|
| Lv1 (ロバスト化なし) | 0.356 | 0.251 |
| + Huber (両側) | 0.350 | 0.199 |
| + Huber + 片側損失 = **Lv2 既定** | **0.297** | **0.186** |
| 参考: 無条件に $3\sigma$ で RANSAC 事前間引き | 0.401 | 0.227 |
| 参考: Huber を Tukey に変更 | 0.474 | 0.337 |

効いているのは Huber ではなく**片側損失**の方 (0.350 → 0.297)。
NLOS が正側にしか出ないという物理を入れて初めて差が出る、と読める。

最後から 2 行目が §5.5 で述べた失敗で、これが常時 RANSAC をやめた理由。
条件付きにした現在の実装は、この条件 (NLOS 15%、8 台) では**一度も起動しない** —
標準化残差が 3 を超えないため。それが意図した挙動で、
RANSAC は本当に壊れたときのためだけに置いてある。

---

## 12. C への移植

C 版の詳細は [third_party/uwb_localizer/c/README.md](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/c/README.md)。ここでは数値計算の観点だけ。

必要になる線形代数は次の 3 つだけ。

| 用途 | 大きさ | 備考 |
|---|---|---|
| 正規方程式 $(J^\top WJ)\Delta = J^\top We$ (§4.2) | 3×3 | LU 分解で十分 |
| Beck 法の $\lambda$ 下限 (§3.5) | 4×4 | コレスキー + 対称 Jacobi 固有値。§3.6 の注を参照 |
| EKF の共分散 (§6.3) | 6×6 | 逐次スカラー更新なので**逆行列は不要**、スカラー除算だけ |

移植の順序は

1. `model.py` の残差とヤコビアン (§1) をそのまま写す
2. `solve_nls` の Gauss-Newton ループ (§4、LM 減衰込み)
3. `Lv3TightlyCoupledEKF` の predict / update (§6)

**動作の照合は、同じ観測列を Python 版と C 版に食わせて `Fix` を突き合わせる**のが
いちばん早い。上流の `third_party/uwb_localizer/tools/crossval.py`（**本リポジトリには Python ツールは無い**）が
これを自動でやっており、
5 シナリオ × Lv0〜Lv3 の 20 通りで**位置が 1e-11 m 以内で一致**することを
確認している。
