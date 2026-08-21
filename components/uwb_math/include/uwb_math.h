/* uwb_math — 小次元の線形代数をスカラー展開した部品集。
 *
 * ESP32-S3 は単精度 FPU しか持たず (除算は近似命令列、sqrt は無い)、
 * double はすべてソフト演算になる。そこで測位 (uwb_loc) と測量 (uwb_survey)
 * が使う行列計算を「固定小次元は完全展開、可変次元は 3x3 カーネルで
 * ブロック化」した形でここにまとめ、一般の LU / Jacobi をホットパスから
 * 追い出す。BLAS も LAPACK も malloc も使わない。
 *
 * 方針
 *   - 2x2 / 3x3 の対称行列はループ無しのスカラー式。除算は原則 1 回、
 *     sqrt は必要な箇所だけ。
 *   - 可変次元 (EKF の nx ∈ {4,6,9}、測量の 3x3 ブロック nb ≤ 8) は
 *     n を実行時に受け取り、内側は 3x3 展開カーネルで回す。
 *   - 一般 LU・一般 Jacobi は**置かない**。参照実装はテスト側にだけ持つ。
 *   - 各関数のコメントに「除算 / sqrt の回数」と失敗条件を書く。
 *
 * 格納形式
 *   vec3  : uwb_real[3]
 *   sym3  : 対称 3x3 を 6 要素でパック  [xx, xy, xz, yy, yz, zz]
 *             S00=s[0] S01=s[1] S02=s[2] S11=s[3] S12=s[4] S22=s[5]
 *   sym2  : 対称 2x2 を 3 要素でパック  [xx, xy, yy]
 *   m3    : 一般 3x3、行優先 uwb_real[9]  (M[i][j] = m[3*i+j])
 *   symn  : 対称 n×n、行優先 (フル格納。EKF の P と同じ)
 *   bchol : 3x3 ブロックの下三角パック + 縁取り 1 行 (uwb_bchol 構造体)
 *
 * 実数型 uwb_real は uwb_loc.h と共有する。どちらを先に include しても
 * 二重定義にならないよう UWB_REAL_DEFINED で守る (uwb_loc.h 側にも同じ
 * ガードを付ける前提)。-DUWB_USE_FLOAT で float、それ以外は double。
 */
#ifndef UWB_MATH_H
#define UWB_MATH_H

#include <float.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------ 実数型 */

#ifndef UWB_REAL_DEFINED
#define UWB_REAL_DEFINED 1
#ifdef UWB_USE_FLOAT
typedef float uwb_real;
#else
typedef double uwb_real;
#endif
#endif /* UWB_REAL_DEFINED */

#ifndef UWB_REAL_IS_FLOAT
#ifdef UWB_USE_FLOAT
#define UWB_REAL_IS_FLOAT 1
#else
#define UWB_REAL_IS_FLOAT 0
#endif
#endif

/** uwb_real の計算機イプシロン (1 と区別できる最小の相対差)。 */
#if UWB_REAL_IS_FLOAT
#define UWB_MATH_EPS ((uwb_real)FLT_EPSILON)
#else
#define UWB_MATH_EPS ((uwb_real)DBL_EPSILON)
#endif

/** 「長さ 0 とみなす」絶対しきい値 [m 等]。uwb_loc の UWB_EPS と同じ値。
 *  float でも正規数 (最小正規数 1.2e-38) なので二乗しても潰れない。 */
#define UWB_MATH_TINY ((uwb_real)1e-12)

/** 3x3 の行列式を「丸め誤差と区別できない」とみなす相対しきい値。
 *  余因子展開の det の丸め誤差は max|s_ij|^3 の数 eps 倍なので、その
 *  64 倍を境にする (double 1.4e-14、float 7.6e-6)。 */
#define UWB_MATH_SING_TOL ((uwb_real)64 * UWB_MATH_EPS)

/** 固有値を「0 とみなす」/ 固有値が「縮退している」とみなす相対しきい値。
 *  uwb_survey の UWB_SURVEY_RANK_TOL と同じ 1e4·eps (double 2.2e-12、
 *  float 1.2e-3)。**必ず eps に比例させる**: 定数 1e-9 を決め打ちすると
 *  単精度でランク落ちを検出できない。 */
#define UWB_MATH_RANK_TOL ((uwb_real)1e4 * UWB_MATH_EPS)

/** 対称 rank-1 更新が想定する最大次数 (EKF の状態数 3x3=9)。関数自体は
 *  ループなので制限は無いが、設計上の上限として明示しておく。 */
#define UWB_MATH_SYMN_MAX 9

/* ------------------------------------------------------ スカラー小道具 */

/** sqrt。float ビルドでは sqrtf を使い、double への昇格を避ける。 */
static inline uwb_real uwb_math_sqrt(uwb_real v)
{
#if UWB_REAL_IS_FLOAT
    return sqrtf(v);
#else
    return sqrt(v);
#endif
}

/** 絶対値。NaN はそのまま返る。 */
static inline uwb_real uwb_math_abs(uwb_real v) { return v < (uwb_real)0 ? -v : v; }

/** lo 以上 hi 以下に収める。NaN は NaN のまま (比較が偽になるため)。 */
static inline uwb_real uwb_math_clamp(uwb_real v, uwb_real lo, uwb_real hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/** a と b の大きい方。 */
static inline uwb_real uwb_math_max(uwb_real a, uwb_real b) { return a > b ? a : b; }

/** NaN なら 1。 */
static inline int uwb_math_isnan(uwb_real v) { return v != v; }

/* ------------------------------------------------------------- vec3 */
/* 3 要素の小さな演算は呼び出しコストの方が大きいので static inline で置く。
 * 除算・sqrt を含むものは除算 / sqrt 回数をコメントに書いてある。 */

/** a·b。乗算 3。 */
static inline uwb_real uwb_v3_dot(const uwb_real *a, const uwb_real *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** ‖a‖²。乗算 3。 */
static inline uwb_real uwb_v3_norm2(const uwb_real *a) { return uwb_v3_dot(a, a); }

/** ‖a‖。sqrt 1。 */
static inline uwb_real uwb_v3_norm(const uwb_real *a) { return uwb_math_sqrt(uwb_v3_norm2(a)); }

/** out = a × b。out が a や b と同じ配列でもよい (一時変数経由)。乗算 6。 */
static inline void uwb_v3_cross(const uwb_real *a, const uwb_real *b, uwb_real *out)
{
    uwb_real x = a[1] * b[2] - a[2] * b[1];
    uwb_real y = a[2] * b[0] - a[0] * b[2];
    uwb_real z = a[0] * b[1] - a[1] * b[0];
    out[0] = x; out[1] = y; out[2] = z;
}

/** out = a − b。 */
static inline void uwb_v3_sub(const uwb_real *a, const uwb_real *b, uwb_real *out)
{
    out[0] = a[0] - b[0]; out[1] = a[1] - b[1]; out[2] = a[2] - b[2];
}

/** out = a + b。 */
static inline void uwb_v3_add(const uwb_real *a, const uwb_real *b, uwb_real *out)
{
    out[0] = a[0] + b[0]; out[1] = a[1] + b[1]; out[2] = a[2] + b[2];
}

/** y += a·x。乗算 3。 */
static inline void uwb_v3_axpy(uwb_real a, const uwb_real *x, uwb_real *y)
{
    y[0] += a * x[0]; y[1] += a * x[1]; y[2] += a * x[2];
}

/** v *= a。乗算 3。 */
static inline void uwb_v3_scale(uwb_real a, uwb_real *v)
{
    v[0] *= a; v[1] *= a; v[2] *= a;
}

/** out = a。 */
static inline void uwb_v3_copy(const uwb_real *a, uwb_real *out)
{
    out[0] = a[0]; out[1] = a[1]; out[2] = a[2];
}

/** v = 0。 */
static inline void uwb_v3_zero(uwb_real *v) { v[0] = v[1] = v[2] = (uwb_real)0; }

/** v を単位ベクトルにして元の長さを返す。
 *  ‖v‖ ≤ UWB_MATH_TINY (または NaN) なら v を零ベクトルにして 0 を返す
 *  ので、呼び出し側は `if (!(len > 0))` で退化を検出できる。
 *  sqrt 1、除算 1 (逆数を 1 回作って 3 回掛ける)。 */
uwb_real uwb_v3_normalize(uwb_real *v);

/** 単位ベクトル u に直交する正規直交基底 (e1, e2) を作る。e1 × e2 = u
 *  (右手系) になる。e1 は「u と最も揃っていない座標軸」から Gram-Schmidt で
 *  作るので桁落ちしない。u が単位ベクトルでない/零なら 0 を返す。
 *  sqrt 1、除算 1。 */
int uwb_v3_perp_basis(const uwb_real *u, uwb_real *e1, uwb_real *e2);

/* ------------------------------------------------------------- sym3 */

/** s = 0。 */
void uwb_sym3_zero(uwb_real *s);

/** s += u uᵀ。乗算 6。 */
void uwb_sym3_add_outer(uwb_real *s, const uwb_real *u);

/** s += w·u uᵀ。乗算 9 (w·u を 3 回、外積 6 回)。 */
void uwb_sym3_add_scaled_outer(uwb_real *s, uwb_real w, const uwb_real *u);

/** 対角に c を足す (s += c·I)。 */
void uwb_sym3_add_diag(uwb_real *s, uwb_real c);

/** 対角を (1+λ) 倍して ridge を足す (LM の減衰: s_ii ← s_ii(1+λ) + ridge)。 */
void uwb_sym3_damp_diag(uwb_real *s, uwb_real lambda, uwb_real ridge);

/** tr(S)。 */
uwb_real uwb_sym3_trace(const uwb_real *s);

/** det(S) を余因子展開で。乗算 11、除算 0。 */
uwb_real uwb_sym3_det(const uwb_real *s);

/** y = S x。乗算 9。y は x と別配列であること。 */
void uwb_sym3_mv(const uwb_real *s, const uwb_real *x, uwb_real *y);

/** xᵀ S x。乗算 9。 */
uwb_real uwb_sym3_quad(const uwb_real *s, const uwb_real *x);

/** パック → 行優先 9 要素 (上下三角を鏡映して埋める)。 */
void uwb_sym3_to_full(const uwb_real *s, uwb_real *m);

/** 行優先 9 要素 → パック (上三角 m[0],m[1],m[2],m[4],m[5],m[8] を読む)。 */
void uwb_sym3_from_full(const uwb_real *m, uwb_real *s);

/** S x = b を余因子で解く。
 *  失敗条件: |det| ≤ UWB_MATH_SING_TOL · max|s_ij|³ (行列式が丸め誤差と
 *  区別できない)、または NaN。失敗時は x に触らず 0 を返す。
 *  b と x は同じ配列でもよい。乗算 ≈ 30、除算 1 (1/det)。
 *  相対精度は κ(S)·eps 程度: float では κ ≳ 1e4 で 3 桁落ちる。 */
int uwb_sym3_solve(const uwb_real *s, const uwb_real *b, uwb_real *x);

/** (S + shift·I) x = b。s は書き換えない。失敗条件・コストは solve と同じ。 */
int uwb_sym3_solve_shifted(const uwb_real *s, uwb_real shift, const uwb_real *b, uwb_real *x);

/** S⁻¹ をパックで返す (inv と s は別配列)。失敗条件は solve と同じ。
 *  乗算 ≈ 25、除算 1。 */
int uwb_sym3_inverse(const uwb_real *s, uwb_real *inv);

/** (S + shift·I)⁻¹。失敗条件・コストは inverse と同じ。 */
int uwb_sym3_inverse_shifted(const uwb_real *s, uwb_real shift, uwb_real *inv);

/** tr(S⁻¹) = (C00 + C11 + C22) / det (C は余因子)。GDOP / CRLB 用で、
 *  逆行列を作らず対角余因子 3 つだけ計算する。失敗条件は solve と同じ。
 *  乗算 ≈ 17、除算 1。sqrt は取らない (呼び出し側で sqrt(tr))。 */
int uwb_sym3_trace_inverse(const uwb_real *s, uwb_real *tr);

/** 固有値を閉形式で (降順 lam[0] ≥ lam[1] ≥ lam[2])。
 *    q = tr/3, p = sqrt(‖S − qI‖²_F / 6), B = (S − qI)/p,
 *    φ = acos(clamp(det(B)/2, −1, 1)) / 3,
 *    λ0 = q + 2p cos φ,  λ2 = q − p (cos φ + √3 sin φ),  λ1 = 3q − λ0 − λ2
 *  p ≤ 4·eps·|q| なら三重縮退とみなして λ = (q, q, q)。
 *  |det(B)/2| > 1 − 1e-4 (2 つの固有値が近接) のときは acos の傾きが立って
 *  近接対の分離が √eps·‖S‖ までぼやけるので、孤立固有値の固有ベクトルの
 *  直交補空間に射影した 2x2 で対を解き直す (deflation)。
 *  結果の絶対誤差は eps·‖S‖ 程度: 最小固有値が ‖S‖ に比べて小さいとき
 *  **相対**精度は出ない (ランク判定の比には十分)。NaN を含めば 0 を返す。
 *  通常経路: sqrt 2 (p, sin φ)、acos 1、cos 1、除算 1 (1/p)。
 *  deflation 経路はさらに sqrt 3、除算 3 (固有ベクトル 1 本 + 直交基底)。 */
int uwb_sym3_eigvals(const uwb_real *s, uwb_real *lam);

/** lam に最も近い固有値に対応する単位固有ベクトル。
 *  内部では uwb_sym3_eig と同じ手順 (孤立固有値は (S − λI) の行の外積
 *  3 通りのうち最大ノルム、残りの対は直交補空間の 2x2) で 3 本まとめて
 *  求め、該当する 1 本を返す。外積法を近接した固有値に直接使うと、誤差
 *  eps·‖S‖/gap が縮退面の**外**にも漏れて ‖Sv − λv‖ が悪化するため。
 *  失敗条件 (0 を返す): その固有値が他の固有値と UWB_MATH_RANK_TOL·‖S‖
 *  以内で縮退している (向きが決まらない)、三重縮退、NaN。
 *  S のスケール (1e-6〜1e6) には依存しない。符号は「絶対値最大の成分が正」。
 *  コストは uwb_sym3_eig と同じ。 */
int uwb_sym3_eigvec(const uwb_real *s, uwb_real lam, uwb_real *v);

/** 最小固有値の固有ベクトル (平面の法線)。lam には 3 固有値 (降順) を
 *  書く (不要なら NULL)。失敗条件: λ1 − λ2 ≤ UWB_MATH_RANK_TOL·‖S‖
 *  (法線が決まらない)、三重縮退、NaN。コストは uwb_sym3_eig と同じ。 */
int uwb_sym3_min_eigvec(const uwb_real *s, uwb_real *lam, uwb_real *v);

/** 固有値 (降順) と固有ベクトル行列 V (行優先 9 要素、**列** k が lam[k]
 *  の固有ベクトル: V[3*i+k])。uwb_survey_sym_eig(…,3) と同じ規約。
 *  手順: 閉形式の固有値 → 最も孤立した固有値のベクトルを行外積で →
 *  その直交補空間 (uwb_v3_perp_basis) に射影した 2x2 で残りの対 →
 *  固有値を Rayleigh 商と 2x2 閉形式で磨き直す。S V = V Λ と VᵀV = I は
 *  どちらも eps·‖S‖ の精度で成り立つ (double 1e-15、float 4e-7 程度)。
 *  縮退があっても失敗しない: 間隔が 16·eps·‖S‖ 以下の固有値は同じ固有
 *  空間とみなし、その中は任意の正規直交基底で埋める (三重縮退なら V = I)。
 *  V は直交行列だが det は ±1 のどちらもありうる (符号は「絶対値最大の
 *  成分が正」に揃えるため)。NaN を含めば 0。
 *  sqrt 6、acos 1、cos 1、除算 5 (eigvals が deflation 経路なら sqrt 9、除算 8)。 */
int uwb_sym3_eig(const uwb_real *s, uwb_real *lam, uwb_real *vec);

/** 固有値列 (降順、半正定値を想定) からランクを数える。
 *  λ_k > max(rel_tol·λ_0, abs_tol) を満たす k の数を返す (0..3)。
 *  負の固有値は 0 扱い。例: 同一平面判定は rel_tol = 0.05² (特異値比 0.05
 *  = 固有値比 0.0025)、測量の縮退判定は 1e-3。 */
int uwb_sym3_rank(const uwb_real *lam, uwb_real rel_tol, uwb_real abs_tol);

/** S ← R S R、R = I − 2 n nᵀ (単位法線 n による鏡映)。
 *    R S R = S − 2 n (Sn)ᵀ − 2 (Sn) nᵀ + 4 (nᵀSn) n nᵀ
 *  共分散を鏡映面の反対側へ移すときに使う。乗算 ≈ 30、除算 0。 */
void uwb_sym3_reflect(uwb_real *s, const uwb_real *n);

/** 余因子行列 adj(S) (対称なので転置不要 = 随伴行列) をパックで adj に書き、
 *  det(S) を返す。S⁻¹ = adj/det。det の大きさで特異判定を自前でしたいとき、
 *  rank 2 の S から値域の射影 (adj ∝ n nᵀ、n はヌルベクトル) を取りたいとき用。
 *  adj と s は別配列。乗算 15、除算 0。 */
uwb_real uwb_sym3_adjugate(const uwb_real *s, uwb_real *adj);

/** uwb_sym3_solve の特異判定しきい値を引数で渡せる版。
 *  失敗条件: |det| ≤ rel_tol · max|s_ij|³、または det が NaN / ±inf。
 *  rel_tol = UWB_MATH_SING_TOL で uwb_sym3_solve と同じ。rel_tol = 0 は
 *  「det が厳密に 0 のときだけ失敗」(部分ピボット LU と同じ意味論。退化した
 *  幾何でも解を返してほしいとき用。ほぼ特異なら解は κ·eps の精度しかなく、
 *  巨大な値になりうる)。b と x は同じ配列でもよい。乗算 ≈ 30、除算 1。 */
int uwb_sym3_solve_tol(const uwb_real *s, const uwb_real *b, uwb_real *x, uwb_real rel_tol);

/** rank 2 の対称 3x3 のヌルベクトル (単位。S n = 0)。3 行のうち 2 行の外積
 *  3 通りのうち最大ノルムのものを採る (2 行が平行だと 0 になるため)。行は
 *  max|s_ij| で正規化してから外積を取るので、S のスケール (1e-6〜1e6) に
 *  依存しない。符号は「絶対値最大の成分が正」。
 *  失敗条件 (0 を返し n に触らない): 外積がすべて 0 (rank ≤ 1)、NaN。
 *  rank 3 の S では「2 行に直交する向き」でしかなくヌルベクトルではないので、
 *  rank は uwb_sym3_eigvals + uwb_sym3_rank で先に確かめること (rank 2 と
 *  判定できる範囲なら n の誤差は eps·λ0/λ1 程度)。
 *  最小固有値の固有ベクトル (uwb_sym3_min_eigvec) と同じ向きだが、固有値を
 *  求めず sqrt 1、除算 2 (1/max, 1/‖n‖)、乗算 ≈ 30 で済む。 */
int uwb_sym3_null_vector(const uwb_real *s, uwb_real *n);

/** rank 1 近似 S ≈ λ v vᵀ の主方向 v (単位)。S の行はすべて v に比例する
 *  ので、最大ノルムの行を正規化する。rank 2 以上の S では「最大ノルム行の向き」
 *  であって固有ベクトルではない。符号は「絶対値最大の成分が正」。
 *  失敗条件 (0 を返し v に触らない): 行ノルム ≤ UWB_MATH_TINY (S ≈ 0)、NaN。
 *  sqrt 1、除算 1。 */
int uwb_sym3_principal_axis(const uwb_real *s, uwb_real *v);

/** 球面拘束付き最小二乗   min  uᵀ M u − 2 bᵀ u   s.t.  ‖u‖ = 1   の厳密解
 *  (M は正定値、lam_min = λ_min(M) > 0 を呼び出し側が uwb_sym3_eigvals で
 *  渡す)。Lagrange 条件は (M + νI) u = b で、大域最小は M + νI ≻ 0 すなわち
 *  ν > −λ_min の側にある (信頼領域部分問題と同じ構造)。
 *      φ(ν) = 1/‖u(ν)‖ − 1,   u(ν) = (M + νI)⁻¹ b
 *  の根を Newton 法で求める (Moré–Sorensen。1/‖u‖ は ν についてほぼ線形で
 *  凹・単調増加なので、左側から始めれば単調に収束し、右側から始めても
 *  1 歩で左側に移る)。
 *      φ'(ν) = uᵀ w / ‖u‖³,   w = (M + νI)⁻¹ u
 *      Δν = −φ/φ' = ‖u‖² (‖u‖ − 1) / (uᵀ w)
 *  ν = 0 から始める。無拘束解 ‖M⁻¹b‖ がすでに 1 なら 1 回目で収束する。
 *  極 (ν ≤ −λ_min) へ飛びそうな Newton 歩は二分 (ν ← (ν + (−λ_min))/2) で抑える。
 *  「hard case」(b が λ_min の固有ベクトルに直交し、ν → −λ_min でも ‖u‖ が
 *  1 に届かない) は反復上限 (30)、または二分が極に達して LDLᵀ が正定値で
 *  なくなった時点 (float では ν が丸めで −λ_min ちょうどになる) で打ち切り、
 *  最後に解けた u を正規化して返す (厳密な最小ではない。高さが形状と矛盾した
 *  異常データでしか起きない)。
 *
 *  Beck の λ 探索 (uwb_loc/uwb_closed_form.c の uwb_beck_gtrs) と同型:
 *  あちらは (Sc + λI) q = r0 の割線方程式 φ(λ) = ‖q‖² − (b̄ + λ/2W) を
 *  λ > −σ_min(Sc) で解く。線形部分 (LDLᵀ で shift 付き solve を 2 回 =
 *  u と w) は同じで、φ の形 (‖q‖² の 2 次形か 1/‖u‖ の凹形か) と安全策
 *  (Beck は区間を括って rtsafe、こちらは極の手前で二分) だけが違う。
 *  将来 Beck をこれに寄せるなら φ に線形項を足した版を同じ骨格で書ける。
 *
 *  失敗条件 (0 を返し u に触らない): ν = 0 で LDLᵀ が正定値でない (M が
 *  正定値でない、NaN)、b = 0、最後の u が正規化できない。
 *  成功時 out_nu (NULL 可) に u を解いた ν を書く (hard case では打ち切り時点の値)。
 *  1 反復: LDLᵀ 1 回 (除算 3) + solve 2 回 + sqrt 1 + 除算 1。通常 1〜3 反復。 */
int uwb_sym3_solve_sphere(const uwb_real *m, const uwb_real *b, uwb_real lam_min,
                          uwb_real *u, uwb_real *out_nu);

/* ------------------------------------------------------------- sym2 */

/** s = 0。 */
void uwb_sym2_zero(uwb_real *s);

/** s += u uᵀ。乗算 3。 */
void uwb_sym2_add_outer(uwb_real *s, const uwb_real *u);

/** s += w·u uᵀ。乗算 5。 */
void uwb_sym2_add_scaled_outer(uwb_real *s, uwb_real w, const uwb_real *u);

/** 対角に c を足す。 */
void uwb_sym2_add_diag(uwb_real *s, uwb_real c);

/** 対角を (1+λ) 倍して ridge を足す。 */
void uwb_sym2_damp_diag(uwb_real *s, uwb_real lambda, uwb_real ridge);

/** tr(S)。 */
uwb_real uwb_sym2_trace(const uwb_real *s);

/** det(S)。乗算 2。 */
uwb_real uwb_sym2_det(const uwb_real *s);

/** S x = b。失敗条件: |det| ≤ UWB_MATH_SING_TOL·max|s_ij|²、または NaN。
 *  b と x は同じ配列でもよい。乗算 6、除算 1。 */
int uwb_sym2_solve(const uwb_real *s, const uwb_real *b, uwb_real *x);

/** S⁻¹ (パック)。失敗条件は solve と同じ。乗算 3、除算 1。 */
int uwb_sym2_inverse(const uwb_real *s, uwb_real *inv);

/** tr(S⁻¹) = (s00 + s11)/det。失敗条件は solve と同じ。除算 1。 */
int uwb_sym2_trace_inverse(const uwb_real *s, uwb_real *tr);

/** 固有値 (降順)。λ = m ± sqrt(d² + b²)、m = (a+c)/2、d = (a−c)/2。
 *  sqrt 1、除算 0。NaN なら 0。 */
int uwb_sym2_eigvals(const uwb_real *s, uwb_real *lam);

/** 固有値 lam の単位固有ベクトル。(S − λI) の 2 行のうち長い方に直交する
 *  ベクトルを取る。S − λI が丸め誤差の範囲 (4·eps·‖S‖) で 0 (二重固有値)
 *  なら 0 を返す。それ以上の間隔なら誤差は向きに eps·‖S‖/gap だけ乗り、
 *  残差 ‖Sv − λv‖ は eps·‖S‖ で収まる。符号は「絶対値最大の成分が正」。
 *  sqrt 1、除算 1。 */
int uwb_sym2_eigvec(const uwb_real *s, uwb_real lam, uwb_real *v);

/** y = S x。乗算 4。y は x と別配列であること。 */
void uwb_sym2_mv(const uwb_real *s, const uwb_real *x, uwb_real *y);

/** (S + shift·I) x = b。s は書き換えない。失敗条件・コストは solve と同じ。 */
int uwb_sym2_solve_shifted(const uwb_real *s, uwb_real shift, const uwb_real *b, uwb_real *x);

/** (S + shift·I)⁻¹。失敗条件・コストは inverse と同じ。 */
int uwb_sym2_inverse_shifted(const uwb_real *s, uwb_real shift, uwb_real *inv);

/** uwb_sym2_solve の特異判定しきい値を引数で渡せる版 (sym3_solve_tol の 2x2)。
 *  失敗条件: |det| ≤ rel_tol · max|s_ij|²、または det が NaN / ±inf。 */
int uwb_sym2_solve_tol(const uwb_real *s, const uwb_real *b, uwb_real *x, uwb_real rel_tol);

/* ------------------------------------------ sym3 / sym2 の LDLᵀ 分解 */
/* 正規方程式・Beck の (Sc + λI)・LLS の AᵀA・球面拘束の (M + νI) を解くための、
 * ピボット無し LDLᵀ (パック対称入力。sqrt 無し、除算は対角の逆数 3 回
 * (2x2 は 2 回) だけ)。
 *
 * なぜ余因子 (uwb_sym3_solve、除算 1) と別に持つか:
 * 余因子展開の行列式は誤差 eps·σ_max³ を持ち、条件数 κ = σ_max/σ_min が
 * 大きいと解の相対誤差が eps·κ·(σ_max/σ_2) まで膨らむ (後退安定でない)。
 * float では κ ≈ 200 の Beck で 2e-4 m、κ ≈ 1e5 の 4 台 LLS で数 m 狂った。
 * LDLᵀ は正定値行列に対して後退安定で、誤差は eps·κ に収まる
 * (同じ条件で 4e-6 m)。GDOP / CRLB のように精度要求が低く条件も良い
 * (単位ベクトルの和) 所だけ余因子 (uwb_sym3_trace_inverse) を使う。
 *
 * 失敗条件 (factor が 0 を返す。f の中身は不定):
 *   require_pd = 1: ピボット d_k が正でない (正定値でない。Beck の極、LLS の
 *                   階数落ち)。NaN も弾く。+inf は通る (呼び出し側で x の
 *                   NaN を見ること)。
 *   require_pd = 0: ピボット d_k が 0 / NaN / ±inf のときだけ失敗。不定値行列
 *                   でも通す (部分ピボット LU と同じ「厳密に特異なときだけ
 *                   失敗」の意味論。NLS の共分散で退化した幾何でも ok=1 と
 *                   巨大な cov を返すため)。
 * ピボット無しなので、正定値でない行列では (require_pd = 0 でも) 途中の
 * 小さなピボットで誤差が膨らみうる。また s[0] = 0 の非特異行列
 * (例: [[0,1],[1,0]]) は d_0 = 0 で失敗する (正規方程式 AᵀA では起きない)。 */

/* factor / solve は static inline で置く: Beck の φ 評価 (1 fix に ≈ 11 回、
 * 各 factor 1 + solve 2) と NLS の Gauss-Newton 反復ごとに呼ばれ、関数呼び出し
 * にすると Beck +18%、Lv1/Lv2 +10% (M3 Ultra、clang -O2、tools/bench_loc) と
 * 演算量に対して呼び出しコストが見えるため (vec3 と同じ扱い)。inverse は
 * 共分散 1 回ぶんなので通常の関数。 */

typedef struct {
    uwb_real l10, l20, l21;          /**< 単位下三角 L の非対角 */
    uwb_real inv_d0, inv_d1, inv_d2; /**< D の逆数 */
} uwb_sym3_ldl;

/** ピボットの合否。require_pd = 0 は 0 / NaN / ±inf だけを弾く。 */
static inline int uwb_ldl_pivot_ok(uwb_real d, int require_pd)
{
    if (require_pd) return d > (uwb_real)0;
    return d != (uwb_real)0 && d == d && d - d == (uwb_real)0;
}

/** (S + shift·I) = L D Lᵀ。s はパック [xx,xy,xz,yy,yz,zz]。除算 3、乗算 8。 */
static inline int uwb_sym3_ldl_factor(const uwb_real *s, uwb_real shift, int require_pd,
                                      uwb_sym3_ldl *f)
{
    uwb_real d0 = s[0] + shift, d1, d2, t;
    if (!uwb_ldl_pivot_ok(d0, require_pd)) return 0;
    f->inv_d0 = (uwb_real)1 / d0;
    f->l10 = s[1] * f->inv_d0;
    f->l20 = s[2] * f->inv_d0;
    d1 = (s[3] + shift) - f->l10 * s[1];             /* a11 − l10² d0 */
    if (!uwb_ldl_pivot_ok(d1, require_pd)) return 0;
    f->inv_d1 = (uwb_real)1 / d1;
    t = s[4] - f->l20 * s[1];                         /* a21 − l20 l10 d0 */
    f->l21 = t * f->inv_d1;
    d2 = (s[5] + shift) - f->l20 * s[2] - f->l21 * t; /* a22 − l20² d0 − l21² d1 */
    if (!uwb_ldl_pivot_ok(d2, require_pd)) return 0;
    f->inv_d2 = (uwb_real)1 / d2;
    return 1;
}

/** L D Lᵀ x = b (前進代入 → D⁻¹ → 後退代入)。b と x は同じ配列でもよい。
 *  除算 0、乗算 9。 */
static inline void uwb_sym3_ldl_solve(const uwb_sym3_ldl *f, const uwb_real *b, uwb_real *x)
{
    uwb_real y0 = b[0];
    uwb_real y1 = b[1] - f->l10 * y0;
    uwb_real y2 = b[2] - f->l20 * y0 - f->l21 * y1;
    uwb_real x2 = y2 * f->inv_d2;
    uwb_real x1 = y1 * f->inv_d1 - f->l21 * x2;
    uwb_real x0 = y0 * f->inv_d0 - f->l10 * x1 - f->l20 * x2;
    x[0] = x0; x[1] = x1; x[2] = x2;
}

/** (S + shift·I)⁻¹ をパックで (factor 1 回 + 単位ベクトル 3 本を解く)。
 *  因子を作り終えてから書くので inv と s は同じ配列でもよい。失敗条件は
 *  factor と同じ (失敗時 inv に触らない)。除算 3、乗算 ≈ 35。 */
int uwb_sym3_ldl_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv);

typedef struct {
    uwb_real l10;            /**< 単位下三角 L の非対角 */
    uwb_real inv_d0, inv_d1; /**< D の逆数 */
} uwb_sym2_ldl;

/** (S + shift·I) = L D Lᵀ。s はパック [xx,xy,yy]。除算 2、乗算 2。 */
static inline int uwb_sym2_ldl_factor(const uwb_real *s, uwb_real shift, int require_pd,
                                      uwb_sym2_ldl *f)
{
    uwb_real d0 = s[0] + shift, d1;
    if (!uwb_ldl_pivot_ok(d0, require_pd)) return 0;
    f->inv_d0 = (uwb_real)1 / d0;
    f->l10 = s[1] * f->inv_d0;
    d1 = (s[2] + shift) - f->l10 * s[1];
    if (!uwb_ldl_pivot_ok(d1, require_pd)) return 0;
    f->inv_d1 = (uwb_real)1 / d1;
    return 1;
}

/** L D Lᵀ x = b。b と x は同じ配列でもよい。除算 0、乗算 3。 */
static inline void uwb_sym2_ldl_solve(const uwb_sym2_ldl *f, const uwb_real *b, uwb_real *x)
{
    uwb_real y1 = b[1] - f->l10 * b[0];
    uwb_real x1 = y1 * f->inv_d1;
    uwb_real x0 = b[0] * f->inv_d0 - f->l10 * x1;
    x[0] = x0; x[1] = x1;
}

/** (S + shift·I)⁻¹ (パック)。失敗条件は factor と同じ。除算 2。 */
int uwb_sym2_ldl_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv);

/* ------------------------------------------- 対称 n×n の rank-1 更新 */

/** P += c·u uᵀ (P は行優先 n×n、対称)。上三角だけ計算して下三角へ鏡映
 *  するので、結果は厳密に対称になる (丸めの非対称が溜まらない)。
 *  乗算 n(n+1)/2 + n、除算 0。n ≤ UWB_MATH_SYMN_MAX を想定。 */
void uwb_symn_rank1_update(uwb_real *P, int n, const uwb_real *u, uwb_real c);

/** P −= u uᵀ · inv_s。EKF の更新 (Joseph 形式は P が対称なら
 *  P − u uᵀ/s に厳密に潰れる。s = hᵀPh + R、u = Ph、inv_s = 1/s)。
 *  呼び出し側で K = u·inv_s も作れるので除算は 1/s の 1 回で済む。
 *  rank1_update(P, n, u, −inv_s) と同じ。 */
void uwb_symn_rank1_downdate(uwb_real *P, int n, const uwb_real *u, uwb_real inv_s);

/* ------------------------------------------------ 3x3 カーネル (m3) */
/* ブロックコレスキーの内側で使う。単体でも使えるよう公開する。
 * すべて行優先 uwb_real[9]。 */

/** 下三角コレスキー a = L Lᵀ を in-place で (上三角は 0 に)。invd[3] に
 *  対角の逆数を書く (後の trsv が除算なしで済むように)。
 *  正定値でない (ピボット ≤ 0 か NaN) なら 0。sqrt 3、除算 3。 */
int uwb_m3_chol(uwb_real *a, uwb_real *invd);

/** L y = b を前進代入 (L は chol の出力、invd はその逆対角)。
 *  y と b は同じ配列でもよい。乗算 6、除算 0。 */
void uwb_m3_trsv_lower(const uwb_real *l, const uwb_real *invd, const uwb_real *b, uwb_real *y);

/** Lᵀ x = y を後退代入。x と y は同じ配列でもよい。乗算 6、除算 0。 */
void uwb_m3_trsv_upper(const uwb_real *l, const uwb_real *invd, const uwb_real *y, uwb_real *x);

/** X ← X L⁻ᵀ (X の各行について L yᵢ = xᵢᵀ を前進代入)。ブロック
 *  コレスキーの非対角ブロック L_IJ = A_IJ L_JJ⁻ᵀ に使う。乗算 18、除算 0。 */
void uwb_m3_trsm_rt(uwb_real *x, const uwb_real *l, const uwb_real *invd);

/** C −= A Bᵀ。乗算 27。 */
void uwb_m3_gemm_nt_sub(uwb_real *c, const uwb_real *a, const uwb_real *b);

/* ------------------------------------- ブロックコレスキー (測量用) */

/** ブロック数の上限 (= 測量のノード数上限 UWB_SURVEY_MAX_NODES)。 */
#define UWB_BCHOL_MAX_BLOCKS 8

/** 下三角パックのブロック数。 */
#define UWB_BCHOL_NBLK (UWB_BCHOL_MAX_BLOCKS * (UWB_BCHOL_MAX_BLOCKS + 1) / 2)

/** 下三角ブロック (I,J) (I ≥ J) のパック添字。 */
#define UWB_BCHOL_IDX(I, J) ((I) * ((I) + 1) / 2 + (J))

/**
 * 未知数 = nb 個の 3 成分ブロック (+ 任意の縁取りスカラー δ) の対称正定値
 * 行列 A と、そのコレスキー因子 L を同じ場所に持つ。
 *
 *   A = [ A_00                       |  a_0 ]
 *       [ A_10  A_11                 |  a_1 ]   A_IJ : 3x3 (I ≥ J のみ保持)
 *       [ ...                        |  ... ]   a_I  : 3 要素 (縁取り行)
 *       [ A_(nb-1)0 ... A_(nb-1)(nb-1)| a_nb-1]  add  : スカラー A[δ,δ]
 *       [ a_0ᵀ ... a_(nb-1)ᵀ         |  add ]
 *
 * 対角ブロックは 9 要素フルで持つ (カーネルを単純にするため。上三角は
 * factor 後に 0 になる)。nb = 8、縁取りありで 349 要素。
 * 測量 (3·8+1 = 25 未知数) の 25x25 フル格納 625 要素に対し 44%。
 *
 * 使い方:
 *   uwb_bchol_zero(&m, nb, has_border);
 *   リンクごとに uwb_bchol_add_pair_outer(&m, i, j, u, c);   (A += J Jᵀ)
 *   uwb_bchol_damp_diag(&m, lambda, ridge);
 *   if (!uwb_bchol_factor(&m)) { ... λ を上げてやり直し ... }
 *   uwb_bchol_solve(&m, b, x);
 * factor は in-place なので、やり直すときは組み直すか事前に構造体を
 * コピーしておく (構造体代入でよい)。
 */
typedef struct {
    uwb_real blk[UWB_BCHOL_NBLK][9];         /**< A_IJ (I ≥ J)、行優先 3x3。factor 後は L_IJ */
    uwb_real brd[3 * UWB_BCHOL_MAX_BLOCKS];  /**< 縁取り行 a (factor 後は L_δ,·) */
    uwb_real bdd;                            /**< A[δ,δ] (factor 後は L_δδ) */
    uwb_real invd[UWB_BCHOL_MAX_BLOCKS][3];  /**< factor 後: 対角ブロックの対角の逆数 */
    uwb_real inv_bdd;                        /**< factor 後: 1/L_δδ */
    int      nb;                             /**< ブロック数 1..UWB_BCHOL_MAX_BLOCKS */
    int      has_border;                     /**< 縁取り行 δ があれば 1 */
    int      factored;                       /**< factor 成功で 1 */
} uwb_bchol;

/** 未知数の総数 3·nb + has_border。 */
static inline int uwb_bchol_dim(const uwb_bchol *m) { return 3 * m->nb + (m->has_border ? 1 : 0); }

/** 全要素を 0 にして nb / has_border を設定する。nb が範囲外なら 0。 */
int uwb_bchol_zero(uwb_bchol *m, int nb, int has_border);

/** 疎ベクトル v = e_i ⊗ u − e_j ⊗ u + c·e_δ (i ≠ j) に対して A += v vᵀ。
 *  測量の距離残差 1 本ぶんのヤコビ行 (ノード i に +u、ノード j に −u、
 *  共通遅延に c) の寄与そのもの。内訳:
 *    A_ii += u uᵀ,  A_jj += u uᵀ,  A_(max,min) −= u uᵀ,
 *    a_i += c·u,  a_j −= c·u,  add += c²     (has_border のときだけ)
 *  乗算 6 (u uᵀ) + 3 (c·u) + 1 (c²)。has_border = 0 なら c は無視する。 */
void uwb_bchol_add_pair_outer(uwb_bchol *m, int i, int j, const uwb_real *u, uwb_real c);

/** 一般の密な寄与: ブロック (I,J) に 3x3 行優先の d を足す。I < J なら
 *  転置して下三角ブロック (J,I) に足す (対称行列なので同じこと)。
 *  I == J のときは d が対称であること。 */
void uwb_bchol_add_block(uwb_bchol *m, int I, int J, const uwb_real *d);

/** 全対角要素 (縁取り含む) を d ← d(1+λ) + ridge にする (LM 減衰)。 */
void uwb_bchol_damp_diag(uwb_bchol *m, uwb_real lambda, uwb_real ridge);

/** A の (r, c) 要素を読む (r, c は 0..dim−1。縁取りは dim−1)。デバッグ・
 *  テスト用。factor 後は L の要素 (r ≥ c のみ意味を持つ)。 */
uwb_real uwb_bchol_get(const uwb_bchol *m, int r, int c);

/** 左向きブロックコレスキー A = L Lᵀ を in-place で。
 *    L_JJ = chol(A_JJ − Σ_{K<J} L_JK L_JKᵀ)
 *    L_IJ = (A_IJ − Σ_{K<J} L_IK L_JKᵀ) L_JJ⁻ᵀ          (I > J)
 *    L_δJ = (a_Jᵀ − Σ_{K<J} L_δK L_JKᵀ) L_JJ⁻ᵀ
 *    L_δδ = sqrt(add − Σ_K ‖L_δK‖²)
 *  正定値でない (3x3 ピボットが非正、L_δδ² ≤ 0、NaN) なら 0 を返す。
 *  その場合 m の中身は壊れている (factored = 0)。
 *  sqrt 3·nb + border、除算 3·nb + border、乗算 ≈ 4.5·nb³ (nb=8 で ≈ 2.6k)。 */
int uwb_bchol_factor(uwb_bchol *m);

/** factor 済みの m で A x = b を解く (前進・後退代入)。b, x は dim 要素。
 *  b と x は同じ配列でもよい。factored でなければ 0。
 *  除算 0 (逆対角を使う)、乗算 ≈ 9·nb²。 */
int uwb_bchol_solve(const uwb_bchol *m, const uwb_real *b, uwb_real *x);

#ifdef __cplusplus
}
#endif

#endif /* UWB_MATH_H */
