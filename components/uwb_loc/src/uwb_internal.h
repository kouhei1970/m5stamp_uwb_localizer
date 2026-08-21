/* ライブラリ内部の共有部分。公開 API には出さない。 */
#ifndef UWB_INTERNAL_H
#define UWB_INTERNAL_H

#include "uwb_loc.h"
#include "uwb_math.h"

/* 「長さ 0 とみなす」固定しきい値 [m 等]。uwb_math.h の UWB_MATH_TINY と同じ値。
 * 丸め誤差の床 (収束判定など) にはこれではなく、精度に比例する
 * UWB_MATH_EPS (float 1.2e-7 / double 2.2e-16) を使うこと。 */
#define UWB_EPS ((uwb_real)1e-12)

/* ------------------------------------------------ 対称 2x2 / 3x3 の LDLᵀ */
/* 正規方程式・Beck の (Sc + λI)・LLS の AᵀA を解くための、ピボット無し LDLᵀ
 * (パック対称入力。sqrt 無し、除算は対角の逆数 3 回 (2x2 は 2 回) だけ)。
 *
 * なぜ余因子 (uwb_sym3_solve、除算 1) ではなく LDLᵀ か:
 * 余因子展開の行列式は誤差 eps·σ_max³ を持ち、条件数 κ = σ_max/σ_min が
 * 大きいと解の相対誤差が eps·κ·(σ_max/σ_2) まで膨らむ (後退安定でない)。
 * float では κ ≈ 200 の Beck で 2e-4 m、κ ≈ 1e5 の 4 台 LLS で数 m 狂った。
 * LDLᵀ は正定値行列に対して後退安定で、誤差は eps·κ に収まる
 * (同じ条件で 4e-6 m)。GDOP / CRLB のように精度要求が低く条件も良い
 * (単位ベクトルの和) 所だけ余因子 (uwb_sym3_trace_inverse) を使う。
 *
 * require_pd = 1: ピボットが正でなければ 0 (Beck の極、LLS の階数落ち)。
 * require_pd = 0: ピボットが 0 / NaN でなければ通す (旧実装の部分ピボット
 *                 LU と同じ「厳密に特異なときだけ失敗」の意味論。NLS の共分散で
 *                 退化した幾何でも ok=1 と巨大な cov を返すため)。 */
typedef struct {
    uwb_real l10, l20, l21;        /* 単位下三角 L の非対角 */
    uwb_real inv_d0, inv_d1, inv_d2; /* D の逆数 */
} uwb_ldl3;

static inline int uwb_ldl3_ok(uwb_real d, int require_pd)
{
    if (require_pd) return d > (uwb_real)0;
    return d != (uwb_real)0 && d == d && d - d == (uwb_real)0;   /* 0 / NaN / inf を弾く */
}

/** (S + shift·I) = L D Lᵀ。s はパック [xx,xy,xz,yy,yz,zz]。除算 3。 */
static inline int uwb_ldl3_factor(const uwb_real *s, uwb_real shift, int require_pd, uwb_ldl3 *f)
{
    uwb_real d0 = s[0] + shift, d1, d2, t;
    if (!uwb_ldl3_ok(d0, require_pd)) return 0;
    f->inv_d0 = (uwb_real)1 / d0;
    f->l10 = s[1] * f->inv_d0;
    f->l20 = s[2] * f->inv_d0;
    d1 = (s[3] + shift) - f->l10 * s[1];             /* a11 − l10² d0 */
    if (!uwb_ldl3_ok(d1, require_pd)) return 0;
    f->inv_d1 = (uwb_real)1 / d1;
    t = s[4] - f->l20 * s[1];                         /* a21 − l20 l10 d0 */
    f->l21 = t * f->inv_d1;
    d2 = (s[5] + shift) - f->l20 * s[2] - f->l21 * t; /* a22 − l20² d0 − l21² d1 */
    if (!uwb_ldl3_ok(d2, require_pd)) return 0;
    f->inv_d2 = (uwb_real)1 / d2;
    return 1;
}

/** L D Lᵀ x = b。b と x は同じ配列でもよい。除算 0、乗算 9。 */
static inline void uwb_ldl3_solve(const uwb_ldl3 *f, const uwb_real *b, uwb_real *x)
{
    uwb_real y0 = b[0];
    uwb_real y1 = b[1] - f->l10 * y0;
    uwb_real y2 = b[2] - f->l20 * y0 - f->l21 * y1;
    uwb_real x2 = y2 * f->inv_d2;
    uwb_real x1 = y1 * f->inv_d1 - f->l21 * x2;
    uwb_real x0 = y0 * f->inv_d0 - f->l10 * x1 - f->l20 * x2;
    x[0] = x0; x[1] = x1; x[2] = x2;
}

/** (S + shift·I)⁻¹ をパックで (単位ベクトル 3 本を解く)。 */
static inline int uwb_ldl3_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv)
{
    uwb_ldl3 f;
    uwb_real e[3], c[3];
    if (!uwb_ldl3_factor(s, shift, require_pd, &f)) return 0;
    e[0] = (uwb_real)1; e[1] = (uwb_real)0; e[2] = (uwb_real)0;
    uwb_ldl3_solve(&f, e, c); inv[0] = c[0]; inv[1] = c[1]; inv[2] = c[2];
    e[0] = (uwb_real)0; e[1] = (uwb_real)1;
    uwb_ldl3_solve(&f, e, c); inv[3] = c[1]; inv[4] = c[2];
    e[1] = (uwb_real)0; e[2] = (uwb_real)1;
    uwb_ldl3_solve(&f, e, c); inv[5] = c[2];
    return 1;
}

typedef struct {
    uwb_real l10;
    uwb_real inv_d0, inv_d1;
} uwb_ldl2;

/** 2x2 版。s はパック [xx,xy,yy]。除算 2。 */
static inline int uwb_ldl2_factor(const uwb_real *s, uwb_real shift, int require_pd, uwb_ldl2 *f)
{
    uwb_real d0 = s[0] + shift, d1;
    if (!uwb_ldl3_ok(d0, require_pd)) return 0;
    f->inv_d0 = (uwb_real)1 / d0;
    f->l10 = s[1] * f->inv_d0;
    d1 = (s[2] + shift) - f->l10 * s[1];
    if (!uwb_ldl3_ok(d1, require_pd)) return 0;
    f->inv_d1 = (uwb_real)1 / d1;
    return 1;
}

static inline void uwb_ldl2_solve(const uwb_ldl2 *f, const uwb_real *b, uwb_real *x)
{
    uwb_real y1 = b[1] - f->l10 * b[0];
    uwb_real x1 = y1 * f->inv_d1;
    uwb_real x0 = b[0] * f->inv_d0 - f->l10 * x1;
    x[0] = x0; x[1] = x1;
}

static inline int uwb_ldl2_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv)
{
    uwb_ldl2 f;
    uwb_real e[2], c[2];
    if (!uwb_ldl2_factor(s, shift, require_pd, &f)) return 0;
    e[0] = (uwb_real)1; e[1] = (uwb_real)0;
    uwb_ldl2_solve(&f, e, c); inv[0] = c[0]; inv[1] = c[1];
    e[0] = (uwb_real)0; e[1] = (uwb_real)1;
    uwb_ldl2_solve(&f, e, c); inv[2] = c[1];
    return 1;
}

/** 有効な観測か (アンカーの添字が範囲内で、そのアンカーが enabled)。 */
int uwb_meas_usable(const uwb_config *cfg, const uwb_meas *m);

/** アンテナ遅延を引いた測距値。 */
uwb_real uwb_corrected(const uwb_config *cfg, const uwb_meas *m);

/** その観測の 1sigma。measurement.sigma が正ならそれを使い、
 *  無ければアンカーの sigma0 + sigma_per_m * d から作る。
 *  quality が 0-1 で与えられていれば sigma を (1 + 3(1-q)) 倍する
 *  (Python 版の MeasurementModel.sigma と同じ)。 */
uwb_real uwb_sigma_of(const uwb_config *cfg, const uwb_meas *m, uwb_real distance);

/** 観測 1 本を評価する。
 *  residual = 観測値 - 予測値 (NLOS なら正)、jac は予測値の位置微分 dh/dp
 *  (単位ベクトル。距離 0 なら零ベクトル)。 */
void uwb_evaluate(const uwb_config *cfg, const uwb_real *p, const uwb_meas *m,
                  uwb_real *residual, uwb_real *jac3, uwb_real *sigma);

/** 自由変数のマスク。dim==2 なら z を固定するので 2 個。 */
int uwb_n_free(const uwb_config *cfg);

/** dim==2 のとき z を z_fixed に、z_bounds があれば範囲に収める。 */
void uwb_project(const uwb_config *cfg, uwb_real *p);

/** ヤコビアンから GDOP を出す (単位化した行の (H^T H)^-1 のトレースの平方根)。
 *  jac の各行は uwb_evaluate が作る単位ベクトルか零ベクトルであること
 *  (零行は飛ばす。単位化はしない)。解けなければ負を返す。 */
uwb_real uwb_gdop_from_jac(const uwb_config *cfg, const uwb_real *jac, int n);

/** 鏡像解を片側に寄せる。side_known/side は呼び出し側が持つ (EKF 用)。
 *  戻り値 1 なら「どちらか決められなかった」= ambiguous。 */
int uwb_resolve_mirror(const uwb_config *cfg, uwb_real *p, uwb_real *cov,
                       int side_known, int side);

/** 平面の法線に対する符号 (鏡像のどちら側か)。平面が無ければ 0。 */
int uwb_mirror_side(const uwb_config *cfg, const uwb_real *p);

/** fix を「失敗」で埋める。 */
void uwb_fix_failed(uwb_fix *out, int n_total);

/** cov から sigma (= sqrt(trace)) を埋める。 */
void uwb_fix_finish(uwb_fix *out);

#endif /* UWB_INTERNAL_H */
