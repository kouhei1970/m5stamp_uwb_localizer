/* uwb_math — 小次元の線形代数 (スカラー展開)。API の説明は uwb_math.h。
 *
 * ここでの数式の書き方: sym3 のパック添字は
 *   s[0]=S00 s[1]=S01 s[2]=S02 s[3]=S11 s[4]=S12 s[5]=S22
 * 余因子 C_ij は対称行列なので adj = C (転置不要)。
 */
#include "uwb_math.h"

#if UWB_REAL_IS_FLOAT
#define R_ACOS acosf
#define R_COS  cosf
#else
#define R_ACOS acos
#define R_COS  cos
#endif

#define R(x) ((uwb_real)(x))

/* 1/3 と 1/6 は定数倍にする (除算を避ける)。√3 は λ2 の閉形式で使う。 */
#define R_THIRD  R(0.33333333333333333333)
#define R_SIXTH  R(0.16666666666666666667)
#define R_SQRT3  R(1.7320508075688772935)

static uwb_real max3_abs(uwb_real a, uwb_real b, uwb_real c)
{
    uwb_real m = uwb_math_abs(a), t = uwb_math_abs(b);
    if (t > m) m = t;
    t = uwb_math_abs(c);
    if (t > m) m = t;
    return m;
}

/* sym3 の要素の最大絶対値 (スケールの代表)。 */
static uwb_real sym3_maxabs(const uwb_real *s)
{
    uwb_real m1 = max3_abs(s[0], s[1], s[2]);
    uwb_real m2 = max3_abs(s[3], s[4], s[5]);
    return m1 > m2 ? m1 : m2;
}

/* ベクトルの符号を「絶対値最大の成分 (同点なら添字が小さい方) が正」に。 */
static void v3_canonical_sign(uwb_real *v)
{
    uwb_real a0 = uwb_math_abs(v[0]), a1 = uwb_math_abs(v[1]), a2 = uwb_math_abs(v[2]);
    int big = 0;
    if (a1 > a0) { big = 1; a0 = a1; }
    if (a2 > a0) { big = 2; }
    if (v[big] < R(0)) { v[0] = -v[0]; v[1] = -v[1]; v[2] = -v[2]; }
}

/* ================================================================ vec3 */

uwb_real uwb_v3_normalize(uwb_real *v)
{
    uwb_real len = uwb_math_sqrt(uwb_v3_norm2(v));
    uwb_real inv;
    if (!(len > UWB_MATH_TINY)) {            /* 零 or NaN */
        v[0] = v[1] = v[2] = R(0);
        return R(0);
    }
    inv = R(1) / len;
    v[0] *= inv; v[1] *= inv; v[2] *= inv;
    return len;
}

int uwb_v3_perp_basis(const uwb_real *u, uwb_real *e1, uwb_real *e2)
{
    uwb_real n2 = uwb_v3_norm2(u);
    uwb_real a0, a1, a2;
    int k;

    /* 単位ベクトルでなければ (e1, e2) の直交性も右手系も保証できない。 */
    if (!(uwb_math_abs(n2 - R(1)) < R(1e-3))) return 0;

    /* u と最も揃っていない座標軸 (|u_k| 最小) を選び、u 成分を落とす:
     *   e1 = axis_k − u_k u   (Gram-Schmidt 1 段)。‖e1‖² = 1 − u_k² ≥ 2/3 */
    a0 = uwb_math_abs(u[0]); a1 = uwb_math_abs(u[1]); a2 = uwb_math_abs(u[2]);
    k = 0;
    if (a1 < a0) { k = 1; a0 = a1; }
    if (a2 < a0) { k = 2; }
    e1[0] = -u[k] * u[0];
    e1[1] = -u[k] * u[1];
    e1[2] = -u[k] * u[2];
    e1[k] += R(1);
    if (!(uwb_v3_normalize(e1) > R(0))) return 0;
    uwb_v3_cross(u, e1, e2);                 /* e1 × e2 = e1 × (u × e1) = u */
    return 1;
}

/* ================================================================ sym3 */

void uwb_sym3_zero(uwb_real *s)
{
    s[0] = s[1] = s[2] = s[3] = s[4] = s[5] = R(0);
}

void uwb_sym3_add_outer(uwb_real *s, const uwb_real *u)
{
    s[0] += u[0] * u[0]; s[1] += u[0] * u[1]; s[2] += u[0] * u[2];
    s[3] += u[1] * u[1]; s[4] += u[1] * u[2]; s[5] += u[2] * u[2];
}

void uwb_sym3_add_scaled_outer(uwb_real *s, uwb_real w, const uwb_real *u)
{
    uwb_real w0 = w * u[0], w1 = w * u[1], w2 = w * u[2];
    s[0] += w0 * u[0]; s[1] += w0 * u[1]; s[2] += w0 * u[2];
    s[3] += w1 * u[1]; s[4] += w1 * u[2]; s[5] += w2 * u[2];
}

void uwb_sym3_add_diag(uwb_real *s, uwb_real c)
{
    s[0] += c; s[3] += c; s[5] += c;
}

void uwb_sym3_damp_diag(uwb_real *s, uwb_real lambda, uwb_real ridge)
{
    uwb_real f = R(1) + lambda;
    s[0] = s[0] * f + ridge;
    s[3] = s[3] * f + ridge;
    s[5] = s[5] * f + ridge;
}

uwb_real uwb_sym3_trace(const uwb_real *s) { return s[0] + s[3] + s[5]; }

/* 対角余因子 3 つと行列式。solve / inverse / trace_inverse の共通部。
 * c[6] = [C00, C01, C02, C11, C12, C22]。乗算 15。 */
static uwb_real sym3_cofactors(const uwb_real *s, uwb_real *c)
{
    c[0] = s[3] * s[5] - s[4] * s[4];          /* C00 */
    c[1] = s[2] * s[4] - s[1] * s[5];          /* C01 = −(S01 S22 − S12 S02) */
    c[2] = s[1] * s[4] - s[3] * s[2];          /* C02 = S01 S12 − S11 S02 */
    c[3] = s[0] * s[5] - s[2] * s[2];          /* C11 */
    c[4] = s[1] * s[2] - s[0] * s[4];          /* C12 = −(S00 S12 − S01 S02) */
    c[5] = s[0] * s[3] - s[1] * s[1];          /* C22 */
    return s[0] * c[0] + s[1] * c[1] + s[2] * c[2];
}

/* |det| が丸め誤差 (max|s|³ の eps 倍) と区別できるか。NaN は偽。 */
static int sym3_det_ok(const uwb_real *s, uwb_real det)
{
    uwb_real m = sym3_maxabs(s);
    return uwb_math_abs(det) > UWB_MATH_SING_TOL * m * m * m;
}

uwb_real uwb_sym3_det(const uwb_real *s)
{
    uwb_real c[6];
    return sym3_cofactors(s, c);
}

void uwb_sym3_mv(const uwb_real *s, const uwb_real *x, uwb_real *y)
{
    y[0] = s[0] * x[0] + s[1] * x[1] + s[2] * x[2];
    y[1] = s[1] * x[0] + s[3] * x[1] + s[4] * x[2];
    y[2] = s[2] * x[0] + s[4] * x[1] + s[5] * x[2];
}

uwb_real uwb_sym3_quad(const uwb_real *s, const uwb_real *x)
{
    uwb_real y[3];
    uwb_sym3_mv(s, x, y);
    return uwb_v3_dot(x, y);
}

void uwb_sym3_to_full(const uwb_real *s, uwb_real *m)
{
    m[0] = s[0]; m[1] = s[1]; m[2] = s[2];
    m[3] = s[1]; m[4] = s[3]; m[5] = s[4];
    m[6] = s[2]; m[7] = s[4]; m[8] = s[5];
}

void uwb_sym3_from_full(const uwb_real *m, uwb_real *s)
{
    s[0] = m[0]; s[1] = m[1]; s[2] = m[2];
    s[3] = m[4]; s[4] = m[5]; s[5] = m[8];
}

int uwb_sym3_solve(const uwb_real *s, const uwb_real *b, uwb_real *x)
{
    uwb_real c[6], inv, x0, x1, x2;
    uwb_real det = sym3_cofactors(s, c);
    if (!sym3_det_ok(s, det)) return 0;
    inv = R(1) / det;
    x0 = (c[0] * b[0] + c[1] * b[1] + c[2] * b[2]) * inv;
    x1 = (c[1] * b[0] + c[3] * b[1] + c[4] * b[2]) * inv;
    x2 = (c[2] * b[0] + c[4] * b[1] + c[5] * b[2]) * inv;
    x[0] = x0; x[1] = x1; x[2] = x2;
    return 1;
}

int uwb_sym3_solve_shifted(const uwb_real *s, uwb_real shift, const uwb_real *b, uwb_real *x)
{
    uwb_real t[6];
    t[0] = s[0] + shift; t[1] = s[1]; t[2] = s[2];
    t[3] = s[3] + shift; t[4] = s[4]; t[5] = s[5] + shift;
    return uwb_sym3_solve(t, b, x);
}

int uwb_sym3_inverse(const uwb_real *s, uwb_real *inv)
{
    uwb_real c[6], f;
    uwb_real det = sym3_cofactors(s, c);
    if (!sym3_det_ok(s, det)) return 0;
    f = R(1) / det;
    inv[0] = c[0] * f; inv[1] = c[1] * f; inv[2] = c[2] * f;
    inv[3] = c[3] * f; inv[4] = c[4] * f; inv[5] = c[5] * f;
    return 1;
}

int uwb_sym3_inverse_shifted(const uwb_real *s, uwb_real shift, uwb_real *inv)
{
    uwb_real t[6];
    t[0] = s[0] + shift; t[1] = s[1]; t[2] = s[2];
    t[3] = s[3] + shift; t[4] = s[4]; t[5] = s[5] + shift;
    return uwb_sym3_inverse(t, inv);
}

int uwb_sym3_trace_inverse(const uwb_real *s, uwb_real *tr)
{
    /* 対角余因子だけ: C00, C11, C22。det には C01, C02 も要るので結局
     * 5 つ計算する (C12 だけ省ける)。 */
    uwb_real c00 = s[3] * s[5] - s[4] * s[4];
    uwb_real c01 = s[2] * s[4] - s[1] * s[5];
    uwb_real c02 = s[1] * s[4] - s[3] * s[2];
    uwb_real c11 = s[0] * s[5] - s[2] * s[2];
    uwb_real c22 = s[0] * s[3] - s[1] * s[1];
    uwb_real det = s[0] * c00 + s[1] * c01 + s[2] * c02;
    if (!sym3_det_ok(s, det)) return 0;
    *tr = (c00 + c11 + c22) / det;
    return 1;
}

/* 固有値 lam の固有ベクトル。内部版: S − λI を max 要素で正規化してから
 * 3 行の外積を取り、最大ノルムのものを採用する。 */
static int sym3_eigvec_impl(const uwb_real *s, uwb_real lam, uwb_real *v)
{
    uwb_real a[6];
    uwb_real m, inv_m;
    uwb_real r0[3], r1[3], r2[3];
    uwb_real c01[3], c12[3], c20[3];
    uwb_real n01, n12, n20, best;
    const uwb_real *pick;

    a[0] = s[0] - lam; a[1] = s[1];       a[2] = s[2];
    a[3] = s[3] - lam; a[4] = s[4];       a[5] = s[5] - lam;
    m = sym3_maxabs(a);
    if (!(m > R(0))) return 0;                /* S = λI (三重縮退) or NaN */
    inv_m = R(1) / m;

    r0[0] = a[0] * inv_m; r0[1] = a[1] * inv_m; r0[2] = a[2] * inv_m;
    r1[0] = r0[1];        r1[1] = a[3] * inv_m; r1[2] = a[4] * inv_m;
    r2[0] = r0[2];        r2[1] = r1[2];        r2[2] = a[5] * inv_m;

    uwb_v3_cross(r0, r1, c01);
    uwb_v3_cross(r1, r2, c12);
    uwb_v3_cross(r2, r0, c20);
    n01 = uwb_v3_norm2(c01);
    n12 = uwb_v3_norm2(c12);
    n20 = uwb_v3_norm2(c20);

    pick = c01; best = n01;
    if (n12 > best) { pick = c12; best = n12; }
    if (n20 > best) { pick = c20; best = n20; }

    /* 正規化後の行ノルムは高々 √3。外積ノルムが τ·(行ノルム積) 以下なら
     * 行が平行 = (S − λI) のランクが 1 以下 = λ が縮退している。 */
    if (!(best > UWB_MATH_RANK_TOL * UWB_MATH_RANK_TOL)) return 0;

    v[0] = pick[0]; v[1] = pick[1]; v[2] = pick[2];
    if (!(uwb_v3_normalize(v) > R(0))) return 0;
    v3_canonical_sign(v);
    return 1;
}

/* 対称 2x2 の固有値 (降順)。sym2 版の内部共有。
 * λ = m ± sqrt(d² + b²)。和の平方根なので桁落ちしない。sqrt 1。 */
static void sym2_eigvals_impl(uwb_real a, uwb_real b, uwb_real c, uwb_real *lam)
{
    uwb_real m  = R(0.5) * (a + c);
    uwb_real d  = R(0.5) * (a - c);
    uwb_real rt = uwb_math_sqrt(d * d + b * b);
    lam[0] = m + rt;
    lam[1] = m - rt;
}

int uwb_sym3_eigvals(const uwb_real *s, uwb_real *lam)
{
    uwb_real q  = (s[0] + s[3] + s[5]) * R_THIRD;
    uwb_real a  = s[0] - q, d = s[3] - q, f = s[5] - q;
    uwb_real p2 = a * a + d * d + f * f + R(2) * (s[1] * s[1] + s[2] * s[2] + s[4] * s[4]);
    uwb_real p  = uwb_math_sqrt(p2 * R_SIXTH);
    uwb_real inv_p, b[6], r, phi, c, sn;
    uwb_real l0, l1, l2;

    if (uwb_math_isnan(p) || uwb_math_isnan(q)) return 0;
    if (!(p > R(4) * UWB_MATH_EPS * uwb_math_abs(q))) {
        /* S ≈ qI。p は丸め誤差でしか無いので三重縮退。 */
        lam[0] = lam[1] = lam[2] = q;
        return 1;
    }

    /* B = (S − qI)/p は tr = 0, ‖B‖_F² = 6 で、det(B)/2 ∈ [−1, 1]。 */
    inv_p = R(1) / p;
    b[0] = a * inv_p;    b[1] = s[1] * inv_p; b[2] = s[2] * inv_p;
    b[3] = d * inv_p;    b[4] = s[4] * inv_p; b[5] = f * inv_p;
    r = uwb_sym3_det(b) * R(0.5);
    r = uwb_math_clamp(r, R(-1), R(1));

    phi = R_ACOS(r) * R_THIRD;                     /* φ ∈ [0, π/3] */
    c   = R_COS(phi);
    sn  = uwb_math_sqrt(R(1) - c * c);             /* sin φ ≥ 0 */
    l0  = q + R(2) * p * c;                        /* q + 2p cos φ */
    l2  = q - p * (c + R_SQRT3 * sn);              /* q + 2p cos(φ + 2π/3) */
    l1  = R(3) * q - l0 - l2;                      /* tr の保存 */

    /* |r| ≈ 1 は「2 つの固有値が縮退」。acos は ±1 の近くで傾きが無限大
     * なので、r の丸め誤差 eps が角度に √eps で効き、縮退した対の分離に
     * √eps·‖S‖ の誤差が乗る。孤立した方は φ について停留点にある
     * (dλ/dφ = 0) ので 2 次精度で正しい。
     * そこでこの領域では、孤立固有値の固有ベクトル v で直交補空間へ射影した
     * 2x2 から対を解き直す (deflation)。2x2 の閉形式は和の平方根なので
     * 桁落ちしない。λ_iso も Rayleigh 商 vᵀSv で磨く。 */
    if (R(1) - uwb_math_abs(r) < R(1e-4)) {
        uwb_real viso[3], e1[3], e2[3], w1[3], w2[3], pair[2], liso;
        /* r ≈ +1: φ ≈ 0   → λ0 孤立、λ1 ≈ λ2
         * r ≈ −1: φ ≈ π/3 → λ2 孤立、λ0 ≈ λ1 */
        liso = (r > R(0)) ? l0 : l2;
        if (sym3_eigvec_impl(s, liso, viso) && uwb_v3_perp_basis(viso, e1, e2)) {
            uwb_sym3_mv(s, viso, w1);
            liso = uwb_v3_dot(viso, w1);
            uwb_sym3_mv(s, e1, w1);
            uwb_sym3_mv(s, e2, w2);
            sym2_eigvals_impl(uwb_v3_dot(e1, w1), uwb_v3_dot(e1, w2), uwb_v3_dot(e2, w2), pair);
            if (r > R(0)) { lam[0] = liso;    lam[1] = pair[0]; lam[2] = pair[1]; }
            else          { lam[0] = pair[0]; lam[1] = pair[1]; lam[2] = liso;    }
            /* 丸めで順序が崩れたら直す (孤立値との間は十分離れている) */
            if (lam[1] > lam[0]) { uwb_real t = lam[0]; lam[0] = lam[1]; lam[1] = t; }
            if (lam[2] > lam[1]) { uwb_real t = lam[1]; lam[1] = lam[2]; lam[2] = t; }
            return 1;
        }
        /* 固有ベクトルが取れない (ほぼ三重縮退) → 閉形式の値をそのまま返す */
    }

    lam[0] = l0; lam[1] = l1; lam[2] = l2;
    return 1;
}

/* 全固有対を「孤立固有値 → その直交補空間の 2x2」の順で求める内部版。
 *
 * 行外積法 (sym3_eigvec_impl) の固有ベクトルの誤差は eps·‖S‖/gap で、
 * gap は**最も近い**固有値との間隔。しかもその誤差は縮退した部分空間の
 * 外 (遠い固有ベクトルの方向) にも漏れるので、近接した対に外積法を直接
 * 使うと ‖Sv − λv‖ が eps·‖S‖²/gap まで悪化する。そこで
 *   1. 最も孤立した固有値 (gap が最大 = スペクトル幅の半分以上) だけ
 *      外積法で取る。誤差は eps·‖S‖/幅 で済む。
 *   2. その直交補空間 (e1, e2) に S を射影した 2x2 T で残りの対を解く。
 *      対の中での向きの誤差は「縮退面の中で回る」だけで、残差は eps 止まり。
 *   3. 固有値も v_iso の Rayleigh 商と T の閉形式で磨き直す
 *      (閉形式の √eps 誤差が消え、S V = V Λ が eps で成り立つ)。
 * 戻り値 0 は NaN。*triple = 三重縮退 (V = I)、*pair_deg = 孤立していない
 * 2 つが縮退 (その面内の基底は任意)、*iso = 孤立固有値の添字 (0 か 2)。 */
static int sym3_eig_impl(const uwb_real *s, uwb_real *lam,
                         uwb_real *v0, uwb_real *v1, uwb_real *v2,
                         int *iso, int *pair_deg, int *triple)
{
    uwb_real viso[3], e1[3], e2[3], w[3], w2[3], va[3], vb[3];
    uwb_real pair[2], liso, ta, tb, tc, cc[2];
    uwb_real scale, thr_num, thr_sem, g01, g12, gmax, gp;
    int k;

    *triple = 0; *pair_deg = 0; *iso = 0;
    if (!uwb_sym3_eigvals(s, lam)) return 0;

    /* 2 種類のしきい値:
     *   thr_num: 分岐の数値的な安全域 (16 eps)。これを超える間隔なら 2x2 の
     *            固有ベクトルは「縮退面の中で」しかぶれず、残差は eps 止まり
     *   thr_sem: 呼び出し側に「縮退 = 向きが決まらない」と伝える意味論上の
     *            境界 (UWB_MATH_RANK_TOL)。min_eigvec / eigvec の失敗条件 */
    scale   = uwb_math_max(uwb_math_abs(lam[0]), uwb_math_abs(lam[2]));
    thr_num = R(16) * UWB_MATH_EPS * scale;
    thr_sem = UWB_MATH_RANK_TOL * scale;
    g01     = lam[0] - lam[1];
    g12     = lam[1] - lam[2];
    gmax    = uwb_math_max(g01, g12);
    *iso    = (g01 >= g12) ? 0 : 2;

    if (!(gmax > thr_num) || !sym3_eigvec_impl(s, lam[*iso], viso)) {
        /* 三重縮退: どの方向も固有ベクトル */
        v0[0] = R(1); v0[1] = R(0); v0[2] = R(0);
        v1[0] = R(0); v1[1] = R(1); v1[2] = R(0);
        v2[0] = R(0); v2[1] = R(0); v2[2] = R(1);
        *triple = 1; *pair_deg = 1;
        return 1;
    }
    *triple = !(gmax > thr_sem);
    uwb_v3_perp_basis(viso, e1, e2);           /* viso は単位なので失敗しない */

    uwb_sym3_mv(s, viso, w);
    liso = uwb_v3_dot(viso, w);                /* Rayleigh 商 (2 次精度) */
    uwb_sym3_mv(s, e1, w);
    uwb_sym3_mv(s, e2, w2);
    ta = uwb_v3_dot(e1, w); tb = uwb_v3_dot(e1, w2); tc = uwb_v3_dot(e2, w2);
    sym2_eigvals_impl(ta, tb, tc, pair);
    gp = pair[0] - pair[1];
    *pair_deg = !(gp > thr_sem);

    {
        uwb_real t[3];
        t[0] = ta; t[1] = tb; t[2] = tc;
        if (gp > thr_num && uwb_sym2_eigvec(t, pair[0], cc)) {
            for (k = 0; k < 3; ++k) va[k] = cc[0] * e1[k] + cc[1] * e2[k];
            uwb_v3_cross(viso, va, vb);        /* viso ⊥ va なので単位 */
        } else {
            uwb_v3_copy(e1, va);
            uwb_v3_copy(e2, vb);
        }
    }

    if (*iso == 0) {
        lam[0] = liso;    lam[1] = pair[0]; lam[2] = pair[1];
        uwb_v3_copy(viso, v0); uwb_v3_copy(va, v1); uwb_v3_copy(vb, v2);
    } else {
        lam[0] = pair[0]; lam[1] = pair[1]; lam[2] = liso;
        uwb_v3_copy(va, v0); uwb_v3_copy(vb, v1); uwb_v3_copy(viso, v2);
    }
    /* 磨き直しで順序が eps だけ崩れることがあるので整える (ベクトルも一緒に) */
    if (lam[1] > lam[0]) {
        uwb_real t = lam[0]; lam[0] = lam[1]; lam[1] = t;
        uwb_v3_copy(v0, w); uwb_v3_copy(v1, v0); uwb_v3_copy(w, v1);
    }
    if (lam[2] > lam[1]) {
        uwb_real t = lam[1]; lam[1] = lam[2]; lam[2] = t;
        uwb_v3_copy(v1, w); uwb_v3_copy(v2, v1); uwb_v3_copy(w, v2);
        if (lam[1] > lam[0]) {
            t = lam[0]; lam[0] = lam[1]; lam[1] = t;
            uwb_v3_copy(v0, w); uwb_v3_copy(v1, v0); uwb_v3_copy(w, v1);
        }
    }
    v3_canonical_sign(v0);
    v3_canonical_sign(v1);
    v3_canonical_sign(v2);
    return 1;
}

int uwb_sym3_eigvec(const uwb_real *s, uwb_real lam, uwb_real *v)
{
    uwb_real l[3], v0[3], v1[3], v2[3], d0, d1, d2;
    int iso, pair_deg, triple, k;
    if (!sym3_eig_impl(s, l, v0, v1, v2, &iso, &pair_deg, &triple)) return 0;
    if (triple) return 0;
    d0 = uwb_math_abs(l[0] - lam);
    d1 = uwb_math_abs(l[1] - lam);
    d2 = uwb_math_abs(l[2] - lam);
    k = 0;
    if (d1 < d0) { k = 1; d0 = d1; }
    if (d2 < d0) { k = 2; }
    if (k != iso && pair_deg) return 0;        /* 縮退した対の中: 向きが決まらない */
    uwb_v3_copy(k == 0 ? v0 : (k == 1 ? v1 : v2), v);
    return 1;
}

int uwb_sym3_min_eigvec(const uwb_real *s, uwb_real *lam, uwb_real *v)
{
    uwb_real l[3], v0[3], v1[3], v2[3];
    int iso, pair_deg, triple;
    if (!sym3_eig_impl(s, l, v0, v1, v2, &iso, &pair_deg, &triple)) return 0;
    if (lam) { lam[0] = l[0]; lam[1] = l[1]; lam[2] = l[2]; }
    if (triple) return 0;
    if (iso != 2 && pair_deg) return 0;        /* λ1 ≈ λ2: 法線が決まらない */
    uwb_v3_copy(v2, v);
    return 1;
}

int uwb_sym3_eig(const uwb_real *s, uwb_real *lam, uwb_real *vec)
{
    uwb_real v0[3], v1[3], v2[3];
    int iso, pair_deg, triple;
    if (!sym3_eig_impl(s, lam, v0, v1, v2, &iso, &pair_deg, &triple)) return 0;
    vec[0] = v0[0]; vec[1] = v1[0]; vec[2] = v2[0];
    vec[3] = v0[1]; vec[4] = v1[1]; vec[5] = v2[1];
    vec[6] = v0[2]; vec[7] = v1[2]; vec[8] = v2[2];
    return 1;
}

int uwb_sym3_rank(const uwb_real *lam, uwb_real rel_tol, uwb_real abs_tol)
{
    uwb_real thr = rel_tol * lam[0];
    int r = 0;
    if (abs_tol > thr) thr = abs_tol;
    if (lam[0] > thr) ++r;
    if (lam[1] > thr) ++r;
    if (lam[2] > thr) ++r;
    return r;
}

void uwb_sym3_reflect(uwb_real *s, const uwb_real *n)
{
    uwb_real w[3], k4, t0, t1, t2;
    uwb_sym3_mv(s, n, w);                          /* w = S n */
    k4 = R(4) * uwb_v3_dot(n, w);                  /* 4 nᵀ S n */
    t0 = k4 * n[0]; t1 = k4 * n[1]; t2 = k4 * n[2];
    /* S_ij − 2 n_i w_j − 2 w_i n_j + (4 nᵀSn) n_i n_j */
    s[0] += -R(4) * n[0] * w[0]              + t0 * n[0];
    s[1] += -R(2) * (n[0] * w[1] + w[0] * n[1]) + t0 * n[1];
    s[2] += -R(2) * (n[0] * w[2] + w[0] * n[2]) + t0 * n[2];
    s[3] += -R(4) * n[1] * w[1]              + t1 * n[1];
    s[4] += -R(2) * (n[1] * w[2] + w[1] * n[2]) + t1 * n[2];
    s[5] += -R(4) * n[2] * w[2]              + t2 * n[2];
}

uwb_real uwb_sym3_adjugate(const uwb_real *s, uwb_real *adj)
{
    return sym3_cofactors(s, adj);
}

int uwb_sym3_solve_tol(const uwb_real *s, const uwb_real *b, uwb_real *x, uwb_real rel_tol)
{
    uwb_real c[6], inv, x0, x1, x2;
    uwb_real det = sym3_cofactors(s, c);
    uwb_real m = sym3_maxabs(s);
    /* NaN は比較が偽で落ちる。±inf は det − det が NaN になるので別に弾く
     * (rel_tol = 0 のとき |inf| > 0 が真になってしまうため)。 */
    if (!(uwb_math_abs(det) > rel_tol * m * m * m) || !(det - det == R(0))) return 0;
    inv = R(1) / det;
    x0 = (c[0] * b[0] + c[1] * b[1] + c[2] * b[2]) * inv;
    x1 = (c[1] * b[0] + c[3] * b[1] + c[4] * b[2]) * inv;
    x2 = (c[2] * b[0] + c[4] * b[1] + c[5] * b[2]) * inv;
    x[0] = x0; x[1] = x1; x[2] = x2;
    return 1;
}

int uwb_sym3_null_vector(const uwb_real *s, uwb_real *n)
{
    uwb_real m = sym3_maxabs(s), inv_m;
    uwb_real r0[3], r1[3], r2[3];
    uwb_real c01[3], c12[3], c20[3], n01, n12, n20, v[3];
    const uwb_real *best;

    /* max3_abs は NaN を「大きくない」扱いで読み飛ばすので和で別に見る。 */
    if (uwb_math_isnan(s[0] + s[1] + s[2] + s[3] + s[4] + s[5])) return 0;
    if (!(m > R(0))) return 0;                /* S = 0 */
    inv_m = R(1) / m;

    /* 行を max|s_ij| で正規化: 外積は s² のオーダーなので、1e-6 スケールの S
     * をそのまま使うと ‖外積‖ ≈ 1e-12 が UWB_MATH_TINY に掛かってしまう。 */
    r0[0] = s[0] * inv_m; r0[1] = s[1] * inv_m; r0[2] = s[2] * inv_m;
    r1[0] = r0[1];        r1[1] = s[3] * inv_m; r1[2] = s[4] * inv_m;
    r2[0] = r0[2];        r2[1] = r1[2];        r2[2] = s[5] * inv_m;

    uwb_v3_cross(r0, r1, c01);
    uwb_v3_cross(r1, r2, c12);
    uwb_v3_cross(r2, r0, c20);
    n01 = uwb_v3_norm2(c01); n12 = uwb_v3_norm2(c12); n20 = uwb_v3_norm2(c20);
    best = c01;
    if (n12 > n01 && n12 >= n20) best = c12;
    else if (n20 > n01)          best = c20;

    uwb_v3_copy(best, v);
    if (!(uwb_v3_normalize(v) > R(0))) return 0;   /* 3 行が平行 (rank ≤ 1) */
    v3_canonical_sign(v);
    uwb_v3_copy(v, n);
    return 1;
}

int uwb_sym3_principal_axis(const uwb_real *s, uwb_real *v)
{
    const uwb_real r0[3] = { s[0], s[1], s[2] };
    const uwb_real r1[3] = { s[1], s[3], s[4] };
    const uwb_real r2[3] = { s[2], s[4], s[5] };
    uwb_real m0, m1, m2, t[3];

    if (uwb_math_isnan(s[0] + s[1] + s[2] + s[3] + s[4] + s[5])) return 0;
    m0 = uwb_v3_norm2(r0); m1 = uwb_v3_norm2(r1); m2 = uwb_v3_norm2(r2);
    if (m1 >= m0 && m1 >= m2) uwb_v3_copy(r1, t);
    else if (m2 >= m0)        uwb_v3_copy(r2, t);
    else                      uwb_v3_copy(r0, t);
    if (!(uwb_v3_normalize(t) > R(0))) return 0;   /* S ≈ 0 */
    v3_canonical_sign(t);
    uwb_v3_copy(t, v);
    return 1;
}

int uwb_sym3_solve_sphere(const uwb_real *m, const uwb_real *b, uwb_real lam_min,
                          uwb_real *u, uwb_real *out_nu)
{
    uwb_real nu = R(0), nu_x = R(0), lo = -lam_min, x[3], w[3];
    uwb_sym3_ldl f;
    int it;

    for (it = 0; it < 30; ++it) {
        uwb_real n2, nrm, uw, dnu, nu_new;

        /* u(ν) = (M + νI)⁻¹ b。因子は w = (M + νI)⁻¹ u にも使い回す。
         * 正定値でなければ極 (ν ≤ −λ_min) か M 自体が不定で、解の側にいない。
         * 最初の ν = 0 で失敗したら M が正定値でない。2 回目以降の失敗は二分で
         * 極に達した hard case (float では ν が丸めで −λ_min ちょうどになる) で、
         * 直前の x を使う。 */
        if (!uwb_sym3_ldl_factor(m, nu, 1, &f)) {
            if (it == 0) return 0;
            break;
        }
        uwb_sym3_ldl_solve(&f, b, x);
        nu_x = nu;
        n2  = uwb_v3_norm2(x);
        nrm = uwb_math_sqrt(n2);
        if (!(nrm > R(0))) return 0;                      /* b = 0 or NaN */
        if (uwb_math_abs(nrm - R(1)) <= R(16) * UWB_MATH_EPS) break;

        uwb_sym3_ldl_solve(&f, x, w);
        uw = uwb_v3_dot(x, w);                            /* = ‖u‖³ φ'(ν) > 0 */
        if (!(uw > R(0))) break;
        dnu    = n2 * (nrm - R(1)) / uw;
        nu_new = nu + dnu;
        if (!(nu_new > lo)) nu_new = R(0.5) * (nu + lo);  /* 極の手前で止める */
        if (nu_new == nu) break;
        nu = nu_new;
    }
    /* 打ち切り時 (hard case) も含め、最後に長さを 1 に揃える */
    if (!(uwb_v3_normalize(x) > R(0))) return 0;
    uwb_v3_copy(x, u);
    if (out_nu) *out_nu = nu_x;
    return 1;
}

/* ================================================================ sym2 */

void uwb_sym2_zero(uwb_real *s) { s[0] = s[1] = s[2] = R(0); }

void uwb_sym2_add_outer(uwb_real *s, const uwb_real *u)
{
    s[0] += u[0] * u[0]; s[1] += u[0] * u[1]; s[2] += u[1] * u[1];
}

void uwb_sym2_add_scaled_outer(uwb_real *s, uwb_real w, const uwb_real *u)
{
    uwb_real w0 = w * u[0], w1 = w * u[1];
    s[0] += w0 * u[0]; s[1] += w0 * u[1]; s[2] += w1 * u[1];
}

void uwb_sym2_add_diag(uwb_real *s, uwb_real c) { s[0] += c; s[2] += c; }

void uwb_sym2_damp_diag(uwb_real *s, uwb_real lambda, uwb_real ridge)
{
    uwb_real f = R(1) + lambda;
    s[0] = s[0] * f + ridge;
    s[2] = s[2] * f + ridge;
}

uwb_real uwb_sym2_trace(const uwb_real *s) { return s[0] + s[2]; }

uwb_real uwb_sym2_det(const uwb_real *s) { return s[0] * s[2] - s[1] * s[1]; }

static int sym2_det_ok(const uwb_real *s, uwb_real det)
{
    uwb_real m = max3_abs(s[0], s[1], s[2]);
    return uwb_math_abs(det) > UWB_MATH_SING_TOL * m * m;
}

int uwb_sym2_solve(const uwb_real *s, const uwb_real *b, uwb_real *x)
{
    uwb_real det = uwb_sym2_det(s), inv, x0, x1;
    if (!sym2_det_ok(s, det)) return 0;
    inv = R(1) / det;
    x0 = (s[2] * b[0] - s[1] * b[1]) * inv;
    x1 = (s[0] * b[1] - s[1] * b[0]) * inv;
    x[0] = x0; x[1] = x1;
    return 1;
}

int uwb_sym2_inverse(const uwb_real *s, uwb_real *inv)
{
    uwb_real det = uwb_sym2_det(s), f;
    if (!sym2_det_ok(s, det)) return 0;
    f = R(1) / det;
    inv[0] = s[2] * f; inv[1] = -s[1] * f; inv[2] = s[0] * f;
    return 1;
}

int uwb_sym2_trace_inverse(const uwb_real *s, uwb_real *tr)
{
    uwb_real det = uwb_sym2_det(s);
    if (!sym2_det_ok(s, det)) return 0;
    *tr = (s[0] + s[2]) / det;
    return 1;
}

int uwb_sym2_eigvals(const uwb_real *s, uwb_real *lam)
{
    sym2_eigvals_impl(s[0], s[1], s[2], lam);
    return !uwb_math_isnan(lam[0]);
}

int uwb_sym2_eigvec(const uwb_real *s, uwb_real lam, uwb_real *v)
{
    uwb_real a = s[0] - lam, b = s[1], c = s[2] - lam;
    uwb_real m = max3_abs(a, b, c);
    uwb_real n0 = a * a + b * b, n1 = b * b + c * c, len, inv;
    /* S − λI が丸め誤差の範囲で 0 (‖S‖ に対して) なら λ は二重固有値で
     * 向きが決まらない。しきい値は数値的な安全域 (4 eps) だけにしてある:
     * それ以上の間隔なら、誤差は固有ベクトルの向きを eps·‖S‖/gap だけ
     * 回すに留まり、残差 ‖Sv − λv‖ は eps·‖S‖ で収まる。 */
    if (!(m > R(4) * UWB_MATH_EPS * max3_abs(s[0], s[1], s[2]))) return 0;
    /* 長い方の行 (a, b) か (b, c) に直交するベクトル */
    if (n0 >= n1) { v[0] = -b; v[1] = a; }
    else          { v[0] = -c; v[1] = b; }
    len = uwb_math_sqrt(v[0] * v[0] + v[1] * v[1]);
    if (!(len > R(0))) return 0;
    inv = R(1) / len;
    v[0] *= inv; v[1] *= inv;
    if ((uwb_math_abs(v[0]) >= uwb_math_abs(v[1]) ? v[0] : v[1]) < R(0)) {
        v[0] = -v[0]; v[1] = -v[1];
    }
    return 1;
}

void uwb_sym2_mv(const uwb_real *s, const uwb_real *x, uwb_real *y)
{
    y[0] = s[0] * x[0] + s[1] * x[1];
    y[1] = s[1] * x[0] + s[2] * x[1];
}

int uwb_sym2_solve_shifted(const uwb_real *s, uwb_real shift, const uwb_real *b, uwb_real *x)
{
    uwb_real t[3];
    t[0] = s[0] + shift; t[1] = s[1]; t[2] = s[2] + shift;
    return uwb_sym2_solve(t, b, x);
}

int uwb_sym2_inverse_shifted(const uwb_real *s, uwb_real shift, uwb_real *inv)
{
    uwb_real t[3];
    t[0] = s[0] + shift; t[1] = s[1]; t[2] = s[2] + shift;
    return uwb_sym2_inverse(t, inv);
}

int uwb_sym2_solve_tol(const uwb_real *s, const uwb_real *b, uwb_real *x, uwb_real rel_tol)
{
    uwb_real det = uwb_sym2_det(s), inv, x0, x1;
    uwb_real m = max3_abs(s[0], s[1], s[2]);
    if (!(uwb_math_abs(det) > rel_tol * m * m) || !(det - det == R(0))) return 0;
    inv = R(1) / det;
    x0 = (s[2] * b[0] - s[1] * b[1]) * inv;
    x1 = (s[0] * b[1] - s[1] * b[0]) * inv;
    x[0] = x0; x[1] = x1;
    return 1;
}

/* ======================================================= LDLᵀ (sym3 / sym2) */
/* factor / solve は uwb_math.h の static inline (式は uwb_loc の旧 uwb_internal.h
 * から演算順序を変えずに移したもの。uwb_loc の回帰テストが数値の同一性を
 * 前提にしている)。ここには inverse だけを置く。 */

int uwb_sym3_ldl_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv)
{
    uwb_sym3_ldl f;
    uwb_real e[3], c[3];
    if (!uwb_sym3_ldl_factor(s, shift, require_pd, &f)) return 0;
    e[0] = R(1); e[1] = R(0); e[2] = R(0);
    uwb_sym3_ldl_solve(&f, e, c); inv[0] = c[0]; inv[1] = c[1]; inv[2] = c[2];
    e[0] = R(0); e[1] = R(1);
    uwb_sym3_ldl_solve(&f, e, c); inv[3] = c[1]; inv[4] = c[2];
    e[1] = R(0); e[2] = R(1);
    uwb_sym3_ldl_solve(&f, e, c); inv[5] = c[2];
    return 1;
}

int uwb_sym2_ldl_inverse(const uwb_real *s, uwb_real shift, int require_pd, uwb_real *inv)
{
    uwb_sym2_ldl f;
    uwb_real e[2], c[2];
    if (!uwb_sym2_ldl_factor(s, shift, require_pd, &f)) return 0;
    e[0] = R(1); e[1] = R(0);
    uwb_sym2_ldl_solve(&f, e, c); inv[0] = c[0]; inv[1] = c[1];
    e[0] = R(0); e[1] = R(1);
    uwb_sym2_ldl_solve(&f, e, c); inv[2] = c[1];
    return 1;
}

/* ============================================== symn rank-1 (EKF の P) */

void uwb_symn_rank1_update(uwb_real *P, int n, const uwb_real *u, uwb_real c)
{
    int a, b;
    for (a = 0; a < n; ++a) {
        uwb_real ca = c * u[a];
        uwb_real *row = P + a * n;
        row[a] += ca * u[a];
        for (b = a + 1; b < n; ++b) {
            uwb_real v = row[b] + ca * u[b];
            row[b] = v;
            P[b * n + a] = v;                      /* 鏡映 */
        }
    }
}

void uwb_symn_rank1_downdate(uwb_real *P, int n, const uwb_real *u, uwb_real inv_s)
{
    uwb_symn_rank1_update(P, n, u, -inv_s);
}

/* ======================================================== m3 カーネル */

int uwb_m3_chol(uwb_real *a, uwb_real *invd)
{
    uwb_real l00, l10, l20, l11, l21, l22, t;

    if (!(a[0] > R(0))) return 0;
    l00 = uwb_math_sqrt(a[0]);
    invd[0] = R(1) / l00;
    l10 = a[3] * invd[0];
    l20 = a[6] * invd[0];

    t = a[4] - l10 * l10;
    if (!(t > R(0))) return 0;
    l11 = uwb_math_sqrt(t);
    invd[1] = R(1) / l11;
    l21 = (a[7] - l20 * l10) * invd[1];

    t = a[8] - l20 * l20 - l21 * l21;
    if (!(t > R(0))) return 0;
    l22 = uwb_math_sqrt(t);
    invd[2] = R(1) / l22;

    a[0] = l00; a[1] = R(0); a[2] = R(0);
    a[3] = l10; a[4] = l11;  a[5] = R(0);
    a[6] = l20; a[7] = l21;  a[8] = l22;
    return 1;
}

void uwb_m3_trsv_lower(const uwb_real *l, const uwb_real *invd, const uwb_real *b, uwb_real *y)
{
    uwb_real y0 = b[0] * invd[0];
    uwb_real y1 = (b[1] - l[3] * y0) * invd[1];
    uwb_real y2 = (b[2] - l[6] * y0 - l[7] * y1) * invd[2];
    y[0] = y0; y[1] = y1; y[2] = y2;
}

void uwb_m3_trsv_upper(const uwb_real *l, const uwb_real *invd, const uwb_real *y, uwb_real *x)
{
    uwb_real x2 = y[2] * invd[2];
    uwb_real x1 = (y[1] - l[7] * x2) * invd[1];
    uwb_real x0 = (y[0] - l[3] * x1 - l[6] * x2) * invd[0];
    x[0] = x0; x[1] = x1; x[2] = x2;
}

void uwb_m3_trsm_rt(uwb_real *x, const uwb_real *l, const uwb_real *invd)
{
    /* X L⁻ᵀ: 行ごとに L yᵀ = xᵀ を前進代入 */
    uwb_m3_trsv_lower(l, invd, x + 0, x + 0);
    uwb_m3_trsv_lower(l, invd, x + 3, x + 3);
    uwb_m3_trsv_lower(l, invd, x + 6, x + 6);
}

void uwb_m3_gemm_nt_sub(uwb_real *c, const uwb_real *a, const uwb_real *b)
{
    c[0] -= a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    c[1] -= a[0] * b[3] + a[1] * b[4] + a[2] * b[5];
    c[2] -= a[0] * b[6] + a[1] * b[7] + a[2] * b[8];
    c[3] -= a[3] * b[0] + a[4] * b[1] + a[5] * b[2];
    c[4] -= a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
    c[5] -= a[3] * b[6] + a[4] * b[7] + a[5] * b[8];
    c[6] -= a[6] * b[0] + a[7] * b[1] + a[8] * b[2];
    c[7] -= a[6] * b[3] + a[7] * b[4] + a[8] * b[5];
    c[8] -= a[6] * b[6] + a[7] * b[7] + a[8] * b[8];
}

/* ================================================ ブロックコレスキー */

int uwb_bchol_zero(uwb_bchol *m, int nb, int has_border)
{
    int i, k;
    if (!m || nb < 1 || nb > UWB_BCHOL_MAX_BLOCKS) return 0;
    for (i = 0; i < UWB_BCHOL_IDX(nb - 1, nb - 1) + 1; ++i)
        for (k = 0; k < 9; ++k) m->blk[i][k] = R(0);
    for (i = 0; i < 3 * nb; ++i) m->brd[i] = R(0);
    for (i = 0; i < nb; ++i) m->invd[i][0] = m->invd[i][1] = m->invd[i][2] = R(0);
    m->bdd        = R(0);
    m->inv_bdd    = R(0);
    m->nb         = nb;
    m->has_border = has_border ? 1 : 0;
    m->factored   = 0;
    return 1;
}

/* 対称 3x3 (パック) をブロックに足す / 引く。 */
static void blk_add_sym(uwb_real *blk, const uwb_real *uu, uwb_real sign)
{
    blk[0] += sign * uu[0]; blk[1] += sign * uu[1]; blk[2] += sign * uu[2];
    blk[3] += sign * uu[1]; blk[4] += sign * uu[3]; blk[5] += sign * uu[4];
    blk[6] += sign * uu[2]; blk[7] += sign * uu[4]; blk[8] += sign * uu[5];
}

void uwb_bchol_add_pair_outer(uwb_bchol *m, int i, int j, const uwb_real *u, uwb_real c)
{
    uwb_real uu[6];
    int I, J;
    if (i == j || i < 0 || j < 0 || i >= m->nb || j >= m->nb) return;

    uu[0] = u[0] * u[0]; uu[1] = u[0] * u[1]; uu[2] = u[0] * u[2];
    uu[3] = u[1] * u[1]; uu[4] = u[1] * u[2]; uu[5] = u[2] * u[2];

    I = i > j ? i : j;
    J = i > j ? j : i;
    blk_add_sym(m->blk[UWB_BCHOL_IDX(i, i)], uu, R(1));
    blk_add_sym(m->blk[UWB_BCHOL_IDX(j, j)], uu, R(1));
    blk_add_sym(m->blk[UWB_BCHOL_IDX(I, J)], uu, R(-1));

    if (m->has_border) {
        uwb_real c0 = c * u[0], c1 = c * u[1], c2 = c * u[2];
        m->brd[3 * i + 0] += c0; m->brd[3 * i + 1] += c1; m->brd[3 * i + 2] += c2;
        m->brd[3 * j + 0] -= c0; m->brd[3 * j + 1] -= c1; m->brd[3 * j + 2] -= c2;
        m->bdd += c * c;
    }
}

void uwb_bchol_add_block(uwb_bchol *m, int I, int J, const uwb_real *d)
{
    uwb_real *b;
    int k;
    if (I < 0 || J < 0 || I >= m->nb || J >= m->nb) return;
    if (I >= J) {
        b = m->blk[UWB_BCHOL_IDX(I, J)];
        for (k = 0; k < 9; ++k) b[k] += d[k];
    } else {
        /* 上三角ブロックの指定は転置して下三角へ */
        b = m->blk[UWB_BCHOL_IDX(J, I)];
        b[0] += d[0]; b[1] += d[3]; b[2] += d[6];
        b[3] += d[1]; b[4] += d[4]; b[5] += d[7];
        b[6] += d[2]; b[7] += d[5]; b[8] += d[8];
    }
}

void uwb_bchol_damp_diag(uwb_bchol *m, uwb_real lambda, uwb_real ridge)
{
    uwb_real f = R(1) + lambda;
    int I;
    for (I = 0; I < m->nb; ++I) {
        uwb_real *b = m->blk[UWB_BCHOL_IDX(I, I)];
        b[0] = b[0] * f + ridge;
        b[4] = b[4] * f + ridge;
        b[8] = b[8] * f + ridge;
    }
    if (m->has_border) m->bdd = m->bdd * f + ridge;
}

uwb_real uwb_bchol_get(const uwb_bchol *m, int r, int c)
{
    int n3 = 3 * m->nb;
    if (c > r) { int t = r; r = c; c = t; }
    if (r < 0 || c < 0 || r >= uwb_bchol_dim(m)) return R(0);
    if (r == n3) {                                /* 縁取り行 */
        return (c == n3) ? m->bdd : m->brd[c];
    }
    return m->blk[UWB_BCHOL_IDX(r / 3, c / 3)][3 * (r % 3) + (c % 3)];
}

int uwb_bchol_factor(uwb_bchol *m)
{
    int nb = m->nb, I, J, K;

    m->factored = 0;
    for (J = 0; J < nb; ++J) {
        uwb_real *ljj = m->blk[UWB_BCHOL_IDX(J, J)];

        /* 対角: A_JJ − Σ_{K<J} L_JK L_JKᵀ → chol */
        for (K = 0; K < J; ++K) {
            const uwb_real *ljk = m->blk[UWB_BCHOL_IDX(J, K)];
            uwb_m3_gemm_nt_sub(ljj, ljk, ljk);
        }
        if (!uwb_m3_chol(ljj, m->invd[J])) return 0;

        /* 下のブロック: L_IJ = (A_IJ − Σ_{K<J} L_IK L_JKᵀ) L_JJ⁻ᵀ */
        for (I = J + 1; I < nb; ++I) {
            uwb_real *lij = m->blk[UWB_BCHOL_IDX(I, J)];
            for (K = 0; K < J; ++K)
                uwb_m3_gemm_nt_sub(lij, m->blk[UWB_BCHOL_IDX(I, K)], m->blk[UWB_BCHOL_IDX(J, K)]);
            uwb_m3_trsm_rt(lij, ljj, m->invd[J]);
        }

        /* 縁取り行: L_δJ = (a_Jᵀ − Σ_{K<J} L_δK L_JKᵀ) L_JJ⁻ᵀ */
        if (m->has_border) {
            uwb_real *x = m->brd + 3 * J;
            for (K = 0; K < J; ++K) {
                const uwb_real *ldk = m->brd + 3 * K;
                const uwb_real *ljk = m->blk[UWB_BCHOL_IDX(J, K)];
                x[0] -= ldk[0] * ljk[0] + ldk[1] * ljk[1] + ldk[2] * ljk[2];
                x[1] -= ldk[0] * ljk[3] + ldk[1] * ljk[4] + ldk[2] * ljk[5];
                x[2] -= ldk[0] * ljk[6] + ldk[1] * ljk[7] + ldk[2] * ljk[8];
            }
            uwb_m3_trsv_lower(ljj, m->invd[J], x, x);
        }
    }

    if (m->has_border) {
        uwb_real t = m->bdd;
        for (K = 0; K < nb; ++K) t -= uwb_v3_norm2(m->brd + 3 * K);
        if (!(t > R(0))) return 0;
        m->bdd     = uwb_math_sqrt(t);
        m->inv_bdd = R(1) / m->bdd;
    }
    m->factored = 1;
    return 1;
}

int uwb_bchol_solve(const uwb_bchol *m, const uwb_real *b, uwb_real *x)
{
    int nb = m->nb, n = uwb_bchol_dim(m), I, K, k;

    if (!m->factored) return 0;
    if (x != b) for (k = 0; k < n; ++k) x[k] = b[k];

    /* 前進: L y = b */
    for (I = 0; I < nb; ++I) {
        uwb_real *xi = x + 3 * I;
        for (K = 0; K < I; ++K) {
            const uwb_real *l = m->blk[UWB_BCHOL_IDX(I, K)];
            const uwb_real *xk = x + 3 * K;
            xi[0] -= l[0] * xk[0] + l[1] * xk[1] + l[2] * xk[2];
            xi[1] -= l[3] * xk[0] + l[4] * xk[1] + l[5] * xk[2];
            xi[2] -= l[6] * xk[0] + l[7] * xk[1] + l[8] * xk[2];
        }
        uwb_m3_trsv_lower(m->blk[UWB_BCHOL_IDX(I, I)], m->invd[I], xi, xi);
    }
    if (m->has_border) {
        uwb_real t = x[3 * nb];
        for (K = 0; K < nb; ++K) t -= uwb_v3_dot(m->brd + 3 * K, x + 3 * K);
        x[3 * nb] = t * m->inv_bdd;
    }

    /* 後退: Lᵀ x = y */
    if (m->has_border) {
        uwb_real xd = x[3 * nb] * m->inv_bdd;
        x[3 * nb] = xd;
        for (K = 0; K < nb; ++K) uwb_v3_axpy(-xd, m->brd + 3 * K, x + 3 * K);
    }
    for (I = nb - 1; I >= 0; --I) {
        uwb_real *xi = x + 3 * I;
        for (K = I + 1; K < nb; ++K) {
            const uwb_real *l = m->blk[UWB_BCHOL_IDX(K, I)];   /* L_KI, 転置して掛ける */
            const uwb_real *xk = x + 3 * K;
            xi[0] -= l[0] * xk[0] + l[3] * xk[1] + l[6] * xk[2];
            xi[1] -= l[1] * xk[0] + l[4] * xk[1] + l[7] * xk[2];
            xi[2] -= l[2] * xk[0] + l[5] * xk[1] + l[8] * xk[2];
        }
        uwb_m3_trsv_upper(m->blk[UWB_BCHOL_IDX(I, I)], m->invd[I], xi, xi);
    }
    return 1;
}
