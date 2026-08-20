/* 閉形式ソルバ — 反復も初期値も要らない解。
 * Python 版 uwb_loc/solvers/closed_form.py に対応する。 */
#include "uwb_internal.h"
#include "uwb_linalg.h"

#include <math.h>

/* 2 次元で解くときは、アンカーの高さの差を測距値から抜いて水平距離にする。 */
static void collect(const uwb_config *cfg, const uwb_meas *meas, int n,
                    uwb_real *pos, uwb_real *rng, uwb_real *w, int *out_n, int *d)
{
    int i, k = 0;
    *d = cfg->dim;
    for (i = 0; i < n && k < UWB_MAX_MEAS; ++i) {
        uwb_real r, s;
        uwb_meas mm;
        if (!uwb_meas_usable(cfg, &meas[i])) continue;
        r = uwb_corrected(cfg, &meas[i]);
        if (!(r > (uwb_real)0)) continue;
        mm = meas[i];
        s = uwb_sigma_of(cfg, &mm, r);

        if (*d == 2) {
            uwb_real dz = cfg->z_fixed - cfg->anchors[meas[i].anchor].p[2];
            uwb_real h2 = r * r - dz * dz;
            r = (uwb_real)sqrt((double)(h2 > (uwb_real)1e-4 ? h2 : (uwb_real)1e-4));
            pos[k * 2 + 0] = cfg->anchors[meas[i].anchor].p[0];
            pos[k * 2 + 1] = cfg->anchors[meas[i].anchor].p[1];
        } else {
            pos[k * 3 + 0] = cfg->anchors[meas[i].anchor].p[0];
            pos[k * 3 + 1] = cfg->anchors[meas[i].anchor].p[1];
            pos[k * 3 + 2] = cfg->anchors[meas[i].anchor].p[2];
        }
        rng[k] = r;
        w[k] = (uwb_real)1 / (s * s);
        ++k;
    }
    *out_n = k;
}

int uwb_lls_trilateration(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real amat[UWB_MAX_MEAS * 3], b[UWB_MAX_MEAS];
    uwb_real ata[9], atb[3], sol[3];
    int m = 0, d = 3, i, k, ref = 0, rows = 0;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;

    /* 基準は測距値が最小のもの。ふつう一番 S/N が良い。 */
    for (i = 1; i < m; ++i) if (rng[i] < rng[ref]) ref = i;

    {
        const uwb_real *aref = &pos[ref * d];
        uwb_real rref = rng[ref];
        uwb_real aa_ref = (uwb_real)0;
        for (k = 0; k < d; ++k) aa_ref += aref[k] * aref[k];

        for (i = 0; i < m; ++i) {
            const uwb_real *ai;
            uwb_real aa = (uwb_real)0, sw;
            if (i == ref) continue;
            ai = &pos[i * d];
            for (k = 0; k < d; ++k) aa += ai[k] * ai[k];
            /* ||p||^2 が差分で消えるので線形になる */
            for (k = 0; k < d; ++k) amat[rows * d + k] = (uwb_real)2 * (aref[k] - ai[k]);
            b[rows] = rng[i] * rng[i] - rref * rref - aa + aa_ref;
            /* 差分をとった行の重みは 2 本の合成。調和平均で近似する。 */
            sw = (uwb_real)sqrt((double)(w[i] * w[ref] / (w[i] + w[ref])));
            for (k = 0; k < d; ++k) amat[rows * d + k] *= sw;
            b[rows] *= sw;
            ++rows;
        }
    }
    if (rows < d) return 0;

    uwb_ata_weighted(amat, 0, rows, d, ata);
    uwb_atb_weighted(amat, 0, b, rows, d, atb);
    /* アンカーが同一平面 (または一直線) だと A^T A が階数落ちする。
     * numpy の lstsq はそのとき最小ノルム解を返すので、こちらも
     * ごく小さいリッジを入れて同じ振る舞いに寄せる。健全な配置では
     * 影響しない大きさ (対角平均の 1e-12 倍)。 */
    {
        uwb_real tr = (uwb_real)0;
        int kk;
        for (kk = 0; kk < d; ++kk) tr += ata[kk * d + kk];
        if (tr > (uwb_real)0) {
            uwb_real ridge = (tr / (uwb_real)d) * (uwb_real)1e-12;
            for (kk = 0; kk < d; ++kk) ata[kk * d + kk] += ridge;
        }
    }
    if (!uwb_solve_lin(ata, atb, sol, d)) return 0;

    for (k = 0; k < d; ++k) {
        if (sol[k] != sol[k]) return 0;
        out[k] = sol[k];
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}

/* ------------------------------------------------------------- Beck GTRS */

typedef struct {
    uwb_real g[16];     /* (d+1)x(d+1) */
    uwb_real h[4];
    int      d, nq;
} beck_ctx;

/* y = (G + lam D)^-1 (h - lam f) を解く。D = diag(1..1,0)、f = (0..0,-0.5)。 */
static int beck_solve_y(const beck_ctx *c, uwb_real lam, uwb_real *y)
{
    uwb_real a[16], rhs[4];
    int i, k;
    for (i = 0; i < c->nq * c->nq; ++i) a[i] = c->g[i];
    for (k = 0; k < c->d; ++k) a[k * c->nq + k] += lam;
    for (k = 0; k < c->nq; ++k) rhs[k] = c->h[k];
    rhs[c->d] += (uwb_real)0.5 * lam;      /* -lam * f, f[d] = -0.5 */
    return uwb_solve_lin(a, rhs, y, c->nq);
}

/* phi(lam) = y^T D y + 2 f^T y。単調減少、根が求める lam。 */
static int beck_phi(const beck_ctx *c, uwb_real lam, uwb_real *phi)
{
    uwb_real y[4];
    uwb_real v = (uwb_real)0;
    int k;
    if (!beck_solve_y(c, lam, y)) return 0;
    for (k = 0; k < c->d; ++k) v += y[k] * y[k];
    v -= y[c->d];                            /* 2 * f^T y = -y[d] */
    if (v != v) return 0;
    *phi = v;
    return 1;
}

int uwb_beck_gtrs(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real amat[UWB_MAX_MEAS * 4], b[UWB_MAX_MEAS];
    beck_ctx c;
    uwb_real lam_lo, lo, hi, span, phi, y[4];
    int m = 0, d = 3, i, k, it;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;
    c.d = d;
    c.nq = d + 1;

    /* A y = b,  y = [p; ||p||^2] */
    for (i = 0; i < m; ++i) {
        for (k = 0; k < d; ++k) amat[i * c.nq + k] = (uwb_real)-2 * pos[i * d + k];
        amat[i * c.nq + d] = (uwb_real)1;
        {
            uwb_real aa = (uwb_real)0;
            for (k = 0; k < d; ++k) aa += pos[i * d + k] * pos[i * d + k];
            b[i] = rng[i] * rng[i] - aa;
        }
    }
    uwb_ata_weighted(amat, w, m, c.nq, c.g);
    uwb_atb_weighted(amat, w, b, m, c.nq, c.h);

    /* lam の下限は一般化固有値 (D, G) の最大値の逆数の符号反転。
     * G = L L^T として S = L^-1 D L^-T は対称なので、その固有値を
     * Jacobi 法で取れる (G^-1 D と同じ固有値)。 */
    {
        uwb_real l[16], s[16], eig[4], gamma;
        uwb_real inv_l[16];
        int r, col;
        /* アンカーが同一平面に乗っていると G はちょうど特異になる
         * ([x y z 1] の列に線形関係が立つため)。ここで解けないのは
         * 「その配置では Beck は使えない」という正しい合図なので、
         * 素直に失敗を返して呼び出し側 (LLS) に任せる。
         * Python 版の beck_gtrs も同じ条件で None を返す。 */
        for (i = 0; i < c.nq * c.nq; ++i) l[i] = c.g[i];
        if (!uwb_cholesky(l, c.nq)) return 0;

        /* L^-1 を前進代入で作る (下三角) */
        for (i = 0; i < c.nq * c.nq; ++i) inv_l[i] = (uwb_real)0;
        for (col = 0; col < c.nq; ++col) {
            inv_l[col * c.nq + col] = (uwb_real)1 / l[col * c.nq + col];
            for (r = col + 1; r < c.nq; ++r) {
                uwb_real acc = (uwb_real)0;
                for (k = col; k < r; ++k) acc += l[r * c.nq + k] * inv_l[k * c.nq + col];
                inv_l[r * c.nq + col] = -acc / l[r * c.nq + r];
            }
        }
        /* S = L^-1 D L^-T。D = diag(1..1, 0) なので最後の列を落とすだけ。 */
        for (r = 0; r < c.nq; ++r) {
            for (col = 0; col < c.nq; ++col) {
                uwb_real acc = (uwb_real)0;
                for (k = 0; k < d; ++k)          /* D の 1 が並ぶ範囲だけ */
                    acc += inv_l[r * c.nq + k] * inv_l[col * c.nq + k];
                s[r * c.nq + col] = acc;
            }
        }
        if (!uwb_sym_eigvals(s, eig, c.nq)) return 0;
        gamma = eig[c.nq - 1];               /* 昇順なので最後が最大 */
        if (!(gamma > UWB_EPS)) return 0;
        lam_lo = (uwb_real)-1 / gamma;
    }

    /* 開区間なので下限のわずかに内側から始める。phi(lam_lo+) は +∞。 */
    span = lam_lo < 0 ? -lam_lo : lam_lo;
    if (span < (uwb_real)1) span = (uwb_real)1;
    lo = lam_lo + (uwb_real)1e-9 * span;
    for (it = 0; it < 60; ++it) {
        if (beck_phi(&c, lo, &phi) && phi > (uwb_real)0) break;
        lo = lam_lo + (lo - lam_lo) * (uwb_real)10;
    }
    if (it == 60) return 0;

    hi = lo + span;
    for (it = 0; it < 200; ++it) {
        if (beck_phi(&c, hi, &phi) && phi < (uwb_real)0) break;
        hi = lo + (hi - lo) * (uwb_real)2;
    }
    if (it == 200) return 0;

    for (it = 0; it < 200; ++it) {
        uwb_real mid = (uwb_real)0.5 * (lo + hi);
        if (!beck_phi(&c, mid, &phi)) return 0;
        if (phi > (uwb_real)0) lo = mid; else hi = mid;
        if (hi - lo < (uwb_real)1e-14 * (hi > (uwb_real)1 ? hi : (uwb_real)1)) break;
    }

    if (!beck_solve_y(&c, (uwb_real)0.5 * (lo + hi), y)) return 0;
    for (k = 0; k < d; ++k) {
        if (y[k] != y[k]) return 0;
        out[k] = y[k];
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}
