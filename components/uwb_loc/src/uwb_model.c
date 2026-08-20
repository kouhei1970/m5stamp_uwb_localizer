/* 観測モデルと共通のヘルパ。Python 版 uwb_loc/model.py に対応する。 */
#include "uwb_internal.h"
#include "uwb_linalg.h"

#include <math.h>
#include <string.h>

const char *uwb_version(void) { return "0.1.0"; }

void uwb_config_init(uwb_config *cfg, const uwb_anchor *anchors, int n_anchors)
{
    if (!cfg) return;
    cfg->anchors      = anchors;
    cfg->n_anchors    = n_anchors;
    cfg->dim          = 3;
    cfg->z_fixed      = (uwb_real)0;
    cfg->use_z_bounds = 0;
    cfg->z_min        = (uwb_real)0;
    cfg->z_max        = (uwb_real)0;
    cfg->max_iter     = 30;
    cfg->tol          = (uwb_real)1e-4;
    /* Huber の 1.345 はガウス誤差で 95% の効率を保つ標準的な値。
     * 片側 0.6 倍は「NLOS は必ず距離を伸ばす」という物理から。 */
    cfg->huber_k      = (uwb_real)1.345;
    cfg->k_pos_scale  = (uwb_real)0.6;
    cfg->one_sided    = 1;
    cfg->chi2_k       = (uwb_real)-1;   /* 負 = レベルごとの既定 */
    cfg->physical_gate= 1;
    cfg->max_range    = (uwb_real)200.0;
}

int uwb_anchor_index(const uwb_config *cfg, const char *id)
{
    int i;
    if (!cfg || !cfg->anchors || !id) return -1;
    for (i = 0; i < cfg->n_anchors; ++i)
        if (strncmp(cfg->anchors[i].id, id, UWB_ID_LEN) == 0) return i;
    return -1;
}

int uwb_meas_usable(const uwb_config *cfg, const uwb_meas *m)
{
    if (m->anchor < 0 || m->anchor >= cfg->n_anchors) return 0;
    if (!cfg->anchors[m->anchor].enabled) return 0;
    if (m->value != m->value) return 0;                 /* NaN */
    return 1;
}

uwb_real uwb_corrected(const uwb_config *cfg, const uwb_meas *m)
{
    return m->value - cfg->anchors[m->anchor].antenna_delay_m;
}

uwb_real uwb_sigma_of(const uwb_config *cfg, const uwb_meas *m, uwb_real distance)
{
    uwb_real s;
    if (m->sigma > (uwb_real)0) {
        s = m->sigma;
    } else {
        const uwb_anchor *a = &cfg->anchors[m->anchor];
        uwb_real s0 = a->sigma0 > (uwb_real)0 ? a->sigma0 : (uwb_real)0.1;
        uwb_real d  = distance > (uwb_real)0 ? distance : (uwb_real)0;
        s = s0 + a->sigma_per_m * d;
    }
    /* 品質値が来ていれば反映する。q=1 でそのまま、q=0 で 4 倍。 */
    if (m->quality >= (uwb_real)0 && m->quality <= (uwb_real)1) {
        s *= (uwb_real)1 + (uwb_real)3 * ((uwb_real)1 - m->quality);
    }
    return s > (uwb_real)1e-6 ? s : (uwb_real)1e-6;
}

void uwb_evaluate(const uwb_config *cfg, const uwb_real *p, const uwb_meas *m,
                  uwb_real *residual, uwb_real *jac3, uwb_real *sigma)
{
    const uwb_anchor *a = &cfg->anchors[m->anchor];
    uwb_real dv[3], d2, d, z;
    int k;

    for (k = 0; k < 3; ++k) dv[k] = p[k] - a->p[k];
    d2 = dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2];
    d  = (uwb_real)sqrt((double)d2);
    z  = uwb_corrected(cfg, m);

    *sigma = uwb_sigma_of(cfg, m, d);
    if (d < UWB_EPS) {
        *residual = (uwb_real)0;
        jac3[0] = jac3[1] = jac3[2] = (uwb_real)0;
        return;
    }
    /* 残差は 観測 - 予測。NLOS で距離が伸びると正になる (片側損失の前提)。 */
    *residual = z - d;
    for (k = 0; k < 3; ++k) jac3[k] = dv[k] / d;
}

int uwb_n_free(const uwb_config *cfg) { return cfg->dim == 2 ? 2 : 3; }

void uwb_project(const uwb_config *cfg, uwb_real *p)
{
    if (cfg->dim == 2) {
        p[2] = cfg->z_fixed;
        return;
    }
    if (cfg->use_z_bounds) {
        if (p[2] < cfg->z_min) p[2] = cfg->z_min;
        if (p[2] > cfg->z_max) p[2] = cfg->z_max;
    }
}

uwb_real uwb_gdop_from_jac(const uwb_config *cfg, const uwb_real *jac, int n)
{
    /* 行を単位化した H から (H^T H)^-1 のトレースの平方根。
     * 「測距の精度が同じでも、置き方でどれだけ位置が暴れるか」を表す。 */
    int nf = uwb_n_free(cfg);
    uwb_real h[UWB_MAX_MEAS * 3];
    uwb_real hth[9], inv[9];
    uwb_real tr = (uwb_real)0;
    int i, k, rows = 0;

    if (n < nf) return (uwb_real)-1;
    for (i = 0; i < n && rows < UWB_MAX_MEAS; ++i) {
        const uwb_real *r = &jac[i * 3];
        uwb_real nrm = (uwb_real)sqrt((double)(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]));
        if (!(nrm > UWB_EPS)) continue;
        for (k = 0; k < nf; ++k) h[rows * nf + k] = r[k] / nrm;
        ++rows;
    }
    if (rows < nf) return (uwb_real)-1;

    uwb_ata_weighted(h, 0, rows, nf, hth);
    if (!uwb_inverse(hth, inv, nf)) return (uwb_real)-1;
    for (k = 0; k < nf; ++k) tr += inv[k * nf + k];
    if (!(tr >= (uwb_real)0)) return (uwb_real)-1;
    return (uwb_real)sqrt((double)tr);
}

/* ------------------------------------------------------------ 鏡像解 */

/* アンカーが同一平面に乗っているかを調べ、乗っていれば法線と offset を返す。
 * 判定は共分散の最小固有値と最大固有値の比 (Python 版の SVD と同じ趣旨)。 */
int uwb_anchors_coplanar(const uwb_config *cfg, uwb_real *normal, uwb_real *offset)
{
    uwb_real center[3] = {0, 0, 0};
    uwb_real cov[9], work[9], eig[3];
    int i, j, k, n = 0;

    if (!cfg || !cfg->anchors) return 0;
    for (i = 0; i < cfg->n_anchors; ++i) {
        if (!cfg->anchors[i].enabled) continue;
        for (k = 0; k < 3; ++k) center[k] += cfg->anchors[i].p[k];
        ++n;
    }
    if (n < 3) return 0;
    for (k = 0; k < 3; ++k) center[k] /= (uwb_real)n;

    for (k = 0; k < 9; ++k) cov[k] = (uwb_real)0;
    for (i = 0; i < cfg->n_anchors; ++i) {
        uwb_real d[3];
        if (!cfg->anchors[i].enabled) continue;
        for (k = 0; k < 3; ++k) d[k] = cfg->anchors[i].p[k] - center[k];
        for (j = 0; j < 3; ++j)
            for (k = 0; k < 3; ++k) cov[j * 3 + k] += d[j] * d[k];
    }

    for (k = 0; k < 9; ++k) work[k] = cov[k];
    if (!uwb_sym_eigvals(work, eig, 3)) return 0;
    if (!(eig[2] > UWB_EPS)) return 0;

    /* 特異値は固有値の平方根。Python 版は sv[-1]/sv[0] < 0.05 で判定する。 */
    {
        uwb_real smin = (uwb_real)sqrt((double)(eig[0] > 0 ? eig[0] : 0));
        uwb_real smax = (uwb_real)sqrt((double)eig[2]);
        if (!(smin / smax < (uwb_real)0.05)) return 0;
    }

    if (normal || offset) {
        /* 最小固有値に対応する固有ベクトル = 平面の法線。
         * 3x3 なので (cov - lambda I) の余因子から直接取れる。 */
        uwb_real a[9], nrm;
        uwb_real lam = eig[0];
        uwb_real nvec[3];
        for (k = 0; k < 9; ++k) a[k] = cov[k];
        a[0] -= lam; a[4] -= lam; a[8] -= lam;
        /* 2 本の行の外積が零空間ベクトル。最も長くなる組を選ぶ。 */
        {
            uwb_real best = (uwb_real)-1;
            int r1, r2;
            nvec[0] = nvec[1] = nvec[2] = (uwb_real)0;
            for (r1 = 0; r1 < 3; ++r1) {
                for (r2 = r1 + 1; r2 < 3; ++r2) {
                    const uwb_real *u = &a[r1 * 3], *v = &a[r2 * 3];
                    uwb_real c[3];
                    uwb_real len;
                    c[0] = u[1] * v[2] - u[2] * v[1];
                    c[1] = u[2] * v[0] - u[0] * v[2];
                    c[2] = u[0] * v[1] - u[1] * v[0];
                    len = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
                    if (len > best) {
                        best = len;
                        for (k = 0; k < 3; ++k) nvec[k] = c[k];
                    }
                }
            }
            nrm = (uwb_real)sqrt((double)(nvec[0] * nvec[0] + nvec[1] * nvec[1] +
                                          nvec[2] * nvec[2]));
            if (!(nrm > UWB_EPS)) return 0;
            for (k = 0; k < 3; ++k) nvec[k] /= nrm;
        }
        if (normal) for (k = 0; k < 3; ++k) normal[k] = nvec[k];
        if (offset) *offset = nvec[0] * center[0] + nvec[1] * center[1] + nvec[2] * center[2];
    }
    return 1;
}

int uwb_mirror_side(const uwb_config *cfg, const uwb_real *p)
{
    uwb_real nrm[3], off, s;
    if (!uwb_anchors_coplanar(cfg, nrm, &off)) return 0;
    s = nrm[0] * p[0] + nrm[1] * p[1] + nrm[2] * p[2] - off;
    return s >= 0 ? 1 : -1;
}

int uwb_resolve_mirror(const uwb_config *cfg, uwb_real *p, uwb_real *cov,
                       int side_known, int side)
{
    uwb_real nrm[3], off, s, mirrored[3];
    int k;

    if (cfg->dim != 3) return 0;
    if (!uwb_anchors_coplanar(cfg, nrm, &off)) return 0;

    s = nrm[0] * p[0] + nrm[1] * p[1] + nrm[2] * p[2] - off;
    for (k = 0; k < 3; ++k) mirrored[k] = p[k] - (uwb_real)2 * s * nrm[k];

    /* 1) z の範囲が与えられていれば、それで選ぶ */
    if (cfg->use_z_bounds) {
        int in_now = (p[2] >= cfg->z_min && p[2] <= cfg->z_max);
        int in_mir = (mirrored[2] >= cfg->z_min && mirrored[2] <= cfg->z_max);
        if (in_now && !in_mir) return 0;
        if (in_mir && !in_now) {
            for (k = 0; k < 3; ++k) p[k] = mirrored[k];
            goto flip_cov;
        }
        return in_now ? 0 : 1;
    }

    /* 2) 前回と同じ側を保つ (連続性)。飛び移らせない方が実害が小さい */
    if (side_known) {
        int now = (s >= 0) ? 1 : -1;
        if (now == side) return 0;
        for (k = 0; k < 3; ++k) p[k] = mirrored[k];
        goto flip_cov;
    }

    /* 3) 手がかりが無い。裏返っている可能性を呼び出し側に伝える */
    return 1;

flip_cov:
    /* 鏡映は直交変換なので共分散は R C R^T で移る (R = I - 2 n n^T)。
     * **鏡映後に解き直さないこと** — 退化した幾何では平面をまたいで戻る。 */
    if (cov) {
        uwb_real r[9], tmp[9], out[9];
        int i, j, m;
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j)
                r[i * 3 + j] = (i == j ? (uwb_real)1 : (uwb_real)0) -
                               (uwb_real)2 * nrm[i] * nrm[j];
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j) {
                uwb_real acc = (uwb_real)0;
                for (m = 0; m < 3; ++m) acc += r[i * 3 + m] * cov[m * 3 + j];
                tmp[i * 3 + j] = acc;
            }
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j) {
                uwb_real acc = (uwb_real)0;
                for (m = 0; m < 3; ++m) acc += tmp[i * 3 + m] * r[j * 3 + m];
                out[i * 3 + j] = acc;
            }
        for (i = 0; i < 9; ++i) cov[i] = out[i];
    }
    return 0;
}

/* ------------------------------------------------------------ fix 補助 */

void uwb_fix_failed(uwb_fix *out, int n_total)
{
    int i;
    for (i = 0; i < 3; ++i) { out->p[i] = (uwb_real)0; out->v[i] = (uwb_real)0; }
    for (i = 0; i < 9; ++i) out->cov[i] = (uwb_real)0;
    out->ok = 0;
    out->n_used = 0;
    out->n_total = n_total;
    out->iterations = 0;
    out->ambiguous = 0;
    out->residual_rms = (uwb_real)-1;
    out->gdop = (uwb_real)-1;
    out->sigma = (uwb_real)-1;
    out->excluded = 0UL;
}

void uwb_fix_finish(uwb_fix *out)
{
    uwb_real tr = out->cov[0] + out->cov[4] + out->cov[8];
    out->sigma = (tr >= (uwb_real)0) ? (uwb_real)sqrt((double)tr) : (uwb_real)-1;
}

/* -------------------------------------------------------- 配置の評価 API */

uwb_real uwb_gdop_at(const uwb_config *cfg, const uwb_real *point)
{
    uwb_real jac[UWB_MAX_MEAS * 3];
    int i, n = 0;
    for (i = 0; i < cfg->n_anchors && n < UWB_MAX_MEAS; ++i) {
        uwb_real dv[3], d;
        int k;
        if (!cfg->anchors[i].enabled) continue;
        for (k = 0; k < 3; ++k) dv[k] = point[k] - cfg->anchors[i].p[k];
        d = (uwb_real)sqrt((double)(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]));
        if (!(d > UWB_EPS)) continue;
        for (k = 0; k < 3; ++k) jac[n * 3 + k] = dv[k] / d;
        ++n;
    }
    return uwb_gdop_from_jac(cfg, jac, n);
}

uwb_real uwb_crlb_at(const uwb_config *cfg, const uwb_real *point)
{
    /* フィッシャー情報 F = sum (1/sigma^2) u u^T。CRLB = sqrt(trace(F^-1))。 */
    int nf = uwb_n_free(cfg);
    uwb_real f[9], inv[9], tr = (uwb_real)0;
    int i, j, k, n = 0;

    for (k = 0; k < 9; ++k) f[k] = (uwb_real)0;
    for (i = 0; i < cfg->n_anchors; ++i) {
        uwb_real dv[3], d, s, w;
        uwb_meas m;
        if (!cfg->anchors[i].enabled) continue;
        for (k = 0; k < 3; ++k) dv[k] = point[k] - cfg->anchors[i].p[k];
        d = (uwb_real)sqrt((double)(dv[0] * dv[0] + dv[1] * dv[1] + dv[2] * dv[2]));
        if (!(d > UWB_EPS)) continue;
        m.anchor = i; m.value = d; m.sigma = (uwb_real)0; m.quality = (uwb_real)-1;
        s = uwb_sigma_of(cfg, &m, d);
        w = (uwb_real)1 / (s * s);
        for (j = 0; j < nf; ++j)
            for (k = 0; k < nf; ++k) f[j * nf + k] += w * (dv[j] / d) * (dv[k] / d);
        ++n;
    }
    if (n < nf) return (uwb_real)-1;
    if (!uwb_inverse(f, inv, nf)) return (uwb_real)-1;
    for (k = 0; k < nf; ++k) tr += inv[k * nf + k];
    if (!(tr >= (uwb_real)0)) return (uwb_real)-1;
    return (uwb_real)sqrt((double)tr);
}
