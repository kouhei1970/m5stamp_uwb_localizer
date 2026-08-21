/* 観測モデルと共通のヘルパ。Python 版 uwb_loc/model.py に対応する。
 *
 * 行列計算は uwb_math (2x2 / 3x3 のスカラー展開) に寄せてあり、
 * 一般の LU や Jacobi はここには無い。 */
#include "uwb_internal.h"

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
    uwb_config_refresh(cfg);
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
    uwb_real dv[3], d, inv, z;

    uwb_v3_sub(p, a->p, dv);
    d = uwb_v3_norm(dv);
    z = uwb_corrected(cfg, m);

    *sigma = uwb_sigma_of(cfg, m, d);
    if (d < UWB_EPS) {
        *residual = (uwb_real)0;
        uwb_v3_zero(jac3);
        return;
    }
    /* 残差は 観測 - 予測。NLOS で距離が伸びると正になる (片側損失の前提)。
     * 単位ベクトルは逆数を 1 回作って掛ける (除算 3 → 1)。 */
    *residual = z - d;
    inv = (uwb_real)1 / d;
    jac3[0] = dv[0] * inv; jac3[1] = dv[1] * inv; jac3[2] = dv[2] * inv;
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

/* ------------------------------------------------------------ GDOP / CRLB */

/* 単位ベクトルの行 (H の行) から tr((HᵀH)⁻¹) の平方根。
 *   3 次元: HᵀH を 6 要素で累積 → 余因子で tr(inv) (除算 1)
 *   2 次元: 単位 3 ベクトルの xy 成分 (再単位化はしない。Python 版と同じ)
 * nf 本未満、または HᵀH が特異 (一直線 / 同一平面の配置で点がその上) なら負。 */
static uwb_real gdop_of_unit_rows(int nf, const uwb_real *rows, int stride, int n)
{
    uwb_real tr;
    int i;
    if (n < nf) return (uwb_real)-1;
    if (nf == 3) {
        uwb_real hth[6];
        uwb_sym3_zero(hth);
        for (i = 0; i < n; ++i) uwb_sym3_add_outer(hth, rows + i * stride);
        if (!uwb_sym3_trace_inverse(hth, &tr)) return (uwb_real)-1;
    } else {
        uwb_real hth[3];
        uwb_sym2_zero(hth);
        for (i = 0; i < n; ++i) uwb_sym2_add_outer(hth, rows + i * stride);
        if (!uwb_sym2_trace_inverse(hth, &tr)) return (uwb_real)-1;
    }
    if (!(tr >= (uwb_real)0)) return (uwb_real)-1;
    return uwb_math_sqrt(tr);
}

uwb_real uwb_gdop_from_jac(const uwb_config *cfg, const uwb_real *jac, int n)
{
    /* 行を単位化した H から (H^T H)^-1 のトレースの平方根。
     * 「測距の精度が同じでも、置き方でどれだけ位置が暴れるか」を表す。
     * uwb_evaluate の行は既に単位ベクトル (距離 0 なら零) なので、
     * 零行だけ飛ばして単位化 (sqrt + 除算 3) はしない。 */
    int nf = uwb_n_free(cfg);
    uwb_real h[UWB_MAX_MEAS * 3];
    int i, rows = 0;

    if (n < nf) return (uwb_real)-1;
    for (i = 0; i < n && rows < UWB_MAX_MEAS; ++i) {
        const uwb_real *r = &jac[i * 3];
        if (!(uwb_v3_norm2(r) > UWB_EPS)) continue;       /* 零行 (距離 0) */
        uwb_v3_copy(r, &h[rows * 3]);
        ++rows;
    }
    return gdop_of_unit_rows(nf, h, 3, rows);
}

/* ------------------------------------------------------------ 鏡像解 */

/* アンカーが同一平面に乗っているかを実際に計算する (キャッシュは見ない)。
 * 判定は中心化した座標の散布行列 (3x3 対称) の最小/最大固有値の比
 * (Python 版の SVD と同じ趣旨: 特異値比 sv_min/sv_max < 0.05
 *  ⟺ 固有値比 < 0.05² = 0.0025。平方根は取らない)。
 * 固有値は閉形式 (uwb_sym3_eigvals)、法線は最小固有値の固有ベクトル
 * (uwb_sym3_min_eigvec)。法線が決まらない (一直線 = 固有値が 2 つ縮退) なら
 * 0 を返す (平面が定義できないので鏡像処理もできない)。 */
static int plane_compute(const uwb_config *cfg, uwb_real *normal, uwb_real *offset)
{
    uwb_real center[3], cov[6], lam[3], nvec[3], lmin;
    int i, n = 0;

    if (!cfg || !cfg->anchors) return 0;
    uwb_v3_zero(center);
    for (i = 0; i < cfg->n_anchors; ++i) {
        if (!cfg->anchors[i].enabled) continue;
        uwb_v3_add(center, cfg->anchors[i].p, center);
        ++n;
    }
    if (n < 3) return 0;
    uwb_v3_scale((uwb_real)1 / (uwb_real)n, center);

    uwb_sym3_zero(cov);
    for (i = 0; i < cfg->n_anchors; ++i) {
        uwb_real d[3];
        if (!cfg->anchors[i].enabled) continue;
        uwb_v3_sub(cfg->anchors[i].p, center, d);
        uwb_sym3_add_outer(cov, d);
    }

    if (!uwb_sym3_eigvals(cov, lam)) return 0;
    if (!(lam[0] > UWB_EPS)) return 0;                 /* 全部同じ点 */
    lmin = lam[2] > (uwb_real)0 ? lam[2] : (uwb_real)0;
    if (!(lmin < (uwb_real)0.0025 * lam[0])) return 0; /* 立体配置 */

    /* 最小固有値に対応する固有ベクトル = 平面の法線 */
    if (!uwb_sym3_min_eigvec(cov, 0, nvec)) return 0;
    if (normal) uwb_v3_copy(nvec, normal);
    if (offset) *offset = uwb_v3_dot(nvec, center);
    return 1;
}

/* キャッシュを作ったときのアンカー配列と今の配列が同じか (enabled と座標を
 * ビット単位で比較。NaN は必ず不一致になり、計算し直しに落ちる)。 */
static int plane_cache_matches(const uwb_config *cfg)
{
    const uwb_plane_cache *c = &cfg->plane;
    int i;
    if (!c->valid || c->n != cfg->n_anchors) return 0;
    for (i = 0; i < c->n; ++i) {
        const uwb_anchor *a = &cfg->anchors[i];
        int en = a->enabled != 0;
        if (en != (int)c->enabled[i]) return 0;
        if (!en) continue;
        if (a->p[0] != c->p[i][0] || a->p[1] != c->p[i][1] || a->p[2] != c->p[i][2]) return 0;
    }
    return 1;
}

void uwb_config_refresh(uwb_config *cfg)
{
    uwb_plane_cache *c;
    int i;

    if (!cfg) return;
    c = &cfg->plane;
    c->valid = 0;
    c->coplanar = 0;
    uwb_v3_zero(c->normal);
    c->offset = (uwb_real)0;
    if (!cfg->anchors || cfg->n_anchors < 0 || cfg->n_anchors > UWB_MAX_ANCHORS) return;

    c->n = cfg->n_anchors;
    for (i = 0; i < c->n; ++i) {
        c->enabled[i] = (unsigned char)(cfg->anchors[i].enabled != 0);
        uwb_v3_copy(cfg->anchors[i].p, c->p[i]);
    }
    c->coplanar = plane_compute(cfg, c->normal, &c->offset);
    if (!c->coplanar) { uwb_v3_zero(c->normal); c->offset = (uwb_real)0; }
    c->valid = 1;
}

int uwb_anchors_coplanar(const uwb_config *cfg, uwb_real *normal, uwb_real *offset)
{
    if (!cfg || !cfg->anchors) return 0;
    if (plane_cache_matches(cfg)) {
        const uwb_plane_cache *c = &cfg->plane;
        if (!c->coplanar) return 0;
        if (normal) uwb_v3_copy(c->normal, normal);
        if (offset) *offset = c->offset;
        return 1;
    }
    /* キャッシュが無い / 合わない (refresh 忘れ、または台数が上限超え) */
    return plane_compute(cfg, normal, offset);
}

int uwb_mirror_side(const uwb_config *cfg, const uwb_real *p)
{
    uwb_real nrm[3], off, s;
    if (!uwb_anchors_coplanar(cfg, nrm, &off)) return 0;
    s = uwb_v3_dot(nrm, p) - off;
    return s >= 0 ? 1 : -1;
}

int uwb_resolve_mirror(const uwb_config *cfg, uwb_real *p, uwb_real *cov,
                       int side_known, int side)
{
    uwb_real nrm[3], off, s, mirrored[3];

    if (cfg->dim != 3) return 0;
    if (!uwb_anchors_coplanar(cfg, nrm, &off)) return 0;

    s = uwb_v3_dot(nrm, p) - off;
    uwb_v3_copy(p, mirrored);
    uwb_v3_axpy((uwb_real)-2 * s, nrm, mirrored);

    /* 1) z の範囲が与えられていれば、それで選ぶ */
    if (cfg->use_z_bounds) {
        int in_now = (p[2] >= cfg->z_min && p[2] <= cfg->z_max);
        int in_mir = (mirrored[2] >= cfg->z_min && mirrored[2] <= cfg->z_max);
        if (in_now && !in_mir) return 0;
        if (in_mir && !in_now) {
            uwb_v3_copy(mirrored, p);
            goto flip_cov;
        }
        return in_now ? 0 : 1;
    }

    /* 2) 前回と同じ側を保つ (連続性)。飛び移らせない方が実害が小さい */
    if (side_known) {
        int now = (s >= 0) ? 1 : -1;
        if (now == side) return 0;
        uwb_v3_copy(mirrored, p);
        goto flip_cov;
    }

    /* 3) 手がかりが無い。裏返っている可能性を呼び出し側に伝える */
    return 1;

flip_cov:
    /* 鏡映は直交変換なので共分散は R C R^T で移る (R = I - 2 n n^T)。
     * 展開形 C − 2n(Cn)ᵀ − 2(Cn)nᵀ + 4(nᵀCn)nnᵀ (uwb_sym3_reflect)。
     * **鏡映後に解き直さないこと** — 退化した幾何では平面をまたいで戻る。 */
    if (cov) {
        uwb_real s6[6];
        uwb_sym3_from_full(cov, s6);
        uwb_sym3_reflect(s6, nrm);
        uwb_sym3_to_full(s6, cov);
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
    out->sigma = (tr >= (uwb_real)0) ? uwb_math_sqrt(tr) : (uwb_real)-1;
}

/* -------------------------------------------------------- 配置の評価 API */

uwb_real uwb_gdop_at(const uwb_config *cfg, const uwb_real *point)
{
    uwb_real jac[UWB_MAX_MEAS * 3];
    int i, n = 0;
    for (i = 0; i < cfg->n_anchors && n < UWB_MAX_MEAS; ++i) {
        uwb_real *u = &jac[n * 3];
        if (!cfg->anchors[i].enabled) continue;
        uwb_v3_sub(point, cfg->anchors[i].p, u);
        if (!(uwb_v3_normalize(u) > UWB_EPS)) continue;
        ++n;
    }
    return gdop_of_unit_rows(uwb_n_free(cfg), jac, 3, n);
}

uwb_real uwb_crlb_at(const uwb_config *cfg, const uwb_real *point)
{
    /* フィッシャー情報 F = sum (1/sigma^2) u u^T。CRLB = sqrt(trace(F^-1))。 */
    int nf = uwb_n_free(cfg);
    uwb_real f3[6], f2[3], tr;
    int i, n = 0;

    uwb_sym3_zero(f3);
    uwb_sym2_zero(f2);
    for (i = 0; i < cfg->n_anchors; ++i) {
        uwb_real u[3], d, s, w;
        uwb_meas m;
        if (!cfg->anchors[i].enabled) continue;
        uwb_v3_sub(point, cfg->anchors[i].p, u);
        d = uwb_v3_normalize(u);
        if (!(d > UWB_EPS)) continue;
        m.anchor = i; m.value = d; m.sigma = (uwb_real)0; m.quality = (uwb_real)-1;
        s = uwb_sigma_of(cfg, &m, d);
        w = (uwb_real)1 / (s * s);
        if (nf == 3) uwb_sym3_add_scaled_outer(f3, w, u);
        else         uwb_sym2_add_scaled_outer(f2, w, u);
        ++n;
    }
    if (n < nf) return (uwb_real)-1;
    if (nf == 3) { if (!uwb_sym3_trace_inverse(f3, &tr)) return (uwb_real)-1; }
    else         { if (!uwb_sym2_trace_inverse(f2, &tr)) return (uwb_real)-1; }
    if (!(tr >= (uwb_real)0)) return (uwb_real)-1;
    return uwb_math_sqrt(tr);
}
