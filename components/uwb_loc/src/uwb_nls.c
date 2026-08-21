/* 重み付き非線形最小二乗とロバスト化。
 * Python 版 uwb_loc/solvers/nls.py + robust.py に対応する。 */
#include "uwb_internal.h"
#include "uwb_linalg.h"

#include <math.h>

static uwb_real absr(uwb_real v) { return v < 0 ? -v : v; }

/* ------------------------------------------------------------ ロバスト重み */

/* Huber の重み倍率。片側損失なら正の残差 (= 距離が伸びた側) だけ厳しくする。 */
static uwb_real huber_weight(const uwb_config *cfg, uwb_real e, uwb_real sigma)
{
    uwb_real u, k, au;
    if (!(cfg->huber_k > (uwb_real)0)) return (uwb_real)1;
    u = e / (sigma > (uwb_real)1e-6 ? sigma : (uwb_real)1e-6);
    k = cfg->huber_k;
    if (cfg->one_sided && e > (uwb_real)0) k *= cfg->k_pos_scale;
    au = absr(u);
    if (au <= k) return (uwb_real)1;
    return k / (au > (uwb_real)1e-9 ? au : (uwb_real)1e-9);
}

/* ------------------------------------------------------------ 作業データ */

typedef struct {
    const uwb_config *cfg;
    const uwb_meas   *meas;      /* 呼び出し側の配列 */
    int               idx[UWB_MAX_MEAS];  /* 使う観測の添字 */
    int               n;
} nls_set;

typedef struct {
    uwb_real p[3];
    uwb_real cov[9];
    uwb_real resid[UWB_MAX_MEAS];
    uwb_real jac[UWB_MAX_MEAS * 3];
    uwb_real sigma[UWB_MAX_MEAS];
    uwb_real w[UWB_MAX_MEAS];
    uwb_real residual_rms;
    uwb_real gdop;
    int      iterations;
    int      ok;
} nls_result;

/* すべての観測を評価して残差・ヤコビアン・重みを埋め、コストを返す。 */
static uwb_real assemble(const nls_set *s, const uwb_real *p, int robust,
                         uwb_real *resid, uwb_real *jac, uwb_real *sigma, uwb_real *w)
{
    uwb_real cost = (uwb_real)0;
    int i;
    for (i = 0; i < s->n; ++i) {
        uwb_real e, j3[3], sg, wi;
        uwb_evaluate(s->cfg, p, &s->meas[s->idx[i]], &e, j3, &sg);
        resid[i] = e;
        jac[i * 3 + 0] = j3[0]; jac[i * 3 + 1] = j3[1]; jac[i * 3 + 2] = j3[2];
        sigma[i] = sg;
        wi = (uwb_real)1 / (sg * sg);
        if (robust) wi *= huber_weight(s->cfg, e, sg);
        w[i] = wi;
        cost += wi * e * e;
    }
    return cost;
}

/* Gauss-Newton + Levenberg-Marquardt 減衰。
 * LM を入れてあるのは、アンカーが一直線・タグが平面上といった退化配置でも
 * 発散せずに止まるようにするため。 */
static void solve_nls(const nls_set *s, const uwb_real *p0, int robust, nls_result *r)
{
    const uwb_config *cfg = s->cfg;
    int nf = uwb_n_free(cfg);
    uwb_real lam = (uwb_real)1e-6;
    uwb_real cost;
    int it, k;

    for (k = 0; k < 3; ++k) r->p[k] = p0[k];
    uwb_project(cfg, r->p);
    r->iterations = 0;
    r->ok = 0;

    if (s->n < nf) {
        r->residual_rms = (uwb_real)-1;
        r->gdop = (uwb_real)-1;
        return;
    }

    cost = assemble(s, r->p, robust, r->resid, r->jac, r->sigma, r->w);

    for (it = 1; it <= cfg->max_iter; ++it) {
        uwb_real jm[UWB_MAX_MEAS * 3], hmat[9], grad[3], step[3];
        uwb_real cand[3], c_new, trace = (uwb_real)0, damp;
        uwb_real r2[UWB_MAX_MEAS], j2[UWB_MAX_MEAS * 3], s2[UWB_MAX_MEAS], w2[UWB_MAX_MEAS];
        int i;

        r->iterations = it;
        for (i = 0; i < s->n; ++i)
            for (k = 0; k < nf; ++k) jm[i * nf + k] = r->jac[i * 3 + k];

        uwb_ata_weighted(jm, r->w, s->n, nf, hmat);
        uwb_atb_weighted(jm, r->w, r->resid, s->n, nf, grad);

        for (k = 0; k < nf; ++k) trace += hmat[k * nf + k];
        damp = trace / (uwb_real)nf;
        if (!(damp > (uwb_real)1e-9)) damp = (uwb_real)1e-9;
        for (k = 0; k < nf; ++k) hmat[k * nf + k] += lam * damp;

        if (!uwb_solve_lin(hmat, grad, step, nf)) break;

        for (k = 0; k < 3; ++k) cand[k] = r->p[k];
        for (k = 0; k < nf; ++k) cand[k] += step[k];
        uwb_project(cfg, cand);

        c_new = assemble(s, cand, robust, r2, j2, s2, w2);

        if (c_new <= cost) {
            uwb_real nrm = (uwb_real)0;
            for (k = 0; k < 3; ++k) r->p[k] = cand[k];
            cost = c_new;
            for (i = 0; i < s->n; ++i) {
                r->resid[i] = r2[i]; r->sigma[i] = s2[i]; r->w[i] = w2[i];
                for (k = 0; k < 3; ++k) r->jac[i * 3 + k] = j2[i * 3 + k];
            }
            lam *= (uwb_real)0.3;
            if (lam < (uwb_real)1e-9) lam = (uwb_real)1e-9;
            for (k = 0; k < nf; ++k) nrm += step[k] * step[k];
            if ((uwb_real)sqrt((double)nrm) < cfg->tol) break;
        } else {
            lam *= (uwb_real)4;
            if (lam > (uwb_real)1e8) break;
        }
    }

    /* 共分散 = (J^T W J)^-1。解けなければ ok=0。 */
    {
        uwb_real jm[UWB_MAX_MEAS * 3], hmat[9], inv[9];
        uwb_real wsum = (uwb_real)0, num = (uwb_real)0;
        int i;
        for (i = 0; i < s->n; ++i)
            for (k = 0; k < nf; ++k) jm[i * nf + k] = r->jac[i * 3 + k];
        uwb_ata_weighted(jm, r->w, s->n, nf, hmat);

        for (k = 0; k < 9; ++k) r->cov[k] = (uwb_real)0;
        r->ok = uwb_inverse(hmat, inv, nf);
        if (r->ok) {
            int a, b;
            for (a = 0; a < nf; ++a)
                for (b = 0; b < nf; ++b) r->cov[a * 3 + b] = inv[a * nf + b];
        }
        for (i = 0; i < s->n; ++i) { wsum += r->w[i]; num += r->w[i] * r->resid[i] * r->resid[i]; }
        r->residual_rms = wsum > (uwb_real)0
                              ? (uwb_real)sqrt((double)(num / wsum)) : (uwb_real)-1;
        r->gdop = uwb_gdop_from_jac(cfg, r->jac, s->n);
        for (k = 0; k < 3; ++k) if (r->p[k] != r->p[k]) r->ok = 0;
    }
}

/* ------------------------------------------------------------ 物理ゲート */

/* 物理的にありえない観測を落とす。解く前にできるので、ほぼ無料で効く。
 *
 * 三角不等式: アンカー間の距離が分かっているので、2 本の測距値の和が
 * それより短い / 差がそれより長い、という組は誤差を見てもありえない。
 *
 * ただし **どちらが悪いかは 1 組だけでは決まらない**ので、Python 版と同じく
 * 「何本と矛盾したか」を数え、過半と矛盾したものだけ落とす。1 本ずつの
 * 小競り合いで巻き添えにしない。 */
static void physical_gate(const uwb_config *cfg, const uwb_meas *meas, int n,
                          nls_set *set, unsigned long *excluded)
{
    int keep[UWB_MAX_MEAS];
    int rng[UWB_MAX_MEAS];
    int conflicts[UWB_MAX_MEAS];
    int i, j, m = 0, nr = 0;
    const uwb_real slack = (uwb_real)1.0;

    for (i = 0; i < n && nr < UWB_MAX_MEAS; ++i) {
        if (!uwb_meas_usable(cfg, &meas[i])) continue;
        if (cfg->physical_gate) {
            /* 生の測距値で見る (Python 版と同じ)。わずかな負値は許す —
             * アンテナ遅延の取り残しで 0 付近が負に出ることがあるため。 */
            uwb_real v = meas[i].value;
            if (v < -slack || v > cfg->max_range) {
                if (i < 32) *excluded |= (1UL << i);
                continue;
            }
        }
        rng[nr++] = i;
    }

    if (cfg->physical_gate && nr >= 3) {
        int limit = (nr - 1) / 2;
        if (limit < 1) limit = 1;
        for (i = 0; i < nr; ++i) conflicts[i] = 0;
        for (i = 0; i < nr; ++i) {
            const uwb_anchor *ai = &cfg->anchors[meas[rng[i]].anchor];
            for (j = i + 1; j < nr; ++j) {
                const uwb_anchor *aj = &cfg->anchors[meas[rng[j]].anchor];
                uwb_real dv[3], dij, ri, rj;
                int k;
                for (k = 0; k < 3; ++k) dv[k] = ai->p[k] - aj->p[k];
                dij = (uwb_real)sqrt((double)(dv[0]*dv[0] + dv[1]*dv[1] + dv[2]*dv[2]));
                ri = meas[rng[i]].value;
                rj = meas[rng[j]].value;
                if (ri + rj + slack < dij || absr(ri - rj) - slack > dij) {
                    ++conflicts[i];
                    ++conflicts[j];
                }
            }
        }
        for (i = 0; i < nr; ++i) {
            if (conflicts[i] > limit) {
                if (rng[i] < 32) *excluded |= (1UL << rng[i]);
                continue;
            }
            keep[m++] = rng[i];
        }
    } else {
        for (i = 0; i < nr; ++i) keep[m++] = rng[i];
    }

    set->n = m;
    for (i = 0; i < m; ++i) set->idx[i] = keep[i];
}

/* ------------------------------------------------------------ 初期値 */

static int initial_guess(const uwb_config *cfg, const nls_set *set,
                         const uwb_meas *meas, uwb_real *p0)
{
    uwb_meas sub[UWB_MAX_MEAS];
    int i;
    for (i = 0; i < set->n; ++i) sub[i] = meas[set->idx[i]];

    if (uwb_beck_gtrs(cfg, sub, set->n, p0)) return 1;
    if (uwb_lls_trilateration(cfg, sub, set->n, p0)) return 1;

    /* 最後の手段: 使えるアンカーの重心 */
    {
        uwb_real c[3] = {0, 0, 0};
        int k, cnt = 0;
        for (i = 0; i < set->n; ++i) {
            const uwb_anchor *a = &cfg->anchors[meas[set->idx[i]].anchor];
            for (k = 0; k < 3; ++k) c[k] += a->p[k];
            ++cnt;
        }
        if (cnt == 0) return 0;
        for (k = 0; k < 3; ++k) p0[k] = c[k] / (uwb_real)cnt;
        uwb_project(cfg, p0);
        return 1;
    }
}

/* --------------------------------------------------------- 共通の仕上げ */

static void fill_fix(const uwb_config *cfg, const nls_result *r, const nls_set *set,
                     int n_total, unsigned long excluded, uwb_fix *out)
{
    int k;
    for (k = 0; k < 3; ++k) { out->p[k] = r->p[k]; out->v[k] = (uwb_real)0; }
    for (k = 0; k < 9; ++k) out->cov[k] = r->cov[k];
    out->ok = r->ok;
    out->n_used = set->n;
    out->n_total = n_total;
    out->iterations = r->iterations;
    out->residual_rms = r->residual_rms;
    out->gdop = r->gdop;
    out->excluded = excluded;
    out->ambiguous = uwb_resolve_mirror(cfg, out->p, out->cov, 0, 0);
    uwb_fix_finish(out);
}

/* chi2 ゲート: 解いてから外れ値を落とし、一度だけ解き直す。 */
/* chi2 ゲートのしきい値。設定が負なら Python 版と同じレベル別の既定を使う
 * (Lv1 は 3.5、Lv2 は 4.0)。Lv2 の方が緩いのは、ロバスト損失が先に効くので
 * ゲートで切りすぎない方が良いため。 */
static uwb_real chi2_threshold(const uwb_config *cfg, int robust)
{
    if (cfg->chi2_k >= (uwb_real)0) return cfg->chi2_k;
    return robust ? (uwb_real)4.0 : (uwb_real)3.5;
}

static void solve_gated(const uwb_config *cfg, const uwb_meas *meas, nls_set *set,
                        const uwb_real *p0, int robust,
                        nls_result *r, unsigned long *excluded)
{
    int nf = uwb_n_free(cfg);
    uwb_real k_gate = chi2_threshold(cfg, robust);
    solve_nls(set, p0, robust, r);

    if (k_gate > (uwb_real)0 && set->n > nf + 1) {
        int keep[UWB_MAX_MEAS], m = 0, i, bad = 0;
        for (i = 0; i < set->n; ++i) {
            uwb_real e, j3[3], sg;
            uwb_evaluate(cfg, r->p, &meas[set->idx[i]], &e, j3, &sg);
            if (absr(e) > k_gate * (sg > (uwb_real)1e-6 ? sg : (uwb_real)1e-6)) {
                bad = 1;
                continue;
            }
            keep[m++] = set->idx[i];
        }
        if (bad && m >= nf + 1) {
            for (i = 0; i < set->n; ++i) {
                int j, found = 0;
                for (j = 0; j < m; ++j) if (keep[j] == set->idx[i]) { found = 1; break; }
                if (!found && set->idx[i] < 32) *excluded |= (1UL << set->idx[i]);
            }
            set->n = m;
            for (i = 0; i < m; ++i) set->idx[i] = keep[i];
            solve_nls(set, r->p, robust, r);
        }
    }
}

/* ------------------------------------------------------------------ Lv0 */

int uwb_solve_lv0(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out)
{
    nls_set set;
    uwb_meas sub[UWB_MAX_MEAS];
    uwb_real p[3];
    unsigned long excluded = 0UL;
    int nf, i, k;

    uwb_fix_failed(out, n);
    if (!cfg || !cfg->anchors || !meas || n <= 0) return 0;
    nf = uwb_n_free(cfg);

    set.cfg = cfg; set.meas = meas; set.n = 0;
    for (i = 0; i < n && set.n < UWB_MAX_MEAS; ++i)
        if (uwb_meas_usable(cfg, &meas[i])) set.idx[set.n++] = i;
    if (set.n < nf + 1) return 0;

    for (i = 0; i < set.n; ++i) sub[i] = meas[set.idx[i]];
    if (!uwb_lls_trilateration(cfg, sub, set.n, p)) return 0;
    uwb_project(cfg, p);

    /* Lv0 は反復しないので、残差と共分散だけ評価して返す */
    {
        nls_result r;
        uwb_real num = (uwb_real)0;
        uwb_real jm[UWB_MAX_MEAS * 3], hmat[9], inv[9];
        for (k = 0; k < 3; ++k) r.p[k] = p[k];
        for (i = 0; i < set.n; ++i) {
            uwb_real e, j3[3], sg;
            uwb_evaluate(cfg, p, &meas[set.idx[i]], &e, j3, &sg);
            r.resid[i] = e; r.sigma[i] = sg;
            for (k = 0; k < 3; ++k) r.jac[i * 3 + k] = j3[k];
            r.w[i] = (uwb_real)1 / (sg * sg);
            num += e * e;
        }
        for (i = 0; i < set.n; ++i)
            for (k = 0; k < nf; ++k) jm[i * nf + k] = r.jac[i * 3 + k];
        uwb_ata_weighted(jm, r.w, set.n, nf, hmat);
        for (k = 0; k < 9; ++k) r.cov[k] = (uwb_real)0;
        r.ok = uwb_inverse(hmat, inv, nf);
        if (r.ok) {
            int a, b;
            for (a = 0; a < nf; ++a)
                for (b = 0; b < nf; ++b) r.cov[a * 3 + b] = inv[a * nf + b];
        }
        /* Lv0 の残差 RMS は Python 版と同じく重み無しの RMS */
        r.residual_rms = (uwb_real)sqrt((double)(num / (uwb_real)set.n));
        r.gdop = uwb_gdop_from_jac(cfg, r.jac, set.n);
        r.iterations = 0;
        r.ok = 1;
        fill_fix(cfg, &r, &set, n, excluded, out);
    }
    return out->ok;
}

/* -------------------------------------------------------------- Lv1 / Lv2 */

static int solve_iterative(const uwb_config *cfg, const uwb_meas *meas, int n,
                           int robust, uwb_fix *out)
{
    nls_set set;
    nls_result r;
    uwb_real p0[3];
    unsigned long excluded = 0UL;
    int nf;

    uwb_fix_failed(out, n);
    if (!cfg || !cfg->anchors || !meas || n <= 0) return 0;
    nf = uwb_n_free(cfg);

    set.cfg = cfg; set.meas = meas;
    physical_gate(cfg, meas, n, &set, &excluded);
    if (set.n < nf + 1) return 0;

    if (!initial_guess(cfg, &set, meas, p0)) return 0;

    solve_gated(cfg, meas, &set, p0, robust, &r, &excluded);

    if (!r.ok) {
        /* 初期値から作り直して一度だけやり直す */
        if (!initial_guess(cfg, &set, meas, p0)) return 0;
        solve_nls(&set, p0, robust, &r);
        if (!r.ok) return 0;
    }

    fill_fix(cfg, &r, &set, n, excluded, out);
    return out->ok;
}

int uwb_solve_lv1(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out)
{
    return solve_iterative(cfg, meas, n, 0, out);
}

int uwb_solve_lv2(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out)
{
    return solve_iterative(cfg, meas, n, 1, out);
}
