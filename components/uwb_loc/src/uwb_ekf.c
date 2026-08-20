/* 密結合 EKF (Lv3)。Python 版 uwb_loc/solvers/ekf.py に対応する。
 *
 * 「密結合」= 位置に直したものではなく**測距値そのもの**で更新する。
 * だから 1 本しか届かないエポックでも情報を使える (立ち上げだけは
 * 球面 1 枚では位置が決まらないので dim+2 本たまるのを待つ)。
 *
 * 更新はスカラー逐次。S がスカラーなので**行列の逆行列が要らない**。
 */
#include "uwb_internal.h"
#include "uwb_linalg.h"

#include <math.h>

static uwb_real absr(uwb_real v) { return v < 0 ? -v : v; }

void uwb_ekf_init(uwb_ekf *e, const uwb_config *cfg, uwb_motion motion, uwb_real sigma_a)
{
    e->cfg = cfg;
    e->motion = motion;
    e->sigma_a = sigma_a > (uwb_real)0 ? sigma_a : (uwb_real)1;
    e->gate = (uwb_real)3;
    e->max_dt = (uwb_real)2;
    e->max_rejects = 5;
    e->nd = cfg->dim;
    e->norder = (motion == UWB_MOTION_CV) ? 2 : 3;
    e->nx = e->nd * e->norder;
    uwb_ekf_reset(e);
}

void uwb_ekf_reset(uwb_ekf *e)
{
    int i;
    for (i = 0; i < UWB_MAX_STATE; ++i) e->x[i] = (uwb_real)0;
    for (i = 0; i < UWB_MAX_STATE * UWB_MAX_STATE; ++i) e->P[i] = (uwb_real)0;
    for (i = 0; i < e->nx; ++i) e->P[i * e->nx + i] = (uwb_real)1e6;
    e->t = (uwb_real)0;
    e->has_t = 0;
    e->initialized = 0;
    e->rejects = 0;
    e->ambiguous = 0;
    e->n_pending = 0;
    e->side_known = 0;
    e->side = 0;
}

/* 状態遷移とプロセスノイズ。
 * 軸ごとに独立とみなし、1 軸分の小行列をクロネッカー積で広げる。
 * 遷移は積分器の連鎖なので F[i][j] = dt^(j-i)/(j-i)!。
 *
 * プロセスノイズは **最上位の微分に連続時間の白色雑音が乗る**モデルで
 * 統一してある (CV なら加速度、CA なら加加速度)。離散版と混ぜると
 * sigma_a の意味がモードによって変わってしまう。 */
static void transition(const uwb_ekf *e, uwb_real dt, uwb_real *f, uwb_real *q)
{
    uwb_real f1[9], q1[9];
    int k = e->norder, nd = e->nd, nx = e->nx;
    int i, j, a;

    for (i = 0; i < k * k; ++i) { f1[i] = (uwb_real)0; q1[i] = (uwb_real)0; }
    for (i = 0; i < k; ++i) f1[i * k + i] = (uwb_real)1;
    /* dt^(j-i)/(j-i)! */
    for (i = 0; i < k; ++i) {
        uwb_real term = (uwb_real)1;
        for (j = i + 1; j < k; ++j) {
            term *= dt / (uwb_real)(j - i);
            f1[i * k + j] = term;
        }
    }

    {
        uwb_real s2 = e->sigma_a * e->sigma_a;
        uwb_real d2 = dt * dt, d3 = d2 * dt, d4 = d3 * dt, d5 = d4 * dt;
        if (k == 2) {
            q1[0] = d3 / (uwb_real)3; q1[1] = d2 / (uwb_real)2;
            q1[2] = d2 / (uwb_real)2; q1[3] = dt;
        } else {
            q1[0] = d5 / (uwb_real)20; q1[1] = d4 / (uwb_real)8;  q1[2] = d3 / (uwb_real)6;
            q1[3] = d4 / (uwb_real)8;  q1[4] = d3 / (uwb_real)3;  q1[5] = d2 / (uwb_real)2;
            q1[6] = d3 / (uwb_real)6;  q1[7] = d2 / (uwb_real)2;  q1[8] = dt;
        }
        for (i = 0; i < k * k; ++i) q1[i] *= s2;
    }

    /* クロネッカー積 (小行列 (x) I_nd) */
    for (i = 0; i < nx * nx; ++i) { f[i] = (uwb_real)0; q[i] = (uwb_real)0; }
    for (i = 0; i < k; ++i)
        for (j = 0; j < k; ++j)
            for (a = 0; a < nd; ++a) {
                int row = i * nd + a, col = j * nd + a;
                f[row * nx + col] = f1[i * k + j];
                q[row * nx + col] = q1[i * k + j];
            }
}

void uwb_ekf_predict(uwb_ekf *e, uwb_real dt)
{
    uwb_real f[UWB_MAX_STATE * UWB_MAX_STATE], q[UWB_MAX_STATE * UWB_MAX_STATE];
    uwb_real xnew[UWB_MAX_STATE], tmp[UWB_MAX_STATE * UWB_MAX_STATE];
    int nx = e->nx, i, j, k;

    if (!(dt > (uwb_real)0)) return;
    transition(e, dt, f, q);

    for (i = 0; i < nx; ++i) {
        uwb_real s = (uwb_real)0;
        for (j = 0; j < nx; ++j) s += f[i * nx + j] * e->x[j];
        xnew[i] = s;
    }
    for (i = 0; i < nx; ++i) e->x[i] = xnew[i];

    /* P = F P F^T + Q */
    for (i = 0; i < nx; ++i)
        for (j = 0; j < nx; ++j) {
            uwb_real s = (uwb_real)0;
            for (k = 0; k < nx; ++k) s += f[i * nx + k] * e->P[k * nx + j];
            tmp[i * nx + j] = s;
        }
    for (i = 0; i < nx; ++i)
        for (j = 0; j < nx; ++j) {
            uwb_real s = (uwb_real)0;
            for (k = 0; k < nx; ++k) s += tmp[i * nx + k] * f[j * nx + k];
            e->P[i * nx + j] = s + q[i * nx + j];
        }
}

static void ekf_position(const uwb_ekf *e, uwb_real *p)
{
    int k;
    for (k = 0; k < 3; ++k) p[k] = (uwb_real)0;
    for (k = 0; k < e->nd; ++k) p[k] = e->x[k];
    if (e->nd == 2) p[2] = e->cfg->z_fixed;
}

/* スナップショット測位で立ち上げる。
 * **立ち上げだけは 1 本では足りない** — 測距 1 本は球面 1 枚でしかない。
 * 走り出したあとは 1 本ずつでも更新できるので、非同期に届く経路のために
 * 直近の測距を貯めておいて、揃った時点で立ち上げる。 */
static int bootstrap(uwb_ekf *e, uwb_real t, const uwb_meas *meas, int n)
{
    uwb_meas seed[UWB_MAX_MEAS];
    uwb_fix snap;
    uwb_real cutoff = t - e->max_dt;
    int i, j, m = 0, want = e->cfg->dim + 2;

    /* 届いた分を貯める */
    for (i = 0; i < n && e->n_pending < UWB_MAX_MEAS; ++i) {
        if (!uwb_meas_usable(e->cfg, &meas[i])) continue;
        e->pending[e->n_pending] = meas[i];
        e->pending_t[e->n_pending] = t;
        ++e->n_pending;
    }
    /* 古いものを捨てる */
    {
        int m2 = 0;
        for (i = 0; i < e->n_pending; ++i) {
            if (e->pending_t[i] < cutoff) continue;
            e->pending[m2] = e->pending[i];
            e->pending_t[m2] = e->pending_t[i];
            ++m2;
        }
        e->n_pending = m2;
    }
    /* アンカーごとに最新の 1 本だけ採る */
    for (i = e->n_pending - 1; i >= 0 && m < UWB_MAX_MEAS; --i) {
        int dup = 0;
        for (j = 0; j < m; ++j)
            if (seed[j].anchor == e->pending[i].anchor) { dup = 1; break; }
        if (!dup) seed[m++] = e->pending[i];
    }

    if (m < want) {
        /* 最低本数にも届かないうちは待つ。届いているのに窓が浅いだけなら
         * もう少し待ち、窓を超えたら最低本数で妥協する。 */
        uwb_real oldest = t;
        for (i = 0; i < e->n_pending; ++i)
            if (e->pending_t[i] < oldest) oldest = e->pending_t[i];
        if (m < e->cfg->dim + 1 || (t - oldest) < e->max_dt) return 0;
    }

    if (!uwb_solve_lv2(e->cfg, seed, m, &snap) || !snap.ok) return 0;

    for (i = 0; i < UWB_MAX_STATE; ++i) e->x[i] = (uwb_real)0;
    for (i = 0; i < UWB_MAX_STATE * UWB_MAX_STATE; ++i) e->P[i] = (uwb_real)0;
    for (i = 0; i < e->nx; ++i) e->P[i * e->nx + i] = (uwb_real)1;
    for (i = 0; i < e->nd; ++i) e->x[i] = snap.p[i];

    /* 位置はスナップショットの共分散をそのまま引き継ぐ。1e-4 を足すのは
     * 共分散がつぶれているとき (無雑音など) に更新が効かなくなるのを防ぐため。 */
    {
        int finite = 1;
        for (i = 0; i < e->nd; ++i)
            for (j = 0; j < e->nd; ++j)
                if (snap.cov[i * 3 + j] != snap.cov[i * 3 + j]) finite = 0;
        if (finite) {
            for (i = 0; i < e->nd; ++i)
                for (j = 0; j < e->nd; ++j)
                    e->P[i * e->nx + j] = snap.cov[i * 3 + j] +
                                          (i == j ? (uwb_real)1e-4 : (uwb_real)0);
        } else {
            for (i = 0; i < e->nd; ++i) e->P[i * e->nx + i] = (uwb_real)4;
        }
    }
    /* 速度 (と加速度) は未知。大きめの分散から始める: 10^(2-k) */
    {
        int k, order;
        for (order = 1; order < e->norder; ++order) {
            uwb_real v = (uwb_real)1;
            int e2 = 2 - order;
            for (k = 0; k < e2; ++k) v *= (uwb_real)10;
            for (k = 0; k < e2; ++k) { }
            if (e2 < 0) v = (uwb_real)1;
            for (i = 0; i < e->nd; ++i) {
                int r = order * e->nd + i;
                for (j = 0; j < e->nx; ++j) e->P[r * e->nx + j] = (uwb_real)0;
                for (j = 0; j < e->nx; ++j) e->P[j * e->nx + r] = (uwb_real)0;
                e->P[r * e->nx + r] = v;
            }
        }
    }

    e->initialized = 1;
    e->n_pending = 0;
    e->ambiguous = snap.ambiguous;
    if (!snap.ambiguous) {
        int s = uwb_mirror_side(e->cfg, snap.p);
        if (s != 0) { e->side_known = 1; e->side = s; }
    }
    return 1;
}

static void diagnostics(uwb_ekf *e, const uwb_meas *meas, int n,
                        int n_used, unsigned long excluded, uwb_fix *out)
{
    uwb_real p[3], jac[UWB_MAX_MEAS * 3];
    int i, k, cnt = 0;
    uwb_real num = (uwb_real)0;

    uwb_fix_failed(out, n);
    ekf_position(e, p);

    for (i = 0; i < n && cnt < UWB_MAX_MEAS; ++i) {
        uwb_real res, j3[3], sg;
        if (!uwb_meas_usable(e->cfg, &meas[i])) continue;
        uwb_evaluate(e->cfg, p, &meas[i], &res, j3, &sg);
        for (k = 0; k < 3; ++k) jac[cnt * 3 + k] = j3[k];
        num += res * res;
        ++cnt;
    }

    for (k = 0; k < 3; ++k) out->p[k] = p[k];
    for (k = 0; k < 3; ++k) out->v[k] = (uwb_real)0;
    for (k = 0; k < e->nd; ++k) out->v[k] = e->x[e->nd + k];
    for (k = 0; k < 9; ++k) out->cov[k] = (uwb_real)0;
    for (i = 0; i < e->nd; ++i)
        for (k = 0; k < e->nd; ++k) out->cov[i * 3 + k] = e->P[i * e->nx + k];

    out->ok = e->initialized;
    out->n_used = n_used;
    out->n_total = n;
    out->iterations = 1;
    out->ambiguous = e->ambiguous;
    out->residual_rms = cnt > 0 ? (uwb_real)sqrt((double)(num / (uwb_real)cnt)) : (uwb_real)-1;
    out->gdop = cnt > 0 ? uwb_gdop_from_jac(e->cfg, jac, cnt) : (uwb_real)-1;
    out->excluded = excluded;
    uwb_fix_finish(out);
}

int uwb_ekf_update(uwb_ekf *e, uwb_real t, const uwb_meas *meas, int n, uwb_fix *out)
{
    unsigned long excluded = 0UL;
    int nx = e->nx, i, n_used = 0;

    uwb_fix_failed(out, n);
    if (!e || !e->cfg || !meas) return 0;

    /* 遅れて届いた観測 (時刻の巻き戻り) は捨てる */
    if (e->has_t && t < e->t - (uwb_real)1e-9) {
        diagnostics(e, meas, n, 0, 0UL, out);
        return out->ok;
    }

    if (e->initialized && e->has_t) {
        uwb_real dt = t - e->t;
        if (dt > e->max_dt) e->initialized = 0;   /* 間が空きすぎ。組み直す */
        else uwb_ekf_predict(e, dt);
    }

    if (!e->initialized) {
        if (!bootstrap(e, t, meas, n)) {
            e->t = t; e->has_t = 1;
            uwb_fix_failed(out, n);
            return 0;
        }
        e->t = t; e->has_t = 1;
        diagnostics(e, meas, n, n, 0UL, out);
        return out->ok;
    }

    e->t = t; e->has_t = 1;

    for (i = 0; i < n; ++i) {
        uwb_real p[3], res, j3[3], sg;
        uwb_real h[UWB_MAX_STATE], ph[UWB_MAX_STATE], kg[UWB_MAX_STATE];
        uwb_real s, tmp[UWB_MAX_STATE * UWB_MAX_STATE];
        int a, b, c;

        if (!uwb_meas_usable(e->cfg, &meas[i])) continue;
        ekf_position(e, p);
        uwb_evaluate(e->cfg, p, &meas[i], &res, j3, &sg);

        for (a = 0; a < nx; ++a) h[a] = (uwb_real)0;
        for (a = 0; a < e->nd; ++a) h[a] = j3[a];

        for (a = 0; a < nx; ++a) {
            uwb_real acc = (uwb_real)0;
            for (b = 0; b < nx; ++b) acc += e->P[a * nx + b] * h[b];
            ph[a] = acc;
        }
        s = sg * sg;
        for (a = 0; a < nx; ++a) s += h[a] * ph[a];

        if (!(s > (uwb_real)0) || s != s) {
            if (i < 32) excluded |= (1UL << i);
            continue;
        }
        /* イノベーションゲート。外れ値を状態に入れない */
        if (absr(res) > e->gate * (uwb_real)sqrt((double)s)) {
            if (i < 32) excluded |= (1UL << i);
            continue;
        }

        for (a = 0; a < nx; ++a) kg[a] = ph[a] / s;
        for (a = 0; a < nx; ++a) e->x[a] += kg[a] * res;

        /* Joseph 形式。数値的に P の対称正定値性が保たれる */
        for (a = 0; a < nx; ++a)
            for (b = 0; b < nx; ++b) {
                uwb_real imkh = (a == b ? (uwb_real)1 : (uwb_real)0) - kg[a] * h[b];
                tmp[a * nx + b] = imkh;
            }
        {
            uwb_real m1[UWB_MAX_STATE * UWB_MAX_STATE];
            for (a = 0; a < nx; ++a)
                for (b = 0; b < nx; ++b) {
                    uwb_real acc = (uwb_real)0;
                    for (c = 0; c < nx; ++c) acc += tmp[a * nx + c] * e->P[c * nx + b];
                    m1[a * nx + b] = acc;
                }
            for (a = 0; a < nx; ++a)
                for (b = 0; b < nx; ++b) {
                    uwb_real acc = (uwb_real)0;
                    for (c = 0; c < nx; ++c) acc += m1[a * nx + c] * tmp[b * nx + c];
                    e->P[a * nx + b] = acc + kg[a] * kg[b] * sg * sg;
                }
            /* 対称化。丸めで崩れるのを毎回直す */
            for (a = 0; a < nx; ++a)
                for (b = a + 1; b < nx; ++b) {
                    uwb_real avg = (uwb_real)0.5 * (e->P[a * nx + b] + e->P[b * nx + a]);
                    e->P[a * nx + b] = avg;
                    e->P[b * nx + a] = avg;
                }
        }
        ++n_used;
    }

    /* ゲートが自分の誤りを守り続けている状態 (棺桶問題) からの復帰。
     * 全部弾き続けるなら、一度捨てて組み直す。 */
    if (n_used == 0 && n > 0) {
        ++e->rejects;
        if (e->rejects >= e->max_rejects) {
            e->initialized = 0;
            e->rejects = 0;
            e->n_pending = 0;
            if (bootstrap(e, t, meas, n)) {
                diagnostics(e, meas, n, n, 0UL, out);
                return out->ok;
            }
            uwb_fix_failed(out, n);
            return 0;
        }
    } else {
        e->rejects = 0;
    }

    diagnostics(e, meas, n, n_used, excluded, out);
    return out->ok;
}
