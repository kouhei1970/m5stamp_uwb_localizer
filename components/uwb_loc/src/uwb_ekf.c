/* 密結合 EKF (Lv3)。Python 版 uwb_loc/solvers/ekf.py に対応する。
 *
 * 「密結合」= 位置に直したものではなく**測距値そのもの**で更新する。
 * だから 1 本しか届かないエポックでも情報を使える (立ち上げだけは
 * 球面 1 枚では位置が決まらないので複数本たまるのを待つ。理想は dim+2 本
 * だが、登録アンカーが dim+1 台しかない構成ではそこまで待たず立ち上げる。
 * 受理条件の詳細は bootstrap() のコメント参照)。
 *
 * 更新はスカラー逐次。S がスカラーなので**行列の逆行列が要らない**。
 * P の更新は対称 rank-1 ダウンデート P −= u uᵀ/s (uwb_symn_rank1_downdate)。
 */
#include "uwb_internal.h"

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
    e->boot_wait_t0 = (uwb_real)0;
    e->has_boot_wait_t0 = 0;
    e->side_known = 0;
    e->side = 0;
}

/* 状態遷移とプロセスノイズ。
 * 軸ごとに独立とみなし、1 軸分の小行列をクロネッカー積で広げる。
 * 遷移は積分器の連鎖なので F[i][j] = dt^(j-i)/(j-i)!。
 *
 * プロセスノイズは **最上位の微分に連続時間の白色雑音が乗る**モデルで
 * 統一してある (CV なら加速度、CA なら加加速度)。離散版と混ぜると
 * sigma_a の意味がモードによって変わってしまう。
 *
 * k x k の小行列 f1/q1 をそのまま返す (nx x nx へのクロネッカー展開はしない)。
 * F = f1 (x) I_nd、Q = q1 (x) I_nd という構造は uwb_ekf_predict() 側で直接使う。
 *
 * 重要な前提: f1 は**単位上三角**である (f1[i*k+i] = 1、j > i のみ非ゼロ、
 * j < i は常にゼロ)。これは積分器の連鎖 (dt^(j-i)/(j-i)!) という遷移の
 * 作り方そのものから来る性質で、uwb_ekf_predict() の in-place 更新はこの前提が
 * 成り立つことに依存している。将来遷移モデルを変えるときはこの性質を
 * 壊さないこと (壊すなら predict 側も作業配列を使う実装に戻す必要がある)。
 *
 * k は 2 (CV) か 3 (CA) しか無いので、分数はすべて定数倍で書く (除算 0)。 */
static void transition(const uwb_ekf *e, uwb_real dt, uwb_real *f1, uwb_real *q1)
{
    int k = e->norder;
    uwb_real s2 = e->sigma_a * e->sigma_a;
    uwb_real d2 = dt * dt, d3 = d2 * dt;

    if (k == 2) {
        f1[0] = (uwb_real)1; f1[1] = dt;
        f1[2] = (uwb_real)0; f1[3] = (uwb_real)1;
        q1[0] = d3 * (uwb_real)(1.0 / 3.0) * s2;  q1[1] = d2 * (uwb_real)0.5 * s2;
        q1[2] = q1[1];                            q1[3] = dt * s2;
    } else {
        uwb_real d4 = d3 * dt, d5 = d4 * dt;
        f1[0] = (uwb_real)1; f1[1] = dt;          f1[2] = d2 * (uwb_real)0.5;
        f1[3] = (uwb_real)0; f1[4] = (uwb_real)1; f1[5] = dt;
        f1[6] = (uwb_real)0; f1[7] = (uwb_real)0; f1[8] = (uwb_real)1;
        q1[0] = d5 * (uwb_real)0.05 * s2;         /* d5/20 */
        q1[1] = d4 * (uwb_real)0.125 * s2;        /* d4/8  */
        q1[2] = d3 * (uwb_real)(1.0 / 6.0) * s2;  /* d3/6  */
        q1[3] = q1[1];
        q1[4] = d3 * (uwb_real)(1.0 / 3.0) * s2;  /* d3/3  */
        q1[5] = d2 * (uwb_real)0.5 * s2;          /* d2/2  */
        q1[6] = q1[2];
        q1[7] = q1[5];
        q1[8] = dt * s2;
    }
}

void uwb_ekf_predict(uwb_ekf *e, uwb_real dt)
{
    uwb_real f1[9], q1[9];
    int nx = e->nx, k = e->norder, nd = e->nd;
    int i, j, a, c;

    if (!(dt > (uwb_real)0)) return;
    transition(e, dt, f1, q1);

    /* 1) x <- F x。f1 は単位上三角なので、i (次数のブロック) を昇順に
     * 処理すれば、右辺で読む x[j*nd+a] (j>i) はまだ今回の更新を受けて
     * おらず、xnew の一時配列なしで in-place に計算できる。 */
    for (i = 0; i < k; ++i)
        for (a = 0; a < nd; ++a) {
            int r = i * nd + a;
            uwb_real s = e->x[r];      /* f1[i][i] = 1 の項 */
            for (j = i + 1; j < k; ++j) s += f1[i * k + j] * e->x[j * nd + a];
            e->x[r] = s;
        }

    /* 2) P <- F P (行方向)。行 r=i*nd+a の更新に足し込むのは行
     * rj=j*nd+a (j>i) だけであり、rj > r が常に成り立つ (r を昇順に
     * 処理する限り rj はまだ未更新)。したがって i (→ a) を昇順に
     * 回せば in-place で正しい。行 r 自身を後から他の行の材料として
     * 使うことはない (j>i' の条件から自分自身の行ブロックは除外される)。 */
    for (i = 0; i < k; ++i)
        for (a = 0; a < nd; ++a) {
            int r = i * nd + a;
            for (j = i + 1; j < k; ++j) {
                uwb_real coef = f1[i * k + j];
                int rj = j * nd + a;
                for (c = 0; c < nx; ++c) e->P[r * nx + c] += coef * e->P[rj * nx + c];
            }
        }

    /* 3) P <- (F P) F^T (列方向)。2) と対称の議論で、列 cj=j*nd+a の
     * 更新に使う列 ci=i*nd+a (i>j) は ci > cj であり、cj を昇順に
     * 処理する限り未更新。in-place で正しい。 */
    for (j = 0; j < k; ++j)
        for (a = 0; a < nd; ++a) {
            int cj = j * nd + a;
            for (i = j + 1; i < k; ++i) {
                uwb_real coef = f1[j * k + i];
                int ci = i * nd + a;
                for (c = 0; c < nx; ++c) e->P[c * nx + cj] += coef * e->P[c * nx + ci];
            }
        }

    /* 4) P <- P + Q。Q = q1 (x) I_nd なので、軸 a が一致する成分にしか
     * 足し込まれない (非対角ブロックは軸をまたぐとゼロ)。 */
    for (i = 0; i < k; ++i)
        for (j = 0; j < k; ++j)
            for (a = 0; a < nd; ++a)
                e->P[(i * nd + a) * nx + (j * nd + a)] += q1[i * k + j];
}

static void ekf_position(const uwb_ekf *e, uwb_real *p)
{
    int k;
    for (k = 0; k < 3; ++k) p[k] = (uwb_real)0;
    for (k = 0; k < e->nd; ++k) p[k] = e->x[k];
    if (e->nd == 2) p[2] = e->cfg->z_fixed;
}

/* cfg に登録されている enabled なアンカー台数。plane_compute()
 * (uwb_model.c) と同じく cfg->anchors[i].enabled を直接見る。
 * bootstrap() が「これ以上待っても増えない上限」として使う。 */
static int n_enabled_anchors(const uwb_config *cfg)
{
    int i, n = 0;
    for (i = 0; i < cfg->n_anchors; ++i)
        if (cfg->anchors[i].enabled) ++n;
    return n;
}

/* スナップショット測位で立ち上げる。
 * **立ち上げだけは 1 本では足りない** — 測距 1 本は球面 1 枚でしかない。
 * 走り出したあとは 1 本ずつでも更新できるので、非同期に届く経路のために
 * 直近の測距を貯めておいて、揃った時点で立ち上げる。
 *
 * 受理条件は次の 3 経路 (優先順、いずれか 1 つで可):
 *   1. m >= dim+2                                   … 理想本数が揃った。即立ち上げ
 *   2. m >= dim+1 かつ m >= 登録 enabled アンカー台数 … これ以上待っても
 *      新しいアンカーは増えない (3台×2D・4台×3D の主構成はここで
 *      最初の1周期に立ち上がる)
 *   3. m >= dim+1 かつ「待ち始めてから max_dt 経過」 … 一部のアンカーが
 *      遮蔽されていて m が dim+1 止まりのままの救済
 * 経路 2/3 が無いと、有効アンカーが dim+1 台しかない構成では m が
 * 決して want (=dim+2) に届かず永遠に立ち上がらない。 */
static int bootstrap(uwb_ekf *e, uwb_real t, const uwb_meas *meas, int n)
{
    uwb_meas seed[UWB_MAX_MEAS];
    uwb_fix snap;
    uwb_real cutoff = t - e->max_dt;
    int was_empty = (e->n_pending == 0);
    int i, j, m = 0, want = e->cfg->dim + 2;

    /* 届いた分を貯める。pending が空からの立ち上がりなら、それを
     * 「待ち始めた時刻」として記録する (刈り込みでは消さない)。 */
    for (i = 0; i < n && e->n_pending < UWB_MAX_MEAS; ++i) {
        if (!uwb_meas_usable(e->cfg, &meas[i])) continue;
        e->pending[e->n_pending] = meas[i];
        e->pending_t[e->n_pending] = t;
        ++e->n_pending;
    }
    if (was_empty && e->n_pending > 0) {
        e->boot_wait_t0 = t;
        e->has_boot_wait_t0 = 1;
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
    /* 全部刈り込まれて空に戻ったら、次に積む 1 本が新しい「待ち始め」になる */
    if (e->n_pending == 0) e->has_boot_wait_t0 = 0;
    /* アンカーごとに最新の 1 本だけ採る */
    for (i = e->n_pending - 1; i >= 0 && m < UWB_MAX_MEAS; --i) {
        int dup = 0;
        for (j = 0; j < m; ++j)
            if (seed[j].anchor == e->pending[i].anchor) { dup = 1; break; }
        if (!dup) seed[m++] = e->pending[i];
    }

    if (m < want) {
        int have_all_anchors = m >= n_enabled_anchors(e->cfg);
        int waited_out = e->has_boot_wait_t0 && (t - e->boot_wait_t0) >= e->max_dt;
        if (m < e->cfg->dim + 1 || !(have_all_anchors || waited_out)) return 0;
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
    e->has_boot_wait_t0 = 0;
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
    out->residual_rms = cnt > 0 ? uwb_math_sqrt(num / (uwb_real)cnt) : (uwb_real)-1;
    out->gdop = cnt > 0 ? uwb_gdop_from_jac(e->cfg, jac, cnt) : (uwb_real)-1;
    out->excluded = excluded;
    uwb_fix_finish(out);
}

int uwb_ekf_update(uwb_ekf *e, uwb_real t, const uwb_meas *meas, int n, uwb_fix *out)
{
    unsigned long excluded = 0UL;
    uwb_real gate2;
    int nx, i, n_used = 0;

    uwb_fix_failed(out, n);
    if (!e || !e->cfg || !meas) return 0;
    nx = e->nx;
    /* イノベーションゲート: |res| > gate·sqrt(s) を平方根なしで res² > gate·|gate|·s
     * と比べる (gate < 0 なら右辺が負になり、旧実装と同じく常に棄却)。 */
    gate2 = e->gate * absr(e->gate);

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
        uwb_real u[UWB_MAX_STATE];
        uwb_real s, inv_s, gain;
        int a, b;

        if (!uwb_meas_usable(e->cfg, &meas[i])) continue;
        ekf_position(e, p);
        uwb_evaluate(e->cfg, p, &meas[i], &res, j3, &sg);

        /* h の非ゼロは先頭 nd 個 (= j3) だけ。
         * u = P h。P は対称 (上三角を更新して鏡映する) なので hᵀP = uᵀ。 */
        for (a = 0; a < nx; ++a) {
            uwb_real su = (uwb_real)0;
            for (b = 0; b < e->nd; ++b) su += e->P[a * nx + b] * j3[b];
            u[a] = su;
        }
        s = sg * sg;
        for (a = 0; a < e->nd; ++a) s += j3[a] * u[a];

        if (!(s > (uwb_real)0) || s != s) {
            if (i < 32) excluded |= (1UL << i);
            continue;
        }
        /* イノベーションゲート。外れ値を状態に入れない */
        if (res * res > gate2 * s) {
            if (i < 32) excluded |= (1UL << i);
            continue;
        }

        /* K = u/s。x += K·res。除算は 1/s の 1 回だけ。 */
        inv_s = (uwb_real)1 / s;
        gain = res * inv_s;
        for (a = 0; a < nx; ++a) e->x[a] += u[a] * gain;

        /* Joseph 形式 (I−Khᵀ)P(I−Khᵀ)ᵀ + KRKᵀ は、P が対称なら厳密に
         * P − K uᵀ = P − u uᵀ/s に潰れる (w = u、v = u − K(s−R) = RK のため)。
         * 対称な rank-1 ダウンデートとして上三角だけ計算し鏡映する。 */
        uwb_symn_rank1_downdate(e->P, nx, u, inv_s);
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
