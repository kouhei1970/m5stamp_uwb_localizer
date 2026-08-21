/* 新旧比較の回帰テスト。
 *
 * 「新実装」= components/uwb_loc (uwb_math ベースに書き換えたもの)。
 * 「参照実装」= 上流 uwb_localizer 凍結版 (このリポジトリの commit 4298c08
 * 時点の components/uwb_loc をそのまま tests/host/loc/ref/ に写したもの。
 * ref/ref_prefix.h で全シンボルを ref_ 接頭辞に付け替えてリンクしている)。
 *
 * このファイルは新実装の "uwb_loc.h" と参照実装のプレーン C ラッパ
 * "ref/ref_bridge.h" の両方を使う。型の衝突を避けるため、参照実装の型
 * (ref/uwb_loc.h の uwb_config など) はここには一切出てこない —
 * ref_bridge.h は double と int だけで受け渡しする。
 *
 * 決定的な擬似乱数で大量のシナリオを生成し、新旧の出力を比較する。
 * 乱数の作り方は test_uwb.c と同じ LCG (再現性のため自前で持つ)。
 *
 * 比較の方針 (表の category に "~" や "/path" が付かない = 合否に使う):
 *   - ok / excluded / n_used / ambiguous は完全一致が必須 (不一致 = 失敗)。
 *   - iterations の不一致は件数を数えるだけ (失敗にはしない)。
 *   - 位置 p (ok のとき) は許容差以下が原則。超えたときは目的関数 (double で
 *     評価。Beck は Σw(‖p−a‖²−r²)²、LLS は正規方程式の残差、NLS は Huber 込み
 *     の Σρ) で「新が旧より悪くない」ことを確かめ、悪くなければ p_3d_path
 *     (報告のみ)、悪ければ p_3d_worse (失敗)。float では Gauss-Newton の
 *     コスト比較が丸めで割れて反復回数が変わり、同じ局所解に別経路で着く。
 *   - cov は相対差 (|Δ|/max(1,|ref|))。目的関数判定を使う所は報告のみ。
 *   - gdop / sigma / residual_rms / v / EKF の状態 x, P は最大差を集計して
 *     表に出すだけ (EKF の P は rank-1 ダウンデートと Joseph 形式が数学的に
 *     同値なだけで、double で 1e-9 相対程度の差が出るのが正常)。
 *   - uwb_beck_gtrs / uwb_lls_trilateration / uwb_gdop_at / uwb_crlb_at /
 *     uwb_anchors_coplanar は「ref 成功なのに新失敗」を失敗、
 *     「ref 失敗なのに新成功」は改善として件数を報告するだけ (float ビルドでは
 *     旧 Beck が FMA 無しだと広く失敗する既知の不具合があるため)。
 *
 * 報告のみにする試行 ("~" 付き category):
 *   - 同一平面判定 (使った観測のアンカーで固有値比 < 0.05²) が立つ配置。
 *     完全同一平面では G が厳密に特異で、旧 Beck はコレスキーの丸めで
 *     たまたま通ることがあり (新は σ_min ≤ 64·eps·σ_max で明示的に失敗)、
 *     その不良条件の解から始めた NLS は別の局所解 / 鏡像に収束しうる。
 *     旧 LLS は常に足す 1e-12 倍のリッジが κ の分だけ偏る (最大 1cm)。
 *     GDOP / CRLB は旧 LU が巨大な値 (1e7 など) を返す所で新は負を返す。
 *   - 新旧で Beck の成否が違う = NLS の初期値が違う試行 (Lv1~ / Lv2~)。
 *   - EKF は位置が一度 TOL_P_EKF を超えて分岐した以降のステップ (EKF/path)。
 *     フィルタは過去に依存するので、それ以降の一致は期待できない。
 *     double では分岐は起きない (diverged_traj = 0)。分岐後に真値へ近いのが
 *     新旧どちらかは p_fail_new_closer / p_fail_ref_closer に数える。
 *
 * 環境変数 REGRESS_DUMP=<試行番号> でその試行の入力を印字する (原因調査用)。
 */
#include "uwb_loc.h"
#include "ref/ref_bridge.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ------------------------------------------------------------ 許容差 */

#if UWB_REAL_IS_FLOAT
#define TOL_P      1e-4    /* 位置 [m] (Lv1/Lv2/Beck。超えたら目的関数で新が悪くないか見る) */
#define TOL_P_LLS  1e-3    /* 位置 [m] (Lv0/LLS。正規方程式の κ ≈ 1e5 で float の丸めが効く) */
#define TOL_P_EKF  1e-2    /* 位置 [m] (EKF。ゲート判定と反復経路が丸めで変わる) */
/* 目的関数の「悪くない」判定: J_new ≤ J_ref (1 + OBJ_REL) + OBJ_ABS。
 * float は J 自体の丸め (残差 0.3 mm 相当 ≈ 1e-3) を床にする */
#define OBJ_REL    1e-2
#define OBJ_ABS    1e-3
#define TOL_RELCOV 1e-2    /* cov / gdop_at / crlb_at の相対差 */
#else
/* Lv1/Lv2: Gauss-Newton は ‖step‖ < tol (1e-4 m) で止まるので、収束の
 * 経路がわずかに違うと最終反復点は 1e-8 m 程度ずれる (2 次収束の残り)。
 * Lv0/LLS: 旧実装は常に対角平均の 1e-12 倍のリッジを足していて、
 * 条件数 κ の分だけ偏る (κ(AᵀWA) ≈ 1e5 の 4 台配置で 5e-7 m を実測。
 * 新実装は numpy lstsq と 1e-10 m で一致する)。 */
#define TOL_P      1e-7
#define TOL_P_LLS  1e-6
#define TOL_P_EKF  1e-7
#define OBJ_REL    1e-3
#define OBJ_ABS    1e-9
#define TOL_RELCOV 1e-6    /* cov は κ 倍に増幅されるので位置より 1 桁緩い */
#endif

/* ------------------------------------------------------------ 乱数 */
/* test_uwb.c と同じ LCG。シードを変えれば独立した系列になる。 */
static unsigned long g_seed = 20260821UL;

static double urand(void)
{
    g_seed = g_seed * 1103515245UL + 12345UL;
    return (double)((g_seed >> 16) & 0x7fffUL) / 32767.0;
}

static double nrand(void) /* Box-Muller */
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static double dist3d(const double a[3], const double b[3])
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* ------------------------------------------------------------ 集計 */

#define NG_PRINT_CAP 12   /* 指標ごとの NG 行の上限 (全指標が見えるように) */

static int g_run = 0;
static int g_fail = 0;

typedef struct {
    const char *cat;
    const char *name;
    double tol;       /* NAN なら「報告のみ」(合否には使わない) */
    double max_diff;
    int    n;
    int    fail;
    int    printed;   /* NG 行を出した数 (指標ごとに NG_PRINT_CAP まで) */
    int    improve;   /* -1 なら「該当しない」。0 以上なら改善件数 */
} Metric;

#define MAX_METRICS 160
static Metric g_metrics[MAX_METRICS];
static int g_nmetrics = 0;

static Metric *metric(const char *cat, const char *name, double tol)
{
    int i;
    for (i = 0; i < g_nmetrics; ++i) {
        if (strcmp(g_metrics[i].cat, cat) == 0 && strcmp(g_metrics[i].name, name) == 0)
            return &g_metrics[i];
    }
    g_metrics[g_nmetrics].cat = cat;
    g_metrics[g_nmetrics].name = name;
    g_metrics[g_nmetrics].tol = tol;
    g_metrics[g_nmetrics].max_diff = 0.0;
    g_metrics[g_nmetrics].n = 0;
    g_metrics[g_nmetrics].fail = 0;
    g_metrics[g_nmetrics].improve = -1;
    g_metrics[g_nmetrics].printed = 0;
    if (g_nmetrics + 1 >= MAX_METRICS) {
        printf("MAX_METRICS (%d) を超えた\n", MAX_METRICS);
        exit(2);
    }
    return &g_metrics[g_nmetrics++];
}

static void m_exact(Metric *m, long a, long b, const char *ctx)
{
    int match = (a == b);
    ++m->n;
    ++g_run;
    if (!match) {
        ++m->fail;
        ++g_fail;
        if (m->printed++ < NG_PRINT_CAP)
            printf("  NG  %s.%s  new=%ld ref=%ld (%s)\n", m->cat, m->name, a, b, ctx);
    }
}

/* 一致件数だけ数える。g_run/g_fail には影響しない (報告専用)。 */
static void m_mismatch(Metric *m, long a, long b)
{
    ++m->n;
    if (a != b) ++m->fail;
}

static void m_num(Metric *m, double diff, int required, const char *ctx)
{
    double ad = diff < 0 ? -diff : diff;
    ++m->n;
    if (ad > m->max_diff) m->max_diff = ad;
    if (required) {
        ++g_run;
        if (ad > m->tol) {
            ++m->fail;
            ++g_fail;
            if (m->printed++ < NG_PRINT_CAP)
                printf("  NG  %s.%s  diff=%.3g > tol=%.3g (%s)\n", m->cat, m->name, ad, m->tol, ctx);
        }
    }
}

/* ref 成功/新失敗 だけを失敗にする。ref 失敗/新成功 は改善として数える。 */
static void m_success(Metric *m, int ref_ok, int new_ok, const char *ctx)
{
    if (m->improve < 0) m->improve = 0;
    ++m->n;
    ++g_run;
    if (ref_ok && !new_ok) {
        ++m->fail;
        ++g_fail;
        if (m->printed++ < NG_PRINT_CAP)
            printf("  NG  %s.%s  ref 成功なのに新失敗 (%s)\n", m->cat, m->name, ctx);
    } else if (!ref_ok && new_ok) {
        ++m->improve;
    }
}

/* ------------------------------------------------------------- 部屋 */

#define ROOM_X 10.0
#define ROOM_Y 8.0
#define ROOM_Z 3.0
#define MAXN   8

typedef struct {
    int    n_anchors;
    double p[MAXN][3];
    int    enabled[MAXN];
    double antenna_delay[MAXN];
} Scenario;

static void gen_anchors(Scenario *s)
{
    int i;
    double zmode = urand();
    double flat_z = urand() * ROOM_Z;

    s->n_anchors = 4 + (int)(urand() * 5.0);   /* 4..8 */
    if (s->n_anchors > MAXN) s->n_anchors = MAXN;
    if (s->n_anchors < 4) s->n_anchors = 4;

    for (i = 0; i < s->n_anchors; ++i) {
        s->p[i][0] = urand() * ROOM_X;
        s->p[i][1] = urand() * ROOM_Y;
        if (zmode < 0.15) {
            s->p[i][2] = flat_z;                            /* 完全同一平面 */
        } else if (zmode < 0.25) {
            s->p[i][2] = flat_z + (urand() - 0.5) * 0.05;    /* ほぼ同一平面 (5cm) */
        } else {
            s->p[i][2] = urand() * ROOM_Z;
        }
        s->enabled[i] = 1;
        s->antenna_delay[i] = 0.0;
    }

    if (urand() < 0.05) {
        int idx = (int)(urand() * s->n_anchors);
        if (idx >= s->n_anchors) idx = s->n_anchors - 1;
        s->enabled[idx] = 0;
    }
    if (urand() < 0.10) {
        for (i = 0; i < s->n_anchors; ++i) s->antenna_delay[i] = urand() * 0.05;
    }
}

/* ------------------------------------------------------------- 観測 */

typedef struct {
    int    anchor;
    double value;
    double sigma;
    double quality;
} MeasGen;

static int gen_meas(const Scenario *s, const double truth[3], double noise_sigma, MeasGen out[MAXN])
{
    int keep[MAXN];
    double bias[MAXN];
    int i, drop_n = 0, nlos_n = 0, m;
    int use_sigma   = (urand() < 0.10);
    int use_quality = (urand() < 0.10);
    double sigma_val = (noise_sigma > 0.0) ? noise_sigma : 0.05;

    for (i = 0; i < s->n_anchors; ++i) { keep[i] = 1; bias[i] = 0.0; }

    if (urand() < 0.30 && s->n_anchors > 4) {
        drop_n = (urand() < 0.5) ? 1 : 2;
        if (drop_n > s->n_anchors - 4) drop_n = s->n_anchors - 4;
        for (i = 0; i < drop_n; ++i) {
            int idx, tries = 0;
            do {
                idx = (int)(urand() * s->n_anchors);
                if (idx >= s->n_anchors) idx = s->n_anchors - 1;
                ++tries;
            } while (!keep[idx] && tries < 20);
            keep[idx] = 0;
        }
    }

    if (urand() < 0.40) {
        nlos_n = (urand() < 0.8) ? 1 : 2;
        for (i = 0; i < nlos_n; ++i) {
            int idx, tries = 0;
            do {
                idx = (int)(urand() * s->n_anchors);
                if (idx >= s->n_anchors) idx = s->n_anchors - 1;
                ++tries;
            } while (bias[idx] != 0.0 && tries < 20);
            bias[idx] = 0.5 + urand() * 2.5;   /* U(0.5, 3.0) */
        }
    }

    m = 0;
    for (i = 0; i < s->n_anchors; ++i) {
        double d, val;
        if (!keep[i]) continue;
        d = dist3d(truth, s->p[i]);
        val = d + s->antenna_delay[i] + bias[i] + noise_sigma * nrand();
        out[m].anchor  = i;
        out[m].value   = val;
        out[m].sigma   = use_sigma ? sigma_val : 0.0;
        out[m].quality = use_quality ? urand() : -1.0;
        ++m;
    }
    return m;
}

/* ------------------------------------------------------------- 設定 */

typedef struct {
    int    dim;
    double z_fixed;
    int    use_z_bounds;
    double z_min, z_max;
    int    max_iter;
    double tol;
    double huber_k;
    double k_pos_scale;
    int    one_sided;
    double chi2_k;
    int    physical_gate;
    double max_range;
} CfgGen;

static void cfg_defaults(CfgGen *c)
{
    /* uwb_config_init (uwb_model.c) の既定値と揃えてある */
    c->dim = 3;
    c->z_fixed = 0.0;
    c->use_z_bounds = 0;
    c->z_min = 0.0; c->z_max = 0.0;
    c->max_iter = 30;
    c->tol = 1e-4;
    c->huber_k = 1.345;
    c->k_pos_scale = 0.6;
    c->one_sided = 1;
    c->chi2_k = -1.0;
    c->physical_gate = 1;
    c->max_range = 200.0;
}

static void gen_cfg(const double truth[3], CfgGen *c)
{
    cfg_defaults(c);
    if (urand() < 0.20) {
        c->dim = 2;
        c->z_fixed = truth[2];
    }
    if (urand() < 0.30) {
        c->use_z_bounds = 1;
        c->z_min = 0.0;
        c->z_max = ROOM_Z;
    }
    if (urand() < 0.15) c->huber_k *= 0.5 + urand() * 1.5;
    if (urand() < 0.15) c->chi2_k = 2.5 + urand() * 3.5;
}

/* ------------------------------------------------------------- 詰め替え */

static void fill_new_anchors(const Scenario *s, uwb_anchor out[MAXN])
{
    int i;
    for (i = 0; i < s->n_anchors; ++i) {
        snprintf(out[i].id, sizeof(out[i].id), "A%d", i);
        out[i].p[0] = (uwb_real)s->p[i][0];
        out[i].p[1] = (uwb_real)s->p[i][1];
        out[i].p[2] = (uwb_real)s->p[i][2];
        out[i].enabled = s->enabled[i];
        out[i].antenna_delay_m = (uwb_real)s->antenna_delay[i];
        out[i].sigma0 = (uwb_real)0.08;
        out[i].sigma_per_m = (uwb_real)0;
    }
}

static void fill_ref_anchors(const Scenario *s, ref_anchor out[MAXN])
{
    int i;
    for (i = 0; i < s->n_anchors; ++i) {
        out[i].p[0] = s->p[i][0];
        out[i].p[1] = s->p[i][1];
        out[i].p[2] = s->p[i][2];
        out[i].enabled = s->enabled[i];
        out[i].antenna_delay_m = s->antenna_delay[i];
        out[i].sigma0 = 0.08;
        out[i].sigma_per_m = 0.0;
    }
}

static void apply_new_cfg(uwb_config *cfg, const uwb_anchor *anchors, int na, const CfgGen *g)
{
    uwb_config_init(cfg, anchors, na);
    cfg->dim           = g->dim;
    cfg->z_fixed       = (uwb_real)g->z_fixed;
    cfg->use_z_bounds  = g->use_z_bounds;
    cfg->z_min         = (uwb_real)g->z_min;
    cfg->z_max         = (uwb_real)g->z_max;
    cfg->max_iter      = g->max_iter;
    cfg->tol           = (uwb_real)g->tol;
    cfg->huber_k       = (uwb_real)g->huber_k;
    cfg->k_pos_scale   = (uwb_real)g->k_pos_scale;
    cfg->one_sided     = g->one_sided;
    cfg->chi2_k        = (uwb_real)g->chi2_k;
    cfg->physical_gate = g->physical_gate;
    cfg->max_range     = (uwb_real)g->max_range;
}

static void fill_ref_cfg(ref_cfg *rc, const CfgGen *g)
{
    rc->dim = g->dim;
    rc->z_fixed = g->z_fixed;
    rc->use_z_bounds = g->use_z_bounds;
    rc->z_min = g->z_min;
    rc->z_max = g->z_max;
    rc->max_iter = g->max_iter;
    rc->tol = g->tol;
    rc->huber_k = g->huber_k;
    rc->k_pos_scale = g->k_pos_scale;
    rc->one_sided = g->one_sided;
    rc->chi2_k = g->chi2_k;
    rc->physical_gate = g->physical_gate;
    rc->max_range = g->max_range;
}

/* --------------------------------------------------------- fix の比較 */

typedef struct {
    Metric *ok, *excluded, *n_used, *ambiguous, *iterations;
    Metric *p_axis, *p_3d, *cov_rel, *gdop, *sigma, *residual_rms, *v;
} FixMetrics;

static FixMetrics make_fix_metrics_tol(const char *cat, double tol_p)
{
    FixMetrics fm;
    fm.ok           = metric(cat, "ok", NAN);
    fm.excluded     = metric(cat, "excluded", NAN);
    fm.n_used       = metric(cat, "n_used", NAN);
    fm.ambiguous    = metric(cat, "ambiguous", NAN);
    fm.iterations   = metric(cat, "iterations", NAN);
    fm.p_axis       = metric(cat, "p_axis_max", tol_p);
    fm.p_3d         = metric(cat, "p_3d", tol_p);
    fm.cov_rel      = metric(cat, "cov_rel", TOL_RELCOV);
    fm.gdop         = metric(cat, "gdop", NAN);
    fm.sigma        = metric(cat, "sigma", NAN);
    fm.residual_rms = metric(cat, "residual_rms", NAN);
    fm.v            = metric(cat, "v", NAN);
    return fm;
}

static FixMetrics make_fix_metrics(const char *cat) { return make_fix_metrics_tol(cat, TOL_P); }

/* strict=1: ok/excluded/n_used/ambiguous は完全一致必須、p/cov は許容差で合否。
 * strict=0: すべて報告のみ (件数と最大差だけ数える)。新旧で初期値が違う
 * (旧 Beck が特異な G をたまたま解いた) 試行や、同一平面アンカーで解が
 * 鏡像・平面上に退化しうる試行に使う。 */
static void judge_by_objective(Metric *strict_m, Metric *path_m, Metric *worse_m,
                               double diff, double J_new, double J_ref, const char *ctx);

typedef struct {
    Metric *path;    /* 許容差超だが目的関数は悪くない (報告のみ) */
    Metric *worse;   /* 許容差超で目的関数も悪い (失敗) */
    double  J_new, J_ref;
} ObjCheck;

static void compare_fix(const FixMetrics *fm, const uwb_fix *nf, const ref_fix *rf,
                        int strict, const ObjCheck *oc, const char *ctx)
{
    if (strict) {
        m_exact(fm->ok, nf->ok, rf->ok, ctx);
        m_exact(fm->excluded, (long)nf->excluded, (long)rf->excluded, ctx);
        m_exact(fm->n_used, nf->n_used, rf->n_used, ctx);
        m_exact(fm->ambiguous, nf->ambiguous, rf->ambiguous, ctx);
    } else {
        m_mismatch(fm->ok, nf->ok, rf->ok);
        m_mismatch(fm->excluded, (long)nf->excluded, (long)rf->excluded);
        m_mismatch(fm->n_used, nf->n_used, rf->n_used);
        m_mismatch(fm->ambiguous, nf->ambiguous, rf->ambiguous);
    }
    m_mismatch(fm->iterations, nf->iterations, rf->iterations);

    if (nf->ok && rf->ok) {
        int k;
        double diffp[3], d3, axmax = 0.0, maxrel = 0.0;
        for (k = 0; k < 3; ++k) {
            diffp[k] = (double)nf->p[k] - rf->p[k];
            if (fabs(diffp[k]) > axmax) axmax = fabs(diffp[k]);
        }
        d3 = sqrt(diffp[0] * diffp[0] + diffp[1] * diffp[1] + diffp[2] * diffp[2]);
        if (strict && oc) {
            m_num(fm->p_axis, axmax, 0, ctx);
            judge_by_objective(fm->p_3d, oc->path, oc->worse, d3, oc->J_new, oc->J_ref, ctx);
        } else {
            m_num(fm->p_axis, axmax, strict, ctx);
            m_num(fm->p_3d, d3, strict, ctx);
        }

        for (k = 0; k < 9; ++k) {
            double dc = fabs((double)nf->cov[k] - rf->cov[k]);
            double denom = (1.0 > fabs(rf->cov[k])) ? 1.0 : fabs(rf->cov[k]);
            double rel = dc / denom;
            if (rel > maxrel) maxrel = rel;
        }
        /* 経路が違えば cov も違うので、目的関数判定を使う場合は報告のみ */
        m_num(fm->cov_rel, maxrel, strict && !oc, ctx);

        m_num(fm->gdop, fabs((double)nf->gdop - rf->gdop), 0, ctx);
        m_num(fm->sigma, fabs((double)nf->sigma - rf->sigma), 0, ctx);
        m_num(fm->residual_rms, fabs((double)nf->residual_rms - rf->residual_rms), 0, ctx);
        {
            double vmax = 0.0;
            for (k = 0; k < 3; ++k) {
                double dv = fabs((double)nf->v[k] - rf->v[k]);
                if (dv > vmax) vmax = dv;
            }
            m_num(fm->v, vmax, 0, ctx);
        }
    }
}


/* ------------------------------------------------- 目的関数 (double で評価) */
/* 新旧の解が許容差を超えて違うとき、「新が旧より悪くないか」を目的関数の値で
 * 判定する (float では反復の経路が丸めで変わり、同じ局所解でも 1e-3 m 程度
 * ずれるため)。観測モデルは uwb_loc と同じ: σ = (meas.sigma > 0 ? それ :
 * 0.08) × (quality が [0,1] なら 1 + 3(1 − q))、下限 1e-6。 */

static double meas_sigma(const MeasGen *m)
{
    double s = (m->sigma > 0.0) ? m->sigma : 0.08;
    if (m->quality >= 0.0 && m->quality <= 1.0) s *= 1.0 + 3.0 * (1.0 - m->quality);
    return s > 1e-6 ? s : 1e-6;
}

/* Beck の目的関数 Σ w (‖p − a‖² − r²)²。dim=2 なら水平距離で。 */
static double objective_beck(const Scenario *scn, const MeasGen *mg, int nmeas,
                             const CfgGen *cg, const double p[3])
{
    double J = 0.0;
    int k;
    for (k = 0; k < nmeas; ++k) {
        const double *a = scn->p[mg[k].anchor];
        double r = mg[k].value - scn->antenna_delay[mg[k].anchor];
        double s = meas_sigma(&mg[k]);
        double d2, e;
        if (!scn->enabled[mg[k].anchor] || !(r > 0.0)) continue;
        if (cg->dim == 2) {
            double dz = cg->z_fixed - a[2];
            double h2 = r * r - dz * dz;
            r = sqrt(h2 > 1e-4 ? h2 : 1e-4);
            d2 = (p[0] - a[0]) * (p[0] - a[0]) + (p[1] - a[1]) * (p[1] - a[1]);
        } else {
            d2 = (p[0] - a[0]) * (p[0] - a[0]) + (p[1] - a[1]) * (p[1] - a[1]) +
                 (p[2] - a[2]) * (p[2] - a[2]);
        }
        e = d2 - r * r;
        J += e * e / (s * s);
    }
    return J;
}

/* LLS の目的関数 Σ sw² (a·x − b)² (基準 = 最小測距、重みは調和平均)。 */
static double objective_lls(const Scenario *scn, const MeasGen *mg, int nmeas,
                            const CfgGen *cg, const double p[3])
{
    double pos[MAXN][3], rng[MAXN], w[MAXN], J = 0.0;
    int m = 0, k, ref = 0, d = cg->dim == 2 ? 2 : 3;
    for (k = 0; k < nmeas; ++k) {
        const double *a = scn->p[mg[k].anchor];
        double r = mg[k].value - scn->antenna_delay[mg[k].anchor];
        double s = meas_sigma(&mg[k]);
        if (!scn->enabled[mg[k].anchor] || !(r > 0.0)) continue;
        if (d == 2) {
            double dz = cg->z_fixed - a[2];
            double h2 = r * r - dz * dz;
            r = sqrt(h2 > 1e-4 ? h2 : 1e-4);
        }
        pos[m][0] = a[0]; pos[m][1] = a[1]; pos[m][2] = a[2];
        rng[m] = r; w[m] = 1.0 / (s * s);
        ++m;
    }
    for (k = 1; k < m; ++k) if (rng[k] < rng[ref]) ref = k;
    for (k = 0; k < m; ++k) {
        double ax = 0.0, aa = 0.0, aref = 0.0, b, sw2;
        int j;
        if (k == ref) continue;
        for (j = 0; j < d; ++j) {
            ax += 2.0 * (pos[ref][j] - pos[k][j]) * p[j];
            aa += pos[k][j] * pos[k][j];
            aref += pos[ref][j] * pos[ref][j];
        }
        b = rng[k] * rng[k] - rng[ref] * rng[ref] - aa + aref;
        sw2 = w[k] * w[ref] / (w[k] + w[ref]);
        J += sw2 * (ax - b) * (ax - b);
    }
    return J;
}

/* NLS の目的関数 Σ ρ(e/σ)。robust なら Huber (片側は k を k_pos_scale 倍)。
 * mask で落とされた観測 (chi2 / 物理ゲート) は除く。 */
static double objective_nls(const Scenario *scn, const MeasGen *mg, int nmeas,
                            const CfgGen *cg, unsigned long mask, int robust, const double p[3])
{
    double J = 0.0;
    int k;
    for (k = 0; k < nmeas; ++k) {
        const double *a = scn->p[mg[k].anchor];
        double s = meas_sigma(&mg[k]);
        double d, e, u;
        if (!scn->enabled[mg[k].anchor]) continue;
        if (k < 32 && (mask & (1UL << k))) continue;
        d = dist3d(p, a);
        e = mg[k].value - scn->antenna_delay[mg[k].anchor] - d;
        u = e / s;
        if (robust && cg->huber_k > 0.0) {
            double kk = cg->huber_k;
            double au = fabs(u);
            if (cg->one_sided && e > 0.0) kk *= cg->k_pos_scale;
            J += (au <= kk) ? 0.5 * u * u : kk * (au - 0.5 * kk);
        } else {
            J += 0.5 * u * u;
        }
    }
    return J;
}

/* 許容差を超えた位置差を、目的関数で「新が旧より悪くない」か判定する。
 * 悪くなければ report-only の metric (path) に回し、悪ければ失敗。 */
static void judge_by_objective(Metric *strict_m, Metric *path_m, Metric *worse_m,
                               double diff, double J_new, double J_ref, const char *ctx)
{
    if (diff <= strict_m->tol) {
        m_num(strict_m, diff, 1, ctx);
        return;
    }
    if (J_new <= J_ref * (1.0 + OBJ_REL) + OBJ_ABS) {
        /* 同じ (かそれ以上に良い) 目的関数値に別経路で到達した */
        if (diff > strict_m->max_diff) strict_m->max_diff = diff;
        ++strict_m->n;
        m_num(path_m, diff, 0, ctx);
    } else {
        char c2[160];
        snprintf(c2, sizeof(c2), "%s J_new=%.6g J_ref=%.6g", ctx, J_new, J_ref);
        m_num(worse_m, diff, 1, c2);
        m_num(strict_m, diff, 0, ctx);
    }
}

/* ------------------------------------------------------- スナップショット */

static void run_snapshot_trials(int trials)
{
    static const double noise_set[4] = {0.0, 0.02, 0.08, 0.3};
    FixMetrics fm_lv0 = make_fix_metrics_tol("Lv0", TOL_P_LLS);
    FixMetrics fm_lv1 = make_fix_metrics("Lv1");
    FixMetrics fm_lv2 = make_fix_metrics("Lv2");
    /* "~" 付きは報告のみのカテゴリ:
     *   Lv0~ / LLS~ : 同一平面判定 (固有値比 < 0.05²) が立つ配置。旧 LLS は
     *                 常に 1e-12 倍のリッジを足していて、条件数 κ が大きい
     *                 ほぼ同一平面の配置では κ·1e-12 の偏りが出る (最大 1cm)。
     *                 新実装は numpy lstsq と同じ最小ノルム解で、偏りは無い
     *   Lv1~ / Lv2~ : 新旧で Beck の成否が違う = NLS の初期値が違う試行。
     *                 完全同一平面では G が特異で旧 Beck はコレスキーが丸めで
     *                 たまたま通ることがあり、その (不良条件の) 解から始めた
     *                 NLS は別の局所解 / 鏡像に収束しうる */
    FixMetrics fm_lv0l = make_fix_metrics_tol("Lv0~", TOL_P_LLS);
    FixMetrics fm_lv1l = make_fix_metrics("Lv1~");
    FixMetrics fm_lv2l = make_fix_metrics("Lv2~");
    Metric *lv0_path   = metric("Lv0", "p_3d_path", NAN);
    Metric *lv0_worse  = metric("Lv0", "p_3d_worse", TOL_P_LLS);
    Metric *lv1_path   = metric("Lv1", "p_3d_path", NAN);
    Metric *lv1_worse  = metric("Lv1", "p_3d_worse", TOL_P);
    Metric *lv2_path   = metric("Lv2", "p_3d_path", NAN);
    Metric *lv2_worse  = metric("Lv2", "p_3d_worse", TOL_P);
    Metric *beck_path  = metric("Beck", "p_3d_path", NAN);
    Metric *beck_worse = metric("Beck", "p_3d_worse", TOL_P);
    Metric *lls_path   = metric("LLS", "p_3d_path", NAN);
    Metric *lls_worse  = metric("LLS", "p_3d_worse", TOL_P_LLS);
    Metric *beck_ok    = metric("Beck", "ok", NAN);
    Metric *beck_sing  = metric("Beck", "ok_coplanar", NAN);  /* 同一平面判定が立つ配置での成否不一致。報告のみ */
    Metric *beck_p     = metric("Beck", "p_3d", TOL_P);
    Metric *beck_pl    = metric("Beck~", "p_3d", NAN);   /* 同一平面判定が立つ配置 (不良条件)。報告のみ */
    Metric *lls_ok     = metric("LLS", "ok", NAN);
    Metric *lls_p      = metric("LLS", "p_3d", TOL_P_LLS);
    Metric *lls_pl     = metric("LLS~", "p_3d", NAN);
    Metric *gdop_ok    = metric("GDOP_at", "ok", NAN);
    Metric *gdop_rel   = metric("GDOP_at", "rel", TOL_RELCOV);
    Metric *crlb_ok    = metric("CRLB_at", "ok", NAN);
    Metric *crlb_rel   = metric("CRLB_at", "rel", TOL_RELCOV);
    Metric *cop_ok     = metric("Coplanar", "ok", NAN);
    Metric *cop_normal = metric("Coplanar", "normal_dot", TOL_P);
    Metric *cop_offset = metric("Coplanar", "offset", TOL_P);
    int t;

    for (t = 0; t < trials; ++t) {
        Scenario scn;
        double truth[3];
        double noise;
        MeasGen mg[MAXN];
        int nmeas;
        CfgGen cg;
        char ctx[64];
        uwb_anchor na[MAXN];
        ref_anchor ra[MAXN];
        uwb_meas nm[MAXN];
        ref_meas rm[MAXN];
        uwb_config ncfg;
        ref_cfg rcfg;
        int cop, cop_used, exact_flat, init_differs;

        gen_anchors(&scn);
        truth[0] = urand() * ROOM_X;
        truth[1] = urand() * ROOM_Y;
        truth[2] = urand() * ROOM_Z;
        noise = noise_set[(int)(urand() * 4.0) % 4];
        nmeas = gen_meas(&scn, truth, noise, mg);
        gen_cfg(truth, &cg);

        fill_new_anchors(&scn, na);
        fill_ref_anchors(&scn, ra);
        {
            int k;
            for (k = 0; k < nmeas; ++k) {
                nm[k].anchor  = mg[k].anchor;
                nm[k].value   = (uwb_real)mg[k].value;
                nm[k].sigma   = (uwb_real)mg[k].sigma;
                nm[k].quality = (uwb_real)mg[k].quality;
                rm[k].anchor  = mg[k].anchor;
                rm[k].value   = mg[k].value;
                rm[k].sigma   = mg[k].sigma;
                rm[k].quality = mg[k].quality;
            }
        }
        apply_new_cfg(&ncfg, na, scn.n_anchors, &cg);
        fill_ref_cfg(&rcfg, &cg);

        /* 環境変数 REGRESS_DUMP=<trial> でその試行の入力を印字する (原因調査用) */
        {
            const char *dump = getenv("REGRESS_DUMP");
            if (dump && atoi(dump) == t) {
                int k;
                printf("DUMP snap %d: n_anchors=%d dim=%d zf=%.6f zb=%d [%.3f,%.3f] huber=%.4f chi2=%.3f noise=%.2f truth=(%.6f,%.6f,%.6f)\n",
                       t, scn.n_anchors, cg.dim, cg.z_fixed, cg.use_z_bounds, cg.z_min, cg.z_max,
                       cg.huber_k, cg.chi2_k, noise, truth[0], truth[1], truth[2]);
                for (k = 0; k < scn.n_anchors; ++k)
                    printf("DUMP  anchor %d: (%.17g, %.17g, %.17g) en=%d delay=%.17g\n", k,
                           scn.p[k][0], scn.p[k][1], scn.p[k][2], scn.enabled[k], scn.antenna_delay[k]);
                for (k = 0; k < nmeas; ++k)
                    printf("DUMP  meas %d: anchor=%d value=%.17g sigma=%.17g quality=%.17g\n", k,
                           mg[k].anchor, mg[k].value, mg[k].sigma, mg[k].quality);
            }
        }

        /* 配置の性質: cop = 同一平面判定 (固有値比 < 0.05²)、
         * exact_flat = 有効アンカーの z が全部同じ (G が厳密に特異) */
        cop = uwb_anchors_coplanar(&ncfg, 0, 0);
        /* 観測に使われたアンカーだけの同一平面判定 (観測を落とした試行では
         * 全台では立体でも使った 4 台がほぼ同一平面、ということがある) */
        {
            uwb_anchor sub[MAXN];
            uwb_config subcfg;
            int k;
            for (k = 0; k < scn.n_anchors; ++k) { sub[k] = na[k]; sub[k].enabled = 0; }
            for (k = 0; k < nmeas; ++k) sub[mg[k].anchor].enabled = na[mg[k].anchor].enabled;
            uwb_config_init(&subcfg, sub, scn.n_anchors);
            cop_used = uwb_anchors_coplanar(&subcfg, 0, 0) || cop;
        }
        {
            int k, first = -1;
            exact_flat = 1;
            for (k = 0; k < scn.n_anchors; ++k) {
                if (!scn.enabled[k]) continue;
                if (first < 0) first = k;
                else if (scn.p[k][2] != scn.p[first][2]) exact_flat = 0;
            }
        }
        snprintf(ctx, sizeof(ctx), "snap %d cop=%d/%d flat=%d dim=%d zb=%d n=%d noise=%.2f",
                 t, cop, cop_used, exact_flat, cg.dim, cg.use_z_bounds, nmeas, noise);

        /* Beck (先に回して、新旧で成否が違う = 初期値が違う試行を知る) */
        {
            uwb_real np[3]; double rp[3];
            int nok = uwb_beck_gtrs(&ncfg, nm, nmeas, np);
            int rok = ref_beck(&rcfg, ra, scn.n_anchors, rm, nmeas, rp);
            init_differs = (nok != rok);
            /* 同一平面判定が立つ配置 (完全同一平面 = G が厳密に特異、または
             * ほぼ同一平面 = σ_min が丸め誤差の桁) では、旧 Beck の成否は丸め
             * 次第なので報告のみ。新実装は σ_min ≤ 64·eps·σ_max で明示的に
             * 失敗を返す (LLS に落ちる)。 */
            if (cop_used && cg.dim == 3) m_mismatch(beck_sing, nok, rok);
            else                         m_success(beck_ok, rok, nok, ctx);
            if (nok && rok) {
                double dx = (double)np[0] - rp[0];
                double dy = (double)np[1] - rp[1];
                double dz = (double)np[2] - rp[2];
                double dp[3];
                dp[0] = (double)np[0]; dp[1] = (double)np[1]; dp[2] = (double)np[2];
                if (cop_used) m_num(beck_pl, sqrt(dx * dx + dy * dy + dz * dz), 0, ctx);
                else judge_by_objective(beck_p, beck_path, beck_worse, sqrt(dx * dx + dy * dy + dz * dz),
                                        objective_beck(&scn, mg, nmeas, &cg, dp),
                                        objective_beck(&scn, mg, nmeas, &cg, rp), ctx);
            }
        }
        /* LLS */
        {
            uwb_real np[3]; double rp[3];
            int nok = uwb_lls_trilateration(&ncfg, nm, nmeas, np);
            int rok = ref_lls(&rcfg, ra, scn.n_anchors, rm, nmeas, rp);
            m_success(lls_ok, rok, nok, ctx);
            if (nok && rok) {
                double dx = (double)np[0] - rp[0];
                double dy = (double)np[1] - rp[1];
                double dz = (double)np[2] - rp[2];
                double dp[3];
                dp[0] = (double)np[0]; dp[1] = (double)np[1]; dp[2] = (double)np[2];
                if (cop_used) m_num(lls_pl, sqrt(dx * dx + dy * dy + dz * dz), 0, ctx);
                else          judge_by_objective(lls_p, lls_path, lls_worse, sqrt(dx * dx + dy * dy + dz * dz),
                                            objective_lls(&scn, mg, nmeas, &cg, dp),
                                            objective_lls(&scn, mg, nmeas, &cg, rp), ctx);
            }
        }
        /* Lv0/1/2 */
        {
            uwb_fix nf; ref_fix rf;
            ObjCheck oc;
            double dp[3];
            (void)uwb_solve_lv0(&ncfg, nm, nmeas, &nf);
            (void)ref_solve(0, &rcfg, ra, scn.n_anchors, rm, nmeas, &rf);
            dp[0] = (double)nf.p[0]; dp[1] = (double)nf.p[1]; dp[2] = (double)nf.p[2];
            oc.path = lv0_path; oc.worse = lv0_worse;
            oc.J_new = objective_lls(&scn, mg, nmeas, &cg, dp);
            oc.J_ref = objective_lls(&scn, mg, nmeas, &cg, rf.p);
            compare_fix(cop_used ? &fm_lv0l : &fm_lv0, &nf, &rf, !cop_used, &oc, ctx);
        }
        {
            uwb_fix nf; ref_fix rf;
            ObjCheck oc;
            double dp[3];
            int lenient;
            (void)uwb_solve_lv1(&ncfg, nm, nmeas, &nf);
            (void)ref_solve(1, &rcfg, ra, scn.n_anchors, rm, nmeas, &rf);
            lenient = init_differs || cop_used;
            dp[0] = (double)nf.p[0]; dp[1] = (double)nf.p[1]; dp[2] = (double)nf.p[2];
            oc.path = lv1_path; oc.worse = lv1_worse;
            oc.J_new = objective_nls(&scn, mg, nmeas, &cg, (unsigned long)nf.excluded, 0, dp);
            oc.J_ref = objective_nls(&scn, mg, nmeas, &cg, rf.excluded, 0, rf.p);
            compare_fix(lenient ? &fm_lv1l : &fm_lv1, &nf, &rf, !lenient, &oc, ctx);
        }
        {
            uwb_fix nf; ref_fix rf;
            ObjCheck oc;
            double dp[3];
            int lenient;
            (void)uwb_solve_lv2(&ncfg, nm, nmeas, &nf);
            (void)ref_solve(2, &rcfg, ra, scn.n_anchors, rm, nmeas, &rf);
            lenient = init_differs || cop_used;
            dp[0] = (double)nf.p[0]; dp[1] = (double)nf.p[1]; dp[2] = (double)nf.p[2];
            oc.path = lv2_path; oc.worse = lv2_worse;
            oc.J_new = objective_nls(&scn, mg, nmeas, &cg, (unsigned long)nf.excluded, 1, dp);
            oc.J_ref = objective_nls(&scn, mg, nmeas, &cg, rf.excluded, 1, rf.p);
            compare_fix(lenient ? &fm_lv2l : &fm_lv2, &nf, &rf, !lenient, &oc, ctx);
        }
        /* GDOP / CRLB (真値の位置で評価) */
        {
            uwb_real pt[3];
            double dpt[3];
            uwb_real ng, nc;
            double rg, rc;
            int nok, rok;
            int k;
            for (k = 0; k < 3; ++k) { pt[k] = (uwb_real)truth[k]; dpt[k] = truth[k]; }

            ng = uwb_gdop_at(&ncfg, pt);
            rg = ref_gdop_at(&rcfg, ra, scn.n_anchors, dpt);
            nok = ng > 0; rok = rg > 0;
            /* 同一平面配置で点が平面上にあると HᵀH が特異。旧 LU は巨大な
             * GDOP (1e7 など) を返し、新実装 (64 eps の行列式判定) は負を返す。
             * どちらも「使えない配置」の意味なので cop のときは報告のみ。 */
            if (cop || rg > 1e3) m_mismatch(gdop_ok, rok, nok);   /* 旧 1e3 超は実質「解けない」 */
            else                 m_success(gdop_ok, rok, nok, ctx);
            if (nok && rok) {
                double denom = (1.0 > fabs(rg)) ? 1.0 : fabs(rg);
                m_num(gdop_rel, fabs((double)ng - rg) / denom, 1, ctx);
            }

            nc = uwb_crlb_at(&ncfg, pt);
            rc = ref_crlb_at(&rcfg, ra, scn.n_anchors, dpt);
            nok = nc > 0; rok = rc > 0;
            if (cop || rc > 1e3) m_mismatch(crlb_ok, rok, nok);
            else                 m_success(crlb_ok, rok, nok, ctx);
            if (nok && rok) {
                double denom = (1.0 > fabs(rc)) ? 1.0 : fabs(rc);
                m_num(crlb_rel, fabs((double)nc - rc) / denom, 1, ctx);
            }
        }
        /* Coplanar */
        {
            uwb_real nn[3], noff;
            double rn[3], roff;
            int nok = uwb_anchors_coplanar(&ncfg, nn, &noff);
            int rok = ref_coplanar(&rcfg, ra, scn.n_anchors, rn, &roff);
            m_success(cop_ok, rok, nok, ctx);
            if (nok && rok) {
                /* 平面の単位法線には ±1 の符号自由度があり (n・p=offset と
                 * (-n)・p=(-offset) は同じ平面)、新旧が逆符号を選ぶことがある。
                 * normal_dot は |dot| で符号を無視して比較できるが、offset は
                 * 符号がずれたままだと (本来一致するはずなのに) 2*|offset| も
                 * 差が出て誤検出になるので、dot の符号に合わせて揃えてから比べる。 */
                double dot = (double)nn[0] * rn[0] + (double)nn[1] * rn[1] + (double)nn[2] * rn[2];
                double roff_aligned = (dot >= 0.0) ? roff : -roff;
                m_num(cop_normal, fabs(fabs(dot) - 1.0), 1, ctx);
                m_num(cop_offset, fabs((double)noff - roff_aligned), 1, ctx);
            }
        }
    }
}

/* ------------------------------------------------------------- EKF */

typedef struct {
    int    motion;   /* 0=CV, 1=CA */
    int    nd;       /* 2 or 3 */
    int    shape;    /* 0=円運動, 1=直線 */
    double dt_base;
    double center[3];
    double radius;
    double omega;
    double dir[3];
    double speed;
    double sigma_a;
} Traj;

static void gen_traj(Traj *tr)
{
    double ang;
    tr->motion = (urand() < 0.5) ? 0 : 1;
    tr->nd     = (urand() < 0.5) ? 2 : 3;
    tr->shape  = (urand() < 0.5) ? 0 : 1;
    tr->dt_base = 0.05 + urand() * 0.15;
    tr->center[0] = ROOM_X * 0.3 + urand() * ROOM_X * 0.4;
    tr->center[1] = ROOM_Y * 0.3 + urand() * ROOM_Y * 0.4;
    tr->center[2] = 0.5 + urand() * (ROOM_Z - 1.0);
    tr->radius = 0.5 + urand() * 2.0;
    tr->omega  = 0.1 + urand() * 0.5;
    ang = urand() * 6.283185307179586;
    tr->dir[0] = cos(ang); tr->dir[1] = sin(ang); tr->dir[2] = 0.0;
    tr->speed = 0.2 + urand() * 1.0;
    tr->sigma_a = 0.3 + urand() * 2.0;
}

static void traj_pos(const Traj *tr, double t, double p[3])
{
    if (tr->shape == 0) {
        p[0] = tr->center[0] + tr->radius * cos(tr->omega * t);
        p[1] = tr->center[1] + tr->radius * sin(tr->omega * t);
    } else {
        p[0] = tr->center[0] + tr->dir[0] * tr->speed * t;
        p[1] = tr->center[1] + tr->dir[1] * tr->speed * t;
    }
    p[2] = tr->center[2];
}

static void run_ekf_trials(int trajectories, int steps)
{
    static const double noise_set[4] = {0.02, 0.05, 0.08, 0.15};
    FixMetrics fm = make_fix_metrics_tol("EKF", TOL_P_EKF);
    /* EKF/path: 軌跡の途中で位置が TOL_P_EKF 以上ずれた (立ち上げのスナップ
     * ショットが別経路で収束した、ゲート判定が丸めで割れた) 以降のステップ。
     * フィルタは過去に依存するので、それ以降の一致は期待できない。報告のみ。
     * double では起きない (0 件であること)。 */
    FixMetrics fmp = make_fix_metrics_tol("EKF/path", TOL_P_EKF);
    Metric *diverged_traj = metric("EKF", "diverged_traj", NAN);  /* fail 欄 = 分岐した軌跡の数 */
    Metric *err_new = metric("EKF", "err_vs_truth_new", NAN);   /* 報告のみ: 真値との差 (新) */
    Metric *err_ref = metric("EKF", "err_vs_truth_ref", NAN);   /* 報告のみ: 真値との差 (旧) */
    Metric *closer_new = metric("EKF", "p_fail_new_closer", NAN); /* 位置差が許容超のうち新の方が真値に近い数 (fail 欄) */
    Metric *closer_ref = metric("EKF", "p_fail_ref_closer", NAN); /* 同、旧の方が近い数 */
    /* EKF~: 同一平面判定が立つ配置の軌跡。立ち上げの Lv2 が鏡像・平面上の
     * 退化解になりうるので報告のみ */
    FixMetrics fml = make_fix_metrics("EKF~");
    Metric *state_x  = metric("EKF", "state_x", NAN);
    Metric *state_P  = metric("EKF", "state_P", NAN);
    Metric *predict_x = metric("EKF", "predict_x", NAN);
    Metric *predict_P = metric("EKF", "predict_P", NAN);
    int trial;

    for (trial = 0; trial < trajectories; ++trial) {
        Scenario scn;
        Traj tr;
        uwb_anchor na[MAXN];
        ref_anchor ra[MAXN];
        uwb_config ncfg;
        ref_cfg rcfg;
        uwb_ekf nek;
        CfgGen cg;
        double noise = noise_set[(int)(urand() * 4.0) % 4];
        int nlos_start = -1, nlos_len = 0, nlos_anchor = 0;
        int rr_start = -1, rr_len = 0;
        int gap_step = -1;
        int teleport_step = -1;
        double teleport_to[3];
        double t = 0.0;
        int i, cop, diverged = 0;

        gen_anchors(&scn);
        gen_traj(&tr);

        cfg_defaults(&cg);
        cg.dim = tr.nd;
        cg.z_fixed = tr.center[2];

        fill_new_anchors(&scn, na);
        fill_ref_anchors(&scn, ra);
        apply_new_cfg(&ncfg, na, scn.n_anchors, &cg);
        fill_ref_cfg(&rcfg, &cg);

        cop = uwb_anchors_coplanar(&ncfg, 0, 0);
        uwb_ekf_init(&nek, &ncfg, (uwb_motion)tr.motion, (uwb_real)tr.sigma_a);
        (void)ref_ekf_init(0, &rcfg, ra, scn.n_anchors, tr.motion, tr.sigma_a);

        if (urand() < 0.30 && steps > 30) {
            nlos_start = 10 + (int)(urand() * (steps - 30));
            nlos_len = 5 + (int)(urand() * 15.0);
            nlos_anchor = (int)(urand() * scn.n_anchors);
            if (nlos_anchor >= scn.n_anchors) nlos_anchor = scn.n_anchors - 1;
        }
        if (urand() < 0.30 && steps > 30) {
            rr_start = 10 + (int)(urand() * (steps - 30));
            rr_len = 5 + (int)(urand() * 15.0);
        }
        if (urand() < 0.20 && steps > 10) gap_step = 5 + (int)(urand() * (steps - 10));
        teleport_to[0] = teleport_to[1] = teleport_to[2] = 0.0;
        if (urand() < 0.15 && steps > 30) {
            teleport_step = 20 + (int)(urand() * (steps - 30));
            teleport_to[0] = urand() * ROOM_X;
            teleport_to[1] = urand() * ROOM_Y;
            teleport_to[2] = 0.5 + urand() * (ROOM_Z - 1.0);
        }

        for (i = 0; i < steps; ++i) {
            double dt = tr.dt_base;
            double truth[3];
            uwb_meas nm[MAXN];
            ref_meas rm[MAXN];
            int n_meas;
            uwb_fix nf; ref_fix rf;
            char ctx[128];

            if (i == gap_step) dt = 3.0;   /* 既定 max_dt=2s を超えるギャップ */
            t += dt;

            if (teleport_step >= 0 && i >= teleport_step) {
                truth[0] = teleport_to[0];
                truth[1] = teleport_to[1];
                truth[2] = teleport_to[2];
            } else {
                traj_pos(&tr, t, truth);
                if (truth[0] < 0.0) truth[0] = 0.0;
                if (truth[0] > ROOM_X) truth[0] = ROOM_X;
                if (truth[1] < 0.0) truth[1] = 0.0;
                if (truth[1] > ROOM_Y) truth[1] = ROOM_Y;
                if (truth[2] < 0.1) truth[2] = 0.1;
                if (truth[2] > ROOM_Z - 0.1) truth[2] = ROOM_Z - 0.1;
            }

            /* 直接 predict も試す (低確率)。 */
            if (urand() < 0.05) {
                double extra = 0.01 + urand() * 0.04;
                int rnx, rinit, ramb, rsk, rside;
                double rx[9], rP[81];
                uwb_ekf_predict(&nek, (uwb_real)extra);
                ref_ekf_predict(0, extra);
                ref_ekf_state(0, &rnx, rx, rP, &rinit, &ramb, &rsk, &rside);
                if (rnx == nek.nx) {
                    int k;
                    double maxdx = 0.0, maxdP = 0.0;
                    for (k = 0; k < nek.nx; ++k) {
                        double d = fabs((double)nek.x[k] - rx[k]);
                        if (d > maxdx) maxdx = d;
                    }
                    for (k = 0; k < nek.nx * nek.nx; ++k) {
                        double d = fabs((double)nek.P[k] - rP[k]);
                        if (d > maxdP) maxdP = d;
                    }
                    m_num(predict_x, maxdx, 0, "predict");
                    m_num(predict_P, maxdP, 0, "predict");
                }
            }

            if (rr_start >= 0 && i >= rr_start && i < rr_start + rr_len) {
                int a = i % scn.n_anchors;
                double d = dist3d(truth, scn.p[a]) + scn.antenna_delay[a] + noise * nrand();
                if (nlos_start >= 0 && i >= nlos_start && i < nlos_start + nlos_len && a == nlos_anchor)
                    d += 1.0;
                nm[0].anchor = a; nm[0].value = (uwb_real)d; nm[0].sigma = (uwb_real)0; nm[0].quality = (uwb_real)-1;
                rm[0].anchor = a; rm[0].value = d; rm[0].sigma = 0.0; rm[0].quality = -1.0;
                n_meas = 1;
            } else {
                int k, m2 = 0;
                for (k = 0; k < scn.n_anchors; ++k) {
                    double d;
                    if (!scn.enabled[k]) continue;
                    d = dist3d(truth, scn.p[k]) + scn.antenna_delay[k] + noise * nrand();
                    if (nlos_start >= 0 && i >= nlos_start && i < nlos_start + nlos_len && k == nlos_anchor)
                        d += 1.0;
                    nm[m2].anchor = k; nm[m2].value = (uwb_real)d; nm[m2].sigma = (uwb_real)0; nm[m2].quality = (uwb_real)-1;
                    rm[m2].anchor = k; rm[m2].value = d; rm[m2].sigma = 0.0; rm[m2].quality = -1.0;
                    ++m2;
                }
                n_meas = m2;
            }

            (void)uwb_ekf_update(&nek, (uwb_real)t, nm, n_meas, &nf);
            (void)ref_ekf_update(0, t, rm, n_meas, &rf);

            {
                double dn[3], dr[3], en = -1.0, er = -1.0;
                int k;
                for (k = 0; k < 3; ++k) { dn[k] = (double)nf.p[k]; dr[k] = rf.p[k]; }
                if (nf.ok && rf.ok) { en = dist3d(dn, truth); er = dist3d(dr, truth); }
                snprintf(ctx, sizeof(ctx), "traj %d step %d cop=%d nd=%d mo=%d n=%d e_new=%.3f e_ref=%.3f",
                         trial, i, cop, tr.nd, tr.motion, n_meas, en, er);
                if (!cop && !diverged && nf.ok && rf.ok && dist3d(dn, dr) > TOL_P_EKF) {
                    diverged = 1;
                    m_mismatch(diverged_traj, 0, 1);
                }
                compare_fix(cop ? &fml : (diverged ? &fmp : &fm), &nf, &rf, !cop && !diverged, 0, ctx);
                if (nf.ok && rf.ok) {
                    m_num(err_new, en, 0, ctx);
                    m_num(err_ref, er, 0, ctx);
                    if (!cop && dist3d(dn, dr) > TOL_P_EKF) {
                        if (en < er) m_mismatch(closer_new, 0, 1);
                        else         m_mismatch(closer_ref, 0, 1);
                    }
                }
            }

            {
                int rnx, rinit, ramb, rsk, rside;
                double rx[9], rP[81];
                ref_ekf_state(0, &rnx, rx, rP, &rinit, &ramb, &rsk, &rside);
                if (rnx == nek.nx) {
                    int k;
                    double maxdx = 0.0, maxdP = 0.0;
                    for (k = 0; k < nek.nx; ++k) {
                        double d = fabs((double)nek.x[k] - rx[k]);
                        if (d > maxdx) maxdx = d;
                    }
                    for (k = 0; k < nek.nx * nek.nx; ++k) {
                        double d = fabs((double)nek.P[k] - rP[k]);
                        if (d > maxdP) maxdP = d;
                    }
                    m_num(state_x, maxdx, 0, ctx);
                    m_num(state_P, maxdP, 0, ctx);
                }
            }
        }
    }
}

/* ------------------------------------------------------------- 表 */

static void print_table(void)
{
    int i;
    printf("\n%-9s %-13s %11s %10s %8s %6s %8s\n",
           "category", "metric", "max_diff", "tol", "n", "fail", "improve");
    for (i = 0; i < g_nmetrics; ++i) {
        Metric *m = &g_metrics[i];
        char tolbuf[16], improvebuf[16];
        if (isnan(m->tol)) snprintf(tolbuf, sizeof(tolbuf), "-");
        else snprintf(tolbuf, sizeof(tolbuf), "%.3g", m->tol);
        if (m->improve < 0) snprintf(improvebuf, sizeof(improvebuf), "-");
        else snprintf(improvebuf, sizeof(improvebuf), "%d", m->improve);
        printf("%-9s %-13s %11.3e %10s %8d %6d %8s\n",
               m->cat, m->name, m->max_diff, tolbuf, m->n, m->fail, improvebuf);
    }
    printf("\n");
}

/* ------------------------------------------------------------- main */

int main(void)
{
    printf("uwb_loc 新旧比較の回帰テスト (%s)\n", UWB_REAL_IS_FLOAT ? "float" : "double");
    printf("参照実装: tests/host/loc/ref (上流 uwb_localizer 凍結版、commit 4298c08)\n");
    printf("----------------------------------------------------------\n");

    run_snapshot_trials(2000);
    run_ekf_trials(1200, 100);

    print_table();

    printf("----------------------------------------------------------\n");
    if (g_fail == 0) {
        printf("OK  %d 件すべて通った\n", g_run);
        return 0;
    }
    printf("NG  %d / %d 件が失敗\n", g_fail, g_run);
    return 1;
}
