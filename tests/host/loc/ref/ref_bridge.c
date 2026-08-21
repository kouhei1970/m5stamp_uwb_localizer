/* ref_bridge — ref_bridge.h (プレーン C API) を参照実装 (ref/uwb_loc.h) で
 * 実装する。
 *
 * このファイルだけが ref 配下の .h を include する。ref 配下の .c と同じく
 * `-include ref_prefix.h`付きでコンパイルされ、呼んでいる uwb_config_init
 * などはプリプロセッサで ref_uwb_config_init に化けるので、ref 配下の .c 側の
 * (同じフラグでビルドされた) 実体とリンクできる。
 *
 * "uwb_loc.h" / "uwb_internal.h" / "uwb_linalg.h" は quote 形式の
 * #include がまず「このファイル自身のディレクトリ (ref/)」を探すので、
 * test_regress.c 側が -I で通す新実装の components/uwb_loc/include/ とは
 * 衝突しない (Makefile 側もこの .c だけは新実装の INCLUDES を渡さない)。
 */
#include "uwb_loc.h"
#include "uwb_linalg.h"
#include "ref_bridge.h"

#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------ 変換補助 */

static void build_anchors(const ref_anchor *ra, int na, uwb_anchor *out)
{
    int i, n = na;
    if (n > REF_MAX_ANCHORS) n = REF_MAX_ANCHORS;
    if (n < 0) n = 0;
    for (i = 0; i < n; ++i) {
        int k;
        snprintf(out[i].id, sizeof(out[i].id), "A%d", i);
        for (k = 0; k < 3; ++k) out[i].p[k] = (uwb_real)ra[i].p[k];
        out[i].enabled         = ra[i].enabled;
        out[i].antenna_delay_m = (uwb_real)ra[i].antenna_delay_m;
        out[i].sigma0          = (uwb_real)ra[i].sigma0;
        out[i].sigma_per_m     = (uwb_real)ra[i].sigma_per_m;
    }
}

static int build_meas(const ref_meas *rm, int n, uwb_meas *out)
{
    int i, m = n;
    if (m > REF_MAX_MEAS) m = REF_MAX_MEAS;
    if (m < 0) m = 0;
    for (i = 0; i < m; ++i) {
        out[i].anchor  = rm[i].anchor;
        out[i].value   = (uwb_real)rm[i].value;
        out[i].sigma   = (uwb_real)rm[i].sigma;
        out[i].quality = (uwb_real)rm[i].quality;
    }
    return m;
}

/* uwb_config_init で既定値を入れてから ref_cfg の全項目で上書きする。 */
static void apply_cfg(uwb_config *cfg, const uwb_anchor *anchors, int na, const ref_cfg *rc)
{
    uwb_config_init(cfg, anchors, na);
    if (!rc) return;
    cfg->dim           = rc->dim;
    cfg->z_fixed       = (uwb_real)rc->z_fixed;
    cfg->use_z_bounds  = rc->use_z_bounds;
    cfg->z_min         = (uwb_real)rc->z_min;
    cfg->z_max         = (uwb_real)rc->z_max;
    cfg->max_iter      = rc->max_iter;
    cfg->tol           = (uwb_real)rc->tol;
    cfg->huber_k       = (uwb_real)rc->huber_k;
    cfg->k_pos_scale   = (uwb_real)rc->k_pos_scale;
    cfg->one_sided     = rc->one_sided;
    cfg->chi2_k        = (uwb_real)rc->chi2_k;
    cfg->physical_gate = rc->physical_gate;
    cfg->max_range     = (uwb_real)rc->max_range;
}

static void fix_to_ref(const uwb_fix *f, ref_fix *out)
{
    int k;
    for (k = 0; k < 3; ++k) out->p[k] = (double)f->p[k];
    for (k = 0; k < 9; ++k) out->cov[k] = (double)f->cov[k];
    for (k = 0; k < 3; ++k) out->v[k] = (double)f->v[k];
    out->ok          = f->ok;
    out->n_used      = f->n_used;
    out->n_total     = f->n_total;
    out->iterations  = f->iterations;
    out->ambiguous   = f->ambiguous;
    out->residual_rms = (double)f->residual_rms;
    out->gdop        = (double)f->gdop;
    out->sigma       = (double)f->sigma;
    out->excluded    = f->excluded;
}

/* ------------------------------------------------------- スナップショット */

int ref_solve(int level, const ref_cfg *rc, const ref_anchor *ra, int na,
             const ref_meas *rm, int n, ref_fix *out)
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_meas   meas[REF_MAX_MEAS];
    uwb_config cfg;
    uwb_fix    fix;
    int mm, ok;

    build_anchors(ra, na, anchors);
    mm = build_meas(rm, n, meas);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);

    switch (level) {
    case 0:  ok = uwb_solve_lv0(&cfg, meas, mm, &fix); break;
    case 1:  ok = uwb_solve_lv1(&cfg, meas, mm, &fix); break;
    default: ok = uwb_solve_lv2(&cfg, meas, mm, &fix); break;
    }
    fix_to_ref(&fix, out);
    return ok;
}

int ref_beck(const ref_cfg *rc, const ref_anchor *ra, int na,
            const ref_meas *rm, int n, double out[3])
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_meas   meas[REF_MAX_MEAS];
    uwb_config cfg;
    uwb_real   p[3] = {0, 0, 0};
    int mm, ok, k;

    build_anchors(ra, na, anchors);
    mm = build_meas(rm, n, meas);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);

    ok = uwb_beck_gtrs(&cfg, meas, mm, p);
    for (k = 0; k < 3; ++k) out[k] = (double)p[k];
    return ok;
}

int ref_lls(const ref_cfg *rc, const ref_anchor *ra, int na,
           const ref_meas *rm, int n, double out[3])
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_meas   meas[REF_MAX_MEAS];
    uwb_config cfg;
    uwb_real   p[3] = {0, 0, 0};
    int mm, ok, k;

    build_anchors(ra, na, anchors);
    mm = build_meas(rm, n, meas);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);

    ok = uwb_lls_trilateration(&cfg, meas, mm, p);
    for (k = 0; k < 3; ++k) out[k] = (double)p[k];
    return ok;
}

double ref_gdop_at(const ref_cfg *rc, const ref_anchor *ra, int na, const double pt[3])
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_config cfg;
    uwb_real   p[3];
    int k;

    build_anchors(ra, na, anchors);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);
    for (k = 0; k < 3; ++k) p[k] = (uwb_real)pt[k];
    return (double)uwb_gdop_at(&cfg, p);
}

double ref_crlb_at(const ref_cfg *rc, const ref_anchor *ra, int na, const double pt[3])
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_config cfg;
    uwb_real   p[3];
    int k;

    build_anchors(ra, na, anchors);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);
    for (k = 0; k < 3; ++k) p[k] = (uwb_real)pt[k];
    return (double)uwb_crlb_at(&cfg, p);
}

int ref_coplanar(const ref_cfg *rc, const ref_anchor *ra, int na,
                 double normal[3], double *offset)
{
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_config cfg;
    uwb_real   nrm[3] = {0, 0, 0}, off = 0;
    int k, ok;

    build_anchors(ra, na, anchors);
    apply_cfg(&cfg, anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);
    ok = uwb_anchors_coplanar(&cfg, nrm, &off);
    if (normal) for (k = 0; k < 3; ++k) normal[k] = (double)nrm[k];
    if (offset) *offset = (double)off;
    return ok;
}

/* ---------------------------------------------------------------- EKF */

typedef struct {
    uwb_anchor anchors[REF_MAX_ANCHORS];
    uwb_config cfg;
    uwb_ekf    ekf;
    int        active;
} ref_ekf_slot;

#define REF_EKF_HANDLES 4
static ref_ekf_slot g_ekf[REF_EKF_HANDLES];

static ref_ekf_slot *ekf_slot(int h)
{
    if (h < 0 || h >= REF_EKF_HANDLES) return 0;
    return &g_ekf[h];
}

int ref_ekf_init(int h, const ref_cfg *rc, const ref_anchor *ra, int na,
                 int motion, double sigma_a)
{
    ref_ekf_slot *s = ekf_slot(h);
    if (!s) return 0;
    build_anchors(ra, na, s->anchors);
    apply_cfg(&s->cfg, s->anchors, na > REF_MAX_ANCHORS ? REF_MAX_ANCHORS : na, rc);
    uwb_ekf_init(&s->ekf, &s->cfg, (uwb_motion)motion, (uwb_real)sigma_a);
    s->active = 1;
    return 1;
}

void ref_ekf_set_params(int h, double gate, double max_dt, int max_rejects)
{
    ref_ekf_slot *s = ekf_slot(h);
    if (!s || !s->active) return;
    s->ekf.gate        = (uwb_real)gate;
    s->ekf.max_dt      = (uwb_real)max_dt;
    s->ekf.max_rejects = max_rejects;
}

void ref_ekf_predict(int h, double dt)
{
    ref_ekf_slot *s = ekf_slot(h);
    if (!s || !s->active) return;
    uwb_ekf_predict(&s->ekf, (uwb_real)dt);
}

int ref_ekf_update(int h, double t, const ref_meas *rm, int n, ref_fix *out)
{
    ref_ekf_slot *s = ekf_slot(h);
    uwb_meas meas[REF_MAX_MEAS];
    uwb_fix  fix;
    int mm, ok;

    if (!s || !s->active) {
        memset(out, 0, sizeof(*out));
        return 0;
    }
    mm = build_meas(rm, n, meas);
    ok = uwb_ekf_update(&s->ekf, (uwb_real)t, meas, mm, &fix);
    fix_to_ref(&fix, out);
    return ok;
}

void ref_ekf_state(int h, int *nx, double x[9], double P[81],
                   int *initialized, int *ambiguous, int *side_known, int *side)
{
    ref_ekf_slot *s = ekf_slot(h);
    int i;

    if (!s || !s->active) {
        if (nx) *nx = 0;
        if (initialized) *initialized = 0;
        if (ambiguous) *ambiguous = 0;
        if (side_known) *side_known = 0;
        if (side) *side = 0;
        return;
    }
    if (nx) *nx = s->ekf.nx;
    if (x) for (i = 0; i < UWB_MAX_STATE; ++i) x[i] = (double)s->ekf.x[i];
    if (P) for (i = 0; i < UWB_MAX_STATE * UWB_MAX_STATE; ++i) P[i] = (double)s->ekf.P[i];
    if (initialized) *initialized = s->ekf.initialized;
    if (ambiguous)   *ambiguous   = s->ekf.ambiguous;
    if (side_known)  *side_known  = s->ekf.side_known;
    if (side)        *side        = s->ekf.side;
}

/* ------------------------------------------------------- Jacobi 固有値 */

int ref_sym_eig(double *a, double *eig, double *vec, int n)
{
    uwb_real la[UWB_LA_MAX * UWB_LA_MAX];
    uwb_real leig[UWB_LA_MAX];
    uwb_real lvec[UWB_LA_MAX * UWB_LA_MAX];
    int i, ok;

    if (n <= 0 || n > UWB_LA_MAX) return 0;
    for (i = 0; i < n * n; ++i) la[i] = (uwb_real)a[i];

    ok = uwb_sym_eig(la, leig, vec ? lvec : 0, n);
    if (!ok) return 0;

    for (i = 0; i < n; ++i) eig[i] = (double)leig[i];
    if (vec) for (i = 0; i < n * n; ++i) vec[i] = (double)lvec[i];
    /* uwb_sym_eig の仕様通り a も破壊されているので、呼び出し側に伝える。 */
    for (i = 0; i < n * n; ++i) a[i] = (double)la[i];
    return 1;
}
