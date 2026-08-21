/* ref_bridge — 参照実装 (ref/uwb_loc.h) をプレーン C の値渡し API で包む。
 *
 * test_regress.c は新実装の "uwb_loc.h" (components/uwb_loc/include/) を
 * include する。参照実装の型 (ref/uwb_loc.h の uwb_config, uwb_anchor, ...)
 * は名前こそ同じだが定義が別の翻訳単位にあり、test_regress.c の中で
 * 両方をヘッダとして持ち込むと構造体タグが衝突する。
 *
 * そこでこの橋渡し層だけが ref 配下の .h を include し (ref_bridge.c 側)、
 * ここ (ref_bridge.h) は double / int だけのプレーンな構造体しか出さない。
 * test_regress.c から見ると「もう一つの測位ライブラリの C API」に見える。
 *
 * 単位・意味は新実装 (uwb_loc.h) と同じ。id はここでは持たず、
 * 内部で "A%d" (%d は配列添字) を割り当てる。
 */
#ifndef UWB_REF_BRIDGE_H
#define UWB_REF_BRIDGE_H

#ifdef __cplusplus
extern "C" {
#endif

/* テストが生成するアンカー/観測は最大 8 本 (Kconfig 既定 UWB_MAX_ANCHORS=8
 * / UWB_MAX_MEAS=8 に合わせてある。tests/host/loc/Makefile の
 * ANCHORS8_DEFS 参照)。参照実装側の内部上限とは独立の、この橋渡し層
 * 自体の受け渡しバッファの大きさ。 */
#define REF_MAX_ANCHORS 8
#define REF_MAX_MEAS    8

typedef struct {
    double p[3];
    int    enabled;
    double antenna_delay_m;
    double sigma0;
    double sigma_per_m;
} ref_anchor;

typedef struct {
    int    anchor;
    double value;
    double sigma;
    double quality;
} ref_meas;

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
} ref_cfg;

typedef struct {
    double p[3];
    double cov[9];
    double v[3];
    int    ok;
    int    n_used;
    int    n_total;
    int    iterations;
    int    ambiguous;
    double residual_rms;
    double gdop;
    double sigma;
    unsigned long excluded;
} ref_fix;

/* レベル 0/1/2 のスナップショット測位。level は 0, 1, 2 のいずれか。 */
int ref_solve(int level, const ref_cfg *cfg, const ref_anchor *anchors, int na,
             const ref_meas *meas, int n, ref_fix *out);

/* Beck 厳密解そのもの。 */
int ref_beck(const ref_cfg *cfg, const ref_anchor *anchors, int na,
            const ref_meas *meas, int n, double out[3]);

/* 基準アンカー差分の線形最小二乗そのもの。 */
int ref_lls(const ref_cfg *cfg, const ref_anchor *anchors, int na,
           const ref_meas *meas, int n, double out[3]);

double ref_gdop_at(const ref_cfg *cfg, const ref_anchor *anchors, int na,
                   const double pt[3]);

double ref_crlb_at(const ref_cfg *cfg, const ref_anchor *anchors, int na,
                   const double pt[3]);

int ref_coplanar(const ref_cfg *cfg, const ref_anchor *anchors, int na,
                 double normal[3], double *offset);

/* ---------------------------------------------------------------- EKF */
/* motion: 0 = CV, 1 = CA (uwb_motion と同じ符号付け)。
 * h はハンドル (0..3)。参照実装の uwb_ekf / uwb_config / アンカー配列を
 * ハンドルごとに static に保持する (ステートフル)。 */
int ref_ekf_init(int h, const ref_cfg *cfg, const ref_anchor *anchors, int na,
                 int motion, double sigma_a);
void ref_ekf_set_params(int h, double gate, double max_dt, int max_rejects);
void ref_ekf_predict(int h, double dt);
int ref_ekf_update(int h, double t, const ref_meas *meas, int n, ref_fix *out);

/* 状態のスナップショット取得 (比較用)。x/P は UWB_MAX_STATE=9 前提の
 * 呼び出し側バッファ。使われている範囲は nx / nx*nx だけ。 */
void ref_ekf_state(int h, int *nx, double x[9], double P[81],
                   int *initialized, int *ambiguous, int *side_known, int *side);

/* ------------------------------------------------------- Jacobi 固有値 */
/* 対称行列の固有値・固有ベクトル (Jacobi 法)。test_uwb.c の
 * test_sym_eig_vectors が使う。a[n*n] は破壊される。vec は列ベクトル形式
 * (uwb_sym_eig と同じレイアウト)。失敗したら 0。 */
int ref_sym_eig(double *a, double *eig, double *vec, int n);

#ifdef __cplusplus
}
#endif

#endif /* UWB_REF_BRIDGE_H */
