/* ライブラリ内部の共有部分。公開 API には出さない。 */
#ifndef UWB_INTERNAL_H
#define UWB_INTERNAL_H

#include "uwb_loc.h"

#define UWB_EPS ((uwb_real)1e-12)

/** 有効な観測か (アンカーの添字が範囲内で、そのアンカーが enabled)。 */
int uwb_meas_usable(const uwb_config *cfg, const uwb_meas *m);

/** アンテナ遅延を引いた測距値。 */
uwb_real uwb_corrected(const uwb_config *cfg, const uwb_meas *m);

/** その観測の 1sigma。measurement.sigma が正ならそれを使い、
 *  無ければアンカーの sigma0 + sigma_per_m * d から作る。
 *  quality が 0-1 で与えられていれば sigma を (1 + 3(1-q)) 倍する
 *  (Python 版の MeasurementModel.sigma と同じ)。 */
uwb_real uwb_sigma_of(const uwb_config *cfg, const uwb_meas *m, uwb_real distance);

/** 観測 1 本を評価する。
 *  residual = 観測値 - 予測値 (NLOS なら正)、jac は予測値の位置微分 dh/dp。 */
void uwb_evaluate(const uwb_config *cfg, const uwb_real *p, const uwb_meas *m,
                  uwb_real *residual, uwb_real *jac3, uwb_real *sigma);

/** 自由変数のマスク。dim==2 なら z を固定するので 2 個。 */
int uwb_n_free(const uwb_config *cfg);

/** dim==2 のとき z を z_fixed に、z_bounds があれば範囲に収める。 */
void uwb_project(const uwb_config *cfg, uwb_real *p);

/** ヤコビアンから GDOP を出す (単位化した行の (H^T H)^-1 のトレース)。
 *  解けなければ負を返す。 */
uwb_real uwb_gdop_from_jac(const uwb_config *cfg, const uwb_real *jac, int n);

/** 鏡像解を片側に寄せる。side_known/side は呼び出し側が持つ (EKF 用)。
 *  戻り値 1 なら「どちらか決められなかった」= ambiguous。 */
int uwb_resolve_mirror(const uwb_config *cfg, uwb_real *p, uwb_real *cov,
                       int side_known, int side);

/** 平面の法線に対する符号 (鏡像のどちら側か)。平面が無ければ 0。 */
int uwb_mirror_side(const uwb_config *cfg, const uwb_real *p);

/** fix を「失敗」で埋める。 */
void uwb_fix_failed(uwb_fix *out, int n_total);

/** cov から sigma (= sqrt(trace)) を埋める。 */
void uwb_fix_finish(uwb_fix *out);

#endif /* UWB_INTERNAL_H */
