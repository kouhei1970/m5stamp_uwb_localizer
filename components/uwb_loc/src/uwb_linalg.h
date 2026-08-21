/* 小さな密行列だけの線形代数。BLAS も LAPACK も要らない。
 *
 * ここで扱う最大の行列は Beck 法の 4x4 と EKF の 9x9。どれも
 * 部分ピボット付き LU で足りる。行優先 (row-major) で持つ。
 */
#ifndef UWB_LINALG_H
#define UWB_LINALG_H

#include "uwb_loc.h"

/* 作業行列の最大次数。Beck 法 (dim+1=4) と EKF (3*3=9) が上限。 */
#define UWB_LA_MAX 9

/** a[n*n] x = b[n] を部分ピボット付き LU で解く。x に書く (b は保持)。
 *  特異なら 0、解けたら 1。a は破壊される。 */
int uwb_solve_lin(uwb_real *a, const uwb_real *b, uwb_real *x, int n);

/** 対称正定値行列の逆行列。inv[n*n] に書く。失敗したら 0。
 *  a は破壊される。 */
int uwb_inverse_spd(uwb_real *a, uwb_real *inv, int n);

/** 一般の正方行列の逆行列 (LU)。失敗したら 0。a は破壊される。 */
int uwb_inverse(uwb_real *a, uwb_real *inv, int n);

/** 対称行列のコレスキー分解 a = L L^T。L を下三角に書く (a を上書き)。
 *  正定値でなければ 0。 */
int uwb_cholesky(uwb_real *a, int n);

/** 対称行列の固有値・固有ベクトルを Jacobi 法で求める。eig[n] に昇順で書く。
 *  vec が非NULL なら vec[n*n] に固有ベクトルを「列ベクトル」として、
 *  eig と同じ昇順で書く (vec[row*n+col] は固有ベクトル col の第 row 成分)。
 *  vec が NULL なら固有ベクトルは計算しない(従来と同じ)。
 *  a は破壊される。収束しなければ 0。 */
int uwb_sym_eig(uwb_real *a, uwb_real *eig, uwb_real *vec, int n);

/** 対称行列の固有値を Jacobi 法で求める。eig[n] に昇順で書く。
 *  a は破壊される。収束しなければ 0。 */
int uwb_sym_eigvals(uwb_real *a, uwb_real *eig, int n);

/** c = a^T diag(w) a  (a は m x n、c は n x n)。w が NULL なら重み 1。 */
void uwb_ata_weighted(const uwb_real *a, const uwb_real *w, int m, int n, uwb_real *c);

/** y = a^T diag(w) b  (a は m x n、b は m、y は n)。w が NULL なら重み 1。 */
void uwb_atb_weighted(const uwb_real *a, const uwb_real *w, const uwb_real *b,
                      int m, int n, uwb_real *y);

#endif /* UWB_LINALG_H */
