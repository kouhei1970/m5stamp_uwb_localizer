/* 測量専用の密行列ソルバ（docs/SURVEY_SPEC.md の S2）。
 *
 * components/uwb_loc/src/uwb_linalg.{c,h} は UWB_LA_MAX = 9 が上限で、
 * 測量の正規方程式（最大 25x25 = 3*8+1）が入らない。かといって uwb_loc は
 * 測位のホットパスなので上限を上げると全体の静的配列が太る。そこで
 * **測量側に必要なぶんだけ**持つ。
 *
 * 入っているのは 2 つだけ:
 *   - コレスキー分解による対称正定値ソルバ（正規方程式用、最大 25x25）
 *   - 固有ベクトル付きの対称固有値分解（MDS の二重中心化行列 8x8 と、
 *     高さフィットの 3x3 用）
 *
 * uwb_linalg には固有ベクトルを返す `uwb_sym_eig()` は **存在しない**
 * （あるのは固有値だけの `uwb_sym_eigvals()`）。仕様書 §2[2] の
 * 「上流の最適化 C-1 で固有ベクトル対応済み」は本リポジトリの現状と
 * 合っていない。uwb_loc は変更禁止なので、ここに自前で置く。
 *
 * 行優先 (row-major)。malloc しない。
 */
#ifndef UWB_SURVEY_DENSE_H
#define UWB_SURVEY_DENSE_H

#include "uwb_survey.h"

/** 固有値分解の最大次数。MDS の Gram 行列（ノード数ぶん）が上限。 */
#define UWB_SURVEY_EIG_MAX UWB_SURVEY_MAX_NODES

/**
 * 対称正定値 a[n*n] に対して a x = b を解く（コレスキー分解）。
 *
 * a は破壊される（下三角に L が残る）。**a の下三角しか読まない**ので、
 * 対称行列の上三角が埋まっていなくてもよい。
 * 正定値でない（対角に非正のピボットが出た）か NaN が出たら 0、
 * 解けたら 1 を返す。
 *
 * n は 1..UWB_SURVEY_MAX_UNKNOWNS。
 */
int uwb_survey_chol_solve(uwb_real *a, const uwb_real *b, uwb_real *x, int n);

/**
 * 対称行列 a[n*n] の固有値と固有ベクトルを循環 Jacobi 法で求める。
 *
 *   eig[k] : 固有値。**降順**（eig[0] が最大）
 *   vec    : n x n 行優先。第 k 固有ベクトルは **列** k、
 *            すなわち vec[i*n + k] が i 番目の成分
 *
 * a は破壊される。NaN が出たら 0、それ以外は 1。
 * 符号は「絶対値最大の成分が正」になるよう正規化してあるので、
 * 同じ入力に対して常に同じ結果になる（再現性のため）。
 *
 * n は 1..UWB_SURVEY_EIG_MAX。
 */
int uwb_survey_sym_eig(uwb_real *a, uwb_real *eig, uwb_real *vec, int n);

#endif /* UWB_SURVEY_DENSE_H */
