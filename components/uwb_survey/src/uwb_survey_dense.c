#include "uwb_survey_dense.h"

#include <math.h>

#define DIDX(i, j, n) ((i) * (n) + (j))

static uwb_real dabs(uwb_real v) { return v < (uwb_real)0 ? -v : v; }

/* ------------------------------------------------------------ Cholesky */

/* a = L L^T の下三角 L を a に上書きしてから、前進代入・後退代入で解く。
 *
 * 正規方程式 (J^T J + lambda*diag) は必ず対称かつ（リッジのおかげで）
 * 正定値なので、部分ピボットは要らない。LU の半分の演算量で済むうえ、
 * 「対角が非正になったら失敗」がそのまま正定値性の検査になる。 */
int uwb_survey_chol_solve(uwb_real *a, const uwb_real *b, uwb_real *x, int n)
{
    int i, j, k;

    if (n <= 0 || n > UWB_SURVEY_MAX_UNKNOWNS) return 0;

    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            uwb_real s = a[DIDX(i, j, n)];
            for (k = 0; k < j; ++k) s -= a[DIDX(i, k, n)] * a[DIDX(j, k, n)];
            if (i == j) {
                if (!(s > (uwb_real)0)) return 0;   /* 正定値でない or NaN */
                a[DIDX(i, i, n)] = (uwb_real)sqrt((double)s);
            } else {
                a[DIDX(i, j, n)] = s / a[DIDX(j, j, n)];
            }
        }
    }

    /* L y = b */
    for (i = 0; i < n; ++i) {
        uwb_real s = b[i];
        for (k = 0; k < i; ++k) s -= a[DIDX(i, k, n)] * x[k];
        x[i] = s / a[DIDX(i, i, n)];
    }
    /* L^T x = y */
    for (i = n - 1; i >= 0; --i) {
        uwb_real s = x[i];
        for (k = i + 1; k < n; ++k) s -= a[DIDX(k, i, n)] * x[k];
        x[i] = s / a[DIDX(i, i, n)];
    }

    for (i = 0; i < n; ++i) if (x[i] != x[i]) return 0;
    return 1;
}

/* ------------------------------------------- 対称固有値分解 (Jacobi + V) */

int uwb_survey_sym_eig(uwb_real *a, uwb_real *eig, uwb_real *vec, int n)
{
    int sweep, i, j, k;

    if (n <= 0 || n > UWB_SURVEY_EIG_MAX) return 0;

    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            vec[DIDX(i, j, n)] = (i == j) ? (uwb_real)1 : (uwb_real)0;

    if (n == 1) {
        eig[0] = a[0];
        return (eig[0] == eig[0]) ? 1 : 0;
    }

    /* 循環 Jacobi。n <= 8 なので 1 スイープ 28 回転、収束は数スイープ。
     *
     * 打ち切りは **相対**で見る。uwb_linalg の絶対値 1e-30 だと行列の
     * スケールが大きいとき（MDS の Gram 行列は m^2 のオーダ）永遠に
     * 到達せず、無駄に 60 スイープ回してしまうため。 */
    for (sweep = 0; sweep < 60; ++sweep) {
        uwb_real off = (uwb_real)0, fro = (uwb_real)0;
        for (i = 0; i < n; ++i) {
            fro += a[DIDX(i, i, n)] * a[DIDX(i, i, n)];
            for (j = i + 1; j < n; ++j) {
                uwb_real v = a[DIDX(i, j, n)];
                off += v * v;
            }
        }
        fro += off;
        if (!(off > (uwb_real)1e-30 * fro + (uwb_real)1e-300)) break;

        for (i = 0; i < n; ++i) {
            for (j = i + 1; j < n; ++j) {
                uwb_real aij = a[DIDX(i, j, n)];
                uwb_real theta, t, c, s;
                if (dabs(aij) < (uwb_real)1e-300) continue;

                /* 2x2 部分行列を対角化する回転角。tan(2*phi) の代わりに
                 * 数値的に安定な t = tan(phi) の直接解を使う（Golub & Van Loan）。 */
                theta = (a[DIDX(j, j, n)] - a[DIDX(i, i, n)]) / ((uwb_real)2 * aij);
                t = (theta >= (uwb_real)0 ? (uwb_real)1 : (uwb_real)-1) /
                    (dabs(theta) + (uwb_real)sqrt((double)(theta * theta + 1)));
                c = (uwb_real)1 / (uwb_real)sqrt((double)(t * t + 1));
                s = t * c;

                for (k = 0; k < n; ++k) {   /* A <- A J（列） */
                    uwb_real aki = a[DIDX(k, i, n)], akj = a[DIDX(k, j, n)];
                    a[DIDX(k, i, n)] = c * aki - s * akj;
                    a[DIDX(k, j, n)] = s * aki + c * akj;
                }
                for (k = 0; k < n; ++k) {   /* A <- J^T A（行） */
                    uwb_real aik = a[DIDX(i, k, n)], ajk = a[DIDX(j, k, n)];
                    a[DIDX(i, k, n)] = c * aik - s * ajk;
                    a[DIDX(j, k, n)] = s * aik + c * ajk;
                }
                for (k = 0; k < n; ++k) {   /* V <- V J（同じ回転を貯める） */
                    uwb_real vki = vec[DIDX(k, i, n)], vkj = vec[DIDX(k, j, n)];
                    vec[DIDX(k, i, n)] = c * vki - s * vkj;
                    vec[DIDX(k, j, n)] = s * vki + c * vkj;
                }
            }
        }
    }

    for (i = 0; i < n; ++i) {
        eig[i] = a[DIDX(i, i, n)];
        if (eig[i] != eig[i]) return 0;
    }

    /* 降順に並べ替える（固有ベクトルの列も一緒に動かす）。n <= 8 なので
     * 選択ソートで十分。 */
    for (i = 0; i < n - 1; ++i) {
        int best = i;
        for (j = i + 1; j < n; ++j) if (eig[j] > eig[best]) best = j;
        if (best != i) {
            uwb_real tv = eig[i]; eig[i] = eig[best]; eig[best] = tv;
            for (k = 0; k < n; ++k) {
                uwb_real t2 = vec[DIDX(k, i, n)];
                vec[DIDX(k, i, n)] = vec[DIDX(k, best, n)];
                vec[DIDX(k, best, n)] = t2;
            }
        }
    }

    /* 符号の正規化: 絶対値最大の成分（同点なら添字が小さい方）を正にする。
     * Jacobi が返す符号は回転の順序に依存するので、これを入れておかないと
     * 「同じ入力なのに MDS の初期配置が鏡像になる」ことが起きうる。 */
    for (j = 0; j < n; ++j) {
        int    big = 0;
        uwb_real bv = (uwb_real)-1;
        for (i = 0; i < n; ++i) {
            uwb_real v = dabs(vec[DIDX(i, j, n)]);
            if (v > bv) { bv = v; big = i; }
        }
        if (vec[DIDX(big, j, n)] < (uwb_real)0)
            for (i = 0; i < n; ++i) vec[DIDX(i, j, n)] = -vec[DIDX(i, j, n)];
    }

    return 1;
}
