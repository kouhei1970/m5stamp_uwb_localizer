#include "uwb_linalg.h"

#include <math.h>

#define IDX(i, j, n) ((i) * (n) + (j))

static uwb_real uwb_abs(uwb_real v) { return v < 0 ? -v : v; }

/* ------------------------------------------------------------------ LU */

/* 部分ピボット付き LU。piv[i] は i 行目に来た元の行番号。 */
static int lu_decompose(uwb_real *a, int *piv, int n)
{
    int i, j, k;
    for (i = 0; i < n; ++i) piv[i] = i;

    for (k = 0; k < n; ++k) {
        /* ピボット選択 — これが無いと、まともな行列でも桁落ちで壊れる */
        int    best = k;
        uwb_real bestv = uwb_abs(a[IDX(k, k, n)]);
        for (i = k + 1; i < n; ++i) {
            uwb_real v = uwb_abs(a[IDX(i, k, n)]);
            if (v > bestv) { bestv = v; best = i; }
        }
        if (!(bestv > (uwb_real)0) || bestv != bestv) return 0;   /* 特異 or NaN */

        if (best != k) {
            for (j = 0; j < n; ++j) {
                uwb_real t = a[IDX(k, j, n)];
                a[IDX(k, j, n)] = a[IDX(best, j, n)];
                a[IDX(best, j, n)] = t;
            }
            { int t = piv[k]; piv[k] = piv[best]; piv[best] = t; }
        }

        for (i = k + 1; i < n; ++i) {
            uwb_real f = a[IDX(i, k, n)] / a[IDX(k, k, n)];
            a[IDX(i, k, n)] = f;
            for (j = k + 1; j < n; ++j) a[IDX(i, j, n)] -= f * a[IDX(k, j, n)];
        }
    }
    return 1;
}

static void lu_solve_vec(const uwb_real *lu, const int *piv, const uwb_real *b,
                         uwb_real *x, int n)
{
    int i, j;
    for (i = 0; i < n; ++i) {
        uwb_real s = b[piv[i]];
        for (j = 0; j < i; ++j) s -= lu[IDX(i, j, n)] * x[j];
        x[i] = s;
    }
    for (i = n - 1; i >= 0; --i) {
        uwb_real s = x[i];
        for (j = i + 1; j < n; ++j) s -= lu[IDX(i, j, n)] * x[j];
        x[i] = s / lu[IDX(i, i, n)];
    }
}

int uwb_solve_lin(uwb_real *a, const uwb_real *b, uwb_real *x, int n)
{
    int piv[UWB_LA_MAX];
    int i;
    if (n <= 0 || n > UWB_LA_MAX) return 0;
    if (!lu_decompose(a, piv, n)) return 0;
    lu_solve_vec(a, piv, b, x, n);
    for (i = 0; i < n; ++i) if (x[i] != x[i]) return 0;   /* NaN 混入 */
    return 1;
}

int uwb_inverse(uwb_real *a, uwb_real *inv, int n)
{
    int piv[UWB_LA_MAX];
    uwb_real e[UWB_LA_MAX], col[UWB_LA_MAX];
    int i, j;
    if (n <= 0 || n > UWB_LA_MAX) return 0;
    if (!lu_decompose(a, piv, n)) return 0;
    for (j = 0; j < n; ++j) {
        for (i = 0; i < n; ++i) e[i] = (i == j) ? (uwb_real)1 : (uwb_real)0;
        lu_solve_vec(a, piv, e, col, n);
        for (i = 0; i < n; ++i) {
            if (col[i] != col[i]) return 0;
            inv[IDX(i, j, n)] = col[i];
        }
    }
    return 1;
}

/* ------------------------------------------------------------ Cholesky */

int uwb_cholesky(uwb_real *a, int n)
{
    int i, j, k;
    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            uwb_real s = a[IDX(i, j, n)];
            for (k = 0; k < j; ++k) s -= a[IDX(i, k, n)] * a[IDX(j, k, n)];
            if (i == j) {
                if (!(s > (uwb_real)0)) return 0;      /* 正定値でない */
                a[IDX(i, j, n)] = (uwb_real)sqrt((double)s);
            } else {
                a[IDX(i, j, n)] = s / a[IDX(j, j, n)];
            }
        }
        for (j = i + 1; j < n; ++j) a[IDX(i, j, n)] = (uwb_real)0;
    }
    return 1;
}

int uwb_inverse_spd(uwb_real *a, uwb_real *inv, int n)
{
    /* 対称でも一般の LU で足りる。正定値性が崩れた行列 (ゲートで観測を
     * 落としすぎた場合など) でも「解けなかった」と返せる方が扱いやすい。 */
    return uwb_inverse(a, inv, n);
}

/* --------------------------------------------------- 対称固有値 (Jacobi) */

int uwb_sym_eigvals(uwb_real *a, uwb_real *eig, int n)
{
    /* 循環 Jacobi 法。n <= 9 なので回転の回数は高々数百回で収まる。
     * 固有ベクトルは要らない (Beck 法は最大固有値しか使わない)。 */
    int sweep, i, j, k;
    if (n <= 0 || n > UWB_LA_MAX) return 0;
    if (n == 1) { eig[0] = a[0]; return 1; }

    for (sweep = 0; sweep < 60; ++sweep) {
        uwb_real off = (uwb_real)0;
        for (i = 0; i < n; ++i)
            for (j = i + 1; j < n; ++j) off += a[IDX(i, j, n)] * a[IDX(i, j, n)];
        if (!(off > (uwb_real)1e-30)) break;

        for (i = 0; i < n; ++i) {
            for (j = i + 1; j < n; ++j) {
                uwb_real aij = a[IDX(i, j, n)];
                uwb_real theta, t, c, s;
                if (uwb_abs(aij) < (uwb_real)1e-300) continue;
                theta = (a[IDX(j, j, n)] - a[IDX(i, i, n)]) / ((uwb_real)2 * aij);
                t = (theta >= 0 ? (uwb_real)1 : (uwb_real)-1) /
                    (uwb_abs(theta) + (uwb_real)sqrt((double)(theta * theta + 1)));
                c = (uwb_real)1 / (uwb_real)sqrt((double)(t * t + 1));
                s = t * c;

                for (k = 0; k < n; ++k) {
                    uwb_real aki = a[IDX(k, i, n)], akj = a[IDX(k, j, n)];
                    a[IDX(k, i, n)] = c * aki - s * akj;
                    a[IDX(k, j, n)] = s * aki + c * akj;
                }
                for (k = 0; k < n; ++k) {
                    uwb_real aik = a[IDX(i, k, n)], ajk = a[IDX(j, k, n)];
                    a[IDX(i, k, n)] = c * aik - s * ajk;
                    a[IDX(j, k, n)] = s * aik + c * ajk;
                }
            }
        }
    }

    for (i = 0; i < n; ++i) {
        eig[i] = a[IDX(i, i, n)];
        if (eig[i] != eig[i]) return 0;
    }
    /* 昇順に並べる (挿入ソート。n <= 9 なので十分) */
    for (i = 1; i < n; ++i) {
        uwb_real v = eig[i];
        for (j = i - 1; j >= 0 && eig[j] > v; --j) eig[j + 1] = eig[j];
        eig[j + 1] = v;
    }
    return 1;
}

/* ---------------------------------------------------------- 重み付き積 */

void uwb_ata_weighted(const uwb_real *a, const uwb_real *w, int m, int n, uwb_real *c)
{
    int i, j, k;
    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j) c[IDX(i, j, n)] = (uwb_real)0;
    for (k = 0; k < m; ++k) {
        uwb_real wk = w ? w[k] : (uwb_real)1;
        for (i = 0; i < n; ++i) {
            uwb_real ai = a[IDX(k, i, n)] * wk;
            for (j = 0; j < n; ++j) c[IDX(i, j, n)] += ai * a[IDX(k, j, n)];
        }
    }
}

void uwb_atb_weighted(const uwb_real *a, const uwb_real *w, const uwb_real *b,
                      int m, int n, uwb_real *y)
{
    int i, k;
    for (i = 0; i < n; ++i) y[i] = (uwb_real)0;
    for (k = 0; k < m; ++k) {
        uwb_real wk = (w ? w[k] : (uwb_real)1) * b[k];
        for (i = 0; i < n; ++i) y[i] += a[IDX(k, i, n)] * wk;
    }
}
