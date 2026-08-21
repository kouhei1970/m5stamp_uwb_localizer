/* uwb_math (components/uwb_math) のホストテスト。
 *
 * uwb_math は「一般の LU / Jacobi は置かない、参照実装はテスト側にだけ持つ」
 * 方針で書かれているので、ここでは components/uwb_loc/src/uwb_linalg.c の
 * 一般 LU (uwb_solve_lin 相当)・循環 Jacobi (uwb_sym_eig 相当) と、
 * components/uwb_survey/src/uwb_survey_dense.c のコレスキー
 * (uwb_survey_chol_solve 相当) を static 関数としてコピーし、
 * uwb_math の 2x2/3x3 展開カーネル・ブロックコレスキーが一般解法と
 * 一致するかどうかを検証する。数値の一致だけでなく、
 *
 *   * a x b は a, b と直交する
 *   * 固有ベクトルは S v = lam v を満たし単位長
 *   * ランク落ちした行列では solve/inverse が失敗を返す
 *   * NaN が混ざっても NaN を返さず ok=0 で返る
 *   * ブロックコレスキーの解はフル行列のコレスキーと一致する
 *
 * といった、アルゴリズムが持つべき性質を確かめる。
 */
#include "uwb_math.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------- 下ごしらえ */

#define R(x) ((uwb_real)(x))

#define TEST_MAXN 25   /* ref_lu_solve / ref_inverse / ref_jacobi_eig / ref_chol_solve の上限 */

/* しきい値は「観測した最大誤差 (stat_note 参照) の 30〜100 倍程度」を
 * 目安に詰めてある。make test / strict / float / sanitize の 4 つが
 * 全部通る範囲で、退行を検出できるだけタイトにする。
 * 実測 (2026-08-21, この構成での make test / make float):
 *   double: 各カテゴリとも最大 ~1.2e-14 (bchol survey だけ ~2.1e-11)
 *   float : 各カテゴリとも最大 ~2.4e-6  (bchol survey は damping を
 *           強めて同水準まで落とした。詳細は test_bchol_survey 内のコメント) */
#if UWB_REAL_IS_FLOAT
#define TOL_EXACT  1e-5   /* ほぼ厳密に一致すべき量 (代数展開の丸めのみ) */
#define TOL_SOLVE  1e-4   /* solve/inverse の相対精度 (条件数 O(10) 程度) */
#define TOL_EIG    1e-4   /* 固有値・固有ベクトルの相対精度 */
#define TOL_LOOSE  2e-4   /* 条件数がやや大きい/桁数が多いケース */
#else
#define TOL_EXACT  1e-12
#define TOL_SOLVE  1e-11
#define TOL_EIG    1e-11
#define TOL_LOOSE  1e-9
#endif

static int g_total = 0;
static int g_fail = 0;

#define CHECK(cond, name)                                              \
    do {                                                                \
        ++g_total;                                                      \
        if (!(cond)) {                                                  \
            ++g_fail;                                                   \
            printf("FAIL: %s (line %d)\n", (name), __LINE__);           \
        }                                                                \
    } while (0)

/* |a-b| / max(1, |a|, |b|) の相対誤差。near() の中身と、統計収集
 * (stat_note) の両方から使う共通実装。 */
static double rel_err(double a, double b)
{
    double diff, scale;
    diff = fabs(a - b);
    scale = fabs(a);
    if (fabs(b) > scale) scale = fabs(b);
    if (scale < 1.0) scale = 1.0;
    return diff / scale;
}

/* |a-b| <= tol * max(1, |a|, |b|) の相対誤差判定。 */
static int near(double a, double b, double tol)
{
    return rel_err(a, b) <= tol;
}

/* ----------------------------------------------------- 観測した最大誤差 */
/* 「参照実装との差」「残差」を項目名ごとに最大値だけ覚えておく。malloc は
 * 使わない (固定長テーブル)。名前は呼び出し側の文字列リテラルをそのまま
 * 指すので、テスト関数のローカル変数にせず必ずリテラルを渡すこと。 */
#define STAT_MAX 32

static struct { const char *name; double worst; } g_stats[STAT_MAX];
static int g_stat_count = 0;

static void stat_note(const char *name, double err)
{
    int i;
    if (err < 0.0) err = -err;
    for (i = 0; i < g_stat_count; ++i) {
        if (strcmp(g_stats[i].name, name) == 0) {
            if (err > g_stats[i].worst) g_stats[i].worst = err;
            return;
        }
    }
    if (g_stat_count < STAT_MAX) {
        g_stats[g_stat_count].name = name;
        g_stats[g_stat_count].worst = err;
        ++g_stat_count;
    }
    /* テーブルが尽きたら (32 項目を超えたら) 静かに無視する。CHECK の
     * 合否には影響しない診断出力なので、ここで落とす必要はない。 */
}

static void stat_report(void)
{
    int i;
    if (g_stat_count == 0) return;
    printf("-- 観測した最大誤差 (参照実装比 or 残差、注記のとおり正規化) --\n");
    for (i = 0; i < g_stat_count; ++i)
        printf("  max err  %-40s %.3e\n", g_stats[i].name, g_stats[i].worst);
}

/* 配列 a[0..count) と b[0..count) の要素ごとの相対誤差の最大値。
 * 「1 件のチェックでフル行列/ベクトルの一致を見る」ときに使う。 */
static double max_rel_diff(const uwb_real *a, const uwb_real *b, int count)
{
    double worst, av, bv, diff, scale, rel;
    int i;
    worst = 0.0;
    for (i = 0; i < count; ++i) {
        av = (double)a[i];
        bv = (double)b[i];
        diff = fabs(av - bv);
        scale = fabs(av);
        if (fabs(bv) > scale) scale = fabs(bv);
        if (scale < 1.0) scale = 1.0;
        rel = diff / scale;
        if (rel > worst) worst = rel;
    }
    return worst;
}

/* max|a_i - b_i| / max(1, max|a_i|, max|b_i|)。固有値ベクトルのように
 * 「0 に近い成分もあるが全体のスケールで見たい」量に使う
 * (max_rel_diff は成分ごとの相対誤差なので、0 に近い固有値だと分母も
 * 小さくなって過大評価しやすい)。 */
static double vec_err_over_norm(const uwb_real *a, const uwb_real *b, int count)
{
    double maxdiff = 0.0, maxv = 0.0, d, va, vb;
    int i;
    for (i = 0; i < count; ++i) {
        d = fabs((double)a[i] - (double)b[i]);
        if (d > maxdiff) maxdiff = d;
        va = fabs((double)a[i]);
        vb = fabs((double)b[i]);
        if (va > maxv) maxv = va;
        if (vb > maxv) maxv = vb;
    }
    if (maxv < 1.0) maxv = 1.0;
    return maxdiff / maxv;
}

/* max|a_i - b_i| (絶対値、正規化なし)。呼び出し側で好きなスケールで
 * 割って使う (‖S‖ で割って相対化する、など)。 */
static double arr_absdiff_max(const uwb_real *a, const uwb_real *b, int count)
{
    double worst = 0.0, d;
    int i;
    for (i = 0; i < count; ++i) {
        d = fabs((double)a[i] - (double)b[i]);
        if (d > worst) worst = d;
    }
    return worst;
}

/* ------------------------------------------------------ 決定的な擬似乱数 */
/* rand() は使わない (再現性のため自前で持つ)。tests/host/loc/test_uwb.c と
 * 同じ LCG。 */

static unsigned long g_seed = 12345UL;

static double urand(void)   /* [0,1) */
{
    g_seed = g_seed * 1103515245UL + 12345UL;
    return (double)((g_seed >> 16) & 0x7fffUL) / 32768.0;
}

#define TEST_PI 3.14159265358979323846

static double nrand(void)   /* 標準正規分布 (Box-Muller) */
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * TEST_PI * u2);
}

/* ----------------------------------------------- 参照実装 (テスト側だけ) */

/* 部分ピボット付き LU (n <= TEST_MAXN)。components/uwb_loc/src/uwb_linalg.c
 * の lu_decompose/lu_solve_vec/uwb_solve_lin/uwb_inverse をコピー。 */

static int ref_lu_decompose(uwb_real *a, int *piv, int n)
{
    int i, j, k, best;
    uwb_real bestv, v, f;
    for (i = 0; i < n; ++i) piv[i] = i;

    for (k = 0; k < n; ++k) {
        best = k;
        bestv = uwb_math_abs(a[k * n + k]);
        for (i = k + 1; i < n; ++i) {
            v = uwb_math_abs(a[i * n + k]);
            if (v > bestv) { bestv = v; best = i; }
        }
        if (!(bestv > R(0)) || bestv != bestv) return 0;   /* 特異 or NaN */

        if (best != k) {
            for (j = 0; j < n; ++j) {
                uwb_real t = a[k * n + j];
                a[k * n + j] = a[best * n + j];
                a[best * n + j] = t;
            }
            { int t = piv[k]; piv[k] = piv[best]; piv[best] = t; }
        }

        for (i = k + 1; i < n; ++i) {
            f = a[i * n + k] / a[k * n + k];
            a[i * n + k] = f;
            for (j = k + 1; j < n; ++j) a[i * n + j] -= f * a[k * n + j];
        }
    }
    return 1;
}

static void ref_lu_solve_vec(const uwb_real *lu, const int *piv, const uwb_real *b,
                              uwb_real *x, int n)
{
    int i, j;
    uwb_real s;
    for (i = 0; i < n; ++i) {
        s = b[piv[i]];
        for (j = 0; j < i; ++j) s -= lu[i * n + j] * x[j];
        x[i] = s;
    }
    for (i = n - 1; i >= 0; --i) {
        s = x[i];
        for (j = i + 1; j < n; ++j) s -= lu[i * n + j] * x[j];
        x[i] = s / lu[i * n + i];
    }
}

/* a は書き換わる (LU 分解の作業領域を兼ねる)。n <= 25。 */
static int ref_lu_solve(uwb_real *a, const uwb_real *b, uwb_real *x, int n)
{
    int piv[TEST_MAXN];
    int i;
    if (n <= 0 || n > TEST_MAXN) return 0;
    if (!ref_lu_decompose(a, piv, n)) return 0;
    ref_lu_solve_vec(a, piv, b, x, n);
    for (i = 0; i < n; ++i) if (x[i] != x[i]) return 0;
    return 1;
}

/* a は書き換わる。n <= 25。 */
static int ref_inverse(uwb_real *a, uwb_real *inv, int n)
{
    int piv[TEST_MAXN];
    uwb_real e[TEST_MAXN], col[TEST_MAXN];
    int i, j;
    if (n <= 0 || n > TEST_MAXN) return 0;
    if (!ref_lu_decompose(a, piv, n)) return 0;
    for (j = 0; j < n; ++j) {
        for (i = 0; i < n; ++i) e[i] = (i == j) ? R(1) : R(0);
        ref_lu_solve_vec(a, piv, e, col, n);
        for (i = 0; i < n; ++i) {
            if (col[i] != col[i]) return 0;
            inv[i * n + j] = col[i];
        }
    }
    return 1;
}

/* 循環 Jacobi 法 (components/uwb_loc/src/uwb_linalg.c の uwb_sym_eig と同じ
 * 回転)。収束判定だけ絶対しきい値ではなく相対 (uwb_survey_sym_eig と同じ
 * 形) にしてある。スケール 1e-6/1e6 のテストでも早期に打ち切られないため。
 * 固有値は降順、固有ベクトルは列 (仕様どおり)。n <= 25、a は書き換わる。 */
static int ref_jacobi_eig(uwb_real *a, uwb_real *eig, uwb_real *vec, int n)
{
    int sweep, i, j, k, jj;
    uwb_real off, fro, aij, theta, t, c, s, v;

    if (n <= 0 || n > TEST_MAXN) return 0;
    if (n == 1) { eig[0] = a[0]; if (vec) vec[0] = R(1); return (eig[0] == eig[0]) ? 1 : 0; }

    if (vec) {
        for (i = 0; i < n; ++i)
            for (j = 0; j < n; ++j) vec[i * n + j] = (i == j) ? R(1) : R(0);
    }

    for (sweep = 0; sweep < 100; ++sweep) {
        off = R(0);
        fro = R(0);
        for (i = 0; i < n; ++i) {
            fro += a[i * n + i] * a[i * n + i];
            for (j = i + 1; j < n; ++j) {
                v = a[i * n + j];
                off += v * v;
            }
        }
        fro += off;
        if (!(off > R(1e-30) * fro + R(1e-300))) break;

        for (i = 0; i < n; ++i) {
            for (j = i + 1; j < n; ++j) {
                aij = a[i * n + j];
                if (uwb_math_abs(aij) < R(1e-300)) continue;

                theta = (a[j * n + j] - a[i * n + i]) / (R(2) * aij);
                t = (theta >= R(0) ? R(1) : R(-1)) /
                    (uwb_math_abs(theta) + uwb_math_sqrt(theta * theta + R(1)));
                c = R(1) / uwb_math_sqrt(t * t + R(1));
                s = t * c;

                for (k = 0; k < n; ++k) {
                    uwb_real aki = a[k * n + i], akj = a[k * n + j];
                    a[k * n + i] = c * aki - s * akj;
                    a[k * n + j] = s * aki + c * akj;
                }
                if (vec) {
                    for (k = 0; k < n; ++k) {
                        uwb_real vki = vec[k * n + i], vkj = vec[k * n + j];
                        vec[k * n + i] = c * vki - s * vkj;
                        vec[k * n + j] = s * vki + c * vkj;
                    }
                }
                for (k = 0; k < n; ++k) {
                    uwb_real aik = a[i * n + k], ajk = a[j * n + k];
                    a[i * n + k] = c * aik - s * ajk;
                    a[j * n + k] = s * aik + c * ajk;
                }
            }
        }
    }

    for (i = 0; i < n; ++i) {
        eig[i] = a[i * n + i];
        if (eig[i] != eig[i]) return 0;
    }

    /* 降順に挿入ソート (固有ベクトルの列も一緒に動かす)。 */
    for (i = 1; i < n; ++i) {
        uwb_real val = eig[i];
        uwb_real vcol[TEST_MAXN];
        int kk;
        if (vec) for (kk = 0; kk < n; ++kk) vcol[kk] = vec[kk * n + i];
        for (jj = i - 1; jj >= 0 && eig[jj] < val; --jj) {
            eig[jj + 1] = eig[jj];
            if (vec) for (kk = 0; kk < n; ++kk) vec[kk * n + jj + 1] = vec[kk * n + jj];
        }
        eig[jj + 1] = val;
        if (vec) for (kk = 0; kk < n; ++kk) vec[kk * n + jj + 1] = vcol[kk];
    }
    return 1;
}

/* components/uwb_survey/src/uwb_survey_dense.c の uwb_survey_chol_solve を
 * コピー。n <= 25、a は書き換わる。 */
static int ref_chol_solve(uwb_real *a, const uwb_real *b, uwb_real *x, int n)
{
    int i, j, k;
    uwb_real s;

    if (n <= 0 || n > TEST_MAXN) return 0;

    for (i = 0; i < n; ++i) {
        for (j = 0; j <= i; ++j) {
            s = a[i * n + j];
            for (k = 0; k < j; ++k) s -= a[i * n + k] * a[j * n + k];
            if (i == j) {
                if (!(s > R(0))) return 0;   /* 正定値でない or NaN */
                a[i * n + i] = uwb_math_sqrt(s);
            } else {
                a[i * n + j] = s / a[j * n + j];
            }
        }
    }

    for (i = 0; i < n; ++i) {
        s = b[i];
        for (k = 0; k < i; ++k) s -= a[i * n + k] * x[k];
        x[i] = s / a[i * n + i];
    }
    for (i = n - 1; i >= 0; --i) {
        s = x[i];
        for (k = i + 1; k < n; ++k) s -= a[k * n + i] * x[k];
        x[i] = s / a[i * n + i];
    }

    for (i = 0; i < n; ++i) if (x[i] != x[i]) return 0;
    return 1;
}

/* ------------------------------------------------------- 3x3 の小道具 */

static void mat3_mul(const uwb_real *a, const uwb_real *b, uwb_real *out)
{
    uwb_real tmp[9], acc;
    int i, j, k;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            acc = R(0);
            for (k = 0; k < 3; ++k) acc += a[i * 3 + k] * b[k * 3 + j];
            tmp[i * 3 + j] = acc;
        }
    for (i = 0; i < 9; ++i) out[i] = tmp[i];
}

/* out = a * b^T */
static void mat3_mul_bt(const uwb_real *a, const uwb_real *b, uwb_real *out)
{
    uwb_real tmp[9], acc;
    int i, j, k;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            acc = R(0);
            for (k = 0; k < 3; ++k) acc += a[i * 3 + k] * b[j * 3 + k];
            tmp[i * 3 + j] = acc;
        }
    for (i = 0; i < 9; ++i) out[i] = tmp[i];
}

static void mat3_col(const uwb_real *v, int k, uwb_real *out)
{
    out[0] = v[3 * 0 + k];
    out[1] = v[3 * 1 + k];
    out[2] = v[3 * 2 + k];
}

/* 配列の要素の絶対値最大 (下限 1)。‖S‖ の代わりに使うノルム。sym3 パック
 * (count=6)、sym2 のフル 4 要素、m3/eig のフル 9 要素、どれにも使う。 */
static double arr_maxabs(const uwb_real *s, int count)
{
    double worst = 0.0, v;
    int i;
    for (i = 0; i < count; ++i) {
        v = fabs((double)s[i]);
        if (v > worst) worst = v;
    }
    if (worst < 1.0) worst = 1.0;
    return worst;
}

/* ------------------------------------------------------- 乱数行列の生成 */

/* ランダム SPD 3x3 (sym3 パック) を S = sum_k u_k u_k^T で作る。k 本の
 * 外積を足すので、k < 3 なら (ランク落ちした) 半正定値になる。 */
static void gen_spd3_rank(int k, uwb_real *s, double scale)
{
    int t;
    uwb_real u[3];
    uwb_sym3_zero(s);
    for (t = 0; t < k; ++t) {
        u[0] = R((urand() * 2.0 - 1.0) * scale);
        u[1] = R((urand() * 2.0 - 1.0) * scale);
        u[2] = R((urand() * 2.0 - 1.0) * scale);
        uwb_sym3_add_outer(s, u);
    }
}

/* gen_spd3_rank(3, ...) と同じだが、たまたま 3 本の外積ベクトルがほぼ
 * 同一平面に乗って条件数が大きくなった (ほぼ特異になった) 場合は
 * 引き直す。solve/inverse ファミリのテストは「まっとうな SPD 行列で
 * 正しく解ける」ことを見たいので、test_sym3_singular の役目である
 * 「ほぼ特異」を事故で混ぜないようにする。条件数の判定には
 * ref_jacobi_eig (このファイル内の参照実装) を使う。 */
static void gen_spd3_wellposed(uwb_real *s, double scale)
{
    int tries;
    for (tries = 0; tries < 200; ++tries) {
        uwb_real full[9], full_copy[9], eig[3];
        double lo, hi, v;
        int k, okref;

        gen_spd3_rank(3, s, scale);
        uwb_sym3_to_full(s, full);
        memcpy(full_copy, full, sizeof full);
        okref = ref_jacobi_eig(full_copy, eig, NULL, 3);
        if (!okref) continue;

        lo = hi = fabs((double)eig[0]);
        for (k = 1; k < 3; ++k) {
            v = fabs((double)eig[k]);
            if (v < lo) lo = v;
            if (v > hi) hi = v;
        }
        if (lo >= 0.1 * hi) return;   /* 条件数 <= 10 */
    }
    /* 保険 (理論上まず来ない): 明示的に良条件な対角行列 */
    uwb_sym3_zero(s);
    s[0] = R(scale); s[3] = R(0.5 * scale); s[5] = R(0.2 * scale);
}

/* ランダムな正規直交基底 (Gram-Schmidt)。q は行優先 9 要素、列が基底ベクトル。 */
static void gram_schmidt3(uwb_real *q)
{
    int c, p, i;
    uwb_real v[3], col[3], d, len;
    for (c = 0; c < 3; ++c) {
        for (;;) {
            v[0] = R(urand() * 2.0 - 1.0);
            v[1] = R(urand() * 2.0 - 1.0);
            v[2] = R(urand() * 2.0 - 1.0);
            for (p = 0; p < c; ++p) {
                col[0] = q[0 * 3 + p];
                col[1] = q[1 * 3 + p];
                col[2] = q[2 * 3 + p];
                d = uwb_v3_dot(v, col);
                uwb_v3_axpy(-d, col, v);
            }
            len = uwb_v3_normalize(v);
            if (len > R(1e-3)) break;
        }
        for (i = 0; i < 3; ++i) q[i * 3 + c] = v[i];
    }
}

/* S = Q diag(lam) Q^T (sym3 パック)。 */
static void sym3_from_eig(const uwb_real *q, const uwb_real *lam, uwb_real *s)
{
    uwb_real m[9], acc;
    int i, j, k;
    for (i = 0; i < 3; ++i)
        for (j = 0; j < 3; ++j) {
            acc = R(0);
            for (k = 0; k < 3; ++k) acc += q[i * 3 + k] * lam[k] * q[j * 3 + k];
            m[i * 3 + j] = acc;
        }
    uwb_sym3_from_full(m, s);
}

/* ランダムな対称行列 (固有値混在)。lam_out に使った固有値 (降順ではない、
 * 生成時の順序のまま) を書く。 */
static void gen_sym3_mixed(uwb_real *s, uwb_real *lam_out, double scale)
{
    uwb_real q[9], lam[3];
    lam[0] = R((urand() * 2.0 - 1.0) * scale);
    lam[1] = R((urand() * 2.0 - 1.0) * scale);
    lam[2] = R((urand() * 2.0 - 1.0) * scale);
    gram_schmidt3(q);
    sym3_from_eig(q, lam, s);
    if (lam_out) { lam_out[0] = lam[0]; lam_out[1] = lam[1]; lam_out[2] = lam[2]; }
}

/* gen_sym3_mixed と同じだが、|lam_i| が scale の 10% を下回らないよう
 * 再抽選する (条件数 <= 10 程度に収める)。solve/inverse ファミリの
 * テストは「まっとうな行列で正しく解ける」ことを見たいので、たまたま固有値
 * が 0 付近に落ちてほぼ特異になった (それ自体は test_sym3_singular の
 * 役目) 事故を排除する。 */
static void gen_sym3_mixed_wellposed(uwb_real *s, double scale)
{
    double floor_v, l0, l1, l2;
    uwb_real q[9], lam[3];
    int tries, ok;

    floor_v = 0.1 * scale;
    ok = 0;
    l0 = l1 = l2 = scale;
    for (tries = 0; tries < 200 && !ok; ++tries) {
        l0 = (urand() * 2.0 - 1.0) * scale;
        l1 = (urand() * 2.0 - 1.0) * scale;
        l2 = (urand() * 2.0 - 1.0) * scale;
        if (fabs(l0) >= floor_v && fabs(l1) >= floor_v && fabs(l2) >= floor_v) ok = 1;
    }
    if (!ok) { l0 = scale; l1 = 0.5 * scale; l2 = 0.2 * scale; }   /* 理論上まず来ない保険 */

    lam[0] = R(l0); lam[1] = R(l1); lam[2] = R(l2);
    gram_schmidt3(q);
    sym3_from_eig(q, lam, s);
}

/* ------------------------------------------------------- 構造的な配置 */

/* 正方形 4 点 (+-1,+-1,0) の共分散 (sum p p^T) = diag(4,4,0)。 */
static void cfg_square(uwb_real *s)
{
    static const uwb_real pts[4][3] = {
        { R(1), R(1), R(0) }, { R(1), R(-1), R(0) },
        { R(-1), R(1), R(0) }, { R(-1), R(-1), R(0) }
    };
    int i;
    uwb_sym3_zero(s);
    for (i = 0; i < 4; ++i) uwb_sym3_add_outer(s, pts[i]);
}

/* 立方体 8 頂点 (+-1,+-1,+-1) の共分散 = 8*I。 */
static void cfg_cube(uwb_real *s)
{
    uwb_real p[3];
    int i, j, k;
    uwb_sym3_zero(s);
    for (i = 0; i < 2; ++i)
        for (j = 0; j < 2; ++j)
            for (k = 0; k < 2; ++k) {
                p[0] = i ? R(1) : R(-1);
                p[1] = j ? R(1) : R(-1);
                p[2] = k ? R(1) : R(-1);
                uwb_sym3_add_outer(s, p);
            }
}

/* xy 平面上の正三角形 (原点中心、外接半径 1)。ランク 2。 */
static void cfg_triangle(uwb_real *s)
{
    uwb_real p[3];
    int i;
    uwb_sym3_zero(s);
    for (i = 0; i < 3; ++i) {
        double ang = 2.0 * TEST_PI * (double)i / 3.0;
        p[0] = R(cos(ang));
        p[1] = R(sin(ang));
        p[2] = R(0);
        uwb_sym3_add_outer(s, p);
    }
}

/* x 軸上の 3 点 (-1,0,0),(0,0,0),(1,0,0)。共分散 = diag(2,0,0)、ランク 1。 */
static void cfg_colinear(uwb_real *s)
{
    static const uwb_real pts[3][3] = {
        { R(-1), R(0), R(0) }, { R(0), R(0), R(0) }, { R(1), R(0), R(0) }
    };
    int i;
    uwb_sym3_zero(s);
    for (i = 0; i < 3; ++i) uwb_sym3_add_outer(s, pts[i]);
}

/* ============================================================ vec3 */

static void test_vec3(void)
{
    int t, i;
    uwb_real zero[3];

    for (t = 0; t < 30; ++t) {
        uwb_real a[3], b[3], c[3], d[3], sum[3];
        uwb_real na, dp;
        for (i = 0; i < 3; ++i) {
            a[i] = R(urand() * 4.0 - 2.0);
            b[i] = R(urand() * 4.0 - 2.0);
        }
        uwb_v3_cross(a, b, c);
        CHECK(near((double)uwb_v3_dot(c, a), 0.0, TOL_LOOSE), "v3 a x b perp a");
        CHECK(near((double)uwb_v3_dot(c, b), 0.0, TOL_LOOSE), "v3 a x b perp b");

        uwb_v3_sub(a, b, d);
        uwb_v3_add(d, b, sum);
        CHECK(max_rel_diff(sum, a, 3) <= TOL_EXACT, "v3 (a-b)+b = a");

        na = uwb_v3_norm(a);
        CHECK(near((double)(na * na), (double)uwb_v3_norm2(a), TOL_EXACT), "v3 norm^2 = norm2");

        dp = uwb_v3_dot(a, a);
        CHECK(near((double)dp, (double)uwb_v3_norm2(a), TOL_EXACT), "v3 a.a = norm2(a)");

        {
            uwb_real y[3], k;
            int ii;
            k = R(urand() * 4.0 - 2.0);
            for (ii = 0; ii < 3; ++ii) y[ii] = b[ii];
            uwb_v3_axpy(k, a, y);
            for (ii = 0; ii < 3; ++ii) {
                double expect = (double)b[ii] + (double)k * (double)a[ii];
                CHECK(near((double)y[ii], expect, TOL_EXACT), "v3 axpy: y += k*a");
            }
        }

        {
            uwb_real sc[3], k;
            int ii;
            k = R(urand() * 3.0 + 0.5);
            for (ii = 0; ii < 3; ++ii) sc[ii] = a[ii];
            uwb_v3_scale(k, sc);
            for (ii = 0; ii < 3; ++ii) {
                double expect = (double)a[ii] * (double)k;
                CHECK(near((double)sc[ii], expect, TOL_EXACT), "v3 scale");
            }
        }
    }

    /* normalize: 通常のベクトル */
    for (t = 0; t < 15; ++t) {
        uwb_real v[3], orig[3], len;
        for (i = 0; i < 3; ++i) { v[i] = R(urand() * 6.0 - 3.0); orig[i] = v[i]; }
        len = uwb_v3_normalize(v);
        CHECK(near((double)len, (double)uwb_v3_norm(orig), TOL_SOLVE), "v3_normalize returns original length");
        CHECK(near((double)uwb_v3_norm(v), 1.0, TOL_SOLVE), "v3_normalize result has unit length");
    }

    /* normalize: 零ベクトル */
    {
        uwb_real v[3];
        uwb_real len;
        v[0] = R(0); v[1] = R(0); v[2] = R(0);
        len = uwb_v3_normalize(v);
        CHECK(len == R(0), "v3_normalize(0) returns 0");
        CHECK(v[0] == R(0) && v[1] == R(0) && v[2] == R(0), "v3_normalize(0) leaves v = 0");
    }

    uwb_v3_zero(zero);
    CHECK(zero[0] == R(0) && zero[1] == R(0) && zero[2] == R(0), "v3_zero");

    /* perp_basis */
    for (t = 0; t < 20; ++t) {
        uwb_real u[3], e1[3], e2[3], cr[3];
        int ok;
        for (i = 0; i < 3; ++i) u[i] = R(urand() * 2.0 - 1.0);
        if (!(uwb_v3_normalize(u) > R(1e-6))) continue;
        ok = uwb_v3_perp_basis(u, e1, e2);
        CHECK(ok == 1, "perp_basis succeeds for a unit vector");
        if (ok) {
            CHECK(near((double)uwb_v3_dot(e1, u), 0.0, TOL_LOOSE), "perp_basis e1.u = 0");
            CHECK(near((double)uwb_v3_dot(e2, u), 0.0, TOL_LOOSE), "perp_basis e2.u = 0");
            CHECK(near((double)uwb_v3_dot(e1, e2), 0.0, TOL_LOOSE), "perp_basis e1.e2 = 0");
            uwb_v3_cross(e1, e2, cr);
            CHECK(max_rel_diff(cr, u, 3) <= TOL_LOOSE, "perp_basis e1 x e2 = u");
        }
    }

    /* perp_basis: 零ベクトル */
    {
        uwb_real u[3], e1[3], e2[3];
        u[0] = R(0); u[1] = R(0); u[2] = R(0);
        CHECK(uwb_v3_perp_basis(u, e1, e2) == 0, "perp_basis(0) fails");
    }
}

/* ============================================================ sym3 演算 */

static void test_sym3_ops(void)
{
    uwb_real s[6], m[9], u[3];
    int i;

    uwb_sym3_zero(s);
    CHECK(s[0] == R(0) && s[1] == R(0) && s[2] == R(0) &&
          s[3] == R(0) && s[4] == R(0) && s[5] == R(0), "sym3_zero");

    u[0] = R(1); u[1] = R(2); u[2] = R(3);
    uwb_sym3_add_outer(s, u);
    /* u u^T = [xx,xy,xz,yy,yz,zz] = [1,2,3,4,6,9] */
    CHECK(near((double)s[0], 1.0, TOL_EXACT) && near((double)s[1], 2.0, TOL_EXACT) &&
          near((double)s[2], 3.0, TOL_EXACT) && near((double)s[3], 4.0, TOL_EXACT) &&
          near((double)s[4], 6.0, TOL_EXACT) && near((double)s[5], 9.0, TOL_EXACT),
          "sym3_add_outer(u=[1,2,3])");

    uwb_sym3_add_scaled_outer(s, R(2), u);
    /* +2*u u^T = [2,4,6,8,12,18] -> 合計 [3,6,9,12,18,27] */
    CHECK(near((double)s[0], 3.0, TOL_EXACT) && near((double)s[1], 6.0, TOL_EXACT) &&
          near((double)s[3], 12.0, TOL_EXACT) && near((double)s[5], 27.0, TOL_EXACT),
          "sym3_add_scaled_outer(w=2)");

    uwb_sym3_add_diag(s, R(1));
    CHECK(near((double)s[0], 4.0, TOL_EXACT) && near((double)s[3], 13.0, TOL_EXACT) &&
          near((double)s[5], 28.0, TOL_EXACT) && near((double)s[1], 6.0, TOL_EXACT),
          "sym3_add_diag");

    {
        uwb_real before[6];
        uwb_real lambda = R(0.5), ridge = R(0.1);
        for (i = 0; i < 6; ++i) before[i] = s[i];
        uwb_sym3_damp_diag(s, lambda, ridge);
        CHECK(near((double)s[0], (double)before[0] * 1.5 + 0.1, TOL_EXACT), "damp_diag xx");
        CHECK(near((double)s[3], (double)before[3] * 1.5 + 0.1, TOL_EXACT), "damp_diag yy");
        CHECK(near((double)s[5], (double)before[5] * 1.5 + 0.1, TOL_EXACT), "damp_diag zz");
        CHECK(near((double)s[1], (double)before[1], TOL_EXACT), "damp_diag leaves xy alone");
        CHECK(near((double)s[2], (double)before[2], TOL_EXACT), "damp_diag leaves xz alone");
        CHECK(near((double)s[4], (double)before[4], TOL_EXACT), "damp_diag leaves yz alone");
    }

    uwb_sym3_to_full(s, m);
    CHECK(near((double)m[1], (double)m[3], TOL_EXACT) &&
          near((double)m[2], (double)m[6], TOL_EXACT) &&
          near((double)m[5], (double)m[7], TOL_EXACT), "sym3_to_full is symmetric");
    CHECK(near((double)m[0], (double)s[0], TOL_EXACT) &&
          near((double)m[4], (double)s[3], TOL_EXACT) &&
          near((double)m[8], (double)s[5], TOL_EXACT), "sym3_to_full diagonal matches");

    {
        uwb_real s2[6];
        uwb_sym3_from_full(m, s2);
        CHECK(max_rel_diff(s, s2, 6) <= TOL_EXACT, "sym3_to_full/from_full roundtrip");
    }

    /* mv/quad: 直接展開した式との照合 */
    for (i = 0; i < 20; ++i) {
        uwb_real ss[6], x[3], y[3], q;
        uwb_real full[9];
        double expect_y0, expect_y1, expect_y2, expect_q;
        int k;
        for (k = 0; k < 6; ++k) ss[k] = R(urand() * 4.0 - 2.0);
        for (k = 0; k < 3; ++k) x[k] = R(urand() * 4.0 - 2.0);
        uwb_sym3_mv(ss, x, y);
        uwb_sym3_to_full(ss, full);
        expect_y0 = (double)full[0] * (double)x[0] + (double)full[1] * (double)x[1] + (double)full[2] * (double)x[2];
        expect_y1 = (double)full[3] * (double)x[0] + (double)full[4] * (double)x[1] + (double)full[5] * (double)x[2];
        expect_y2 = (double)full[6] * (double)x[0] + (double)full[7] * (double)x[1] + (double)full[8] * (double)x[2];
        CHECK(near((double)y[0], expect_y0, TOL_EXACT) && near((double)y[1], expect_y1, TOL_EXACT) &&
              near((double)y[2], expect_y2, TOL_EXACT), "sym3_mv matches full expansion");

        q = uwb_sym3_quad(ss, x);
        expect_q = (double)x[0] * expect_y0 + (double)x[1] * expect_y1 + (double)x[2] * expect_y2;
        CHECK(near((double)q, expect_q, TOL_EXACT), "sym3_quad = x.(S x)");
        CHECK(near((double)q, (double)uwb_v3_dot(x, y), TOL_EXACT), "sym3_quad = x . mv(S,x)");
    }
}

/* ============================================== sym3 solve/inverse ファミリ */

/* 1 個の sym3 行列に対して solve/inverse/trace_inverse/solve_shifted/
 * inverse_shifted を参照 LU と比較する。s は書き換えない。 */
static void run_sym3_solve_case(const uwb_real *s)
{
    uwb_real full[9], full_copy[9];
    uwb_real b[3], x[3], xref[3];
    uwb_real inv[6], invfull[9];
    uwb_real ref_inv[9];
    uwb_real tr;
    uwb_real shift, s_shift[6], xs1[3], xs2[3];
    uwb_real inv_s1[6], inv_s2[6];
    int i, ok, okref, ok1, ok2;

    uwb_sym3_to_full(s, full);

    for (i = 0; i < 3; ++i) b[i] = R(urand() * 4.0 - 2.0);

    memcpy(full_copy, full, sizeof full);
    ok = uwb_sym3_solve(s, b, x);
    okref = ref_lu_solve(full_copy, b, xref, 3);
    CHECK(ok == okref, "sym3_solve success matches ref LU");
    if (ok && okref) {
        double e = max_rel_diff(x, xref, 3);
        stat_note("sym3_solve vs ref LU (rel, max(1,|a|,|b|))", e);
        CHECK(e <= TOL_SOLVE, "sym3_solve matches ref LU");
        {
            uwb_real resid[3];
            double rn;
            uwb_sym3_mv(s, x, resid);
            resid[0] -= b[0]; resid[1] -= b[1]; resid[2] -= b[2];
            rn = sqrt((double)resid[0] * (double)resid[0] + (double)resid[1] * (double)resid[1] +
                      (double)resid[2] * (double)resid[2]);
            CHECK(rn <= TOL_SOLVE * arr_maxabs(s, 6) * (fabs((double)x[0]) + fabs((double)x[1]) + fabs((double)x[2]) + 1.0),
                  "sym3_solve residual is small");
        }
    }

    memcpy(full_copy, full, sizeof full);
    ok1 = uwb_sym3_inverse(s, inv);
    memcpy(full_copy, full, sizeof full);
    ok2 = ref_inverse(full_copy, ref_inv, 3);
    CHECK(ok1 == ok2, "sym3_inverse success matches ref LU");
    if (ok1 && ok2) {
        double e;
        uwb_sym3_to_full(inv, invfull);
        e = max_rel_diff(invfull, ref_inv, 9);
        stat_note("sym3_inverse vs ref LU (rel, max(1,|a|,|b|))", e);
        CHECK(e <= TOL_SOLVE, "sym3_inverse matches ref inverse");
    }

    memcpy(full_copy, full, sizeof full);
    ok1 = uwb_sym3_trace_inverse(s, &tr);
    memcpy(full_copy, full, sizeof full);
    ok2 = ref_inverse(full_copy, ref_inv, 3);
    CHECK(ok1 == ok2, "sym3_trace_inverse success matches ref LU");
    if (ok1 && ok2) {
        double reftr = (double)ref_inv[0] + (double)ref_inv[4] + (double)ref_inv[8];
        double e = rel_err((double)tr, reftr);
        stat_note("sym3_trace_inverse vs ref LU (rel, max(1,|a|,|b|))", e);
        CHECK(e <= TOL_SOLVE, "sym3_trace_inverse matches trace(ref inverse)");
    }

    /* solve_shifted / inverse_shifted: S + shift*I を自前で作って比較 */
    shift = R(urand() * 3.0);
    for (i = 0; i < 6; ++i) s_shift[i] = s[i];
    uwb_sym3_add_diag(s_shift, shift);

    ok1 = uwb_sym3_solve(s_shift, b, xs1);
    ok2 = uwb_sym3_solve_shifted(s, shift, b, xs2);
    CHECK(ok1 == ok2, "sym3_solve_shifted success matches manual shift");
    if (ok1 && ok2) CHECK(max_rel_diff(xs1, xs2, 3) <= TOL_SOLVE, "sym3_solve_shifted matches manual shift");

    ok1 = uwb_sym3_inverse(s_shift, inv_s1);
    ok2 = uwb_sym3_inverse_shifted(s, shift, inv_s2);
    CHECK(ok1 == ok2, "sym3_inverse_shifted success matches manual shift");
    if (ok1 && ok2) CHECK(max_rel_diff(inv_s1, inv_s2, 6) <= TOL_SOLVE, "sym3_inverse_shifted matches manual shift");
}

static void test_sym3_solve_family(void)
{
    static const double scales[3] = { 1e-6, 1.0, 1e6 };
    int si, t;
    uwb_real s[6];

    for (si = 0; si < 3; ++si) {
        for (t = 0; t < 20; ++t) {
            gen_spd3_wellposed(s, scales[si]);
            run_sym3_solve_case(s);
        }
        for (t = 0; t < 20; ++t) {
            gen_sym3_mixed_wellposed(s, scales[si]);
            run_sym3_solve_case(s);
        }
    }
}

/* ------------------------------------------------ ほぼ特異な solve の失敗 */

static void test_sym3_singular(void)
{
    uwb_real s[6], b[3], x[3], inv[6], tr;
    int t;

    /* ランク 2 (SPD だが特異): u が 2 本だけ */
    for (t = 0; t < 10; ++t) {
        gen_spd3_rank(2, s, 1.0);
        b[0] = R(urand() * 2.0 - 1.0); b[1] = R(urand() * 2.0 - 1.0); b[2] = R(urand() * 2.0 - 1.0);
        CHECK(uwb_sym3_solve(s, b, x) == 0, "sym3_solve fails on rank-2 SPD");
        CHECK(uwb_sym3_inverse(s, inv) == 0, "sym3_inverse fails on rank-2 SPD");
        CHECK(uwb_sym3_trace_inverse(s, &tr) == 0, "sym3_trace_inverse fails on rank-2 SPD");
    }

    /* 完全な零行列 */
    uwb_sym3_zero(s);
    b[0] = R(1); b[1] = R(2); b[2] = R(3);
    CHECK(uwb_sym3_solve(s, b, x) == 0, "sym3_solve fails on zero matrix");
    CHECK(uwb_sym3_inverse(s, inv) == 0, "sym3_inverse fails on zero matrix");
    CHECK(uwb_sym3_trace_inverse(s, &tr) == 0, "sym3_trace_inverse fails on zero matrix");

    /* NaN 混入 */
    {
        uwb_real zero_val = R(0);
        uwb_real nan_val = zero_val / zero_val;
        gen_spd3_rank(3, s, 1.0);
        s[0] = nan_val;
        CHECK(uwb_sym3_solve(s, b, x) == 0, "sym3_solve fails on NaN");
        CHECK(uwb_sym3_inverse(s, inv) == 0, "sym3_inverse fails on NaN");
        CHECK(uwb_sym3_trace_inverse(s, &tr) == 0, "sym3_trace_inverse fails on NaN");
    }

    /* 条件数を制御した SPD: 指定した範囲内なら成功して精度が出る */
    {
#if UWB_REAL_IS_FLOAT
        double kappa_ok = 1e2;
#else
        double kappa_ok = 1e3;
#endif
        int i;
        for (i = 0; i < 20; ++i) {
            uwb_real q[9], lam[3];
            uwb_real full_copy[9], xref[3];
            int ok, okref;
            gram_schmidt3(q);
            lam[0] = R(kappa_ok);
            lam[1] = R(1.0 + urand());
            lam[2] = R(1.0);
            sym3_from_eig(q, lam, s);
            b[0] = R(urand() * 2.0 - 1.0); b[1] = R(urand() * 2.0 - 1.0); b[2] = R(urand() * 2.0 - 1.0);
            ok = uwb_sym3_solve(s, b, x);
            CHECK(ok == 1, "sym3_solve succeeds at moderate condition number");
            if (ok) {
                uwb_real full[9];
                uwb_sym3_to_full(s, full);
                memcpy(full_copy, full, sizeof full);
                okref = ref_lu_solve(full_copy, b, xref, 3);
                CHECK(okref == 1 && max_rel_diff(x, xref, 3) <= TOL_LOOSE * kappa_ok,
                      "sym3_solve keeps precision at moderate condition number");
            }
        }
    }
}

/* ============================================================ sym3_eigvals */

static void test_sym3_eigvals(void)
{
    int t;
    uwb_real s[6];

    /* ランダム対称 (正負混在)、参照 Jacobi と比較 */
    for (t = 0; t < 50; ++t) {
        uwb_real full[9], full_copy[9];
        uwb_real lam[3], reflam[TEST_MAXN];
        int ok, okref;
        gen_sym3_mixed(s, NULL, 10.0);
        ok = uwb_sym3_eigvals(s, lam);
        uwb_sym3_to_full(s, full);
        memcpy(full_copy, full, sizeof full);
        okref = ref_jacobi_eig(full_copy, reflam, NULL, 3);
        CHECK(ok == 1 && okref == 1, "sym3_eigvals / ref_jacobi_eig both succeed");
        if (ok && okref) {
            double e = vec_err_over_norm(lam, reflam, 3);
            stat_note("sym3_eigvals vs ref Jacobi (|d lam|/max|lam|)", e);
            CHECK(e <= TOL_EIG, "sym3_eigvals matches ref Jacobi");
        }
    }

    /* スケール 1e-6 / 1e6 */
    {
        static const double scales[2] = { 1e-6, 1e6 };
        int si;
        for (si = 0; si < 2; ++si) {
            for (t = 0; t < 10; ++t) {
                uwb_real full[9], full_copy[9];
                uwb_real lam[3], reflam[TEST_MAXN];
                int ok, okref;
                gen_sym3_mixed(s, NULL, scales[si]);
                ok = uwb_sym3_eigvals(s, lam);
                uwb_sym3_to_full(s, full);
                memcpy(full_copy, full, sizeof full);
                okref = ref_jacobi_eig(full_copy, reflam, NULL, 3);
                CHECK(ok == 1 && okref == 1, "sym3_eigvals succeeds at extreme scale");
                if (ok && okref) {
                    double e = vec_err_over_norm(lam, reflam, 3);
                    stat_note("sym3_eigvals vs ref Jacobi (|d lam|/max|lam|)", e);
                    CHECK(e <= TOL_EIG, "sym3_eigvals matches ref Jacobi at extreme scale");
                }
            }
        }
    }

    /* 縮退: (2,2,2), (3,3,1), (3,1,1), (1,1,0), (1,0,0), 零 */
    {
        static const uwb_real degen[6][3] = {
            { R(2), R(2), R(2) }, { R(3), R(3), R(1) }, { R(3), R(1), R(1) },
            { R(1), R(1), R(0) }, { R(1), R(0), R(0) }, { R(0), R(0), R(0) }
        };
        int d;
        for (d = 0; d < 6; ++d) {
            uwb_real q[9], lam[3];
            int ok;
            gram_schmidt3(q);
            sym3_from_eig(q, degen[d], s);
            ok = uwb_sym3_eigvals(s, lam);
            CHECK(ok == 1, "sym3_eigvals succeeds on degenerate spectrum");
            if (ok) CHECK(max_rel_diff(lam, degen[d], 3) <= TOL_EIG, "sym3_eigvals matches degenerate spectrum");
        }
    }

    /* 構造的な配置 */
    {
        uwb_real lam[3];
        int ok;

        cfg_square(s);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds on square");
        if (ok) {
            uwb_real want[3] = { R(4), R(4), R(0) };
            CHECK(max_rel_diff(lam, want, 3) <= TOL_EIG, "square covariance = diag(4,4,0)");
        }

        cfg_cube(s);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds on cube");
        if (ok) {
            uwb_real want[3] = { R(8), R(8), R(8) };
            CHECK(max_rel_diff(lam, want, 3) <= TOL_EIG, "cube covariance = 8*I");
        }

        cfg_triangle(s);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds on triangle");
        if (ok) CHECK(near((double)lam[2], 0.0, TOL_LOOSE), "triangle covariance has a zero eigenvalue (planar)");

        cfg_colinear(s);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds on colinear points");
        if (ok) {
            uwb_real want[3] = { R(2), R(0), R(0) };
            CHECK(max_rel_diff(lam, want, 3) <= TOL_EIG, "colinear covariance = diag(2,0,0)");
        }
    }
}

/* =============================================================== sym3_rank */

static void test_sym3_rank(void)
{
    struct { void (*build)(uwb_real *); int want; const char *name; } cases[4] = {
        { cfg_cube, 3, "cube" }, { cfg_square, 2, "square" },
        { cfg_colinear, 1, "colinear" }, { NULL, 0, "zero" }
    };
    int c;
    for (c = 0; c < 4; ++c) {
        uwb_real s[6], lam[3];
        int ok, got;
        if (cases[c].build) cases[c].build(s); else uwb_sym3_zero(s);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds for rank test config");
        if (!ok) continue;

        got = uwb_sym3_rank(lam, R(0.0025), R(0));
        CHECK(got == cases[c].want, "sym3_rank matches expected rank (rel_tol=0.0025)");

        got = uwb_sym3_rank(lam, R(1e-3), R(0));
        CHECK(got == cases[c].want, "sym3_rank matches expected rank (rel_tol=1e-3)");
    }
}

/* ======================================================= sym3_eigvec 系 */

static void test_sym3_eigvec(void)
{
    int t;
    uwb_real s[6];

    /* 固有値が相異なるランダム行列 */
    for (t = 0; t < 20; ++t) {
        uwb_real lam[3], v[3], minv[3], minlam[3];
        int ok, k, sign_ok;
        double nrm;
        gen_sym3_mixed(s, NULL, 5.0);
        ok = uwb_sym3_eigvals(s, lam);
        CHECK(ok == 1, "sym3_eigvals succeeds (eigvec setup)");
        if (!ok) continue;
        if (!(fabs((double)lam[0] - (double)lam[1]) > 1e-3 * arr_maxabs(s, 6)) ||
            !(fabs((double)lam[1] - (double)lam[2]) > 1e-3 * arr_maxabs(s, 6))) continue;   /* たまたま近縮退なら飛ばす */

        for (k = 0; k < 3; ++k) {
            int okv;
            okv = uwb_sym3_eigvec(s, lam[k], v);
            CHECK(okv == 1, "sym3_eigvec succeeds for a simple eigenvalue");
            if (!okv) continue;

            {
                uwb_real sv[3];
                double rn;
                uwb_sym3_mv(s, v, sv);
                sv[0] -= lam[k] * v[0]; sv[1] -= lam[k] * v[1]; sv[2] -= lam[k] * v[2];
                rn = sqrt((double)sv[0] * (double)sv[0] + (double)sv[1] * (double)sv[1] + (double)sv[2] * (double)sv[2]);
                rn /= arr_maxabs(s, 6);
                stat_note("sym3_eigvec/min_eigvec residual ||Sv-lam v||/||S||", rn);
                CHECK(rn <= TOL_EIG, "sym3_eigvec residual ||S v - lam v|| is small");
            }
            nrm = (double)uwb_v3_norm(v);
            stat_note("sym3_eigvec/min_eigvec |norm(v)-1|", fabs(nrm - 1.0));
            CHECK(near(nrm, 1.0, TOL_EIG), "sym3_eigvec result has unit norm");

            {
                int big = 0;
                double bv = -1.0;
                int ii;
                for (ii = 0; ii < 3; ++ii) {
                    double av = fabs((double)v[ii]);
                    if (av > bv) { bv = av; big = ii; }
                }
                sign_ok = (double)v[big] >= 0.0;
                CHECK(sign_ok, "sym3_eigvec sign convention: largest component positive");
            }
        }

        ok = uwb_sym3_min_eigvec(s, minlam, minv);
        CHECK(ok == 1, "sym3_min_eigvec succeeds for distinct eigenvalues");
        if (ok) {
            uwb_real vk[3], smv[3];
            double rn2, nrm2;
            int okv = uwb_sym3_eigvec(s, lam[2], vk);
            CHECK(okv == 1 && max_rel_diff(minv, vk, 3) <= TOL_EIG, "sym3_min_eigvec matches sym3_eigvec(lam_min)");
            CHECK(max_rel_diff(minlam, lam, 3) <= TOL_EIG, "sym3_min_eigvec reports the same spectrum");

            uwb_sym3_mv(s, minv, smv);
            smv[0] -= minlam[2] * minv[0]; smv[1] -= minlam[2] * minv[1]; smv[2] -= minlam[2] * minv[2];
            rn2 = sqrt((double)smv[0] * (double)smv[0] + (double)smv[1] * (double)smv[1] + (double)smv[2] * (double)smv[2]);
            rn2 /= arr_maxabs(s, 6);
            stat_note("sym3_eigvec/min_eigvec residual ||Sv-lam v||/||S||", rn2);

            nrm2 = (double)uwb_v3_norm(minv);
            stat_note("sym3_eigvec/min_eigvec |norm(v)-1|", fabs(nrm2 - 1.0));
        }
    }

    /* 縮退: (3,3,1) の lam=3、(2,2,2) の任意 lam は失敗する */
    {
        uwb_real q[9], v[3];
        uwb_real lam331[3] = { R(3), R(3), R(1) };
        uwb_real lam222[3] = { R(2), R(2), R(2) };
        gram_schmidt3(q);
        sym3_from_eig(q, lam331, s);
        CHECK(uwb_sym3_eigvec(s, R(3), v) == 0, "sym3_eigvec fails at a degenerate eigenvalue (3,3,1)@3");
        CHECK(uwb_sym3_eigvec(s, R(1), v) == 1, "sym3_eigvec succeeds at the simple eigenvalue (3,3,1)@1");

        gram_schmidt3(q);
        sym3_from_eig(q, lam222, s);
        CHECK(uwb_sym3_eigvec(s, R(2), v) == 0, "sym3_eigvec fails on a triple-degenerate matrix");
    }

    /* 正方形配置: min_eigvec は (0,0,+-1) 方向で成功する */
    {
        uwb_real v[3], lam[3];
        int ok;
        cfg_square(s);
        ok = uwb_sym3_min_eigvec(s, lam, v);
        CHECK(ok == 1, "sym3_min_eigvec succeeds on square (rank 2)");
        if (ok) {
            CHECK(near((double)uwb_math_abs(v[2]), 1.0, TOL_EIG) &&
                  near((double)v[0], 0.0, TOL_LOOSE) && near((double)v[1], 0.0, TOL_LOOSE),
                  "square min_eigvec is +-z");
        }
    }

    /* 立方体配置 (8*I): min_eigvec は失敗する */
    {
        uwb_real v[3], lam[3];
        cfg_cube(s);
        CHECK(uwb_sym3_min_eigvec(s, lam, v) == 0, "sym3_min_eigvec fails on cube (8*I)");
    }

    /* ランク 2 の平面配置: 法線ベクトルが得られる */
    for (t = 0; t < 10; ++t) {
        uwb_real n0[3], e1[3], e2[3], v[3], lam[3];
        int ok, i, np;
        for (i = 0; i < 3; ++i) n0[i] = R(urand() * 2.0 - 1.0);
        if (!(uwb_v3_normalize(n0) > R(1e-6))) continue;
        if (uwb_v3_perp_basis(n0, e1, e2) == 0) continue;

        uwb_sym3_zero(s);
        for (np = 0; np < 6; ++np) {
            uwb_real p[3], a, b;
            a = R(urand() * 4.0 - 2.0); b = R(urand() * 4.0 - 2.0);
            p[0] = a * e1[0] + b * e2[0];
            p[1] = a * e1[1] + b * e2[1];
            p[2] = a * e1[2] + b * e2[2];
            uwb_sym3_add_outer(s, p);
        }
        ok = uwb_sym3_min_eigvec(s, lam, v);
        CHECK(ok == 1, "sym3_min_eigvec succeeds on a planar point set");
        if (ok) CHECK(near(fabs((double)uwb_v3_dot(v, n0)), 1.0, TOL_LOOSE), "planar min_eigvec is the plane normal");
    }
}

/* ============================================================ sym3_eig */

static void test_sym3_eig(void)
{
    int t;
    uwb_real s[6];

    for (t = 0; t < 20; ++t) {
        uwb_real lam[3], vec[9];
        int ok;
        gen_sym3_mixed(s, NULL, 5.0);
        ok = uwb_sym3_eig(s, lam, vec);
        CHECK(ok == 1, "sym3_eig succeeds");
        if (!ok) continue;

        {
            uwb_real sv[9], vd[9];
            int k;
            for (k = 0; k < 3; ++k) {
                uwb_real col[3], y[3];
                mat3_col(vec, k, col);
                uwb_sym3_mv(s, col, y);
                sv[0 * 3 + k] = y[0]; sv[1 * 3 + k] = y[1]; sv[2 * 3 + k] = y[2];
                vd[0 * 3 + k] = col[0] * lam[k]; vd[1 * 3 + k] = col[1] * lam[k]; vd[2 * 3 + k] = col[2] * lam[k];
            }
            {
                double e = arr_absdiff_max(sv, vd, 9) / arr_maxabs(s, 6);
                stat_note("sym3_eig ||SV-V*lam||/||S||", e);
                CHECK(e <= TOL_EIG, "sym3_eig: S V = V diag(lam)");
            }
        }
        {
            uwb_real vtv[9], ident[9];
            uwb_real vt[9];
            int i, j;
            for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) vt[i * 3 + j] = vec[j * 3 + i];
            mat3_mul(vt, vec, vtv);
            for (i = 0; i < 9; ++i) ident[i] = R(0);
            ident[0] = R(1); ident[4] = R(1); ident[8] = R(1);
            {
                double e = arr_absdiff_max(vtv, ident, 9);
                stat_note("sym3_eig ||V^T V - I||", e);
                CHECK(e <= TOL_EIG, "sym3_eig: V^T V = I");
            }
        }
    }

    /* 縮退した構成でも成立すること */
    {
        uwb_real q[9];
        uwb_real configs[5][3] = {
            { R(3), R(3), R(1) }, { R(3), R(1), R(1) }, { R(2), R(2), R(2) },
            { R(4), R(4), R(0) }, { R(8), R(8), R(8) }
        };
        int c;
        for (c = 0; c < 5; ++c) {
            uwb_real lam[3], vec[9];
            int ok;
            gram_schmidt3(q);
            sym3_from_eig(q, configs[c], s);
            ok = uwb_sym3_eig(s, lam, vec);
            CHECK(ok == 1, "sym3_eig succeeds on a degenerate spectrum");
            if (!ok) continue;
            {
                uwb_real sv[9], vd[9];
                int k;
                for (k = 0; k < 3; ++k) {
                    uwb_real col[3], y[3];
                    mat3_col(vec, k, col);
                    uwb_sym3_mv(s, col, y);
                    sv[0 * 3 + k] = y[0]; sv[1 * 3 + k] = y[1]; sv[2 * 3 + k] = y[2];
                    vd[0 * 3 + k] = col[0] * lam[k]; vd[1 * 3 + k] = col[1] * lam[k]; vd[2 * 3 + k] = col[2] * lam[k];
                }
                {
                    double e = arr_absdiff_max(sv, vd, 9) / arr_maxabs(s, 6);
                    stat_note("sym3_eig ||SV-V*lam||/||S||", e);
                    CHECK(e <= TOL_EIG, "sym3_eig (degenerate): S V = V diag(lam)");
                }
            }
            {
                uwb_real vtv[9], ident[9], vt[9];
                int i, j;
                for (i = 0; i < 3; ++i) for (j = 0; j < 3; ++j) vt[i * 3 + j] = vec[j * 3 + i];
                mat3_mul(vt, vec, vtv);
                for (i = 0; i < 9; ++i) ident[i] = R(0);
                ident[0] = R(1); ident[4] = R(1); ident[8] = R(1);
                {
                    double e = arr_absdiff_max(vtv, ident, 9);
                    stat_note("sym3_eig ||V^T V - I||", e);
                    CHECK(e <= TOL_EIG, "sym3_eig (degenerate): V^T V = I");
                }
            }
        }
    }
}

/* ========================================================= sym3_reflect */

static void test_sym3_reflect(void)
{
    int t;
    for (t = 0; t < 20; ++t) {
        uwb_real s[6], sfull[9], n[3], rmat[9], tmp[9], expect[9];
        uwb_real got[6], gotfull[9];
        int i, p, q;
        for (i = 0; i < 6; ++i) s[i] = R(urand() * 4.0 - 2.0);
        for (i = 0; i < 3; ++i) n[i] = R(urand() * 2.0 - 1.0);
        if (!(uwb_v3_normalize(n) > R(1e-6))) continue;

        uwb_sym3_to_full(s, sfull);

        for (i = 0; i < 9; ++i) rmat[i] = R(0);
        rmat[0] = R(1); rmat[4] = R(1); rmat[8] = R(1);
        for (p = 0; p < 3; ++p)
            for (q = 0; q < 3; ++q) rmat[p * 3 + q] -= R(2) * n[p] * n[q];

        mat3_mul(rmat, sfull, tmp);
        mat3_mul(tmp, rmat, expect);

        for (i = 0; i < 6; ++i) got[i] = s[i];
        uwb_sym3_reflect(got, n);
        uwb_sym3_to_full(got, gotfull);

        CHECK(max_rel_diff(gotfull, expect, 9) <= TOL_EXACT, "sym3_reflect matches R S R (R = I - 2nn^T)");
    }
}

/* ================================================================ sym2 */

static void test_sym2(void)
{
    int t;
    for (t = 0; t < 30; ++t) {
        uwb_real s[3], b[2], x[2], xref[2];
        uwb_real full[4], full_copy[4];
        uwb_real inv[3], invfull[4], ref_inv[4];
        uwb_real tr;
        int ok, okref;

        s[0] = R(urand() * 4.0 - 2.0);
        s[1] = R(urand() * 4.0 - 2.0);
        s[2] = R(urand() * 4.0 - 2.0);
        /* SPD にしたい半分の反復では対角優位に寄せる (特異率を下げる) */
        if (t % 2 == 0) {
            s[0] = R((double)uwb_math_abs(s[0]) + 2.0);
            s[2] = R((double)uwb_math_abs(s[2]) + 2.0);
        }
        full[0] = s[0]; full[1] = s[1]; full[2] = s[1]; full[3] = s[2];

        CHECK(near((double)uwb_sym2_trace(s), (double)s[0] + (double)s[2], TOL_EXACT), "sym2_trace");
        CHECK(near((double)uwb_sym2_det(s),
                    (double)s[0] * (double)s[2] - (double)s[1] * (double)s[1], TOL_EXACT), "sym2_det");

        b[0] = R(urand() * 2.0 - 1.0); b[1] = R(urand() * 2.0 - 1.0);
        memcpy(full_copy, full, sizeof full);
        ok = uwb_sym2_solve(s, b, x);
        okref = ref_lu_solve(full_copy, b, xref, 2);
        CHECK(ok == okref, "sym2_solve success matches ref LU");
        if (ok && okref) {
            double e = max_rel_diff(x, xref, 2);
            stat_note("sym2_solve vs ref LU (rel, max(1,|a|,|b|))", e);
            CHECK(e <= TOL_SOLVE, "sym2_solve matches ref LU");
        }

        memcpy(full_copy, full, sizeof full);
        ok = uwb_sym2_inverse(s, inv);
        okref = ref_inverse(full_copy, ref_inv, 2);
        CHECK(ok == okref, "sym2_inverse success matches ref LU");
        if (ok && okref) {
            invfull[0] = inv[0]; invfull[1] = inv[1]; invfull[2] = inv[1]; invfull[3] = inv[2];
            CHECK(max_rel_diff(invfull, ref_inv, 4) <= TOL_SOLVE, "sym2_inverse matches ref inverse");
        }

        memcpy(full_copy, full, sizeof full);
        ok = uwb_sym2_trace_inverse(s, &tr);
        okref = ref_inverse(full_copy, ref_inv, 2);
        CHECK(ok == okref, "sym2_trace_inverse success matches ref LU");
        if (ok && okref) CHECK(near((double)tr, (double)ref_inv[0] + (double)ref_inv[3], TOL_SOLVE),
                                "sym2_trace_inverse matches trace(ref inverse)");

        {
            uwb_real lam[3], reflam[TEST_MAXN];
            memcpy(full_copy, full, sizeof full);
            ok = uwb_sym2_eigvals(s, lam);
            okref = ref_jacobi_eig(full_copy, reflam, NULL, 2);
            CHECK(ok == 1 && okref == 1, "sym2_eigvals / ref_jacobi_eig both succeed");
            if (ok && okref) {
                double e = vec_err_over_norm(lam, reflam, 2);
                stat_note("sym2_eigvals vs ref Jacobi (|d lam|/max|lam|)", e);
                CHECK(e <= TOL_EIG, "sym2_eigvals matches ref Jacobi");
            }

            if (ok && !(fabs((double)lam[0] - (double)lam[1]) < 1e-6 * arr_maxabs(full, 4))) {
                uwb_real v[2];
                int okv = uwb_sym2_eigvec(s, lam[0], v);
                CHECK(okv == 1, "sym2_eigvec succeeds for a simple eigenvalue");
                if (okv) {
                    double s0 = (double)v[0], s1 = (double)v[1];
                    double sx = (double)full[0] * s0 + (double)full[1] * s1;
                    double sy = (double)full[2] * s0 + (double)full[3] * s1;
                    double rn = sqrt((sx - (double)lam[0] * s0) * (sx - (double)lam[0] * s0) +
                                      (sy - (double)lam[0] * s1) * (sy - (double)lam[0] * s1));
                    CHECK(rn <= TOL_EIG * arr_maxabs(full, 4), "sym2_eigvec residual is small");
                    CHECK(near(sqrt(s0 * s0 + s1 * s1), 1.0, TOL_EIG), "sym2_eigvec has unit norm");
                }
            }
        }
    }

    /* 特異 (完全に潰れた行列) */
    {
        uwb_real s[3], b[2], x[2], inv[3], tr;
        s[0] = R(0); s[1] = R(0); s[2] = R(0);
        CHECK(uwb_sym2_solve(s, b, x) == 0, "sym2_solve fails on zero matrix");
        CHECK(uwb_sym2_inverse(s, inv) == 0, "sym2_inverse fails on zero matrix");
        CHECK(uwb_sym2_trace_inverse(s, &tr) == 0, "sym2_trace_inverse fails on zero matrix");

        /* ランク 1 (行列式 0 だが非零) */
        s[0] = R(1); s[1] = R(1); s[2] = R(1);
        CHECK(uwb_sym2_solve(s, b, x) == 0, "sym2_solve fails on singular (rank-1) matrix");
        CHECK(uwb_sym2_inverse(s, inv) == 0, "sym2_inverse fails on singular (rank-1) matrix");
    }

    /* 縮退 (c*I) で eigvec が失敗する */
    {
        uwb_real s[3], v[2];
        s[0] = R(3); s[1] = R(0); s[2] = R(3);
        CHECK(uwb_sym2_eigvec(s, R(3), v) == 0, "sym2_eigvec fails on c*I");
    }
}

/* ================================================ symn rank-1 update/downdate */

static void test_symn_rank1(void)
{
    static const int ns[3] = { 4, 6, 9 };
    static const int nds[2] = { 2, 3 };
    int ni, di, rep;

    for (ni = 0; ni < 3; ++ni) {
        int n = ns[ni];
        for (di = 0; di < 2; ++di) {
            int nd = nds[di];
            for (rep = 0; rep < 5; ++rep) {
                uwb_real a[9 * 9], p0[9 * 9], h[9], u[9];
                uwb_real p_up[9 * 9], p_down[9 * 9];
                uwb_real expect_up[9 * 9], expect_down[9 * 9];
                uwb_real k[9], khkt[9 * 9], imk_h[9 * 9], tmp[9 * 9];
                uwb_real s_scalar, r_ridge, c_coeff;
                int i, j, kk;

                for (i = 0; i < n * n; ++i) a[i] = R(nrand());
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) {
                        uwb_real acc = R(0);
                        for (kk = 0; kk < n; ++kk) acc += a[i * n + kk] * a[j * n + kk];
                        if (i == j) acc += R(1);
                        p0[i * n + j] = acc;
                    }

                for (i = 0; i < n; ++i) h[i] = (i < nd) ? R(urand() * 2.0 - 1.0) : R(0);
                for (i = 0; i < n; ++i) {
                    uwb_real acc = R(0);
                    for (j = 0; j < n; ++j) acc += p0[i * n + j] * h[j];
                    u[i] = acc;
                }
                {
                    uwb_real hu = R(0);
                    for (i = 0; i < n; ++i) hu += h[i] * u[i];
                    r_ridge = R(0.1 + urand());
                    s_scalar = hu + r_ridge;
                }

                memcpy(p_down, p0, sizeof(uwb_real) * (size_t)(n * n));
                uwb_symn_rank1_downdate(p_down, n, u, R(1) / s_scalar);

                /* 参照: Joseph 形式 (I - K h^T) P (I - K h^T)^T + r K K^T, K = u/s */
                for (i = 0; i < n; ++i) k[i] = u[i] / s_scalar;
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) imk_h[i * n + j] = (i == j ? R(1) : R(0)) - k[i] * h[j];
                /* tmp = (I-Kh^T) P0 */
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) {
                        uwb_real acc = R(0);
                        for (kk = 0; kk < n; ++kk) acc += imk_h[i * n + kk] * p0[kk * n + j];
                        tmp[i * n + j] = acc;
                    }
                /* expect_down = tmp (I-Kh^T)^T */
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) {
                        uwb_real acc = R(0);
                        for (kk = 0; kk < n; ++kk) acc += tmp[i * n + kk] * imk_h[j * n + kk];
                        expect_down[i * n + j] = acc;
                    }
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) khkt[i * n + j] = r_ridge * k[i] * k[j];
                for (i = 0; i < n * n; ++i) expect_down[i] += khkt[i];

                {
                    double e = max_rel_diff(p_down, expect_down, n * n);
                    stat_note("symn_rank1_downdate vs Joseph form (rel, max(1,|a|,|b|))", e);
                    CHECK(e <= TOL_SOLVE, "symn_rank1_downdate matches Joseph form");
                }
                {
                    int sym_ok = 1;
                    for (i = 0; i < n; ++i)
                        for (j = 0; j < n; ++j)
                            if (p_down[i * n + j] != p_down[j * n + i]) sym_ok = 0;
                    CHECK(sym_ok, "symn_rank1_downdate result is exactly symmetric");
                }

                c_coeff = R(urand() * 3.0 - 1.5);
                memcpy(p_up, p0, sizeof(uwb_real) * (size_t)(n * n));
                uwb_symn_rank1_update(p_up, n, u, c_coeff);
                for (i = 0; i < n; ++i)
                    for (j = 0; j < n; ++j) expect_up[i * n + j] = p0[i * n + j] + c_coeff * u[i] * u[j];
                CHECK(max_rel_diff(p_up, expect_up, n * n) <= TOL_SOLVE, "symn_rank1_update matches P + c u u^T");
                {
                    int sym_ok = 1;
                    for (i = 0; i < n; ++i)
                        for (j = 0; j < n; ++j)
                            if (p_up[i * n + j] != p_up[j * n + i]) sym_ok = 0;
                    CHECK(sym_ok, "symn_rank1_update result is exactly symmetric");
                }
            }
        }
    }
}

/* =================================================================== m3 */

static void test_m3(void)
{
    int t;
    for (t = 0; t < 20; ++t) {
        uwb_real a[9], b3[3], u[9], invd[3];
        uwb_real l[9], llt[9];
        int i, j, ok;

        /* ランダム SPD 3x3 (B B^T + I) */
        for (i = 0; i < 9; ++i) u[i] = R(urand() * 2.0 - 1.0);
        mat3_mul_bt(u, u, a);
        a[0] += R(1); a[4] += R(1); a[8] += R(1);

        memcpy(l, a, sizeof a);
        ok = uwb_m3_chol(l, invd);
        CHECK(ok == 1, "m3_chol succeeds on SPD matrix");
        if (!ok) continue;

        mat3_mul_bt(l, l, llt);
        {
            double e = max_rel_diff(llt, a, 9);
            stat_note("m3_chol ||L L^T - A|| (rel, max(1,|a|,|b|))", e);
            CHECK(e <= TOL_SOLVE, "m3_chol: L L^T = A");
        }
        {
            int diag_ok = 1;
            for (i = 0; i < 3; ++i)
                if (!near((double)invd[i], 1.0 / (double)l[i * 3 + i], TOL_SOLVE)) diag_ok = 0;
            CHECK(diag_ok, "m3_chol invd = 1/diag(L)");
        }

        for (i = 0; i < 3; ++i) b3[i] = R(urand() * 4.0 - 2.0);
        {
            uwb_real y[3], resid[3];
            double rn;
            uwb_m3_trsv_lower(l, invd, b3, y);
            resid[0] = l[0] * y[0] - b3[0];
            resid[1] = l[3] * y[0] + l[4] * y[1] - b3[1];
            resid[2] = l[6] * y[0] + l[7] * y[1] + l[8] * y[2] - b3[2];
            rn = sqrt((double)resid[0] * (double)resid[0] + (double)resid[1] * (double)resid[1] +
                      (double)resid[2] * (double)resid[2]);
            CHECK(rn <= TOL_SOLVE * arr_maxabs(a, 9), "m3_trsv_lower solves L y = b");

            {
                uwb_real x[3], resid2[3];
                double rn2;
                uwb_m3_trsv_upper(l, invd, y, x);
                /* L^T x = y : row i of L^T is column i of L */
                resid2[0] = l[0] * x[0] + l[3] * x[1] + l[6] * x[2] - y[0];
                resid2[1] = l[4] * x[1] + l[7] * x[2] - y[1];
                resid2[2] = l[8] * x[2] - y[2];
                rn2 = sqrt((double)resid2[0] * (double)resid2[0] + (double)resid2[1] * (double)resid2[1] +
                           (double)resid2[2] * (double)resid2[2]);
                CHECK(rn2 <= TOL_SOLVE * arr_maxabs(a, 9), "m3_trsv_upper solves L^T x = y");
            }
        }

        {
            uwb_real x[9], xcopy[9], linv[9], expect[9];
            uwb_real e[3], y[3];
            int col;
            for (i = 0; i < 9; ++i) x[i] = R(urand() * 2.0 - 1.0);
            memcpy(xcopy, x, sizeof x);

            for (col = 0; col < 3; ++col) {
                e[0] = (col == 0) ? R(1) : R(0);
                e[1] = (col == 1) ? R(1) : R(0);
                e[2] = (col == 2) ? R(1) : R(0);
                y[0] = e[0] * invd[0];
                y[1] = (e[1] - l[3] * y[0]) * invd[1];
                y[2] = (e[2] - l[6] * y[0] - l[7] * y[1]) * invd[2];
                linv[0 * 3 + col] = y[0]; linv[1 * 3 + col] = y[1]; linv[2 * 3 + col] = y[2];
            }
            mat3_mul_bt(xcopy, linv, expect);   /* X * linv^T */

            uwb_m3_trsm_rt(x, l, invd);
            CHECK(max_rel_diff(x, expect, 9) <= TOL_SOLVE, "m3_trsm_rt: X <- X L^-T");
        }

        {
            uwb_real ma[9], mb[9], c[9], expect[9], prod[9];
            for (i = 0; i < 9; ++i) { ma[i] = R(urand() * 2.0 - 1.0); mb[i] = R(urand() * 2.0 - 1.0); c[i] = R(urand() * 2.0 - 1.0); }
            for (i = 0; i < 9; ++i) expect[i] = c[i];
            mat3_mul_bt(ma, mb, prod);
            for (i = 0; i < 9; ++i) expect[i] -= prod[i];
            uwb_m3_gemm_nt_sub(c, ma, mb);
            CHECK(max_rel_diff(c, expect, 9) <= TOL_SOLVE, "m3_gemm_nt_sub: C -= A B^T");
        }

        (void)j;
    }

    /* 非正定値なら chol は失敗する */
    for (t = 0; t < 5; ++t) {
        uwb_real a[9], invd[3];
        int i, j;
        for (i = 0; i < 3; ++i)
            for (j = 0; j < 3; ++j) a[i * 3 + j] = (i == j) ? R(-1) : R(0);
        CHECK(uwb_m3_chol(a, invd) == 0, "m3_chol fails on a non-positive-definite matrix");
    }
}

/* =============================================================== bchol */

static double bchol_get_max_diff(const uwb_bchol *m, const uwb_real *full, int dim)
{
    double worst = 0.0, got, want, diff, scale;
    int r, c;
    for (r = 0; r < dim; ++r) {
        for (c = 0; c <= r; ++c) {
            got = (double)uwb_bchol_get(m, r, c);
            want = (double)full[r * dim + c];
            diff = fabs(got - want);
            scale = fabs(want); if (scale < 1.0) scale = 1.0;
            if (diff / scale > worst) worst = diff / scale;
        }
    }
    return worst;
}

static void gen_full_spd(int dim, uwb_real *full)
{
    uwb_real b[TEST_MAXN * TEST_MAXN], acc;
    int i, j, k;
    for (i = 0; i < dim * dim; ++i) b[i] = R(nrand());
    for (i = 0; i < dim; ++i)
        for (j = 0; j < dim; ++j) {
            acc = R(0);
            for (k = 0; k < dim; ++k) acc += b[i * dim + k] * b[j * dim + k];
            if (i == j) acc += R(1);
            full[i * dim + j] = acc;
        }
}

static void test_bchol_dense(void)
{
    int nb, border;
    for (nb = 1; nb <= UWB_BCHOL_MAX_BLOCKS; ++nb) {
        for (border = 0; border <= 1; ++border) {
            uwb_bchol m;
            uwb_real full[TEST_MAXN * TEST_MAXN], full_copy[TEST_MAXN * TEST_MAXN];
            int dim = 3 * nb + (border ? 1 : 0);
            int i, j;
            int ok;

            gen_full_spd(dim, full);
            CHECK(uwb_bchol_zero(&m, nb, border) == 1, "bchol_zero accepts a valid (nb, border)");

            for (i = 0; i < nb; ++i) {
                for (j = 0; j <= i; ++j) {
                    uwb_real d[9];
                    int ii, jj;
                    for (ii = 0; ii < 3; ++ii)
                        for (jj = 0; jj < 3; ++jj) d[ii * 3 + jj] = full[(3 * i + ii) * dim + (3 * j + jj)];
                    uwb_bchol_add_block(&m, i, j, d);
                }
            }
            if (border) {
                int b_idx = dim - 1;
                int k;
                for (i = 0; i < nb; ++i)
                    for (k = 0; k < 3; ++k) m.brd[3 * i + k] = full[b_idx * dim + (3 * i + k)];
                m.bdd = full[b_idx * dim + b_idx];
            }

            CHECK(bchol_get_max_diff(&m, full, dim) <= TOL_EXACT, "bchol_get matches the assembled full matrix");

            memcpy(full_copy, full, sizeof(uwb_real) * (size_t)(dim * dim));
            ok = uwb_bchol_factor(&m);
            CHECK(ok == 1, "bchol_factor succeeds on B B^T + I");
            if (ok) {
                uwb_real b_rhs[TEST_MAXN], x[TEST_MAXN], xref[TEST_MAXN];
                int okref;
                for (i = 0; i < dim; ++i) b_rhs[i] = R(urand() * 4.0 - 2.0);
                CHECK(uwb_bchol_solve(&m, b_rhs, x) == 1, "bchol_solve succeeds after factor");
                memcpy(full_copy, full, sizeof(uwb_real) * (size_t)(dim * dim));
                okref = ref_chol_solve(full_copy, b_rhs, xref, dim);
                CHECK(okref == 1, "ref_chol_solve succeeds on the same matrix");
                if (okref) {
                    double e = max_rel_diff(x, xref, dim);
                    stat_note("bchol dense vs ref_chol_solve (rel, max(1,|a|,|b|))", e);
                    CHECK(e <= TOL_LOOSE, "bchol_solve matches ref_chol_solve");
                }

                {
                    uwb_real resid[TEST_MAXN];
                    double rn = 0.0, bn = 0.0, acc, e;
                    int r, cc;
                    for (r = 0; r < dim; ++r) {
                        acc = 0.0;
                        for (cc = 0; cc < dim; ++cc) acc += (double)full[r * dim + cc] * (double)x[cc];
                        resid[r] = R(acc - (double)b_rhs[r]);
                        rn += (double)resid[r] * (double)resid[r];
                        bn += (double)b_rhs[r] * (double)b_rhs[r];
                    }
                    rn = sqrt(rn);
                    bn = sqrt(bn);
                    if (bn < 1.0) bn = 1.0;
                    e = rn / bn;
                    stat_note("bchol dense residual ||Ax-b||/||b||", e);
                    CHECK(e <= TOL_LOOSE, "bchol_solve residual ||A x - b|| is small");
                }
            }
        }
    }
}

static void test_bchol_survey(void)
{
    int nb, border;
    for (nb = 1; nb <= UWB_BCHOL_MAX_BLOCKS; ++nb) {
        for (border = 0; border <= 1; ++border) {
            uwb_bchol m;
            uwb_real full[TEST_MAXN * TEST_MAXN], full_copy[TEST_MAXN * TEST_MAXN];
            uwb_real pos[UWB_BCHOL_MAX_BLOCKS][3];
            int dim = 3 * nb + (border ? 1 : 0);
            int i, j, k, q;
#if UWB_REAL_IS_FLOAT
            /* float は仮数 24bit しかないので、λ=1e-3 では測量型の構造
             * (ノード配置によっては条件数が大きくなりうる) で精度が
             * 出ないことがある。意味 (LM 減衰で正定値化する) は変えず、
             * 減衰を強めて条件数を落とす。 */
            uwb_real lambda = R(3e-1), ridge = R(1e-4);
#else
            uwb_real lambda = R(1e-3), ridge = R(1e-12);
#endif
            int ok;

            for (i = 0; i < nb; ++i) {
                pos[i][0] = R(urand() * 10.0 - 5.0);
                pos[i][1] = R(urand() * 10.0 - 5.0);
                pos[i][2] = R(urand() * 10.0 - 5.0);
            }
            for (i = 0; i < dim * dim; ++i) full[i] = R(0);
            CHECK(uwb_bchol_zero(&m, nb, border) == 1, "bchol_zero accepts a valid (nb, border) [survey]");

            /* 全ペア (完全グラフ): nb >= 3 なら各ノードの次数は nb-1 >= 2 */
            for (i = 0; i < nb; ++i) {
                for (j = 0; j < i; ++j) {
                    uwb_real diff[3], u[3], c_coeff, len;
                    c_coeff = border ? R(2) : R(0);
                    uwb_v3_sub(pos[i], pos[j], diff);
                    len = uwb_v3_normalize(diff);
                    if (!(len > R(1e-6))) continue;
                    u[0] = diff[0]; u[1] = diff[1]; u[2] = diff[2];
                    uwb_bchol_add_pair_outer(&m, i, j, u, c_coeff);

                    for (k = 0; k < 3; ++k) {
                        int ri = 3 * i + k, rj = 3 * j + k;
                        for (q = 0; q < 3; ++q) {
                            int ci = 3 * i + q, cj = 3 * j + q;
                            full[ri * dim + ci] += u[k] * u[q];
                            full[rj * dim + cj] += u[k] * u[q];
                            full[ri * dim + cj] -= u[k] * u[q];
                            full[rj * dim + ci] -= u[k] * u[q];
                        }
                    }
                    if (border) {
                        int b_idx = dim - 1;
                        for (k = 0; k < 3; ++k) {
                            full[(3 * i + k) * dim + b_idx] += c_coeff * u[k];
                            full[b_idx * dim + (3 * i + k)] += c_coeff * u[k];
                            full[(3 * j + k) * dim + b_idx] -= c_coeff * u[k];
                            full[b_idx * dim + (3 * j + k)] -= c_coeff * u[k];
                        }
                        full[b_idx * dim + b_idx] += c_coeff * c_coeff;
                    }
                }
            }

            uwb_bchol_damp_diag(&m, lambda, ridge);
            for (i = 0; i < dim; ++i) full[i * dim + i] = full[i * dim + i] * (R(1) + lambda) + ridge;

            CHECK(bchol_get_max_diff(&m, full, dim) <= TOL_LOOSE, "bchol_get matches the damped survey matrix");

            ok = uwb_bchol_factor(&m);
            CHECK(ok == 1, "bchol_factor succeeds on damped survey structure");
            if (ok) {
                uwb_real b_rhs[TEST_MAXN], x[TEST_MAXN], xref[TEST_MAXN];
                int okref;
                for (i = 0; i < dim; ++i) b_rhs[i] = R(urand() * 4.0 - 2.0);
                CHECK(uwb_bchol_solve(&m, b_rhs, x) == 1, "bchol_solve succeeds after factor [survey]");
                memcpy(full_copy, full, sizeof(uwb_real) * (size_t)(dim * dim));
                okref = ref_chol_solve(full_copy, b_rhs, xref, dim);
                CHECK(okref == 1, "ref_chol_solve succeeds on the damped survey matrix");
                if (okref) {
                    double e = max_rel_diff(x, xref, dim);
                    stat_note("bchol survey vs ref_chol_solve (rel, max(1,|a|,|b|))", e);
                    CHECK(e <= TOL_LOOSE, "bchol_solve matches ref_chol_solve [survey]");
                }
                {
                    uwb_real resid[TEST_MAXN];
                    double rn = 0.0, bn = 0.0, acc, e;
                    int r, cc;
                    for (r = 0; r < dim; ++r) {
                        acc = 0.0;
                        for (cc = 0; cc < dim; ++cc) acc += (double)full[r * dim + cc] * (double)x[cc];
                        resid[r] = R(acc - (double)b_rhs[r]);
                        rn += (double)resid[r] * (double)resid[r];
                        bn += (double)b_rhs[r] * (double)b_rhs[r];
                    }
                    rn = sqrt(rn);
                    bn = sqrt(bn);
                    if (bn < 1.0) bn = 1.0;
                    e = rn / bn;
                    stat_note("bchol survey residual ||Ax-b||/||b||", e);
                    CHECK(e <= TOL_LOOSE, "bchol_solve residual is small [survey]");
                }
            }
        }
    }
}

static void test_bchol_failure_modes(void)
{
    int nb;
    uwb_bchol m;

    /* lambda=ridge=0 の測量型 (完全グラフ) はゲージ不定で特異 */
    for (nb = 2; nb <= 5; ++nb) {
        uwb_real pos[UWB_BCHOL_MAX_BLOCKS][3];
        int i, j;
        for (i = 0; i < nb; ++i) {
            pos[i][0] = R(urand() * 10.0 - 5.0);
            pos[i][1] = R(urand() * 10.0 - 5.0);
            pos[i][2] = R(urand() * 10.0 - 5.0);
        }
        uwb_bchol_zero(&m, nb, 0);
        for (i = 0; i < nb; ++i) {
            for (j = 0; j < i; ++j) {
                uwb_real diff[3], u[3], len;
                uwb_v3_sub(pos[i], pos[j], diff);
                len = uwb_v3_normalize(diff);
                if (!(len > R(1e-6))) continue;
                u[0] = diff[0]; u[1] = diff[1]; u[2] = diff[2];
                uwb_bchol_add_pair_outer(&m, i, j, u, R(0));
            }
        }
        CHECK(uwb_bchol_factor(&m) == 0, "bchol_factor fails on a gauge-indeterminate (undamped) system");
    }

    /* factor されていない構造体で solve すると失敗する */
    {
        uwb_real b[6] = { R(0), R(0), R(0), R(0), R(0), R(0) };
        uwb_real x[6];
        uwb_bchol_zero(&m, 2, 0);
        CHECK(uwb_bchol_solve(&m, b, x) == 0, "bchol_solve fails on a non-factored structure");
    }

    /* nb が範囲外 */
    CHECK(uwb_bchol_zero(&m, 0, 0) == 0, "bchol_zero fails for nb=0");
    CHECK(uwb_bchol_zero(&m, UWB_BCHOL_MAX_BLOCKS + 1, 0) == 0, "bchol_zero fails for nb=MAX+1");
    CHECK(uwb_bchol_zero(&m, -1, 0) == 0, "bchol_zero fails for nb<0");
}

/* ================================================================= main */

int main(void)
{
    test_vec3();
    test_sym3_ops();
    test_sym3_solve_family();
    test_sym3_singular();
    test_sym3_eigvals();
    test_sym3_rank();
    test_sym3_eigvec();
    test_sym3_eig();
    test_sym3_reflect();
    test_sym2();
    test_symn_rank1();
    test_m3();
    test_bchol_dense();
    test_bchol_survey();
    test_bchol_failure_modes();

    stat_report();

    if (g_fail == 0) {
        printf("OK  %d 件すべて通った\n", g_total);
        return 0;
    }
    printf("=== %d 件中 %d 件失敗 ===\n", g_total, g_fail);
    return 1;
}
