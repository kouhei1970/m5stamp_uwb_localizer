/* C 版の検証。
 *
 * 数値の一致だけでなく、**アルゴリズムが持つべき性質**を確かめる
 * (Python 版のテストと同じ方針):
 *
 *   * 無雑音なら閉形式が真値を返す
 *   * Beck 法は LLS より偏りが小さい
 *   * NLOS が 1 本混じっても Lv2 は Lv1 ほど崩れない
 *   * EKF は 1 本ずつの更新でも追従する
 *   * 同一平面配置で鏡像が検出される
 *   * 数値が壊れても NaN を返さず ok=0 で返る
 */
#include "uwb_loc.h"
#include "uwb_linalg.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
static int g_run = 0;

/* 許容誤差は精度に合わせる。float は仮数 24 bit ≒ 有効 7 桁しかないので、
 * double と同じしきい値では「アルゴリズムは正しいのに落ちる」ことになる。
 * 数字を緩めているのではなく、**単精度で出せる精度を書いている**。 */
#if UWB_REAL_IS_FLOAT
#define TOL_EXACT  2e-4    /* 無雑音での位置一致 [m] */
#define TOL_TIGHT  1e-5    /* 幾何的に厳密なはずの量 */
/* 実測値 (4x4, make float): 残差最大 4.5e-6 / 直交性最大 3.5e-7。
 * 余裕を持たせて一桁以上離す。 */
#define TOL_EIGVEC 1e-4    /* Jacobi 固有ベクトルの残差 |A v - lambda v| */
#define TOL_ORTHO  1e-5    /* 固有ベクトルの直交性 (V^T V の単位行列からのずれ) */
#else
#define TOL_EXACT  1e-6
#define TOL_TIGHT  1e-9
/* 実測値 (4x4, make test): 残差最大 4.0e-15 / 直交性最大 6.7e-16。 */
#define TOL_EIGVEC 1e-9
#define TOL_ORTHO  1e-9
#endif

#define CHECK(cond, ...)                                                       \
    do {                                                                       \
        ++g_run;                                                               \
        if (!(cond)) {                                                         \
            ++g_fail;                                                          \
            printf("  NG  %s:%d  ", __FILE__, __LINE__);                       \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
        }                                                                      \
    } while (0)

static double dist3(const uwb_real *a, const uwb_real *b)
{
    double dx = (double)a[0] - (double)b[0];
    double dy = (double)a[1] - (double)b[1];
    double dz = (double)a[2] - (double)b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* 高さをばらした 6 台。3 次元で解ける最低限より少し余裕がある配置。 */
static uwb_anchor ANCH[6] = {
    {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A1", {7.8, 0.2, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A4", {4.0, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A5", {4.0, 5.8, 0.3}, 1, 0.0, 0.08, 0.0}
};

/* 決定的な擬似乱数 (再現性のため自前で持つ) */
static unsigned long g_seed = 12345UL;
static double urand(void)
{
    g_seed = g_seed * 1103515245UL + 12345UL;
    return (double)((g_seed >> 16) & 0x7fff) / 32767.0;
}
static double nrand(void)  /* Box-Muller */
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static void make_meas(const uwb_config *cfg, const uwb_real *truth,
                      uwb_meas *m, int n, double noise)
{
    int i;
    for (i = 0; i < n; ++i) {
        m[i].anchor = i;
        m[i].value = (uwb_real)(dist3(truth, cfg->anchors[i].p) + noise * nrand());
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }
}

/* ------------------------------------------------------------------ */

static void test_exact_when_noiseless(void)
{
    uwb_config cfg;
    uwb_meas m[6];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};
    uwb_real p[3];

    uwb_config_init(&cfg, ANCH, 6);
    make_meas(&cfg, truth, m, 6, 0.0);

    CHECK(uwb_lls_trilateration(&cfg, m, 6, p), "LLS が解けない");
    CHECK(dist3(p, truth) < TOL_EXACT, "LLS 誤差 %.3g m", dist3(p, truth));

    CHECK(uwb_beck_gtrs(&cfg, m, 6, p), "Beck が解けない");
    CHECK(dist3(p, truth) < TOL_EXACT, "Beck 誤差 %.3g m", dist3(p, truth));

    CHECK(uwb_solve_lv0(&cfg, m, 6, &fix) && fix.ok, "Lv0 が解けない");
    CHECK(dist3(fix.p, truth) < TOL_EXACT, "Lv0 誤差 %.3g m", dist3(fix.p, truth));

    CHECK(uwb_solve_lv1(&cfg, m, 6, &fix) && fix.ok, "Lv1 が解けない");
    CHECK(dist3(fix.p, truth) < TOL_EXACT, "Lv1 誤差 %.3g m", dist3(fix.p, truth));

    CHECK(uwb_solve_lv2(&cfg, m, 6, &fix) && fix.ok, "Lv2 が解けない");
    CHECK(dist3(fix.p, truth) < TOL_EXACT, "Lv2 誤差 %.3g m", dist3(fix.p, truth));
}

static void test_minimum_anchor_count(void)
{
    uwb_config cfg;
    uwb_meas m[6];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};

    uwb_config_init(&cfg, ANCH, 6);
    make_meas(&cfg, truth, m, 6, 0.0);

    /* 3 次元は未知数 3 つ。4 本あれば解け、3 本では解けない。 */
    CHECK(uwb_solve_lv2(&cfg, m, 4, &fix) && fix.ok, "4 本で解けるはず");
    CHECK(!uwb_solve_lv2(&cfg, m, 3, &fix), "3 本では解けないはず");
    CHECK(fix.ok == 0, "解けないとき ok=0 であるべき");
    CHECK(!uwb_solve_lv2(&cfg, m, 0, &fix), "0 本では解けないはず");
}

static void test_beck_beats_lls_on_noise(void)
{
    /* Beck は差分をとらないので偏りが小さい。多数回の平均で比べる。 */
    uwb_config cfg;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};
    double sum_lls = 0.0, sum_beck = 0.0;
    int i, n = 300;

    uwb_config_init(&cfg, ANCH, 6);
    g_seed = 999UL;
    for (i = 0; i < n; ++i) {
        uwb_meas m[6];
        uwb_real a[3], b[3];
        make_meas(&cfg, truth, m, 6, 0.08);
        if (uwb_lls_trilateration(&cfg, m, 6, a)) sum_lls += dist3(a, truth);
        if (uwb_beck_gtrs(&cfg, m, 6, b)) sum_beck += dist3(b, truth);
    }
    printf("     LLS 平均誤差 %.3f m / Beck 平均誤差 %.3f m\n",
           sum_lls / n, sum_beck / n);
    CHECK(sum_beck <= sum_lls, "Beck が LLS より悪い (%.3f > %.3f)",
          sum_beck / n, sum_lls / n);
}

static void test_lv2_resists_one_nlos(void)
{
    /* NLOS は距離を伸ばす。1 本混ぜて Lv1 と Lv2 を比べる。 */
    uwb_config cfg;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};
    double sum1 = 0.0, sum2 = 0.0;
    int i, n = 200;

    uwb_config_init(&cfg, ANCH, 6);
    g_seed = 4242UL;
    for (i = 0; i < n; ++i) {
        uwb_meas m[6];
        uwb_fix f1, f2;
        make_meas(&cfg, truth, m, 6, 0.05);
        m[2].value += (uwb_real)1.5;             /* A2 だけ 1.5 m 伸びた */
        if (uwb_solve_lv1(&cfg, m, 6, &f1) && f1.ok) sum1 += dist3(f1.p, truth);
        if (uwb_solve_lv2(&cfg, m, 6, &f2) && f2.ok) sum2 += dist3(f2.p, truth);
    }
    printf("     NLOS 1 本: Lv1 %.3f m / Lv2 %.3f m\n", sum1 / n, sum2 / n);
    CHECK(sum2 < sum1, "Lv2 が Lv1 より崩れている (%.3f >= %.3f)", sum2 / n, sum1 / n);
}

static void test_outlier_is_excluded(void)
{
    uwb_config cfg;
    uwb_meas m[6];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};

    uwb_config_init(&cfg, ANCH, 6);
    make_meas(&cfg, truth, m, 6, 0.0);
    m[3].value += (uwb_real)3.0;                 /* 明らかな外れ値 */

    CHECK(uwb_solve_lv2(&cfg, m, 6, &fix) && fix.ok, "Lv2 が解けない");
    CHECK((fix.excluded & (1UL << 3)) != 0UL, "A3 が除外されていない (mask=%lu)",
          fix.excluded);
    CHECK(dist3(fix.p, truth) < 0.2, "外れ値に引っぱられた (%.3f m)", dist3(fix.p, truth));
}

static void test_2d_mode(void)
{
    uwb_config cfg;
    uwb_meas m[6];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};

    uwb_config_init(&cfg, ANCH, 6);
    cfg.dim = 2;
    cfg.z_fixed = (uwb_real)1.2;
    make_meas(&cfg, truth, m, 6, 0.0);

    CHECK(uwb_solve_lv2(&cfg, m, 6, &fix) && fix.ok, "2D で解けない");
    CHECK(fabs((double)fix.p[2] - (double)(uwb_real)1.2) < TOL_TIGHT,
          "z が固定されていない (%.6f)", (double)fix.p[2]);
    CHECK(dist3(fix.p, truth) < TOL_EXACT, "2D 誤差 %.3g m", dist3(fix.p, truth));
}

static void test_coplanar_is_detected(void)
{
    /* 天井 4 隅 = 同一平面。3 次元では鏡像が原理的に選べない。 */
    static uwb_anchor flat[4] = {
        {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
        {"A1", {7.8, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
        {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
        {"A3", {0.2, 5.8, 2.4}, 1, 0.0, 0.08, 0.0}
    };
    uwb_config cfg;
    uwb_real nrm[3], off;
    uwb_meas m[4];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)4.0, (uwb_real)3.0, (uwb_real)1.2};
    uwb_real mirror[3];
    int i;

    uwb_config_init(&cfg, flat, 4);
    CHECK(uwb_anchors_coplanar(&cfg, nrm, &off), "同一平面を検出できていない");
    CHECK(fabs(fabs((double)nrm[2]) - 1.0) < TOL_EXACT, "法線が z 方向でない (%.4f)", (double)nrm[2]);

    /* 真値と鏡像で、全アンカーからの距離が完全に一致することを確かめる */
    mirror[0] = truth[0]; mirror[1] = truth[1];
    mirror[2] = (uwb_real)(2.0 * 2.4 - 1.2);
    for (i = 0; i < 4; ++i) {
        double a = dist3(truth, flat[i].p), b = dist3(mirror, flat[i].p);
        CHECK(fabs(a - b) < TOL_TIGHT, "鏡像の距離が一致しない (%.9f vs %.9f)", a, b);
    }

    make_meas(&cfg, truth, m, 4, 0.0);
    CHECK(uwb_solve_lv2(&cfg, m, 4, &fix) && fix.ok, "同一平面でも解は出るはず");
    CHECK(fix.ambiguous == 1, "ambiguous が立っていない");

    /* z の範囲を教えれば片側に決まる */
    cfg.use_z_bounds = 1;
    cfg.z_min = (uwb_real)0;
    cfg.z_max = (uwb_real)2.3;
    CHECK(uwb_solve_lv2(&cfg, m, 4, &fix) && fix.ok, "z_bounds つきで解けない");
    CHECK(fix.ambiguous == 0, "z_bounds を与えたのに ambiguous のまま");
    CHECK(fix.p[2] <= (uwb_real)2.3 + (uwb_real)1e-6, "z が範囲外 (%.3f)", (double)fix.p[2]);
}

static void test_gdop_and_crlb(void)
{
    static uwb_anchor flat[4] = {
        {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
        {"A1", {7.8, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
        {"A2", {7.8, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
        {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0}
    };
    uwb_config good, bad;
    uwb_real center[3] = {(uwb_real)4.0, (uwb_real)3.0, (uwb_real)1.2};
    uwb_real g_good, g_bad, crlb;

    uwb_config_init(&good, ANCH, 6);
    uwb_config_init(&bad, flat, 4);

    g_good = uwb_gdop_at(&good, center);
    g_bad = uwb_gdop_at(&bad, center);
    printf("     GDOP: 6 台立体 %.2f / 4 台平面 %.2f\n", (double)g_good, (double)g_bad);
    CHECK(g_good > 0, "GDOP が出ない");
    CHECK(g_good < g_bad, "良い配置の GDOP が悪い配置より大きい");

    crlb = uwb_crlb_at(&good, center);
    CHECK(crlb > 0 && crlb < 1.0, "CRLB が変 (%.3f)", (double)crlb);
}

static void test_sym_eig_vectors(void)
{
    /* 対称だが特別な構造のない 4x4。恒等行列だと固有ベクトルが何でも
     * 通ってしまうので検証にならない。 */
    uwb_real A[16] = {
        4, 1, 0, 2,
        1, 3, 1, 0,
        0, 1, 5, 1,
        2, 0, 1, 6
    };
    uwb_real orig[16];   /* uwb_sym_eig は a を破壊するので残しておく */
    uwb_real work[16];
    uwb_real eig[4];
    uwb_real vec[16];
    int i, j, k;

    memcpy(orig, A, sizeof(A));
    memcpy(work, A, sizeof(A));

    CHECK(uwb_sym_eig(work, eig, vec, 4), "固有値・固有ベクトルが求まらない");

    for (i = 1; i < 4; ++i)
        CHECK(eig[i - 1] <= eig[i] + (uwb_real)TOL_TIGHT,
              "固有値が昇順でない (%.6f > %.6f)", (double)eig[i - 1], (double)eig[i]);

    /* 固有ベクトル残差 A v_i == eig_i v_i。元の (破壊されていない) 行列で確認する。
     * vec[row*4+col] が固有ベクトル col の第 row 成分。 */
    for (i = 0; i < 4; ++i) {
        double res = 0.0;
        for (j = 0; j < 4; ++j) {
            double av = 0.0;
            for (k = 0; k < 4; ++k) av += (double)orig[j * 4 + k] * (double)vec[k * 4 + i];
            double diff = av - (double)eig[i] * (double)vec[j * 4 + i];
            res += diff * diff;
        }
        res = sqrt(res);
        CHECK(res < TOL_EIGVEC, "固有ベクトル残差が大きい (列 %d, %.3g)", i, res);
    }

    /* 直交性 V^T V == I */
    for (i = 0; i < 4; ++i) {
        for (j = 0; j < 4; ++j) {
            double dot = 0.0, want = (i == j) ? 1.0 : 0.0;
            for (k = 0; k < 4; ++k) dot += (double)vec[k * 4 + i] * (double)vec[k * 4 + j];
            CHECK(fabs(dot - want) < TOL_ORTHO,
                  "V^T V が単位行列でない (%d,%d)=%.3g", i, j, dot);
        }
    }
}

static void test_ekf_tracks_a_moving_tag(void)
{
    uwb_config cfg;
    uwb_ekf ekf;
    double sum = 0.0;
    int i, n = 0;

    uwb_config_init(&cfg, ANCH, 6);
    uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)1.0);
    g_seed = 777UL;

    for (i = 0; i < 200; ++i) {
        uwb_real t = (uwb_real)(i * 0.1);
        uwb_real truth[3];
        uwb_meas m[6];
        uwb_fix fix;
        /* 円運動 */
        truth[0] = (uwb_real)(4.0 + 2.0 * cos(i * 0.05));
        truth[1] = (uwb_real)(3.0 + 2.0 * sin(i * 0.05));
        truth[2] = (uwb_real)1.2;
        make_meas(&cfg, truth, m, 6, 0.06);
        if (uwb_ekf_update(&ekf, t, m, 6, &fix) && fix.ok && i > 20) {
            sum += dist3(fix.p, truth);
            ++n;
        }
    }
    CHECK(n > 150, "EKF が追従していない (成功 %d 回)", n);
    printf("     EKF 平均誤差 %.3f m (%d エポック)\n", sum / (n ? n : 1), n);
    CHECK(sum / (n ? n : 1) < 0.25, "EKF の誤差が大きい (%.3f m)", sum / (n ? n : 1));
}

static void test_ekf_bootstraps_from_one_range_at_a_time(void)
{
    /* 1 本ずつ届く経路 (順繰りの TWR、BLE 通知) でも立ち上がるか。
     * 測距 1 本は球面 1 枚でしかないので、貯めて揃うのを待つ実装。 */
    uwb_config cfg;
    uwb_ekf ekf;
    uwb_fix fix;
    int i, ok_count = 0;

    uwb_config_init(&cfg, ANCH, 6);
    uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)1.0);
    g_seed = 31337UL;

    for (i = 0; i < 300; ++i) {
        uwb_real t = (uwb_real)(i * 0.02);
        uwb_real truth[3] = {(uwb_real)3.5, (uwb_real)2.5, (uwb_real)1.2};
        uwb_meas one;
        int a = i % 6;
        one.anchor = a;
        one.value = (uwb_real)(dist3(truth, ANCH[a].p) + 0.05 * nrand());
        one.sigma = (uwb_real)0;
        one.quality = (uwb_real)-1;
        if (uwb_ekf_update(&ekf, t, &one, 1, &fix) && fix.ok) {
            ++ok_count;
            if (i > 100) {
                CHECK(dist3(fix.p, truth) < 0.5,
                      "1 本ずつ更新の誤差が大きい (%.3f m, i=%d)", dist3(fix.p, truth), i);
                break;
            }
        }
    }
    CHECK(ok_count > 0, "1 本ずつでは立ち上がらなかった");
}

static void test_ekf_recovers_from_gate_lockout(void)
{
    /* ゲートが自分の誤りを守り続ける「棺桶問題」からの復帰。 */
    uwb_config cfg;
    uwb_ekf ekf;
    uwb_fix fix;
    uwb_real here[3] = {(uwb_real)2.0, (uwb_real)2.0, (uwb_real)1.2};
    uwb_real there[3] = {(uwb_real)6.5, (uwb_real)5.0, (uwb_real)1.2};
    int i;

    uwb_config_init(&cfg, ANCH, 6);
    uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)0.05); /* わざと鈍くする */
    g_seed = 5150UL;

    for (i = 0; i < 40; ++i) {
        uwb_meas m[6];
        make_meas(&cfg, here, m, 6, 0.02);
        uwb_ekf_update(&ekf, (uwb_real)(i * 0.1), m, 6, &fix);
    }
    /* いきなり瞬間移動させる。ゲートが全部弾くはず。 */
    for (i = 40; i < 80; ++i) {
        uwb_meas m[6];
        make_meas(&cfg, there, m, 6, 0.02);
        uwb_ekf_update(&ekf, (uwb_real)(i * 0.1), m, 6, &fix);
    }
    CHECK(fix.ok, "復帰後に ok が立っていない");
    CHECK(dist3(fix.p, there) < 1.0, "棺桶問題から復帰できていない (%.3f m)",
          dist3(fix.p, there));
}

static void test_no_nan_on_degenerate_input(void)
{
    /* 一直線に並べる = 3 次元では解けない配置。NaN を返さず ok=0 で返ること。 */
    static uwb_anchor line[5] = {
        {"L0", {0.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0},
        {"L1", {1.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0},
        {"L2", {2.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0},
        {"L3", {3.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0},
        {"L4", {4.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0}
    };
    uwb_config cfg;
    uwb_meas m[5];
    uwb_fix fix;
    int i, k;

    uwb_config_init(&cfg, line, 5);
    for (i = 0; i < 5; ++i) {
        m[i].anchor = i;
        m[i].value = (uwb_real)2.0;
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }
    uwb_solve_lv2(&cfg, m, 5, &fix);
    for (k = 0; k < 3; ++k)
        CHECK(fix.p[k] == fix.p[k], "退化入力で NaN が出た (p[%d])", k);

    /* 全部同じ点 = 完全に退化 */
    for (i = 0; i < 5; ++i) { m[i].value = (uwb_real)0; }
    uwb_solve_lv2(&cfg, m, 5, &fix);
    for (k = 0; k < 3; ++k)
        CHECK(fix.p[k] == fix.p[k], "ゼロ距離で NaN が出た (p[%d])", k);
}

static void test_antenna_delay_and_quality(void)
{
    uwb_anchor delayed[6];
    uwb_config cfg;
    uwb_meas m[6];
    uwb_fix fix;
    uwb_real truth[3] = {(uwb_real)3.1, (uwb_real)2.4, (uwb_real)1.2};
    int i;

    memcpy(delayed, ANCH, sizeof(ANCH));
    for (i = 0; i < 6; ++i) delayed[i].antenna_delay_m = (uwb_real)0.25;

    uwb_config_init(&cfg, delayed, 6);
    /* 実測は 25 cm 長く出る。アンカー側に遅延を持たせてあるので相殺されるはず。 */
    for (i = 0; i < 6; ++i) {
        m[i].anchor = i;
        m[i].value = (uwb_real)(dist3(truth, delayed[i].p) + 0.25);
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }
    CHECK(uwb_solve_lv2(&cfg, m, 6, &fix) && fix.ok, "遅延つきで解けない");
    CHECK(dist3(fix.p, truth) < TOL_EXACT, "アンテナ遅延が補正されていない (%.4f m)",
          dist3(fix.p, truth));

    /* 品質値が低いと sigma が膨らみ、共分散が大きくなる */
    {
        uwb_fix low;
        for (i = 0; i < 6; ++i) m[i].quality = (uwb_real)0.0;
        CHECK(uwb_solve_lv2(&cfg, m, 6, &low) && low.ok, "品質値つきで解けない");
        CHECK(low.sigma > fix.sigma, "品質が低いのに sigma が増えていない");
    }
}

static void test_version_matches_python(void)
{
    CHECK(strcmp(uwb_version(), "0.1.0") == 0, "バージョンが違う: %s", uwb_version());
}

/* ------------------------------------------------------------------ */

int main(void)
{
    printf("uwb_loc C 版のテスト (%s)\n",
           UWB_REAL_IS_FLOAT ? "float" : "double");
    printf("----------------------------------------------------------\n");

    test_version_matches_python();
    test_exact_when_noiseless();
    test_minimum_anchor_count();
    test_beck_beats_lls_on_noise();
    test_lv2_resists_one_nlos();
    test_outlier_is_excluded();
    test_2d_mode();
    test_coplanar_is_detected();
    test_gdop_and_crlb();
    test_sym_eig_vectors();
    test_ekf_tracks_a_moving_tag();
    test_ekf_bootstraps_from_one_range_at_a_time();
    test_ekf_recovers_from_gate_lockout();
    test_no_nan_on_degenerate_input();
    test_antenna_delay_and_quality();

    printf("----------------------------------------------------------\n");
    if (g_fail == 0) {
        printf("OK  %d 件すべて通った\n", g_run);
        return 0;
    }
    printf("NG  %d / %d 件が失敗\n", g_fail, g_run);
    return 1;
}
