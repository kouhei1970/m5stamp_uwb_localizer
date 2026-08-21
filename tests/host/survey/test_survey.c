/**
 * @file test_survey.c
 * @brief components/uwb_survey（アンカー自動測量の計算本体）を、実機なしで
 *        合成データだけで検証するホスト側テスト（docs/SURVEY_SPEC.md の S6）。
 *
 * 既知配置 → 理論距離 → 復元 → 誤差評価、という流れ。乱数は自前の
 * xorshift32 に固定シードを与えるので、**どの環境でも同じ数値が出る**
 * （libc の rand() は実装依存なので使わない）。
 *
 * tests/host/loc/test_uwb.c および tests/host/pipeline
 * と同じ CHECK() マクロの流儀（PASS/FAIL をその場で数え、失敗時だけ内容を
 * 表示する）を踏襲する。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "uwb_survey.h"
#include "uwb_survey_dense.h"

static int g_run  = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                     \
    do {                                                     \
        ++g_run;                                             \
        if (!(cond)) {                                       \
            ++g_fail;                                        \
            printf("  NG  %s:%d  ", __FILE__, __LINE__);     \
            printf(__VA_ARGS__);                             \
            printf("\n");                                    \
        }                                                    \
    } while (0)

/* 単精度ビルド (-DUWB_USE_FLOAT) では桁落ちで cm 級までしか詰められない。 */
#if UWB_REAL_IS_FLOAT
#define TOL_EXACT ((uwb_real)2e-3)   /* 復元座標 [m] */
#define TOL_LIN   1e-5               /* 線形ソルバ・固有分解の解の誤差 */
#define TOL_ORTH  1e-5               /* 固有ベクトルの直交性・トレース保存（相対） */
#else
#define TOL_EXACT ((uwb_real)1e-6)
#define TOL_LIN   1e-9
#define TOL_ORTH  1e-12
#endif

#define PI_D 3.14159265358979323846

/* ==================================================================== *
 * 既知配置（8ノード）
 *
 * どの前置き部分（先頭 4..8 台）を取っても
 *   - 同一平面でない
 *   - ノード0 が XY 原点、ノード1 が +X 軸上（y=0, x>0）
 *   - 添字 2 以上で |y| が最大なのはノード3（y=+4.9 > 0）
 * となるように選んである。つまり **真値そのものが uwb_survey の
 * 正規形**なので、復元結果と直接比べられる。
 * ==================================================================== */
static const double TRUTH[8][3] = {
    {0.00, 0.00, 0.30},  /* 0: 床置き */
    {5.00, 0.00, 2.40},  /* 1: 天井付近 */
    {4.20, 4.60, 0.35},
    {0.40, 4.90, 2.35},
    {2.50, 2.20, 2.45},
    {2.20, 1.50, 0.90},  /* 5: タグ（6台目として測量に参加） */
    {4.80, 2.50, 1.80},
    {1.00, 3.60, 0.15},
};

/* ------------------------------------------------------------ 乱数 */

typedef struct { unsigned int s; } rng_t;

static void rng_seed(rng_t *r, unsigned int seed) { r->s = seed ? seed : 1u; }

static unsigned int rng_next(rng_t *r)
{
    unsigned int x = r->s;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    r->s = x;
    return x;
}

/* [0,1) の一様乱数。上位 24bit だけ使う。 */
static double rng_uniform(rng_t *r)
{
    return (double)(rng_next(r) >> 8) * (1.0 / 16777216.0);
}

/* Box-Muller。標準正規。 */
static double rng_normal(rng_t *r)
{
    double u1 = rng_uniform(r);
    double u2 = rng_uniform(r);
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * PI_D * u2);
}

/* ------------------------------------------------------------ 小道具 */

static double truth_dist(int i, int j)
{
    double dx = TRUTH[i][0] - TRUTH[j][0];
    double dy = TRUTH[i][1] - TRUTH[j][1];
    double dz = TRUTH[i][2] - TRUTH[j][2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double res_dist(const uwb_survey_result *o, int i, int j)
{
    double dx = (double)(o->pos[i][0] - o->pos[j][0]);
    double dy = (double)(o->pos[i][1] - o->pos[j][1]);
    double dz = (double)(o->pos[i][2] - o->pos[j][2]);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/** 真値と同じ座標系で組んだ入力を作る（理論距離 + 全ノードの高さ実測）。
 *  delay は片道のアンテナ遅延 [m]（観測は r = d + 2*delay）。 */
static void make_input(uwb_survey_input *in, int n, double delay)
{
    int i, j;
    uwb_survey_input_init(in, n);
    for (i = 0; i < n; ++i) {
        for (j = i + 1; j < n; ++j) {
            uwb_real v = (uwb_real)(truth_dist(i, j) + 2.0 * delay);
            in->dist[i][j] = in->dist[j][i] = v;
            in->have[i][j] = in->have[j][i] = 1;
        }
        in->height[i]      = (uwb_real)TRUTH[i][2];
        in->have_height[i] = 1;
    }
}

/** 距離に平均0・標準偏差 sigma のガウス雑音を乗せる（対称性は保つ）。 */
static void add_dist_noise(uwb_survey_input *in, int n, double sigma, rng_t *r)
{
    int i, j;
    for (i = 0; i < n; ++i)
        for (j = i + 1; j < n; ++j) {
            uwb_real v;
            if (!in->have[i][j]) continue;
            v = (uwb_real)((double)in->dist[i][j] + sigma * rng_normal(r));
            in->dist[i][j] = in->dist[j][i] = v;
        }
}

static void add_height_noise(uwb_survey_input *in, int n, double sigma, rng_t *r)
{
    int i;
    for (i = 0; i < n; ++i)
        if (in->have_height[i])
            in->height[i] = (uwb_real)((double)in->height[i] + sigma * rng_normal(r));
}

static void drop_link(uwb_survey_input *in, int i, int j)
{
    in->have[i][j] = in->have[j][i] = 0;
}

/** 先頭 keep 台だけ高さ実測を残す。 */
static void keep_heights(uwb_survey_input *in, int n, int keep)
{
    int i;
    for (i = keep; i < n; ++i) in->have_height[i] = 0;
}

/** 真値との最大座標誤差 [m]。 */
static double max_pos_err(const uwb_survey_result *o, int n)
{
    double worst = 0.0;
    int i, k;
    for (i = 0; i < n; ++i)
        for (k = 0; k < 3; ++k) {
            double e = fabs((double)o->pos[i][k] - TRUTH[i][k]);
            if (e > worst) worst = e;
        }
    return worst;
}

/** 真値との座標誤差 RMS（3成分をまとめて） [m]。 */
static double rms_pos_err(const uwb_survey_result *o, int n)
{
    double s = 0.0;
    int i, k;
    for (i = 0; i < n; ++i)
        for (k = 0; k < 3; ++k) {
            double e = (double)o->pos[i][k] - TRUTH[i][k];
            s += e * e;
        }
    return sqrt(s / (double)(3 * n));
}

/** ノード間距離の最大誤差 [m]（座標系に依存しない「形」の誤差）。 */
static double max_shape_err(const uwb_survey_result *o, int n)
{
    double worst = 0.0;
    int i, j;
    for (i = 0; i < n; ++i)
        for (j = i + 1; j < n; ++j) {
            double e = fabs(res_dist(o, i, j) - truth_dist(i, j));
            if (e > worst) worst = e;
        }
    return worst;
}

/** 正規形（ノード0 が XY 原点、ノード1 が +X 軸上）になっているか。 */
static void check_convention(const uwb_survey_result *o, const char *tag)
{
    CHECK(fabs((double)o->pos[0][0]) < (double)TOL_EXACT &&
              fabs((double)o->pos[0][1]) < (double)TOL_EXACT,
          "%s: ノード0 が XY 原点でない (%.9f, %.9f)", tag, (double)o->pos[0][0],
          (double)o->pos[0][1]);
    CHECK(fabs((double)o->pos[1][1]) < (double)TOL_EXACT, "%s: ノード1 が y=0 でない (%.9f)", tag,
          (double)o->pos[1][1]);
    CHECK((double)o->pos[1][0] > 0.0, "%s: ノード1 が +X 側にない (%.9f)", tag,
          (double)o->pos[1][0]);
}

/* ==================================================================== *
 * 1. 理論距離からの完全復元（6ノード・非同一平面）
 * ==================================================================== */
static void scenario1_exact(void)
{
    uwb_survey_input  in;
    uwb_survey_result out;
    double err;

    printf("--- 1. 理論距離からの完全復元（6ノード） ---\n");

    make_input(&in, 6, 0.0);
    CHECK(uwb_survey_solve(&in, &out) == 1, "解けなかった");
    CHECK(out.ok == 1, "ok=0");
    CHECK(out.degenerate == 0, "非同一平面なのに degenerate=1");
    CHECK(out.excluded == 0UL, "外れ値が無いのに excluded=0x%lX", out.excluded);

    err = max_pos_err(&out, 6);
    printf("    最大座標誤差 = %.3e m / 残差RMS = %.3e m / 反復 %d 回\n", err,
           (double)out.residual_rms, out.iterations);
    CHECK(err < (double)TOL_EXACT, "復元誤差が大きい: %.3e m", err);
    CHECK((double)out.residual_rms < (double)TOL_EXACT, "距離残差 RMS が大きい: %.3e m",
          (double)out.residual_rms);
    CHECK(fabs((double)out.common_delay_m) < (double)TOL_EXACT,
          "遅延0のはずが %.6f m", (double)out.common_delay_m);
    check_convention(&out, "exact6");
}

/* ==================================================================== *
 * 2. ノイズ耐性（sigma = 5cm）
 * ==================================================================== */
static void noise_case(int n, double sigma_d, double sigma_h, unsigned int seed, int trials)
{
    double worst = 0.0, sum_rms = 0.0, worst_delay = 0.0, worst_shape = 0.0;
    int    t, nfail = 0;
    rng_t  rng;

    rng_seed(&rng, seed);
    for (t = 0; t < trials; ++t) {
        uwb_survey_input  in;
        uwb_survey_result out;
        double e;

        make_input(&in, n, 0.0);
        add_dist_noise(&in, n, sigma_d, &rng);
        if (sigma_h > 0.0) add_height_noise(&in, n, sigma_h, &rng);

        if (!uwb_survey_solve(&in, &out) || !out.ok) { ++nfail; continue; }
        e = max_pos_err(&out, n);
        if (e > worst) worst = e;
        sum_rms += rms_pos_err(&out, n);
        if (fabs((double)out.common_delay_m) > worst_delay)
            worst_delay = fabs((double)out.common_delay_m);
        e = max_shape_err(&out, n);
        if (e > worst_shape) worst_shape = e;
    }

    printf("    n=%d sigma_d=%.0fcm sigma_h=%.0fcm x%d回: "
           "最大座標誤差 %.4f m / 平均RMS %.4f m / 最大形状誤差 %.4f m / |δ|最大 %.4f m\n",
           n, sigma_d * 100.0, sigma_h * 100.0, trials, worst, sum_rms / (double)trials,
           worst_shape, worst_delay);
    CHECK(nfail == 0, "n=%d のノイズ試行で %d 回解けなかった", n, nfail);
}

static void scenario2_noise(void)
{
    printf("--- 2. ノイズ耐性（距離 sigma=5cm） ---\n");
    noise_case(6, 0.05, 0.00, 12345u, 50);
    noise_case(6, 0.05, 0.01, 22222u, 50);
    noise_case(8, 0.05, 0.01, 33333u, 50);
    noise_case(5, 0.05, 0.01, 44444u, 50);

    /* 5cm ノイズなら、6ノードでも座標誤差は 15cm を超えない（実測値は上の表示）。 */
    {
        uwb_survey_input  in;
        uwb_survey_result out;
        rng_t             rng;
        double            e;
        rng_seed(&rng, 777u);
        make_input(&in, 6, 0.0);
        add_dist_noise(&in, 6, 0.05, &rng);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "ノイズ付きで解けなかった");
        e = max_pos_err(&out, 6);
        CHECK(e < 0.30, "5cm ノイズで座標誤差 %.3f m は大きすぎる", e);
        CHECK(out.excluded == 0UL,
              "5cm のばらつきでリンクを落としてはいけない (excluded=0x%lX)", out.excluded);
    }
}

/* ==================================================================== *
 * 3. 共通遅延の推定
 * ==================================================================== */
static void scenario3_delay(void)
{
    static const double delays[] = {0.0, 0.05, 0.10, 0.1543, 0.30, -0.04};
    size_t k;

    printf("--- 3. 共通アンテナ遅延の推定 ---\n");

    for (k = 0; k < sizeof(delays) / sizeof(delays[0]); ++k) {
        uwb_survey_input  in;
        uwb_survey_result out;
        double            de, pe;

        make_input(&in, 6, delays[k]);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "δ=%.4f で解けなかった", delays[k]);
        de = fabs((double)out.common_delay_m - delays[k]);
        pe = max_pos_err(&out, 6);
        printf("    δ=%+.4f m → 推定 %+.9f m (誤差 %.3e) / 座標誤差 %.3e m\n", delays[k],
               (double)out.common_delay_m, de, pe);
        CHECK(de < (double)TOL_EXACT, "δ=%.4f の推定誤差が大きい: %.3e", delays[k], de);
        CHECK(pe < (double)TOL_EXACT, "δ=%.4f で座標誤差が大きい: %.3e", delays[k], pe);
    }

    /* ノイズ入りでの遅延推定精度（8ノード = リンク28本で冗長度が高い）。 */
    {
        rng_t  rng;
        double worst = 0.0, sum = 0.0;
        int    t;
        rng_seed(&rng, 98765u);
        for (t = 0; t < 50; ++t) {
            uwb_survey_input  in;
            uwb_survey_result out;
            double            e;
            make_input(&in, 8, 0.12);
            add_dist_noise(&in, 8, 0.05, &rng);
            CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "ノイズ+δ で解けなかった");
            e = fabs((double)out.common_delay_m - 0.12);
            sum += e;
            if (e > worst) worst = e;
        }
        printf("    δ=0.12 m + 距離ノイズ5cm・8ノード x50回: 平均誤差 %.4f m / 最大 %.4f m\n",
               sum / 50.0, worst);
        CHECK(worst < 0.10, "ノイズ下の遅延推定誤差が大きすぎる: %.4f m", worst);
    }

    /* n=4 は m=6 < 3n-6+1=7 なので δ は識別不能。0 に固定して返すこと。 */
    {
        uwb_survey_input  in;
        uwb_survey_result out;
        make_input(&in, 4, 0.10);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "n=4 が解けなかった");
        CHECK((double)out.common_delay_m == 0.0,
              "n=4 では δ を 0 固定にするはずが %.6f", (double)out.common_delay_m);
    }
}

/* ==================================================================== *
 * 4. 欠測リンク
 * ==================================================================== */
static void scenario4_missing(void)
{
    uwb_survey_input  in;
    uwb_survey_result out;
    double            err;

    printf("--- 4. 欠測リンク ---\n");

    /* 1本欠け（6ノード15本 → 14本）。未知数 19 − ゲージ6 = 13 なので冗長度1。 */
    make_input(&in, 6, 0.08);
    drop_link(&in, 1, 4);
    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "1本欠けで解けなかった");
    err = max_pos_err(&out, 6);
    printf("    1本欠け(1-4): 座標誤差 %.3e m / δ誤差 %.3e / 反復 %d\n", err,
           fabs((double)out.common_delay_m - 0.08), out.iterations);
    CHECK(err < (double)TOL_EXACT, "1本欠けの復元誤差: %.3e m", err);
    CHECK(fabs((double)out.common_delay_m - 0.08) < (double)TOL_EXACT, "1本欠けでδがずれた");

    /* 2本欠け（13本）。未知数 19 − ゲージ6 = 13 なので **冗長度0**。
     *
     * このとき距離方程式系は「解が1つに決まる」保証を失う。実際この入力には
     * 残差ぴったり0 の別解が存在し、ソルバはそちらに落ちることがある
     * （下の表示を見ると残差 RMS は 1e-15 で、真値と同じくらい完璧に
     * データを説明している）。**これは実装の欠陥ではなく入力側の情報不足**で、
     * 仕様書 §5 の「5台でも解けるが冗長度0になる旨を警告」と同じ話。
     * したがってここで検証できるのは「データに矛盾しない解を返すこと」まで。 */
    make_input(&in, 6, 0.08);
    drop_link(&in, 1, 4);
    drop_link(&in, 0, 3);
    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "2本欠けで解けなかった");
    err = max_pos_err(&out, 6);
    printf("    2本欠け(1-4, 0-3)【冗長度0】: 残差RMS %.3e m / 座標誤差 %.4f m / "
           "δ %+.4f (真値 +0.08)\n",
           (double)out.residual_rms, err, (double)out.common_delay_m);
    CHECK((double)out.residual_rms < 1e-6, "2本欠けで距離データを説明できていない");

    /* 3本欠け（12本 = 3n-6）。δ は識別不能になり 0 固定へ落ちる。
     * これも冗長度0なので、検証できるのは「残差0の解を返すこと」まで。 */
    make_input(&in, 6, 0.0);
    drop_link(&in, 1, 4);
    drop_link(&in, 0, 3);
    drop_link(&in, 2, 5);
    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "3本欠けで解けなかった");
    err = max_pos_err(&out, 6);
    printf("    3本欠け【冗長度0・δ固定】: 残差RMS %.3e m / 座標誤差 %.4f m\n",
           (double)out.residual_rms, err);
    CHECK((double)out.residual_rms < 1e-6, "3本欠けで距離データを説明できていない");
    CHECK((double)out.common_delay_m == 0.0, "3本欠けでは δ を固定するはず");

    /* 8ノードなら 2本欠けても冗長度が十分（28→26本、未知数25−6=19）。 */
    make_input(&in, 8, 0.12);
    drop_link(&in, 1, 4);
    drop_link(&in, 0, 3);
    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "8ノード2本欠けで解けなかった");
    err = max_pos_err(&out, 8);
    printf("    8ノード2本欠け: 座標誤差 %.3e m / δ誤差 %.3e\n", err,
           fabs((double)out.common_delay_m - 0.12));
    CHECK(err < (double)TOL_EXACT, "8ノード2本欠けの復元誤差: %.3e m", err);
    CHECK(fabs((double)out.common_delay_m - 0.12) < (double)TOL_EXACT,
          "8ノード2本欠けでδがずれた");

    /* 8ノードなら 4本欠けても余裕。 */
    make_input(&in, 8, 0.15);
    drop_link(&in, 0, 5);
    drop_link(&in, 1, 6);
    drop_link(&in, 2, 7);
    drop_link(&in, 3, 4);
    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "8ノード4本欠けで解けなかった");
    err = max_pos_err(&out, 8);
    printf("    8ノード4本欠け: 座標誤差 %.3e m / δ誤差 %.3e\n", err,
           fabs((double)out.common_delay_m - 0.15));
    CHECK(err < (double)TOL_EXACT, "8ノード4本欠けの復元誤差: %.3e m", err);

    /* ノイズ入りでの欠測。 */
    {
        rng_t rng;
        rng_seed(&rng, 4242u);
        make_input(&in, 8, 0.10);
        drop_link(&in, 0, 5);
        drop_link(&in, 1, 6);
        add_dist_noise(&in, 8, 0.05, &rng);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "欠測+ノイズで解けなかった");
        printf("    8ノード2本欠け+5cmノイズ: 座標誤差 %.4f m / δ誤差 %.4f m\n",
               max_pos_err(&out, 8), fabs((double)out.common_delay_m - 0.10));
    }

    /* ノード0 のリンクを 3 本未満にすると解けない（位置が円周上に残る）。 */
    make_input(&in, 6, 0.0);
    drop_link(&in, 0, 1);
    drop_link(&in, 0, 2);
    drop_link(&in, 0, 3);
    drop_link(&in, 0, 4);
    CHECK(uwb_survey_solve(&in, &out) == 0 && out.ok == 0,
          "次数2のノードがあるのに解けてしまった");

    /* 非連結（{0,1,2,3} と {4,5} が切り離される）→ 解けない。 */
    make_input(&in, 6, 0.0);
    {
        int i, j;
        for (i = 0; i < 4; ++i)
            for (j = 4; j < 6; ++j) drop_link(&in, i, j);
    }
    CHECK(uwb_survey_solve(&in, &out) == 0 && out.ok == 0, "非連結なのに解けてしまった");
}

/* ==================================================================== *
 * 5. 縮退配置の検出
 * ==================================================================== */
static void set_flat_input(uwb_survey_input *in, int n, const double pos[][3])
{
    int i, j;
    uwb_survey_input_init(in, n);
    for (i = 0; i < n; ++i) {
        for (j = i + 1; j < n; ++j) {
            double dx = pos[i][0] - pos[j][0];
            double dy = pos[i][1] - pos[j][1];
            double dz = pos[i][2] - pos[j][2];
            uwb_real v = (uwb_real)sqrt(dx * dx + dy * dy + dz * dz);
            in->dist[i][j] = in->dist[j][i] = v;
            in->have[i][j] = in->have[j][i] = 1;
        }
        in->height[i]      = (uwb_real)pos[i][2];
        in->have_height[i] = 1;
    }
}

static void scenario5_degenerate(void)
{
    uwb_survey_input  in;
    uwb_survey_result out;

    printf("--- 5. 縮退配置の検出 ---\n");

    /* (a) 完全な同一平面（全ノードが z=1.5） */
    {
        static const double flat[6][3] = {{0.0, 0.0, 1.5}, {5.0, 0.0, 1.5}, {5.0, 4.0, 1.5},
                                          {0.0, 4.0, 1.5}, {2.5, 2.0, 1.5}, {1.0, 3.0, 1.5}};
        set_flat_input(&in, 6, flat);
        CHECK(uwb_survey_solve(&in, &out) == 0, "同一平面なのに成功を返した");
        CHECK(out.degenerate == 1, "同一平面で degenerate=1 にならなかった");
        CHECK(out.ok == 0, "degenerate なのに ok=1");
    }

    /* (b) 直線配置 */
    {
        static const double line[6][3] = {{0.0, 0.0, 1.0}, {1.0, 0.0, 1.0}, {2.3, 0.0, 1.0},
                                          {3.1, 0.0, 1.0}, {4.7, 0.0, 1.0}, {5.9, 0.0, 1.0}};
        set_flat_input(&in, 6, line);
        CHECK(uwb_survey_solve(&in, &out) == 0, "直線配置なのに成功を返した");
        CHECK(out.degenerate == 1, "直線配置で degenerate=1 にならなかった");
    }

    /* (c) 「ほぼ」同一平面（厚み ±2cm）。測距ばらつきより薄い third dimension は
     *     観測できないので、物理しきい値（5cm）で縮退と判定する。 */
    {
        static const double near_flat[6][3] = {{0.0, 0.0, 1.50}, {5.0, 0.0, 1.51},
                                               {5.0, 4.0, 1.49}, {0.0, 4.0, 1.52},
                                               {2.5, 2.0, 1.48}, {1.0, 3.0, 1.50}};
        set_flat_input(&in, 6, near_flat);
        CHECK(uwb_survey_solve(&in, &out) == 0, "ほぼ同一平面なのに成功を返した");
        CHECK(out.degenerate == 1, "厚み2cm で degenerate=1 にならなかった");
    }

    /* (d) 本物の3次元配置は縮退扱いしない（偽陽性が無いこと） */
    {
        make_input(&in, 6, 0.0);
        CHECK(uwb_survey_solve(&in, &out) == 1, "正常な配置が解けなかった");
        CHECK(out.degenerate == 0, "正常な3次元配置を縮退と誤判定した");
    }

    /* (e) 4ノードの正四面体相当（最小構成）も縮退にしない */
    {
        static const double tetra[4][3] = {
            {0.0, 0.0, 0.0}, {4.0, 0.0, 0.0}, {2.0, 3.5, 0.0}, {2.0, 1.2, 3.0}};
        set_flat_input(&in, 4, tetra);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.degenerate == 0,
              "四面体を縮退と誤判定した");
    }
}

/* ==================================================================== *
 * 6. 鏡像（キラリティ）
 *
 * 【重要な発見】高さの実測では鏡像は決まらない。
 * 反転した配置 S*p に対する高さ予測は u.(S p)+c = (S u).p+c で、S が直交
 * なので u -> S u は単位球面の全単射。つまり反転前後で「実現できる高さの
 * 組」は完全に一致し、高さ残差の最小値も一致する（必ず引き分け）。
 * 仕様書 §2[4]-1 の「小さい方を採用」は原理的に機能しない。
 *
 * 実装は規約（[5] 正規化後、|y| 最大のノードの y を正にする）で一意化する
 * ので、ここでは「どんな入力の並べ方でも同じ側に収束すること」を検証する。
 * ==================================================================== */
static void scenario6_mirror(void)
{
    uwb_survey_input  in;
    uwb_survey_result out, out2;
    int               i, k;

    printf("--- 6. 鏡像（キラリティ） ---\n");

    /* (a) 真値を y 反転した配置は、距離も高さも真値と完全に同一。
     *     → ソルバはどちらの入力からも「規約側」（＝真値）を返すはず。 */
    {
        double mirrored_diff = 0.0;
        make_input(&in, 6, 0.0);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "解けなかった");
        CHECK(max_pos_err(&out, 6) < (double)TOL_EXACT, "規約側（真値）に収束しなかった");
        for (i = 0; i < 6; ++i) {
            double e = fabs((double)out.pos[i][1] - (-TRUTH[i][1]));
            if (e > mirrored_diff) mirrored_diff = e;
        }
        CHECK(mirrored_diff > 1.0,
              "鏡像側と区別できていない（y の食い違いが %.3f m しかない）", mirrored_diff);
        CHECK(out.mirror_resolved == 0,
              "高さでは鏡像を決められないので mirror_resolved は 0 のはず");
    }

    /* (b) ノード2 と 3 を入れ替えて入力しても、対応する物理ノードの座標は
     *     同じ（＝内部の固有ベクトルの符号や初期配置の向きに依存しない）。 */
    {
        int perm[6] = {0, 1, 3, 2, 4, 5};   /* 入力添字 i には物理ノード perm[i] を置く */
        int a, b;
        double worst = 0.0;

        uwb_survey_input_init(&in, 6);
        for (a = 0; a < 6; ++a) {
            for (b = a + 1; b < 6; ++b) {
                uwb_real v = (uwb_real)truth_dist(perm[a], perm[b]);
                in.dist[a][b] = in.dist[b][a] = v;
                in.have[a][b] = in.have[b][a] = 1;
            }
            in.height[a]      = (uwb_real)TRUTH[perm[a]][2];
            in.have_height[a] = 1;
        }
        CHECK(uwb_survey_solve(&in, &out2) == 1 && out2.ok, "並べ替えた入力が解けなかった");
        for (a = 0; a < 6; ++a)
            for (k = 0; k < 3; ++k) {
                double e = fabs((double)out2.pos[a][k] - TRUTH[perm[a]][k]);
                if (e > worst) worst = e;
            }
        printf("    ノード2/3 を入れ替えた入力: 対応座標の最大誤差 %.3e m\n", worst);
        CHECK(worst < (double)TOL_EXACT,
              "並べ替えで別の鏡像に落ちた（誤差 %.3e m）", worst);
    }

    /* (c) ノイズが乗ってもキラリティは裏返らない。 */
    {
        rng_t rng;
        int   t, flipped = 0;
        rng_seed(&rng, 5150u);
        for (t = 0; t < 50; ++t) {
            make_input(&in, 6, 0.0);
            add_dist_noise(&in, 6, 0.05, &rng);
            if (!uwb_survey_solve(&in, &out) || !out.ok) continue;
            /* ノード3 の y は真値 +4.9。符号が逆なら鏡像側に落ちている。 */
            if ((double)out.pos[3][1] < 0.0) ++flipped;
        }
        CHECK(flipped == 0, "5cm ノイズでキラリティが %d/50 回裏返った", flipped);
    }
}

/* ==================================================================== *
 * 7. 高さ実測が不足しているとき（Lv1 相当へのフォールバック）
 * ==================================================================== */
static void scenario7_few_heights(void)
{
    uwb_survey_input  in;
    uwb_survey_result out;

    printf("--- 7. 高さ実測が3点未満のフォールバック ---\n");

    /* (a) 高さ2点。傾きは1自由度しか決まらないので座標は真値と一致しないが、
     *     「形」（ノード間距離）は完全に復元できていなければならない。
     *     実測した2点の高さはぴったり合う（残差0で当てられる）。 */
    {
        double se;
        make_input(&in, 6, 0.07);
        keep_heights(&in, 6, 2);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "高さ2点で解けなかった");
        se = max_shape_err(&out, 6);
        printf("    高さ2点: 形状誤差 %.3e m / δ誤差 %.3e / 座標誤差 %.3f m（座標系は未確定）\n",
               se, fabs((double)out.common_delay_m - 0.07), max_pos_err(&out, 6));
        CHECK(se < (double)TOL_EXACT, "高さ2点で形状が崩れた: %.3e m", se);
        CHECK(fabs((double)out.pos[0][2] - TRUTH[0][2]) < (double)TOL_EXACT,
              "実測した高さ[0] に合っていない (%.6f vs %.6f)", (double)out.pos[0][2],
              TRUTH[0][2]);
        CHECK(fabs((double)out.pos[1][2] - TRUTH[1][2]) < (double)TOL_EXACT,
              "実測した高さ[1] に合っていない");
        CHECK(out.mirror_resolved == 0, "高さ2点で mirror_resolved=1 はおかしい");
        check_convention(&out, "h2");
    }

    /* (b) 高さ1点。z 並進だけが決まる。 */
    {
        make_input(&in, 6, 0.0);
        keep_heights(&in, 6, 1);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "高さ1点で解けなかった");
        CHECK(max_shape_err(&out, 6) < (double)TOL_EXACT, "高さ1点で形状が崩れた");
        CHECK(fabs((double)out.pos[0][2] - TRUTH[0][2]) < (double)TOL_EXACT,
              "実測した高さ[0] に合っていない");
        check_convention(&out, "h1");
    }

    /* (c) 高さ0点。ゲージは XY 規約ぶんしか決まらないが、形は出る。 */
    {
        make_input(&in, 6, 0.0);
        keep_heights(&in, 6, 0);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "高さ0点で解けなかった");
        CHECK(max_shape_err(&out, 6) < (double)TOL_EXACT, "高さ0点で形状が崩れた");
        check_convention(&out, "h0");
    }

    /* (d) 高さ**ちょうど3点**。傾き2 + z並進1 の 3 自由度に対して式も 3 本
     *     なので、実測高さは残差0 でぴったり当たる。だが **鏡像は残る**。
     *
     *     3点を中心化すると 2 次元しか張らないので、上方向 u のうち
     *     「3点が作る平面の法線成分」は |u|=1 からその大きさだけが決まり、
     *     **符号は決まらない**。この符号は 3 次元の鏡像そのもので、
     *     どちらを選んでも 3 点の高さは同じ。しかも未実測ノードの z は
     *     符号によって変わるので、[5] の y 反転（z を変えない）では
     *     一致させられない。つまり **高さ3点では解が 2 つ残る**。
     *
     *     4点以上あれば u は一意に決まり、2 つの鏡像解の z は全ノードで
     *     一致するので、y 反転の規約だけで潰せる（(e) で確認）。 */
    {
        double e;
        make_input(&in, 6, 0.09);
        keep_heights(&in, 6, 3);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "高さ3点で解けなかった");
        e = max_pos_err(&out, 6);
        printf("    高さ3点: 形状誤差 %.3e m / 座標誤差 %.4f m（鏡像2解が残る）\n",
               max_shape_err(&out, 6), e);
        CHECK(max_shape_err(&out, 6) < (double)TOL_EXACT, "高さ3点で形状が崩れた");
        CHECK(fabs((double)out.common_delay_m - 0.09) < (double)TOL_EXACT,
              "高さ3点でδがずれた");
        for (int i = 0; i < 3; ++i)
            CHECK(fabs((double)out.pos[i][2] - TRUTH[i][2]) < (double)TOL_EXACT,
                  "実測した高さ[%d] に合っていない (%.6f vs %.6f)", i,
                  (double)out.pos[i][2], TRUTH[i][2]);
        check_convention(&out, "h3");
    }

    /* (e) 高さ4点あれば座標まで一意に決まる（鏡像は y 反転の規約で潰れる）。 */
    {
        double e;
        make_input(&in, 6, 0.09);
        keep_heights(&in, 6, 4);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "高さ4点で解けなかった");
        e = max_pos_err(&out, 6);
        printf("    高さ4点: 座標誤差 %.3e m（ここから一意）\n", e);
        CHECK(e < (double)TOL_EXACT, "高さ4点で座標が合わない: %.3e m", e);
    }
}

/* ==================================================================== *
 * 8. ノード数 4..8
 * ==================================================================== */
static void scenario8_node_counts(void)
{
    int n;

    printf("--- 8. ノード数 4〜8 ---\n");

    for (n = 4; n <= 8; ++n) {
        uwb_survey_input  in;
        uwb_survey_result out;
        /* n=4 は δ が識別不能（m=6 < 3n-6+1=7）なので δ=0 で試す。 */
        double delay = (n == 4) ? 0.0 : 0.11;
        double err;

        make_input(&in, n, delay);
        CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "n=%d が解けなかった", n);
        err = max_pos_err(&out, n);
        printf("    n=%d: 座標誤差 %.3e m / δ %+.9f (真値 %+.2f) / 反復 %d / 残差RMS %.2e\n",
               n, err, (double)out.common_delay_m, delay, out.iterations,
               (double)out.residual_rms);
        CHECK(err < (double)TOL_EXACT, "n=%d の復元誤差: %.3e m", n, err);
        CHECK(fabs((double)out.common_delay_m - delay) < (double)TOL_EXACT,
              "n=%d の δ 誤差", n);
        /* n を超える添字はゼロのまま */
        CHECK(out.pos[UWB_SURVEY_MAX_NODES - 1][0] == (uwb_real)0 || n == 8,
              "未使用の pos がゼロ初期化されていない");
    }

    /* 範囲外のノード数は弾く */
    {
        uwb_survey_input  in;
        uwb_survey_result out;
        make_input(&in, 6, 0.0);
        in.n = 3;
        CHECK(uwb_survey_solve(&in, &out) == 0 && out.ok == 0, "n=3 を受け付けてしまった");
        in.n = UWB_SURVEY_MAX_NODES + 1;
        CHECK(uwb_survey_solve(&in, &out) == 0 && out.ok == 0, "n が上限超でも受け付けた");
        CHECK(uwb_survey_solve(NULL, &out) == 0, "in=NULL を受け付けた");
        CHECK(uwb_survey_solve(&in, NULL) == 0, "out=NULL を受け付けた");
    }
}

/* ==================================================================== *
 * 9. 外れ値リンクの棄却
 * ==================================================================== */
static void scenario9_outlier(void)
{
    uwb_survey_input  in;
    uwb_survey_result out;
    unsigned long     bit;
    double            err;

    printf("--- 9. 外れ値リンクの棄却 ---\n");

    /* 8ノード（28本、冗長度9）で 1本だけ +2.0 m ずれた測距を混ぜる。 */
    make_input(&in, 8, 0.10);
    in.dist[2][5] += (uwb_real)2.0;
    in.dist[5][2] += (uwb_real)2.0;

    CHECK(uwb_survey_solve(&in, &out) == 1 && out.ok, "外れ値入りで解けなかった");
    bit = 1UL << uwb_survey_link_index(2, 5);
    CHECK((out.excluded & bit) != 0UL, "リンク(2,5)が棄却されなかった (excluded=0x%lX)",
          out.excluded);
    err = max_pos_err(&out, 8);
    printf("    +2.0m の外れ値1本: excluded=0x%lX / 座標誤差 %.3e m / δ誤差 %.3e\n",
           out.excluded, err, fabs((double)out.common_delay_m - 0.10));
    CHECK(err < (double)TOL_EXACT, "外れ値を除いた後の復元誤差: %.3e m", err);

    /* リンク添字のマッピングが一意で 32bit に収まること。 */
    {
        int i, j, seen[32];
        for (i = 0; i < 32; ++i) seen[i] = 0;
        for (j = 1; j < UWB_SURVEY_MAX_NODES; ++j)
            for (i = 0; i < j; ++i) {
                int b = uwb_survey_link_index(i, j);
                CHECK(b >= 0 && b < 32, "link_index(%d,%d)=%d が範囲外", i, j, b);
                CHECK(seen[b] == 0, "link_index が衝突 (%d,%d)", i, j);
                seen[b] = 1;
                CHECK(uwb_survey_link_index(j, i) == b, "link_index が対称でない");
            }
        CHECK(uwb_survey_link_index(3, 3) < 0, "link_index(i,i) が負を返さない");
        CHECK(uwb_survey_link_index(0, UWB_SURVEY_MAX_NODES) < 0,
              "link_index の範囲チェックが効いていない");
    }
}

/* ==================================================================== *
 * 10. 密行列ソルバ単体（S2）
 * ==================================================================== */
static void scenario10_dense(void)
{
    printf("--- 10. 測量専用の密行列ソルバ ---\n");

    /* (a) 3x3 の対称正定値をコレスキーで解く */
    {
        uwb_real a[9] = {4, 1, 1, 1, 3, 0, 1, 0, 2};
        uwb_real b[3] = {6, 4, 3};   /* 真値 x = (1,1,1) */
        uwb_real x[3];
        CHECK(uwb_survey_chol_solve(a, b, x, 3) == 1, "3x3 のコレスキーが失敗した");
        CHECK(fabs((double)x[0] - 1.0) < TOL_LIN && fabs((double)x[1] - 1.0) < TOL_LIN &&
                  fabs((double)x[2] - 1.0) < TOL_LIN,
              "3x3 の解が違う (%.9f, %.9f, %.9f)", (double)x[0], (double)x[1], (double)x[2]);
    }

    /* (b) 正定値でない行列は 0 を返す */
    {
        uwb_real a[4] = {1, 2, 2, 1};   /* 固有値 3, -1 */
        uwb_real b[2] = {1, 1};
        uwb_real x[2];
        CHECK(uwb_survey_chol_solve(a, b, x, 2) == 0, "非正定値を受け入れてしまった");
    }

    /* (c) 25x25（最大サイズ）。A = D + 1*1^T（対角優位で正定値）を解く。 */
    {
        static uwb_real a[UWB_SURVEY_MAX_UNKNOWNS * UWB_SURVEY_MAX_UNKNOWNS];
        static uwb_real b[UWB_SURVEY_MAX_UNKNOWNS];
        static uwb_real x[UWB_SURVEY_MAX_UNKNOWNS];
        const int n = UWB_SURVEY_MAX_UNKNOWNS;
        int i, j;
        double worst = 0.0;

        CHECK(n == 25, "未知数の上限が 25 でない (%d)", n);
        for (i = 0; i < n; ++i) {
            b[i] = (uwb_real)0;
            for (j = 0; j < n; ++j) {
                uwb_real v = (uwb_real)1 + ((i == j) ? (uwb_real)(i + 1) : (uwb_real)0);
                a[i * n + j] = v;
                b[i] += v;              /* 真値 x = (1,1,...,1) */
            }
        }
        CHECK(uwb_survey_chol_solve(a, b, x, n) == 1, "25x25 のコレスキーが失敗した");
        for (i = 0; i < n; ++i) {
            double e = fabs((double)x[i] - 1.0);
            if (e > worst) worst = e;
        }
        printf("    25x25 コレスキー: 最大誤差 %.3e\n", worst);
        CHECK(worst < TOL_LIN, "25x25 の解の誤差が大きい: %.3e", worst);
        CHECK(uwb_survey_chol_solve(a, b, x, UWB_SURVEY_MAX_UNKNOWNS + 1) == 0,
              "上限超の n を受け付けた");
    }

    /* (d) 固有分解: 固有値が降順で、A v = lambda v を満たすこと */
    {
        const int n = 4;
        uwb_real  src[16] = {4, 1, 0, 2, 1, 3, 1, 0, 0, 1, 5, 1, 2, 0, 1, 6};
        uwb_real  a[16], eig[4], vec[16];
        int       i, j, k;
        double    worst_res = 0.0, worst_orth = 0.0;

        memcpy(a, src, sizeof(a));
        CHECK(uwb_survey_sym_eig(a, eig, vec, n) == 1, "4x4 の固有分解が失敗した");
        for (k = 0; k + 1 < n; ++k)
            CHECK((double)eig[k] >= (double)eig[k + 1], "固有値が降順でない");

        for (k = 0; k < n; ++k)
            for (i = 0; i < n; ++i) {
                double s = 0.0;
                for (j = 0; j < n; ++j) s += (double)src[i * n + j] * (double)vec[j * n + k];
                s -= (double)eig[k] * (double)vec[i * n + k];
                if (fabs(s) > worst_res) worst_res = fabs(s);
            }
        for (i = 0; i < n; ++i)
            for (j = 0; j < n; ++j) {
                double s = 0.0;
                for (k = 0; k < n; ++k) s += (double)vec[k * n + i] * (double)vec[k * n + j];
                s -= (i == j) ? 1.0 : 0.0;
                if (fabs(s) > worst_orth) worst_orth = fabs(s);
            }
        printf("    4x4 固有分解: |Av-λv|max = %.3e / 直交性のずれ %.3e\n", worst_res,
               worst_orth);
        CHECK(worst_res < TOL_LIN, "固有ベクトルが合っていない: %.3e", worst_res);
        CHECK(worst_orth < TOL_ORTH, "固有ベクトルが直交していない: %.3e", worst_orth);

        /* トレース = 固有値の和（保存量の検算） */
        {
            double tr = 0.0, sum = 0.0;
            for (i = 0; i < n; ++i) { tr += (double)src[i * n + i]; sum += (double)eig[i]; }
            CHECK(fabs(tr - sum) < TOL_ORTH * (1.0 + fabs(tr)),
                  "トレースが合わない (%.12f vs %.12f)", tr, sum);
        }

        /* 同じ入力なら符号まで含めて必ず同じ結果（再現性）。 */
        {
            uwb_real a2[16], eig2[4], vec2[16];
            int      same = 1;
            memcpy(a2, src, sizeof(a2));
            CHECK(uwb_survey_sym_eig(a2, eig2, vec2, n) == 1, "2回目の固有分解が失敗");
            for (i = 0; i < n * n; ++i) if (vec2[i] != vec[i]) same = 0;
            CHECK(same == 1, "固有ベクトルが再現しない");
        }

        CHECK(uwb_survey_sym_eig(a, eig, vec, UWB_SURVEY_EIG_MAX + 1) == 0,
              "上限超の n を受け付けた");
    }
}

/* ==================================================================== */

int main(void)
{
    printf("=== tests/host/survey: uwb_survey アンカー自動測量 合成データ検証 ===\n");
    printf("UWB_SURVEY_MAX_NODES=%d UWB_SURVEY_MAX_LINKS=%d UWB_SURVEY_MAX_UNKNOWNS=%d "
           "UWB_REAL_IS_FLOAT=%d\n\n",
           UWB_SURVEY_MAX_NODES, UWB_SURVEY_MAX_LINKS, UWB_SURVEY_MAX_UNKNOWNS,
           UWB_REAL_IS_FLOAT);

    scenario1_exact();
    scenario2_noise();
    scenario3_delay();
    scenario4_missing();
    scenario5_degenerate();
    scenario6_mirror();
    scenario7_few_heights();
    scenario8_node_counts();
    scenario9_outlier();
    scenario10_dense();

    printf("\n=== %d 件中 %d 件失敗 ===\n", g_run, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
