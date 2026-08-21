/* uwb_loc C 版のマイクロベンチマーク。
 *
 * 目的は最適化の効果を数値で示すこと。構造を使った書き換え (Joseph 形式の
 * O(nx^2) 化、EKF predict の Kronecker 構造利用、Beck 二分法のコレスキー
 * 使い回しなど) をする前に、**今の実装がどれだけ時間を食っているか**を
 * ここで記録しておき、最適化の前後で diff できるようにする。
 *
 *   make bench
 *
 * 計測対象を個別に分けて 10 万回ずつ回し、1 回あたりの平均時間を出す
 * (1 回だけでは clock_gettime の分解能以下で測れないため)。
 *
 * 出力は 2 つ:
 *   - 人間可読の表 (stdout)
 *   - 同じ内容の TSV (bench_result.tsv。最適化前後で diff しやすい)
 *
 * 注意:
 *   - -O2 でビルドする (Makefile 側で固定。実運用と揃える)
 *   - 結果を volatile なアキュムレータ g_sink に足し込み、
 *     コンパイラがループごと消してしまわないようにしてある
 *   - アンカーは意図的に**同一平面にしていない** (同一平面だと Beck 法が
 *     早期に失敗して測定にならない。詳細はリポジトリ外だが
 *     m5stack_uwb/docs/ANCHOR_PLACEMENT.md を参照)
 *   - このファイルはホスト (macOS/Linux) 専用。malloc は使っていないが、
 *     clock_gettime と stdio はライブラリ本体には持ち込まない
 *     (c/src 以下はこのファイルに依存しない)
 */
/* clock_gettime/CLOCK_MONOTONIC (POSIX) と snprintf (C99) を両方見せる。
 * macOS は _POSIX_C_SOURCE を定義すると _DARWIN_C_SOURCE 無しに
 * snprintf まで隠してしまうので、両方定義しておく (Linux では
 * _DARWIN_C_SOURCE は単に無視される)。 */
#define _POSIX_C_SOURCE 199309L
#define _DARWIN_C_SOURCE

#include "uwb_loc.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ------------------------------------------------------------ 計測基盤 */

#define BENCH_REPS 100000L

/* 最適化でループごと消されないための捨て場。毎回足し込む。 */
static volatile double g_sink = 0.0;

static double now_sec(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

typedef struct {
    const char *name; /* 関数名 */
    char        cond[32]; /* 条件 (アンカー台数・モデルなど) */
    long        reps;
    double      total_s;
} bench_row;

#define MAX_ROWS 64
static bench_row g_rows[MAX_ROWS];
static int       g_nrows = 0;

static void record(const char *name, const char *cond, long reps, double total_s)
{
    if (g_nrows >= MAX_ROWS) return;
    g_rows[g_nrows].name = name;
    memset(g_rows[g_nrows].cond, 0, sizeof g_rows[g_nrows].cond);
    strncpy(g_rows[g_nrows].cond, cond, sizeof g_rows[g_nrows].cond - 1);
    g_rows[g_nrows].reps = reps;
    g_rows[g_nrows].total_s = total_s;
    ++g_nrows;
}

/* ------------------------------------------------------------ アンカー配置 */

/* 8 台。crossval.py の "room" 配置と同じ座標で、z を 0.3/2.4 で互い違いに
 * してある。先頭 N 台を取っても (N=4,5,6,8) 同一平面にならない —
 * 起動時に uwb_anchors_coplanar() でも確かめる (check_not_coplanar)。
 * 同一平面だと Beck 法が早期に失敗し、計測にならない。 */
static const uwb_anchor ANCHORS[8] = {
    {"A0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A1", {7.8, 0.2, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A3", {0.2, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A4", {4.0, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"A5", {4.0, 5.8, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A6", {0.2, 3.0, 0.3}, 1, 0.0, 0.08, 0.0},
    {"A7", {7.8, 3.0, 2.4}, 1, 0.0, 0.08, 0.0},
};

static const uwb_real TRUTH[3] = {(uwb_real)4.0, (uwb_real)3.0, (uwb_real)1.2};
static const int ANCHOR_COUNTS[] = {4, 5, 6, 8};
#define N_COUNTS ((int)(sizeof(ANCHOR_COUNTS) / sizeof(ANCHOR_COUNTS[0])))

static double dist3(const uwb_real *a, const uwb_real *b)
{
    double dx = (double)a[0] - (double)b[0];
    double dy = (double)a[1] - (double)b[1];
    double dz = (double)a[2] - (double)b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* 測距値には小さな摂動を乗せる (ゼロ残差だと退化した特別扱いになりかねない
 * ため)。ただし **乱数は使わない**: 乱数だと構成 (N) によって chi2 ゲートの
 * 再解や Gauss-Newton の反復回数がたまたま跳ね上がることがあり、
 * 台数依存を見たいベンチにノイズが乗ってしまう (実際に 0.08 m 相当の
 * 正規乱数で試したところ、Lv2 の反復回数が N=4 で 24 回、N=8 で 2 回と
 * 大きく暴れ、1 回あたりの時間が台数と無関係に 4 倍前後ぶれた)。
 * 振幅 1 cm の決定的な摂動なら chi2 ゲート (既定で概ね 0.3 m 相当) には
 * 全く届かず、GN も 1-2 反復で収束が安定するので、台数依存だけが
 * 素直に見える。 */
static void make_meas(int n, uwb_meas *m, double wobble)
{
    int i;
    for (i = 0; i < n; ++i) {
        double d = dist3(TRUTH, ANCHORS[i].p) + wobble * sin((double)(i + 1) * 1.37);
        m[i].anchor = i;
        m[i].value = (uwb_real)d;
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }
}

static void check_not_coplanar(const uwb_config *cfg, int n)
{
    if (uwb_anchors_coplanar(cfg, NULL, NULL))
        fprintf(stderr, "警告: アンカー %d 台が同一平面になっている。計測が無意味かもしれない\n", n);
}

/* ------------------------------------------------------------ Lv0/Lv1/Lv2 */

static void bench_solve(const char *name, int level, int n)
{
    uwb_config cfg;
    uwb_meas   meas[8];
    uwb_fix    fix;
    char       cond[32];
    double     t0, t1;
    long       i;
    int        ok = 0;

    uwb_config_init(&cfg, ANCHORS, n);
    check_not_coplanar(&cfg, n);
    make_meas(n, meas, 0.01);

    /* 解けているかは計測の外で 1 回だけ確かめる */
    switch (level) {
    case 0: ok = uwb_solve_lv0(&cfg, meas, n, &fix) && fix.ok; break;
    case 1: ok = uwb_solve_lv1(&cfg, meas, n, &fix) && fix.ok; break;
    case 2: ok = uwb_solve_lv2(&cfg, meas, n, &fix) && fix.ok; break;
    default: break;
    }
    if (!ok)
        fprintf(stderr, "警告: %s (N=%d) が解けていない。計測値が無意味かもしれない\n", name, n);

    t0 = now_sec();
    for (i = 0; i < BENCH_REPS; ++i) {
        switch (level) {
        case 0: uwb_solve_lv0(&cfg, meas, n, &fix); break;
        case 1: uwb_solve_lv1(&cfg, meas, n, &fix); break;
        case 2: uwb_solve_lv2(&cfg, meas, n, &fix); break;
        default: break;
        }
        g_sink += (double)fix.p[0] + (double)fix.ok;
    }
    t1 = now_sec();

    snprintf(cond, sizeof cond, "N=%d", n);
    record(name, cond, BENCH_REPS, t1 - t0);
}

/* ------------------------------------------------------------------ Beck */

static void bench_beck(int n)
{
    uwb_config cfg;
    uwb_meas   meas[8];
    uwb_real   out[3];
    char       cond[32];
    double     t0, t1;
    long       i;

    uwb_config_init(&cfg, ANCHORS, n);
    check_not_coplanar(&cfg, n);
    make_meas(n, meas, 0.01);

    if (!uwb_beck_gtrs(&cfg, meas, n, out))
        fprintf(stderr, "警告: uwb_beck_gtrs (N=%d) が解けていない\n", n);

    t0 = now_sec();
    for (i = 0; i < BENCH_REPS; ++i) {
        uwb_beck_gtrs(&cfg, meas, n, out);
        g_sink += (double)out[0];
    }
    t1 = now_sec();

    snprintf(cond, sizeof cond, "N=%d", n);
    record("uwb_beck_gtrs", cond, BENCH_REPS, t1 - t0);
}

/* -------------------------------------------------------------- EKF predict */

static void bench_ekf_predict(uwb_motion motion, const char *label)
{
    uwb_config cfg;
    uwb_ekf    ekf;
    double     t0, t1;
    long       i;
    const uwb_real dt = (uwb_real)0.05;

    /* predict はアンカーを見ない (F/Q は dt とモデルだけで決まる)。
     * cfg は dim=3 であれば何でもよいので ANCHORS をそのまま使う。 */
    uwb_config_init(&cfg, ANCHORS, 8);
    uwb_ekf_init(&ekf, &cfg, motion, (uwb_real)1.0);

    t0 = now_sec();
    for (i = 0; i < BENCH_REPS; ++i) {
        uwb_ekf_predict(&ekf, dt);
        g_sink += (double)ekf.x[0];
    }
    t1 = now_sec();

    record("uwb_ekf_predict", label, BENCH_REPS, t1 - t0);
}

/* --------------------------------------------------------------- EKF update */

/* uwb_ekf_update() の「立ち上げ後、1 本ずつ (または N 本まとめて) 来る
 * 測距で更新する」経路だけを計りたい。bootstrap() (Lv2 スナップショットを
 * 使う立ち上げ) を計測に混ぜないよう、状態を直接「立ち上がった後」の形に
 * 揃える。uwb_ekf の各フィールドはヘッダで公開されている
 * (呼び出し側が構造体を持つ設計なので malloc なしで直接触れる)。 */
static void force_ready(uwb_ekf *e, const uwb_real *p)
{
    int i, j, nx = e->nx, nd = e->nd;

    for (i = 0; i < UWB_MAX_STATE; ++i) e->x[i] = (uwb_real)0;
    for (i = 0; i < UWB_MAX_STATE * UWB_MAX_STATE; ++i) e->P[i] = (uwb_real)0;
    for (i = 0; i < nd; ++i) e->x[i] = p[i];
    for (i = 0; i < nx; ++i)
        for (j = 0; j < nx; ++j)
            e->P[i * nx + j] = (i == j) ? (i < nd ? (uwb_real)1.0 : (uwb_real)100.0)
                                         : (uwb_real)0;
    e->t = (uwb_real)0;
    e->has_t = 1;
    e->initialized = 1;
    e->rejects = 0;
    e->n_pending = 0;
    e->ambiguous = 0;
    e->side_known = 0;
    e->side = 0;
}

static void bench_ekf_update(uwb_motion motion, const char *mlabel, int n)
{
    uwb_config cfg;
    uwb_ekf    ekf;
    uwb_meas   meas[8];
    uwb_fix    fix;
    char       cond[32];
    double     t0, t1;
    long       i;

    uwb_config_init(&cfg, ANCHORS, n);
    check_not_coplanar(&cfg, n);
    make_meas(n, meas, 0.01);

    uwb_ekf_init(&ekf, &cfg, motion, (uwb_real)1.0);
    force_ready(&ekf, TRUTH);

    /* 毎回同じ時刻 (dt=0) で呼ぶ。predict は dt<=0 で即 return するので、
     * ここで計っているのはほぼ update (Joseph 形式) 本体だけになる。 */
    t0 = now_sec();
    for (i = 0; i < BENCH_REPS; ++i) {
        uwb_ekf_update(&ekf, (uwb_real)0, meas, n, &fix);
        g_sink += (double)fix.p[0];
    }
    t1 = now_sec();

    snprintf(cond, sizeof cond, "%s, N=%d", mlabel, n);
    record("uwb_ekf_update", cond, BENCH_REPS, t1 - t0);
}

/* ------------------------------------------------------------------ 出力 */

/* 相対比は「同じ関数名の中で最初に出てきた行 (=最小のアンカー台数、
 * predict なら CV)」を基準にする。台数依存・モデル依存がそのまま見える。 */
static void report(FILE *tsv)
{
    const char *last_name = "";
    double      baseline = 1.0;
    int         i;

    printf("%-18s %-16s %10s %12s %8s\n",
           "関数", "条件", "回数", "us/回", "相対比");
    printf("--------------------------------------------------------------------\n");
    if (tsv) fprintf(tsv, "function\tcondition\treps\tus_per_call\tratio\n");

    for (i = 0; i < g_nrows; ++i) {
        double us = g_rows[i].total_s / (double)g_rows[i].reps * 1e6;
        double ratio;
        if (strcmp(g_rows[i].name, last_name) != 0) {
            baseline = us;
            last_name = g_rows[i].name;
        }
        ratio = us / baseline;
        printf("%-18s %-16s %10ld %12.4f %7.2fx\n",
               g_rows[i].name, g_rows[i].cond, g_rows[i].reps, us, ratio);
        if (tsv)
            fprintf(tsv, "%s\t%s\t%ld\t%.6f\t%.4f\n",
                    g_rows[i].name, g_rows[i].cond, g_rows[i].reps, us, ratio);
    }
    printf("--------------------------------------------------------------------\n");
}

int main(void)
{
    int  i;
    FILE *tsv;

    printf("uwb_loc ベンチマーク (%s, 1関数あたり %ld 回)\n",
           UWB_REAL_IS_FLOAT ? "float" : "double", (long)BENCH_REPS);
    printf("======================================================================\n");

    for (i = 0; i < N_COUNTS; ++i) bench_solve("uwb_solve_lv0", 0, ANCHOR_COUNTS[i]);
    for (i = 0; i < N_COUNTS; ++i) bench_solve("uwb_solve_lv1", 1, ANCHOR_COUNTS[i]);
    for (i = 0; i < N_COUNTS; ++i) bench_solve("uwb_solve_lv2", 2, ANCHOR_COUNTS[i]);
    for (i = 0; i < N_COUNTS; ++i) bench_beck(ANCHOR_COUNTS[i]);

    bench_ekf_predict(UWB_MOTION_CV, "CV(nx=6)");
    bench_ekf_predict(UWB_MOTION_CA, "CA(nx=9)");

    for (i = 0; i < N_COUNTS; ++i) bench_ekf_update(UWB_MOTION_CV, "CV(nx=6)", ANCHOR_COUNTS[i]);
    for (i = 0; i < N_COUNTS; ++i) bench_ekf_update(UWB_MOTION_CA, "CA(nx=9)", ANCHOR_COUNTS[i]);

    tsv = fopen("bench_result.tsv", "w");
    if (!tsv) fprintf(stderr, "警告: bench_result.tsv を書けなかった (カレントディレクトリを確認)\n");

    report(tsv);

    if (tsv) {
        fclose(tsv);
        printf("\nTSV: bench_result.tsv\n");
    }
    printf("(g_sink = %.6g — 最適化で消えていないことの確認用)\n", g_sink);
    return 0;
}
