/**
 * @file test_survey_tape.c
 * @brief components/uwb_survey/src/uwb_survey_tape.c
 *        （メジャー実測値からのアンカー座標 閉形式計算）のホスト側検証。
 *
 * test_survey.c と同じ CHECK() マクロの流儀（PASS/FAIL をその場で数え、
 * 失敗時だけ内容を表示する）。閉形式（反復なし）なので、ノイズを乗せた
 * 統計的な検証は行わない — 既知配置から作った距離が丸め誤差レベルで
 * 元の座標に戻ることと、異常入力が仕様どおりのエラーになることを
 * 確認すれば十分（uwb_survey_tape.h 冒頭コメント参照）。
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "uwb_survey_tape.h"

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

#if UWB_REAL_IS_FLOAT
#define TOL ((uwb_real)2e-3) /* 単精度は桁落ちでcm級までしか詰められない */
#else
#define TOL ((uwb_real)1e-9)
#endif

/* ==================================================================== *
 * 既知配置（8ノード）。
 *   - index2 は規約どおり +y（真値そのものが正規形になるよう選んである）
 *   - index3, index6 は -y（d(2,k) の有無で符号解決が変わることの確認用）
 *   - 高さはバラバラ（高さ差からの水平距離補正の確認用）
 * ==================================================================== */
static const double T[8][3] = {
    {0.00,  0.00, 0.30},  /* 0: 基準（原点） */
    {5.00,  0.00, 2.40},  /* 1: 基準（+X軸） */
    {4.20,  4.60, 0.35},  /* 2: +y 側（規約） */
    {0.40, -4.90, 2.35},  /* 3: -y 側 */
    {2.50,  2.20, 2.45},
    {-1.20, 1.50, 0.90},
    {4.80, -2.50, 1.80},  /* 6: -y 側 */
    {1.00,  3.60, 0.15},
};

static double dist3(int i, int j)
{
    double dx = T[i][0] - T[j][0], dy = T[i][1] - T[j][1], dz = T[i][2] - T[j][2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/** 必須の距離（d(0,1), d(0,k), d(1,k)）と高さを詰める。with_d2k なら
 *  d(2,k) (k>=3) も詰める（符号解決に使わせる）。 */
static void make_input(uwb_survey_tape_input *in, int n, int with_d2k)
{
    int i, j;
    uwb_survey_tape_input_init(in, n);
    for (i = 0; i < n; ++i) in->z[i] = (uwb_real)T[i][2];

    in->dist[0][1] = (uwb_real)dist3(0, 1);
    in->have[0][1] = 1;
    for (j = 2; j < n; ++j) {
        in->dist[0][j] = (uwb_real)dist3(0, j);
        in->have[0][j] = 1;
        in->dist[1][j] = (uwb_real)dist3(1, j);
        in->have[1][j] = 1;
    }
    if (with_d2k) {
        for (j = 3; j < n; ++j) {
            in->dist[2][j] = (uwb_real)dist3(2, j);
            in->have[2][j] = 1;
        }
    }
}

static double max_pos_err(const uwb_survey_tape_result *o, int n)
{
    double worst = 0.0;
    int i, k;
    for (i = 0; i < n; ++i)
        for (k = 0; k < 3; ++k) {
            double e = fabs((double)o->pos[i][k] - T[i][k]);
            if (e > worst) worst = e;
        }
    return worst;
}

/* ==================================================================== *
 * 1. 往復テスト（d(2,k) あり）: n=3..8、真値と一致すること
 * ==================================================================== */
static void scenario1_roundtrip(void)
{
    int n;
    printf("--- 1. 往復テスト（d(2,k) あり、真値と一致） ---\n");
    for (n = UWB_SURVEY_TAPE_MIN_NODES; n <= UWB_SURVEY_TAPE_MAX_NODES; ++n) {
        uwb_survey_tape_input  in;
        uwb_survey_tape_result out;
        double err;

        make_input(&in, n, 1);
        CHECK(uwb_survey_tape_solve(&in, &out) == 1, "n=%d: 解けなかった", n);
        CHECK(out.ok == 1, "n=%d: ok=0 (status=%d)", n, (int)out.status);
        CHECK(out.missing == 0UL, "n=%d: missing=0x%lX", n, out.missing);
        CHECK(out.clamped_pairs == 0UL, "n=%d: clamped_pairs=0x%lX", n, out.clamped_pairs);
        CHECK(out.clamped_nodes == 0UL, "n=%d: clamped_nodes=0x%lX", n, out.clamped_nodes);
        CHECK(out.sign_unresolved == 0UL, "n=%d: sign_unresolved=0x%lX（d(2,k)を入れたのに）", n,
              out.sign_unresolved);

        err = max_pos_err(&out, n);
        CHECK(err < (double)TOL, "n=%d: 復元誤差が大きい: %.3e m", n, err);
    }
    printf("    最大 n=%d まで復元誤差 < %.1e m\n", UWB_SURVEY_TAPE_MAX_NODES, (double)TOL);
}

/* ==================================================================== *
 * 2. 符号解決（index k>=3 の y の符号）
 * ==================================================================== */
static void scenario2_sign(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;
    const int n = 7; /* index3, index6 が -y */

    printf("--- 2. 符号解決（d(2,k) の有無） ---\n");

    /* d(2,k) あり → 正しい符号（-y）に決まる */
    make_input(&in, n, 1);
    CHECK(uwb_survey_tape_solve(&in, &out) == 1, "解けなかった");
    CHECK(out.sign_unresolved == 0UL, "d(2,k)ありなのに sign_unresolved=0x%lX", out.sign_unresolved);
    CHECK(out.pos[3][1] < (uwb_real)0, "index3 が -y に決まらなかった: y=%.4f", (double)out.pos[3][1]);
    CHECK(out.pos[6][1] < (uwb_real)0, "index6 が -y に決まらなかった: y=%.4f", (double)out.pos[6][1]);
    CHECK(max_pos_err(&out, n) < (double)TOL, "d(2,k)ありなのに復元誤差が大きい: %.3e",
          max_pos_err(&out, n));

    /* d(2,k) なし → 規約(+y)のまま、未確定フラグが立つ */
    make_input(&in, n, 0);
    CHECK(uwb_survey_tape_solve(&in, &out) == 1, "解けなかった");
    CHECK(out.sign_unresolved == ((1UL << 3) | (1UL << 4) | (1UL << 5) | (1UL << 6)),
          "sign_unresolved が期待値と違う: 0x%lX", out.sign_unresolved);
    CHECK(out.pos[3][1] > (uwb_real)0, "d(2,k)無しなのに index3 が +y に倒れていない: y=%.4f",
          (double)out.pos[3][1]);
    CHECK(out.pos[6][1] > (uwb_real)0, "d(2,k)無しなのに index6 が +y に倒れていない: y=%.4f",
          (double)out.pos[6][1]);
    /* index4, index5 は真値がもともと +y なので、規約に倒れても座標は合う。 */
    CHECK(fabs((double)out.pos[4][1] - T[4][1]) < (double)TOL, "index4 の y が真値と違う");
    CHECK(fabs((double)out.pos[5][1] - T[5][1]) < (double)TOL, "index5 の y が真値と違う");
    /* index3, index6 は符号が逆になっているはず（x, z は合う）。 */
    CHECK(fabs((double)out.pos[3][0] - T[3][0]) < (double)TOL, "index3 の x が真値と違う");
    CHECK(fabs((double)out.pos[3][1] + T[3][1]) < (double)TOL, "index3 の y が「反転」になっていない");
    CHECK(fabs((double)out.pos[3][2] - T[3][2]) < (double)TOL, "index3 の z が真値と違う");
}

/* ==================================================================== *
 * 3. 高さ補正（斜距離 → 水平距離）の直接検算
 * ==================================================================== */
static void scenario3_height_correction(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;

    printf("--- 3. 高さ補正の直接検算（3-4-5 の直角三角形） ---\n");

    /* index0=(0,0,0), index1 は水平3m・高さ4mなので斜距離5m → h01=3。
     * index2 は index0 から水平4m・同じ高さ(0) → h02=4、index1 からは
     * 水平方向の位置関係から距離を計算: index1=(3,0), index2=(x2,y2)。
     * ここでは index2 を index0 から見て垂直（+X軸に直交、y軸方向）
     * 4mの位置、つまり (0,4,0) に置く。d(0,2)=4（斜距離=水平距離、
     * 高さ差0）、d(1,2)=sqrt(3^2+4^2)=5（水平5m、高さ差0）。 */
    uwb_survey_tape_input_init(&in, 3);
    in.z[0] = 0.0; in.z[1] = 4.0; in.z[2] = 0.0;
    in.dist[0][1] = 5.0; in.have[0][1] = 1; /* 水平3m + 高さ4m → 斜距離5m */
    in.dist[0][2] = 4.0; in.have[0][2] = 1; /* 高さ差0なので水平距離そのもの */
    in.dist[1][2] = 5.0; in.have[1][2] = 1; /* 水平5m（3-4-5三角形）、高さ差4m
                                              * → 斜距離 sqrt(5^2+4^2) のはず
                                              * だが下でそちらを使う */
    /* 上のd(1,2)は高さ差を考慮していない値なので、正しい斜距離に差し替える。
     * index1=(3,0,4), index2=(0,4,0) の直線距離: dx=3,dy=-4,dz=4
     *   -> sqrt(9+16+16) = sqrt(41) */
    in.dist[1][2] = (uwb_real)sqrt(9.0 + 16.0 + 16.0);

    CHECK(uwb_survey_tape_solve(&in, &out) == 1, "解けなかった");
    CHECK(out.ok == 1, "ok=0 (status=%d)", (int)out.status);
    CHECK(fabs((double)out.pos[0][0]) < (double)TOL && fabs((double)out.pos[0][1]) < (double)TOL,
          "index0 が原点でない: (%.4f, %.4f)", (double)out.pos[0][0], (double)out.pos[0][1]);
    CHECK(fabs((double)out.pos[1][0] - 3.0) < (double)TOL, "h01 が 3m にならない: %.4f",
          (double)out.pos[1][0]);
    CHECK(fabs((double)out.pos[1][1]) < (double)TOL, "index1 が y=0 でない: %.4f",
          (double)out.pos[1][1]);
    CHECK(fabs((double)out.pos[2][0]) < (double)TOL, "index2 の x が 0 にならない: %.4f",
          (double)out.pos[2][0]);
    CHECK(fabs((double)out.pos[2][1] - 4.0) < (double)TOL, "index2 の y が 4m にならない: %.4f",
          (double)out.pos[2][1]);
    printf("    d(0,1)=5m(水平3m+高さ4m) → 復元 x1=%.4f / d(0,2)=4m(高さ差0) → y2=%.4f\n",
           (double)out.pos[1][0], (double)out.pos[2][1]);
}

/* ==================================================================== *
 * 4. 異常系: 必須の距離が未入力
 * ==================================================================== */
static void scenario4_missing(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;
    unsigned long expect;

    printf("--- 4. 異常系: 必須の距離が未入力 ---\n");

    /* n=4、d(1,3) だけ抜く。他は全部入れる。 */
    make_input(&in, 4, 1);
    in.have[1][3] = 0;
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "欠けているのに解けてしまった");
    CHECK(out.ok == 0, "ok=1になってしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_MISSING, "status が ERR_MISSING でない: %d",
          (int)out.status);
    expect = 1UL << (unsigned)uwb_survey_link_index(1, 3);
    CHECK(out.missing == expect, "missing が期待値と違う: 0x%lX (期待 0x%lX)", out.missing, expect);

    /* 複数欠けている場合、全部集めて返すこと（一問一答にしない）。 */
    make_input(&in, 5, 1);
    in.have[0][1] = 0; /* 基準線 */
    in.have[0][3] = 0;
    in.have[1][4] = 0;
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "欠けているのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_MISSING, "status が ERR_MISSING でない");
    expect = (1UL << (unsigned)uwb_survey_link_index(0, 1)) |
             (1UL << (unsigned)uwb_survey_link_index(0, 3)) |
             (1UL << (unsigned)uwb_survey_link_index(1, 4));
    CHECK(out.missing == expect, "missing が3本まとめて集まっていない: 0x%lX (期待 0x%lX)",
          out.missing, expect);
}

/* ==================================================================== *
 * 5. 異常系: 基準線が定義できない（d(0,1) の水平距離が0）
 * ==================================================================== */
static void scenario5_baseline(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;

    printf("--- 5. 異常系: 基準線が定義できない ---\n");

    /* index0, index1 を真上/真下に置く（高さ差だけで水平距離が無い）。
     * d(0,1) を高さ差ちょうどにすると根号が厳密に0（丸め不要）。 */
    uwb_survey_tape_input_init(&in, 3);
    in.z[0] = 0.0; in.z[1] = 3.0; in.z[2] = 0.0;
    in.dist[0][1] = 3.0; in.have[0][1] = 1; /* 水平距離0 */
    in.dist[0][2] = 4.0; in.have[0][2] = 1;
    in.dist[1][2] = 5.0; in.have[1][2] = 1;

    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "水平距離0なのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_BASELINE, "status が ERR_BASELINE でない: %d",
          (int)out.status);
    CHECK(out.err_i == 0 && out.err_j == 1, "err_i/err_j が (0,1) でない: (%d,%d)", out.err_i,
          out.err_j);
}

/* ==================================================================== *
 * 6. 異常系: 三角不等式違反（測定同士が矛盾）
 * ==================================================================== */
static void scenario6_inconsistent(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;

    printf("--- 6. 異常系: 三角不等式違反・根号が大きく負 ---\n");

    /* (a) (0,1,2) の三角形が成立しない: d01=10 に対し d02=d12=1 では
     *     index2 がどこにも届かない。 */
    uwb_survey_tape_input_init(&in, 3);
    in.dist[0][1] = 10.0; in.have[0][1] = 1;
    in.dist[0][2] = 1.0;  in.have[0][2] = 1;
    in.dist[1][2] = 1.0;  in.have[1][2] = 1;
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "三角不等式違反なのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_INCONSISTENT, "status が ERR_INCONSISTENT でない: %d",
          (int)out.status);
    CHECK(out.err_i == -1 && out.err_j == 2, "err_i/err_j が (-1,2) でない: (%d,%d)", out.err_i,
          out.err_j);

    /* (b) k>=3 でも同様に検出できること。 */
    uwb_survey_tape_input_init(&in, 4);
    in.dist[0][1] = 10.0; in.have[0][1] = 1;
    in.dist[0][2] = 6.0;  in.have[0][2] = 1;
    in.dist[1][2] = 6.0;  in.have[1][2] = 1;
    in.dist[0][3] = 1.0;  in.have[0][3] = 1;
    in.dist[1][3] = 1.0;  in.have[1][3] = 1;
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "三角不等式違反(k=3)なのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_INCONSISTENT, "status が ERR_INCONSISTENT でない: %d",
          (int)out.status);
    CHECK(out.err_i == -1 && out.err_j == 3, "err_i/err_j が (-1,3) でない: (%d,%d)", out.err_i,
          out.err_j);

    /* (c) 斜距離が高さ差より短い（d(0,k) の根号が大きく負）: ERR_INCONSISTENT
     *     かつ err がペア (0,2) を指すこと（基準線ではない普通のペア）。 */
    uwb_survey_tape_input_init(&in, 3);
    in.z[0] = 0.0; in.z[2] = 10.0;
    in.dist[0][1] = 5.0; in.have[0][1] = 1;
    in.dist[0][2] = 1.0; in.have[0][2] = 1; /* 高さ差10mよりずっと短い */
    in.dist[1][2] = 5.0; in.have[1][2] = 1;
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "高さ矛盾なのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_INCONSISTENT, "status が ERR_INCONSISTENT でない: %d",
          (int)out.status);
    CHECK(out.err_i == 0 && out.err_j == 2, "err_i/err_j が (0,2) でない: (%d,%d)", out.err_i,
          out.err_j);
}

/* ==================================================================== *
 * 7. 丸め: 根号がわずかに負（1cm² 以内）は0に丸めて続行する
 * ==================================================================== */
static void scenario7_clamp(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;
    unsigned long pair02;

    printf("--- 7. 丸め: 根号がわずかに負（測定誤差として0扱い） ---\n");

    /* 基準線 (0,1) は正常（h01=5、高さ差0）。node2 は「node0 の真上
     * （水平距離0）」に置きたいが、d(0,2) をわずかに短く測ってしまい
     * d(0,2)^2 - (z0-z2)^2 = -0.00005（1cm^2=1e-4 以内）になったとする。
     * d(1,2) は正確な値（h12=h01=5 になるように選んだ sqrt(34)）。
     * この場合、水平距離が0に丸められて node2 は (0,0,z2) に解ける
     * （node0 のちょうど真上）。 */
    uwb_survey_tape_input_init(&in, 3);
    in.z[0] = 0.0; in.z[1] = 0.0; in.z[2] = 3.0;
    in.dist[0][1] = 5.0; in.have[0][1] = 1;
    in.dist[1][2] = (uwb_real)sqrt(34.0); in.have[1][2] = 1;
    in.dist[0][2] = (uwb_real)sqrt(9.0 - 0.00005); in.have[0][2] = 1;

    CHECK(uwb_survey_tape_solve(&in, &out) == 1, "丸めで済むはずが解けなかった (status=%d)",
          (int)out.status);
    CHECK(out.ok == 1, "ok=0");
    pair02 = 1UL << (unsigned)uwb_survey_link_index(0, 2);
    CHECK((out.clamped_pairs & pair02) != 0UL, "clamped_pairs に (0,2) のビットが立っていない: 0x%lX",
          out.clamped_pairs);
    /* x がほぼ0になる際の丸め誤差で y^2 側もわずかに負に触れ、
     * clamped_nodes(node2) が立つことがある。両方とも「1cm^2以内の
     * 測定誤差として丸めた」という意味では一貫しているので、ここでは
     * 立っていても失敗にはしない（座標が合っていることの方を見る）。 */
    CHECK(fabs((double)out.pos[2][0]) < (double)TOL && fabs((double)out.pos[2][1]) < (double)TOL &&
              fabs((double)out.pos[2][2] - 3.0) < (double)TOL,
          "node2 が node0 の真上 (0,0,3) に解けなかった: (%.6f, %.6f, %.6f)", (double)out.pos[2][0],
          (double)out.pos[2][1], (double)out.pos[2][2]);
    printf("    d(0,2)^2 - 高さ差^2 = -5e-5 → 0 に丸めて node2=(0,0,3) / ok=%d\n", out.ok);
}

/* ==================================================================== *
 * 8. n の範囲外 / NULL
 * ==================================================================== */
static void scenario8_range_and_null(void)
{
    uwb_survey_tape_input  in;
    uwb_survey_tape_result out;

    printf("--- 8. n の範囲外・NULL 引数 ---\n");

    uwb_survey_tape_input_init(&in, UWB_SURVEY_TAPE_MIN_NODES - 1);
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "n が小さすぎるのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_N_RANGE, "status が ERR_N_RANGE でない: %d",
          (int)out.status);

    uwb_survey_tape_input_init(&in, UWB_SURVEY_TAPE_MAX_NODES + 1);
    CHECK(uwb_survey_tape_solve(&in, &out) == 0, "n が大きすぎるのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_N_RANGE, "status が ERR_N_RANGE でない: %d",
          (int)out.status);

    CHECK(uwb_survey_tape_solve(NULL, &out) == 0, "in=NULL なのに解けてしまった");
    CHECK(out.status == UWB_SURVEY_TAPE_ERR_N_RANGE, "in=NULL のとき status が ERR_N_RANGE でない: %d",
          (int)out.status);

    CHECK(uwb_survey_tape_solve(&in, NULL) == 0, "out=NULL なのに1を返した");
}

/* ==================================================================== *
 * main
 * ==================================================================== */
int main(void)
{
    scenario1_roundtrip();
    scenario2_sign();
    scenario3_height_correction();
    scenario4_missing();
    scenario5_baseline();
    scenario6_inconsistent();
    scenario7_clamp();
    scenario8_range_and_null();

    printf("\n=== %d 件中 %d 件失敗 ===\n", g_run, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
