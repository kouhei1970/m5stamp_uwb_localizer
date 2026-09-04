/* test_handedness.c — 反証テスト:
 *   「tag ファームウェアの測位ソルバ (components/uwb_loc の Lv0/Lv2 と
 *    EKF Lv3、および components/uwb_survey/src/uwb_survey_tape.c の
 *    メジャー測量ソルバ) は、与えられたアンカー座標と同じ右手系で位置を
 *    返す（鏡像を返さない）」という主張の反証を試みる。
 *
 * 方針 (実行前に失敗の定義を書く):
 *   - 3D: 非平面配置のアンカーで真値 p_true の測距を厳密計算し、Lv0/Lv2
 *     の解が p_true に一致し、(x,-y,z)・(-x,y,z)・(x,y,2*z_plane-z) の
 *     いずれの鏡像候補よりも近いことを確認する。5mm雑音を乗せた
 *     モンテカルロ (100 回) でも同様に確認する。
 *   - 2D固定高さ: 共面3アンカー・非対称三角形で (x,y) が反転していない
 *     ことを確認する。z_fixed の符号は座標系の規約であって鏡像バグでは
 *     ないことも合わせて確認する (水平位置は不変、z だけ指定値になる)。
 *   - EKF (Lv3): 1本ずつの逐次更新で真値に収束し、鏡像に収束しないこと
 *     を確認する。
 *   - テープ測量 (uwb_survey_tape): index2 が +y、index3 が -y という
 *     真の右手系配置で、d(2,3) ありなら index3 が正しく -y 側に出ること、
 *     d(2,3) 無しでは仕様通り +y に畳まれ sign_unresolved が立つこと
 *     (これはバグではなく仕様なので FAIL にしない) を確認する。
 *
 * このファイルは components/ 配下のソースを一切変更せず、公開 API
 * (uwb_loc.h, uwb_survey_tape.h) だけを呼ぶ。既存の test_uwb.c /
 * test_regress.c は変更しない。
 */
#include "uwb_loc.h"
#include "uwb_survey_tape.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_fail = 0;
static int g_run = 0;

#define CHECK(cond, ...)                                                      \
    do {                                                                      \
        ++g_run;                                                              \
        if (!(cond)) {                                                        \
            ++g_fail;                                                         \
            printf("  NG  %s:%d  ", __FILE__, __LINE__);                      \
            printf(__VA_ARGS__);                                              \
            printf("\n");                                                     \
        }                                                                     \
    } while (0)

#define D(x) ((double)(x))

static double dist3(const uwb_real *a, const uwb_real *b)
{
    double dx = D(a[0]) - D(b[0]);
    double dy = D(a[1]) - D(b[1]);
    double dz = D(a[2]) - D(b[2]);
    return sqrt(dx * dx + dy * dy + dz * dz);
}

static double dist3d(const double *a, const double *b)
{
    double dx = a[0] - b[0], dy = a[1] - b[1], dz = a[2] - b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* 決定的な擬似乱数 (test_uwb.c と同じ手法。ファイルを分けて再実装する
 * ことで既存テストへの依存を作らない)。 */
static unsigned long g_seed = 20260904UL;
static double urand(void)
{
    g_seed = g_seed * 1103515245UL + 12345UL;
    return (double)((g_seed >> 16) & 0x7fff) / 32767.0;
}
static double nrand(void)
{
    double u1 = urand(), u2 = urand();
    if (u1 < 1e-12) u1 = 1e-12;
    return sqrt(-2.0 * log(u1)) * cos(6.283185307179586 * u2);
}

static void print_pt_mm(const char *label, const double *p)
{
    printf("    %-16s (%9.2f, %9.2f, %9.2f) mm\n", label, p[0] * 1000.0,
           p[1] * 1000.0, p[2] * 1000.0);
}

/* ------------------------------------------------------------------ */
/* (a) 3D: 非平面アンカー配置。閉形式 (Lv0) とロバスト NLS (Lv2) の両方を
 *     無雑音・5mm雑音の両方で調べる。 */

static void run_3d_case(const char *name, const uwb_anchor *anch, int n,
                        const double *p_true, unsigned long seed)
{
    uwb_config cfg;
    uwb_meas m[8];
    uwb_fix fix0, fix2;
    double mirror[3][3];
    double z_plane;
    int i, k, coplanar;
    double d_mirror[3];

    printf("\n[3D] %s (アンカー %d 台)\n", name, n);

    uwb_config_init(&cfg, anch, n);

    coplanar = uwb_anchors_coplanar(&cfg, NULL, NULL);
    CHECK(!coplanar, "%s: 想定外に同一平面 (このケースは非平面のはず)", name);
    printf("    同一平面判定: %s (非平面が前提)\n", coplanar ? "coplanar" : "non-coplanar");

    /* 鏡像候補の z_plane はアンカーの平均高さ (アンカーは正確には同一平面
     * ではないので、これは幾何学的に厳密な対称面ではなく、あくまで
     * 「z の符号を反転するような実装バグ」を拾うための目安値)。 */
    z_plane = 0.0;
    for (i = 0; i < n; ++i) z_plane += D(anch[i].p[2]);
    z_plane /= (double)n;

    mirror[0][0] = p_true[0];  mirror[0][1] = -p_true[1]; mirror[0][2] = p_true[2];
    mirror[1][0] = -p_true[0]; mirror[1][1] = p_true[1];  mirror[1][2] = p_true[2];
    mirror[2][0] = p_true[0];  mirror[2][1] = p_true[1];  mirror[2][2] = 2.0 * z_plane - p_true[2];

    print_pt_mm("truth", p_true);
    print_pt_mm("mirror(x,-y,z)", mirror[0]);
    print_pt_mm("mirror(-x,y,z)", mirror[1]);
    print_pt_mm("mirror(x,y,2zp-z)", mirror[2]);

    for (i = 0; i < n; ++i) {
        uwb_real pt[3] = {(uwb_real)p_true[0], (uwb_real)p_true[1], (uwb_real)p_true[2]};
        m[i].anchor = i;
        m[i].value = (uwb_real)dist3(pt, anch[i].p);
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }

    /* --- 無雑音: Lv0 (閉形式) --- */
    {
        double pd[3];
        int ok = uwb_solve_lv0(&cfg, m, n, &fix0);
        CHECK(ok && fix0.ok, "%s: Lv0 が解けない", name);
        pd[0] = D(fix0.p[0]); pd[1] = D(fix0.p[1]); pd[2] = D(fix0.p[2]);
        print_pt_mm("Lv0 解", pd);
        double err = dist3d(pd, p_true);
        printf("    Lv0 誤差 %.4f mm\n", err * 1000.0);
        CHECK(err < 0.001, "%s: Lv0 誤差が1mmを超える (%.4f mm)", name, err * 1000.0);
        for (k = 0; k < 3; ++k) {
            d_mirror[k] = dist3d(pd, mirror[k]);
            CHECK(err < d_mirror[k], "%s: Lv0 解が鏡像候補%d に真値より近い (真値差%.4fmm 鏡像差%.4fmm)",
                  name, k, err * 1000.0, d_mirror[k] * 1000.0);
        }
    }

    /* --- 無雑音: Lv2 (ロバストNLS、屋内既定) --- */
    {
        double pd[3];
        int ok = uwb_solve_lv2(&cfg, m, n, &fix2);
        CHECK(ok && fix2.ok, "%s: Lv2 が解けない", name);
        pd[0] = D(fix2.p[0]); pd[1] = D(fix2.p[1]); pd[2] = D(fix2.p[2]);
        print_pt_mm("Lv2 解", pd);
        double err = dist3d(pd, p_true);
        printf("    Lv2 誤差 %.4f mm\n", err * 1000.0);
        CHECK(err < 0.001, "%s: Lv2 誤差が1mmを超える (%.4f mm)", name, err * 1000.0);
        for (k = 0; k < 3; ++k) {
            d_mirror[k] = dist3d(pd, mirror[k]);
            CHECK(err < d_mirror[k], "%s: Lv2 解が鏡像候補%d に真値より近い (真値差%.4fmm 鏡像差%.4fmm)",
                  name, k, err * 1000.0, d_mirror[k] * 1000.0);
        }
    }

    /* --- 5mm雑音 x100回: Lv2 --- */
    {
        int trials = 100, t;
        int fail_pos = 0, fail_mirror = 0, fail_solve = 0;
        double max_err = 0.0;

        g_seed = seed;
        for (t = 0; t < trials; ++t) {
            uwb_fix fix;
            double pd[3];
            for (i = 0; i < n; ++i) {
                uwb_real pt[3] = {(uwb_real)p_true[0], (uwb_real)p_true[1], (uwb_real)p_true[2]};
                m[i].anchor = i;
                m[i].value = (uwb_real)(dist3(pt, anch[i].p) + 0.005 * nrand());
                m[i].sigma = (uwb_real)0;
                m[i].quality = (uwb_real)-1;
            }
            if (!uwb_solve_lv2(&cfg, m, n, &fix) || !fix.ok) { ++fail_solve; continue; }
            pd[0] = D(fix.p[0]); pd[1] = D(fix.p[1]); pd[2] = D(fix.p[2]);
            {
                double err = dist3d(pd, p_true);
                if (err > max_err) max_err = err;
                if (err > 0.05) ++fail_pos;
                for (k = 0; k < 3; ++k) {
                    double dm = dist3d(pd, mirror[k]);
                    if (dm < err) ++fail_mirror;
                }
            }
        }
        printf("    5mm雑音 x%d回: 解けない %d件 / 5cm超 %d件 / 鏡像の方が近い %d件 / 最大誤差 %.1f mm\n",
               trials, fail_solve, fail_pos, fail_mirror, max_err * 1000.0);
        CHECK(fail_solve == 0, "%s: 雑音下で解けない試行がある (%d/%d)", name, fail_solve, trials);
        CHECK(fail_pos == 0, "%s: 雑音下で5cmを超える試行がある (%d/%d)", name, fail_pos, trials);
        CHECK(fail_mirror == 0, "%s: 雑音下で鏡像の方が近い試行がある (%d/%d)", name, fail_mirror, trials);
    }
}

/* ------------------------------------------------------------------ */
/* (b) 2D固定高さモード。共面3アンカー、非対称三角形。 */

static void test_2d_fixed_height(void)
{
    static uwb_anchor anch[3] = {
        {"C0", {0.0, 0.0, 1.9}, 1, 0.0, 0.08, 0.0},
        {"C1", {3.0, 0.0, 1.9}, 1, 0.0, 0.08, 0.0},
        {"C2", {1.0, 2.6, 1.9}, 1, 0.0, 0.08, 0.0}
    };
    uwb_config cfg;
    uwb_meas m[3];
    uwb_fix fix, fix_wrong_sign;
    double truth[3] = {1.2, 1.0, 0.9};
    double mirror_x[3], mirror_mx[3];
    int i, ok;

    printf("\n[2D固定高さ] 共面3アンカー (z=1.9m)、非対称三角形\n");
    print_pt_mm("truth", truth);

    uwb_config_init(&cfg, anch, 3);
    cfg.dim = 2;
    cfg.z_fixed = (uwb_real)0.9; /* アンカー面(z=1.9)より0.9m低い、が絶対座標としては z=0.9 */

    for (i = 0; i < 3; ++i) {
        uwb_real pt[3] = {(uwb_real)truth[0], (uwb_real)truth[1], (uwb_real)truth[2]};
        m[i].anchor = i;
        m[i].value = (uwb_real)dist3(pt, anch[i].p);
        m[i].sigma = (uwb_real)0;
        m[i].quality = (uwb_real)-1;
    }

    ok = uwb_solve_lv2(&cfg, m, 3, &fix);
    CHECK(ok && fix.ok, "2D固定高さ: Lv2 が解けない");
    {
        double pd[3] = {D(fix.p[0]), D(fix.p[1]), D(fix.p[2])};
        double err = dist3d(pd, truth);
        print_pt_mm("Lv2 解 (z_fixed=+0.9)", pd);
        printf("    誤差 %.4f mm\n", err * 1000.0);
        CHECK(err < 0.001, "2D固定高さ: 誤差が1mmを超える (%.4f mm)", err * 1000.0);
        CHECK(fabs(pd[2] - 0.9) < 1e-6, "2D固定高さ: z が z_fixed に固定されていない (%.6f)", pd[2]);

        mirror_x[0] = truth[0]; mirror_x[1] = -truth[1]; mirror_x[2] = truth[2];
        mirror_mx[0] = -truth[0]; mirror_mx[1] = truth[1]; mirror_mx[2] = truth[2];
        CHECK(err < dist3d(pd, mirror_x), "2D固定高さ: (x,-y) 鏡像の方が近い");
        CHECK(err < dist3d(pd, mirror_mx), "2D固定高さ: (-x,y) 鏡像の方が近い");
    }

    /* z_fixed の符号を反転 (アンカー平面をまたいだ鏡側) しても、
     * 測距値 m[] は変えていない (真値 z=0.9 のまま計算した値)。
     * 水平距離 h^2 = d^2-(z_fixed-z_anchor)^2 は符号によらないので、
     * (x,y) は不変・z だけ指定した値になるはず。
     * これは「鏡像バグ」ではなく「z_fixed は絶対高さなのでユーザが
     * 符号を間違えると違う高さの答えが返る」という運用上の注意点。 */
    cfg.z_fixed = (uwb_real)(2.0 * 1.9 - 0.9); /* = 2.9 (アンカー面に対して真値と対称の高さ) */
    ok = uwb_solve_lv2(&cfg, m, 3, &fix_wrong_sign);
    CHECK(ok && fix_wrong_sign.ok, "2D固定高さ: z_fixed符号反転で解けない");
    {
        double pd[3] = {D(fix_wrong_sign.p[0]), D(fix_wrong_sign.p[1]), D(fix_wrong_sign.p[2])};
        print_pt_mm("z_fixed=+2.9 解", pd);
        printf("    [仕様確認・バグではない] z_fixed の符号を反転しても x,y は不変、"
               "z は指定値 (2.9m) になるはず\n");
        CHECK(fabs(pd[0] - D(fix.p[0])) < 1e-6 && fabs(pd[1] - D(fix.p[1])) < 1e-6,
              "2D固定高さ: z_fixed符号反転でx,yまで変わった (実装がz_fixedの符号を特別扱いしている?)");
        CHECK(fabs(pd[2] - 2.9) < 1e-6, "2D固定高さ: z が指定した z_fixed(2.9) になっていない (%.6f)", pd[2]);
    }
}

/* ------------------------------------------------------------------ */
/* (c) EKF (Lv3)。1本ずつの逐次更新で真値に収束するか。 */

static void test_ekf_no_mirror(void)
{
    static uwb_anchor anch[4] = {
        {"A0", {0.0, 0.0, 0.86}, 1, 0.0, 0.08, 0.0},
        {"A1", {1.78, 0.0, 1.98}, 1, 0.0, 0.08, 0.0},
        {"A2", {1.42, -1.96, 1.89}, 1, 0.0, 0.08, 0.0},
        {"A3", {-0.46, -2.93, 2.07}, 1, 0.0, 0.08, 0.0}
    };
    uwb_config cfg;
    uwb_ekf ekf;
    uwb_fix fix;
    double truth[3] = {0.3, -1.2, 0.5};
    double mirror[3][3];
    double z_plane = 0.0;
    int i, k, n = 4, cycles = 50;

    printf("\n[EKF Lv3] 実配置4アンカー、1本ずつ逐次更新 x%d 回\n", cycles);
    print_pt_mm("truth", truth);

    for (i = 0; i < n; ++i) z_plane += D(anch[i].p[2]);
    z_plane /= (double)n;
    mirror[0][0] = truth[0];  mirror[0][1] = -truth[1]; mirror[0][2] = truth[2];
    mirror[1][0] = -truth[0]; mirror[1][1] = truth[1];  mirror[1][2] = truth[2];
    mirror[2][0] = truth[0];  mirror[2][1] = truth[1];  mirror[2][2] = 2.0 * z_plane - truth[2];

    uwb_config_init(&cfg, anch, n);
    uwb_ekf_init(&ekf, &cfg, UWB_MOTION_CV, (uwb_real)1.0);
    memset(&fix, 0, sizeof(fix));

    for (i = 0; i < cycles; ++i) {
        uwb_real t = (uwb_real)((i + 1) * 0.05);
        uwb_meas one;
        int a = i % n;
        uwb_real pt[3] = {(uwb_real)truth[0], (uwb_real)truth[1], (uwb_real)truth[2]};
        one.anchor = a;
        one.value = (uwb_real)dist3(pt, anch[a].p);
        one.sigma = (uwb_real)0;
        one.quality = (uwb_real)-1;
        uwb_ekf_update(&ekf, t, &one, 1, &fix);
    }

    CHECK(fix.ok, "EKF: %d サイクル後も ok=0 (収束していない)", cycles);
    {
        double pd[3] = {D(fix.p[0]), D(fix.p[1]), D(fix.p[2])};
        double err = dist3d(pd, truth);
        print_pt_mm("EKF 解", pd);
        printf("    誤差 %.2f mm (ambiguous=%d)\n", err * 1000.0, fix.ambiguous);
        CHECK(err < 0.05, "EKF: 収束後の誤差が5cmを超える (%.2f mm)", err * 1000.0);
        for (k = 0; k < 3; ++k) {
            double dm = dist3d(pd, mirror[k]);
            CHECK(err < dm, "EKF: 鏡像候補%d の方が真値より近い (真値差%.2fmm 鏡像差%.2fmm)",
                  k, err * 1000.0, dm * 1000.0);
        }
    }
}

/* ------------------------------------------------------------------ */
/* (d) テープ測量 (uwb_survey_tape)。index2=+y, index3=-y の真の右手系配置。 */

static void set_d(uwb_survey_tape_input *in, int i, int j, double d)
{
    in->dist[i][j] = (uwb_real)d;
    in->have[i][j] = 1;
}

static void test_tape_survey_handedness(void)
{
    /* 真値: index0を原点、index1を+X軸上に置いた「規約どおり」の座標系
     * (テープ測量ソルバはこの基準線から自分で座標系を作るので、真値も
     * 同じ基準線を使えば出力とそのまま比較できる)。 */
    double T[4][3] = {
        {0.0, 0.0, 2.0},
        {5.0, 0.0, 2.2},
        {2.0, 3.0, 1.8},   /* index2: +y 側 (規約) */
        {2.0, -3.0, 2.5}   /* index3: -y 側 (規約と逆。d(2,3)で見分けが要る) */
    };
    uwb_survey_tape_input in;
    uwb_survey_tape_result out;
    int i;

    printf("\n[テープ測量] index2=+y / index3=-y の真値配置\n");
    for (i = 0; i < 4; ++i) {
        char label[16];
        snprintf(label, sizeof(label), "truth[%d]", i);
        print_pt_mm(label, T[i]);
    }

    /* --- d(2,3) あり: 符号が一意に決まるはず --- */
    uwb_survey_tape_input_init(&in, 4);
    for (i = 0; i < 4; ++i) in.z[i] = (uwb_real)T[i][2];
    set_d(&in, 0, 1, dist3d(T[0], T[1]));
    set_d(&in, 0, 2, dist3d(T[0], T[2]));
    set_d(&in, 1, 2, dist3d(T[1], T[2]));
    set_d(&in, 0, 3, dist3d(T[0], T[3]));
    set_d(&in, 1, 3, dist3d(T[1], T[3]));
    set_d(&in, 2, 3, dist3d(T[2], T[3]));

    memset(&out, 0, sizeof(out));
    CHECK(uwb_survey_tape_solve(&in, &out) && out.ok,
          "テープ測量: d(2,3)ありで解けない (status=%d)", (int)out.status);
    for (i = 0; i < 4; ++i) {
        double pd[3] = {D(out.pos[i][0]), D(out.pos[i][1]), D(out.pos[i][2])};
        char label[16];
        double err;
        snprintf(label, sizeof(label), "pos[%d]", i);
        print_pt_mm(label, pd);
        err = dist3d(pd, T[i]);
        CHECK(err < 0.001, "テープ測量: index%d の誤差が1mmを超える (%.4f mm)", i, err * 1000.0);
    }
    CHECK(D(out.pos[3][1]) < 0.0, "テープ測量: d(2,3)ありなのに index3 が -y 側に出ていない (y=%.1f mm)",
          D(out.pos[3][1]) * 1000.0);
    CHECK(out.sign_unresolved == 0UL, "テープ測量: d(2,3)ありなのに sign_unresolved が立っている (0x%lx)",
          out.sign_unresolved);

    /* --- d(2,3) 無し: 仕様どおり +y に畳まれ、sign_unresolved が立つはず
     *     (docs/SURVEY_SPEC.md および uwb_survey_tape.h の規約どおり。
     *      これはソルバのバグではなく「材料が無ければ+y既定に倒す」という
     *      明記された仕様なので FAIL 扱いにしない)。 */
    {
        uwb_survey_tape_input in2 = in;
        uwb_survey_tape_result out2;
        in2.have[2][3] = 0;
        in2.have[3][2] = 0;
        memset(&out2, 0, sizeof(out2));
        CHECK(uwb_survey_tape_solve(&in2, &out2) && out2.ok,
              "テープ測量: d(2,3)無しで解けない (status=%d)", (int)out2.status);
        {
            double pd[3] = {D(out2.pos[3][0]), D(out2.pos[3][1]), D(out2.pos[3][2])};
            print_pt_mm("d(2,3)無し pos[3]", pd);
            printf("    [仕様確認・バグではない] d(2,3)が無いので index3 は規約どおり "
                   "+y側 (y=%.1f mm、真値は %.1f mm) に畳まれ、sign_unresolved が立つはず\n",
                   pd[1] * 1000.0, T[3][1] * 1000.0);
        }
        CHECK(D(out2.pos[3][1]) > 0.0,
              "テープ測量(d23無し): 規約(+y既定)通りに畳まれていない (y=%.1f mm)",
              D(out2.pos[3][1]) * 1000.0);
        CHECK((out2.sign_unresolved & (1UL << 3)) != 0UL,
              "テープ測量(d23無し): sign_unresolved(index3) が立っていない (0x%lx)",
              out2.sign_unresolved);
    }
}

/* ------------------------------------------------------------------ */

int main(void)
{
    double p_true_a[3] = {0.3, -1.2, 0.5};
    static uwb_anchor real_layout[4] = {
        {"A0", {0.0, 0.0, 0.86}, 1, 0.0, 0.08, 0.0},
        {"A1", {1.78, 0.0, 1.98}, 1, 0.0, 0.08, 0.0},
        {"A2", {1.42, -1.96, 1.89}, 1, 0.0, 0.08, 0.0},
        {"A3", {-0.46, -2.93, 2.07}, 1, 0.0, 0.08, 0.0}
    };
    double p_true_b[3] = {1.0, 2.0, 0.8};
    static uwb_anchor generic5[5] = {
        {"B0", {0.0, 0.0, 0.0}, 1, 0.0, 0.08, 0.0},
        {"B1", {4.0, 0.0, 1.0}, 1, 0.0, 0.08, 0.0},
        {"B2", {4.0, 3.0, 2.0}, 1, 0.0, 0.08, 0.0},
        {"B3", {0.0, 3.0, 1.5}, 1, 0.0, 0.08, 0.0},
        {"B4", {2.0, 1.5, 3.0}, 1, 0.0, 0.08, 0.0}
    };

    printf("uwb_loc / uwb_survey_tape 右手系(鏡像なし)反証テスト (%s)\n",
           UWB_REAL_IS_FLOAT ? "float" : "double");
    printf("============================================================\n");

    run_3d_case("実配置4アンカー (A0-A3)", real_layout, 4, p_true_a, 1001UL);
    run_3d_case("汎用5アンカー (B0-B4)", generic5, 5, p_true_b, 2002UL);
    test_2d_fixed_height();
    test_ekf_no_mirror();
    test_tape_survey_handedness();

    printf("\n============================================================\n");
    if (g_fail == 0) {
        printf("OK  %d 件すべて通った (鏡像は検出されなかった)\n", g_run);
        return 0;
    }
    printf("NG  %d / %d 件が失敗\n", g_fail, g_run);
    return 1;
}
