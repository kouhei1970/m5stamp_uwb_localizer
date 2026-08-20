/**
 * @file main.c
 * @brief Phase 4 Step 1 動作確認: 合成データで uwb_loc（測位ソルバ）が
 *        ESP32-S3 上で動くことを見る。実機UWBハードウェア不要（UWBモジュール
 *        不要、ボード選択も不要）。components/uwb_loc の Lv0（三辺測量）/
 *        Lv2（Beck+Huber）/Lv3（EKF）に、既知アンカー配置と既知タグ座標から
 *        計算した理論距離を測距値として与え、真値との誤差・GDOP・実行時間・
 *        スタック使用量を表示する。
 *
 * 確認項目（docs/PLAN.md Phase 4 Step 1 のタスク定義に対応）:
 *   4.1 非同一平面のアンカー4台配置
 *   4.2 既知タグ座標から理論距離を作り測距値として与える
 *   4.3 Lv0/Lv2 で解いて真値との誤差がほぼゼロ（数値誤差レベル）になること
 *   4.4 ノイズを乗せた場合、1台に大きな外れ値を入れた場合
 *       （5台中1台を外れ値にしたとき Lv2 が弾いて解が残ること）
 *   4.5 uwb_anchors_coplanar() で同一平面配置（全アンカー z=0）が検出できること
 *   4.6 uwb_gdop_at() の値を表示
 *   4.7 各ソルバの実行時間を esp_timer_get_time() で計測して表示
 *   4.8 スタック使用量の目安
 *
 * 上記に加えて、実機到着後の最適化判断(UWB_LOC_USE_FLOAT の効果見積もり)の
 * 土台として、bench.c に基本演算(float/double の加減乗除・積和・sqrt)と
 * 9x9行列(EKFの共分散P相当)へのメモリアクセスパターンのサイクル計測を
 * 実装してある(bench_run()、詳細は bench.h/bench.c 参照)。こちらは
 * esp_timer の us 分解能では拾えない1桁台のサイクル差を見るために
 * esp_cpu_get_cycle_count() を使う、より低レベルの計測。
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bench.h"
#include "uwb_loc.h"

static const char *TAG = "uwb_soltest";

/* 合成データの計算自体は軽いが、Lv2/EKF の呼び出し連鎖はスタックを
 * それなりに使う（third_party/uwb_localizer/c/README.md 実測: double・
 * UWB_MAX_ANCHORS=16/32構成で最大6.6KB。UWB_MAX_ANCHORS=8構成ならもっと
 * 小さいはずだが、実測して 4.8 で報告するために専用タスクへ切り出し、
 * ESP-IDF既定のメインタスク(3584B)より余裕を持たせる）。 */
#define SOLTEST_TASK_STACK_BYTES 8192
#define SOLTEST_TASK_PRIORITY    5

#define TIMING_ITERATIONS 50

/* 円周率。libm の math.h が M_PI を定義するか処理系依存なので自前で持つ。 */
#define SOLTEST_PI 3.14159265358979323846

/* ---- 4.1 アンカー配置（非同一平面） ----
 * 5m x 5m x 2.4m 程度の部屋を想定。高さをばらしてあるので3次元で解ける
 * （全部同じ高さに置くと鏡像解の候補が消せなくなる。4.5 のシナリオで確認）。 */
static const uwb_anchor ANCHORS5[5] = {
    /* id       x    y    z          有効 遅延  sigma0 sigma/m */
    {"A0", {0.0, 0.0, 2.4}, 1, 0.0, 0.05, 0.0},
    {"A1", {5.0, 0.0, 0.2}, 1, 0.0, 0.05, 0.0},
    {"A2", {5.0, 5.0, 2.4}, 1, 0.0, 0.05, 0.0},
    {"A3", {0.0, 5.0, 0.2}, 1, 0.0, 0.05, 0.0},
    {"A4", {2.5, 2.5, 2.4}, 1, 0.0, 0.05, 0.0},
};
#define N_ANCHORS5 ((int)(sizeof(ANCHORS5) / sizeof(ANCHORS5[0])))

/* 同一平面配置（4.5 用）: 全アンカーを z=0 に置く */
static const uwb_anchor ANCHORS_FLAT[4] = {
    {"F0", {0.0, 0.0, 0.0}, 1, 0.0, 0.05, 0.0},
    {"F1", {5.0, 0.0, 0.0}, 1, 0.0, 0.05, 0.0},
    {"F2", {5.0, 5.0, 0.0}, 1, 0.0, 0.05, 0.0},
    {"F3", {0.0, 5.0, 0.0}, 1, 0.0, 0.05, 0.0},
};
#define N_ANCHORS_FLAT ((int)(sizeof(ANCHORS_FLAT) / sizeof(ANCHORS_FLAT[0])))

/* 同一平面配置その2: 天井取り付け (z=2.4、原点を通らない平面) を想定。
 * ANCHORS_FLAT (z=0) との対比用。理由は scenario_coplanar() のコメント参照。 */
static const uwb_anchor ANCHORS_CEIL[4] = {
    {"C0", {0.2, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"C1", {7.8, 0.2, 2.4}, 1, 0.0, 0.08, 0.0},
    {"C2", {7.8, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
    {"C3", {0.2, 5.8, 2.4}, 1, 0.0, 0.08, 0.0},
};
#define N_ANCHORS_CEIL ((int)(sizeof(ANCHORS_CEIL) / sizeof(ANCHORS_CEIL[0])))

/* 真値のタグ座標 [m]（4.2）*/
static const uwb_real TAG_TRUE[3] = {2.0, 3.0, 1.2};

/* ============================================================ ヘルパ */

static double dist3(const uwb_real *a, const uwb_real *b)
{
    double dx = (double)a[0] - (double)b[0];
    double dy = (double)a[1] - (double)b[1];
    double dz = (double)a[2] - (double)b[2];
    return sqrt(dx * dx + dy * dy + dz * dz);
}

/* [0,1) の一様乱数から標準正規乱数を作る（Box-Muller）。 rand() は
 * soltest_task の先頭で固定シードにしてあるので毎回同じ結果になる。 */
static double gauss_noise(void)
{
    double u1 = ((double)rand() + 1.0) / ((double)RAND_MAX + 1.0);
    double u2 = (double)rand() / (double)RAND_MAX;
    return sqrt(-2.0 * log(u1)) * cos(2.0 * SOLTEST_PI * u2);
}

/* 理論距離（+ノイズ、+バイアス）から meas[] を作る（4.2）。
 * bias_m が非NULLなら各アンカーの距離に加算する（外れ値作り用、4.4b）。 */
static void build_measurements(const uwb_anchor *anchors, int n, const uwb_real *tag,
                                const double *bias_m, double noise_sigma_m, uwb_meas *meas)
{
    for (int i = 0; i < n; ++i) {
        double d = dist3(tag, anchors[i].p);
        if (bias_m != NULL) {
            d += bias_m[i];
        }
        if (noise_sigma_m > 0.0) {
            d += noise_sigma_m * gauss_noise();
        }
        meas[i].anchor  = i;
        meas[i].value   = (uwb_real)d;
        meas[i].sigma   = 0;  /* 0 ならアンカー設定 (sigma0/sigma_per_m) から作る */
        meas[i].quality = -1; /* 不明 */
    }
}

static void print_fix(const char *label, const uwb_fix *fix)
{
    if (!fix->ok) {
        ESP_LOGE(TAG, "%-18s ok=0 (測位失敗) n_used=%d/%d", label, fix->n_used, fix->n_total);
        return;
    }
    double e = dist3(fix->p, TAG_TRUE);
    ESP_LOGI(TAG,
             "%-18s p=(%.4f, %.4f, %.4f) err=%.6e m  n_used=%d/%d  gdop=%.3f  "
             "residual_rms=%.4f  sigma=%.4f  excluded=0x%08lX",
             label, (double)fix->p[0], (double)fix->p[1], (double)fix->p[2], e, fix->n_used,
             fix->n_total, (double)fix->gdop, (double)fix->residual_rms, (double)fix->sigma,
             (unsigned long)fix->excluded);
}

/* ============================================================ 4.3 無雑音 */

static void scenario_exact(const uwb_config *cfg, const uwb_meas *meas, int n)
{
    ESP_LOGI(TAG, "--- 4.3 無雑音: 真値との誤差が数値誤差レベルになること ---");

    uwb_fix fix;
    uwb_solve_lv0(cfg, meas, n, &fix);
    print_fix("Lv0 (無雑音)", &fix);

    uwb_solve_lv2(cfg, meas, n, &fix);
    print_fix("Lv2 (無雑音)", &fix);
}

/* ============================================================ 4.4a ノイズ */

static void scenario_noisy(const uwb_config *cfg, int n)
{
    ESP_LOGI(TAG, "--- 4.4a ノイズ入り測距 (sigma=0.05m 相当) ---");

    uwb_meas meas[N_ANCHORS5];
    build_measurements(ANCHORS5, n, TAG_TRUE, NULL, 0.05, meas);

    uwb_fix fix;
    uwb_solve_lv0(cfg, meas, n, &fix);
    print_fix("Lv0 (ノイズ)", &fix);

    uwb_solve_lv2(cfg, meas, n, &fix);
    print_fix("Lv2 (ノイズ)", &fix);
}

/* ============================================================ 4.4b 外れ値 */

static void scenario_outlier(const uwb_config *cfg, int n)
{
    ESP_LOGI(TAG, "--- 4.4b 外れ値混入 (5台中1台に+3.0mのNLOSバイアス) ---");

    double bias[N_ANCHORS5];
    for (int i = 0; i < N_ANCHORS5; ++i) {
        bias[i] = 0.0;
    }
    bias[2] = 3.0; /* A2 (添字2) を強制的に外れ値にする */

    uwb_meas meas[N_ANCHORS5];
    build_measurements(ANCHORS5, n, TAG_TRUE, bias, 0.02, meas);

    uwb_fix fix;
    uwb_solve_lv0(cfg, meas, n, &fix);
    print_fix("Lv0 (外れ値混入)", &fix);
    ESP_LOGW(TAG, "  ^ Lv0 には外れ値を弾く機構が無いので誤差が大きく出るのが正常");

    uwb_solve_lv2(cfg, meas, n, &fix);
    print_fix("Lv2 (外れ値混入)", &fix);
    bool a2_excluded = (fix.excluded & (1UL << 2)) != 0;
    ESP_LOGI(TAG, "  ^ Lv2 が A2 (添字2) を弾いたか: %s / 解は残ったか(ok): %s",
             a2_excluded ? "YES" : "no", fix.ok ? "YES" : "no");
}

/* ============================================================ 4.5 同一平面 */

/**
 * 4.5 同一平面配置の検出。まず ANCHORS_FLAT (全アンカー z=0) で
 * uwb_anchors_coplanar() が検出できることを見る。
 *
 * 続けて Lv0/Lv2 で実際に解かせると、**z=0 配置では Lv2 (Lv1 も同様) が
 * ok=0 を返す**という、事前調査で見つけた実コードの挙動を確認する
 * （third_party は無改造なので、これは仕様として受け止めて回避策を示す）:
 *
 *   - Lv0 (uwb_lls_trilateration) はリッジ正則化つき最小二乗で、z 方向に
 *     情報が無いとき z=0 寄りの最小ノルム解を返す
 *   - Beck 法 (Lv2 の初期値作りに使う) は「アンカーが同一平面 → 行列が
 *     厳密に特異」という理由で意図的に失敗を返し、Lv0 にフォールバックする
 *     （uwb_closed_form.c のコメント参照）。ここまでは正常
 *   - ところが Lv0 の初期値がちょうど z=0、かつアンカー平面もちょうど z=0
 *     だと、初期値とアンカーの z が完全一致するため、そこでの距離の
 *     z 方向ヤコビアン (dh/dz) が全観測で厳密に 0 になる。Gauss-Newton は
 *     この点から一歩も動けず、収束後の共分散行列 (J^T W J) が厳密に特異の
 *     ままになり、共分散計算 (uwb_inverse) が失敗して ok=0 になる
 *
 * ANCHORS_CEIL (z=2.4、平面が原点を通らない) では同じ現象が起きず、
 * Lv2 は ok=1・ambiguous=1 で解ける（third_party/uwb_localizer/c/tests/
 * test_uwb.c の test_coplanar_is_detected もこちらの構成でテストしている）。
 *
 * 実運用上の教訓: **同一平面配置を uwb_anchors_coplanar() で検出したら、
 * Lv2 の ok を過信せず、dim=2 (z_fixed) に倒すか、Lv0 の結果を使うか、
 * 少なくともワールド座標の原点をアンカー平面からずらしておく**こと。
 */
static void scenario_coplanar(void)
{
    ESP_LOGI(TAG, "--- 4.5 同一平面配置の検出 ---");

    /* --- ケースA: 全アンカー z=0 (床置き想定) --- */
    uwb_config cfg_flat;
    uwb_config_init(&cfg_flat, ANCHORS_FLAT, N_ANCHORS_FLAT);

    uwb_real normal[3] = {0, 0, 0};
    uwb_real offset     = 0;
    int coplanar = uwb_anchors_coplanar(&cfg_flat, normal, &offset);
    ESP_LOGI(TAG,
             "[z=0] uwb_anchors_coplanar() = %d (1=同一平面)  normal=(%.3f,%.3f,%.3f) offset=%.3f",
             coplanar, (double)normal[0], (double)normal[1], (double)normal[2], (double)offset);

    uwb_real tag_flat[3] = {2.0, 3.0, 1.2};
    uwb_meas meas_flat[N_ANCHORS_FLAT];
    build_measurements(ANCHORS_FLAT, N_ANCHORS_FLAT, tag_flat, NULL, 0.0, meas_flat);

    uwb_fix fix;
    uwb_solve_lv0(&cfg_flat, meas_flat, N_ANCHORS_FLAT, &fix);
    ESP_LOGI(TAG, "[z=0] Lv0: ok=%d ambiguous=%d p=(%.4f,%.4f,%.4f)", fix.ok, fix.ambiguous,
             (double)fix.p[0], (double)fix.p[1], (double)fix.p[2]);

    uwb_solve_lv2(&cfg_flat, meas_flat, N_ANCHORS_FLAT, &fix);
    ESP_LOGW(TAG, "[z=0] Lv2: ok=%d (既知の注意点: 平面がちょうど z=0 だと GN が動けず ok=0 に "
                  "なりやすい。関数コメント参照)",
             fix.ok);

    /* --- ケースB: 天井 4 隅 z=2.4 (平面が原点を通らない) --- */
    uwb_config cfg_ceil;
    uwb_config_init(&cfg_ceil, ANCHORS_CEIL, N_ANCHORS_CEIL);

    coplanar = uwb_anchors_coplanar(&cfg_ceil, normal, &offset);
    ESP_LOGI(TAG,
             "[z=2.4] uwb_anchors_coplanar() = %d (1=同一平面)  normal=(%.3f,%.3f,%.3f) offset=%.3f",
             coplanar, (double)normal[0], (double)normal[1], (double)normal[2], (double)offset);

    uwb_real tag_ceil[3] = {4.0, 3.0, 1.2};
    uwb_meas meas_ceil[N_ANCHORS_CEIL];
    build_measurements(ANCHORS_CEIL, N_ANCHORS_CEIL, tag_ceil, NULL, 0.0, meas_ceil);

    uwb_solve_lv2(&cfg_ceil, meas_ceil, N_ANCHORS_CEIL, &fix);
    ESP_LOGI(TAG, "[z=2.4] Lv2: ok=%d ambiguous=%d p=(%.4f,%.4f,%.4f)  (原点を通らない平面なら "
                  "Lv2 も解ける)",
             fix.ok, fix.ambiguous, (double)fix.p[0], (double)fix.p[1], (double)fix.p[2]);
}

/* ============================================================ 4.6 GDOP */

static void scenario_gdop(const uwb_config *cfg)
{
    ESP_LOGI(TAG, "--- 4.6 GDOP ---");
    uwb_real gdop_center = uwb_gdop_at(cfg, TAG_TRUE);
    ESP_LOGI(TAG, "uwb_gdop_at(部屋中央付近) = %.3f (5超で配置が悪い目安)", (double)gdop_center);

    uwb_real corner[3] = {4.8, 4.8, 0.3};
    uwb_real gdop_corner = uwb_gdop_at(cfg, corner);
    ESP_LOGI(TAG, "uwb_gdop_at(部屋の隅寄り)   = %.3f", (double)gdop_corner);
}

/* ============================================================ 4.7 実行時間 */

typedef int (*solver_fn)(const uwb_config *, const uwb_meas *, int, uwb_fix *);

static void time_solver(const char *label, solver_fn fn, const uwb_config *cfg,
                         const uwb_meas *meas, int n)
{
    int64_t t_min = INT64_MAX, t_max = 0, t_sum = 0;
    uwb_fix fix;
    fix.ok = 0;

    for (int i = 0; i < TIMING_ITERATIONS; ++i) {
        int64_t t0 = esp_timer_get_time();
        fn(cfg, meas, n, &fix);
        int64_t t1 = esp_timer_get_time();
        int64_t dt = t1 - t0;
        if (dt < t_min) {
            t_min = dt;
        }
        if (dt > t_max) {
            t_max = dt;
        }
        t_sum += dt;
    }
    double avg = (double)t_sum / (double)TIMING_ITERATIONS;
    ESP_LOGI(TAG, "%-18s min=%lld us  avg=%.1f us  max=%lld us  (%d回, n_meas=%d, ok=%d)", label,
             (long long)t_min, avg, (long long)t_max, TIMING_ITERATIONS, n, fix.ok);
}

static void scenario_timing(const uwb_config *cfg, const uwb_meas *meas, int n)
{
    ESP_LOGI(TAG, "--- 4.7 実行時間計測 (esp_timer_get_time, %d回平均) ---", TIMING_ITERATIONS);
    time_solver("Lv0", uwb_solve_lv0, cfg, meas, n);
    time_solver("Lv2", uwb_solve_lv2, cfg, meas, n);
}

/* ============================================================ Lv3 EKF */

static void scenario_ekf(const uwb_config *cfg, int n)
{
    ESP_LOGI(TAG, "--- Lv3 EKF: 直線的に動く点を追わせて実行時間も見る ---");

    uwb_ekf ekf;
    uwb_ekf_init(&ekf, cfg, UWB_MOTION_CV, 0.5);

    int64_t t_min = INT64_MAX, t_max = 0, t_sum = 0;
    const int n_epoch = 40;
    double t = 0.0;
    uwb_fix fix;
    fix.ok = 0;

    for (int k = 0; k < n_epoch; ++k) {
        uwb_real tag[3] = {
            (uwb_real)(1.0 + 0.05 * k),
            (uwb_real)(1.0 + 0.03 * k),
            (uwb_real)(1.2),
        };
        uwb_meas meas[N_ANCHORS5];
        build_measurements(ANCHORS5, n, tag, NULL, 0.03, meas);

        int64_t t0 = esp_timer_get_time();
        uwb_ekf_update(&ekf, (uwb_real)t, meas, n, &fix);
        int64_t t1 = esp_timer_get_time();
        int64_t dt = t1 - t0;
        if (dt < t_min) {
            t_min = dt;
        }
        if (dt > t_max) {
            t_max = dt;
        }
        t_sum += dt;

        t += 0.1; /* 100ms 周期の測距を想定 */
    }

    double avg = (double)t_sum / (double)n_epoch;
    ESP_LOGI(TAG, "EKF update        min=%lld us  avg=%.1f us  max=%lld us  (%d epoch)",
             (long long)t_min, avg, (long long)t_max, n_epoch);
    if (fix.ok) {
        ESP_LOGI(TAG, "EKF 最終推定: p=(%.4f,%.4f,%.4f) v=(%.4f,%.4f,%.4f)", (double)fix.p[0],
                 (double)fix.p[1], (double)fix.p[2], (double)fix.v[0], (double)fix.v[1],
                 (double)fix.v[2]);
    } else {
        ESP_LOGW(TAG, "EKF 最終推定: ok=0 (立ち上げ未完了？)");
    }
}

/* ============================================================ タスク本体 */

static void soltest_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "=== uwb_loc ソルバ動作確認 (合成データ, 実機UWBハード不要) ===");
    ESP_LOGI(TAG, "uwb_loc version = %s", uwb_version());
    ESP_LOGI(TAG, "UWB_MAX_ANCHORS=%d UWB_MAX_MEAS=%d UWB_REAL_IS_FLOAT=%d", UWB_MAX_ANCHORS,
             UWB_MAX_MEAS, UWB_REAL_IS_FLOAT);
    if (UWB_REAL_IS_FLOAT) {
        ESP_LOGW(TAG, "float ビルド: 以下の Lv0/Lv2 の err= (真値との誤差) を double ビルドの"
                      "ログと突き合わせると、Beck法二分法・共分散の桁落ち影響が定量的に分かる"
                      "(取り込み元 README の警告どおり Lv2 で悪化する見込み。"
                      "components/uwb_loc/Kconfig の UWB_LOC_USE_FLOAT で切り替えてビルドし直すこと)");
    } else {
        ESP_LOGI(TAG, "double ビルド (既定)。float ビルド (CONFIG_UWB_LOC_USE_FLOAT=y) との"
                      "err= (真値との誤差) 比較が精度検証の基準になる");
    }

    srand(1234); /* 再現性のため固定シード */

    uwb_config cfg5;
    uwb_config_init(&cfg5, ANCHORS5, N_ANCHORS5);

    uwb_meas meas_exact[N_ANCHORS5];
    build_measurements(ANCHORS5, N_ANCHORS5, TAG_TRUE, NULL, 0.0, meas_exact);

    scenario_exact(&cfg5, meas_exact, N_ANCHORS5);
    scenario_noisy(&cfg5, N_ANCHORS5);
    scenario_outlier(&cfg5, N_ANCHORS5);
    scenario_coplanar();
    scenario_gdop(&cfg5);
    scenario_timing(&cfg5, meas_exact, N_ANCHORS5);
    scenario_ekf(&cfg5, N_ANCHORS5);

    /* 基本演算・メモリアクセスのマイクロベンチ (bench.c)。ソルバ計測より
     * 低レベルの、UWB_LOC_USE_FLOAT の効果見積もりの土台になる数字。 */
    bench_run();

    ESP_LOGI(TAG, "--- 4.8 スタック使用量の目安 ---");
    UBaseType_t hwm_words = uxTaskGetStackHighWaterMark(NULL);
    /* ESP-IDF (xtensa/riscv とも) は StackType_t = uint8_t なので、
     * uxTaskGetStackHighWaterMark() の戻り値はそのままバイト数になる。 */
    ESP_LOGI(TAG, "soltest_task: 確保 %d bytes, 最悪時点での残り %u bytes (使用推定 %d bytes)",
             SOLTEST_TASK_STACK_BYTES, (unsigned)hwm_words,
             (int)SOLTEST_TASK_STACK_BYTES - (int)hwm_words);

    ESP_LOGI(TAG, "=== soltest 完了 ===");
    vTaskDelete(NULL);
}

void app_main(void)
{
    /* esp_cpu_get_cycle_count() (bench.c) はコアごとに独立したカウンタ
     * (Xtensa CCOUNT) を読む。soltest_task が計測の途中でコアを跨いで
     * 移動すると、計測開始(t0)と終了(t1)が別コアの値になり差分が
     * 無意味になってしまう。ESP-IDF既定のSMPスケジューラはピン留めしない
     * 限りタスクをどちらのコアにも動かし得るので、CPU0へ固定しておく。 */
    xTaskCreatePinnedToCore(soltest_task, "soltest", SOLTEST_TASK_STACK_BYTES, NULL,
                             SOLTEST_TASK_PRIORITY, NULL, 0);
}
