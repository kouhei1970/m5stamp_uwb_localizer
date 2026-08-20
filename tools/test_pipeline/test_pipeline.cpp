/**
 * @file test_pipeline.cpp
 * @brief components/uwb_ranging のハード非依存部分（AnchorTable /
 * PositioningPipeline）を、実機ハードウェアなしで検証するホスト側テスト。
 *
 * 合成測距データ（既知アンカー配置＋既知タグ座標から計算した理論距離）を
 * uwb::RangingSample[] の形で組み立て、uwb::PositioningPipeline::solve() に
 * 直接渡す。スケジューラ（components/uwb_ranging/src/uwb_ranging_scheduler.cpp、
 * ESP-IDF依存）は経由しない — ここで検証したいのは測位パイプラインの論理
 * （欠測の扱い、外れ値ゲート、同一平面/原点通過の検出と警告、
 * 「測位不能」の返し方）であり、TWRの通信そのものではないため。
 *
 * third_party/uwb_localizer/c/tests/test_uwb.c と同じ CHECK() マクロの
 * 流儀（PASS/FAILをその場でカウントし、失敗時だけ内容を表示する）を踏襲する。
 */
#include <cmath>
#include <cstdio>

#include "uwb_qm33120_units.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_pipeline.hpp"

using namespace uwb;

static int g_run  = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                            \
    do {                                                            \
        ++g_run;                                                    \
        if (!(cond)) {                                               \
            ++g_fail;                                                \
            std::printf("  NG  %s:%d  ", __FILE__, __LINE__);         \
            std::printf(__VA_ARGS__);                                 \
            std::printf("\n");                                        \
        }                                                             \
    } while (0)

namespace {

float dist3(const float* a, const float* b)
{
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/** entries/tag から、全アンカー分の RangingSample を（欠測なしで）組み立てる。 */
void makeSamples(const AnchorEntry* entries, size_t n, const float* tag, RangingSample* out)
{
    for (size_t i = 0; i < n; ++i) {
        out[i].anchor_index = i;
        out[i].ok            = true;
        out[i].distance_m    = dist3(tag, entries[i].pos);
        out[i].elapsed_ms    = 5;
    }
}

} // namespace

/* ==================================================================== *
 * 1. アンカー4台・非同一平面・ノイズなし → 真値と一致
 * ==================================================================== */
static void scenario1_exact_4anchors()
{
    std::printf("--- 1. アンカー4台・非同一平面・ノイズなし ---\n");

    static const AnchorEntry entries[4] = {
        {0x0001, {0.0f, 0.0f, 2.4f}, 0.0f, true},
        {0x0002, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0003, {5.0f, 5.0f, 2.4f}, 0.0f, true},
        {0x0004, {0.0f, 5.0f, 0.2f}, 0.0f, true},
    };
    const float truth[3] = {2.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 4), "AnchorTable::set() 失敗");

    const PlacementCheck pc = table.checkPlacement();
    CHECK(!pc.coplanar, "4台が非同一平面のはずなのに coplanar=1 と判定された");

    RangingSample samples[4];
    makeSamples(entries, 4, truth, samples);

    PositioningPipeline pipeline(table);

    const PositionResult lv0 = pipeline.solve(samples, 4, SolverLevel::Lv0);
    CHECK(lv0.solvable && lv0.ok, "Lv0 が解けなかった (solvable=%d ok=%d)", lv0.solvable, lv0.ok);
    CHECK(dist3(lv0.p, truth) < 1e-3f, "Lv0 の位置が真値と一致しない (誤差=%.6f)",
          static_cast<double>(dist3(lv0.p, truth)));

    const PositionResult lv2 = pipeline.solve(samples, 4, SolverLevel::Lv2);
    CHECK(lv2.solvable && lv2.ok, "Lv2 が解けなかった (solvable=%d ok=%d)", lv2.solvable, lv2.ok);
    CHECK(dist3(lv2.p, truth) < 1e-3f, "Lv2 の位置が真値と一致しない (誤差=%.6f)",
          static_cast<double>(dist3(lv2.p, truth)));
    CHECK(!lv2.ambiguous, "非同一平面配置なのに ambiguous=1 になった");
    CHECK(lv2.excluded == 0, "無雑音・外れ値なしなのに excluded=0x%lx", lv2.excluded);
}

/* ==================================================================== *
 * 2. アンカー5台で1台に大きな外れ値 → Lv2が弾き、解が維持される
 * ==================================================================== */
static void scenario2_outlier_rejected()
{
    std::printf("--- 2. アンカー5台、1台に大きな外れ値 ---\n");

    static const AnchorEntry entries[5] = {
        {0x0001, {0.0f, 0.0f, 2.4f}, 0.0f, true}, {0x0002, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0003, {5.0f, 5.0f, 2.4f}, 0.0f, true}, {0x0004, {0.0f, 5.0f, 0.2f}, 0.0f, true},
        {0x0005, {2.5f, 2.5f, 2.4f}, 0.0f, true},
    };
    const float truth[3] = {2.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 5), "AnchorTable::set() 失敗");

    RangingSample samples[5];
    makeSamples(entries, 5, truth, samples);
    const size_t outlierIdx = 2;
    samples[outlierIdx].distance_m += 3.0f; // 大きなNLOSバイアスを模擬

    PositioningPipeline pipeline(table);
    const PositionResult lv2 = pipeline.solve(samples, 5, SolverLevel::Lv2);

    CHECK(lv2.solvable && lv2.ok, "外れ値混入で解が維持されなかった (solvable=%d ok=%d)", lv2.solvable,
          lv2.ok);
    const bool excludedOutlier = (lv2.excluded & (1UL << outlierIdx)) != 0;
    CHECK(excludedOutlier, "外れ値アンカー(添字%zu)が excluded に入っていない (excluded=0x%lx)",
          outlierIdx, lv2.excluded);
    CHECK(dist3(lv2.p, truth) < 0.05f, "外れ値を弾いた後も真値から離れすぎている (誤差=%.4f)",
          static_cast<double>(dist3(lv2.p, truth)));
}

/* ==================================================================== *
 * 3. アンカー5台のうち2台が欠測 → 有効3件なので「測位不能」
 * ==================================================================== */
static void scenario3_two_missing_unsolvable()
{
    std::printf("--- 3. アンカー5台のうち2台が欠測（有効3件） ---\n");

    static const AnchorEntry entries[5] = {
        {0x0001, {0.0f, 0.0f, 2.4f}, 0.0f, true}, {0x0002, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0003, {5.0f, 5.0f, 2.4f}, 0.0f, true}, {0x0004, {0.0f, 5.0f, 0.2f}, 0.0f, true},
        {0x0005, {2.5f, 2.5f, 2.4f}, 0.0f, true},
    };
    const float truth[3] = {2.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 5), "AnchorTable::set() 失敗");

    RangingSample samples[5];
    makeSamples(entries, 5, truth, samples);
    samples[3].ok = false; // 欠測（応答なし/タイムアウト）
    samples[4].ok = false;

    PositioningPipeline pipeline(table);
    const PositionResult lv2 = pipeline.solve(samples, 5, SolverLevel::Lv2);

    CHECK(lv2.nTotal == 3, "有効測距数の数え方がおかしい (nTotal=%d, 期待3)", lv2.nTotal);
    CHECK(!lv2.solvable, "有効測距3件なのにソルバを呼んでしまった (solvable=%d)", lv2.solvable);
    CHECK(!lv2.ok, "有効測距3件なのに ok=1 になった（測位不能を返すべき）");
}

/* ==================================================================== *
 * 4. アンカー5台のうち1台が欠測 → 有効4件で解ける
 * ==================================================================== */
static void scenario4_one_missing_solvable()
{
    std::printf("--- 4. アンカー5台のうち1台が欠測（有効4件） ---\n");

    static const AnchorEntry entries[5] = {
        {0x0001, {0.0f, 0.0f, 2.4f}, 0.0f, true}, {0x0002, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0003, {5.0f, 5.0f, 2.4f}, 0.0f, true}, {0x0004, {0.0f, 5.0f, 0.2f}, 0.0f, true},
        {0x0005, {2.5f, 2.5f, 2.4f}, 0.0f, true},
    };
    const float truth[3] = {2.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 5), "AnchorTable::set() 失敗");

    RangingSample samples[5];
    makeSamples(entries, 5, truth, samples);
    samples[4].ok = false; // 1台だけ欠測。外れ値バイアスは入れない（無雑音）

    PositioningPipeline pipeline(table);
    const PositionResult lv2 = pipeline.solve(samples, 5, SolverLevel::Lv2);

    CHECK(lv2.nTotal == 4, "有効測距数の数え方がおかしい (nTotal=%d, 期待4)", lv2.nTotal);
    CHECK(lv2.solvable, "有効測距4件あるのに solvable=0 になった");
    CHECK(lv2.ok, "有効測距4件あるのに ok=0 になった (残差RMS=%.4f)",
          static_cast<double>(lv2.residualRms));
    CHECK(dist3(lv2.p, truth) < 1e-3f, "1台欠測時の位置が真値と一致しない (誤差=%.6f)",
          static_cast<double>(dist3(lv2.p, truth)));
}

/* ==================================================================== *
 * 5. 同一平面配置（天井4隅、原点は通らない）→ coplanarが検出され警告が出る
 * ==================================================================== */
static void scenario5_coplanar_detected()
{
    std::printf("--- 5. 同一平面配置（天井、原点は通らない） ---\n");

    static const AnchorEntry entries[4] = {
        {0x0001, {0.2f, 0.2f, 2.4f}, 0.0f, true}, {0x0002, {7.8f, 0.2f, 2.4f}, 0.0f, true},
        {0x0003, {7.8f, 5.8f, 2.4f}, 0.0f, true}, {0x0004, {0.2f, 5.8f, 2.4f}, 0.0f, true},
    };
    const float truth[3] = {4.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 4), "AnchorTable::set() 失敗");

    const PlacementCheck pc = table.checkPlacement();
    CHECK(pc.coplanar, "天井4隅の同一平面配置なのに coplanar=0 と判定された"
                        "（呼び出し側はこの値を見て警告ログを出す）");
    CHECK(!pc.originWarning,
          "平面オフセット=2.4m でワールド原点を通らないはずなのに originWarning=1 になった "
          "(offset=%.4f)",
          static_cast<double>(pc.offsetM));

    // 同一平面でも Lv2 自体は解ける（ただし高さは鏡像の可能性があるので
    // ambiguous=1 になる。docs/ANCHOR_PLACEMENT.md の実測どおりであることも
    // 併せて確認しておく）。
    RangingSample samples[4];
    makeSamples(entries, 4, truth, samples);
    PositioningPipeline pipeline(table);
    const PositionResult lv2 = pipeline.solve(samples, 4, SolverLevel::Lv2);
    CHECK(lv2.solvable, "同一平面配置(非原点)で solvable=0 になった");
    CHECK(lv2.ok, "同一平面配置(非原点)で ok=0 になった（原点通過ケースと混同していないか確認）");
    CHECK(lv2.ambiguous, "同一平面配置なのに ambiguous=0 になった（高さが一意に決まるのは不自然）");
}

/* ==================================================================== *
 * 6. アンカー平面が z=0（原点を通る）→ Lv2が失敗し、パイプラインが
 *    それを「測位不能」として正しく扱う
 * ==================================================================== */
static void scenario6_origin_plane_fails()
{
    std::printf("--- 6. アンカー平面が原点を通る (z=0) ---\n");

    static const AnchorEntry entries[4] = {
        {0x0001, {0.0f, 0.0f, 0.0f}, 0.0f, true}, {0x0002, {5.0f, 0.0f, 0.0f}, 0.0f, true},
        {0x0003, {5.0f, 5.0f, 0.0f}, 0.0f, true}, {0x0004, {0.0f, 5.0f, 0.0f}, 0.0f, true},
    };
    const float truth[3] = {2.0f, 3.0f, 1.2f};

    AnchorTable table;
    CHECK(table.set(entries, 4), "AnchorTable::set() 失敗");

    const PlacementCheck pc = table.checkPlacement();
    CHECK(pc.coplanar, "z=0の4台は同一平面のはずなのに coplanar=0 と判定された");
    CHECK(pc.originWarning,
          "平面が厳密に原点を通る(z=0)はずなのに originWarning=0 になった (offset=%.6f)",
          static_cast<double>(pc.offsetM));

    RangingSample samples[4];
    makeSamples(entries, 4, truth, samples);
    PositioningPipeline pipeline(table);
    const PositionResult lv2 = pipeline.solve(samples, 4, SolverLevel::Lv2);

    // 実測で判明している既知の挙動 (docs/ANCHOR_PLACEMENT.md):
    // 有効測距は4件あるのでソルバは呼ばれる (solvable=1) が、
    // Beck法が同一平面かつ原点通過で特異になり Gauss-Newton も動けず、
    // 共分散が特異になって Lv2 は ok=0 を返す。
    CHECK(lv2.solvable, "有効測距4件あるのに solvable=0 になった（原点通過の判定と混同していないか）");
    CHECK(!lv2.ok, "既知の失敗条件(原点通過の同一平面)なのに ok=1 になった。"
                   "third_party 側の挙動が変わった可能性がある");

    // パイプラインの呼び出し側からすると、この ok=0 がそのまま「測位不能」の
    // シグナルになる。2D固定へのフォールバックが用意されていることも確認する。
    table.setDimension2D(truth[2]);
    const PositionResult lv2_2d = pipeline.solve(samples, 4, SolverLevel::Lv2);
    CHECK(lv2_2d.solvable && lv2_2d.ok, "dim=2 フォールバック後も解けなかった (solvable=%d ok=%d)",
          lv2_2d.solvable, lv2_2d.ok);
    if (lv2_2d.solvable && lv2_2d.ok) {
        const float xyErr = std::sqrt((lv2_2d.p[0] - truth[0]) * (lv2_2d.p[0] - truth[0]) +
                                       (lv2_2d.p[1] - truth[1]) * (lv2_2d.p[1] - truth[1]));
        CHECK(xyErr < 1e-3f, "dim=2 フォールバック後のxy位置が真値と一致しない (誤差=%.6f)",
              static_cast<double>(xyErr));
    }
    table.setDimension3D();
}

/* ==================================================================== *
 * 7. docs/REIMPL_PLAN.md R1: usToUus()（実us -> UUS 変換）の検算
 * ==================================================================== */
static void scenario7_r1_us_to_uus()
{
    std::printf("--- 7. R1: uwb::detail::usToUus() 検算 ---\n");
    using uwb::detail::usToUus;

    // docs/REIMPL_PLAN.md R1 に明記された例。
    CHECK(usToUus(650) == 634, "650実us -> 634UUSのはずが %u", usToUus(650));
    CHECK(usToUus(1000) == 975, "1000実us -> 975UUSのはずが %u", usToUus(1000));

    // 端点・追加確認: 0はそのまま0。
    CHECK(usToUus(0) == 0, "0実us -> 0UUSのはずが %u", usToUus(0));

    // 4992/5120 = 499.2/512 が正確な比になる値では割り切れる
    // (10000実us = 9750 UUS ちょうど。9750*512/499.2 = 10000 で検算可能)。
    CHECK(usToUus(10000) == 9750, "10000実us -> 9750UUSのはずが %u", usToUus(10000));

    // 1 UUS ≒ 1.02564 us なので、実usの方がUUS値よりわずかに大きい
    // (Qorvoの*_UUS定数を誤ってそのまま使うと、本APIのUUSとしては
    // 約2.5%長い遅延になる、というR1の指摘そのものを裏から確認する)。
    CHECK(usToUus(400) < 400, "400実us のUUS換算(%u)が400を下回らない", usToUus(400));
}

/* ==================================================================== *
 * 8. docs/REIMPL_PLAN.md R8: sfdTimeoutFromPhy()（SFDタイムアウト自動計算）
 *    の検算。プリアンブル長を変えたときに正しい値へ追随することを確認する。
 * ==================================================================== */
static void scenario8_r8_sfd_timeout_auto()
{
    std::printf("--- 8. R8: uwb::detail::sfdTimeoutFromPhy() 検算 ---\n");
    using uwb::detail::sfdTimeoutFromPhy;

    // 既定PHY (preamble128 / SFD8 / PAC8): 128+1+8-8=129。
    // sfdTimeout==129固定だった旧デフォルトと同じ値になることを確認
    // (デフォルトPHYでの挙動に回帰が無いこと)。
    CHECK(sfdTimeoutFromPhy(0, 128, 8, 8) == 129, "既定PHYの自動計算値が129でない (%u)",
          sfdTimeoutFromPhy(0, 128, 8, 8));

    // R8の核心: preambleLengthをLen256に変えると、257であるべき値に
    // 追随すること（sfdTimeout=129固定だった旧実装はここで129のまま
    // 止まっていた = 受信率が激減する罠だった）。
    CHECK(sfdTimeoutFromPhy(0, 256, 8, 8) == 257, "preamble256での自動計算値が257でない (%u)",
          sfdTimeoutFromPhy(0, 256, 8, 8));

    // 別の組み合わせでも式通りに追随すること (preamble512/SFD16/PAC32)。
    CHECK(sfdTimeoutFromPhy(0, 512, 16, 32) == 497, "preamble512/SFD16/PAC32の自動計算値が497でない (%u)",
          sfdTimeoutFromPhy(0, 512, 16, 32));

    // ユーザが非0を明示指定した場合はそれを優先し、自動計算しないこと
    // （既存挙動の維持）。
    CHECK(sfdTimeoutFromPhy(200, 256, 8, 8) == 200, "非0の明示指定(200)が上書きされた (%u)",
          sfdTimeoutFromPhy(200, 256, 8, 8));
}

int main()
{
    std::printf("=== tools/test_pipeline: uwb_ranging 測位パイプライン 合成データ検証 ===\n");
    std::printf("UWB_MAX_ANCHORS=%d UWB_MAX_MEAS=%d UWB_REAL_IS_FLOAT=%d\n\n", UWB_MAX_ANCHORS,
                UWB_MAX_MEAS, UWB_REAL_IS_FLOAT);

    scenario1_exact_4anchors();
    scenario2_outlier_rejected();
    scenario3_two_missing_unsolvable();
    scenario4_one_missing_solvable();
    scenario5_coplanar_detected();
    scenario6_origin_plane_fails();
    scenario7_r1_us_to_uus();
    scenario8_r8_sfd_timeout_auto();

    std::printf("\n=== %d 件中 %d 件失敗 ===\n", g_run, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
