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
 * さらに components/uwb_cfgstore のハード非依存部分（設定のシリアライズ/
 * デシリアライズ）も同じバイナリで検算する。NVSへの実際の読み書きは
 * ESP-IDF 依存なのでここには含めず、バイト列の往復・境界値・壊れたデータの
 * 扱いだけを見る（実機が無い段階で検証できるのはこの層まで）。
 *
 * third_party/uwb_localizer/c/tests/test_uwb.c と同じ CHECK() マクロの
 * 流儀（PASS/FAILをその場でカウントし、失敗時だけ内容を表示する）を踏襲する。
 */
#include <cmath>
#include <cstdio>

#include <cstring>

#include "uwb_cfgstore_blob.hpp"
#include "uwb_qm33120_frame_match.hpp"
#include "uwb_qm33120_timing.hpp"
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

/* ==================================================================== *
 * 9. docs/REIMPL_PLAN.md R2: uwb::detail::frameMatchesExpectation()
 *    （受信フレームが「待っていたフレームか」の純関数判定）の検算。
 *    uwb_qm33120_twr.cpp の6箇所の受信ループが、この関数のfalseを
 *    「エラーではなく受信継続」の合図として使う（本体側はハードウェアが
 *    絡むためホストで直接は検証できないが、判定ロジックそのものは
 *    ここで検算できる）。
 * ==================================================================== */
static void scenario9_r2_frame_match()
{
    std::printf("--- 9. R2: uwb::detail::frameMatchesExpectation() 検算 ---\n");
    using uwb::detail::FrameExpectation;
    using uwb::detail::frameMatchesExpectation;
    using uwb::detail::ParsedFrameSummary;

    // requestRange()/requestDSRange() が使う形: sequence/panId/src/dstを
    // すべて厳密照合する。全部一致すればtrue。
    const FrameExpectation strict{/*checkSequence=*/true, 7, 0xDECA, 0x0002, 0x0001};

    {
        const ParsedFrameSummary allMatch{true, true, 7, 0xDECA, 0x0002, 0x0001};
        CHECK(frameMatchesExpectation(allMatch, strict), "全項目一致のフレームがfalseになった");
    }
    {
        // 【R2の核心】シーケンス番号だけ不一致 = 「他人（別リンク）宛て」の
        // 典型例。エラーではなく「待っていたフレームではない」というfalseに
        // なることを確認する（呼び出し側はこれを見てdwt_rxenable()して
        // 受信継続する。中断はしない）。
        const ParsedFrameSummary wrongSeq{true, true, 8, 0xDECA, 0x0002, 0x0001};
        CHECK(!frameMatchesExpectation(wrongSeq, strict), "シーケンス番号不一致がtrueになった");
    }
    {
        // 別リンクの送信元アドレス（5台round-robin構成で最も起こりやすい
        // 不一致パターン）。
        const ParsedFrameSummary wrongSrc{true, true, 7, 0xDECA, 0x0005, 0x0001};
        CHECK(!frameMatchesExpectation(wrongSrc, strict), "送信元アドレス不一致がtrueになった");
    }
    {
        const ParsedFrameSummary wrongDst{true, true, 7, 0xDECA, 0x0002, 0x0003};
        CHECK(!frameMatchesExpectation(wrongDst, strict), "宛先アドレス不一致がtrueになった");
    }
    {
        const ParsedFrameSummary wrongPan{true, true, 7, 0xBEEF, 0x0002, 0x0001};
        CHECK(!frameMatchesExpectation(wrongPan, strict), "PAN ID不一致がtrueになった");
    }
    {
        // ヘッダ解析自体が失敗（frameLenが短すぎる等）。payloadOk側は
        // don't careでもheaderOk=falseなら必ずfalse（短絡評価）。
        const ParsedFrameSummary badHeader{false, true, 7, 0xDECA, 0x0002, 0x0001};
        CHECK(!frameMatchesExpectation(badHeader, strict), "headerOk=falseなのにtrueになった");
    }
    {
        // 関数コード/フレーム長不一致（例: "TWR"応答待ちなのに別の関数コード
        // のフレームが来た）。
        const ParsedFrameSummary badPayload{true, false, 7, 0xDECA, 0x0002, 0x0001};
        CHECK(!frameMatchesExpectation(badPayload, strict), "payloadOk=falseなのにtrueになった");
    }

    // respondRange()/respondDSRange() のPoll待ちが使う形: checkSequence=false
    // かつ src=0(don't care)。「どのTagからのPollでも受理する」の再現。
    const FrameExpectation pollWait{/*checkSequence=*/false, 0, 0xDECA, /*src=*/0, 0x0002};
    {
        const ParsedFrameSummary fromTagA{true, true, 3, 0xDECA, 0x0001, 0x0002};
        const ParsedFrameSummary fromTagB{true, true, 200, 0xDECA, 0x0009, 0x0002};
        CHECK(frameMatchesExpectation(fromTagA, pollWait), "src=0(don't care)でTagAからのPollがfalseになった");
        CHECK(frameMatchesExpectation(fromTagB, pollWait),
              "src=0(don't care)でシーケンス番号違い/別TagからのPollがfalseになった");
    }
    {
        // dstだけは常に照合する（自分宛てでなければ無条件で不一致）。
        const ParsedFrameSummary toOtherAnchor{true, true, 3, 0xDECA, 0x0001, 0x0003};
        CHECK(!frameMatchesExpectation(toOtherAnchor, pollWait), "宛先違いのPollがdon't careですり抜けた");
    }
}


/* ==================================================================== *
 * 10. uwb_cfgstore: 設定のシリアライズ/デシリアライズ
 *
 * 実機が無いので NVS そのものは叩けないが、「NVSに置くバイト列」を作る/
 * 読む部分はホストで完全に検算できる。ここで見るのは
 *   (a) 往復して値が1ビットも変わらないこと
 *   (b) 境界値（件数の下限/上限、座標・遅延の上限、アドレスの端）
 *   (c) 壊れたデータ・未初期化フラッシュを**必ず**弾くこと
 *       （弾けば呼び出し側がコンパイル時の既定値へフォールバックする）
 * ==================================================================== */

namespace {

using uwb::cfg::BlobStatus;

/** 2つの AnchorEntry がビット単位で同じか（float は == で厳密比較する。
 *  シリアライズはビットパターンを保存するので丸めは起きないはず）。 */
bool sameEntry(const AnchorEntry& a, const AnchorEntry& b)
{
    return a.short_addr == b.short_addr && a.enabled == b.enabled && a.pos[0] == b.pos[0] &&
           a.pos[1] == b.pos[1] && a.pos[2] == b.pos[2] && a.antenna_delay_m == b.antenna_delay_m;
}

/** CRC を計算し直して末尾へ書く（バイト列を意図的に改ざんしたあとに使う）。 */
void refreshCrc(uint8_t* blob, size_t len)
{
    const uint32_t crc = uwb::cfg::crc32(blob, len - uwb::cfg::kBlobCrcSize);
    uint8_t* p          = blob + len - uwb::cfg::kBlobCrcSize;
    p[0]                 = static_cast<uint8_t>(crc & 0xFFu);
    p[1]                 = static_cast<uint8_t>((crc >> 8) & 0xFFu);
    p[2]                 = static_cast<uint8_t>((crc >> 16) & 0xFFu);
    p[3]                 = static_cast<uint8_t>((crc >> 24) & 0xFFu);
}

} // namespace

static void scenario10_cfgstore_roundtrip()
{
    std::printf("--- 10. uwb_cfgstore: アンカーテーブルの往復 ---\n");

    // firmware/tag の kAnchors[] と同じ形（実運用でそのまま保存される値）。
    const AnchorEntry src[5] = {
        {0x0002, {0.0f, 0.0f, 2.4f}, 0.0f, true},
        {0x0003, {5.0f, 0.0f, 0.2f}, 0.125f, true},
        {0x0004, {5.0f, 5.0f, 2.4f}, -0.0625f, true},
        {0x0005, {0.0f, 5.0f, 0.2f}, 0.0f, false}, // enabled=false も往復すること
        {0x0006, {2.5f, 2.5f, 2.4f}, 0.1f, true},  // 0.1f は2進で割り切れない値
    };

    uint8_t blob[uwb::cfg::kMaxBlobSize];
    size_t len = 0;
    CHECK(uwb::cfg::serializeAnchorTable(src, 5, blob, sizeof(blob), &len) == BlobStatus::Ok,
          "5件のシリアライズに失敗した");
    CHECK(len == uwb::cfg::anchorTableBlobSize(5), "長さが anchorTableBlobSize(5)=%zu と違う (%zu)",
          uwb::cfg::anchorTableBlobSize(5), len);
    CHECK(len == 12 + 20 * 5 + 4, "バイト列の長さが仕様（ヘッダ12+20*件数+CRC4）と違う: %zu", len);

    AnchorEntry back[kMaxAnchors];
    size_t count = 0;
    CHECK(uwb::cfg::deserializeAnchorTable(blob, len, back, kMaxAnchors, &count) == BlobStatus::Ok,
          "5件のデシリアライズに失敗した");
    CHECK(count == 5, "復元件数が違う: %zu", count);
    for (size_t i = 0; i < 5; ++i) {
        CHECK(sameEntry(src[i], back[i]), "エントリ%zuが往復で変化した", i);
    }

    // 件数の下限（1件）と上限（kMaxAnchors）。
    AnchorEntry many[kMaxAnchors];
    for (size_t i = 0; i < kMaxAnchors; ++i) {
        many[i].short_addr      = static_cast<uint16_t>(0x0010 + i);
        many[i].pos[0]          = static_cast<float>(i) * 1.5f;
        many[i].pos[1]          = -static_cast<float>(i);
        many[i].pos[2]          = 0.25f * static_cast<float>(i);
        many[i].antenna_delay_m = 0.001f * static_cast<float>(i);
        many[i].enabled         = ((i % 2) == 0);
    }
    CHECK(uwb::cfg::serializeAnchorTable(many, 1, blob, sizeof(blob), &len) == BlobStatus::Ok,
          "1件（下限）のシリアライズに失敗した");
    CHECK(uwb::cfg::deserializeAnchorTable(blob, len, back, kMaxAnchors, &count) == BlobStatus::Ok &&
              count == 1 && sameEntry(many[0], back[0]),
          "1件（下限）の往復に失敗した");

    CHECK(uwb::cfg::serializeAnchorTable(many, kMaxAnchors, blob, sizeof(blob), &len) == BlobStatus::Ok,
          "%zu件（上限）のシリアライズに失敗した", kMaxAnchors);
    CHECK(len == uwb::cfg::kMaxBlobSize, "上限件数のときの長さが kMaxBlobSize と違う: %zu", len);
    CHECK(uwb::cfg::deserializeAnchorTable(blob, len, back, kMaxAnchors, &count) == BlobStatus::Ok &&
              count == kMaxAnchors,
          "%zu件（上限）のデシリアライズに失敗した", kMaxAnchors);
    for (size_t i = 0; i < kMaxAnchors; ++i) {
        CHECK(sameEntry(many[i], back[i]), "上限件数のエントリ%zuが往復で変化した", i);
    }

    // 自局アドレス（アンカー側）の往復。0x0000 と 0xFFFE が端。
    const uint16_t addrs[3] = {0x0000, 0x0002, 0xFFFE};
    for (int i = 0; i < 3; ++i) {
        uint8_t abuf[uwb::cfg::anchorAddrBlobSize()];
        size_t alen  = 0;
        uint16_t got = 0xAAAA;
        CHECK(uwb::cfg::serializeAnchorAddr(addrs[i], abuf, sizeof(abuf), &alen) == BlobStatus::Ok,
              "アドレス0x%04Xのシリアライズに失敗した", addrs[i]);
        CHECK(alen == uwb::cfg::anchorAddrBlobSize() && alen == 20,
              "アドレスのバイト列長が仕様（ヘッダ12+本体4+CRC4=20）と違う: %zu", alen);
        CHECK(uwb::cfg::deserializeAnchorAddr(abuf, alen, &got) == BlobStatus::Ok && got == addrs[i],
              "アドレス0x%04Xの往復に失敗した (got=0x%04X)", addrs[i], got);
    }
}

static void scenario11_cfgstore_boundaries()
{
    std::printf("--- 11. uwb_cfgstore: 境界値と不正な入力 ---\n");

    uint8_t blob[uwb::cfg::kMaxBlobSize];
    size_t len = 0;
    AnchorEntry e{0x0002, {0.0f, 0.0f, 2.4f}, 0.0f, true};

    // 件数 0 と 上限+1 は拒否する。
    CHECK(uwb::cfg::serializeAnchorTable(&e, 0, blob, sizeof(blob), &len) == BlobStatus::BadCount,
          "件数0がBadCountにならなかった");
    CHECK(uwb::cfg::serializeAnchorTable(&e, kMaxAnchors + 1, blob, sizeof(blob), &len) ==
              BlobStatus::BadCount,
          "件数が上限超なのにBadCountにならなかった");

    // 出力バッファが1バイト足りない。
    CHECK(uwb::cfg::serializeAnchorTable(&e, 1, blob, uwb::cfg::anchorTableBlobSize(1) - 1, &len) ==
              BlobStatus::TooShort,
          "出力バッファ不足がTooShortにならなかった");

    // nullptr。
    CHECK(uwb::cfg::serializeAnchorTable(nullptr, 1, blob, sizeof(blob), &len) == BlobStatus::NullArg,
          "entries=nullptrがNullArgにならなかった");
    CHECK(uwb::cfg::deserializeAnchorTable(blob, 32, nullptr, kMaxAnchors, nullptr) == BlobStatus::NullArg,
          "out=nullptrがNullArgにならなかった");

    // 座標の上限ちょうどは通り、少し超えると弾く。
    AnchorEntry lim{0x0002, {uwb::cfg::kMaxCoordM, -uwb::cfg::kMaxCoordM, 0.0f}, uwb::cfg::kMaxAntennaDelayM,
                     true};
    CHECK(uwb::cfg::isValidAnchorEntry(lim), "座標・遅延が上限ちょうどなのに弾かれた");
    CHECK(uwb::cfg::serializeAnchorTable(&lim, 1, blob, sizeof(blob), &len) == BlobStatus::Ok,
          "上限ちょうどのエントリを書けなかった");

    AnchorEntry over = lim;
    over.pos[0]       = uwb::cfg::kMaxCoordM * 1.001f;
    CHECK(!uwb::cfg::isValidAnchorEntry(over), "座標が上限超なのに通った");
    CHECK(uwb::cfg::serializeAnchorTable(&over, 1, blob, sizeof(blob), &len) == BlobStatus::BadEntry,
          "座標が上限超なのにBadEntryにならなかった");

    AnchorEntry overDelay = lim;
    overDelay.antenna_delay_m = uwb::cfg::kMaxAntennaDelayM * 1.001f;
    CHECK(uwb::cfg::serializeAnchorTable(&overDelay, 1, blob, sizeof(blob), &len) == BlobStatus::BadEntry,
          "アンテナ遅延が上限超なのにBadEntryにならなかった");

    // NaN / Inf は書かせない（測位ソルバへ渡ると全体が壊れる）。
    AnchorEntry nan = lim;
    nan.pos[2]       = std::nanf("");
    CHECK(!uwb::cfg::isValidAnchorEntry(nan), "NaN座標が通った");
    CHECK(uwb::cfg::serializeAnchorTable(&nan, 1, blob, sizeof(blob), &len) == BlobStatus::BadEntry,
          "NaN座標がBadEntryにならなかった");

    AnchorEntry inf = lim;
    inf.pos[1]       = HUGE_VALF;
    CHECK(uwb::cfg::serializeAnchorTable(&inf, 1, blob, sizeof(blob), &len) == BlobStatus::BadEntry,
          "Inf座標がBadEntryにならなかった");

    // ブロードキャストアドレスは自局にもアンカーにも使えない。
    CHECK(!uwb::cfg::isValidShortAddr(0xFFFF), "0xFFFFが有効アドレス扱いされた");
    CHECK(uwb::cfg::isValidShortAddr(0xFFFE), "0xFFFEが無効アドレス扱いされた");
    AnchorEntry bcast = lim;
    bcast.short_addr   = 0xFFFF;
    CHECK(uwb::cfg::serializeAnchorTable(&bcast, 1, blob, sizeof(blob), &len) == BlobStatus::BadEntry,
          "アンカーの0xFFFFがBadEntryにならなかった");
    uint8_t abuf[uwb::cfg::anchorAddrBlobSize()];
    CHECK(uwb::cfg::serializeAnchorAddr(0xFFFF, abuf, sizeof(abuf), nullptr) == BlobStatus::BadEntry,
          "自局アドレスの0xFFFFがBadEntryにならなかった");

    // 受け側の容量が足りない場合（保存は5件、受けは3件ぶんしか無い）。
    const AnchorEntry five[5] = {
        {0x0002, {0.0f, 0.0f, 2.4f}, 0.0f, true}, {0x0003, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0004, {5.0f, 5.0f, 2.4f}, 0.0f, true}, {0x0005, {0.0f, 5.0f, 0.2f}, 0.0f, true},
        {0x0006, {2.5f, 2.5f, 2.4f}, 0.0f, true},
    };
    CHECK(uwb::cfg::serializeAnchorTable(five, 5, blob, sizeof(blob), &len) == BlobStatus::Ok,
          "5件のシリアライズに失敗した");
    AnchorEntry small[3];
    size_t count = 0;
    CHECK(uwb::cfg::deserializeAnchorTable(blob, len, small, 3, &count) == BlobStatus::TooShort,
          "受け側の容量不足がTooShortにならなかった");
}

static void scenario12_cfgstore_corruption()
{
    std::printf("--- 12. uwb_cfgstore: 壊れたデータ・未初期化フラッシュ ---\n");

    const AnchorEntry src[4] = {
        {0x0002, {0.0f, 0.0f, 2.4f}, 0.0f, true},
        {0x0003, {5.0f, 0.0f, 0.2f}, 0.0f, true},
        {0x0004, {5.0f, 5.0f, 2.4f}, 0.0f, true},
        {0x0005, {0.0f, 5.0f, 0.2f}, 0.0f, true},
    };
    uint8_t good[uwb::cfg::kMaxBlobSize];
    size_t len = 0;
    CHECK(uwb::cfg::serializeAnchorTable(src, 4, good, sizeof(good), &len) == BlobStatus::Ok,
          "基準となるバイト列を作れなかった");

    AnchorEntry out[kMaxAnchors];
    size_t count = 0;
    uint8_t bad[uwb::cfg::kMaxBlobSize];

    // (1) 未初期化フラッシュ。全0 と 全0xFF のどちらも「設定が無い」と
    //     判定できること（＝呼び出し側が既定値へ倒せること）。
    std::memset(bad, 0x00, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadMagic,
          "全0のバイト列がBadMagicにならなかった");
    std::memset(bad, 0xFF, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadMagic,
          "全0xFFのバイト列がBadMagicにならなかった");

    // (2) 短すぎる（書き込み途中で電源が落ちた等）。
    CHECK(uwb::cfg::deserializeAnchorTable(good, 0, out, kMaxAnchors, &count) == BlobStatus::TooShort,
          "長さ0がTooShortにならなかった");
    CHECK(uwb::cfg::deserializeAnchorTable(good, uwb::cfg::kBlobHeaderSize + uwb::cfg::kBlobCrcSize - 1, out,
                                            kMaxAnchors, &count) == BlobStatus::TooShort,
          "ヘッダ+CRC未満がTooShortにならなかった");

    // (3) フォーマット版が違う（将来の版で書かれた設定を読んだ場合）。
    std::memcpy(bad, good, len);
    bad[4] = static_cast<uint8_t>(uwb::cfg::kBlobVersion + 1);
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadVersion,
          "版違いがBadVersionにならなかった");

    // (4) 種別が違う（アドレスのキーにテーブルが入っていた等）。
    CHECK(uwb::cfg::deserializeAnchorAddr(good, len, nullptr) == BlobStatus::NullArg,
          "outAddr=nullptrがNullArgにならなかった");
    uint16_t addr = 0;
    CHECK(uwb::cfg::deserializeAnchorAddr(good, len, &addr) == BlobStatus::BadKind,
          "テーブルのバイト列をアドレスとして読んだのにBadKindにならなかった");

    // (5) CRC不一致（1ビット化け）。**CRCを直さずに**中身だけ書き換える。
    std::memcpy(bad, good, len);
    bad[uwb::cfg::kBlobHeaderSize + 4] ^= 0x01u; // 先頭エントリの pos[0] の最下位バイト
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadCrc,
          "1ビット化けがBadCrcにならなかった");

    // (6) 件数だけ改ざん（長さと合わなくなる）。CRCも直しておく＝
    //     CRCでは検出できないので、長さの整合で弾く必要がある。
    std::memcpy(bad, good, len);
    bad[8] = 5; // count 4 -> 5
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadLength,
          "件数改ざん（長さ不整合）がBadLengthにならなかった");

    // (7) 件数が上限超。
    std::memcpy(bad, good, len);
    bad[8] = static_cast<uint8_t>(kMaxAnchors + 1);
    bad[9] = 0;
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadCount,
          "件数が上限超なのにBadCountにならなかった");

    // (8) 件数0。
    std::memcpy(bad, good, len);
    bad[8] = 0;
    bad[9] = 0;
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadCount,
          "件数0がBadCountにならなかった");

    // (9) enabled が 0/1 以外（CRCは通るが値としてありえない）。
    std::memcpy(bad, good, len);
    bad[uwb::cfg::kBlobHeaderSize + 2] = 0x7F;
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadEntry,
          "enabledが0/1以外なのにBadEntryにならなかった");

    // (10) 座標にNaNが入っている（CRCは通る）。ソルバへ渡す前に弾くこと。
    std::memcpy(bad, good, len);
    const uint8_t nanBits[4] = {0x00, 0x00, 0xC0, 0x7F}; // quiet NaN (LE)
    std::memcpy(bad + uwb::cfg::kBlobHeaderSize + 4, nanBits, 4);
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadEntry,
          "NaN座標がBadEntryにならなかった");

    // (11) 座標が桁違い（約 1.99e30 m）。CRCは通るが範囲外。
    std::memcpy(bad, good, len);
    const uint8_t hugeBits[4] = {0xCA, 0xF2, 0x49, 0x71}; // 1e30f (LE)
    std::memcpy(bad + uwb::cfg::kBlobHeaderSize + 4, hugeBits, 4);
    refreshCrc(bad, len);
    CHECK(uwb::cfg::deserializeAnchorTable(bad, len, out, kMaxAnchors, &count) == BlobStatus::BadEntry,
          "桁違いの座標がBadEntryにならなかった");

    // (12) アドレスのバイト列に 0xFFFF が入っている（CRCは通る）。
    uint8_t abuf[uwb::cfg::anchorAddrBlobSize()];
    size_t alen = 0;
    CHECK(uwb::cfg::serializeAnchorAddr(0x0003, abuf, sizeof(abuf), &alen) == BlobStatus::Ok,
          "アドレスのバイト列を作れなかった");
    abuf[uwb::cfg::kBlobHeaderSize + 0] = 0xFF;
    abuf[uwb::cfg::kBlobHeaderSize + 1] = 0xFF;
    refreshCrc(abuf, alen);
    CHECK(uwb::cfg::deserializeAnchorAddr(abuf, alen, &addr) == BlobStatus::BadEntry,
          "アドレス0xFFFFがBadEntryにならなかった");

    // (13) CRC-32 そのものの検算（既知ベクトル "123456789" -> 0xCBF43926）。
    const uint8_t vec[9] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    CHECK(uwb::cfg::crc32(vec, sizeof(vec)) == 0xCBF43926u, "CRC-32の既知ベクトルが合わない (0x%08lX)",
          static_cast<unsigned long>(uwb::cfg::crc32(vec, sizeof(vec))));

    // (14) ログ表示に使う名前が必ず返ること（nullptrを返さない）。
    const BlobStatus all[] = {BlobStatus::Ok,        BlobStatus::TooShort,  BlobStatus::BadMagic,
                              BlobStatus::BadVersion, BlobStatus::BadKind,   BlobStatus::BadCount,
                              BlobStatus::BadLength,  BlobStatus::BadCrc,    BlobStatus::BadEntry,
                              BlobStatus::NullArg};
    for (size_t i = 0; i < sizeof(all) / sizeof(all[0]); ++i) {
        CHECK(uwb::cfg::blobStatusName(all[i]) != nullptr, "blobStatusName()がnullptrを返した (i=%zu)", i);
    }
}

/* ==================================================================== *
 * 13. docs/TIMING_PRESETS.md §2 プリセット表の検算（タスクE-1）。
 *     uwb_qm33120_timing.hpp は ESP-IDF/Qorvo SDK 非依存なのでここから
 *     直接includeでき、実物の timingPresetSs()/timingPresetDs() を叩ける。
 * ==================================================================== */
static void scenario13_timing_preset_table()
{
    std::printf("--- 13. docs/TIMING_PRESETS.md §2 プリセット表の検算 ---\n");
    using uwb::TimingProfile;

    // §2.1 RangeConfig(SS-TWR)。
    {
        const uwb::TimingPresetSs p = uwb::timingPresetSs(TimingProfile::PollingBoth);
        CHECK(p.responseTxDelayUus == 3000 && p.responseRxAfterTxDelayUus == 500 && p.rxTimeoutUus == 4500,
              "SS PollingBothが表と違う (tx=%u rx=%u to=%u)", p.responseTxDelayUus, p.responseRxAfterTxDelayUus,
              p.rxTimeoutUus);
    }
    {
        const uwb::TimingPresetSs p = uwb::timingPresetSs(TimingProfile::AnchorIrq);
        CHECK(p.responseTxDelayUus == 878 && p.responseRxAfterTxDelayUus == 400 && p.rxTimeoutUus == 1200,
              "SS AnchorIrqが表と違う (tx=%u rx=%u to=%u)", p.responseTxDelayUus, p.responseRxAfterTxDelayUus,
              p.rxTimeoutUus);
    }
    {
        const uwb::TimingPresetSs p = uwb::timingPresetSs(TimingProfile::BothIrq);
        CHECK(p.responseTxDelayUus == 878 && p.responseRxAfterTxDelayUus == 400 && p.rxTimeoutUus == 1200,
              "SS BothIrqが表と違う (tx=%u rx=%u to=%u)", p.responseTxDelayUus, p.responseRxAfterTxDelayUus,
              p.rxTimeoutUus);
    }

    // §2.2 DSRangeConfig(DS-TWR)。
    {
        const uwb::TimingPresetDs p = uwb::timingPresetDs(TimingProfile::PollingBoth);
        CHECK(p.responseTxDelayUus == 3000 && p.responseRxAfterTxDelayUus == 1500 && p.finalTxDelayUus == 1800 &&
                  p.finalRxAfterResponseTxDelayUus == 500 && p.resultRxAfterFinalTxDelayUus == 200 &&
                  p.rxTimeoutUus == 3000,
              "DS PollingBothが表と違う (tx=%u rx=%u ftx=%u frx=%u rrx=%u to=%u)", p.responseTxDelayUus,
              p.responseRxAfterTxDelayUus, p.finalTxDelayUus, p.finalRxAfterResponseTxDelayUus,
              p.resultRxAfterFinalTxDelayUus, p.rxTimeoutUus);
    }
    {
        const uwb::TimingPresetDs p = uwb::timingPresetDs(TimingProfile::AnchorIrq);
        CHECK(p.responseTxDelayUus == 878 && p.responseRxAfterTxDelayUus == 400 && p.finalTxDelayUus == 1365 &&
                  p.finalRxAfterResponseTxDelayUus == 500 && p.resultRxAfterFinalTxDelayUus == 0 &&
                  p.rxTimeoutUus == 1200,
              "DS AnchorIrqが表と違う (tx=%u rx=%u ftx=%u frx=%u rrx=%u to=%u)", p.responseTxDelayUus,
              p.responseRxAfterTxDelayUus, p.finalTxDelayUus, p.finalRxAfterResponseTxDelayUus,
              p.resultRxAfterFinalTxDelayUus, p.rxTimeoutUus);
    }
    {
        const uwb::TimingPresetDs p = uwb::timingPresetDs(TimingProfile::BothIrq);
        CHECK(p.responseTxDelayUus == 878 && p.responseRxAfterTxDelayUus == 400 && p.finalTxDelayUus == 683 &&
                  p.finalRxAfterResponseTxDelayUus == 200 && p.resultRxAfterFinalTxDelayUus == 0 &&
                  p.rxTimeoutUus == 1200,
              "DS BothIrqが表と違う (tx=%u rx=%u ftx=%u frx=%u rrx=%u to=%u)", p.responseTxDelayUus,
              p.responseRxAfterTxDelayUus, p.finalTxDelayUus, p.finalRxAfterResponseTxDelayUus,
              p.resultRxAfterFinalTxDelayUus, p.rxTimeoutUus);
    }

    // その他のヘルパ。
    CHECK(std::strcmp(uwb::timingProfileName(TimingProfile::PollingBoth), "PollingBoth") == 0,
          "timingProfileName(PollingBoth)が違う");
    CHECK(std::strcmp(uwb::timingProfileName(TimingProfile::AnchorIrq), "AnchorIrq") == 0,
          "timingProfileName(AnchorIrq)が違う");
    CHECK(std::strcmp(uwb::timingProfileName(TimingProfile::BothIrq), "BothIrq") == 0,
          "timingProfileName(BothIrq)が違う");
    CHECK(std::strcmp(uwb::timingProfileName(static_cast<TimingProfile>(99)), "?") == 0,
          "timingProfileName(不正値)が\"?\"でない");

    CHECK(!uwb::timingProfileNeedsAnchorIrq(TimingProfile::PollingBoth), "needsAnchorIrq(PollingBoth)がtrue");
    CHECK(uwb::timingProfileNeedsAnchorIrq(TimingProfile::AnchorIrq), "needsAnchorIrq(AnchorIrq)がfalse");
    CHECK(uwb::timingProfileNeedsAnchorIrq(TimingProfile::BothIrq), "needsAnchorIrq(BothIrq)がfalse");
    CHECK(!uwb::timingProfileNeedsTagIrq(TimingProfile::PollingBoth), "needsTagIrq(PollingBoth)がtrue");
    CHECK(!uwb::timingProfileNeedsTagIrq(TimingProfile::AnchorIrq), "needsTagIrq(AnchorIrq)がtrue");
    CHECK(uwb::timingProfileNeedsTagIrq(TimingProfile::BothIrq), "needsTagIrq(BothIrq)がfalse");

    CHECK(uwb::timingProfileValid(0) && uwb::timingProfileValid(1) && uwb::timingProfileValid(2),
          "0..2がtimingProfileValid()でfalseになった");
    CHECK(!uwb::timingProfileValid(3), "3がtimingProfileValid()でtrueになった");
}

/* ==================================================================== *
 * 14. PollingBothがRangeConfig/DSRangeConfigの既定値と一致すること
 *     （タスクE-2）。
 *
 * 【重要な限界】RangeConfig/DSRangeConfig自体（uwb_qm33120_types.hpp）は
 * uwb_port.h経由でdriver/spi_master.h(ESP-IDF)を引き込むため、ホストからは
 * includeできない（payloadMatches()と同じ、本プロジェクトの既存の制約。
 * tools/test_pipeline/Makefileが types.hpp を include path に含めておらず、
 * -I$(QM33120_DIR)/include のみでビルドしていることからも分かる）。
 * したがってここでは構造体を実際にインスタンス化する代わりに、
 * uwb_qm33120_types.hppのメンバ初期化子の値をそのまま転記して比較する
 * （転記元の行番号を各値にコメントで添える。ズレたらどちらかを直す判断が
 * 要る）。
 * ==================================================================== */
static void scenario14_timing_preset_matches_struct_defaults()
{
    std::printf("--- 14. PollingBothがRangeConfig/DSRangeConfigの既定値と一致すること ---\n");

    // 出典: uwb_qm33120_types.hpp RangeConfig（responseRxAfterTxDelayUus:307,
    // responseTxDelayUus:310, rxTimeoutUus:313）。
    const uint32_t ssDefaultResponseTxDelayUus        = 3000;
    const uint32_t ssDefaultResponseRxAfterTxDelayUus = 500;
    const uint32_t ssDefaultRxTimeoutUus                = 4500;
    const uwb::TimingPresetSs ssPreset = uwb::timingPresetSs(uwb::TimingProfile::PollingBoth);
    CHECK(ssPreset.responseTxDelayUus == ssDefaultResponseTxDelayUus &&
              ssPreset.responseRxAfterTxDelayUus == ssDefaultResponseRxAfterTxDelayUus &&
              ssPreset.rxTimeoutUus == ssDefaultRxTimeoutUus,
          "SS PollingBothがRangeConfigの既定値と一致しない");

    // 出典: uwb_qm33120_types.hpp DSRangeConfig（responseRxAfterTxDelayUus:367,
    // responseTxDelayUus:371, finalTxDelayUus:375,
    // finalRxAfterResponseTxDelayUus:379, resultRxAfterFinalTxDelayUus:403,
    // rxTimeoutUus:407）。
    const uint32_t dsDefaultResponseTxDelayUus             = 3000;
    const uint32_t dsDefaultResponseRxAfterTxDelayUus      = 1500;
    const uint32_t dsDefaultFinalTxDelayUus                 = 1800;
    const uint32_t dsDefaultFinalRxAfterResponseTxDelayUus = 500;
    const uint32_t dsDefaultResultRxAfterFinalTxDelayUus   = 200;
    const uint32_t dsDefaultRxTimeoutUus                    = 3000;
    const uwb::TimingPresetDs dsPreset = uwb::timingPresetDs(uwb::TimingProfile::PollingBoth);
    CHECK(dsPreset.responseTxDelayUus == dsDefaultResponseTxDelayUus &&
              dsPreset.responseRxAfterTxDelayUus == dsDefaultResponseRxAfterTxDelayUus &&
              dsPreset.finalTxDelayUus == dsDefaultFinalTxDelayUus &&
              dsPreset.finalRxAfterResponseTxDelayUus == dsDefaultFinalRxAfterResponseTxDelayUus &&
              dsPreset.resultRxAfterFinalTxDelayUus == dsDefaultResultRxAfterFinalTxDelayUus &&
              dsPreset.rxTimeoutUus == dsDefaultRxTimeoutUus,
          "DS PollingBothがDSRangeConfigの既定値と一致しない");
}

/* ==================================================================== *
 * 15. docs/TIMING_PRESETS.md §1.2 締切式の検算（タスクE-3）。
 *     各プロファイルについて:
 *       responseRxAfterTxDelayUusの実us < responseTxDelayUusの実us - 179
 *       finalRxAfterResponseTxDelayUusの実us < finalTxDelayUusの実us - 179
 *     （実us換算 = uus * 512 / 499.2）
 * ==================================================================== */
namespace {
double uusToRealUsForDeadlineCheck(uint32_t uus)
{
    return static_cast<double>(uus) * 512.0 / 499.2;
}
} // namespace

static void scenario15_timing_deadline_formula()
{
    std::printf("--- 15. docs/TIMING_PRESETS.md §1.2 締切式の検算 ---\n");
    using uwb::TimingProfile;

    const TimingProfile profiles[] = {TimingProfile::PollingBoth, TimingProfile::AnchorIrq, TimingProfile::BothIrq};
    for (TimingProfile p : profiles) {
        const uwb::TimingPresetSs ss  = uwb::timingPresetSs(p);
        const double ssTxReal          = uusToRealUsForDeadlineCheck(ss.responseTxDelayUus);
        const double ssRxReal          = uusToRealUsForDeadlineCheck(ss.responseRxAfterTxDelayUus);
        CHECK(ssRxReal < (ssTxReal - 179.0),
              "SS %s: responseRxAfterTxDelayUus(実%.1fus)がresponseTxDelayUus-179(実%.1fus)以上",
              uwb::timingProfileName(p), ssRxReal, ssTxReal - 179.0);

        const uwb::TimingPresetDs ds     = uwb::timingPresetDs(p);
        const double dsRespTxReal         = uusToRealUsForDeadlineCheck(ds.responseTxDelayUus);
        const double dsRespRxReal         = uusToRealUsForDeadlineCheck(ds.responseRxAfterTxDelayUus);
        CHECK(dsRespRxReal < (dsRespTxReal - 179.0),
              "DS %s: responseRxAfterTxDelayUus(実%.1fus)がresponseTxDelayUus-179(実%.1fus)以上",
              uwb::timingProfileName(p), dsRespRxReal, dsRespTxReal - 179.0);

        const double dsFinalTxReal = uusToRealUsForDeadlineCheck(ds.finalTxDelayUus);
        const double dsFinalRxReal = uusToRealUsForDeadlineCheck(ds.finalRxAfterResponseTxDelayUus);
        CHECK(dsFinalRxReal < (dsFinalTxReal - 179.0),
              "DS %s: finalRxAfterResponseTxDelayUus(実%.1fus)がfinalTxDelayUus-179(実%.1fus)以上",
              uwb::timingProfileName(p), dsFinalRxReal, dsFinalTxReal - 179.0);
    }
}

/* ==================================================================== *
 * 16. applyTimingProfile()がプリセットに無いフィールドを壊さないこと
 *     （タスクE-4）。
 *
 * 【重要な限界】RangeConfig/DSRangeConfig/applyTimingProfile()自体は
 * ESP-IDF依存(uwb_qm33120_types.hppがuwb_port.h経由でdriver/spi_master.hを
 * 引き込む)のためホストからincludeできない（シナリオ14と同じ制約）。
 * ここではuwb_qm33120_types.hppのapplyTimingProfile()の実装
 * （*Uusフィールドへの代入のみ、他のフィールドには一切触れない）を
 * そのまま模した局所ミラーで検算する。実装がズレたら意味が無いので、
 * uwb_qm33120_types.hppのapplyTimingProfile()を変更したときは必ずここも
 * 見比べること。
 * ==================================================================== */
namespace {

struct MirrorRangeConfig {
    uint16_t panId                     = 0xDECA;
    uint16_t initiatorAddress          = 0x0001;
    uint16_t responderAddress          = 0x0002;
    uint32_t responseRxAfterTxDelayUus = 500;
    uint32_t responseTxDelayUus        = 3000;
    uint32_t rxTimeoutUus              = 4500;
    uint32_t hostTimeoutMs             = 10;
    bool enableClockOffsetCorrection    = true;
};

void mirrorApplyTimingProfile(MirrorRangeConfig& cfg, uwb::TimingProfile p)
{
    const uwb::TimingPresetSs preset = uwb::timingPresetSs(p);
    cfg.responseTxDelayUus            = preset.responseTxDelayUus;
    cfg.responseRxAfterTxDelayUus     = preset.responseRxAfterTxDelayUus;
    cfg.rxTimeoutUus                  = preset.rxTimeoutUus;
}

struct MirrorDsRangeConfig {
    uint16_t panId                          = 0xDECA;
    uint16_t initiatorAddress               = 0x0001;
    uint16_t responderAddress               = 0x0002;
    uint32_t responseRxAfterTxDelayUus      = 1500;
    uint32_t responseTxDelayUus             = 3000;
    uint32_t finalTxDelayUus                = 1800;
    uint32_t finalRxAfterResponseTxDelayUus = 500;
    uint32_t resultRxAfterFinalTxDelayUus   = 200;
    uint32_t rxTimeoutUus                   = 3000;
    uint32_t hostTimeoutMs                  = 10;
    uint8_t resultRepeatCount               = 1;
    uint32_t resultRepeatGapMs              = 3;
};

void mirrorApplyTimingProfile(MirrorDsRangeConfig& cfg, uwb::TimingProfile p)
{
    const uwb::TimingPresetDs preset   = uwb::timingPresetDs(p);
    cfg.responseTxDelayUus              = preset.responseTxDelayUus;
    cfg.responseRxAfterTxDelayUus       = preset.responseRxAfterTxDelayUus;
    cfg.finalTxDelayUus                 = preset.finalTxDelayUus;
    cfg.finalRxAfterResponseTxDelayUus  = preset.finalRxAfterResponseTxDelayUus;
    cfg.resultRxAfterFinalTxDelayUus    = preset.resultRxAfterFinalTxDelayUus;
    cfg.rxTimeoutUus                    = preset.rxTimeoutUus;
}

} // namespace

static void scenario16_apply_timing_profile_preserves_other_fields()
{
    std::printf("--- 16. applyTimingProfile()がプリセット外フィールドを壊さないこと ---\n");

    // --- RangeConfig(SS-TWR) ---
    {
        MirrorRangeConfig cfg;
        cfg.panId                    = 0x1234;
        cfg.initiatorAddress         = 0x5678;
        cfg.responderAddress         = 0x9ABC;
        cfg.hostTimeoutMs            = 42;
        cfg.enableClockOffsetCorrection = false;
        mirrorApplyTimingProfile(cfg, uwb::TimingProfile::AnchorIrq);

        CHECK(cfg.panId == 0x1234 && cfg.initiatorAddress == 0x5678 && cfg.responderAddress == 0x9ABC,
              "SS: panId/アドレスが変化した");
        CHECK(cfg.hostTimeoutMs == 42, "SS: hostTimeoutMsが変化した");
        CHECK(cfg.enableClockOffsetCorrection == false, "SS: enableClockOffsetCorrectionが変化した");
        CHECK(cfg.responseTxDelayUus == 878 && cfg.responseRxAfterTxDelayUus == 400 && cfg.rxTimeoutUus == 1200,
              "SS: プリセットが正しく適用されなかった (AnchorIrq)");
    }

    // --- DSRangeConfig(DS-TWR) ---
    {
        MirrorDsRangeConfig cfg;
        cfg.panId               = 0x1111;
        cfg.initiatorAddress    = 0x2222;
        cfg.responderAddress    = 0x3333;
        cfg.hostTimeoutMs       = 99;
        cfg.resultRepeatCount   = 7;
        cfg.resultRepeatGapMs   = 123;
        mirrorApplyTimingProfile(cfg, uwb::TimingProfile::BothIrq);

        CHECK(cfg.panId == 0x1111 && cfg.initiatorAddress == 0x2222 && cfg.responderAddress == 0x3333,
              "DS: panId/アドレスが変化した");
        CHECK(cfg.hostTimeoutMs == 99, "DS: hostTimeoutMsが変化した");
        CHECK(cfg.resultRepeatCount == 7, "DS: resultRepeatCountが変化した");
        CHECK(cfg.resultRepeatGapMs == 123, "DS: resultRepeatGapMsが変化した");
        CHECK(cfg.responseTxDelayUus == 878 && cfg.responseRxAfterTxDelayUus == 400 && cfg.finalTxDelayUus == 683 &&
                  cfg.finalRxAfterResponseTxDelayUus == 200 && cfg.resultRxAfterFinalTxDelayUus == 0 &&
                  cfg.rxTimeoutUus == 1200,
              "DS: プリセットが正しく適用されなかった (BothIrq)");
    }
}

/* ==================================================================== *
 * 17. payloadMatchesEither()/readTimingTag() の検算（タスクE-5）。
 *
 * 【重要な限界】これらの実体は uwb_qm33120_internal.hpp（src/、esp_timer.h /
 * deca_device_api.h に依存）にあり、既存の payloadMatches() 同様ホストから
 * includeできない。ここでは同ヘッダの実装（ヘッダ長9バイト+FCS長2バイト、
 * memcmpによる先頭一致、末尾2バイトの読み出し）をそのまま模した局所ミラーで
 * 検算する。実装がズレたら意味が無いので、uwb_qm33120_internal.hppの
 * payloadMatchesEither()/readTimingTag()を変更したときは必ずここも見比べる
 * こと。
 * ==================================================================== */
namespace {

constexpr size_t kMirrorHeaderLen = 9; // uwb_qm33120_internal.hpp kShortAddressHeaderLen
constexpr size_t kMirrorFcsLen    = 2; // deca_device_api.h FCS_LEN (=2UL)

uint16_t mirrorFrameLen(size_t payloadLen)
{
    return static_cast<uint16_t>(kMirrorHeaderLen + payloadLen + kMirrorFcsLen);
}

bool mirrorPayloadMatches(const uint8_t* frame, uint16_t frameLen, const char* payload, size_t prefixLength,
                           size_t expectedPayloadLength)
{
    return (frame != nullptr) && (payload != nullptr) && (prefixLength <= expectedPayloadLength) &&
           (frameLen == mirrorFrameLen(expectedPayloadLength)) &&
           (std::memcmp(&frame[kMirrorHeaderLen], payload, prefixLength) == 0);
}

bool mirrorPayloadMatchesEither(const uint8_t* frame, uint16_t frameLen, const char* payload, size_t prefixLength,
                                 size_t lenLegacy, size_t lenTagged)
{
    return mirrorPayloadMatches(frame, frameLen, payload, prefixLength, lenLegacy) ||
           mirrorPayloadMatches(frame, frameLen, payload, prefixLength, lenTagged);
}

bool mirrorReadTimingTag(const uint8_t* frame, uint16_t frameLen, size_t lenLegacy, uint8_t& version,
                          uint8_t& profile)
{
    const size_t lenTagged = lenLegacy + 2;
    if ((frame == nullptr) || (frameLen != mirrorFrameLen(lenTagged))) {
        return false;
    }
    version = frame[kMirrorHeaderLen + lenLegacy];
    profile = frame[kMirrorHeaderLen + lenLegacy + 1];
    return true;
}

} // namespace

static void scenario17_payload_matches_either_and_timing_tag()
{
    std::printf("--- 17. payloadMatchesEither()/readTimingTag() 検算（ミラー実装） ---\n");

    // 旧長(3バイトpayload、版情報なし)の "DWP" Poll。
    uint8_t legacy[14] = {0};
    std::memcpy(&legacy[kMirrorHeaderLen], "DWP", 3);
    CHECK(mirrorPayloadMatchesEither(legacy, sizeof(legacy), "DWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5),
          "旧長(payload3)のPollが受理されなかった");
    {
        uint8_t v = 0xAA, p = 0xAA;
        CHECK(!mirrorReadTimingTag(legacy, sizeof(legacy), 3, v, p),
              "旧長フレームがreadTimingTagでtrueになった");
    }

    // 新長(5バイトpayload、版/種別付き)の "DWP" Poll。
    uint8_t tagged[16] = {0};
    std::memcpy(&tagged[kMirrorHeaderLen], "DWP", 3);
    tagged[kMirrorHeaderLen + 3] = 7; // version
    tagged[kMirrorHeaderLen + 4] = 1; // profile = AnchorIrq(1)
    CHECK(mirrorPayloadMatchesEither(tagged, sizeof(tagged), "DWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5),
          "新長(payload5)のPollが受理されなかった");
    {
        uint8_t v = 0, p = 0;
        CHECK(mirrorReadTimingTag(tagged, sizeof(tagged), 3, v, p),
              "新長フレームがreadTimingTagでfalseになった");
        CHECK(v == 7 && p == 1, "version/profileの往復が一致しない (v=%u p=%u)", v, p);
    }

    // それ以外の長さ(旧/新のどちらでもない)は拒否する。
    uint8_t wrongLen[15] = {0};
    std::memcpy(&wrongLen[kMirrorHeaderLen], "DWP", 3);
    CHECK(!mirrorPayloadMatchesEither(wrongLen, sizeof(wrongLen), "DWP", 3, 3, 5),
          "旧(3)/新(5)どちらでもない長さが受理された");

    // 長さが合っていても関数コード(先頭3バイト)が違えば拒否する。
    uint8_t wrongCode[14] = {0};
    std::memcpy(&wrongCode[kMirrorHeaderLen], "XXX", 3);
    CHECK(!mirrorPayloadMatchesEither(wrongCode, sizeof(wrongCode), "DWP", 3, 3, 5),
          "関数コード不一致が受理された");
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
    scenario9_r2_frame_match();
    scenario10_cfgstore_roundtrip();
    scenario11_cfgstore_boundaries();
    scenario12_cfgstore_corruption();
    scenario13_timing_preset_table();
    scenario14_timing_preset_matches_struct_defaults();
    scenario15_timing_deadline_formula();
    scenario16_apply_timing_profile_preserves_other_fields();
    scenario17_payload_matches_either_and_timing_tag();

    std::printf("\n=== %d 件中 %d 件失敗 ===\n", g_run, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
