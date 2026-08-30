/**
 * @file uwb_ranging_scheduler.cpp
 * @brief uwb::RangingScheduler の実装（ハード依存、ESP-IDF/FreeRTOS を使う）。
 *
 * アンカー登録テーブル（AnchorTable）を順にポーリングし、1台ぶんの測距ごとに
 * Qm33120::requestRange() / requestDSRange() を呼ぶ。欠測は常態として扱い、
 * ここではエラーログを出さない（欠測をどう扱うかは呼び出し側 firmware の責務）。
 */
#include "uwb_ranging_scheduler.hpp"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

namespace uwb {

namespace {

/**
 * @brief 現在時刻を ms 単位で返す（firmware/twr の main.cpp と同じ idiom）。
 *
 * uint32_t 同士の引き算はオーバーフロー時も 32bit wraparound で自然に
 * 正しい差分になるため、経過時間の計算はこの値をそのまま引き算するだけでよい。
 */
uint32_t nowMs()
{
    return static_cast<uint32_t>(esp_timer_get_time() / 1000);
}

} // namespace

RangingScheduler::RangingScheduler(Qm33120& radio, AnchorTable& table, const SchedulerConfig& cfg)
    : radio_(radio), table_(table), cfg_(cfg)
{
}

RangingSample RangingScheduler::rangeOne(size_t anchorIndex, uint32_t& attemptsUsed)
{
    RangingSample sample;
    sample.anchor_index = anchorIndex;
    sample.ok             = false;
    attemptsUsed          = 0;

    const AnchorEntry& anchor = table_.entry(anchorIndex);

    // attempt==0: 最初の試行。attempt>=1: 再試行（最大cfg_.retryMax回）。
    // 失敗した最初の試行の後だけ、cfg_.retryDelayMsぶん待ってから打ち直す
    // 根拠はSchedulerConfig::retryDelayMsのフィールドコメント参照
    // （DS-TWRはアンカーのFinal待ち窓のため≥10ms必須、SS-TWRは不要）。
    // attempt==0: the first attempt. attempt>=1: a retry (up to
    // cfg_.retryMax times). Only after a failed attempt do we wait
    // cfg_.retryDelayMs before firing the retry's Poll - see the
    // SchedulerConfig::retryDelayMs field comment for why (DS-TWR needs
    // >=10ms because of the ANCHOR's Final-wait window; SS-TWR does not).
    for (uint8_t attempt = 0; attempt <= cfg_.retryMax; ++attempt) {
        if (attempt > 0 && cfg_.retryDelayMs > 0) {
            // pdMS_TO_TICKS() は CONFIG_FREERTOS_HZ 次第で切り捨てられ得る
            // （例: 100Hzなら1tick=10msで、10ms未満の指定は0tickになり得る）。
            // 待ちを指定したのに0tickになって「待ちなし」と等価になるのを
            // 避けるため、最低1tickは待つ。
            // pdMS_TO_TICKS() truncates depending on CONFIG_FREERTOS_HZ
            // (e.g. at 100Hz, 1 tick = 10ms, so a delay under 10ms would
            // round down to 0 ticks). Wait at least 1 tick whenever a
            // delay was requested, so it never silently becomes "no wait".
            const TickType_t delayTicks = pdMS_TO_TICKS(cfg_.retryDelayMs);
            vTaskDelay((delayTicks > 0) ? delayTicks : 1);
        }

        // タスクF(docs/HANDOFF.md §5): 測距「開始」の直前に埋める。観測値
        // sample.distance_m が「タグがどこにいた時の距離か」を表すには、TWR
        // 往復（最大 elapsed_ms 程度）の間タグが動いていないと仮定する必要が
        // あり、その仮定の基準点は往復の開始時刻であるべきだから
        // （終了時刻を使うと、往復中の移動量ぶん系統的に遅れた時刻になる）。
        // 再試行のたびに書き直す — 返すsampleは「実際に採用した試行」の
        // 開始時刻を持つべきだから。
        sample.t_us = esp_timer_get_time();

        if (cfg_.method == RangingMethod::DS) {
            DSRangeConfig cfg           = cfg_.dsDefaults;
            cfg.panId                    = cfg_.panId;
            cfg.initiatorAddress         = cfg_.tagShortAddr;
            cfg.responderAddress         = anchor.short_addr;

            const DSRangeResult result = radio_.requestDSRange(cfg);

            // distanceM は uwb_qm33120 側で既に m 換算済みの値。distanceMm から
            // 二重に変換しないよう、そのまま使う。
            sample.ok           = result.success;
            sample.distance_m   = result.distanceM;
            sample.elapsed_ms   = result.elapsedMs;
        } else {
            RangeConfig cfg             = cfg_.ssDefaults;
            cfg.panId                    = cfg_.panId;
            cfg.initiatorAddress         = cfg_.tagShortAddr;
            cfg.responderAddress         = anchor.short_addr;

            const RangeResult result = radio_.requestRange(cfg);

            sample.ok           = result.success;
            sample.distance_m   = result.distanceM;
            sample.elapsed_ms   = result.elapsedMs;
        }

        attemptsUsed = static_cast<uint32_t>(attempt) + 1;

        if (sample.ok) {
            // このサイクルで最初に成功した時点で打ち切り、これ以上再試行しない。
            // First success in this cycle - stop, no more retries needed.
            break;
        }
    }

    return sample;
}

size_t RangingScheduler::runCycle(RangingSample* samplesOut, size_t maxSamples)
{
    // 前回の周の開始時刻から cfg_.cycleIntervalMs 経つまで待つ（初回は待たない。
    // 既に経過済みなら待たない）。
    if (cfg_.cycleIntervalMs > 0 && hasLastCycleStart_) {
        const uint32_t elapsed = nowMs() - lastCycleStartMs_;
        if (elapsed < cfg_.cycleIntervalMs) {
            vTaskDelay(pdMS_TO_TICKS(cfg_.cycleIntervalMs - elapsed));
        }
    }

    // ここから「周の開始時刻」を更新し、実測時間の計測を始める
    // （上の待ち時間は lastCycleMs() に含めない）。
    lastCycleStartMs_    = nowMs();
    hasLastCycleStart_    = true;
    const uint32_t cycleWorkStartMs = nowMs();

    size_t n = 0;
    bool hasLastAnchorStartMs = false;
    uint32_t lastAnchorStartMs = 0;

    for (size_t i = 0; i < table_.size(); ++i) {
        if (!table_.entry(i).enabled) {
            // 無効化されたアンカーは測距を試みない。統計にも計上しない。
            continue;
        }

        // 2台目以降のみ、直前の測距開始から perAnchorIntervalMs 経つまで待つ。
        if (hasLastAnchorStartMs && cfg_.perAnchorIntervalMs > 0) {
            const uint32_t elapsed = nowMs() - lastAnchorStartMs;
            if (elapsed < cfg_.perAnchorIntervalMs) {
                vTaskDelay(pdMS_TO_TICKS(cfg_.perAnchorIntervalMs - elapsed));
            }
        }

        lastAnchorStartMs    = nowMs();
        hasLastAnchorStartMs = true;

        // rangeOne() は内部で cfg_.retryMax 回まで再試行する（待ち時間・
        // 追加の無線試行を含む）。その所要時間は lastAnchorStartMs の基準点
        // （このアンカーの測距開始）から次のアンカーの perAnchorIntervalMs
        // 待ち判定、および下のlastCycleMs_の両方に自然に含まれる — 周の
        // 内側で起きるループなので、ここで特別な対応は要らない。
        // rangeOne() retries internally up to cfg_.retryMax times
        // (including the wait and any extra radio attempts). That time is
        // automatically folded into both the next anchor's
        // perAnchorIntervalMs wait (measured from lastAnchorStartMs) and
        // lastCycleMs_ below - retries happen inside the cycle, so no
        // special-casing is needed here.
        uint32_t attemptsUsed       = 0;
        const RangingSample sample = rangeOne(i, attemptsUsed);

        // attempts/successes はサイクル単位（このアンカーへ runCycle() が
        // 試みた回数）のまま数える。再試行の回数と、再試行で救済された
        // サイクル数は別フィールドに積む（uwb_ranging_types.hpp
        // AnchorStats のフィールドコメント参照）。
        // attempts/successes stay per-cycle counters (how many runCycle()
        // calls tried this anchor). Retry counts and rescued cycles are
        // tallied into separate fields (see the AnchorStats field
        // comments in uwb_ranging_types.hpp).
        stats_[i].attempts++;
        if (sample.ok) {
            stats_[i].successes++;
        }
        stats_[i].retries += (attemptsUsed - 1);
        if (sample.ok && attemptsUsed > 1) {
            stats_[i].rescued++;
        }

        if (n < maxSamples) {
            samplesOut[n++] = sample;
        }
    }

    // 純粋なポーリング所要時間（ステップ2の待ち時間は含まない）。
    lastCycleMs_ = nowMs() - cycleWorkStartMs;

    return n;
}

const AnchorStats& RangingScheduler::stats(size_t anchorIndex) const
{
    // 範囲外アクセスを避けるため kMaxAnchors-1 にクランプする。
    const size_t clamped = (anchorIndex >= kMaxAnchors) ? (kMaxAnchors - 1) : anchorIndex;
    return stats_[clamped];
}

void RangingScheduler::resetStats()
{
    for (size_t i = 0; i < kMaxAnchors; ++i) {
        stats_[i] = AnchorStats{};
    }
}

} // namespace uwb
