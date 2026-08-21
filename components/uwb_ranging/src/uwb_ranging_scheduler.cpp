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

RangingSample RangingScheduler::rangeOne(size_t anchorIndex)
{
    RangingSample sample;
    sample.anchor_index = anchorIndex;
    sample.ok             = false;

    // タスクF(docs/HANDOFF.md §5): 測距「開始」の直前に埋める。観測値
    // sample.distance_m が「タグがどこにいた時の距離か」を表すには、TWR
    // 往復（最大 elapsed_ms 程度）の間タグが動いていないと仮定する必要が
    // あり、その仮定の基準点は往復の開始時刻であるべきだから
    // （終了時刻を使うと、往復中の移動量ぶん系統的に遅れた時刻になる）。
    sample.t_us = esp_timer_get_time();

    const AnchorEntry& anchor = table_.entry(anchorIndex);

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

        const RangingSample sample = rangeOne(i);

        stats_[i].attempts++;
        if (sample.ok) {
            stats_[i].successes++;
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
