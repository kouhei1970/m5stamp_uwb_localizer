/**
 * @file uwb_ranging_scheduler.hpp
 * @brief アンカー登録テーブルを順に回して測距する測距スケジューラ（ハード依存）。
 *
 * uwb_ranging_anchor_table.hpp / uwb_ranging_pipeline.hpp と異なり、本ファイルは
 * uwb::Qm33120（components/uwb_qm33120、ESP-IDF依存）を直接使う。ホストビルド
 * （tools/test_pipeline）はこのファイルをコンパイル対象に含めない。
 */
#pragma once

#include "uwb_qm33120.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

/** 測距方式。DS-TWRの方が精度が良いがAnchor側で距離計算するため往復が増える
 *  （PROGRESS.md Phase 2 の申し送り参照）。SS-TWRはTag側で完結し軽い。 */
enum class RangingMethod {
    SS, //!< requestRange() / respondRange()
    DS, //!< requestDSRange() / respondDSRange()
};

/** RangingScheduler の動作設定。 */
struct SchedulerConfig {
    RangingMethod method  = RangingMethod::DS;
    uint16_t panId         = 0xDECA;
    uint16_t tagShortAddr = 0x0001; //!< 自分（Tag/Initiator）のショートアドレス

    /** 同一周内で、あるアンカーへの測距を終えてから次のアンカーへ測距を
     *  始めるまでの最小間隔 [ms]。0 なら間を置かず連続実行する。 */
    uint32_t perAnchorIntervalMs = 0;

    /** 1周終えてから次の runCycle() 呼び出しが実際に測距を始めるまでの
     *  最小間隔 [ms]。0 なら間を置かない（呼び出し側のループ間隔がそのまま
     *  周期になる）。R6（更新レート）を詰めたいときは 0 にして、
     *  perAnchorIntervalMs も 0 にした上で lastCycleMs() の実測値を見ながら
     *  台数と方式(SS/DS)を調整する運用を想定。 */
    uint32_t cycleIntervalMs = 0;

    /** SS-TWR使用時の個別パラメータの既定値。initiatorAddress/responderAddress/
     *  panId は runCycle() が毎回アンカーごとに上書きするので、ここでは
     *  タイムアウト等のみ意味を持つ。 */
    RangeConfig ssDefaults;

    /** DS-TWR使用時の個別パラメータの既定値。ssDefaults 同様、アドレス系は
     *  runCycle() が上書きする。 */
    DSRangeConfig dsDefaults;
};

/**
 * @brief アンカー登録テーブルを順にポーリングする測距スケジューラ。
 *
 * 欠測（応答なし/タイムアウト）は常態として扱う: runCycle() は失敗した
 * アンカーをスキップして次へ進むだけで、エラー扱いにはしない。統計
 * （stats()）で成功率を、lastCycleMs() で1周の所要時間を確認できる
 * （R6: 更新レート設計の実測根拠）。
 */
class RangingScheduler {
public:
    RangingScheduler(Qm33120& radio, AnchorTable& table, const SchedulerConfig& cfg = SchedulerConfig());

    void setConfig(const SchedulerConfig& cfg) { cfg_ = cfg; }
    const SchedulerConfig& config() const { return cfg_; }

    /**
     * @brief アンカー登録テーブルを1周し、各アンカーへ測距する。
     *
     * 無効化（AnchorEntry::enabled==false）されたアンカーは試行自体しない
     * （統計にも計上しない）。有効なアンカーそれぞれについて、応答が
     * 得られなくても（RangingSample::ok==false のまま）次のアンカーへ進む。
     *
     * @param samplesOut 結果の書き込み先。呼び出し側が table.size() 件以上の
     *                   容量を用意すること。
     * @param maxSamples samplesOut の容量。
     * @return 実際に書き込んだ件数（有効化されているアンカーの数。
     *         maxSamples で頭打ちになる）。
     */
    size_t runCycle(RangingSample* samplesOut, size_t maxSamples);

    /** 直近の runCycle() 1回で、有効な全アンカーを1周ポーリングするのに
     *  要した時間 [ms]（cfg_.cycleIntervalMs による待ち合わせ分は含まない
     *  = 純粋なポーリング所要時間。R6の実効更新レートを知りたい場合は、
     *  呼び出し側で runCycle() の呼び出し時刻そのものの差分を取ること）。 */
    uint32_t lastCycleMs() const { return lastCycleMs_; }

    /** anchorIndex 番目のアンカーの測距成功率統計。 */
    const AnchorStats& stats(size_t anchorIndex) const;

    /** 全アンカーの統計をゼロに戻す。 */
    void resetStats();

private:
    Qm33120& radio_;
    AnchorTable& table_;
    SchedulerConfig cfg_;
    AnchorStats stats_[kMaxAnchors];
    uint32_t lastCycleMs_       = 0;
    uint32_t lastCycleStartMs_  = 0;
    bool hasLastCycleStart_      = false;

    /** 1台ぶんの測距を実行し、RangingSample を埋める。 */
    RangingSample rangeOne(size_t anchorIndex);
};

} // namespace uwb
