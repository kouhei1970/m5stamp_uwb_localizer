/**
 * @file uwb_ranging_scheduler.hpp
 * @brief アンカー登録テーブルを順に回して測距する測距スケジューラ（ハード依存）。
 *
 * uwb_ranging_anchor_table.hpp / uwb_ranging_pipeline.hpp と異なり、本ファイルは
 * uwb::Qm33120（components/uwb_qm33120、ESP-IDF依存）を直接使う。ホストビルド
 * （tests/host/pipeline）はこのファイルをコンパイル対象に含めない。
 */
#pragma once

#include "uwb_qm33120.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

/** 測距方式。DS-TWRの方が精度が良いがAnchor側で距離計算するため往復が増える
 *  （docs/archive/PROGRESS.md Phase 2 の申し送り参照）。SS-TWRはTag側で完結し軽い。 */
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

    /** 1台のアンカーへの測距が失敗したとき、同じ周内で即座に許す追加試行の
     *  回数。0なら1回試行して失敗ならそのままスキップする（従来の挙動）。
     *  1台あたりの最大試行回数は 1+retryMax になる。firmware/twr での実測
     *  （UWB_TWR_RETRY_MAX=2、docs/HANDOFF.md §0-C「再試行系列」「再試行の
     *  待ち時間」）でSS-TWR/DS-TWRとも周期成功率99.9%まで上がることを
     *  確認済み。 */
    uint8_t retryMax = 0;

    /** 再試行1回ごとに、そのPoll送信前に待つ時間 [ms]。retryMax==0のときは
     *  使われない。
     *
     *  DS-TWRでは**10ms以上が必須**: アンカーは自身のResponse送信後、次の
     *  Finalを待つ窓（850kbps/256・PollingBothプリセットでResponse遅延
     *  3000UUS + W4R（Wait-for-Response）1500UUS + RXタイムアウト3000UUS、
     *  合計するとPoll受信からおよそ+8.2msまで）の中にいる間、再試行の
     *  Pollを聞いていない（アンカー側は`final_wait error=RX_ERROR`として
     *  記録する）。即時再試行（0ms、固定間隔なし）だと2回目の試行の失敗率
     *  が39〜44%だったのに対し、10ms待つと12%まで下がり、周期成功率は
     *  96%台から99.9〜100%まで上がった（docs/HANDOFF.md §0-C「再試行の
     *  待ち時間」e37〜e41、RETRY_MAX=2実測）。
     *
     *  SS-TWRでは待ちは不要: Response待ち失敗の時点でアンカーは既にPoll
     *  待ちへ戻っているため、2msの短い間隔だけで2回目の失敗率7.5%
     *  （独立試行相当。同節e34）。 */
    uint32_t retryDelayMs = 0;

    /** SS-TWR使用時の個別パラメータの既定値。initiatorAddress/responderAddress/
     *  panId は runCycle() が毎回アンカーごとに上書きするので、ここでは
     *  タイムアウト等のみ意味を持つ。 */
    RangeConfig ssDefaults;

    /** DS-TWR使用時の個別パラメータの既定値。ssDefaults 同様、アドレス系は
     *  runCycle() が上書きする。 */
    DSRangeConfig dsDefaults;
};

/**
 * @brief 測距1本が成功するたびに呼ばれるフック（逐次EKF観測更新のための
 * 差し込み口）。
 *
 * runCycle() が rangeOne() から RangingSample::ok==true を受け取った
 * **直後、同じタスク文脈で同期的に**呼ぶ（新しいタスク・キューは作らない）。
 * uwb_ekf（Lv3）はスレッド安全でないため（uwb_ranging_pipeline.hpp
 * PositioningPipeline冒頭コメント参照）、測距を回すタスクの外へ非同期に
 * 持ち出さずにこの場で処理し切る設計にしてある。失敗した測距
 * （RangingSample::ok==false）では呼ばれない。
 *
 * uwb_net.hpp の StatusJsonFn と同じ「関数ポインタ + void* コンテキスト」の
 * 作法（std::function は使わない。ハード依存側は組み込み向けに軽量な
 * 呼び出し規約で揃えてある）。user には setSampleHook() に渡したポインタが
 * そのまま渡る。
 */
using RangingSampleHook = void (*)(const RangingSample& sample, void* user);

/**
 * @brief アンカー登録テーブルを順にポーリングする測距スケジューラ。
 *
 * 欠測（応答なし/タイムアウト）は常態として扱う: 最初の試行が失敗した
 * アンカーは、cfg_.retryMax回まで同じ周内で即座に再試行し（各試行の前に
 * cfg_.retryDelayMsだけ待つ。DS-TWRでの根拠はSchedulerConfig::retryDelayMs
 * のフィールドコメント参照）、それでも失敗したらそのままスキップして次へ
 * 進むだけで、エラー扱いにはしない。統計（stats()）で成功率・再試行回数・
 * 再試行で救済された回数を、lastCycleMs() で1周の所要時間を確認できる
 * （R6: 更新レート設計の実測根拠。再試行に要した時間もlastCycleMs()に
 * 含まれる — rangeOne()内の待ち・再試行は周の内側で起きるため）。
 */
class RangingScheduler {
public:
    RangingScheduler(Qm33120& radio, AnchorTable& table, const SchedulerConfig& cfg = SchedulerConfig());

    void setConfig(const SchedulerConfig& cfg) { cfg_ = cfg; }
    const SchedulerConfig& config() const { return cfg_; }

    /**
     * @brief 測距1本ごとのフック（RangingSampleHook）を登録する。
     *
     * fn==nullptr なら解除（既定は未登録＝呼ばれない）。呼び出し側
     * （uwb::RangingService）が起動時に1回だけ登録する想定。フック自体は
     * runCycle() を呼んでいるタスクからしか呼ばれないので、登録・解除も
     * 同じタスクから行うこと（他タスクから触るなら呼び出し側で排他すること。
     * 本クラス自体は排他しない）。
     */
    void setSampleHook(RangingSampleHook fn, void* user = nullptr)
    {
        sampleHook_     = fn;
        sampleHookUser_ = user;
    }

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

    RangingSampleHook sampleHook_     = nullptr; //!< setSampleHook() 参照。既定は未登録
    void* sampleHookUser_               = nullptr;

    /** 1台ぶんの測距を実行し、RangingSample を埋める。cfg_.retryMax回までは
     *  内部で再試行し（cfg_.retryDelayMsぶん待ってから）、最初に成功した
     *  時点で打ち切る。実際に使った試行回数（1以上、再試行なしなら1）を
     *  attemptsUsed へ書き戻す。 */
    RangingSample rangeOne(size_t anchorIndex, uint32_t& attemptsUsed);
};

} // namespace uwb
