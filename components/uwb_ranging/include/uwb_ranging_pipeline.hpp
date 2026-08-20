/**
 * @file uwb_ranging_pipeline.hpp
 * @brief 測距結果から測位までを担う測位パイプライン（ハード非依存）。
 *
 * ESP-IDF/FreeRTOS には依存しない（uwb_loc.h と uwb_ranging_anchor_table.hpp
 * のみに依存）。ホスト（tools/test_pipeline）でもそのままコンパイル・実行
 * できるので、実機無しでの合成データ検証はこのクラスを直接叩けばよい。
 *
 * 呼び出し順の想定:
 *   1. 起動時に AnchorTable::checkPlacement() を確認（同一平面/原点通過の警告）
 *   2. 毎周期、スケジューラ（uwb_ranging_scheduler、ハード依存）が
 *      RangingSample[] を作る
 *   3. solve() に渡して PositionResult を得る（Lv0は初期値確認用、Lv2が本番）
 *   4. 移動体追従が要るなら updateEkf() を毎周期呼ぶ（Lv3）
 */
#pragma once

#include "uwb_loc.h"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

class PositioningPipeline {
public:
    explicit PositioningPipeline(AnchorTable& table, const PositioningConfig& cfg = PositioningConfig());

    void setConfig(const PositioningConfig& cfg) { cfg_ = cfg; }
    const PositioningConfig& config() const { return cfg_; }

    AnchorTable& table() { return table_; }
    const AnchorTable& table() const { return table_; }

    /**
     * @brief 1周期ぶんの測距結果から位置を解く（config().defaultLevel を使う）。
     *
     * 有効測距数（samples のうち ok==true かつ有効なアンカーを指すもの）が
     * 閾値未満（3D時4件、2D時3件。PositioningConfig::minValid3d/minValid2d）
     * なら、ソルバを呼ばずに「測位不能」（solvable=false, ok=false）を返す。
     */
    PositionResult solve(const RangingSample* samples, size_t nSamples);

    /**
     * @brief solve() のレベル指定版。level は Lv0 または Lv2 のみ対応
     * （Lv3 は状態を持つので updateEkf() を使うこと。level に Lv3 を渡した
     * 場合は何もせず solvable=false を返す）。
     */
    PositionResult solve(const RangingSample* samples, size_t nSamples, SolverLevel level);

    // ------------------------------------------------------------ Lv3 (EKF)
    //
    // **重要: uwb_ekf はスレッド安全でない**（内部に x/P/pending[] 等の
    // 可変状態を持つ）。initEkf/resetEkf/predictEkf/updateEkf は
    // **必ず単一タスクから呼ぶこと**。Lv0/Lv2 (solve()) は無状態の純関数
    // なので、こちらは複数タスクから同時に呼んでも問題ない。

    /** EKFを初期化する。呼び出し前に table に有効なアンカーが
     *  登録されていること。 */
    void initEkf(uwb_motion motion = UWB_MOTION_CV, float sigmaA = 0.5f);

    /** 状態を破棄して初期化直後に戻す（設定は保つ）。 */
    void resetEkf();

    bool ekfInitialized() const { return ekfInitialized_; }

    /** 観測が届かない間、dtS 秒ぶん予測だけ進める。 */
    void predictEkf(float dtS);

    /**
     * @brief 観測を入れてEKFを更新する。tS は単調増加の秒。
     *
     * Lv0/Lv2 と異なり最小観測数のゲートは掛けない（0本でもよい。その場合は
     * 内部で予測のみ進む）。initEkf() 未実行なら solvable=false を返す。
     */
    PositionResult updateEkf(double tS, const RangingSample* samples, size_t nSamples);

private:
    AnchorTable& table_;
    PositioningConfig cfg_;
    uwb_meas measBuf_[kMaxAnchors]{};

    uwb_ekf ekf_{};
    bool ekfInitialized_ = false;

    /**
     * @brief samples のうち有効なものだけを measBuf_ に詰める。
     * @param usedAnchorIndexOut measBuf_[i] が指すアンカーテーブル添字を書く
     *        （呼び出し側が kMaxAnchors 分の領域を用意しておくこと。excluded
     *        ビットマスクをアンカー添字基準へ変換するのに使う）。
     * @return 詰めた件数（＝有効測距数）。
     */
    size_t buildMeasurements(const RangingSample* samples, size_t nSamples, size_t* usedAnchorIndexOut);

    /** uwb_fix から PositionResult へ、excluded のビットマスク変換込みで詰め替える。 */
    void fillResult(const uwb_fix& fix, const size_t* usedAnchorIndex, size_t nValid, SolverLevel level,
                     PositionResult* out) const;
};

} // namespace uwb
