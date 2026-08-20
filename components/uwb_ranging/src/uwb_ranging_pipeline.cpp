/**
 * @file uwb_ranging_pipeline.cpp
 * @brief uwb::PositioningPipeline の実装。ハード非依存（uwb_loc.h のみに依存）。
 */
#include "uwb_ranging_pipeline.hpp"

namespace uwb {

PositioningPipeline::PositioningPipeline(AnchorTable& table, const PositioningConfig& cfg)
    : table_(table), cfg_(cfg)
{
}

size_t PositioningPipeline::buildMeasurements(const RangingSample* samples, size_t nSamples,
                                               size_t* usedAnchorIndexOut)
{
    size_t nValid = 0;
    for (size_t i = 0; i < nSamples && nValid < kMaxAnchors; ++i) {
        const RangingSample& s = samples[i];
        if (!s.ok) {
            continue; // 欠測（応答なし/タイムアウト）はスキップする
        }
        if (s.anchor_index >= table_.size()) {
            continue; // 登録テーブル範囲外は無視
        }
        if (!table_.entry(s.anchor_index).enabled) {
            continue; // 無効化されたアンカーは測位に使わない
        }

        uwb_meas& m = measBuf_[nValid];
        m.anchor    = static_cast<int>(s.anchor_index); // uwb_config.anchors の添字と一致させる
        m.value     = static_cast<uwb_real>(s.distance_m); // アンテナ遅延未補正のまま渡す
                                                              // (uwb_loc が anchor.antenna_delay_m で補正する)
        m.sigma     = 0;  // 0 => アンカーの sigma0/sigma_per_m から作られる
        m.quality   = -1; // 不明（品質情報を今のところ持たないため）

        usedAnchorIndexOut[nValid] = s.anchor_index;
        ++nValid;
    }
    return nValid;
}

void PositioningPipeline::fillResult(const uwb_fix& fix, const size_t* usedAnchorIndex, size_t nValid,
                                      SolverLevel level, PositionResult* out) const
{
    out->levelUsed = level;
    out->ok         = (fix.ok != 0);
    out->ambiguous  = (fix.ambiguous != 0);
    out->p[0]       = static_cast<float>(fix.p[0]);
    out->p[1]       = static_cast<float>(fix.p[1]);
    out->p[2]       = static_cast<float>(fix.p[2]);
    out->gdop        = static_cast<float>(fix.gdop);
    out->residualRms = static_cast<float>(fix.residual_rms);
    out->sigma        = static_cast<float>(fix.sigma);
    out->nUsed  = fix.n_used;
    out->nTotal = fix.n_total;

    // uwb_fix.excluded はソルバへ渡した観測配列（measBuf_）の添字基準。
    // 呼び出し側が扱いやすいよう、アンカーテーブルの添字基準へ変換する。
    out->excluded = 0;
    for (size_t i = 0; i < nValid && i < (sizeof(unsigned long) * 8); ++i) {
        if ((fix.excluded & (1UL << i)) != 0) {
            out->excluded |= (1UL << usedAnchorIndex[i]);
        }
    }
}

PositionResult PositioningPipeline::solve(const RangingSample* samples, size_t nSamples)
{
    return solve(samples, nSamples, cfg_.defaultLevel);
}

PositionResult PositioningPipeline::solve(const RangingSample* samples, size_t nSamples, SolverLevel level)
{
    PositionResult result;
    result.levelUsed = level;

    if (level == SolverLevel::Lv3) {
        // Lv3(EKF) は状態を持つので solve() ではなく updateEkf() を使うこと。
        result.solvable = false;
        return result;
    }

    size_t usedAnchorIndex[kMaxAnchors];
    const size_t nValid = buildMeasurements(samples, nSamples, usedAnchorIndex);
    result.nTotal        = static_cast<int>(nValid);

    const int minRequired = table_.isDimension2D() ? cfg_.minValid2d : cfg_.minValid3d;
    if (static_cast<int>(nValid) < minRequired) {
        // 欠測を常態として扱う: 有効測距が足りない周期は解を出さず「測位不能」を返す。
        result.solvable = false;
        result.ok         = false;
        return result;
    }
    result.solvable = true;

    uwb_fix fix{};
    if (level == SolverLevel::Lv0) {
        uwb_solve_lv0(&table_.config(), measBuf_, static_cast<int>(nValid), &fix);
    } else {
        uwb_solve_lv2(&table_.config(), measBuf_, static_cast<int>(nValid), &fix);
    }
    fillResult(fix, usedAnchorIndex, nValid, level, &result);
    return result;
}

void PositioningPipeline::initEkf(uwb_motion motion, float sigmaA)
{
    uwb_ekf_init(&ekf_, &table_.config(), motion, static_cast<uwb_real>(sigmaA));
    ekfInitialized_ = true;
}

void PositioningPipeline::resetEkf()
{
    uwb_ekf_reset(&ekf_);
}

void PositioningPipeline::predictEkf(float dtS)
{
    uwb_ekf_predict(&ekf_, static_cast<uwb_real>(dtS));
}

PositionResult PositioningPipeline::updateEkf(double tS, const RangingSample* samples, size_t nSamples)
{
    PositionResult result;
    result.levelUsed = SolverLevel::Lv3;

    size_t usedAnchorIndex[kMaxAnchors];
    const size_t nValid = buildMeasurements(samples, nSamples, usedAnchorIndex);
    result.nTotal         = static_cast<int>(nValid);

    if (!ekfInitialized_) {
        result.solvable = false;
        return result;
    }
    result.solvable = true;

    // Lv0/Lv2 と違い最小観測数のゲートは掛けない。0本でも uwb_ekf_update()
    // 内部で予測のみ進む（measBuf_ は常に非NULLの有効な配列なので安全）。
    uwb_fix fix{};
    uwb_ekf_update(&ekf_, static_cast<uwb_real>(tS), measBuf_, static_cast<int>(nValid), &fix);
    fillResult(fix, usedAnchorIndex, nValid, SolverLevel::Lv3, &result);
    return result;
}

} // namespace uwb
