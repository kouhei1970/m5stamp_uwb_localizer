/**
 * @file uwb_ranging_anchor_table.cpp
 * @brief uwb::AnchorTable の実装。ハード非依存（uwb_loc.h のみに依存）。
 */
#include "uwb_ranging_anchor_table.hpp"

#include <cstdio>
#include <cstring>

namespace uwb {

AnchorTable::AnchorTable()
{
    // 台数0でも uwb_config_init() 自体は安全（n_anchors=0、anchors は
    // storage_ を指すだけで参照はされない）。呼び出し側が set() するまでの
    // 既定状態として素直に初期化しておく。
    uwb_config_init(&config_, storage_, 0);
    configInitialized_ = true;
}

bool AnchorTable::set(const AnchorEntry* entries, size_t count)
{
    if (entries == nullptr || count == 0 || count > kMaxAnchors) {
        return false;
    }

    for (size_t i = 0; i < count; ++i) {
        entries_[i] = entries[i];
    }
    count_ = count;

    rebuildStorage();
    return true;
}

bool AnchorTable::update(size_t index, const AnchorEntry& entry)
{
    if (index >= count_) {
        return false;
    }
    entries_[index] = entry;

    // config_.anchors (== storage_) の該当要素だけを書き換える。配列の
    // アドレスも n_anchors も変わらないので uwb_config_init() は不要
    // （ヘッダのコメント参照）。
    uwb_anchor& dst = storage_[index];
    std::snprintf(dst.id, UWB_ID_LEN, "A%04X", static_cast<unsigned>(entry.short_addr));
    dst.p[0]             = static_cast<uwb_real>(entry.pos[0]);
    dst.p[1]             = static_cast<uwb_real>(entry.pos[1]);
    dst.p[2]             = static_cast<uwb_real>(entry.pos[2]);
    dst.enabled          = entry.enabled ? 1 : 0;
    dst.antenna_delay_m  = static_cast<uwb_real>(entry.antenna_delay_m);
    dst.sigma0           = 0; // 0 => uwb_loc 側の既定 (0.1m) にフォールバック
    dst.sigma_per_m       = 0;
    // 座標 / enabled が変わったので、アンカー配置だけで決まる派生値
    // (同一平面判定のキャッシュ) を作り直す。呼ばなくても uwb_loc 側が
    // キャッシュの不一致を検出して毎回計算し直すので結果は変わらないが、
    // 毎 fix の鏡像処理で固有値計算が走ることになる。
    uwb_config_refresh(&config_);
    return true;
}

int AnchorTable::indexOfShortAddr(uint16_t short_addr) const
{
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].short_addr == short_addr) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

void AnchorTable::setDimension3D()
{
    config_.dim = 3;
}

void AnchorTable::setDimension2D(float zFixedM)
{
    config_.dim     = 2;
    config_.z_fixed = static_cast<uwb_real>(zFixedM);
}

PlacementCheck AnchorTable::checkPlacement(float originPlaneEpsM) const
{
    PlacementCheck result;

    uwb_real normal[3] = {0, 0, 0};
    uwb_real offset     = 0;
    result.coplanar = (uwb_anchors_coplanar(&config_, normal, &offset) != 0);
    if (!result.coplanar) {
        return result;
    }

    result.normal[0] = normal[0];
    result.normal[1] = normal[1];
    result.normal[2] = normal[2];
    result.offsetM    = offset;

    // 平面 n・p = offset がワールド原点 (0,0,0) を通るとき offset == 0。
    // normal は単位ベクトルなので offset はそのまま「原点から平面までの
    // 符号付き距離」[m] になる。
    const uwb_real eps = static_cast<uwb_real>(originPlaneEpsM);
    const uwb_real absOffset = (offset < 0) ? -offset : offset;
    result.originWarning = (absOffset <= eps);

    return result;
}

size_t AnchorTable::enabledCount() const
{
    size_t n = 0;
    for (size_t i = 0; i < count_; ++i) {
        if (entries_[i].enabled) {
            ++n;
        }
    }
    return n;
}

ModeDecision AnchorTable::evaluateMode()
{
    const PlacementCheck placement = checkPlacement();
    modeDecision_                    = decidePositioningMode(enabledCount(), placement, modeOverride_, zFixedM_);

    if (modeDecision_.mode == PositioningMode::Mode2D) {
        setDimension2D(zFixedM_);
    } else {
        // RangingOnly のときも dim=3 にしておく（測位そのものをしないので実害は
        // 無いが、次に有効台数が増えて自動的にMode2D/Mode3Dへ移るときの起点を
        // そろえておくため）。
        setDimension3D();
    }
    return modeDecision_;
}

void AnchorTable::rebuildStorage()
{
    uwb_config previous = config_;
    const bool hadPrevious = configInitialized_;

    for (size_t i = 0; i < count_; ++i) {
        uwb_anchor& dst          = storage_[i];
        const AnchorEntry& src   = entries_[i];
        std::snprintf(dst.id, UWB_ID_LEN, "A%04X", static_cast<unsigned>(src.short_addr));
        dst.p[0]             = static_cast<uwb_real>(src.pos[0]);
        dst.p[1]             = static_cast<uwb_real>(src.pos[1]);
        dst.p[2]             = static_cast<uwb_real>(src.pos[2]);
        dst.enabled          = src.enabled ? 1 : 0;
        dst.antenna_delay_m  = static_cast<uwb_real>(src.antenna_delay_m);
        dst.sigma0           = 0; // 0 => uwb_loc 側の既定 (0.1m) にフォールバック
        dst.sigma_per_m       = 0;
    }
    // 使わない残りのスロットもゼロクリアしておく（デバッグ時に古い値が
    // 残って混乱しないように。config_.n_anchors=count_ の範囲外なので
    // ソルバからは参照されないが、念のため）。
    for (size_t i = count_; i < kMaxAnchors; ++i) {
        storage_[i] = uwb_anchor{};
    }

    uwb_config_init(&config_, storage_, static_cast<int>(count_));

    if (hadPrevious) {
        // anchors/n_anchors 以外は直前の設定を引き継ぐ（setDimension2D() 等で
        // 変更済みのフィールドを uwb_config_init() の既定値で踏み潰さない）。
        const uwb_anchor* newAnchors = config_.anchors;
        const int newNAnchors        = config_.n_anchors;
        config_          = previous;
        config_.anchors   = newAnchors;
        config_.n_anchors = newNAnchors;
        // previous と一緒に古い派生値キャッシュ (config_.plane) も戻って
        // しまうので、新しいアンカー配置で作り直す。
        uwb_config_refresh(&config_);
    }
    configInitialized_ = true;
}

} // namespace uwb
