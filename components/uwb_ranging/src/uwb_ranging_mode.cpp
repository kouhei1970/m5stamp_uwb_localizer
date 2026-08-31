/**
 * @file uwb_ranging_mode.cpp
 * @brief uwb_ranging_mode.hpp の実装。ハード非依存。
 */
#include "uwb_ranging_mode.hpp"

namespace uwb {

const char* positioningModeName(PositioningMode mode)
{
    switch (mode) {
    case PositioningMode::RangingOnly:
        return "ranging";
    case PositioningMode::Mode2D:
        return "2d";
    case PositioningMode::Mode3D:
        return "3d";
    }
    return "unknown";
}

const char* modeReasonText(ModeReason reason)
{
    switch (reason) {
    case ModeReason::TooFewAnchors:
        return "有効アンカーが2台以下のため測距のみ（測位しません）";
    case ModeReason::ThreeAnchors:
        return "有効アンカーが3台のため2D測位（高さ固定）";
    case ModeReason::CoplanarFallback:
        return "有効アンカーが同一平面上にあるため2D測位（高さ固定）へフォールバック";
    case ModeReason::Volumetric:
        return "有効アンカーが4台以上・立体配置のため3D測位";
    case ModeReason::ForcedTwoD:
        return "手動設定により2D測位を強制";
    case ModeReason::ForcedThreeD:
        return "手動設定により3D測位を強制";
    }
    return "unknown";
}

ModeDecision decidePositioningMode(size_t enabledCount, const PlacementCheck& placement, ModeOverride override,
                                    float zFixedM)
{
    ModeDecision d;
    d.override     = override;
    d.enabledCount = enabledCount;
    d.coplanar     = placement.coplanar;
    d.zFixedM      = zFixedM;
    d.placement    = placement;

    if (override == ModeOverride::Force2D) {
        d.mode   = PositioningMode::Mode2D;
        d.reason = ModeReason::ForcedTwoD;
        return d;
    }
    if (override == ModeOverride::Force3D) {
        d.mode                  = PositioningMode::Mode3D;
        d.reason                = ModeReason::ForcedThreeD;
        d.forcedCoplanarWarning = placement.coplanar;
        return d;
    }

    // override == Auto: 台数と配置だけで決める。
    if (enabledCount <= 2) {
        d.mode   = PositioningMode::RangingOnly;
        d.reason = ModeReason::TooFewAnchors;
    } else if (enabledCount == 3) {
        d.mode   = PositioningMode::Mode2D;
        d.reason = ModeReason::ThreeAnchors;
    } else if (placement.coplanar) {
        d.mode   = PositioningMode::Mode2D;
        d.reason = ModeReason::CoplanarFallback;
    } else {
        d.mode   = PositioningMode::Mode3D;
        d.reason = ModeReason::Volumetric;
    }
    return d;
}

} // namespace uwb
