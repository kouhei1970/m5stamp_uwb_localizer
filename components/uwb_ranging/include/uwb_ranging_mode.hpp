/**
 * @file uwb_ranging_mode.hpp
 * @brief アンカー台数・配置・手動オーバーライドから測位モードを決めるロジック
 * （ハード非依存）。
 *
 * タグは登録済みアンカーの「有効」台数と配置（同一平面か立体か）だけで、
 * 実際に解ける測位モードを自動的に切り替える:
 *
 *   有効台数 <= 2            : RangingOnly（測距のみ、測位しない。
 *                              PositioningConfig::minValid2d=3未満のため）
 *   有効台数 == 3             : Mode2D（3点は必ず同一平面になるので3D測位はできない）
 *   有効台数 >= 4・同一平面   : Mode2D（3D測位が原理的に不安定/失敗するため
 *                              高さ固定へフォールバック。警告ログの対象）
 *   有効台数 >= 4・立体配置   : Mode3D
 *
 * 手動オーバーライド（ModeOverride::Force2D/Force3D）を指定すると、上の
 * 自動判定より常に優先される（firmware/tag の `mode` コンソールコマンド）。
 * 強制3Dで実際には同一平面だった場合は ModeDecision::forcedCoplanarWarning
 * が立つので、呼び出し側は ambiguous フラグに注意するよう警告できる。
 *
 * ESP-IDF/FreeRTOS には依存しない（uwb_ranging_types.hpp のみに依存）ので、
 * ホスト（tests/host/pipeline）でもそのままコンパイル・実行できる。
 */
#pragma once

#include <cstddef>

#include "uwb_ranging_types.hpp"

namespace uwb {

/** 実際に使う測位モード。 */
enum class PositioningMode {
    RangingOnly, //!< 測距のみ。ソルバ/EKFを呼ばない（有効アンカー2台以下）
    Mode2D,      //!< 2D測位（dim=2、高さ固定）
    Mode3D,      //!< 3D測位（dim=3）
};

/** 手動オーバーライド。console の `mode` コマンドで切り替える。既定 Auto。 */
enum class ModeOverride {
    Auto,    //!< 台数・配置から自動判定する（既定）
    Force2D, //!< 常に2D測位を使う
    Force3D, //!< 常に3D測位を使う
};

/** モードを決めた理由。ログ・console の `mode` 表示に使う。 */
enum class ModeReason {
    TooFewAnchors,     //!< auto: 有効台数<=2
    ThreeAnchors,      //!< auto: 有効台数==3
    CoplanarFallback,  //!< auto: 有効台数>=4だが同一平面
    Volumetric,        //!< auto: 有効台数>=4かつ立体配置
    ForcedTwoD,        //!< 手動: mode 2d
    ForcedThreeD,      //!< 手動: mode 3d
};

/** PositioningMode の短い名前（"ranging"/"2d"/"3d"）。JSON の "mode" フィールド、
 *  console 表示の両方で使う。**Web ダッシュボード側とのインターフェース契約
 *  なので綴りを変えないこと**（firmware/tag/main/main.cpp の
 *  printFixLine()/printAnchorsLine() 参照）。 */
const char* positioningModeName(PositioningMode mode);

/** ModeReason の人間可読の説明（日本語）。ログ・console `mode` コマンド用。 */
const char* modeReasonText(ModeReason reason);

/**
 * @brief モード判定の結果。AnchorTable::evaluateMode() が返し、以後
 * AnchorTable::modeDecision() でも同じ値を参照できる。
 */
struct ModeDecision {
    PositioningMode mode = PositioningMode::RangingOnly;
    ModeReason reason      = ModeReason::TooFewAnchors;
    ModeOverride override = ModeOverride::Auto; //!< 判定に使ったオーバーライド設定
    size_t enabledCount    = 0;                  //!< 有効アンカー台数
    bool coplanar            = false;             //!< 有効アンカーが同一平面か
                                                    //!< （enabledCount<3のときは常にfalse）
    bool forcedCoplanarWarning = false; //!< 強制3D(ForcedThreeD)なのに実際は同一平面だった場合true
    float zFixedM               = 0.0f;  //!< mode==Mode2D のときに使われた固定高さ [m]
    PlacementCheck placement{};          //!< checkPlacement() の生の結果（ログの詳細表示用）
};

/**
 * @brief 有効アンカー台数・配置・手動オーバーライドから測位モードを決める
 * （純粋関数。副作用なし。AnchorTable::evaluateMode() から呼ぶほか、
 * ホストテストから直接呼んで判定ロジックだけを検証できる）。
 *
 * @param enabledCount 有効アンカー台数（AnchorTable::enabledCount()）
 * @param placement    AnchorTable::checkPlacement() の結果
 *                      （enabledCount<3のときはcoplanar=falseでよい。
 *                      plane_compute() 自体がn<3でfalseを返す仕様）
 * @param override      手動オーバーライド設定
 * @param zFixedM        2D測位で使う固定高さ [m]（結果にそのまま転記するだけ。
 *                        呼び出し側が AnchorTable::setDimension2D() 等に渡す）
 */
ModeDecision decidePositioningMode(size_t enabledCount, const PlacementCheck& placement, ModeOverride override,
                                    float zFixedM);

} // namespace uwb
