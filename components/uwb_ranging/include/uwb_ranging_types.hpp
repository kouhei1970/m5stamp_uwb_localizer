/**
 * @file uwb_ranging_types.hpp
 * @brief components/uwb_ranging 全体で共有する値型（ハード非依存）。
 *
 * このファイルはハード非依存側（uwb_ranging_anchor_table / uwb_ranging_pipeline）
 * とハード依存側（uwb_ranging_scheduler）の両方から参照される。ESP-IDF や
 * FreeRTOS のヘッダには一切依存しないので、ホスト（tests/host/pipeline）で
 * そのままコンパイルできる。依存するのは components/uwb_loc（uwb_loc.h）のみ。
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "uwb_loc.h"

namespace uwb {

/** 同時に扱えるアンカー数の上限。components/uwb_loc の UWB_MAX_ANCHORS
 *  （Kconfig 既定8、ホストビルドでは Makefile が -D で同じ値を渡す）に合わせる。
 *  台数そのものは AnchorTable::set() に渡すテーブルの長さで決まり、
 *  ここはあくまで静的確保のための上限。 */
inline constexpr size_t kMaxAnchors = UWB_MAX_ANCHORS;

/**
 * @brief アンカー登録テーブルの1エントリ。
 *
 * PLAN.md / 委譲指示で指定された形そのまま。台数はこの構造体の配列の長さで
 * 決まり、コンパイル時に決め打ちしない。
 */
struct AnchorEntry {
    uint16_t short_addr = 0;         //!< TWR のショートアドレス
    float pos[3]         = {0, 0, 0}; //!< ワールド座標 [m]
    float antenna_delay_m = 0.0f;     //!< 測距値から引くオフセット（Phase 3 で校正）
    bool enabled           = false;   //!< false なら測距/測位から除外する
};

/** 測位に使うソルバのレベル。 */
enum class SolverLevel {
    Lv0, //!< 閉形式の三辺測量。初期値確認・配線/座標系の確認用
    Lv2, //!< Beck厳密解 + Huberロバスト化。屋内運用の本番
    Lv3, //!< 密結合EKF。移動体むけ（別APIで呼ぶ。solve()の引数には使わない）
};

/**
 * @brief 1台ぶんの測距試行結果（スケジューラが1周ごとに埋める）。
 *
 * ハード非依存にするため、TWRの生の結果（uwb::RangeResult/DSRangeResult）
 * ではなく、パイプラインが必要とする最小限の情報だけを持つ。ホスト側の
 * 合成テストも同じ構造体を直接組み立てて PositioningPipeline::solve() に渡せる。
 */
struct RangingSample {
    size_t anchor_index    = 0;     //!< AnchorTable 内の添字（table.entry(i) の i）
    bool ok                 = false; //!< 測距に成功したか。false なら欠測扱い
    float distance_m        = 0.0f;  //!< 生の測距値 [m]（アンテナ遅延は未補正のまま渡す。
                                      //!< 補正は uwb_loc 側が anchor.antenna_delay_m で行う）
    uint32_t elapsed_ms     = 0;     //!< この1回のTWR往復に要した時間 [ms]

    //!< この測距が行われた時刻 [us]。esp_timer_get_time() の生値（起動からの単調増加）。
    //!< 0 は「未設定」。1周32msで5台を順に測るため、同一周でも台ごとに最大32msの差がある
    //!< （docs/HANDOFF.md §5 F。1m/s で 32mm の歪み）。
    int64_t t_us = 0;
};

/** 1台ぶんの測距成功率統計。 */
struct AnchorStats {
    uint32_t attempts  = 0; //!< スケジューラがこのアンカーへ測距を試みた回数
    uint32_t successes = 0; //!< うち成功した回数

    /** 成功率 [0,1]。attempts==0 のときは 0 を返す。 */
    float successRate() const
    {
        return (attempts == 0) ? 0.0f : (static_cast<float>(successes) / static_cast<float>(attempts));
    }
};

/** PositioningPipeline の動作設定。 */
struct PositioningConfig {
    /** solve() が level 引数省略時に使う既定レベル。 */
    SolverLevel defaultLevel = SolverLevel::Lv2;

    /** 3D測位に必要な最小有効測距数（既定4。x,y,zの3自由度+幾何的曖昧性解消）。 */
    int minValid3d = 4;

    /** 2D測位（dim=2、高さ固定）に必要な最小有効測距数（既定3）。 */
    int minValid2d = 3;

    /** アンカー平面がワールド原点を「通っている」とみなす距離しきい値 [m]。
     *  docs/ANCHOR_PLACEMENT.md の実測では z=0.000 で Lv2 が ok=0 に、
     *  z=0.001 でも鏡像解を返しており、境界を厳密に特定できていない
     *  （実測点が2つしかないため）。安全側に倒して既定 0.05m とする。 */
    float originPlaneEpsM = 0.05f;
};

/**
 * @brief 起動時のアンカー配置チェック結果。
 *
 * AnchorTable::checkPlacement() が返す。ログ警告を出すかどうかは
 * 呼び出し側（firmware/tag 等）の判断に委ねる。
 */
struct PlacementCheck {
    bool coplanar       = false; //!< true なら全アンカーが同一平面上にある
    bool originWarning  = false; //!< true なら coplanar かつ、その平面がほぼワールド原点を通る
                                  //!< （Lv2 が ok=0 で失敗することが実測で判明している条件）
    uwb_real normal[3]  = {0, 0, 0}; //!< coplanar==true のときの平面法線（単位ベクトル）
    uwb_real offsetM    = 0;         //!< 平面のオフセット（n・p = offsetM）
};

/**
 * @brief 測位結果（uwb_fix のラッパ + パイプライン独自の付加情報）。
 *
 * 単位は m。float にしてあるのは、JSON化やログ出力・ホスト側テストの
 * 比較のしやすさのため（uwb_loc 内部の計算自体は uwb_real＝既定double のまま）。
 */
struct PositionResult {
    bool ok         = false; //!< uwb_fix.ok。false なら p は無意味
    bool ambiguous  = false; //!< uwb_fix.ambiguous。true なら「高さが信用できない」
    bool solvable    = false; //!< 有効測距数が閾値以上でソルバを実際に呼んだか
                               //!< （false のときは「測位不能」。ok も必然的に false）

    float p[3]      = {0, 0, 0}; //!< 推定位置 [m]
    float gdop       = 0.0f;      //!< 幾何精度劣化係数
    float residualRms = 0.0f;     //!< 残差RMS [m]
    float sigma       = 0.0f;     //!< 位置誤差の代表値 [m]

    int nUsed   = 0; //!< 実際に採用された観測数（ロバストゲートで弾かれた分は含まない）
    int nTotal  = 0; //!< ソルバへ渡した観測数（＝有効測距数）

    /** 外れ値として弾かれたアンカーのビットマスク。**bit i は AnchorTable の
     *  添字 i に対応する**（uwb_fix.excluded がソルバへ渡した観測配列の添字
     *  基準なのに対し、こちらは呼び出し側が扱いやすいようアンカーテーブルの
     *  添字基準へ変換済み）。 */
    unsigned long excluded = 0;

    SolverLevel levelUsed = SolverLevel::Lv2; //!< 実際に使われたレベル
};

} // namespace uwb
