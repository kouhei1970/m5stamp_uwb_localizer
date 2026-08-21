/**
 * @file uwb_qm33120_timing.hpp
 * @brief 遅延プリセット（役割 × IRQ有無）の確定値。docs/TIMING_PRESETS.md の
 * 実装（同ドキュメント §6「実装の置き場所」で本ヘッダの置き場所として名指し
 * されている）。
 *
 * uwb_qm33120_units.hpp / uwb_qm33120_frame_match.hpp と同じ方針で、
 * ESP-IDF / Qorvo SDK のヘッダに一切依存しない（<cstdint> のみ）。
 * ホスト側 tools/test_pipeline からそのまま include して検算できるように
 * するため。
 *
 * 数値そのものの導出根拠（フレーム air time・折返し時間・DWD受信窓の
 * 落とし穴等）は docs/TIMING_PRESETS.md §1-2 を参照。ここでは確定済みの値を
 * 転記するだけで、このファイル単体では導出の議論はしない。
 */
#pragma once

#include <cstdint>

namespace uwb {

/**
 * @brief 遅延プリセットの種別。フレームに1バイトで載る（docs/TIMING_PRESETS.md
 * §3.1）。値そのもの（0/1/2）はワイヤフォーマットの一部なので変更しないこと
 * （変えると新旧ファーム間で種別の読み違いが起きる）。
 */
enum class TimingProfile : uint8_t {
    PollingBoth = 0, //!< 両側ポーリング（現状の既定値と完全に同一。docs/TIMING_PRESETS.md §2）
    AnchorIrq   = 1, //!< アンカーのみ IRQ（標準構成）
    BothIrq     = 2, //!< 両側 IRQ（StampFly で別配線できた場合）
};

/**
 * @brief プリセット表の版。timingPresetSs()/timingPresetDs() が返す数値を
 * 1つでも変えたら必ず上げること（docs/TIMING_PRESETS.md §3.1）。
 *
 * フレーム（Poll/Response）に kTimingPresetVersion と実際に適用した
 * TimingProfile を載せ、受信側が「相手と自分のプリセットが本当に一致して
 * いるか」を検出できるようにする（docs/TIMING_PRESETS.md §0, §3.3）。
 */
inline constexpr uint8_t kTimingPresetVersion = 1;

/** SS-TWR（RangeConfig）用の遅延プリセット3フィールド。単位は UUS。 */
struct TimingPresetSs {
    uint32_t responseTxDelayUus;
    uint32_t responseRxAfterTxDelayUus;
    uint32_t rxTimeoutUus;
};

/** DS-TWR（DSRangeConfig）用の遅延プリセット6フィールド。単位は UUS。 */
struct TimingPresetDs {
    uint32_t responseTxDelayUus;
    uint32_t responseRxAfterTxDelayUus;
    uint32_t finalTxDelayUus;
    uint32_t finalRxAfterResponseTxDelayUus;
    uint32_t resultRxAfterFinalTxDelayUus;
    uint32_t rxTimeoutUus;
};

/**
 * @brief SS-TWR（RangeConfig）用プリセット値（docs/TIMING_PRESETS.md §2.1、
 * 確定値・1つも変えないこと）。
 *
 * PollingBoth の列は RangeConfig のメンバ初期化子
 * （uwb_qm33120_types.hpp: responseRxAfterTxDelayUus=500,
 * responseTxDelayUus=3000, rxTimeoutUus=4500）と完全に一致する。
 * したがって PollingBoth を既定にする限り既存の挙動は一切変わらない。
 */
constexpr TimingPresetSs timingPresetSs(TimingProfile p)
{
    switch (p) {
    case TimingProfile::AnchorIrq:
        return TimingPresetSs{/*responseTxDelayUus=*/878, /*responseRxAfterTxDelayUus=*/400,
                               /*rxTimeoutUus=*/1200};
    case TimingProfile::BothIrq:
        return TimingPresetSs{/*responseTxDelayUus=*/878, /*responseRxAfterTxDelayUus=*/400,
                               /*rxTimeoutUus=*/1200};
    case TimingProfile::PollingBoth:
    default:
        return TimingPresetSs{/*responseTxDelayUus=*/3000, /*responseRxAfterTxDelayUus=*/500,
                               /*rxTimeoutUus=*/4500};
    }
}

/**
 * @brief DS-TWR（DSRangeConfig）用プリセット値（docs/TIMING_PRESETS.md §2.2、
 * 確定値・1つも変えないこと）。
 *
 * PollingBoth の列は DSRangeConfig のメンバ初期化子（uwb_qm33120_types.hpp:
 * responseRxAfterTxDelayUus=1500, responseTxDelayUus=3000, finalTxDelayUus=1800,
 * finalRxAfterResponseTxDelayUus=500, resultRxAfterFinalTxDelayUus=200,
 * rxTimeoutUus=3000）と完全に一致する。
 *
 * 【resultRxAfterFinalTxDelayUus が IRQ プリセットで 0 になっている理由】
 * （docs/TIMING_PRESETS.md §1.4）: DWD（結果フレーム）はアンカーが Final を
 * 受信した直後に DWT_START_TX_IMMEDIATE で即時送信する（respondDSRange()）。
 * つまり「アンカーの折返しが速くなる（＝IRQで応答する）ほど DWD はタグに
 * 早く着く」。ポーリングでは折返しに〜1.2ms かかるため、200 UUS(≈205µs) の
 * 受信窓オープン猶予でも間に合う。しかし IRQ では折返しが〜0.3msまで縮み、
 * DWD が〜0.26ms後（速ければ60µs後）に到達してしまうため、200 UUS では
 * 受信窓を開くのが遅すぎてプリアンブルの先頭を取りこぼす。値を小さくし
 * すぎて困ることは無い（受信機が少し長くONになるだけ）が、大きすぎると
 * 丸ごと失敗するので非対称に0側へ倒す。
 */
constexpr TimingPresetDs timingPresetDs(TimingProfile p)
{
    switch (p) {
    case TimingProfile::AnchorIrq:
        return TimingPresetDs{/*responseTxDelayUus=*/878,
                               /*responseRxAfterTxDelayUus=*/400,
                               /*finalTxDelayUus=*/1365,
                               /*finalRxAfterResponseTxDelayUus=*/500,
                               /*resultRxAfterFinalTxDelayUus=*/0,
                               /*rxTimeoutUus=*/1200};
    case TimingProfile::BothIrq:
        return TimingPresetDs{/*responseTxDelayUus=*/878,
                               /*responseRxAfterTxDelayUus=*/400,
                               /*finalTxDelayUus=*/683,
                               /*finalRxAfterResponseTxDelayUus=*/200,
                               /*resultRxAfterFinalTxDelayUus=*/0,
                               /*rxTimeoutUus=*/1200};
    case TimingProfile::PollingBoth:
    default:
        return TimingPresetDs{/*responseTxDelayUus=*/3000,
                               /*responseRxAfterTxDelayUus=*/1500,
                               /*finalTxDelayUus=*/1800,
                               /*finalRxAfterResponseTxDelayUus=*/500,
                               /*resultRxAfterFinalTxDelayUus=*/200,
                               /*rxTimeoutUus=*/3000};
    }
}

/** プリセット種別の表示名（ログ出力・不一致警告用）。 */
constexpr const char* timingProfileName(TimingProfile p)
{
    switch (p) {
    case TimingProfile::PollingBoth:
        return "PollingBoth";
    case TimingProfile::AnchorIrq:
        return "AnchorIrq";
    case TimingProfile::BothIrq:
        return "BothIrq";
    default:
        return "?";
    }
}

/** このプリセットは「アンカー側の IRQ」を前提にしているか（AnchorIrq/BothIrq）。 */
constexpr bool timingProfileNeedsAnchorIrq(TimingProfile p)
{
    return (p == TimingProfile::AnchorIrq) || (p == TimingProfile::BothIrq);
}

/** このプリセットは「タグ側の IRQ」も前提にしているか（BothIrq のみ）。 */
constexpr bool timingProfileNeedsTagIrq(TimingProfile p)
{
    return p == TimingProfile::BothIrq;
}

/** raw がフレームに載ってきた TimingProfile の値として妥当か（0..2）。 */
constexpr bool timingProfileValid(uint8_t raw)
{
    return raw <= static_cast<uint8_t>(TimingProfile::BothIrq);
}

} // namespace uwb
