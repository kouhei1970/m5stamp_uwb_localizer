/**
 * @file uwb_qm33120_timing.hpp
 * @brief 遅延プリセット（役割 × IRQ有無）の確定値。docs/TIMING_PRESETS.md の
 * 実装（同ドキュメント §6「実装の置き場所」で本ヘッダの置き場所として名指し
 * されている）。
 *
 * uwb_qm33120_units.hpp / uwb_qm33120_frame_match.hpp と同じ方針で、
 * ESP-IDF / Qorvo SDK のヘッダに一切依存しない（<cstdint> のみ）。
 * ホスト側 tests/host/pipeline からそのまま include して検算できるように
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
 *
 * 【2026-08-29 DS-TWR原因特定、1→2】timingPresetDs() の PollingBoth 列
 * (finalTxDelayUus 1800→3000, finalRxAfterResponseTxDelayUus 500→1500) を
 * 変更したため上げた（docs/HANDOFF.md §0-C(2)、docs/TIMING_PRESETS.md の
 * 2026-08-29追記参照）。旧ファーム（版1のまま）と新ファーム（版2）が混在
 * すると、フレームに載った版番号の不一致として checkTimingTag() が検出し
 * 警告する（測距自体は続行する。§3.3）。
 */
inline constexpr uint8_t kTimingPresetVersion = 2;

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
 * 版2の確定値）。
 *
 * PollingBoth の列は DSRangeConfig のメンバ初期化子（uwb_qm33120_twr_config.hpp:
 * responseRxAfterTxDelayUus=1500, responseTxDelayUus=3000, finalTxDelayUus=3000,
 * finalRxAfterResponseTxDelayUus=1500, resultRxAfterFinalTxDelayUus=200,
 * rxTimeoutUus=3000）と完全に一致する。
 *
 * 【2026-08-29 DS-TWR原因特定、finalTxDelayUus 1800→3000・
 * finalRxAfterResponseTxDelayUus 500→1500（版1→2）】850 kbps / preamble 256
 * （本番機の実運用値）での実測でDS-TWRだけが0〜23%（SS-TWRは同条件で
 * 99.95%）という結果になった原因の1つ。旧値1800 UUSでは、タグがFinalを
 * 起動する時点でDW3000 UM §9.4.1エラッタの締切（現在+プリアンブル長+
 * SFD長+20µs）まで0.03〜1.0msしか無く、無警告で送信されないことが
 * あった。Response側（responseTxDelayUus=3000）は同じ850kbps/256で
 * 1.3〜2.3msの余裕があり実証済みなので、Final側もそれと対称な3000/1500
 * に揃えた。再導出の詳細（フレーム air time・エラッタの締切式）は
 * docs/TIMING_PRESETS.md の2026-08-29追記、docs/HANDOFF.md §0-C(2)参照。
 *
 * 【AnchorIrq/BothIrq のDS列は未検証のまま】この2列（finalTxDelayUus=1365/683
 * 等）は6.8 Mbps・preamble 128 前提で導出されたもので、上記の
 * 850kbps/preamble256での再検証は行っていない。IRQ運用（本番機のKconfig
 * 既定はBothIrq）で850kbps/256相当のPHYを使う場合は、このプリセットを
 * そのまま信用せず本節と同じ手順で再導出すること。
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
        // 2026-08-29 DS-TWR原因特定 (版1→2): finalTxDelayUus 1800→3000,
        // finalRxAfterResponseTxDelayUus 500→1500 (docs/HANDOFF.md §0-C(2))。
        return TimingPresetDs{/*responseTxDelayUus=*/3000,
                               /*responseRxAfterTxDelayUus=*/1500,
                               /*finalTxDelayUus=*/3000,
                               /*finalRxAfterResponseTxDelayUus=*/1500,
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
