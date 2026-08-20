/**
 * @file uwb_qm33120_frame_match.hpp
 * @brief 受信フレームが「待っていたフレームか、それとも他人（別リンク）
 * 宛てのフレームで捨ててよいか」を判定する純関数。docs/REIMPL_PLAN.md R2。
 *
 * uwb_qm33120_units.hpp と同じ方針で、ESP-IDF/Qorvo SDK ヘッダに一切依存
 * しない（<cstdint> のみ）。tools/test_pipeline （ホスト側ビルド）から
 * そのまま include して検算できる。
 *
 * --- 背景 (docs/CRITICAL_REVIEW.md【重大1】 / docs/REIMPL_PLAN.md R2) ---
 * 5台 round-robin 構成では、待っているフレームと異なるアドレス/シーケンス
 * 番号のフレームを受信することがある。これは「エラー」ではなく「他人
 * （別リンク）宛てのフレームを1枚拾っただけ」であり、測距シーケンス全体を
 * 破棄すべきではない。
 *
 * Qorvo 公式サンプル（docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/
 * ex_06b_ss_twr_responder/ss_twr_responder.c:166, ex_05b_ds_twr_resp/
 * ds_twr_responder.c:187,238 ほか、grep -a で確認済み）は memcmp() による
 * 照合に失敗しても何もせず、次の while(1) 周回で dwt_rxenable() を
 * 呼び直すだけで、測距を「エラー」として中断しない。本プロジェクトは
 * 1回のTWR呼び出し（requestRange 等）が hostTimeoutMs で区切られた
 * 有限ループなので、公式の「無限に周回する」構造そのものは踏襲できない。
 * 代わりに「不一致は受信継続、hostTimeoutMsで最終的に諦める」という
 * 同じ精神をこの純関数＋呼び出し側のループで再現する
 * （uwb_qm33120_twr.cpp 参照）。
 *
 * uwb_qm33120_internal.hpp の parseShortAddressFrame()/payloadMatches() が
 * 出したヘッダ・ペイロード照合の可否を、この1個のbool判定に還元する。
 * バイト列そのものはここでは一切扱わない（既存のバイト解析ロジックは
 * 変更しない、という docs/REIMPL_PLAN.md「無理に構造を壊さない」の方針）。
 */
#pragma once

#include <cstdint>

namespace uwb::detail {

/**
 * @brief parseShortAddressFrame()/payloadMatches() の結果をまとめたもの。
 * headerOk が false のときは sequence/panId/src/dst は未定義（呼び出し側は
 * 参照しない: frameMatchesExpectation() は headerOk を最初に見て
 * 短絡評価する）。
 */
struct ParsedFrameSummary {
    bool headerOk;   //!< parseShortAddressFrame() の戻り値
    bool payloadOk;  //!< payloadMatches()（関数コード＋フレーム長）の戻り値
    uint8_t sequence;
    uint16_t panId;
    uint16_t src;
    uint16_t dst;
};

/**
 * @brief 受信フレームに期待する条件。
 * @c src を 0 にすると送信元アドレスは照合しない（例: Anchor がどの Tag
 * からの Poll でも受理する respondRange()/respondDSRange() の Poll 待ち。
 * 元コードもここは src を照合していなかった）。0x0000 は本プロジェクトの
 * どのアドレス割当にも使わない予約値なので sentinel として安全
 * （uwb_qm33120_types.hpp の既定値は initiatorAddress=0x0001 から)。
 */
struct FrameExpectation {
    bool checkSequence = false;
    uint8_t sequence    = 0;
    uint16_t panId       = 0;
    uint16_t src          = 0; //!< 0 = don't care
    uint16_t dst          = 0;
};

/**
 * @brief この受信フレームは「待っていたフレーム」か。
 *
 * false を返した場合、R2 の方針は「エラーにせず捨てて受信継続」
 * （呼び出し側で dwt_rxenable(DWT_START_RX_IMMEDIATE) を呼んでループを
 * 続ける。uwb_qm33120_twr.cpp 参照）。
 */
constexpr bool frameMatchesExpectation(const ParsedFrameSummary& f, const FrameExpectation& e)
{
    return f.headerOk && f.payloadOk && (!e.checkSequence || f.sequence == e.sequence) && (f.panId == e.panId) &&
           (e.src == 0 || f.src == e.src) && (f.dst == e.dst);
}

} // namespace uwb::detail
