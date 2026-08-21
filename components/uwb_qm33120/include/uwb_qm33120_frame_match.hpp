/**
 * @file uwb_qm33120_frame_match.hpp
 * @brief 受信フレームが「待っていたフレームか、それとも他人（別リンク）
 * 宛てのフレームで捨ててよいか」を判定する純関数、およびその判定材料となる
 * バイト列レベルの解析ヘルパ（フレーム長計算・ペイロード照合・タイミング
 * タグ読み出し）。docs/REIMPL_PLAN.md R2。
 *
 * uwb_qm33120_units.hpp / uwb_qm33120_timing.hpp と同じ方針で、ESP-IDF/Qorvo
 * SDK ヘッダに一切依存しない（<cstdint> と <cstring>（memcmp() 用。標準
 * ヘッダなのでホストでもESP-IDFでも使える）のみ）。tools/test_pipeline
 * （ホスト側ビルド）からそのまま include して検算できる。
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
 * uwb_qm33120_internal.hpp の parseShortAddressFrame() が出したヘッダ照合の
 * 可否と、本ヘッダの payloadMatches() 系が出したペイロード照合の可否を、
 * frameMatchesExpectation() の1個のbool判定に還元する。
 *
 * kShortAddressHeaderLen / shortAddressFrameLength() / payloadMatches() /
 * payloadMatchesEither() / readTimingTag() は、元は uwb_qm33120_internal.hpp
 * （esp_timer.h / deca_device_api.h に依存するためホストからincludeできな
 * かった）にあったものをそのままここへ移した（ロジック自体は変更していない。
 * docs/REIMPL_PLAN.md「無理に構造を壊さない」の方針）。
 * uwb_qm33120_internal.hpp は本ヘッダを #include するだけの薄い転送になって
 * いる。
 *
 * --- FCS_LEN について ---
 * shortAddressFrameLength() はフレーム末尾のFCS（Frame Check Sequence）長を
 * 必要とするが、その定義 `FCS_LEN`
 * (components/qm33120w_sdk/deca_device_api.h:236,
 * `#define FCS_LEN 2UL //!< Default FCS is 2 bytes`) は Qorvo SDK のマクロ
 * であり、SDK非依存の本ヘッダから直接参照できない。そこで
 * `uwb::detail::kFcsLen = 2` としてこの値を根拠つきで複製し、
 * uwb_qm33120_internal.hpp 側に
 * `static_assert(detail::kFcsLen == FCS_LEN, ...)` を置くことで、SDK側の
 * 値が変わった場合にビルドで気づけるようにしてある。
 */
#pragma once

#include <cstdint>
#include <cstring>

namespace uwb::detail {

/**
 * @brief フレーム末尾の FCS（Frame Check Sequence）長（バイト）。
 *
 * Qorvo SDK の `FCS_LEN`
 * (components/qm33120w_sdk/deca_device_api.h:236,
 * `#define FCS_LEN 2UL //!< Default FCS is 2 bytes`) と同じ値を、SDK非依存の
 * 本ヘッダのために複製したもの。SDK側の値が変わっても気づけるよう、
 * uwb_qm33120_internal.hpp に
 * `static_assert(detail::kFcsLen == FCS_LEN, ...)` を置いてある。
 */
constexpr uint16_t kFcsLen = 2;

/** IEEE 802.15.4 短アドレス形式フレームのヘッダ長（バイト）。frame[0..8]:
 * FC(2) + seq(1) + panId(2) + dst(2) + src(2)。 */
constexpr uint16_t kShortAddressHeaderLen = 9;

/** @brief payloadLength バイトのペイロードを積んだ短アドレス形式フレームの
 * 全長（ヘッダ + ペイロード + FCS）。 */
constexpr uint16_t shortAddressFrameLength(size_t payloadLength)
{
    return static_cast<uint16_t>(kShortAddressHeaderLen + payloadLength + kFcsLen);
}

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

inline bool payloadMatches(const uint8_t* frame, uint16_t frameLen, const char* payload, size_t prefixLength,
                            size_t expectedPayloadLength)
{
    return (frame != nullptr) && (payload != nullptr) && (prefixLength <= expectedPayloadLength) &&
           (frameLen == shortAddressFrameLength(expectedPayloadLength)) &&
           (memcmp(&frame[kShortAddressHeaderLen], payload, prefixLength) == 0);
}

/**
 * @brief 期待するペイロード長が2通りあるときの照合（docs/TIMING_PRESETS.md §3、
 * タスクC-1）。新旧フレームの両対応: 版情報を持たない旧ファームのフレーム
 * （payload長 lenLegacy）と、末尾に版/種別2バイトを追加した新フレーム
 * （payload長 lenTagged）のどちらでも受理する。関数コード（prefixLength分の
 * 先頭バイト）はどちらの長さでも同じ位置・同じ内容を要求する。
 *
 * 【payloadMatches()は書き換えない】ホスト側検算（tools/test_pipeline）や
 * 既存呼び出し箇所が依存する既存の純関数のため、そのまま2回呼ぶだけに留める。
 */
inline bool payloadMatchesEither(const uint8_t* frame, uint16_t frameLen, const char* payload, size_t prefixLength,
                                  size_t lenLegacy, size_t lenTagged)
{
    return payloadMatches(frame, frameLen, payload, prefixLength, lenLegacy) ||
           payloadMatches(frame, frameLen, payload, prefixLength, lenTagged);
}

/**
 * @brief 拡張ペイロード末尾2バイト（kTimingPresetVersion / TimingProfile の
 * raw値）を読む（docs/TIMING_PRESETS.md §3.1、タスクC-1）。
 *
 * lenTagged (= lenLegacy+2、Poll/Responseどちらも常に+2バイト固定。
 * docs/TIMING_PRESETS.md §3.2 の表参照) の長さのフレームだけを対象にする。
 * 呼び出し側は payloadMatchesEither() で長さ・関数コードの一致を確認した
 * 「直後」に呼ぶ想定（uwb_qm33120_twr.cpp 参照）だが、本関数自身も frameLen
 * を見て安全側に倒す: lenTagged 相当でなければ version/profile を書き換えず
 * false を返す。
 *
 * @return 拡張されている(=lenTagged長)なら true。旧版(lenLegacy長)、または
 * それ以外の長さなら false（version/profileは未変更のまま）。
 */
inline bool readTimingTag(const uint8_t* frame, uint16_t frameLen, size_t lenLegacy, uint8_t& version,
                           uint8_t& profile)
{
    const size_t lenTagged = lenLegacy + 2;
    if ((frame == nullptr) || (frameLen != shortAddressFrameLength(lenTagged))) {
        return false;
    }
    version = frame[kShortAddressHeaderLen + lenLegacy];
    profile = frame[kShortAddressHeaderLen + lenLegacy + 1];
    return true;
}

} // namespace uwb::detail
