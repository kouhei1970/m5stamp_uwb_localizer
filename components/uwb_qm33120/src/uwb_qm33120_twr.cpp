/**
 * @file uwb_qm33120_twr.cpp
 * @brief Phase 2 Step 2: SS-TWR / DS-TWR ranging methods for uwb::Qm33120.
 *
 * ESP-IDF port of third_party/M5Stamp-UWB/src/M5Stamp_UWB.cpp (M5Stack
 * Technology CO LTD, MIT), specifically:
 *   - M5Stamp_UWB::requestRange()    SS-TWR initiator  cpp:785-889
 *   - M5Stamp_UWB::respondRange()    SS-TWR responder  cpp:891-1009
 *   - M5Stamp_UWB::requestDSRange()  DS-TWR initiator  cpp:1010-1161
 *   - M5Stamp_UWB::respondDSRange()  DS-TWR responder（距離計算はここ） cpp:1163-1377
 *
 * 移植方針は Step 1 (uwb_qm33120.cpp) と同じ「1行ずつ突き合わせ」。
 * 各関数の中はロジック・計算式・バイト位置・エンディアン・32bit/40bit の別・
 * 符号付き/符号なしの別を一切変えていない。意図的な変更点は以下のみ:
 *
 *  - millis() -> detail::nowMs(), delay(N) -> vTaskDelay(pdMS_TO_TICKS(N))
 *    （Step 1 と同じ wraparound-safe idiom。sdkconfig.defaults で
 *    CONFIG_FREERTOS_HZ=1000 にしてあるので delay(1) 相当の粒度も同じ）
 *  - `static constexpr double speedOfLight = 299702547.0;` /
 *    `static constexpr uint64_t uusToDwtTime = 65536ULL;` が原本では
 *    requestRange/requestDSRange/respondDSRange の3関数それぞれの先頭で
 *    重複定義されていた（cpp:787-788, 1012-1013, 1165-1166）。
 *    speedOfLight は requestDSRange 内では未使用で `(void)speedOfLight;`
 *    で警告を抑制していた。ここでは Step 1 で前倒し用意済みの
 *    uwb_qm33120_internal.hpp の detail::kSpeedOfLightMPerS /
 *    detail::kUusToDwtTime に一本化し、使わない関数では単に参照しない
 *    （値は変更なし。速度は真空中の光速ではなく実効値なので
 *    299792458 に「修正」したりしないこと）
 *  - get16le/get32le/get40le/set16le/set32le, buildShortAddressFrame/
 *    parseShortAddressFrame/payloadMatches, rxStatusToError,
 *    stopRadioAndClear{Status,RxStatus,TxStatus,IoStatus},
 *    readTxTimestamp64()/readRxTimestamp64() は
 *    uwb_qm33120_internal.hpp の detail:: 版（Step 1 で前倒し移植済み）を使う
 *  - Qm33120::Impl 型の完全な定義を uwb_qm33120_impl.hpp から include する
 *    （uwb_qm33120.cpp 側の変更点についてはそちらのファイル先頭コメント参照）
 *
 * アンテナ遅延の手動加算（cpp:946, 1087, 1226 = respTxTs/finalTxTs/respTxTsPlan
 * の算出）は削らずそのまま残している。チップ側の自動アンテナ遅延補正
 * （dwt_setrxantennadelay()/dwt_settxantennadelay()、uwb_qm33120.cpp の
 * init() 参照）とは役割が別（「遅延送信の起動時刻」と「アンテナから実際に
 * 電波が出る時刻」の差を埋めるもの）であり、二重計上ではない。
 *
 * --- docs/REIMPL_PLAN.md Phase 2R での変更 (R2/R4/R9) ---
 * 上記の「移植」段階のあとに、一次資料（Qorvo API rev9p3 / DW3000 UM /
 * DW3720 API Guide）に基づく修正を入れている。数式・バイト位置・
 * エンディアン等は引き続き一切変更していない。
 *
 *  - 【R2】6箇所（各関数のフレーム照合直後）で、期待外のフレーム
 *    （他人/別リンク宛て、シーケンス番号不一致等）を受信したときに
 *    dwt_forcetrxoff() して測距シーケンス全体を Error::RangeFrameMismatch
 *    で中断していたのをやめ、dwt_rxenable(DWT_START_RX_IMMEDIATE) で
 *    受信を再開して hostTimeoutMs まで待ち続けるようにした。根拠は
 *    uwb_qm33120_frame_match.hpp 冒頭コメントと、各関数内のR2コメント
 *    （Qorvo公式 ex_05a/ex_05b/ex_06a/ex_06b の受信ループ構造）を参照。
 *    フレーム照合の可否判定そのものは
 *    uwb::detail::frameMatchesExpectation()（新規、ESP-IDF非依存の純関数、
 *    tools/test_pipeline でホスト検算可能）に切り出した。
 *  - 【R4】requestRange()（SS-TWR initiator）のToF計算に
 *    dwt_readclockoffset() によるクロックオフセット補正を追加した
 *    （既定で有効。RangeConfig::enableClockOffsetCorrection で無効化可）。
 *    DS-TWR側（requestDSRange/respondDSRange）は非対称DS-TWR式が原理的に
 *    クロックオフセットに免疫があるため変更していない
 *    （DW3000 UM §12.3、docs/refs/DW3000_Family_User_Manual_wayback.txt
 *    13998行「the typical clock induced error is in the low picosecond
 *    range even with 20 ppm crystals」）。詳細は requestRange() 内のコメント。
 *  - 【R9】dwt_setpreambledetecttimeout() は当初、公式DS-TWRサンプルが
 *    使っている値（PRE_TIMEOUT=5 PAC）にだけ倣って requestDSRange()/
 *    respondDSRange() に追加していたが、これは移植ミスだった
 *    （docs/REVIEW_2026-08-21.md §0 #1、docs/REIMPL_PLAN.md R9）。PRETOC
 *    は「RXが開放された時刻」を起点に走り出すタイマーであり（SDK
 *    components/qm33120w_sdk/deca_device_api.h:2368-2372「X ≥ 1 sets a
 *    timeout equal to (X+1)*PAC」、UM
 *    docs/refs/DW3000_Family_User_Manual_wayback.txt:8152-8158「The
 *    preamble detection timeout starts running as soon as the receiver
 *    is enabled to hunt for preamble」）、公式 ex_05a/ex_05b が5 PACで
 *    動くのはRX開放直後（数µs後）に相手のプリアンブルが来るよう遅延を
 *    調律しているため。本実装はRXを相手プリアンブル到達の0.3〜1.4ms前に
 *    開ける設計（PollingBoth: タグはPoll送信後1500 UUSでRX開始、アンカー
 *    のResponseは3000 UUS後、等）なので、6 PAC ≈ 49µs（PAC8時）のPRETOC
 *    は必ずRXPTO（プリアンブル検出タイムアウト）を起こしDS-TWRが常に
 *    失敗する。よって requestDSRange()/respondDSRange() とも0（無効）に
 *    戻した。SS-TWR（requestRange/respondRange）はもともと0のまま
 *    （公式ex_06a/ex_06bが一度も使っていないため）。PRETOCを使う場合は
 *    「公式に倣って5」ではなく、RX開放時刻から相手のプリアンブル先頭が
 *    届くまでの時間をPAC単位で計算して設定すること（詳細は各関数の
 *    コメント）。
 *
 * --- IRQ（起床信号）対応 (docs/IRQ_POLICY.md、タスクC) ---
 * 4関数（requestRange/respondRange/requestDSRange/respondDSRange）すべての
 * 受信待ちループで、`vTaskDelay(pdMS_TO_TICKS(1))` を
 * `(void)uwb_port_irq_wait(1)` に置き換えている。設計の要点:
 *
 *  - **IRQ は「起床信号」としてのみ使う。** dwt_readsysstatuslo() による
 *    ステータスレジスタの判読、frameMatchesExpectation() によるフレーム
 *    照合、エラー処理は一切変更していない。ポーリング経路とIRQ経路で
 *    復号ロジックは完全に同一の1本のコードパスを通る。
 *  - **dwt_setcallbacks() / dwt_isr() は使わない。** 使うと
 *    ステータス判読・フレーム取得がSDK内部のコールバック経路に分岐し、
 *    ポーリング経路と2本立てになる。バグの温床・検証コストの二重化を
 *    避けるため、意図的に使わない
 *    （docs/IRQ_POLICY.md「ポーリング経路は劣化版ではなく正規の動作
 *    モードとして保守する」）。
 *  - **uwb_port_irq_wait(1) は IRQ が無効なとき
 *    vTaskDelay(pdMS_TO_TICKS(1)) と完全に等価に振る舞う**
 *    （components/uwb_port/include/uwb_port.h）。したがって
 *    Qm33120::Config::use_irq が false、または pin_irq が未配線、または
 *    ISR登録に失敗した場合、この4関数の挙動はIRQ対応前とビット単位で
 *    同一になる。
 *  - 各受信待ちループへ入る直前（RXを起動した直後）に
 *    uwb_port_irq_clear_pending() を呼び、取りこぼした（前回のフレーム分の）
 *    通知を捨ててから待ちに入る。エッジを取りこぼしても待ちは最大1msで
 *    必ず抜ける（ポーリングにフォールバックするだけ）ので安全側に落ちる。
 *  - **置き換えたのは「受信を待つループの中の1msスリープ」だけ。**
 *    自分の送信完了(DWT_INT_TXFRS_BIT_MASK)を待つループ（respondRange()の
 *    Response送信後、respondDSRange()のDWD結果送信後）や、結果フレーム
 *    再送間隔(resultRepeatGapMs)のvTaskDelayは対象外（そのまま）。
 *    dwt_setinterrupt()で有効化しているのはRXFCG/RX_TO/RX_ERRのみで
 *    TXFRSは含まれないため、これらのTX待ちループをIRQ待ちに置き換えると
 *    IRQが来ずに毎回タイムアウトしてしまう。
 */
#include "uwb_qm33120.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern "C" {
#include "deca_device_api.h"
#include "deca_interface.h"
}

#include "uwb_qm33120_frame_match.hpp"
#include "uwb_qm33120_impl.hpp"
#include "uwb_qm33120_internal.hpp"
#include "uwb_qm33120_timing.hpp"

namespace uwb {

namespace {

static const char* const kTimingLogTag = "uwb_qm33120_twr";

/** Qm33120::Impl::warned_peers[8] のサイズと一致させること（uwb_qm33120_impl.hpp）。 */
static constexpr uint8_t kMaxWarnedPeers = 8;

/**
 * @brief 相手（peerAddr）から受け取ったフレームのタイミングプリセット
 * 版/種別を検査し、不一致なら1回だけ警告する
 * （docs/TIMING_PRESETS.md §3.3、タスクC-3）。測距は続行する（拒否すると
 * 切り分けが難しくなるため。docs/IRQ_POLICY.md「不一致を検出したら警告する」）。
 *
 * 記録・あふれ対策の方針は uwb_qm33120_impl.hpp の warned_peers コメント参照。
 *
 * 【Qm33120::Impl& を取らない理由】Implはprivateなネスト型
 * (uwb_qm33120.hpp)のため、クラスの外に定義するこの無名namespace内の
 * 自由関数はその型名を書けない（フル定義がuwb_qm33120_impl.hpp経由で見えて
 * いてもアクセス指定は別）。そのため呼び出し側(Qm33120のメンバ関数内、
 * _impl経由でのアクセスは許可される)から必要なフィールドを個別に渡す。
 *
 * @param warnedPeers  Qm33120::Impl::warned_peers（呼び出し側が渡す）。
 * @param warnedCount  Qm33120::Impl::warned_count（呼び出し側が渡す）。
 * @param myProfile    自分の config.timing_profile。
 * @param context      ログに出す短い文脈（どの関数からの呼び出しか）。
 * @param peerAddr     相手のショートアドレス。
 * @param extended     detail::readTimingTag() の戻り値
 *                      （true=拡張ペイロード=版/種別が読めた、false=旧ファーム）。
 * @param peerVersion  extended==true のときだけ有効。
 * @param peerProfile  extended==true のときだけ有効（TimingProfile の raw値）。
 */
void checkTimingTagAndWarn(uint16_t* warnedPeers, uint8_t& warnedCount, TimingProfile myProfile,
                            const char* context, uint16_t peerAddr, bool extended, uint8_t peerVersion,
                            uint8_t peerProfile)
{
    for (uint8_t i = 0; i < warnedCount; ++i) {
        if (warnedPeers[i] == peerAddr) {
            return; // 既に警告済み。
        }
    }

    bool mismatch = false;
    if (!extended) {
        ESP_LOGW(kTimingLogTag,
                 "%s: peer=0x%04X は版情報を持たないファーム（旧長のフレーム）を送ってきました。"
                 "タイミングプリセットが一致している保証がありません。"
                 "タグとアンカーを同じプリセットで焼き直すこと（docs/TIMING_PRESETS.md）。",
                 context, static_cast<unsigned>(peerAddr));
        mismatch = true;
    } else if (peerVersion != kTimingPresetVersion) {
        ESP_LOGW(kTimingLogTag,
                 "%s: peer=0x%04X とプリセット表の版が違います (mine=%u peer=%u)。"
                 "タグとアンカーを同じプリセットで焼き直すこと（docs/TIMING_PRESETS.md）。",
                 context, static_cast<unsigned>(peerAddr), static_cast<unsigned>(kTimingPresetVersion),
                 static_cast<unsigned>(peerVersion));
        mismatch = true;
    } else if (peerProfile != static_cast<uint8_t>(myProfile)) {
        const char* peerName =
            timingProfileValid(peerProfile) ? timingProfileName(static_cast<TimingProfile>(peerProfile)) : "?";
        ESP_LOGW(kTimingLogTag,
                 "%s: peer=0x%04X とタイミングプリセットの種別が違います (mine=%s peer=%s)。"
                 "タグとアンカーを同じプリセットで焼き直すこと（docs/TIMING_PRESETS.md）。",
                 context, static_cast<unsigned>(peerAddr), timingProfileName(myProfile), peerName);
        mismatch = true;
    }

    if (mismatch && (warnedCount < kMaxWarnedPeers)) {
        warnedPeers[warnedCount++] = peerAddr;
    }
}

} // namespace

RangeResult Qm33120::requestRange(const RangeConfig& range)
{
    // cpp:785-889. SS-TWR initiator (Tag). ロジック変更なし。
    RangeResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    // 【タスクC-2】Poll ペイロードにプリセットの版/種別を載せる
    // (docs/TIMING_PRESETS.md §3.2: "TWP" 3バイト -> 5バイト)。送るのは
    // 「Kconfigで設定した種別」ではなく「実際に適用した種別」
    // (_impl->config.timing_profile。docs/TIMING_PRESETS.md §3.1)。
    uint8_t pollFrame[14]       = {0};
    const uint8_t pollPayload[] = {'T', 'W', 'P', kTimingPresetVersion,
                                    static_cast<uint8_t>(_impl->config.timing_profile)};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    detail::buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    pollPayload, sizeof(pollPayload));

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }
    // DWT_RESPONSE_EXPECTED によりTX完了後ただちにRXが開始される。受信待ち
    // ループへ入る直前に、取りこぼした通知を捨てる（本ファイル冒頭コメント
    // 参照）。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rawFrame[32] = {0};
            uint8_t rangingBit   = 0;
            uint16_t frameLen    = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(rawFrame)) {
                frameLen = sizeof(rawFrame);
            }
            dwt_readrxdata(rawFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            RxResult parsed;
            const bool headerOk = detail::parseShortAddressFrame(rawFrame, frameLen, parsed);
            // 【タスクC-2】"TWR" ペイロードは 11(旧) または 13(版/種別付き)
            // バイトのどちらでも受理する (docs/TIMING_PRESETS.md §3.3)。
            const bool payloadOk =
                detail::payloadMatchesEither(rawFrame, frameLen, "TWR", 3, /*lenLegacy=*/11, /*lenTagged=*/13);
            const detail::ParsedFrameSummary summary{headerOk,      payloadOk,     parsed.sequence,
                                                       parsed.panId, parsed.src,    parsed.dst};
            const detail::FrameExpectation expect{/*checkSequence=*/true, pollSeq, range.panId,
                                                   range.responderAddress, range.initiatorAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                // 【タスクC-3】相手(Anchor)のプリセット版/種別を読み、
                // 不一致なら警告する（測距は続行する）。
                {
                    uint8_t peerVersion = 0, peerProfile = 0;
                    const bool extended =
                        detail::readTimingTag(rawFrame, frameLen, /*lenLegacy=*/11, peerVersion, peerProfile);
                    checkTimingTagAndWarn(_impl->warned_peers, _impl->warned_count, _impl->config.timing_profile,
                                           "requestRange: Response<-ANCHOR", range.responderAddress, extended,
                                           peerVersion, peerProfile);
                }

                const uint32_t pollTxTs = dwt_readtxtimestamplo32();
                const uint32_t respRxTs = dwt_readrxtimestamplo32(static_cast<dwt_ip_sts_segment_e>(0));
                const uint32_t pollRxTs = detail::get32le(&rawFrame[12]);
                const uint32_t respTxTs = detail::get32le(&rawFrame[16]);
                const int32_t rtdInit   = static_cast<int32_t>(respRxTs - pollTxTs);
                const int32_t rtdResp   = static_cast<int32_t>(respTxTs - pollRxTs);

                if ((rtdInit <= 0) || (rtdResp <= 0)) {
                    result.elapsedMs = detail::nowMs() - startMs;
                    result.error     = Error::RangeTimestampInvalid;
                    setError(result.error);
                    return result;
                }

                // 【R4】SS-TWR クロックオフセット補正 (docs/REIMPL_PLAN.md R4)。
                // 一次資料: docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/
                // ex_06a_ss_twr_initiator/ss_twr_initiator.c:199-211（grep -aで直接確認済み）。
                //   clockOffsetRatio = ((float)dwt_readclockoffset()) / (uint32_t)(1 << 26);
                //   rtd_init = resp_rx_ts - poll_tx_ts;
                //   rtd_resp = resp_tx_ts - poll_rx_ts;
                //   tof = ((rtd_init - rtd_resp * (1 - clockOffsetRatio)) / 2.0) * DWT_TIME_UNITS;
                // 式の出典（同じ形）: DW3000 UM §12.2 Figure 31 の直後の式
                // T_prop = (1/2)(T_round - T_reply(1-C_offset))
                // (docs/refs/DW3000_Family_User_Manual_wayback.txt:13924-13928)。
                // dwt_readclockoffset() の戻り値の意味
                // (components/qm33120w_sdk/deca_device_api.h:2795-2801):
                // 「2^26で割るとppmオフセットになる signed 16-bit」「ローカル(RX)
                // クロックが相手(TX)より遅いと正の値」。TOF計算に使うのは
                // ppm値そのものではなく2^26で割った「比」(ppmにするにはさらに
                // ×1e6が必要、DW3720 API Guide §5.4.13)。CIAが動作していないと
                // 0を返す（同guide 5.4.13 Notes「If the CIA is not running,
                // this function will return 0」）ため、無効化フラグが立って
                // いなくてもCIA未動作時は補正量0＝無補正と等価で安全側に落ちる。
                // DS-TWR（requestDSRange/respondDSRange）には入れない:
                // 非対称DS-TWR式は原理的にクロックオフセットへ免疫がある
                // （DW3000 UM §12.3、本ファイル冒頭コメント参照）。
                float clockOffsetRatio = 0.0f;
                if (range.enableClockOffsetCorrection) {
                    clockOffsetRatio = static_cast<float>(dwt_readclockoffset()) / static_cast<float>(1UL << 26);
                }
                const double tof = ((static_cast<double>(rtdInit) -
                                      static_cast<double>(rtdResp) * (1.0 - static_cast<double>(clockOffsetRatio))) /
                                     2.0) *
                                    DWT_TIME_UNITS;
                result.distanceM = static_cast<float>(tof * detail::kSpeedOfLightMPerS);
                result.distanceMm =
                    static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));
                result.clockOffsetPpm = clockOffsetRatio * 1.0e6f; // 実機デバッグ用に可視化（R4）
                result.sequence        = pollSeq;
                result.elapsedMs       = detail::nowMs() - startMs;
                result.success         = true;
                result.error           = Error::Ok;
                setError(Error::Ok);
                return result;
            }

            // 【R2】アドレス/シーケンス不一致は「エラー」ではなく「他人
            // （別リンク）宛てのフレームを1枚拾っただけ」。測距シーケンスを
            // 破棄せず、受信を再開して hostTimeoutMs まで待ち続ける。
            // 根拠・dwt_setrxtimeout()再設定が不要な理由は本ファイル冒頭の
            // R2コメントとuwb_qm33120_frame_match.hppを参照。
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    detail::stopRadioAndClearRxStatus();
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = Error::RxTimeout;
    setError(result.error);
    return result;
}

ResponderResult Qm33120::respondRange(const RangeConfig& range)
{
    // cpp:891-1009. SS-TWR responder (Anchor)。ロジック変更なし。
    ResponderResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearRxStatus();
        result.error = Error::RxStartFailed;
        setError(result.error);
        return result;
    }
    // 受信待ちループへ入る直前に、取りこぼした通知を捨てる
    // (本ファイル冒頭コメント参照)。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t pollFrame[32] = {0};
            uint8_t rangingBit    = 0;
            uint16_t frameLen     = dwt_getframelength(&rangingBit);
            if (frameLen > sizeof(pollFrame)) {
                frameLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, frameLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            RxResult parsed;
            const bool headerOk = detail::parseShortAddressFrame(pollFrame, frameLen, parsed);
            // 【タスクC-2】"TWP" ペイロードは 3(旧) または 5(版/種別付き)
            // バイトのどちらでも受理する (docs/TIMING_PRESETS.md §3.3)。
            const bool payloadOk =
                detail::payloadMatchesEither(pollFrame, frameLen, "TWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5);
            const detail::ParsedFrameSummary summary{headerOk,      payloadOk,  parsed.sequence,
                                                       parsed.panId, parsed.src, parsed.dst};
            // src は照合しない（どの Tag からの Poll でも受理する。元コードの
            // 挙動そのまま）。
            const detail::FrameExpectation expect{/*checkSequence=*/false, 0, range.panId, /*src=*/0,
                                                   range.responderAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                // 【タスクC-3】相手(Tag)のプリセット版/種別を読み、
                // 不一致なら警告する（測距は続行する）。
                {
                    uint8_t peerVersion = 0, peerProfile = 0;
                    const bool extended =
                        detail::readTimingTag(pollFrame, frameLen, /*lenLegacy=*/3, peerVersion, peerProfile);
                    checkTimingTagAndWarn(_impl->warned_peers, _impl->warned_count, _impl->config.timing_profile,
                                           "respondRange: Poll<-TAG", parsed.src, extended, peerVersion,
                                           peerProfile);
                }

                uint8_t ts[5] = {0};
                dwt_readrxtimestamp(ts, static_cast<dwt_ip_sts_segment_e>(0));
                const uint64_t pollRxTs = detail::get40le(ts);
                const uint32_t respTxTime =
                    static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * detail::kUusToDwtTime)) >> 8);
                const uint64_t respTxTs =
                    ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

                // 【タスクC-2】Response ペイロードにもプリセットの版/種別を
                // 載せる (docs/TIMING_PRESETS.md §3.2: "TWR" 11バイト -> 13バイト)。
                uint8_t respPayload[13] = {'T', 'W', 'R'};
                detail::set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
                detail::set32le(&respPayload[7], static_cast<uint32_t>(respTxTs));
                respPayload[11] = kTimingPresetVersion;
                respPayload[12] = static_cast<uint8_t>(_impl->config.timing_profile);

                uint8_t respFrame[22] = {0};
                detail::buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress,
                                                parsed.src, respPayload, sizeof(respPayload));

                dwt_setdelayedtrxtime(respTxTime);
                if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
                    detail::stopRadioAndClearTxStatus();
                    result.error = Error::TxDataFailed;
                    setError(result.error);
                    return result;
                }
                dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
                if (dwt_starttx(DWT_START_TX_DELAYED) != DWT_SUCCESS) {
                    detail::stopRadioAndClearTxStatus();
                    result.error = Error::TxStartFailed;
                    setError(result.error);
                    return result;
                }

                const uint32_t txStartMs = detail::nowMs();
                while ((detail::nowMs() - txStartMs) < 20) {
                    const uint32_t txStatus = dwt_readsysstatuslo();
                    if ((txStatus & DWT_INT_TXFRS_BIT_MASK) != 0) {
                        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                        result.success   = true;
                        result.sequence  = parsed.sequence;
                        result.requester = parsed.src;
                        result.elapsedMs = detail::nowMs() - startMs;
                        result.error     = Error::Ok;
                        setError(Error::Ok);
                        return result;
                    }
                    vTaskDelay(pdMS_TO_TICKS(1));
                }

                detail::stopRadioAndClearTxStatus();
                result.error = Error::TxTimeout;
                setError(result.error);
                return result;
            }

            // 【R2】不一致 -> 受信継続（本ファイル冒頭コメント参照）。
            dwt_rxenable(DWT_START_RX_IMMEDIATE);
        }

        if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    detail::stopRadioAndClearRxStatus();
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = Error::RxTimeout;
    setError(result.error);
    return result;
}

DSRangeResult Qm33120::requestDSRange(const DSRangeConfig& range)
{
    // cpp:1010-1161. DS-TWR initiator (Tag)。ロジック変更なし。
    // 距離はここでは計算しない: Anchor(respondDSRange)が計算した値を
    // "DWD" フレームでそのまま受信して result.distanceMm/distanceM に採用する
    // (cpp:1152-1153)。
    DSRangeResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    // 【タスクC-2】Poll ペイロードにプリセットの版/種別を載せる
    // (docs/TIMING_PRESETS.md §3.2: "DWP" 3バイト -> 5バイト)。requestRange()
    // と同じ理由で「実際に適用した種別」を送る。
    uint8_t pollFrame[14]       = {0};
    const uint8_t pollPayload[] = {'D', 'W', 'P', kTimingPresetVersion,
                                    static_cast<uint8_t>(_impl->config.timing_profile)};
    const uint8_t pollSeq       = ++_impl->tx_sequence;
    detail::buildShortAddressFrame(pollFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    pollPayload, sizeof(pollPayload));

    detail::stopRadioAndClearIoStatus();
    // 【R9】プリアンブル検出タイムアウト(PAC単位)。0=無効。
    // PRETOCはRXが開放された時刻を起点に走り出すタイマーであり（SDK
    // deca_device_api.h:2368-2372「X ≥ 1 sets a timeout equal to
    // (X+1)*PAC」、UM DW3000_Family_User_Manual_wayback.txt:8152-8158
    // 「starts running as soon as the receiver is enabled to hunt for
    // preamble」）、本実装はRXをAnchorのResponse送信予定より0.3〜1.4ms
    // 早く開ける設計（PollingBoth: Poll送信後1500 UUSでRX開始）なので、
    // 公式ex_05a_ds_twr_init/ds_twr_initiator.c:92,155のPRE_TIMEOUT=5
    // （6 PAC≈49µs@PAC8）をそのまま設定すると必ずRXPTOで失敗する
    // （本ファイル冒頭R9コメント参照）。PRETOCを使うならRX開放〜相手
    // プリアンブル先頭到達までの時間をPAC単位で計算して設定すること。
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(range.responseRxAfterTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);

    if (dwt_writetxdata(sizeof(pollFrame), pollFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(pollFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }
    // DWT_RESPONSE_EXPECTED によりTX完了後ただちにRXが開始される。受信待ち
    // ループへ入る直前に、取りこぼした通知を捨てる（本ファイル冒頭コメント
    // 参照）。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    uint8_t respFrame[32]     = {0};
    uint16_t respLen          = 0;
    RxResult parsed;
    bool respMatched          = false;
    bool respMismatchSeen     = false; // R2診断用: hostTimeoutMs内で不一致フレームを1回以上見たか
    const uint32_t startMs    = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            respLen            = dwt_getframelength(&rangingBit);
            if (respLen > sizeof(respFrame)) {
                respLen = sizeof(respFrame);
            }
            dwt_readrxdata(respFrame, respLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            const bool headerOk = detail::parseShortAddressFrame(respFrame, respLen, parsed);
            // 【タスクC-2】"DWR" ペイロードは 11(旧) または 13(版/種別付き)
            // バイトのどちらでも受理する (docs/TIMING_PRESETS.md §3.3)。
            const bool payloadOk =
                detail::payloadMatchesEither(respFrame, respLen, "DWR", 3, /*lenLegacy=*/11, /*lenTagged=*/13);
            const detail::ParsedFrameSummary summary{headerOk,      payloadOk,  parsed.sequence,
                                                       parsed.panId, parsed.src, parsed.dst};
            const detail::FrameExpectation expect{/*checkSequence=*/true, pollSeq, range.panId,
                                                   range.responderAddress, range.initiatorAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                respMatched = true;
                // 【タスクC-3】相手(Anchor)のプリセット版/種別を読み、
                // 不一致なら警告する（測距は続行する）。
                uint8_t peerVersion = 0, peerProfile = 0;
                const bool extended =
                    detail::readTimingTag(respFrame, respLen, /*lenLegacy=*/11, peerVersion, peerProfile);
                checkTimingTagAndWarn(_impl->warned_peers, _impl->warned_count, _impl->config.timing_profile,
                                       "requestDSRange: Response<-ANCHOR", range.responderAddress, extended,
                                       peerVersion, peerProfile);
            } else {
                // 【R2】不一致 -> エラーにせず受信継続（本ファイル冒頭コメント参照）。
                respMismatchSeen = true;
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        } else if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        if (respMatched) {
            break;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    if (!respMatched) {
        // hostTimeoutMs を使い切っても一致するResponseを受け取れなかった。
        // 不一致フレームを1枚以上見ていれば RangeFrameMismatch、
        // 何も受信できなかった（respLen==0のまま）なら RxTimeout。
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = respMismatchSeen ? Error::RangeFrameMismatch : Error::RxTimeout;
        setError(result.error);
        return result;
    }

    const uint64_t pollTxTs    = detail::readTxTimestamp64();
    const uint64_t respRxTs    = detail::readRxTimestamp64();
    const uint32_t finalTxTime =
        static_cast<uint32_t>((respRxTs + (range.finalTxDelayUus * detail::kUusToDwtTime)) >> 8);
    const uint64_t finalTxTs = ((static_cast<uint64_t>(finalTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    uint8_t finalPayload[15] = {'D', 'W', 'F'};
    detail::set32le(&finalPayload[3], static_cast<uint32_t>(pollTxTs));
    detail::set32le(&finalPayload[7], static_cast<uint32_t>(respRxTs));
    detail::set32le(&finalPayload[11], static_cast<uint32_t>(finalTxTs));

    uint8_t finalFrame[24] = {0};
    detail::buildShortAddressFrame(finalFrame, pollSeq, range.panId, range.initiatorAddress, range.responderAddress,
                                    finalPayload, sizeof(finalPayload));

    dwt_setdelayedtrxtime(finalTxTime);
    dwt_setrxaftertxdelay(range.resultRxAfterFinalTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    if (dwt_writetxdata(sizeof(finalFrame), finalFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(finalFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }
    // DWT_RESPONSE_EXPECTED によりTX完了後ただちにRXが開始される。受信待ち
    // ループへ入る直前に、取りこぼした通知を捨てる（本ファイル冒頭コメント
    // 参照）。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    uint8_t distFrame[32]         = {0};
    uint16_t distLen              = 0;
    bool distMatched               = false;
    bool distMismatchSeen          = false; // R2診断用。上のrespMismatchSeenと同じ意味
    const uint32_t resultStartMs = detail::nowMs();
    while ((detail::nowMs() - resultStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            distLen            = dwt_getframelength(&rangingBit);
            if (distLen > sizeof(distFrame)) {
                distLen = sizeof(distFrame);
            }
            dwt_readrxdata(distFrame, distLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            const bool headerOk  = detail::parseShortAddressFrame(distFrame, distLen, parsed);
            const bool payloadOk = detail::payloadMatches(distFrame, distLen, "DWD", 3, 7);
            const detail::ParsedFrameSummary summary{headerOk,      payloadOk,  parsed.sequence,
                                                       parsed.panId, parsed.src, parsed.dst};
            const detail::FrameExpectation expect{/*checkSequence=*/true, pollSeq, range.panId,
                                                   range.responderAddress, range.initiatorAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                distMatched = true;
            } else {
                // 【R2】不一致 -> エラーにせず受信継続。
                distMismatchSeen = true;
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        } else if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.sequence  = pollSeq;
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        if (distMatched) {
            break;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    if (!distMatched) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = distMismatchSeen ? Error::RangeFrameMismatch : Error::RxTimeout;
        setError(result.error);
        return result;
    }

    result.distanceMm = static_cast<int32_t>(detail::get32le(&distFrame[12]));
    result.distanceM  = static_cast<float>(result.distanceMm) / 1000.0f;
    result.sequence   = pollSeq;
    result.elapsedMs  = detail::nowMs() - startMs;
    result.success    = true;
    result.error      = Error::Ok;
    setError(Error::Ok);
    return result;
}

DSResponderResult Qm33120::respondDSRange(const DSRangeConfig& range)
{
    // cpp:1163-1377. DS-TWR responder (Anchor)。距離計算はこの関数の中で行う
    // （非対称DS-TWR標準式、cpp:1294-1319）。ロジック変更なし。
    DSResponderResult result;
    if (!_impl->initialized) {
        result.error = Error::ConfigFailed;
        setError(result.error);
        return result;
    }

    detail::stopRadioAndClearIoStatus();
    dwt_setpreambledetecttimeout(0);
    dwt_setrxaftertxdelay(0);
    dwt_setrxtimeout(0);

    if (dwt_rxenable(DWT_START_RX_IMMEDIATE) != DWT_SUCCESS) {
        detail::stopRadioAndClearRxStatus();
        result.error = Error::RxStartFailed;
        setError(result.error);
        return result;
    }
    // 受信待ちループへ入る直前に、取りこぼした通知を捨てる
    // (本ファイル冒頭コメント参照)。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    uint8_t pollFrame[32] = {0};
    uint16_t pollLen      = 0;
    RxResult parsed;
    bool pollMatched       = false;
    bool pollMismatchSeen  = false; // R2診断用（他の受信ループと同じ意味）
    const uint32_t startMs = detail::nowMs();
    while ((detail::nowMs() - startMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            pollLen            = dwt_getframelength(&rangingBit);
            if (pollLen > sizeof(pollFrame)) {
                pollLen = sizeof(pollFrame);
            }
            dwt_readrxdata(pollFrame, pollLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD);

            const bool headerOk = detail::parseShortAddressFrame(pollFrame, pollLen, parsed);
            // 【タスクC-2】"DWP" ペイロードは 3(旧) または 5(版/種別付き)
            // バイトのどちらでも受理する (docs/TIMING_PRESETS.md §3.3)。
            const bool payloadOk =
                detail::payloadMatchesEither(pollFrame, pollLen, "DWP", 3, /*lenLegacy=*/3, /*lenTagged=*/5);
            const detail::ParsedFrameSummary summary{headerOk,      payloadOk,  parsed.sequence,
                                                       parsed.panId, parsed.src, parsed.dst};
            // src は照合しない（どの Tag からの Poll でも受理する。元コードの
            // 挙動そのまま。respondRange() と同じ）。
            const detail::FrameExpectation expect{/*checkSequence=*/false, 0, range.panId, /*src=*/0,
                                                   range.responderAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                pollMatched = true;
                // 【タスクC-3】相手(Tag)のプリセット版/種別を読み、
                // 不一致なら警告する（測距は続行する）。
                uint8_t peerVersion = 0, peerProfile = 0;
                const bool extended =
                    detail::readTimingTag(pollFrame, pollLen, /*lenLegacy=*/3, peerVersion, peerProfile);
                checkTimingTagAndWarn(_impl->warned_peers, _impl->warned_count, _impl->config.timing_profile,
                                       "respondDSRange: Poll<-TAG", parsed.src, extended, peerVersion,
                                       peerProfile);
            } else {
                // 【R2】不一致 -> 受信継続（本ファイル冒頭コメント参照）。
                pollMismatchSeen = true;
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        } else if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        if (pollMatched) {
            break;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    if (!pollMatched) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = pollMismatchSeen ? Error::RangeFrameMismatch : Error::RxTimeout;
        setError(result.error);
        return result;
    }

    const uint64_t pollRxTs = detail::readRxTimestamp64();
    const uint32_t respTxTime =
        static_cast<uint32_t>((pollRxTs + (range.responseTxDelayUus * detail::kUusToDwtTime)) >> 8);
    const uint64_t respTxTsPlan =
        ((static_cast<uint64_t>(respTxTime & 0xFFFFFFFEUL)) << 8) + _impl->tx_antenna_delay;

    // 【タスクC-2】Response ペイロードにもプリセットの版/種別を載せる
    // (docs/TIMING_PRESETS.md §3.2: "DWR" 11バイト -> 13バイト)。
    uint8_t respPayload[13] = {'D', 'W', 'R'};
    detail::set32le(&respPayload[3], static_cast<uint32_t>(pollRxTs));
    detail::set32le(&respPayload[7], static_cast<uint32_t>(respTxTsPlan));
    respPayload[11] = kTimingPresetVersion;
    respPayload[12] = static_cast<uint8_t>(_impl->config.timing_profile);

    uint8_t respFrame[22] = {0};
    detail::buildShortAddressFrame(respFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src,
                                    respPayload, sizeof(respPayload));

    dwt_setdelayedtrxtime(respTxTime);
    dwt_setrxaftertxdelay(range.finalRxAfterResponseTxDelayUus);
    dwt_setrxtimeout(range.rxTimeoutUus);
    // 【R9】Final待ち受け直前のプリアンブル検出タイムアウト。0=無効。
    // PRETOCはRXが開放された時刻を起点に走るタイマーであり（本ファイル
    // 冒頭R9コメント参照）、本実装はRXをTagのFinal送信予定より早く開ける
    // 設計（PollingBoth: Response送信後3000 UUSでRX開始）なので、公式
    // ex_05b_ds_twr_resp/ds_twr_responder.c:90,200-203のPRE_TIMEOUT=5を
    // そのまま設定すると必ずRXPTOで失敗する。Poll待ち（本関数冒頭、
    // 相手がいつ来るか分からない開放待ち）も同じ理由で0のまま据え置く。
    dwt_setpreambledetecttimeout(0);
    if (dwt_writetxdata(sizeof(respFrame), respFrame, 0) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxDataFailed;
        setError(result.error);
        return result;
    }
    dwt_writetxfctrl(sizeof(respFrame) + FCS_LEN, 0, 1);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS) {
        detail::stopRadioAndClearIoStatus();
        result.error = Error::TxStartFailed;
        setError(result.error);
        return result;
    }
    // DWT_RESPONSE_EXPECTED によりTX完了後ただちにRXが開始される。受信待ち
    // ループへ入る直前に、取りこぼした通知を捨てる（本ファイル冒頭コメント
    // 参照）。IRQが無効なら何もしない。
    uwb_port_irq_clear_pending();

    uint8_t finalFrame[32]      = {0};
    uint16_t finalLen           = 0;
    RxResult finalParsed;
    bool finalMatched            = false;
    bool finalMismatchSeen       = false; // R2診断用
    const uint32_t finalStartMs = detail::nowMs();
    while ((detail::nowMs() - finalStartMs) < range.hostTimeoutMs) {
        const uint32_t status = dwt_readsysstatuslo();
        if ((status & DWT_INT_RXFCG_BIT_MASK) != 0) {
            uint8_t rangingBit = 0;
            finalLen           = dwt_getframelength(&rangingBit);
            if (finalLen > sizeof(finalFrame)) {
                finalLen = sizeof(finalFrame);
            }
            dwt_readrxdata(finalFrame, finalLen, 0);
            dwt_writesysstatuslo(SYS_STATUS_ALL_RX_GOOD | DWT_INT_TXFRS_BIT_MASK);

            const bool headerOk  = detail::parseShortAddressFrame(finalFrame, finalLen, finalParsed);
            const bool payloadOk = detail::payloadMatches(finalFrame, finalLen, "DWF", 3, 15);
            const detail::ParsedFrameSummary summary{
                headerOk, payloadOk, finalParsed.sequence, finalParsed.panId, finalParsed.src, finalParsed.dst};
            // 期待するsequence/srcは、この応答が誰宛てだったか（先に受信した
            // Pollの送信元）に紐づく動的な値。元コードのfinalParsed.sequence
            // != parsed.sequence / finalParsed.src != parsed.src と同じ。
            const detail::FrameExpectation expect{/*checkSequence=*/true, parsed.sequence, range.panId, parsed.src,
                                                   range.responderAddress};
            if (detail::frameMatchesExpectation(summary, expect)) {
                finalMatched = true;
            } else {
                // 【R2】不一致 -> 受信継続。
                finalMismatchSeen = true;
                dwt_rxenable(DWT_START_RX_IMMEDIATE);
            }
        } else if ((status & (SYS_STATUS_ALL_RX_TO | SYS_STATUS_ALL_RX_ERR)) != 0) {
            detail::stopRadioAndClearRxStatus();
            result.sequence  = parsed.sequence;
            result.requester = parsed.src;
            result.elapsedMs = detail::nowMs() - startMs;
            result.error     = detail::rxStatusToError(status);
            setError(result.error);
            return result;
        }
        if (finalMatched) {
            break;
        }
        // IRQが来るか1ms経つまで待つ。IRQ無効時はvTaskDelay(pdMS_TO_TICKS(1))
        // と完全に等価（本ファイル冒頭コメント参照）。
        (void)uwb_port_irq_wait(1);
    }

    if (!finalMatched) {
        detail::stopRadioAndClearRxStatus();
        result.sequence  = finalParsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = finalMismatchSeen ? Error::RangeFrameMismatch : Error::RxTimeout;
        setError(result.error);
        return result;
    }

    const uint64_t finalRxTs   = detail::readRxTimestamp64();
    const uint32_t pollTxTs32  = detail::get32le(&finalFrame[12]);
    const uint32_t respRxTs32  = detail::get32le(&finalFrame[16]);
    const uint32_t finalTxTs32 = detail::get32le(&finalFrame[20]);
    const uint32_t pollRxTs32  = static_cast<uint32_t>(pollRxTs);
    const uint32_t respTxTs32  = static_cast<uint32_t>(detail::readTxTimestamp64());
    const uint32_t finalRxTs32 = static_cast<uint32_t>(finalRxTs);

    const double ra          = static_cast<double>(static_cast<uint32_t>(respRxTs32 - pollTxTs32));
    const double rb          = static_cast<double>(static_cast<uint32_t>(finalRxTs32 - respTxTs32));
    const double da          = static_cast<double>(static_cast<uint32_t>(finalTxTs32 - respRxTs32));
    const double db          = static_cast<double>(static_cast<uint32_t>(respTxTs32 - pollRxTs32));
    const double denominator = ra + rb + da + db;
    if (denominator <= 0.0) {
        result.sequence  = parsed.sequence;
        result.requester = parsed.src;
        result.elapsedMs = detail::nowMs() - startMs;
        result.error     = Error::RangeTimestampInvalid;
        setError(result.error);
        return result;
    }

    const double tofDtu = ((ra * rb) - (da * db)) / denominator;
    const double tof    = tofDtu * DWT_TIME_UNITS;
    result.distanceM    = static_cast<float>(tof * detail::kSpeedOfLightMPerS);
    result.distanceMm   = static_cast<int32_t>((result.distanceM * 1000.0f) + (result.distanceM >= 0 ? 0.5f : -0.5f));

    uint8_t distPayload[7] = {'D', 'W', 'D'};
    detail::set32le(&distPayload[3], static_cast<uint32_t>(result.distanceMm));
    uint8_t distFrame[16] = {0};
    detail::buildShortAddressFrame(distFrame, parsed.sequence, range.panId, range.responderAddress, parsed.src,
                                    distPayload, sizeof(distPayload));

    uint8_t sentCount         = 0;
    const uint8_t repeatCount = range.resultRepeatCount == 0 ? 1 : range.resultRepeatCount;
    for (uint8_t i = 0; i < repeatCount; ++i) {
        dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
        if (dwt_writetxdata(sizeof(distFrame), distFrame, 0) != DWT_SUCCESS) {
            continue;
        }
        dwt_writetxfctrl(sizeof(distFrame) + FCS_LEN, 0, 0);
        if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS) {
            detail::stopRadioAndClearTxStatus();
            continue;
        }

        bool txDone              = false;
        const uint32_t txStartMs = detail::nowMs();
        while ((detail::nowMs() - txStartMs) < 20) {
            if ((dwt_readsysstatuslo() & DWT_INT_TXFRS_BIT_MASK) != 0) {
                dwt_writesysstatuslo(DWT_INT_TXFRS_BIT_MASK);
                txDone = true;
                sentCount++;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!txDone) {
            detail::stopRadioAndClearTxStatus();
        }
        if (i + 1 < repeatCount) {
            vTaskDelay(pdMS_TO_TICKS(range.resultRepeatGapMs));
        }
    }

    result.success   = sentCount > 0;
    result.sequence  = parsed.sequence;
    result.requester = parsed.src;
    result.elapsedMs = detail::nowMs() - startMs;
    result.error     = result.success ? Error::Ok : Error::TxTimeout;
    setError(result.error);
    return result;
}

} // namespace uwb
