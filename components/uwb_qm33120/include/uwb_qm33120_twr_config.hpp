/**
 * @file uwb_qm33120_twr_config.hpp
 * @brief TWR（SS-TWR / DS-TWR）のタイミング・アドレス設定構造体
 * (uwb::RangeConfig / uwb::DSRangeConfig) と、遅延プリセット適用関数
 * (uwb::applyTimingProfile()) の実体。
 *
 * uwb_qm33120_units.hpp / uwb_qm33120_timing.hpp / uwb_qm33120_frame_match.hpp
 * と同じ方針で、このヘッダは ESP-IDF / Qorvo SDK ヘッダに依存しない
 * （<cstdint> と uwb_qm33120_timing.hpp のみ）。uwb_qm33120_types.hpp は
 * uwb_port.h 経由で driver/spi_master.h (ESP-IDF) を引き込むため、そのままでは
 * ホスト側テスト (tools/test_pipeline) から include できない。ここに切り出した
 * 構造体・関数は固定幅整数/bool のフィールドと純粋な代入のみで ESP-IDF/Qorvo
 * SDK の型を一切使わないため、ホストでビルド・検算できる。
 *
 * uwb_qm33120_types.hpp（同ディレクトリ）はこのヘッダを #include するだけの
 * 薄い転送になっている。uwb::RangeConfig / uwb::DSRangeConfig /
 * uwb::applyTimingProfile() の実体・コメントはこちらが正であり、
 * uwb:: 名前空間・型名・シグネチャは types.hpp 側から見て一切変わらない。
 */
#pragma once

#include <cstdint>

#include "uwb_qm33120_timing.hpp"

namespace uwb {

/**
 * @brief Single-sided two-way ranging timing and address configuration.
 * 1:1 with M5Stamp_UWBRangeConfig (M5Stamp_UWB_Types.h:184-192), including
 * default values.
 *
 * --- 単位について（docs/REIMPL_PLAN.md R1） ---
 * 詳細は docs/UNITS.md。
 * `*Uus` フィールドの単位は UUS (UWB microsecond)。
 * 1 UUS = 512/499.2 us = 1.02564... us
 * (`deca_device_api.h:2681` dwt_setrxaftertxdelay() 「The delay is in UWB
 * microseconds」/ 同2360行 dwt_setrxtimeout() 「in 1.0256 us
 * (512/499.2MHz) units」より)。以下2フィールドはこの単位のまま SDK API に
 * 直接渡り、`responseTxDelayUus` だけ `uwb::detail::kUusToDwtTime`(=65536,
 * uwb_qm33120_internal.hpp) 倍して DTU (1 DTU=15.65ps) に変換してから
 * `dwt_setdelayedtrxtime()` へ渡す（uwb_qm33120_twr.cpp 参照。関数ごとの
 * 行はフィールドコメント参照）。
 * 【罠】Qorvo公式サンプルの `*_UUS` 定数（例:
 * `POLL_RX_TO_RESP_TX_DLY_UUS`）は実マイクロ秒であり、このUUSとは値が違う
 * （65536/63898≒1.0256倍）。そのまま代入すると約2.5%長くなる。移植する
 * ときは `uwb::detail::usToUus()`（uwb_qm33120_units.hpp）で変換すること。
 */
struct RangeConfig {
    uint16_t panId                     = 0xDECA;
    uint16_t initiatorAddress          = 0x0001;
    uint16_t responderAddress          = 0x0002;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:76, respondRange は即時応答なので0固定:174)。
    uint32_t responseRxAfterTxDelayUus = 500;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:214,226 で応答側の遅延送信時刻を計算)。
    uint32_t responseTxDelayUus        = 3000;
    //!< UUS。dwt_setrxtimeout() に直接渡る
    //!< (uwb_qm33120_twr.cpp: requestRange:77, respondRange は即時応答なので0固定:175)。
    uint32_t rxTimeoutUus              = 4500;
    /**
     * ms。ホスト側ポーリングループのタイムアウト。SDK APIには渡らない
     * (detail::nowMs() ベース。requestRange/respondRange 双方で使用)。
     *
     * 【docs/REIMPL_PLAN.md R9】旧既定値100はチップ側のRXタイムアウト
     * (rxTimeoutUus=4500UUS≈4.615ms)の20倍以上あり、5アンカー構成では
     * 「応答が来ない」ケース1つで最大100msストールしうる。
     * 【R2適用後の根拠】R2により、不一致フレームを受信するたびに
     * dwt_rxenable()でRXタイムアウトが再アームされる（本ファイルの
     * uwb_qm33120_twr.cppコメント、ull_setrxtimeout()の解析を参照）ため、
     * hostTimeoutMsは「チップ側1回分のRXタイムアウト」ではなく「不一致
     * フレームが何度再アームされても最終的に抜けるための上位バックストップ」
     * として機能する。10msは、チップ側の最大RXタイムアウト(4500UUS≈4.615ms)
     * を1回分丸ごと待っても打ち切らない値でありながら、不一致フレーム1枚
     * あたりの処理時間（フレーム受信+照合、air timeにして高々百数十µs
     * オーダー、docs/CRITICAL_REVIEW.md「フレーム air time の実測算」参照）
     * を何十回分も吸収できる余裕を残す。100msの1/10にすることで
     * 「5アンカー×hostTimeoutMs」のストール上限も同じ比率で縮む。
     */
    uint32_t hostTimeoutMs             = 10;
    /**
     * 【docs/REIMPL_PLAN.md R4】requestRange()（SS-TWR initiator）のToF計算に
     * dwt_readclockoffset() によるクロックオフセット補正を適用するか。
     * 既定で有効（true）。無効にすると旧挙動（無補正、rtd_resp係数=1固定）
     * に戻る。実機での比較検証用のフラグ（respondRange 側には効果はない。
     * SS-TWR responder は補正の主体ではなくフレームにタイムスタンプを
     * 載せるだけなので変更不要）。詳細は uwb_qm33120_twr.cpp
     * requestRange() 内のコメント参照。
     */
    bool enableClockOffsetCorrection    = true;
};

/**
 * @brief Double-sided two-way ranging timing and address configuration.
 * 1:1 with M5Stamp_UWBDSRangeConfig (M5Stamp_UWB_Types.h:197-210), including
 * default values.
 *
 * --- 単位について（docs/REIMPL_PLAN.md R1。RangeConfig と同じ規則） ---
 * 詳細は docs/UNITS.md。
 * `*Uus` フィールドの単位は UUS。1 UUS = 512/499.2 us = 1.02564... us。
 * `responseTxDelayUus` / `finalTxDelayUus` の2つだけが
 * `uwb::detail::kUusToDwtTime`(=65536) 倍されて DTU に変換され
 * `dwt_setdelayedtrxtime()` へ渡る。残りは UUS のまま
 * `dwt_setrxaftertxdelay()` / `dwt_setrxtimeout()` に直接渡る。
 * Qorvo公式の `*_UUS` 定数（実マイクロ秒）をそのまま代入しないこと
 * （`uwb::detail::usToUus()` で変換する。RangeConfig のコメント参照）。
 */
struct DSRangeConfig {
    uint16_t panId                          = 0xDECA;
    uint16_t initiatorAddress               = 0x0001;
    uint16_t responderAddress               = 0x0002;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る。Poll送信後、Response受信を
    //!< 開始するまでの待ち（uwb_qm33120_twr.cpp: requestDSRange:301）。
    uint32_t responseRxAfterTxDelayUus      = 1500;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Responder側がPoll受信時刻を基準に自分のResponse遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: respondDSRange:499,511）。
    uint32_t responseTxDelayUus             = 3000;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Initiator側がResponse受信時刻を基準に自分のFinal遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: requestDSRange:359,371）。
    uint32_t finalTxDelayUus                = 1800;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る。Responder側が自分の
    //!< 遅延Response送信後、Final受信を開始するまでの待ち
    //!< （uwb_qm33120_twr.cpp: respondDSRange:512）。
    uint32_t finalRxAfterResponseTxDelayUus = 500;
    /**
     * UUS。dwt_setrxaftertxdelay() に直接渡る。Initiator側がFinal送信後、
     * 結果("DWD")受信を開始するまでの待ち
     * （uwb_qm33120_twr.cpp: requestDSRange:372）。
     *
     * 【docs/REIMPL_PLAN.md R3-1】旧既定値500 UUS(≈512.8µs実us)は、
     * DWD結果フレームの送信がAnchor側で「delayed TXで時刻を予約する」
     * のではなく「Final受信直後にDWT_START_TX_IMMEDIATEで即時送信する」
     * 方式（respondDSRange()、SPIレジスタ書き込み数回のみでvTaskDelay等の
     * 意図的な遅延を挟まない）であるにもかかわらず大きすぎた。
     * Initiator側のRXが512.8µs後まで開かないため、Anchorがそれより速く
     * 返信した場合は最初のDWD送信を取りこぼす
     * （docs/CRITICAL_REVIEW.md【重大3】、L<363µsで約36%取りこぼしの実測算）。
     * 200 UUS(≈205.1µs実us)へ下げた根拠: DWDフレーム自体の空中線上の
     * 全長（SHR 138.4µs [preamble128+SFD8=136シンボル×1017.63ns/シンボル、
     * docs/refs/DW3000_Datasheet_wayback.txt Table 18（§4.4）より本値を
     * 独立に再検算し一致を確認] + PHR + データ、docs/CRITICAL_REVIEW.md
     * 「フレーム air time の実測算」より合計約179µs）をわずかに上回る
     * 205.1µsであれば、Anchor側の実処理時間（意図的な遅延なしの
     * SPIレジスタ書き込み数回、既存の1ms輪ポーリング実装でも数十〜百数十µs
     * オーダーと見積もれる）に対して十分な余裕がありながら、
     * 旧値(512.8µs)の半分以下に圧縮できる。
     */
    uint32_t resultRxAfterFinalTxDelayUus   = 200;
    //!< UUS。dwt_setrxtimeout() に直接渡る。Response/Final/結果それぞれの
    //!< 受信タイムアウトに共用される
    //!< （uwb_qm33120_twr.cpp: requestDSRange:302,373, respondDSRange:513）。
    uint32_t rxTimeoutUus                   = 3000;
    /**
     * ms。ホスト側ポーリングループのタイムアウト。SDK APIには渡らない。
     * 【docs/REIMPL_PLAN.md R9】RangeConfig::hostTimeoutMs と同じ理由・
     * 同じ根拠で100→10に変更（コメント参照）。DS-TWRのrxTimeoutUus=3000
     * (≈3.077ms)に対しても10msは1回分丸ごと待っても打ち切らない値。
     */
    uint32_t hostTimeoutMs                  = 10;
    /**
     * 単位なし（回数）。SDK APIには渡らない。Responder が計算した距離
     * ("DWD"結果フレーム)を送る回数（uwb_qm33120_twr.cpp: respondDSRange:603）。
     *
     * 【docs/REIMPL_PLAN.md R3-1】旧既定値3は、resultRepeatGapMs=3ms /
     * rxTimeoutUus=3000UUS(=3.077ms)と一度も整合していなかった
     * (docs/CRITICAL_REVIEW.md【重大3】)。DWD#3はTagのDWD受信ウィンドウが
     * 閉じた後(ウィンドウ終端の2〜3.5ms後)に送信されるため物理的に受信
     * 不能な上、次のAnchorの応答窓(8.6〜11.7ms)のど真ん中に着弾して
     * 次のリンクの測距を巻き添えにする。既定を1にしてこの破壊的な
     * リピートを止める。リピート機構自体は削除しない（respondDSRange()の
     * for(i<repeatCount)ループはそのまま残っており、呼び出し側がこの
     * フィールドを2以上に設定すれば実機比較検証用に旧挙動へ戻せる。
     * firmware/anchor, firmware/twr は現状Kconfig化しておらず
     * main.cpp内のstatic constexprで直接指定している）。
     */
    uint8_t resultRepeatCount               = 1;
    //!< ms。SDK APIには渡らない。結果フレーム再送の間隔
    //!< （uwb_qm33120_twr.cpp: respondDSRange:630, vTaskDelay）。
    //!< resultRepeatCount>1のときのみ意味を持つ（R3-1で既定1になったため
    //!< 既定構成では未使用）。
    uint32_t resultRepeatGapMs              = 3;
};

/**
 * @brief RangeConfig（SS-TWR）へ遅延プリセットを適用する（docs/TIMING_PRESETS.md、
 * タスクB）。responseTxDelayUus/responseRxAfterTxDelayUus/rxTimeoutUus の
 * 3フィールドだけを書き換える。panId/initiatorAddress/responderAddress/
 * hostTimeoutMs/enableClockOffsetCorrection には一切触れない
 * （呼び出し側がこれらを設定した後に呼ぶこと。RangeConfig単体にはプリセット
 * 適用の有無を記録するフィールドは無いため、呼び出す順序は呼び出し側の責務）。
 */
inline void applyTimingProfile(RangeConfig& cfg, TimingProfile p)
{
    const TimingPresetSs preset  = timingPresetSs(p);
    cfg.responseTxDelayUus        = preset.responseTxDelayUus;
    cfg.responseRxAfterTxDelayUus = preset.responseRxAfterTxDelayUus;
    cfg.rxTimeoutUus              = preset.rxTimeoutUus;
}

/**
 * @brief DSRangeConfig（DS-TWR）へ遅延プリセットを適用する（docs/TIMING_PRESETS.md、
 * タスクB）。responseTxDelayUus/responseRxAfterTxDelayUus/finalTxDelayUus/
 * finalRxAfterResponseTxDelayUus/resultRxAfterFinalTxDelayUus/rxTimeoutUus の
 * 6フィールドだけを書き換える。panId/initiatorAddress/responderAddress/
 * hostTimeoutMs/resultRepeatCount/resultRepeatGapMs には一切触れない。
 */
inline void applyTimingProfile(DSRangeConfig& cfg, TimingProfile p)
{
    const TimingPresetDs preset          = timingPresetDs(p);
    cfg.responseTxDelayUus                = preset.responseTxDelayUus;
    cfg.responseRxAfterTxDelayUus         = preset.responseRxAfterTxDelayUus;
    cfg.finalTxDelayUus                   = preset.finalTxDelayUus;
    cfg.finalRxAfterResponseTxDelayUus    = preset.finalRxAfterResponseTxDelayUus;
    cfg.resultRxAfterFinalTxDelayUus      = preset.resultRxAfterFinalTxDelayUus;
    cfg.rxTimeoutUus                      = preset.rxTimeoutUus;
}

} // namespace uwb
