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
 * ホスト側テスト (tests/host/pipeline) から include できない。ここに切り出した
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
 * --- 単位について（docs/archive/REIMPL_PLAN.md R1） ---
 * 詳細は docs/GLOSSARY.md。
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
     * 【docs/archive/REIMPL_PLAN.md R9】旧既定値100はチップ側のRXタイムアウト
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
     * オーダー、docs/archive/CRITICAL_REVIEW.md「フレーム air time の実測算」参照）
     * を何十回分も吸収できる余裕を残す。100msの1/10にすることで
     * 「5アンカー×hostTimeoutMs」のストール上限も同じ比率で縮む。
     */
    uint32_t hostTimeoutMs             = 10;
    /**
     * 【docs/archive/REIMPL_PLAN.md R4】requestRange()（SS-TWR initiator）のToF計算に
     * dwt_readclockoffset() によるクロックオフセット補正を適用するか。
     * 既定で有効（true）。無効にすると旧挙動（無補正、rtd_resp係数=1固定）
     * に戻る。実機での比較検証用のフラグ（respondRange 側には効果はない。
     * SS-TWR responder は補正の主体ではなくフレームにタイムスタンプを
     * 載せるだけなので変更不要）。詳細は uwb_qm33120_twr.cpp
     * requestRange() 内のコメント参照。
     */
    bool enableClockOffsetCorrection    = true;

    /**
     * 【修正2】docs/archive/REVIEW_2026-08-21.md TWR層M2。ms。
     * respondRange()（Anchor）が **最初のフレーム（Poll）を待つ**
     * 受信ループだけに使うホスト側タイムアウト。requestRange()
     * （Tag、Response待ち）は hostTimeoutMs の方を使い続け、この
     * フィールドの影響を受けない（末尾に追加。既存の公開APIは壊さない）。
     *
     * firmware/anchor/main/main.cpp の runRole() は、respondRange() が
     * Error::RxTimeout（まだPollが来ていないだけ）を返すたびに即
     * continue して respondRange() を呼び直す。呼び直しの都度
     * dwt_rxenable(DWT_START_RX_IMMEDIATE) するまでの一瞬、受信機が
     * 実質「聞いていない」窓ができる。hostTimeoutMs（既定10ms）のように
     * 短い値だと、この「聞いていない窓」が来る頻度が上がり、Poll を
     * 取りこぼす確率が無視できなくなる（Anchorはどのタイミングで
     * Pollが来るか分からず待つ側なので、Response/Final待ちのように
     * 「既にシーケンスに入っていて次のフレームの到着時刻がほぼ予測できる」
     * 局面とは事情が異なる）。既定を200msへ伸ばし、呼び直しの頻度を
     * 下げることで取りこぼしを減らす。
     *
     * 【測距の遅延プリセット（UUS値）は一切変更していない】これは
     * ホスト側（ESP32側）のポーリングループの上限時間であり、
     * SDK APIには渡らない（hostTimeoutMs と同じ性質。フィールド
     * コメント参照）。responseTxDelayUus等のUUS値は不変。
     */
    uint32_t pollHostTimeoutMs          = 200;

    /**
     * 【2026-08-29 DS-TWR原因特定、docs/HANDOFF.md §0-C(1)】true にすると、
     * respondRange()/respondDSRange() の Poll 待ちループが「タイムアウト
     * (RXFTO/RXPTO) ではない RX エラー」を見た**最初の1回**で
     * forcetrxoff() して即 return する旧挙動に戻る（2026-08-29 より前は
     * これが唯一の挙動だった）。既定 false は、RXエラーを
     * dwt_writesysstatuslo()でクリアして dwt_rxenable(DWT_START_RX_IMMEDIATE)
     * するだけで受信を継続し、pollHostTimeoutMs を使い切るまで Poll を
     * 待ち続ける（Qorvo公式 ex_05b_ds_twr_resp / ex_06b_ss_twr_resp の
     * 受信ループと同じ構造。両者ともRXエラーで抜けず、ステータスを
     * クリアして即座に受信を再開する）。
     *
     * 【なぜ既定を変えたか】旧挙動（true相当）は、Poll待ちの
     * 200msの窓と、タグ側の測距周期（既定200ms、tick=1ms）が両方とも
     * 1msティックに量子化されているため、一度 Poll がこの「窓の境界での
     * forcetrxoff→再オープンの数百µsの空白」に落ちると、以後ずっと
     * そこに留まり続ける（周期が完全に一致している限り位相がずれない）。
     * RXエラーで即座に打ち切って再オープンする挙動そのものが、この
     * 空白を毎回作り出していた。窓の再位相合わせは「Poll待ちがRXエラー
     * で戻ったとき」にしか起きなかったため、この2つが組み合わさって
     * DS-TWRだけが位相ロックしていた（SS-TWRのrespondRange()も同じ
     * ループ構造だが、タグ側の周期が異なりRXエラー自体の発生頻度も
     * 違うため、実測ではSS-TWRは99.95%・DS-TWRは0〜23%という非対称な
     * 結果になっていた。docs/HANDOFF.md §0-C(1)参照）。
     *
     * If true, restores the pre-2026-08-29 behaviour: the Poll wait
     * returns Error::RxError on the first non-timeout RX error instead of
     * absorbing it and continuing to listen. Default false matches Qorvo's
     * own ex_05b/ex_06b responder examples, which never abort the receive
     * loop on an RX error.
     */
    bool pollWaitReturnOnRxError         = false;
};

/**
 * @brief Double-sided two-way ranging timing and address configuration.
 * 1:1 with M5Stamp_UWBDSRangeConfig (M5Stamp_UWB_Types.h:197-210), including
 * default values.
 *
 * --- 単位について（docs/archive/REIMPL_PLAN.md R1。RangeConfig と同じ規則） ---
 * 詳細は docs/GLOSSARY.md。
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
    //!<
    //!< 【2026-08-30実機確認】同様の注意: 本フィールドもdwt_setrxaftertxdelay()
    //!< 基準で自分のPoll送信完了（末尾）起点。responseTxDelayUusはPollの
    //!< RMARKER起点で、両者はPoll残り伝送時間分ずれる（850kbps/16Bで
    //!< ≈202µs、6.8Mbpsで≈41µs。詳細はfinalRxAfterResponseTxDelayUus参照）。
    //!< Same caveat as finalRxAfterResponseTxDelayUus: counts from the END
    //!< of our own Poll TX, while responseTxDelayUus counts from the Poll's
    //!< RMARKER instead - offset by ~202us (850kbps/16B) / ~41us (6.8Mbps)
    //!< of remaining on-air time (docs/TIMING_PRESETS.md section 2.4).
    uint32_t responseRxAfterTxDelayUus      = 1500;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Responder側がPoll受信時刻を基準に自分のResponse遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: respondDSRange:499,511）。
    uint32_t responseTxDelayUus             = 3000;
    //!< UUS。× kUusToDwtTime してDTU化し dwt_setdelayedtrxtime() に渡る。
    //!< Initiator側がResponse受信時刻を基準に自分のFinal遅延送信時刻を計算
    //!< するのに使う（uwb_qm33120_twr.cpp: requestDSRange:359,371）。
    //!<
    //!< 【2026-08-29 DS-TWR原因特定、docs/HANDOFF.md §0-C(2)、1800→3000】
    //!< 旧値1800 UUS(≈1846µs実us)は、850kbps/preamble256（本番機の実運用値。
    //!< 旧値は6.8Mbps/preamble128前提のまま流用されていた）では、タグが
    //!< Finalを起動する時点で締切まで0.03〜1.0msしか残っておらず、
    //!< DW3000 UM §9.4.1エラッタ（予約時刻が「現在+プリアンブル長+SFD長+
    //!< 20µs」未満だとHPDWARNすら立たず無警告で送信されない。SYS_STATE_LO
    //!< が0x000D0000＝PMSC_STATE=TX/TX_STATE=IDLEのまま固まる）を頻繁に
    //!< 踏んでいた（DS-TWR実測0〜23%、SS-TWRは同条件で99.95%）。
    //!< Response側（responseTxDelayUus=3000、上のフィールド）は同じ
    //!< 850kbps/256で1.3〜2.3msの余裕があり実証済みなので、Final側も
    //!< 同じ3000 UUSに揃えて対称にした。詳細な再導出（フレーム長・
    //!< エラッタの締切式）は docs/TIMING_PRESETS.md の2026-08-29追記を参照。
    uint32_t finalTxDelayUus                = 3000;
    //!< UUS。dwt_setrxaftertxdelay() に直接渡る。Responder側が自分の
    //!< 遅延Response送信後、Final受信を開始するまでの待ち
    //!< （uwb_qm33120_twr.cpp: respondDSRange:512）。
    //!<
    //!< 【2026-08-29 DS-TWR原因特定、500→1500】finalTxDelayUusを3000へ
    //!< 揃えたのに合わせ、アンカーのFinal受信窓もresponseRxAfterTxDelayUus
    //!< （上、1500）と同じ1500 UUSにして対称にした。窓が開くのは
    //!< Response送信終了+1500 UUS(≈1.54ms)後、Finalのプリアンブル到達は
    //!< Response送信終了+3000 UUS−179µs実us(≈2.90ms)後なので、窓は
    //!< プリアンブル到達より先に開く（§1.2の締切不等式を満たす）。
    //!< 窓の終わり（RXFTOまで、rxTimeoutUus=3000UUS）はFinal末尾
    //!< （Response送信終了+3000+SHR+PHR+データ、約3.36ms後）を覆う必要が
    //!< あり、実際に覆っている（docs/TIMING_PRESETS.md参照。UMはRX_FWTOが
    //!< フレーム受信中も数え続け、フレームの途中で打ち切りうると明記して
    //!< いるため、窓はフレーム全体を覆わなければならない）。
    //!<
    //!< 【2026-08-30実機確認】注意: 本フィールドはdwt_setrxaftertxdelay()
    //!< 基準で自分のResponse送信完了（末尾）起点。finalTxDelayUusは
    //!< ResponseのRMARKER起点で、両者はResponse残り伝送時間分ずれる
    //!< （850kbps/24Bで≈267µs、6.8Mbpsで≈41µs。混同するとFinal全滅、e30）。
    //!< Counts from the END of our own Response TX (dwt_setrxaftertxdelay()),
    //!< while finalTxDelayUus counts from the Response's RMARKER instead -
    //!< offset by ~267us (850kbps/24B) / ~41us (6.8Mbps) of remaining
    //!< on-air time. Confusing the two silently drops every Final (e30).
    uint32_t finalRxAfterResponseTxDelayUus = 1500;
    /**
     * UUS。dwt_setrxaftertxdelay() に直接渡る。Initiator側がFinal送信後、
     * 結果("DWD")受信を開始するまでの待ち
     * （uwb_qm33120_twr.cpp: requestDSRange:372）。
     *
     * 【docs/archive/REIMPL_PLAN.md R3-1】旧既定値500 UUS(≈512.8µs実us)は、
     * DWD結果フレームの送信がAnchor側で「delayed TXで時刻を予約する」
     * のではなく「Final受信直後にDWT_START_TX_IMMEDIATEで即時送信する」
     * 方式（respondDSRange()、SPIレジスタ書き込み数回のみでvTaskDelay等の
     * 意図的な遅延を挟まない）であるにもかかわらず大きすぎた。
     * Initiator側のRXが512.8µs後まで開かないため、Anchorがそれより速く
     * 返信した場合は最初のDWD送信を取りこぼす
     * （docs/archive/CRITICAL_REVIEW.md【重大3】、L<363µsで約36%取りこぼしの実測算）。
     * 200 UUS(≈205.1µs実us)へ下げた根拠: DWDフレーム自体の空中線上の
     * 全長（SHR 138.4µs [preamble128+SFD8=136シンボル×1017.63ns/シンボル、
     * docs/refs/DW3000_Datasheet_wayback.txt Table 18（§4.4）より本値を
     * 独立に再検算し一致を確認] + PHR + データ、docs/archive/CRITICAL_REVIEW.md
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
     * 【docs/archive/REIMPL_PLAN.md R9】RangeConfig::hostTimeoutMs と同じ理由・
     * 同じ根拠で100→10に変更（コメント参照）。DS-TWRのrxTimeoutUus=3000
     * (≈3.077ms)に対しても10msは1回分丸ごと待っても打ち切らない値。
     *
     * 【2026-08-29 DS-TWR原因特定、10→20】respondDSRange()のFinal待ち
     * ループはこのフィールドを使う（本構造体コメント冒頭参照）。Final受信
     * 窓はfinalRxAfterResponseTxDelayUus(=1500 UUS)後に開き、
     * rxTimeoutUus(=3000 UUS)後にハードウェアRXFTOで自動的に閉じるので、
     * Response送信終了を基準にすると窓が閉じるのは1500+3000=4500 UUS
     * (≈4.6ms)後——さらにResponseの遅延送信自体がPoll受信から3000 UUS
     * (≈3.1ms)後なので、Final待ちループが開始する（Response遅延送信を
     * 予約した直後の）ホスト時刻を基準にすると、ハードウェアRXFTOは
     * 合計で約7.5ms後に立つ。旧値10msはこのハード側タイムアウトの前に
     * ホスト側タイムアウトが先に切れることは無かった（7.5ms<10ms）が、
     * 余裕が2.5msしか無く、ログ出力等でホスト側のタイミングにジッタが
     * 乗ると逆転しうる。ホスト
     * タイムアウトが先に切れると forcetrxoff() が測距シーケンスの途中
     * （Finalがまだ空中を飛んでいる最中かもしれない時刻）で発火し、
     * 受信機を無用に止めてしまう。20msへ伸ばして余裕を確保する
     * （SS-TWR側のhostTimeoutMsは変更しない。理由は同フィールドの
     * コメント参照）。
     */
    uint32_t hostTimeoutMs                  = 20;
    /**
     * 単位なし（回数）。SDK APIには渡らない。Responder が計算した距離
     * ("DWD"結果フレーム)を送る回数（uwb_qm33120_twr.cpp: respondDSRange:603）。
     *
     * 【docs/archive/REIMPL_PLAN.md R3-1】旧既定値3は、resultRepeatGapMs=3ms /
     * rxTimeoutUus=3000UUS(=3.077ms)と一度も整合していなかった
     * (docs/archive/CRITICAL_REVIEW.md【重大3】)。DWD#3はTagのDWD受信ウィンドウが
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

    /**
     * 【修正2】docs/archive/REVIEW_2026-08-21.md TWR層M2。ms。
     * respondDSRange()（Anchor）が **最初のフレーム（Poll）を待つ**
     * 受信ループ（本関数冒頭のループ。finalStartMs から始まる
     * Final待ちループとは別物）だけに使うホスト側タイムアウト。
     * requestDSRange()（Tag、Response/Final・結果フレーム待ち）と
     * respondDSRange() 自身のFinal待ちは引き続き hostTimeoutMs を使い、
     * このフィールドの影響を受けない（既にシーケンスに入っていて次の
     * フレームの到着時刻がほぼ予測できる局面は変更しない。末尾に追加。
     * 既存の公開APIは壊さない）。
     *
     * RangeConfig::pollHostTimeoutMs と同じ理由・同じ既定値（200ms）。
     * 詳細はそちらのフィールドコメント参照。
     *
     * 【測距の遅延プリセット（UUS値）は一切変更していない】これは
     * ホスト側（ESP32側）のポーリングループの上限時間であり、
     * SDK APIには渡らない（hostTimeoutMs と同じ性質）。
     */
    uint32_t pollHostTimeoutMs              = 200;

    /**
     * 【2026-08-29 DS-TWR原因特定、docs/HANDOFF.md §0-C(1)】
     * RangeConfig::pollWaitReturnOnRxError と同じフィールド・同じ既定値
     * (false)・同じ理由。respondDSRange() の Poll 待ちループに適用される
     * （こちらのフィールドコメント参照）。
     */
    bool pollWaitReturnOnRxError            = false;
};

/**
 * @brief RangeConfig（SS-TWR）へ遅延プリセットを適用する（docs/TIMING_PRESETS.md、
 * タスクB）。responseTxDelayUus/responseRxAfterTxDelayUus/rxTimeoutUus の
 * 3フィールドだけを書き換える。panId/initiatorAddress/responderAddress/
 * hostTimeoutMs/enableClockOffsetCorrection/pollHostTimeoutMs（修正2で追加）
 * /pollWaitReturnOnRxError（2026-08-29追加）には一切触れない
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
 * hostTimeoutMs/resultRepeatCount/resultRepeatGapMs/pollHostTimeoutMs
 * （修正2で追加）/pollWaitReturnOnRxError（2026-08-29追加）には一切触れない。
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
