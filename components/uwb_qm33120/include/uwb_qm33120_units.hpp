/**
 * @file uwb_qm33120_units.hpp
 * @brief 実マイクロ秒(us) <-> UUS(UWB microseconds) の変換ヘルパと、SFD
 * タイムアウト自動計算式。docs/archive/REIMPL_PLAN.md の R1 / R8 で追加。
 * 詳細な単位リファレンスは docs/UNITS.md。
 *
 * このヘッダは ESP-IDF / Qorvo SDK のヘッダに一切依存しない（<cstdint> のみ）。
 * uwb_qm33120_types.hpp は uwb_port.h 経由で driver/spi_master.h (ESP-IDF)
 * を引き込むため、そのままではホスト側テスト (tests/host/pipeline) から
 * インクルードできない。ここに切り出した2関数は純粋な整数演算のみなので
 * ホストでビルド・検算できる — R1「この関数の正しさをホストで検算する
 * テストを足すこと」、R8「プリアンブル長を変えたときに正しい値になることを
 * 確認するホストテストを足すこと」の対象はこのヘッダ。
 *
 * uwb_qm33120_types.hpp からもこのヘッダを include し、
 * uwb::detail::usToUus() を公開ヘッダ経由で使えるようにしてある。
 */
#pragma once

#include <cstdint>

namespace uwb::detail {

/**
 * @brief 実マイクロ秒(us) を UUS(UWB microsecond) へ変換する。
 *
 * 1 UUS = 512/499.2 us = 1.02564... us
 * （`components/qm33120w_sdk/deca_device_api.h:2681` dwt_setrxaftertxdelay()
 * 「The delay is in UWB microseconds, 20-bit value」、同ファイル 2360行
 * dwt_setrxtimeout() 「in 1.0256 us (512/499.2MHz) units」より）。
 *
 * 【注意 / このヘルパが必要な理由】
 * Qorvo 公式サンプルの共有定義 (docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/
 * Src/examples/shared_data/shared_defines.h:37, `UUS_TO_DWT_TIME 63898`) は
 * 「実マイクロ秒 -> DTU」の係数であり、本プロジェクトの
 * `uwb::detail::kUusToDwtTime = 65536`（uwb_qm33120_internal.hpp、
 * 「真の UUS -> DTU」の係数）とは異なる世代・異なる単位の定数である
 * （65536 / 63898 ≒ 1.02564 = ちょうど1 UUSの実us換算と同じ比）。
 * つまり Qorvo サンプルの `*_UUS` という名の定数（例:
 * `POLL_RX_TO_RESP_TX_DLY_UUS`）は実際には「実マイクロ秒」で書かれており、
 * 本APIが期待する UUS ではない。RangeConfig/DSRangeConfig の `*Uus`
 * フィールドへそのまま代入すると、意図した遅延より約2.5%長くなる
 * （docs/archive/REIMPL_PLAN.md R1）。Qorvo の値を移植するときは必ずこの関数を
 * 通すこと。
 *
 * 丸めは四捨五入（+分母の半分してから切り捨て）。
 * 例: usToUus(650) == 634, usToUus(1000) == 975.
 */
constexpr uint32_t usToUus(uint32_t us)
{
    // uus = us / (512/499.2) = us * (499.2/512) = us * 4992/5120
    return (us * 4992u + 2560u) / 5120u;
}

/**
 * @brief SFD タイムアウトの自動計算式（PhyConfig::sfdTimeout の実体）。
 *
 * `currentValue` が非0ならユーザ明示指定としてそのまま返す。0のときだけ
 * Qorvo 推奨式で自動計算する:
 *   SFD timeout = preamble length + 1 + SFD length - PAC length
 * (docs/refs/DW3720_API_Guide_v4.9.pdf §5.2.1、および
 * components/uwb_qm33120/src/uwb_qm33120.cpp の既存コメント
 * "Qorvo recommended formula" と同一の式)。
 *
 * 【docs/archive/REIMPL_PLAN.md R8】既定PHY(preamble128/SFD8/PAC8)では129が正しい
 * 値と一致するが、`preambleLength` を変える（例: Len256）と本来257で
 * あるべきところが129のまま固定されてしまう罠があった。既定を0（自動計算）
 * にし、この式が必ず動くようにする。0をそのままSDKへ渡すとSDK既定の4161に
 * 落ちてしまうため、ラッパ側（uwb_qm33120.cpp の makeSfdTimeout()）が
 * 必ずこの関数で計算してから dwt_config_t::sfdTO に渡す。
 *
 * @param currentValue     PhyConfig::sfdTimeout の現在値。0なら自動計算。
 * @param preambleSymbols  プリアンブル長（シンボル数）。
 * @param sfdLength        SFD長（シンボル数）。
 * @param pacLength        PACサイズ（シンボル数）。
 */
constexpr uint16_t sfdTimeoutFromPhy(uint16_t currentValue, uint16_t preambleSymbols, uint16_t sfdLength,
                                      uint16_t pacLength)
{
    return (currentValue != 0) ? currentValue
                                : static_cast<uint16_t>(preambleSymbols + 1 + sfdLength - pacLength);
}

} // namespace uwb::detail
