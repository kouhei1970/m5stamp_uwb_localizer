/**
 * @file uwb_cfgstore_blob.hpp
 * @brief 設定のバイト列表現（シリアライズ/デシリアライズ）。**ハード非依存**。
 *
 * NVS へ書き込む前のバイト列を作る部分と、読み出したバイト列を検査して
 * 構造体へ戻す部分だけをここに切り出してある。ESP-IDF / FreeRTOS / NVS の
 * ヘッダには一切依存しないので、ホスト（tests/host/pipeline）でそのまま
 * コンパイルして往復テスト・境界値テスト・壊れたデータのテストができる。
 * 実機が無い段階で検算できるのはこの層まで、という切り分けでもある。
 *
 * NVS 側の入出力（nvs_open / nvs_get_blob / nvs_set_blob）は
 * uwb_cfgstore.hpp / src/uwb_cfgstore.cpp が担当する。
 *
 * 依存するのは uwb_ranging_types.hpp（uwb::AnchorEntry / uwb::kMaxAnchors）
 * のみで、そちらも uwb_loc.h にしか依存しない。
 *
 * ------------------------------------------------------------------
 * バイト列の形式（すべてリトルエンディアン。ESP32-S3 もホストの x86/ARM も
 * リトルエンディアン IEEE754 だが、実装は明示的にバイト単位で組み立てる）
 * ------------------------------------------------------------------
 *
 * 共通ヘッダ（12 バイト）
 *
 *   オフセット  幅  内容
 *   0           4   マジック "UCFG" (0x55 0x43 0x46 0x47)
 *   4           2   フォーマット版（kBlobVersion）
 *   6           2   種別（BlobKind）
 *   8           2   要素数（種別ごとの意味。AnchorAddr では常に 1）
 *   10          2   予約（0）
 *
 * 本体（種別ごと）＋ 末尾 4 バイトの CRC32（先頭から CRC 直前までが対象）
 *
 * 種別 BlobKind::AnchorTable の本体 = 1 件 20 バイト × 要素数
 *
 *   オフセット  幅  内容
 *   0           2   short_addr
 *   2           1   enabled（0 または 1。それ以外は破損とみなす）
 *   3           1   予約（0）
 *   4           4   pos[0] [m]（IEEE754 単精度のビットパターン）
 *   8           4   pos[1] [m]
 *   12          4   pos[2] [m]
 *   16          4   antenna_delay_m [m]
 *
 * 種別 BlobKind::AnchorAddr の本体 = 4 バイト
 *
 *   オフセット  幅  内容
 *   0           2   ショートアドレス
 *   2           2   予約（0）
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "uwb_ranging_types.hpp"

namespace uwb {
namespace cfg {

/** バイト列の先頭 4 バイト（"UCFG" をリトルエンディアンの u32 として見た値）。 */
inline constexpr uint32_t kBlobMagic = 0x47464355u;

/** フォーマット版。互換性を壊す変更をしたら上げる（読み出し側は不一致を
 *  拒否して既定値へフォールバックする＝古い設定は捨てられるが起動はする）。 */
inline constexpr uint16_t kBlobVersion = 1;

/** 共通ヘッダの長さ [バイト]。 */
inline constexpr size_t kBlobHeaderSize = 12;

/** 末尾 CRC32 の長さ [バイト]。 */
inline constexpr size_t kBlobCrcSize = 4;

/** AnchorTable 種別の 1 エントリの長さ [バイト]。 */
inline constexpr size_t kAnchorEntrySize = 20;

/** AnchorAddr 種別の本体長 [バイト]。 */
inline constexpr size_t kAnchorAddrBodySize = 4;

/** バイト列の種別。 */
enum class BlobKind : uint16_t {
    AnchorTable = 1, //!< タグ側: アンカー登録テーブル
    AnchorAddr  = 2, //!< アンカー側: 自分のショートアドレス
};

/** シリアライズ/デシリアライズの結果。Ok 以外はすべて「既定値へ
 *  フォールバックする」以外の選択肢が無い状態を表す。 */
enum class BlobStatus {
    Ok = 0,
    TooShort,   //!< 出力バッファが足りない、または入力がヘッダ+CRCより短い
    BadMagic,   //!< 先頭 4 バイトが "UCFG" でない（未初期化の NVS 等）
    BadVersion, //!< 既知のフォーマット版でない
    BadKind,    //!< 期待した種別でない
    BadCount,   //!< 要素数が 0 または kMaxAnchors 超
    BadLength,  //!< 要素数から計算される長さと実際の長さが一致しない
    BadCrc,     //!< CRC32 不一致（ビット化け・途中で切れた書き込み）
    BadEntry,   //!< 個々の値が異常（NaN/Inf、範囲外、ブロードキャストアドレス等）
    NullArg,    //!< 引数が nullptr
};

/** BlobStatus の名前（ログ出力用）。未知の値でも "unknown" を返し nullptr は返さない。 */
const char* blobStatusName(BlobStatus status);

/** 座標として受け付ける絶対値の上限 [m]。これを超える値は破損とみなす。
 *  屋内測位の用途では 10km もあれば十分で、NaN/Inf や桁化けを弾ける。 */
inline constexpr float kMaxCoordM = 10000.0f;

/** アンテナ遅延オフセットとして受け付ける絶対値の上限 [m]。
 *  実際の校正値は ±1m 未満のはずだが、余裕を見て 100m とする。 */
inline constexpr float kMaxAntennaDelayM = 100.0f;

/** ブロードキャストアドレス。自局アドレス・アンカーアドレスには使えない。 */
inline constexpr uint16_t kBroadcastShortAddr = 0xFFFF;

/**
 * @brief CRC-32/ISO-HDLC（いわゆる zlib の crc32。多項式 0xEDB88320、
 * 初期値 0xFFFFFFFF、最後に反転）をビット単位で計算する。
 *
 * 数百バイトしか流さないのでテーブルは持たない。ESP-IDF の esp_crc32_le()
 * を使わないのは、この層をホストでもコンパイルできるようにするため。
 */
uint32_t crc32(const uint8_t* data, size_t len);

// ----------------------------------------------------------------- 長さ計算

/** AnchorTable 種別のバイト列全体の長さ [バイト]。 */
inline constexpr size_t anchorTableBlobSize(size_t count)
{
    return kBlobHeaderSize + kAnchorEntrySize * count + kBlobCrcSize;
}

/** AnchorAddr 種別のバイト列全体の長さ [バイト]。 */
inline constexpr size_t anchorAddrBlobSize()
{
    return kBlobHeaderSize + kAnchorAddrBodySize + kBlobCrcSize;
}

/** 想定しうる最大のバイト列長（呼び出し側がスタック上に確保する目安）。 */
inline constexpr size_t kMaxBlobSize = kBlobHeaderSize + kAnchorEntrySize * kMaxAnchors + kBlobCrcSize;

// --------------------------------------------------------- シリアライズ

/**
 * @brief アンカー登録テーブルをバイト列にする。
 *
 * @param entries  書き出すエントリ（count 件）
 * @param count    件数。1 以上 kMaxAnchors 以下でなければ BadCount。
 * @param out      出力先
 * @param outCap   out の容量 [バイト]。足りなければ TooShort。
 * @param outLen   実際に書いた長さ [バイト]（nullptr 可）
 * @return Ok / NullArg / BadCount / BadEntry / TooShort
 *
 * @note 値の妥当性検査（NaN、範囲外、ブロードキャストアドレス）は
 *       **書き込み時にも**行う。壊れた値を NVS に残さないため。
 */
BlobStatus serializeAnchorTable(const AnchorEntry* entries, size_t count, uint8_t* out, size_t outCap,
                                 size_t* outLen);

/**
 * @brief バイト列をアンカー登録テーブルへ戻す。
 *
 * @param data      入力バイト列
 * @param len       入力長 [バイト]
 * @param out       出力先（outCap 件ぶんの領域）
 * @param outCap    out の容量 [件]
 * @param outCount  復元した件数（nullptr 可）
 * @return Ok 以外が返ったときは out の内容は不定。呼び出し側は**必ず**
 *         コンパイル時の既定値へフォールバックすること。
 */
BlobStatus deserializeAnchorTable(const uint8_t* data, size_t len, AnchorEntry* out, size_t outCap,
                                   size_t* outCount);

/**
 * @brief アンカー自身のショートアドレスをバイト列にする。
 * @return Ok / NullArg / BadEntry（0xFFFF）/ TooShort
 */
BlobStatus serializeAnchorAddr(uint16_t addr, uint8_t* out, size_t outCap, size_t* outLen);

/**
 * @brief バイト列をアンカー自身のショートアドレスへ戻す。
 * @return Ok 以外なら outAddr は書き換えられない。
 */
BlobStatus deserializeAnchorAddr(const uint8_t* data, size_t len, uint16_t* outAddr);

// --------------------------------------------------------------- 値の検査

/**
 * @brief AnchorEntry 1 件が設定値として妥当か。
 *
 * コンソールからの入力検査にも使う（同じ基準で弾かないと、コンソールでは
 * 通るのに保存すると弾かれる、という不整合が起きる）。
 *
 * 条件: short_addr が 0xFFFF でない / pos[0..2] と antenna_delay_m が
 * 有限値で上限以内。short_addr==0 は「未設定スロット」として許容する
 * （enabled==false のプレースホルダに使う）。
 */
bool isValidAnchorEntry(const AnchorEntry& entry);

/** ショートアドレスとして妥当か（0xFFFF はブロードキャストなので不可）。 */
bool isValidShortAddr(uint16_t addr);

} // namespace cfg
} // namespace uwb
