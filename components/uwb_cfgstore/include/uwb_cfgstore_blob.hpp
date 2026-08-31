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
 *
 * 種別 BlobKind::PositioningMode の本体 = 8 バイト
 *
 *   オフセット  幅  内容
 *   0           1   測位モードの手動オーバーライド（kModeOverrideAuto/Force2D/Force3D）
 *   1           1   予約（0）
 *   2           2   予約（0）
 *   4           4   2D測位の固定高さ z_fixed_m [m]（IEEE754 単精度のビットパターン）
 *
 *   AnchorTable/AnchorAddr とは独立のキー（NVS上も別の名前空間キー）に
 *   保存する。この機能を追加する前のNVS（このキー自体が存在しない）から
 *   起動した場合は ConfigStore::loadPositioningMode() が既定値
 *   （override=Auto、z_fixed=Kconfig UWB_TAG_FIXED_Z_MM）へフォールバックする
 *   ので、AnchorTable 側のフォーマット・キーには一切手を入れていない
 *   （後方互換は「新しいキーが無ければ既定値」という既存の設計をそのまま踏襲）。
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

/** PositioningMode 種別の本体長 [バイト]。 */
inline constexpr size_t kPositioningModeBodySize = 8;

/** バイト列の種別。 */
enum class BlobKind : uint16_t {
    AnchorTable    = 1, //!< タグ側: アンカー登録テーブル
    AnchorAddr     = 2, //!< アンカー側: 自分のショートアドレス
    PositioningMode = 3, //!< タグ側: 測位モードの手動オーバーライド + 2D固定高さ
};

/** 測位モードの手動オーバーライドのコード（NVSブロブでの符号化）。
 *  uwb::ModeOverride（components/uwb_ranging/include/uwb_ranging_mode.hpp）
 *  と1対1に対応する。cfgstore のこの層をハード非依存かつ uwb_ranging_mode.hpp
 *  非依存に保つため、列挙型そのものではなく raw な uint8_t として扱う
 *  （ESP-IDF依存側の uwb_cfgstore.cpp が ModeOverride との変換を行う）。 */
inline constexpr uint8_t kModeOverrideAuto    = 0;
inline constexpr uint8_t kModeOverrideForce2D = 1;
inline constexpr uint8_t kModeOverrideForce3D = 2;

/** 2D固定高さ(z_fixed)として受け付ける絶対値の上限 [m]。
 *  Kconfig UWB_TAG_FIXED_Z_MM の範囲（±10000mm）と揃える。 */
inline constexpr float kMaxZFixedM = 10.0f;

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

/** PositioningMode 種別のバイト列全体の長さ [バイト]。 */
inline constexpr size_t positioningModeBlobSize()
{
    return kBlobHeaderSize + kPositioningModeBodySize + kBlobCrcSize;
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

/**
 * @brief 測位モードの手動オーバーライド + 2D固定高さをバイト列にする。
 *
 * @param overrideCode kModeOverrideAuto/Force2D/Force3D のいずれか
 * @param zFixedM      2D固定高さ [m]。override の値に関わらず常に保存する
 *                      （auto判定が同一平面フォールバックでMode2Dへ切り替わった
 *                      ときにも使われるため）
 * @return Ok / NullArg / BadEntry（overrideCodeが未知の値、または zFixedM が
 *         非有限値・範囲外）/ TooShort
 */
BlobStatus serializePositioningMode(uint8_t overrideCode, float zFixedM, uint8_t* out, size_t outCap,
                                     size_t* outLen);

/**
 * @brief バイト列を測位モードの手動オーバーライド + 2D固定高さへ戻す。
 * @return Ok 以外なら out* は書き換えられない。呼び出し側は既定値
 *         （override=Auto、zFixedM=Kconfig UWB_TAG_FIXED_Z_MM）へフォールバック
 *         すること。
 */
BlobStatus deserializePositioningMode(const uint8_t* data, size_t len, uint8_t* outOverrideCode, float* outZFixedM);

/** overrideCode が既知の値（kModeOverrideAuto/Force2D/Force3D）か。 */
bool isValidModeOverrideCode(uint8_t code);

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
