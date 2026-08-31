/**
 * @file uwb_cfgstore_blob.cpp
 * @brief uwb_cfgstore_blob.hpp の実装。**ハード非依存**（ESP-IDF に依存しない）。
 *
 * ホスト（tests/host/pipeline）でも同じソースをそのままコンパイルして
 * 往復・境界値・破損データの検算に使う。実機が無い段階で検証できるのは
 * この層まで、という切り分け。
 */
#include "uwb_cfgstore_blob.hpp"

#include <cmath>
#include <cstring>

namespace uwb {
namespace cfg {

namespace {

/** u16 をリトルエンディアンで書く。 */
void put16(uint8_t* p, uint16_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

/** u32 をリトルエンディアンで書く。 */
void put32(uint8_t* p, uint32_t v)
{
    p[0] = static_cast<uint8_t>(v & 0xFFu);
    p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
    p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
    p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

/** リトルエンディアンの u16 を読む。 */
uint16_t get16(const uint8_t* p)
{
    return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8));
}

/** リトルエンディアンの u32 を読む。 */
uint32_t get32(const uint8_t* p)
{
    return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
           (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

/** float を IEEE754 単精度のビットパターンとしてリトルエンディアンで書く。
 *  型パニングは memcpy 経由で行う（union やポインタキャストは strict aliasing
 *  違反になりうるため）。 */
void putFloat(uint8_t* p, float v)
{
    uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    put32(p, bits);
}

/** putFloat() の逆。 */
float getFloat(const uint8_t* p)
{
    const uint32_t bits = get32(p);
    float v             = 0.0f;
    std::memcpy(&v, &bits, sizeof(v));
    return v;
}

/** 有限（NaN でも Inf でもない）かつ絶対値が limit 以下か。 */
bool inRange(float v, float limit)
{
    if (!std::isfinite(v)) {
        return false;
    }
    return (v >= -limit) && (v <= limit);
}

/**
 * @brief 共通ヘッダを書く。
 * @param kind  種別
 * @param count 要素数（種別ごとの意味）
 */
void writeHeader(uint8_t* out, BlobKind kind, uint16_t count)
{
    put32(out + 0, kBlobMagic);
    put16(out + 4, kBlobVersion);
    put16(out + 6, static_cast<uint16_t>(kind));
    put16(out + 8, count);
    put16(out + 10, 0);
}

/**
 * @brief 共通ヘッダと末尾 CRC を検査する。
 *
 * @param data        入力
 * @param len         入力長
 * @param expectKind  期待する種別
 * @param expectedLen 0 以外なら、この長さと一致しなければ BadLength
 * @param outCount    ヘッダの要素数を書く（nullptr 可）
 *
 * 検査順序に意味がある: 長さ → マジック → 版 → 種別 → CRC。
 * **未初期化の NVS を読んだとき（全 0 や 0xFF）に BadMagic が返る**ように
 * マジックを先に見る。CRC を先に見ると、たまたま一致した場合に意味の無い
 * 値を採用してしまう。
 */
BlobStatus checkHeader(const uint8_t* data, size_t len, BlobKind expectKind, size_t expectedLen,
                        uint16_t* outCount)
{
    if (data == nullptr) {
        return BlobStatus::NullArg;
    }
    if (len < kBlobHeaderSize + kBlobCrcSize) {
        return BlobStatus::TooShort;
    }
    if (get32(data + 0) != kBlobMagic) {
        return BlobStatus::BadMagic;
    }
    if (get16(data + 4) != kBlobVersion) {
        return BlobStatus::BadVersion;
    }
    if (get16(data + 6) != static_cast<uint16_t>(expectKind)) {
        return BlobStatus::BadKind;
    }
    if (expectedLen != 0 && len != expectedLen) {
        return BlobStatus::BadLength;
    }

    const uint32_t storedCrc   = get32(data + len - kBlobCrcSize);
    const uint32_t computedCrc = crc32(data, len - kBlobCrcSize);
    if (storedCrc != computedCrc) {
        return BlobStatus::BadCrc;
    }

    if (outCount != nullptr) {
        *outCount = get16(data + 8);
    }
    return BlobStatus::Ok;
}

} // namespace

const char* blobStatusName(BlobStatus status)
{
    switch (status) {
    case BlobStatus::Ok:
        return "ok";
    case BlobStatus::TooShort:
        return "too_short";
    case BlobStatus::BadMagic:
        return "bad_magic";
    case BlobStatus::BadVersion:
        return "bad_version";
    case BlobStatus::BadKind:
        return "bad_kind";
    case BlobStatus::BadCount:
        return "bad_count";
    case BlobStatus::BadLength:
        return "bad_length";
    case BlobStatus::BadCrc:
        return "bad_crc";
    case BlobStatus::BadEntry:
        return "bad_entry";
    case BlobStatus::NullArg:
        return "null_arg";
    }
    return "unknown";
}

uint32_t crc32(const uint8_t* data, size_t len)
{
    if (data == nullptr) {
        return 0;
    }
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint32_t>(data[i]);
        for (int bit = 0; bit < 8; ++bit) {
            const uint32_t mask = static_cast<uint32_t>(-static_cast<int32_t>(crc & 1u));
            crc                  = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return ~crc;
}

bool isValidShortAddr(uint16_t addr)
{
    return addr != kBroadcastShortAddr;
}

bool isValidAnchorEntry(const AnchorEntry& entry)
{
    if (!isValidShortAddr(entry.short_addr)) {
        return false;
    }
    for (int i = 0; i < 3; ++i) {
        if (!inRange(entry.pos[i], kMaxCoordM)) {
            return false;
        }
    }
    return inRange(entry.antenna_delay_m, kMaxAntennaDelayM);
}

BlobStatus serializeAnchorTable(const AnchorEntry* entries, size_t count, uint8_t* out, size_t outCap,
                                 size_t* outLen)
{
    if (entries == nullptr || out == nullptr) {
        return BlobStatus::NullArg;
    }
    if (count == 0 || count > kMaxAnchors) {
        return BlobStatus::BadCount;
    }
    const size_t need = anchorTableBlobSize(count);
    if (outCap < need) {
        return BlobStatus::TooShort;
    }
    // 壊れた値を NVS に残さないため、書き込み時にも検査する。
    for (size_t i = 0; i < count; ++i) {
        if (!isValidAnchorEntry(entries[i])) {
            return BlobStatus::BadEntry;
        }
    }

    writeHeader(out, BlobKind::AnchorTable, static_cast<uint16_t>(count));
    for (size_t i = 0; i < count; ++i) {
        uint8_t* p             = out + kBlobHeaderSize + kAnchorEntrySize * i;
        const AnchorEntry& src = entries[i];
        put16(p + 0, src.short_addr);
        p[2] = src.enabled ? 1u : 0u;
        p[3] = 0;
        putFloat(p + 4, src.pos[0]);
        putFloat(p + 8, src.pos[1]);
        putFloat(p + 12, src.pos[2]);
        putFloat(p + 16, src.antenna_delay_m);
    }
    put32(out + need - kBlobCrcSize, crc32(out, need - kBlobCrcSize));

    if (outLen != nullptr) {
        *outLen = need;
    }
    return BlobStatus::Ok;
}

BlobStatus deserializeAnchorTable(const uint8_t* data, size_t len, AnchorEntry* out, size_t outCap,
                                   size_t* outCount)
{
    if (out == nullptr) {
        return BlobStatus::NullArg;
    }

    // 件数はヘッダにしか無いので、先に長さ非依存の検査だけ通してから
    // 件数を読み、そのうえで長さ整合と CRC を確かめる。
    uint16_t count           = 0;
    const BlobStatus headerOk = checkHeader(data, len, BlobKind::AnchorTable, 0, &count);
    if (headerOk != BlobStatus::Ok) {
        return headerOk;
    }
    if (count == 0 || count > kMaxAnchors) {
        return BlobStatus::BadCount;
    }
    if (len != anchorTableBlobSize(count)) {
        return BlobStatus::BadLength;
    }
    if (outCap < count) {
        return BlobStatus::TooShort;
    }

    for (size_t i = 0; i < count; ++i) {
        const uint8_t* p = data + kBlobHeaderSize + kAnchorEntrySize * i;
        AnchorEntry e;
        e.short_addr = get16(p + 0);
        if (p[2] > 1u) {
            // CRC が通っているのに 0/1 以外なら、書いた側のバグか
            // 想定外のフォーマット。既定値へ倒す。
            return BlobStatus::BadEntry;
        }
        e.enabled         = (p[2] != 0);
        e.pos[0]          = getFloat(p + 4);
        e.pos[1]          = getFloat(p + 8);
        e.pos[2]          = getFloat(p + 12);
        e.antenna_delay_m = getFloat(p + 16);
        if (!isValidAnchorEntry(e)) {
            return BlobStatus::BadEntry;
        }
        out[i] = e;
    }

    if (outCount != nullptr) {
        *outCount = count;
    }
    return BlobStatus::Ok;
}

BlobStatus serializeAnchorAddr(uint16_t addr, uint8_t* out, size_t outCap, size_t* outLen)
{
    if (out == nullptr) {
        return BlobStatus::NullArg;
    }
    if (!isValidShortAddr(addr)) {
        return BlobStatus::BadEntry;
    }
    const size_t need = anchorAddrBlobSize();
    if (outCap < need) {
        return BlobStatus::TooShort;
    }

    writeHeader(out, BlobKind::AnchorAddr, 1);
    put16(out + kBlobHeaderSize + 0, addr);
    put16(out + kBlobHeaderSize + 2, 0);
    put32(out + need - kBlobCrcSize, crc32(out, need - kBlobCrcSize));

    if (outLen != nullptr) {
        *outLen = need;
    }
    return BlobStatus::Ok;
}

BlobStatus deserializeAnchorAddr(const uint8_t* data, size_t len, uint16_t* outAddr)
{
    if (outAddr == nullptr) {
        return BlobStatus::NullArg;
    }

    uint16_t count            = 0;
    const BlobStatus headerOk = checkHeader(data, len, BlobKind::AnchorAddr, anchorAddrBlobSize(), &count);
    if (headerOk != BlobStatus::Ok) {
        return headerOk;
    }
    if (count != 1) {
        return BlobStatus::BadCount;
    }

    const uint16_t addr = get16(data + kBlobHeaderSize + 0);
    if (!isValidShortAddr(addr)) {
        return BlobStatus::BadEntry;
    }
    *outAddr = addr;
    return BlobStatus::Ok;
}

bool isValidModeOverrideCode(uint8_t code)
{
    return code == kModeOverrideAuto || code == kModeOverrideForce2D || code == kModeOverrideForce3D;
}

BlobStatus serializePositioningMode(uint8_t overrideCode, float zFixedM, uint8_t* out, size_t outCap,
                                     size_t* outLen)
{
    if (out == nullptr) {
        return BlobStatus::NullArg;
    }
    // 壊れた値をNVSに残さないため、書き込み時にも検査する
    // （isValidAnchorEntry()と同じ方針。uwb_cfgstore_blob.hpp冒頭コメント参照）。
    if (!isValidModeOverrideCode(overrideCode) || !inRange(zFixedM, kMaxZFixedM)) {
        return BlobStatus::BadEntry;
    }
    const size_t need = positioningModeBlobSize();
    if (outCap < need) {
        return BlobStatus::TooShort;
    }

    writeHeader(out, BlobKind::PositioningMode, 1); // count=1（常に1件のレコード。AnchorAddrと同じ流儀）
    out[kBlobHeaderSize + 0] = overrideCode;
    out[kBlobHeaderSize + 1] = 0;
    put16(out + kBlobHeaderSize + 2, 0);
    putFloat(out + kBlobHeaderSize + 4, zFixedM);
    put32(out + need - kBlobCrcSize, crc32(out, need - kBlobCrcSize));

    if (outLen != nullptr) {
        *outLen = need;
    }
    return BlobStatus::Ok;
}

BlobStatus deserializePositioningMode(const uint8_t* data, size_t len, uint8_t* outOverrideCode, float* outZFixedM)
{
    if (outOverrideCode == nullptr || outZFixedM == nullptr) {
        return BlobStatus::NullArg;
    }

    uint16_t count            = 0;
    const BlobStatus headerOk = checkHeader(data, len, BlobKind::PositioningMode, positioningModeBlobSize(), &count);
    if (headerOk != BlobStatus::Ok) {
        return headerOk;
    }
    if (count != 1) {
        return BlobStatus::BadCount;
    }

    const uint8_t overrideCode = data[kBlobHeaderSize + 0];
    const float zFixedM         = getFloat(data + kBlobHeaderSize + 4);
    if (!isValidModeOverrideCode(overrideCode) || !inRange(zFixedM, kMaxZFixedM)) {
        return BlobStatus::BadEntry;
    }

    *outOverrideCode = overrideCode;
    *outZFixedM       = zFixedM;
    return BlobStatus::Ok;
}

} // namespace cfg
} // namespace uwb
