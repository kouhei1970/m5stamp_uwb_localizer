/**
 * @file uwb_cfgstore.cpp
 * @brief uwb::ConfigStore の実装（NVS 入出力。ESP-IDF 依存）。
 *
 * バイト列の組み立て・検査そのものは uwb_cfgstore_blob.cpp（ハード非依存、
 * ホストで検算済み）に任せ、ここは nvs_open / nvs_get_blob / nvs_set_blob と
 * 「失敗したら既定値へ倒す」判断だけを担当する。
 */
#include "uwb_cfgstore.hpp"

#include <cstring>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

namespace uwb {

namespace {
const char* kLogTag = "uwb_cfgstore";
} // namespace

bool ConfigStore::ready_ = false;

const char* configSourceName(ConfigSource source)
{
    return (source == ConfigSource::Nvs) ? "nvs" : "default";
}

esp_err_t ConfigStore::init()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // パーティションが埋まっている / 旧版フォーマット。ここで消さないと
        // 以後 NVS が一切使えない（起動時の定型処理）。設定は失われるが、
        // 失われても既定値で動くのが本コンポーネントの前提。
        ESP_LOGW(kLogTag, "NVS を初期化できないため消去してやり直します (err=%s)", esp_err_to_name(err));
        const esp_err_t eraseErr = nvs_flash_erase();
        if (eraseErr != ESP_OK) {
            ESP_LOGE(kLogTag, "nvs_flash_erase() 失敗 (err=%s)", esp_err_to_name(eraseErr));
        }
        err = nvs_flash_init();
    }

    ready_ = (err == ESP_OK);
    if (!ready_) {
        ESP_LOGE(kLogTag,
                 "NVS を使えません (err=%s)。設定はすべてコンパイル時の既定値を使い、"
                 "保存は失敗しますが動作は継続します",
                 esp_err_to_name(err));
    }
    return err;
}

bool ConfigStore::isReady()
{
    return ready_;
}

/* --------------------------------------------------------------------------
 * アンカー側: 自局ショートアドレス
 * -------------------------------------------------------------------------- */

ConfigSource ConfigStore::loadAnchorAddr(uint16_t defaultAddr, uint16_t* outAddr)
{
    if (outAddr == nullptr) {
        return ConfigSource::Default;
    }
    // 何があっても既定値が入った状態で返れるよう、先に既定値を入れておく。
    *outAddr = defaultAddr;

    if (!ready_) {
        return ConfigSource::Default;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // ESP_ERR_NVS_NOT_FOUND = 名前空間がまだ無い（＝未設定）。正常系。
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kLogTag, "nvs_open() 失敗 (err=%s) 既定値を使います", esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    uint8_t buf[cfg::anchorAddrBlobSize()];
    size_t len = sizeof(buf);
    err         = nvs_get_blob(handle, kKeyAnchorAddr, buf, &len);
    nvs_close(handle);

    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kLogTag, "nvs_get_blob(%s) 失敗 (err=%s) 既定値を使います", kKeyAnchorAddr,
                     esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    uint16_t addr             = 0;
    const cfg::BlobStatus st = cfg::deserializeAnchorAddr(buf, len, &addr);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGW(kLogTag, "保存されたショートアドレスが不正 (%s) 既定値 0x%04X を使います",
                 cfg::blobStatusName(st), static_cast<unsigned>(defaultAddr));
        return ConfigSource::Default;
    }

    *outAddr = addr;
    return ConfigSource::Nvs;
}

esp_err_t ConfigStore::saveAnchorAddr(uint16_t addr)
{
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[cfg::anchorAddrBlobSize()];
    size_t len                = 0;
    const cfg::BlobStatus st = cfg::serializeAnchorAddr(addr, buf, sizeof(buf), &len);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGE(kLogTag, "ショートアドレスをシリアライズできません (%s)", cfg::blobStatusName(st));
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, kKeyAnchorAddr, buf, len);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* --------------------------------------------------------------------------
 * タグ側: アンカー登録テーブル
 * -------------------------------------------------------------------------- */

ConfigSource ConfigStore::loadAnchorTable(const AnchorEntry* defaults, size_t defaultCount,
                                           AnchorEntry* out, size_t outCap, size_t* outCount)
{
    if (out == nullptr || outCount == nullptr) {
        return ConfigSource::Default;
    }

    // まず既定値を詰める。以降どこで失敗しても、この状態のまま返る。
    size_t nDefault = 0;
    if (defaults != nullptr) {
        nDefault = (defaultCount < outCap) ? defaultCount : outCap;
        for (size_t i = 0; i < nDefault; ++i) {
            out[i] = defaults[i];
        }
    }
    *outCount = nDefault;

    if (!ready_) {
        return ConfigSource::Default;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kLogTag, "nvs_open() 失敗 (err=%s) 既定値を使います", esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    uint8_t buf[cfg::kMaxBlobSize];
    size_t len = sizeof(buf);
    err         = nvs_get_blob(handle, kKeyAnchorTable, buf, &len);
    nvs_close(handle);

    if (err != ESP_OK) {
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            // ESP_ERR_NVS_INVALID_LENGTH（保存されている blob が buf より
            // 大きい）もここに来る。テーブルの上限が変わったときなど。
            ESP_LOGW(kLogTag, "nvs_get_blob(%s) 失敗 (err=%s) 既定値を使います", kKeyAnchorTable,
                     esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    // 一時領域へ復元し、**全件通ったときだけ** out へ写す。途中で弾かれた
    // 場合に NVS の値と既定値が混ざるのを防ぐ。
    AnchorEntry tmp[kMaxAnchors];
    size_t count              = 0;
    const cfg::BlobStatus st = cfg::deserializeAnchorTable(buf, len, tmp, kMaxAnchors, &count);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGW(kLogTag, "保存されたアンカーテーブルが不正 (%s) 既定値 %u 件を使います",
                 cfg::blobStatusName(st), static_cast<unsigned>(nDefault));
        return ConfigSource::Default;
    }
    if (count > outCap) {
        ESP_LOGW(kLogTag, "保存されたアンカーテーブルが %u 件で受け側の容量 %u 件を超えています。既定値を使います",
                 static_cast<unsigned>(count), static_cast<unsigned>(outCap));
        return ConfigSource::Default;
    }

    for (size_t i = 0; i < count; ++i) {
        out[i] = tmp[i];
    }
    *outCount = count;
    return ConfigSource::Nvs;
}

esp_err_t ConfigStore::saveAnchorTable(const AnchorEntry* entries, size_t count)
{
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[cfg::kMaxBlobSize];
    size_t len                = 0;
    const cfg::BlobStatus st = cfg::serializeAnchorTable(entries, count, buf, sizeof(buf), &len);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGE(kLogTag, "アンカーテーブルをシリアライズできません (%s)", cfg::blobStatusName(st));
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, kKeyAnchorTable, buf, len);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* --------------------------------------------------------------------------
 * タグ側: 測位モード（手動オーバーライド + 2D固定高さ）
 * -------------------------------------------------------------------------- */

ConfigSource ConfigStore::loadPositioningMode(ModeOverride defaultOverride, float defaultZFixedM,
                                               ModeOverride* outOverride, float* outZFixedM)
{
    if (outOverride == nullptr || outZFixedM == nullptr) {
        return ConfigSource::Default;
    }
    // 何があっても既定値が入った状態で返れるよう、先に既定値を入れておく。
    *outOverride = defaultOverride;
    *outZFixedM  = defaultZFixedM;

    if (!ready_) {
        return ConfigSource::Default;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READONLY, &handle);
    if (err != ESP_OK) {
        // ESP_ERR_NVS_NOT_FOUND = 名前空間がまだ無い（＝未設定、またはこの機能
        // 追加前のNVS）。正常系。
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kLogTag, "nvs_open() 失敗 (err=%s) 既定値を使います", esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    uint8_t buf[cfg::positioningModeBlobSize()];
    size_t len = sizeof(buf);
    err         = nvs_get_blob(handle, kKeyPositioningMode, buf, &len);
    nvs_close(handle);

    if (err != ESP_OK) {
        // ESP_ERR_NVS_NOT_FOUND = キー自体が無い。この機能追加前のNVSから
        // 起動した場合は必ずここを通り、既定値（Auto/Kconfig値）へ倒れる。
        if (err != ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(kLogTag, "nvs_get_blob(%s) 失敗 (err=%s) 既定値を使います", kKeyPositioningMode,
                     esp_err_to_name(err));
        }
        return ConfigSource::Default;
    }

    uint8_t overrideCode      = 0;
    float zFixedM              = 0.0f;
    const cfg::BlobStatus st = cfg::deserializePositioningMode(buf, len, &overrideCode, &zFixedM);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGW(kLogTag, "保存された測位モード設定が不正 (%s) 既定値を使います", cfg::blobStatusName(st));
        return ConfigSource::Default;
    }

    *outOverride = static_cast<ModeOverride>(overrideCode);
    *outZFixedM  = zFixedM;
    return ConfigSource::Nvs;
}

esp_err_t ConfigStore::savePositioningMode(ModeOverride override, float zFixedM)
{
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t buf[cfg::positioningModeBlobSize()];
    size_t len                = 0;
    const cfg::BlobStatus st = cfg::serializePositioningMode(static_cast<uint8_t>(override), zFixedM, buf,
                                                                sizeof(buf), &len);
    if (st != cfg::BlobStatus::Ok) {
        ESP_LOGE(kLogTag, "測位モード設定をシリアライズできません (%s)", cfg::blobStatusName(st));
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(handle, kKeyPositioningMode, buf, len);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

/* -------------------------------------------------------------------------- */

esp_err_t ConfigStore::eraseAll()
{
    if (!ready_) {
        return ESP_ERR_INVALID_STATE;
    }

    nvs_handle_t handle = 0;
    esp_err_t err        = nvs_open(kNamespace, NVS_READWRITE, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        // まだ何も書かれていない。既に工場出荷状態なので成功扱い。
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_erase_all(handle);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

} // namespace uwb
