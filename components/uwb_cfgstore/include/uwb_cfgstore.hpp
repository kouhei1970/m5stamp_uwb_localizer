/**
 * @file uwb_cfgstore.hpp
 * @brief NVS に設定を永続化する設定ストア（タグ／アンカー共用）。
 *
 * 実機で最も繰り返す作業（アンカー 5 台に別々のショートアドレスを入れる、
 * 設置後にアンカー座標を数 cm 直す）を、**再ビルド・再書き込み無しで**
 * できるようにするための層。実際の書き換え口はシリアルコンソール
 * （firmware/anchor/main/anchor_console.cpp,
 *  firmware/tag/main/tag_console.cpp）で、本クラスはその保存先を提供する。
 *
 * 設計上の約束（**壊れても必ず起動すること**が最優先）:
 *
 *   - load 系は**常に成功する**。NVS が空・未初期化・破損・版違い・
 *     CRC 不一致・値が異常、いずれの場合も引数で渡された
 *     **コンパイル時の既定値**を出力に詰めて返る。返り値はどちらを
 *     採用したかを示すだけで、エラーコードではない。
 *   - したがって購入者が何も設定しなければ、従来どおり Kconfig
 *     （アンカー）や kAnchors[]（タグ）の値でそのまま動く（後方互換）。
 *   - init() が失敗した場合も同様で、以後 load 系は既定値を返し続ける。
 *     save 系だけが ESP_ERR_INVALID_STATE を返す。
 *
 * バイト列の形式そのものと検査は uwb_cfgstore_blob.hpp 側にある
 * （ESP-IDF 非依存で、ホストのテストから直接叩ける）。
 */
#pragma once

#include "esp_err.h"

#include "uwb_cfgstore_blob.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

/** 設定をどこから読んだか。 */
enum class ConfigSource {
    Default, //!< コンパイル時の既定値（NVS が空・破損・未初期化）
    Nvs,     //!< NVS に保存された値
};

/** ConfigSource の名前（ログや info コマンドの表示用）。 */
const char* configSourceName(ConfigSource source);

/**
 * @brief NVS 上の設定ストア。状態はプロセス全体で 1 つなので全て static。
 */
class ConfigStore {
public:
    /** NVS の名前空間。 */
    static constexpr const char* kNamespace = "uwb_cfg";

    /** アンカー自身のショートアドレスのキー。 */
    static constexpr const char* kKeyAnchorAddr = "anchor_addr";

    /** タグのアンカー登録テーブルのキー。 */
    static constexpr const char* kKeyAnchorTable = "anchor_tbl";

    /**
     * @brief NVS フラッシュを初期化する。app_main の最初のほうで 1 回呼ぶ。
     *
     * ESP_ERR_NVS_NO_FREE_PAGES / ESP_ERR_NVS_NEW_VERSION_FOUND
     * （＝パーティションが埋まっている、または旧版フォーマット）のときは
     * nvs_flash_erase() してから初期化し直す。ESP-IDF の定型処理だが、
     * **これをやらないと NVS 破損で起動しなくなる**ので必ず通す。
     *
     * @return ESP_OK 以外でも呼び出し側は処理を続けてよい（以後は既定値で動く）。
     */
    static esp_err_t init();

    /** init() が成功して NVS が使える状態か。 */
    static bool isReady();

    // ------------------------------------------------- アンカー側（自局アドレス）

    /**
     * @brief 自分のショートアドレスを読む。
     *
     * @param defaultAddr NVS に無い/壊れている場合に使う既定値
     *                    （Kconfig の CONFIG_UWB_ANCHOR_SHORT_ADDR）
     * @param outAddr     結果。**必ず書き込まれる**（nullptr は不可）
     * @return どちらの値を採用したか
     */
    static ConfigSource loadAnchorAddr(uint16_t defaultAddr, uint16_t* outAddr);

    /**
     * @brief 自分のショートアドレスを保存する。
     * @return ESP_OK / ESP_ERR_INVALID_STATE（init 未了）/ ESP_ERR_INVALID_ARG
     *         （0xFFFF はブロードキャストなので不可）/ NVS のエラー
     */
    static esp_err_t saveAnchorAddr(uint16_t addr);

    // ------------------------------------------------- タグ側（アンカー登録テーブル）

    /**
     * @brief アンカー登録テーブルを読む。
     *
     * @param defaults      NVS に無い/壊れている場合に使う既定値（tag の kAnchors[]）
     * @param defaultCount  defaults の件数
     * @param out           結果の書き込み先（outCap 件ぶん）
     * @param outCap        out の容量 [件]
     * @param outCount      復元した件数。**必ず書き込まれる**
     * @return どちらの値を採用したか
     *
     * @note 途中で失敗しても out には既定値が入った状態で返る（部分的に
     *       NVS の値が混ざることはない。復元は一時領域へ行い、全件通った
     *       ときだけ out へ写す）。
     */
    static ConfigSource loadAnchorTable(const AnchorEntry* defaults, size_t defaultCount, AnchorEntry* out,
                                         size_t outCap, size_t* outCount);

    /**
     * @brief アンカー登録テーブルを保存する。
     * @return ESP_OK / ESP_ERR_INVALID_STATE / ESP_ERR_INVALID_ARG（値が異常）/ NVS のエラー
     */
    static esp_err_t saveAnchorTable(const AnchorEntry* entries, size_t count);

    // ------------------------------------------------------------------ 消去

    /**
     * @brief 本コンポーネントの名前空間の内容を全消去する（工場出荷状態）。
     *
     * 次回起動時（および直後の load 系）は必ずコンパイル時の既定値になる。
     * 他コンポーネントが使う NVS のキーには触らない。
     */
    static esp_err_t eraseAll();

private:
    static bool ready_;
};

} // namespace uwb
