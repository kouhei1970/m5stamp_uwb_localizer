/**
 * @file uwb_ranging_anchor_table.hpp
 * @brief アンカー登録テーブル（ハード非依存）。
 *
 * uwb_loc の `uwb_config.anchors` はポインタ借用であり、`uwb_config_init()`
 * はコピーを取らない（docs/archive/PROGRESS.md Phase 4 Step 1 で確認済みの仕様）。
 * このクラスは AnchorEntry の配列から `uwb_anchor` の安定した記憶域
 * （このオブジェクトが生きている限り有効な配列）を作り、`uwb_config` を
 * その記憶域を指すように初期化して保持する。テーブルを差し替えるときは
 * set() を呼び直せばよい（内部で同じ記憶域を書き換えるので `uwb_config` の
 * 再初期化も自動的にやり直される）。
 *
 * ESP-IDF/FreeRTOS には依存しない（uwb_loc.h のみに依存）ので、ホスト
 * （tests/host/pipeline）でもそのままコンパイル・実行できる。
 */
#pragma once

#include "uwb_loc.h"
#include "uwb_ranging_mode.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

class AnchorTable {
public:
    AnchorTable();

    /**
     * @brief テーブルを丸ごと差し替える。
     *
     * @param entries 新しいテーブルの内容。要素数は count。
     * @param count   要素数。kMaxAnchors を超える分は無視される
     *                （false を返す）。
     * @return 成功したら true。count が 0 または kMaxAnchors 超なら false
     *         （false のときテーブルは変更しない）。
     */
    bool set(const AnchorEntry* entries, size_t count);

    /**
     * @brief 1エントリだけ書き換える（例: NVSから読んだアドレスを反映する等）。
     *
     * 同じ記憶域を書き換えるので `uwb_config` の再初期化は不要（anchors
     * ポインタも n_anchors も変わらない）。
     *
     * @return index が現在の台数未満なら true。範囲外なら false。
     */
    bool update(size_t index, const AnchorEntry& entry);

    /** 現在登録されている台数。 */
    size_t size() const { return count_; }

    /** index 番目のエントリ。範囲外アクセスは呼び出し側の責任
     *  （index < size() を確認すること）。 */
    const AnchorEntry& entry(size_t index) const { return entries_[index]; }

    /**
     * @brief short_addr からテーブル添字を引く。
     * @return 見つかれば [0, size()) の添字。見つからなければ -1。
     */
    int indexOfShortAddr(uint16_t short_addr) const;

    /** uwb_loc へ渡す設定（anchors は本クラスの内部記憶域を指す）。
     *  dim/z_fixed/huber_k 等のソルバパラメータは呼び出し側が
     *  mutableConfig() 経由で必要に応じて上書きしてよい
     *  （本クラスは anchors/n_anchors 以外のフィールドには関与しない）。 */
    const uwb_config& config() const { return config_; }
    uwb_config& mutableConfig() { return config_; }

    /** 現在 dim==2（2D固定測位）か。 */
    bool isDimension2D() const { return config_.dim == 2; }

    /** dim=3（既定）に戻す。 */
    void setDimension3D();

    /**
     * @brief dim=2 + z_fixed に切り替える。
     *
     * 同一平面配置で3D測位を諦め、水平位置だけを解かせたいときに呼ぶ
     * （docs/ANCHOR_PLACEMENT.md の配置ルール参照。x,yは同一平面配置でも
     * 正しく出ることが実測済み）。
     *
     * @param zFixedM タグの想定高さ [m]
     */
    void setDimension2D(float zFixedM);

    /**
     * @brief 起動時のアンカー配置チェック。**必ず呼ぶこと。**
     *
     * uwb_anchors_coplanar() を呼び、同一平面配置なら coplanar=true を、
     * さらにその平面がワールド原点をほぼ通っているなら originWarning=true
     * を返す（実測で Lv2 が ok=0 になることが分かっている条件。
     * PositioningConfig::originPlaneEpsM がしきい値）。
     *
     * 呼び出し側の責務: coplanar なら警告ログを出す。originWarning なら
     * さらに「アンカー平面が原点を通っています。ワールド原点をずらして
     * ください」の警告を出す。3Dのまま解かせたくなければ setDimension2D()
     * を呼ぶ。
     */
    PlacementCheck checkPlacement(float originPlaneEpsM = 0.05f) const;

    /** 現在登録されている台数のうち、enabled==true のものの数。 */
    size_t enabledCount() const;

    /** 測位モードの手動オーバーライド設定（既定 Auto）。evaluateMode() が使う。 */
    void setModeOverride(ModeOverride mode) { modeOverride_ = mode; }
    ModeOverride modeOverride() const { return modeOverride_; }

    /** 2D測位（dim=2）で使う固定高さ [m]。evaluateMode() が使う。既定は0
     *  （呼び出し側が起動時に Kconfig UWB_TAG_FIXED_Z_MM 等から必ず設定すること）。 */
    void setZFixedM(float zFixedM) { zFixedM_ = zFixedM; }
    float zFixedM() const { return zFixedM_; }

    /**
     * @brief 有効アンカー台数・配置・modeOverride()/zFixedM() から測位モードを
     * 決定し（uwb_ranging_mode.hpp の decidePositioningMode()）、dim/z_fixed へ
     * 反映する。
     *
     * 呼ぶタイミング: 起動時（テーブル構築直後）と、テーブルを変える操作
     * （set()/update()）・setModeOverride()/setZFixedM() の後に必ず呼ぶこと
     * （firmware/tag/main/main.cpp の applyModePolicy() 参照）。
     *
     * @return 決定内容（ログ・JSON出力に使う）。以後 modeDecision() でも
     *         同じ値を返す。
     */
    ModeDecision evaluateMode();

    /** 直近の evaluateMode() の結果。呼ぶ前は既定値（RangingOnly/TooFewAnchors）。 */
    const ModeDecision& modeDecision() const { return modeDecision_; }

    /** 現在の EKF チューニング（既定はコンストラクタ既定値 = EkfTuning{} と同じ）。 */
    const EkfTuning& ekfTuning() const { return ekfTuning_; }

    /**
     * @brief EKF チューニングを差し替える。
     *
     * ekfTuning() を更新し、登録済み全アンカー(0..size())の
     * uwb_anchor.sigma0/sigma_per_m を tuning.sigmaR0/sigmaRPerM で
     * 上書きする（座標・enabled 等は変えないので rebuildStorage() は
     * 呼ばない）。Q（プロセス雑音）・ゲートへの反映は呼び出し側
     * （RangingService::reinitEkf() が PositioningPipeline::initEkf() を
     * 呼び直す）の仕事 — このクラスは R（観測雑音）の記憶域だけを持つ。
     *
     * @note set()/update() で新しいアンカーを登録・書き換えたときも、
     *       ここで最後に渡したチューニングの sigmaR0/sigmaRPerM がそのまま
     *       使われる（rebuildStorage()/update() の実装参照）。
     */
    void setEkfTuning(const EkfTuning& tuning);

private:
    AnchorEntry entries_[kMaxAnchors]{};
    uwb_anchor storage_[kMaxAnchors]{}; //!< uwb_config.anchors が指す安定記憶域
    size_t count_ = 0;
    uwb_config config_{};
    bool configInitialized_ = false; //!< 2回目以降の set() で dim/z_fixed 等を
                                      //!< 引き継ぐために、初回かどうかを覚えておく

    ModeOverride modeOverride_ = ModeOverride::Auto; //!< setModeOverride() 参照
    float zFixedM_               = 0.0f;               //!< setZFixedM() 参照
    ModeDecision modeDecision_{};                        //!< evaluateMode() の最新結果
    EkfTuning ekfTuning_{};                               //!< setEkfTuning() 参照

    /**
     * @brief entries_[0..count_) から storage_ を作り直し、uwb_config_init()
     * をやり直す（set() からのみ呼ぶ。update() は記憶域の中身だけを書き換える
     * ので不要）。
     *
     * uwb_config_init() は anchors/n_anchors 以外の全フィールドも既定値へ
     * 戻してしまうため、2回目以降の呼び出しでは setDimension2D() 等で
     * 変更済みのフィールド（dim/z_fixed/use_z_bounds/z_min/z_max や、
     * 呼び出し側が mutableConfig() 経由で変えた huber_k 等も含め全部）を
     * 退避しておき、初期化後に復元する。初回はまだ何も設定されていないので
     * 素直に既定値（dim=3 等）のままにする。
     */
    void rebuildStorage();
};

} // namespace uwb
