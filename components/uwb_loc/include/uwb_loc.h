/* uwb_loc — チップ非依存の UWB 測位ライブラリ (C 版)
 *
 * Python 版 (uwb_loc) の測位部分だけを C99 に移したもの。マイコンで動かす
 * ことを前提にしている。
 *
 *   * **malloc を呼ばない。** 必要な作業領域はすべて構造体の中かスタックにある
 *   * **依存は libm の sqrt/fabs だけ。** printf も文字列処理も使わない
 *   * 扱う行列は最大 6x6。倍精度でも数 KB に収まる
 *
 * 入っているもの (Python 版と同じ 4 段階):
 *
 *   Lv0  uwb_solve_lv0   閉形式の三辺測量。反復なし。配線と座標系の確認用
 *   Lv1  uwb_solve_lv1   重み付き非線形最小二乗 + chi2 ゲート
 *   Lv2  uwb_solve_lv2   Beck 厳密解 + Huber ロバスト化。屋内の既定
 *   Lv3  uwb_ekf_*       密結合 EKF。移動体むけ
 *
 * 入っていないもの: HAL・シミュレータ・UI・自己測量。
 * それらはホスト側 (Python 版) の仕事。詳しくは c/README.md。
 *
 * 使い方の最短形:
 *
 *     uwb_anchor anchors[4] = {
 *         {"A0", {0.0, 0.0, 2.4}, 1, 0.0, 0.1, 0.0},
 *         ...
 *     };
 *     uwb_config cfg;
 *     uwb_config_init(&cfg, anchors, 4);
 *
 *     uwb_meas meas[4];
 *     meas[0].anchor = 0; meas[0].value = 4.85; ...
 *
 *     uwb_fix fix;
 *     if (uwb_solve_lv2(&cfg, meas, 4, &fix) && fix.ok) {
 *         use(fix.p[0], fix.p[1], fix.p[2]);
 *     }
 */
#ifndef UWB_LOC_H
#define UWB_LOC_H

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 設定 */

/* 実数型。既定は double。-DUWB_USE_FLOAT で float になる (RAM/ROM 節約、
 * ただし Beck 法の二分法と共分散の桁落ちに弱くなるので、まず double で
 * 動かしてから切り替えること)。 */
#ifdef UWB_USE_FLOAT
typedef float uwb_real;
#define UWB_REAL_IS_FLOAT 1
#else
typedef double uwb_real;
#define UWB_REAL_IS_FLOAT 0
#endif

/* 同時に扱えるアンカー数と 1 エポックの観測数。静的に確保するので、
 * 実際の台数に合わせて小さくすると RAM が減る。 */
#ifndef UWB_MAX_ANCHORS
#define UWB_MAX_ANCHORS 16
#endif
#ifndef UWB_MAX_MEAS
#define UWB_MAX_MEAS 32
#endif

/* アンカー ID の長さ (終端含む)。ID を使わない (添字だけで済ませる)
 * 構成なら 1 にしてよい。 */
#ifndef UWB_ID_LEN
#define UWB_ID_LEN 16
#endif

/* EKF の状態次元の上限: 3 次元 x (位置,速度,加速度) = 9 */
#define UWB_MAX_STATE 9

/* ------------------------------------------------------------------ 型 */

/** アンカー (座標が分かっている固定点)。単位は m、右手系で z が上。 */
typedef struct {
    char     id[UWB_ID_LEN];  /**< 表示・照合用。測位には使わない */
    uwb_real p[3];            /**< 座標 [m] */
    int      enabled;         /**< 0 なら使わない */
    uwb_real antenna_delay_m; /**< 測距値から引くオフセット [m] */
    uwb_real sigma0;          /**< 距離 0 での 1sigma [m]。0 なら 0.1 とみなす */
    uwb_real sigma_per_m;     /**< 距離に比例して増える分 [m/m] */
} uwb_anchor;

/** 観測 1 本 (測距)。 */
typedef struct {
    int      anchor;  /**< アンカー配列の添字。範囲外なら無視される */
    uwb_real value;   /**< 測距値 [m] */
    uwb_real sigma;   /**< 1sigma [m]。0 以下ならアンカーの設定から作る */
    uwb_real quality; /**< 0-1 の信頼度。負なら「不明」。低いと sigma を膨らませる */
} uwb_meas;

/** 測位の設定。`uwb_config_init` で初期化してから必要な項目を変える。 */
typedef struct {
    const uwb_anchor *anchors;
    int               n_anchors;

    int      dim;          /**< 2 か 3。2 なら高さを z_fixed に固定 */
    uwb_real z_fixed;      /**< dim==2 のときの高さ [m] */

    int      use_z_bounds; /**< 1 なら z を [z_min, z_max] に収める */
    uwb_real z_min, z_max;

    int      max_iter;     /**< Gauss-Newton の最大反復 */
    uwb_real tol;          /**< 収束判定 (更新量のノルム) [m] */

    /* ロバスト化 (Lv2) */
    uwb_real huber_k;      /**< Huber のしきい値 (正規化残差)。既定 1.345 */
    uwb_real k_pos_scale;  /**< 正側 (NLOS 側) に掛ける係数。既定 0.6 */
    int      one_sided;    /**< 1 なら正側だけ厳しく見る */

    /* 外れ値ゲート */
    uwb_real chi2_k;       /**< |残差| > chi2_k * sigma を外す。0 で無効。
                            *   負ならレベル別の既定 (Lv1: 3.5 / Lv2: 4.0)。
                            *   Python 版の既定と同じ値になる */
    int      physical_gate;/**< 1 なら距離の妥当性と三角不等式で足切り */
    uwb_real max_range;    /**< 物理ゲートの上限 [m]。既定 200 */
} uwb_config;

/** 測位結果。 */
typedef struct {
    uwb_real p[3];        /**< 位置 [m] */
    uwb_real cov[9];      /**< 位置の共分散 (行優先 3x3) [m^2] */
    uwb_real v[3];        /**< 速度 [m/s]。Lv3 のみ、他は 0 */

    int      ok;          /**< 0 なら p は意味を持たない。**必ず見ること** */
    int      n_used;      /**< 使った観測数 */
    int      n_total;     /**< 渡された観測数 */
    int      iterations;  /**< 反復回数 */
    int      ambiguous;   /**< 1 なら鏡像解かもしれない (高さが信用できない) */

    uwb_real residual_rms;/**< 残差 RMS [m]。大きい = 座標か単位を疑う */
    uwb_real gdop;        /**< 幾何精度劣化係数。5 超で配置が悪い */
    uwb_real sigma;       /**< 位置誤差の代表値 = sqrt(trace(cov)) [m] */

    /** 外れ値として落とした観測のビットマスク (観測の添字に対応)。
     *  観測が 32 本を超える分は記録されない。 */
    unsigned long excluded;
} uwb_fix;

/* ------------------------------------------------------------------ 設定 */

/** 設定を既定値で埋める。anchors は呼び出し側が保持し続けること。 */
void uwb_config_init(uwb_config *cfg, const uwb_anchor *anchors, int n_anchors);

/** ID からアンカーの添字を引く。見つからなければ -1。 */
int uwb_anchor_index(const uwb_config *cfg, const char *id);

/* ------------------------------------------------------- スナップショット */

/** Lv0 — 閉形式の三辺測量。反復も初期値も不要。 */
int uwb_solve_lv0(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out);

/** Lv1 — 重み付き非線形最小二乗 + chi2 ゲート。 */
int uwb_solve_lv1(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out);

/** Lv2 — Beck 厳密解を初期値に、Huber + 片側損失でロバスト化。屋内の既定。
 *
 *  Python 版の Lv2 との違いは **RANSAC を持たないこと**だけ。
 *  Python 側で `use_ransac=False` にすると結果が一致する (c/README.md 参照)。 */
int uwb_solve_lv2(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_fix *out);

/** Beck の厳密解そのもの (初期値づくりや動作確認用)。
 *  解けたら 1 を返し out[0..dim-1] に位置を書く。 */
int uwb_beck_gtrs(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out);

/** 基準アンカー差分による線形最小二乗そのもの。 */
int uwb_lls_trilateration(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out);

/* ------------------------------------------------------------------ Lv3 */

typedef enum {
    UWB_MOTION_CV = 0, /**< 等速。位置と速度を持つ */
    UWB_MOTION_CA = 1  /**< 等加速度。加速度まで持つ。機敏な動きむけ */
} uwb_motion;

/** 密結合 EKF。呼び出し側が構造体を持つ (malloc しない)。 */
typedef struct {
    const uwb_config *cfg;
    uwb_motion motion;
    uwb_real   sigma_a;    /**< 最上位微分の連続白色雑音の強さ */
    uwb_real   gate;       /**< イノベーションゲート (sigma 単位)。既定 3 */
    uwb_real   max_dt;     /**< これ以上間が空いたら組み直す [s]。既定 2 */
    int        max_rejects;/**< 連続で全棄却がこの回数続いたら組み直す */

    int      nd, norder, nx;
    uwb_real x[UWB_MAX_STATE];
    uwb_real P[UWB_MAX_STATE * UWB_MAX_STATE];
    uwb_real t;
    int      has_t;
    int      initialized;
    int      rejects;
    int      ambiguous;

    /* 立ち上げ待ちの観測 (1 本ずつ届く経路むけ) */
    uwb_meas pending[UWB_MAX_MEAS];
    uwb_real pending_t[UWB_MAX_MEAS];
    int      n_pending;

    /* 鏡像解の「どちら側か」の記憶 */
    int      side_known;
    int      side;
} uwb_ekf;

/** EKF を初期化する。cfg は保持し続けること。 */
void uwb_ekf_init(uwb_ekf *ekf, const uwb_config *cfg, uwb_motion motion, uwb_real sigma_a);

/** 状態を捨てて初期化直後に戻す (設定は保つ)。 */
void uwb_ekf_reset(uwb_ekf *ekf);

/** dt 秒ぶん予測だけ進める (観測が来ない時間)。 */
void uwb_ekf_predict(uwb_ekf *ekf, uwb_real dt);

/** 観測を入れて更新する。t は単調増加の秒。
 *  1 エポックに 1 本しか無くてもよい (立ち上げ後は 1 本ずつ更新できる)。 */
int uwb_ekf_update(uwb_ekf *ekf, uwb_real t, const uwb_meas *meas, int n, uwb_fix *out);

/* ------------------------------------------------------------ 配置の評価 */

/** その点の GDOP。置く前に配置の良し悪しを見る。解けなければ負を返す。 */
uwb_real uwb_gdop_at(const uwb_config *cfg, const uwb_real *point);

/** 位置誤差の理論下限 (CRLB) [m]。解けなければ負を返す。 */
uwb_real uwb_crlb_at(const uwb_config *cfg, const uwb_real *point);

/** アンカーが同一平面に並んでいるか。1 なら 3 次元では鏡像解が出る。
 *  normal/offset が非 NULL なら平面 (n.p = offset) を書き出す。 */
int uwb_anchors_coplanar(const uwb_config *cfg, uwb_real *normal, uwb_real *offset);

/* ------------------------------------------------------------ バージョン */

#define UWB_VERSION_MAJOR 0
#define UWB_VERSION_MINOR 1
#define UWB_VERSION_PATCH 0

/** "0.1.0" のような文字列。Python 版と合わせてある。 */
const char *uwb_version(void);

#ifdef __cplusplus
}
#endif

#endif /* UWB_LOC_H */
