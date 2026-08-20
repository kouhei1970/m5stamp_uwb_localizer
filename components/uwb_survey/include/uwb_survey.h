/* uwb_survey — アンカー自動測量（相互測距だけからノード座標を復元する）
 *
 * docs/SURVEY_SPEC.md の [2]〜[5] に相当する **純計算部分**。
 * ESP-IDF にも FreeRTOS にも UWB チップにも依存しない。ホストで
 * `cc` だけでビルドできる（tools/test_survey/Makefile 参照）。
 *
 *   * **malloc を呼ばない。** 作業領域はすべてスタックの固定長配列
 *   * 依存は <math.h> / <string.h> と uwb_loc.h の uwb_real だけ
 *   * 扱う行列は最大 25x25（= 3*8+1 個の未知数）。倍精度で 5KB
 *
 * components/uwb_loc/ と同じ流儀で書いてある（将来 uwb_localizer の c/ へ
 * 還元できるように）。
 *
 * ------------------------------------------------------------------------
 * 何を解いているか
 *
 * N 台のノードを適当な場所に置き、総当たりで相互測距する。得られるのは
 *
 *     r_ij = d_ij + 2*delta        (i<j, d_ij = |p_i - p_j|)
 *
 * だけ。delta は全ノード共通のアンテナ遅延（片道ぶん、単位 m）で、
 * 往復で 2*delta 効く。ここから
 *
 *     p_0 .. p_{N-1}  (3N 個) と delta (1 個)
 *
 * を推定する。距離だけでは **並進3 + 回転3 + 鏡像1** ぶんが決まらない
 * （ゲージの不定性）ので、
 *
 *   - 実測した高さ h_k で 傾き2 + z並進1 を固定（[4]）
 *   - ノード0 を XY 原点、ノード1 を +X 軸上 に置く規約で
 *     ヨー1 + XY並進2 を固定（[5]）
 *   - 鏡像は **高さでは決まらない**（後述）ので、決まった規約で潰す
 *
 * ------------------------------------------------------------------------
 * 【重要】鏡像（キラリティ）は高さ実測では決まらない
 *
 * 仕様書 [4] は「反転あり/なしの両方で高さ残差を計算し、小さい方を採用」
 * としているが、これは**数学的に必ず引き分けになる**。
 *
 *   ある反射 S（直交・det=-1）で反転した配置 S*p に対し、高さの予測値は
 *   任意の単位ベクトル u を「上」として  z_k = u.(S p_k) + c = (S u).p_k + c。
 *   S は直交なので u -> S u は単位球面の全単射。つまり
 *   **反転前に実現できる高さの組と反転後に実現できる高さの組は完全に一致する**。
 *   よってどちらの鏡像でも高さ残差の最小値は同じ値になる。
 *
 * 実際、床に置いた3台と天井の1台という配置を z=0 面で反転し、x 軸まわりに
 * 180 度回すと、高さはすべて元通りで xy だけが鏡像になる。
 *
 * したがって本実装は
 *   - 仕様どおり両方の高さ残差を計算して比較し（診断のため）、
 *     有意差があれば小さい方を採って `mirror_resolved = 1`
 *   - 引き分け（＝常にこちら）なら **規約**で潰して `mirror_resolved = 0`
 * とする。規約は「[5] の正規化後、|y| が最大のノード（添字最小優先）の
 * y を正にする」。これにより **どんな初期値・どちらの鏡像から出発しても
 * 出力は必ず同じ**になる（再現性はある。ただし現実の左右が当たっている
 * 保証はない）。
 *
 * 現実の左右を確定させたいなら、距離と高さ以外の情報（方位、
 * 「アンカー2 は入口から見て右」といった1ビットの事前知識、など）が要る。
 *
 * ------------------------------------------------------------------------
 * 使い方
 *
 *     uwb_survey_input in;
 *     uwb_survey_result out;
 *     uwb_survey_input_init(&in, 6);
 *     in.dist[0][1] = in.dist[1][0] = 4.83;  in.have[0][1] = in.have[1][0] = 1;
 *     ...
 *     in.height[0] = 0.30; in.have_height[0] = 1;
 *     ...
 *     if (uwb_survey_solve(&in, &out) && out.ok) {
 *         use(out.pos[i]);  use(out.common_delay_m);
 *     }
 *
 * ------------------------------------------------------------------------
 * スタック使用量
 *
 * `uwb_survey_solve()` は正規方程式 25x25 をスタックに置く。倍精度で
 * 呼び出し全体の深さは **約 10KB**（実測: solve 3.1KB + lm_run 6.2KB +
 * sym_eig 0.6KB。-DUWB_USE_FLOAT なら約半分）。ESP-IDF では
 * **16KB 以上**のスタックを持つタスクから呼ぶこと
 * （app_main の既定 3.5KB では足りない）。
 * グローバル変数は一切持たないので再入可能。
 */
#ifndef UWB_SURVEY_H
#define UWB_SURVEY_H

/* uwb_real（既定 double、-DUWB_USE_FLOAT で float）だけを借りる。 */
#include "uwb_loc.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 寸法 */

/** 測量に参加できるノード数の上限。components/uwb_loc の Kconfig 既定
 *  （UWB_LOC_MAX_ANCHORS=8）に合わせてある。増やすと未知数が増え、
 *  正規方程式（3N+1 の二乗）が二次で膨らむので注意。 */
#ifndef UWB_SURVEY_MAX_NODES
#define UWB_SURVEY_MAX_NODES 8
#endif

/** 3次元の形状を距離だけから決めるのに必要な最小ノード数。
 *  4 未満では四面体が作れない。 */
#define UWB_SURVEY_MIN_NODES 4

/** リンク（ノード対）の最大本数 = N(N-1)/2。N=8 で 28。 */
#define UWB_SURVEY_MAX_LINKS \
    (UWB_SURVEY_MAX_NODES * (UWB_SURVEY_MAX_NODES - 1) / 2)

/** 未知数の最大個数 = 座標 3N + 共通遅延 1。N=8 で 25。 */
#define UWB_SURVEY_MAX_UNKNOWNS (3 * UWB_SURVEY_MAX_NODES + 1)

/* ------------------------------------------------------------------ 型 */

/** 測量の入力。 */
typedef struct {
    int      n;                                                   /**< ノード数 (4..UWB_SURVEY_MAX_NODES) */
    uwb_real dist[UWB_SURVEY_MAX_NODES][UWB_SURVEY_MAX_NODES];    /**< 相互測距 [m]。生の値
                                                                   *   （アンテナ遅延を含んだまま）を入れる */
    int      have[UWB_SURVEY_MAX_NODES][UWB_SURVEY_MAX_NODES];    /**< 1 = 測定あり、0 = 欠測 */
    uwb_real height[UWB_SURVEY_MAX_NODES];                        /**< 実測した高さ [m] */
    int      have_height[UWB_SURVEY_MAX_NODES];                   /**< 1 = 高さの実測あり */
} uwb_survey_input;

/** 測量の結果。 */
typedef struct {
    uwb_real pos[UWB_SURVEY_MAX_NODES][3]; /**< 推定座標 [m]。i >= n は 0 */
    uwb_real common_delay_m;               /**< 推定した共通アンテナ遅延 [m]（**片道あたり**）。
                                            *   観測モデルは r = d + 2*common_delay_m。
                                            *   識別可能でない構成（後述）では 0 に固定される */
    int      ok;                           /**< 0 なら pos は意味を持たない。**必ず見ること** */
    int      iterations;                   /**< Gauss-Newton(LM) の受理反復回数の合計 */
    uwb_real residual_rms;                 /**< 距離残差 RMS [m]。大きい = 測距か配置を疑う */
    int      degenerate;                   /**< 1 = 配置が縮退（同一平面・直線）。このとき ok=0 */
    int      mirror_resolved;              /**< 1 = 高さ残差の比較で鏡像が決まった。
                                            *   **理論上ここは常に 0 になる**（ヘッダ先頭の説明参照）。
                                            *   0 のときは規約で一意化してある */
    unsigned long excluded;                /**< 外れ値として落としたリンクのビットマスク。
                                            *   ビット位置は uwb_survey_link_index(i,j) */
} uwb_survey_result;

/* ------------------------------------------------------------------ API */

/** 入力をゼロ埋めして n だけ設定する。have[][] / have_height[] も 0 になる。 */
void uwb_survey_input_init(uwb_survey_input *in, int n);

/**
 * 測量を解く。
 *
 * 手順は docs/SURVEY_SPEC.md §2 のとおり:
 *   [2] 欠測リンクを最短経路で補完 → 古典的 MDS で初期配置
 *   [3] Levenberg-Marquardt で距離残差を最小化（共通遅延も同時推定）
 *   [4] 実測高さで 傾き2 + z並進1 を固定（鏡像は規約で一意化）
 *   [5] ノード0 を XY 原点、ノード1 を +X 軸上に置く
 *
 * 解けたら 1 を返し out->ok にも 1 を書く。解けなければ 0。
 * 失敗時も out は必ずゼロ初期化されており、縮退が理由なら
 * out->degenerate = 1 と MDS の生の座標が入る（診断用）。
 *
 * 失敗する条件:
 *   - n が範囲外 / 引数が NULL
 *   - 測距が取れているリンクだけでは形状が決まらない
 *     （どれかのノードのリンクが 3 本未満、リンク総数が 3n-6 未満、
 *      リンクのグラフが非連結）
 *   - MDS の固有値が退化（同一平面・直線配置）→ degenerate = 1
 *   - LM が破綻した（正規方程式が解けない、NaN が出た）
 *
 * 共通遅延の識別可能性: リンク本数 m が 3n-6+1 以上でないと delta は
 * 形状の自由度と縮退して決まらない。**n=4（最大 6 本 vs 必要 7 本）は
 * 常にこれに該当する**ので、その場合 delta は 0 に固定して座標だけを解く。
 */
int uwb_survey_solve(const uwb_survey_input *in, uwb_survey_result *out);

/** リンク (i,j) に対応する excluded のビット位置。i!=j なら順序は問わない。
 *  ノード数に依存しない並び（j*(j-1)/2 + i, i<j）なので、n が変わっても
 *  同じペアは同じビットになる。範囲外なら -1。 */
int uwb_survey_link_index(int i, int j);

#ifdef __cplusplus
}
#endif

#endif /* UWB_SURVEY_H */
