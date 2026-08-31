/* uwb_survey_tape — メジャー（巻尺）実測値からのアンカー座標 閉形式計算。
 *
 * uwb_survey.h の uwb_survey_solve()（総当たり相互測距からの自己測量、
 * MDS初期値 + Levenberg-Marquardt）とは**別物**。こちらは「アンカーを
 * 設置したあと、巻尺で測った少数の距離（基準2台からの距離が中心）」を
 * 入力に、反復も最小二乗も使わない**閉形式**（三辺測量の代数式そのもの）
 * で座標を求める軽量な関数。ESP-IDF にも FreeRTOS にも依存しない C99。
 *
 * 使う場面のイメージ:
 *   - アンカーを壁や天井に留めた「あと」に、巻尺で
 *       d(0,1)                 基準2台間
 *       d(0,k), d(1,k)  (k>=2) 基準2台から残り各台まで
 *       d(2,k)          (k>=3、任意) 左右（鏡像）を決めるための追加測定
 *     を測って入力する。台数が少ない・LM のような反復計算をESP32-S3上で
 *     回したくない・「巻尺だけで今すぐ座標を出したい」場合に向く。
 *   - 全台の総当たり測距（ESP-NOWでの自動収集）が使えるなら
 *     uwb_survey_solve() の方が測定の手間が少なく精度も高い。
 *
 * 座標系の規約（呼び出し側 = firmware/tag/main/tag_console.cpp の
 * `survey` コマンド群が前提にしている、アンカー登録テーブルの規約と同じ）:
 *   - index 0 が XY 原点
 *   - index 1 が +X 軸上（y=0, x>0）
 *   - index 2 は +y 側に固定（規約）
 *   - index k>=3 の y の符号は d(2,k) があればそれで判定し、無ければ index 2
 *     と同じ +y 側に倒す（`sign_unresolved` で警告）
 *   - z（高さ）は uwb_survey_solve() のように「実測高さへ最小二乗で合わせる」
 *     のではなく、入力 z[i] を**そのまま**出力の z 座標に使う（メジャーで
 *     測った高さがそのまま最終座標になる、という単純な設計）
 *
 * 計算の中身（docs/SURVEY_SPEC.md「メジャー実測値からのアンカー座標計算」
 * 節）:
 *   1. 斜距離 d と高さ差 (z_i − z_j) から水平距離 h を出す（ピタゴラスの
 *      定理の逆算）: h_ij² = d_ij² − (z_i − z_j)²
 *   2. index 0, 1 を基準線に取り、逐次三辺測量の要領で index k の (x, y) を
 *      閉形式で求める（uwb_survey.c の trilat_init() と同じ余弦定理だが、
 *      こちらは全対距離が要らず、基準2台からの距離だけで済む代わりに
 *      反復も固有分解も無い）
 *
 * 依存は <math.h>（uwb_math.h 経由）だけで malloc は呼ばない。作業領域は
 * すべて呼び出し元のスタック上に置ける小さな構造体（8ノードで数百バイト）。
 */
#ifndef UWB_SURVEY_TAPE_H
#define UWB_SURVEY_TAPE_H

/* uwb_survey.h から UWB_SURVEY_MAX_NODES と uwb_survey_link_index() を
 * 借りる。ビットマスクの添字規約（ペア (i,j) → j*(j-1)/2+i）を
 * uwb_survey_result.excluded と揃えるための再利用で、依存が増えるわけ
 * ではない（uwb_survey.h は uwb_math.h だけに依存する純ヘッダ）。 */
#include "uwb_survey.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ 寸法 */

/** メジャー測量に参加できる最小ノード数。基準2台 (index 0,1) + 残り1台
 *  (index 2) の「三角形」が最小構成（これ未満は平面座標を決められない）。 */
#define UWB_SURVEY_TAPE_MIN_NODES 3

/** 最大ノード数。uwb_survey_solve() と同じ上限（AnchorTable の
 *  既定 kMaxAnchors=8）に揃えてある。 */
#define UWB_SURVEY_TAPE_MAX_NODES UWB_SURVEY_MAX_NODES

/** 根号の中身がわずかに負になったとき「メジャーの読み取り誤差」とみなして
 *  0 に丸め、計算を続ける許容量 [m²]。1cm 程度の誤差に相当する
 *  （0.01m の2乗）。これを超える負値は測定同士が矛盾しているとみなし
 *  エラーにする。 */
#define UWB_SURVEY_TAPE_CLAMP_TOL ((uwb_real)(0.01 * 0.01))

/* ------------------------------------------------------------------ 型 */

/** uwb_survey_tape_solve() が失敗した理由。 */
typedef enum {
    UWB_SURVEY_TAPE_OK = 0,
    UWB_SURVEY_TAPE_ERR_N_RANGE,      /**< n が UWB_SURVEY_TAPE_MIN_NODES..MAX_NODES の範囲外 */
    UWB_SURVEY_TAPE_ERR_MISSING,      /**< 必須の距離が未入力（out->missing にどのペアかが立つ） */
    UWB_SURVEY_TAPE_ERR_BASELINE,     /**< d(0,1) の水平距離が0以下（高さ差と同じかそれ以上で
                                       *   +X軸を定義できない。out->err_i=0, err_j=1） */
    UWB_SURVEY_TAPE_ERR_INCONSISTENT  /**< 根号の中身が測定誤差では説明できないほど負
                                       *   （三角不等式などの矛盾）。out->err_i/err_j 参照 */
} uwb_survey_tape_status;

/** メジャー測量の入力。 */
typedef struct {
    int      n; /**< ノード数 (UWB_SURVEY_TAPE_MIN_NODES..UWB_SURVEY_TAPE_MAX_NODES) */

    /** 実測した斜距離 [m]。使うのは
     *    dist[0][1]                  （必須）
     *    dist[0][k], dist[1][k]      （k=2..n-1、必須）
     *    dist[2][k]                  （k=3..n-1、任意。y の符号判定にだけ使う）
     *  のみで、それ以外の要素（例えば dist[3][5]）は無視する。
     *  [i][j] と [j][i] のどちらに入れてもよく（両方入れれば平均を採る。
     *  uwb_survey.c の build_links() と同じ流儀）、i==j は無視する。 */
    uwb_real dist[UWB_SURVEY_TAPE_MAX_NODES][UWB_SURVEY_TAPE_MAX_NODES];
    int      have[UWB_SURVEY_TAPE_MAX_NODES][UWB_SURVEY_TAPE_MAX_NODES]; /**< 1 = 実測あり */

    /** 各ノードの高さ [m]。index0 を基準にした相対高さという想定だが、
     *  z[0] を 0 以外にしてもよい（例えば「床からの高さ」を直接入れたい
     *  場合）。この値は最小二乗で調整せず、そのまま出力の z 座標になる。
     *  既定は 0（uwb_survey_tape_input_init() でゼロ埋めされる）。 */
    uwb_real z[UWB_SURVEY_TAPE_MAX_NODES];
} uwb_survey_tape_input;

/** メジャー測量の結果。 */
typedef struct {
    uwb_real pos[UWB_SURVEY_TAPE_MAX_NODES][3]; /**< 推定座標 [m]。i >= n は 0 */
    int      ok;                                /**< 0 なら pos は意味を持たない */
    uwb_survey_tape_status status;              /**< 失敗理由（ok=1 なら UWB_SURVEY_TAPE_OK） */

    /** 必須の距離が未入力のときに立つビットマスク。ビット位置は
     *  uwb_survey_link_index(i,j)。1本でも立っていれば status は
     *  UWB_SURVEY_TAPE_ERR_MISSING（未入力を全部集めてから返すので、
     *  「まず (0,2) を埋めたら次は (1,3) が要ると言われる」という
     *  一問一答にはならない）。 */
    unsigned long missing;

    /** 根号の中身がわずかに負（UWB_SURVEY_TAPE_CLAMP_TOL 以内）で 0 に
     *  丸めたペアのビットマスク。ビット位置は uwb_survey_link_index(i,j)。
     *  h_ij²（水平距離の2乗）の丸めだけを記録する。エラーにはしていないが、
     *  巻尺の読み方や高さの実測を疑うべき、という警告。 */
    unsigned long clamped_pairs;

    /** ノード k (2 <= k < n) の y² 側（(0,1,k) の三角形の残り一辺）を
     *  0 に丸めたときのビットマスク。ビット位置は k そのもの
     *  （ペアではなく単一ノードなので clamped_pairs とは別に持つ）。 */
    unsigned long clamped_nodes;

    /** d(2,k) が無いために y の符号を規約（+y）のまま確定できなかった
     *  ノード k (3 <= k < n) のビットマスク。ビット位置は k。立っていたら
     *  「そのノードは鏡像になっている（左右が逆）」可能性がある。 */
    unsigned long sign_unresolved;

    /** status がエラーのとき、原因を示すノード添字。該当しなければ -1。
     *    ERR_BASELINE:     err_i=0, err_j=1
     *    ERR_INCONSISTENT: err_i>=0 のとき → d(err_i,err_j) の水平距離が
     *                       計算できない（斜距離が高さ差より短い）
     *                       err_i==-1 のとき → ノード err_j の三角形
     *                       (0, 1, err_j) が矛盾（三角不等式違反） */
    int err_i;
    int err_j;
} uwb_survey_tape_result;

/* ------------------------------------------------------------------ API */

/** 入力をゼロ埋めして n だけ設定する（dist/have/z は全部 0）。 */
void uwb_survey_tape_input_init(uwb_survey_tape_input *in, int n);

/**
 * @brief メジャー実測値から閉形式で座標を求める。
 *
 * 手順:
 *   1. 必須の距離（d(0,1)、および各 k=2..n-1 の d(0,k)・d(1,k)）が全部
 *      揃っているか確認する。1本でも欠けていれば ok=0、
 *      status=UWB_SURVEY_TAPE_ERR_MISSING、out->missing に欠けている
 *      ペア全部のビットを立てて返す（この時点では座標計算をしない）。
 *   2. d(0,1) と高さ差から水平距離 h01 を出し、index0=(0,0,z0)、
 *      index1=(h01,0,z1) を置く。h01 <= 0（高さ差が d(0,1) 以上）なら
 *      +X 軸が定義できないので ok=0、status=UWB_SURVEY_TAPE_ERR_BASELINE。
 *   3. k=2..n-1 について、h0k・h1k（同様に高さ補正した水平距離）から
 *      余弦定理で x を、ピタゴラスの定理で |y| を求める。根号の中身が
 *      大きく負なら ok=0、status=UWB_SURVEY_TAPE_ERR_INCONSISTENT
 *      （わずかな負は UWB_SURVEY_TAPE_CLAMP_TOL 以内なら 0 に丸めて続行し
 *      out->clamped_pairs / out->clamped_nodes に記録する）。
 *   4. y の符号は k=2 なら +y に固定。k>=3 は d(2,k) があればそれに近い方
 *      （出力座標から d(2,k) までの3次元距離の予測値と実測値の差が小さい
 *      方）を採用し、無ければ +y のまま out->sign_unresolved に記録する。
 *
 * 成功したら 1 を返し ok=1、失敗したら 0 を返し ok=0（out は必ず
 * ゼロ初期化された上で、分かる範囲の診断フィールドが埋まる）。
 * out が NULL なら何もせず 0。in が NULL、または in->n が範囲外
 * （UWB_SURVEY_TAPE_MIN_NODES..UWB_SURVEY_TAPE_MAX_NODES の外）なら
 * status=UWB_SURVEY_TAPE_ERR_N_RANGE で 0。
 */
int uwb_survey_tape_solve(const uwb_survey_tape_input *in, uwb_survey_tape_result *out);

#ifdef __cplusplus
}
#endif

#endif /* UWB_SURVEY_TAPE_H */
