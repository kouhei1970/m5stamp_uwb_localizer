/* uwb_survey — アンカー自動測量の計算本体（docs/SURVEY_SPEC.md §2 [2]〜[5]）。
 *
 * 数式の導出はそれぞれの節に書いてある。全体の流れは
 *
 *   build_links()        入力を「リンクの列」に直し、可解性を検査する
 *   complete_distances() 欠測リンクを最短経路で埋める（初期値づくり用）
 *   trilat_init()        逐次三辺測量（ピボット付きコレスキー 3 段）で初期配置
 *   lm_run()             Levenberg-Marquardt で距離残差を最小化
 *   escape_local_minima() ノードを隣接平面で鏡映して局所解から抜ける
 *   loo_drop_link()      外れ値リンクを leave-one-out で1本ずつ落として解き直す
 *   shape_degenerate()   収束した最終形状を PCA して同一平面・直線を検出
 *   fit_up()             実測高さに合う「上」方向 u と z オフセット c を求める
 *   apply_frame()        u を +z に持ってくる回転と z 並進を掛ける
 *   apply_xy_convention() ノード0 を XY 原点、ノード1 を +X 軸上へ
 *   fix_chirality()      鏡像を chirality 入力（無ければ |y| 規約）で一意化する
 *
 * 線形代数はすべて components/uwb_math（スカラー展開した 3x3 / 3x3 ブロック）
 * で済ませる。一般の固有分解・一般のコレスキーは使わない。ESP32-S3 は
 * 単精度 FPU しか持たないので、除算と sqrt の回数を各関数のコメントに書く。
 */
#include "uwb_survey.h"

#include <string.h>

#define NMAX UWB_SURVEY_MAX_NODES
#define LMAX UWB_SURVEY_MAX_LINKS
#define XMAX UWB_SURVEY_MAX_UNKNOWNS

/* ------------------------------------------------------------ 調整定数 */

/** LM の最大反復。uwb_loc の max_iter=30 より多く取る — 測量は設置時に
 *  1 回だけ走る処理で、速度より収束の確実さが要るため。 */
#ifndef UWB_SURVEY_MAX_ITER
#define UWB_SURVEY_MAX_ITER 100
#endif

/** 収束判定（更新量ノルム [m]）。uwb_loc の tol=1e-4 と同じ「更新量で見る」
 *  流儀だが、測量は数値誤差レベル（1e-6 m）の復元を要求されるので厳しくする。
 *  単精度ではここまで届かないので、「減衰を上げても下がらない」で止まる。 */
#ifndef UWB_SURVEY_TOL
#define UWB_SURVEY_TOL ((uwb_real)1e-10)
#endif

/** LM 減衰係数の初期値・下限・上限。
 *
 *  下限と絶対リッジは **計算機イプシロンに比例**させる（1e4·eps = double
 *  2.2e-12、float 1.2e-3）。以前の決め打ち 1e-12 は単精度では eps の 1e-5
 *  倍で、(a) ゲージ方向（J^T J のヌル空間）の勾配が丸め誤差 eps·|g| だけ
 *  残るのを 1e-12 で割って巨大な剛体移動が出る、(b) 対角 ~1 の行列に
 *  1e-12 を足しても消えるのでコレスキーのピボットが非正になる、の二重苦で
 *  float ビルドのコレスキー失敗が 221 回・呼出が 2.5 倍になっていた
 *  （docs/REVIEW_2026-08-21.md A-9）。 */
#ifndef UWB_SURVEY_LM_INIT
#define UWB_SURVEY_LM_INIT ((uwb_real)1e-3)
#endif
#define UWB_SURVEY_LM_MIN  UWB_MATH_RANK_TOL
#define UWB_SURVEY_LM_MAX  ((uwb_real)1e12)

/** 正規方程式の対角に必ず足す絶対リッジ。孤立した未知数（対角が 0）でも
 *  コレスキーが落ちないようにするための保険。ヤコビ行は単位ベクトル
 *  なので対角は O(次数) ≈ 1〜10 の無次元量、リッジも無次元でよい。 */
#define UWB_SURVEY_RIDGE_ABS UWB_MATH_RANK_TOL

/** 縮退判定: 第3主軸の二乗広がりが最大主軸のこの割合を下回ったら同一平面と
 *  みなす（スケール非依存の保険）。1e-3 だと 80m×3m×2.4m の細長い配置
 *  （(2.4/80)² = 9e-4）で偽陽性になるので 1e-5（厚み/長さ < 0.3%）にした
 *  （A-7）。物理的な判定は下の DEGEN_EXTENT が担う。 */
#ifndef UWB_SURVEY_DEGEN_RATIO
#define UWB_SURVEY_DEGEN_RATIO ((uwb_real)1e-5)
#endif

/** 縮退判定: 第3主軸方向の RMS 広がり [m] がこれ未満なら縮退。
 *  UWB の測距ばらつき（数 cm）より小さい厚みは観測できない、という物理。 */
#ifndef UWB_SURVEY_DEGEN_EXTENT
#define UWB_SURVEY_DEGEN_EXTENT ((uwb_real)0.05)
#endif

/** 縮退判定（最終形状）: 収束した形状を最良近似平面に押し潰して LM を
 *  解き直したときの距離残差 RMS [m] がこれ未満なら、第3次元は測距精度で
 *  観測できていない（= 縮退）とみなす。
 *
 *  厚みの RMS（DEGEN_EXTENT）だけでは同一平面 + ノイズを捕まえられない:
 *  平面配置では ∂d/∂z = 0 なので、距離の誤差 σ は z² /(2d) ≈ σ すなわち
 *  z ≈ sqrt(2dσ)（d=4m, σ=5cm で 0.6m）の見かけの厚みとして吸収される。
 *  逆に「平面に押し潰しても残差が測距ばらつき程度しか増えない」なら、
 *  その配置の z は観測できていない。厚み h の配置が平面化で増やす残差は
 *  h²/(2d) 程度なので、この判定は h ≲ sqrt(2·d·THR)（d=5m で 0.8m）より
 *  薄い配置を縮退として弾くことになる。そのような配置は σ=5cm の測距
 *  では z の誤差が ≈ σ·d/h ≳ 0.3m になり、どのみち使い物にならない。
 *
 *  値の根拠: 同一平面 + 測距ばらつき σ のとき平面フィットの残差二乗和は
 *  σ²·χ²(dof2)、dof2 = m − (2n−3+1)。RMS = σ·sqrt(χ²/m) は n=6 (m=15,
 *  dof2=5) で中央値 0.58σ、99.9% 点 1.17σ、n=8 (m=28, dof2=14) で 0.71σ /
 *  1.14σ。σ=5cm なら 6cm が 99.9% 点に当たる。
 *  冗長度 0（3 次元フィットの残差が必ず 0）では測距ばらつきの情報が無く、
 *  2 次元モデルの自由度も余らない（n=5 で dof2=2）ので本物の 3 次元配置
 *  でも平面で説明できてしまう。その場合この判定は行わない（呼び出し側が
 *  冗長度 0 を必ず警告する前提。SURVEY_SPEC §1 (E)）。 */
#ifndef UWB_SURVEY_DEGEN_PLANAR_RMS
#define UWB_SURVEY_DEGEN_PLANAR_RMS ((uwb_real)0.06)
#endif

/** 縮退判定（最終形状）の相対版: 平面フィットの残差 RMS が 3 次元フィットの
 *  残差 RMS の K 倍未満なら縮退。ノイズの実現値が大きい試行では平面残差も
 *  絶対しきい値を超えうるが、そのとき 3 次元残差も同じだけ大きい（どちらも
 *  同じ測距誤差から来る）ので、比で見れば捕まる。同一平面 + ノイズでは
 *  cost2/cost3 ≈ 1 + 1.5·F(3,2) で K²=9 を超えるのは 16% 程度、本物の 3 次元
 *  配置（5m 級の部屋）では planar ≈ 0.2m vs rms3d ≈ 0.02m で比 ≈ 10 なので
 *  K=3 は両者の間に入る。 */
#ifndef UWB_SURVEY_DEGEN_PLANAR_K
#define UWB_SURVEY_DEGEN_PLANAR_K ((uwb_real)3.0)
#endif

/** 外れ値リンクの判定（leave-one-out）。リンク k を 1 本外して解き直した
 *  ときのコスト（残差二乗和）の減少量 Δ_k = cost_full − cost_{−k} が
 *
 *      Δ_k > (OUTLIER_Z · OUTLIER_SIGMA)²        （既定 (4 × 0.05m)² = 0.04 m²）
 *
 *  を超える最大の k を落とす。線形近似では Δ_k = e_k² / (1 − h_k)（e_k は
 *  全リンク解の残差、h_k はてこ比）で、外れ値が無ければ Δ_k ~ σ²·χ²(1)。
 *  n=8（28 本）で Z=4 なら誤棄却は 28 × P(χ²(1) > 16) ≈ 0.2%、n=6（冗長度 2）
 *  では Δ_k ≤ cost_full ~ σ²·χ²(2) なので P(χ²(2) > 16) ≈ 0.03%。
 *  逆に大きさ ε の外れ値は Δ_k ≈ (1 − h_k)·ε² なので、|ε| > Z·σ/sqrt(1−h_k)
 *  なら落とせる（平均 1−h = 冗長度/リンク数: n=6 で 0.13、n=8 で 0.32。
 *  てこ比の高いリンクほど外れ値が解に吸収されて見えにくい）。
 *
 *  σ はデータから推定せず **仮定値（UWB の測距ばらつき 5cm）** を使う。
 *  データから推定しようとすると、(a) 冗長度 2 では残差に 2 自由度しか無く
 *  「無雑音 + 小さい外れ値」と「5cm ノイズ」を区別できない、(b) 残差の
 *  MAD や「他のリンクの Δ の中央値」は外れ値が部分的に吸収されると膨らみ
 *  （マスキング）、冗長度の低い 6 台構成で外れ値を見逃す（レビュー A-2:
 *  (2,5) に +2m で残差 −0.16、MAD しきい値 0.30 に届かず 1.33m 誤る）、
 *  (c) 外したあとの残差から推定すると 2 本目の外れ値で膨らむ、ため。
 *  測距ばらつきは機器の性質でほぼ一定、環境に依る誤差はバイアス（= 落とし
 *  たい外れ値）として現れる、という判断。
 *  Δ_k ≤ cost_full なので、cost_full がしきい値以下なら leave-one-out は
 *  走らせない（外れ値の無い入力では通常ここで終わり、追加コストは無い）。 */
#ifndef UWB_SURVEY_OUTLIER_SIGMA
#define UWB_SURVEY_OUTLIER_SIGMA ((uwb_real)0.05)
#endif
#ifndef UWB_SURVEY_OUTLIER_Z
#define UWB_SURVEY_OUTLIER_Z ((uwb_real)4.0)
#endif
#define UWB_SURVEY_OUTLIER_DELTA_MIN \
    (UWB_SURVEY_OUTLIER_Z * UWB_SURVEY_OUTLIER_Z * UWB_SURVEY_OUTLIER_SIGMA * UWB_SURVEY_OUTLIER_SIGMA)

/** leave-one-out の LM（全リンク解からのウォームスタート、および鏡映後）の
 *  反復上限と収束判定。Δ_k の比較にしか使わないので本解きほど詰めない。
 *  打ち切られた場合は cost_{−k} が大きめに出る（= Δ_k が小さめ）ので、
 *  誤棄却の側には倒れない。上限 30 → 20 → 12 で判定結果（無雑音 1 本 ×
 *  全位置、2 本 × 全ペア、ノイズ下の正答率・誤棄却率）は 1 件も変わらず、
 *  コレスキー呼出は n=8 の外れ値入りで 8650 → 5190 回/solve に減った。 */
#ifndef UWB_SURVEY_LOO_MAX_ITER
#define UWB_SURVEY_LOO_MAX_ITER 12
#endif
#ifndef UWB_SURVEY_LOO_TOL
#define UWB_SURVEY_LOO_TOL ((uwb_real)1e-6)
#endif

/** 落とすリンクの最大本数。 */
#define UWB_SURVEY_MAX_DROP 2

/** 推定した共通遅延 |δ| がこれ [m] を超えたら delay_suspect を立てる。
 *  アンテナ遅延は片道 0.15m 前後（DW3000 の既定 16385 tick ≈ 0.154m）
 *  なので、0.3m を超える値は配線・配置・測距の誤りを疑う。 */
#ifndef UWB_SURVEY_DELAY_SUSPECT
#define UWB_SURVEY_DELAY_SUSPECT ((uwb_real)0.30)
#endif

/** 最短経路補完で使う「無限大」。加算しても溢れない大きさにする。 */
#define UWB_SURVEY_INF ((uwb_real)1e18)

/* ------------------------------------------------------------ 内部状態 */

typedef struct {
    int      n;               /* ノード数 */
    int      nlink;           /* 測定のあるリンク本数 */
    int      li[LMAX];        /* リンクの端点（li < lj） */
    int      lj[LMAX];
    int      lbit[LMAX];      /* excluded のビット位置 */
    uwb_real r[LMAX];         /* 測距値 [m]（遅延を含んだ生の値） */
    int      use[LMAX];       /* 1 = 採用中 */
    uwb_real p[NMAX][3];      /* 現在の座標 */
    uwb_real delay;           /* 共通遅延（片道） */
    int      est_delay;       /* 0 なら delay を 0 に固定して解く */
    int      planar;          /* 1 なら plane_n に直交する方向へ動かさない（縮退判定用） */
    uwb_real plane_n[3];      /* planar のときの平面法線（単位） */
} survey_ctx;

int uwb_survey_link_index(int i, int j)
{
    int a, b;
    if (i == j) return -1;
    if (i < 0 || j < 0 || i >= NMAX || j >= NMAX) return -1;
    a = (i < j) ? i : j;
    b = (i < j) ? j : i;
    /* ノード数に依存しない三角並び: (0,1)=0, (0,2)=1, (1,2)=2, (0,3)=3, ... */
    return (b * (b - 1)) / 2 + a;
}

void uwb_survey_input_init(uwb_survey_input *in, int n)
{
    if (!in) return;
    memset(in, 0, sizeof(*in));
    in->n = n;
}

/* =====================================================================
 * [1] 入力の取り込みと可解性の検査
 * ===================================================================== */

/* dist は対称のはずだが、両方向とも有効なら平均を採る（往復2回の測距を
 * そのまま入れられるようにするため）。片方だけが有効（もう片方が欠測か
 * NaN・非正）ならその片方を使う（A-8: 片方向の NaN でリンクを丸ごと
 * 捨てない）。両方とも無効なら欠測。
 *
 * 可解性の必要条件を 2 つ見る:
 *   (a) どのノードもリンクを 3 本以上持つこと。3次元の位置 3 自由度を
 *       距離で決めるには最低 3 本要る（2 本だと円周上に残る）
 *   (b) リンク総数 m >= 3n-6。3次元の形状自由度はゲージ 6 を引いて 3n-6
 *
 * さらに共通遅延 delta の識別可能性は m >= 3n-6+1。満たさない場合
 * （n=4 は最大 6 本 vs 必要 7 本なので常に該当）は delta=0 に固定する。
 * 固定しないと「配置全体をわずかに縮めて delta を増やす」方向が完全に
 * 平坦な谷になり、リッジで最小ノルム解が選ばれるだけで意味のある値に
 * ならないため、明示的に落としておく方が正直。 */
static int build_links(const uwb_survey_input *in, survey_ctx *c)
{
    int i, j, k = 0;
    int deg[NMAX];
    int shape;

    c->n = in->n;
    for (i = 0; i < c->n; ++i) deg[i] = 0;

    for (j = 1; j < c->n; ++j) {
        for (i = 0; i < j; ++i) {
            uwb_real v;
            /* 「有効」= フラグあり かつ 正の有限値（NaN は比較が偽になる） */
            int vij = (in->have[i][j] != 0) && (in->dist[i][j] > (uwb_real)0) &&
                      (in->dist[i][j] < UWB_SURVEY_INF);
            int vji = (in->have[j][i] != 0) && (in->dist[j][i] > (uwb_real)0) &&
                      (in->dist[j][i] < UWB_SURVEY_INF);
            if (!vij && !vji) continue;
            if (vij && vji)      v = (uwb_real)0.5 * (in->dist[i][j] + in->dist[j][i]);
            else if (vij)        v = in->dist[i][j];
            else                 v = in->dist[j][i];

            c->li[k]   = i;
            c->lj[k]   = j;
            c->r[k]    = v;
            c->use[k]  = 1;
            c->lbit[k] = uwb_survey_link_index(i, j);
            ++k;
            ++deg[i];
            ++deg[j];
        }
    }
    c->nlink     = k;
    c->delay     = (uwb_real)0;
    c->est_delay = 0;
    c->planar    = 0;
    uwb_v3_zero(c->plane_n);

    for (i = 0; i < c->n; ++i) if (deg[i] < 3) return 0;

    shape = 3 * c->n - 6;
    if (k < shape) return 0;
    c->est_delay = (k >= shape + 1) ? 1 : 0;
    return 1;
}

/* =====================================================================
 * [2] 欠測リンクの補完と逐次三辺測量による初期配置
 * ===================================================================== */

/* 欠測リンクの距離を **グラフ上の最短経路**で埋める（Floyd-Warshall）。
 *
 * なぜ最短経路か:
 *   - 初期値づくりは距離行列が全部埋まっていないと使えない。埋め方は
 *     「平均値で埋める」「反復付き重み付き MDS(SMACOF)」など色々あるが、
 *     最短経路（測地距離）は **三角不等式を必ず満たす**ので、Gram 行列が
 *     極端に非正定値になりにくい。ISOMAP で使われているのと同じ理屈
 *   - 直線見通しなら r_ij ≈ 実距離なので、2 ホップの和は真値の上界に
 *     なるだけで済む（少し伸びた初期値になるが、続く LM が直す）
 *   - n<=8 なので O(n^3)=512 回。malloc も反復も要らない
 *
 * 非連結（どこかへ辿り着けない）なら 0 を返す。除算 0、sqrt 0。 */
static int complete_distances(const survey_ctx *c, uwb_real d[NMAX][NMAX])
{
    int i, j, k, n = c->n;

    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            d[i][j] = (i == j) ? (uwb_real)0 : UWB_SURVEY_INF;

    for (k = 0; k < c->nlink; ++k) {
        d[c->li[k]][c->lj[k]] = c->r[k];
        d[c->lj[k]][c->li[k]] = c->r[k];
    }

    for (k = 0; k < n; ++k)
        for (i = 0; i < n; ++i)
            for (j = 0; j < n; ++j) {
                uwb_real s;
                if (d[i][k] >= UWB_SURVEY_INF || d[k][j] >= UWB_SURVEY_INF) continue;
                s = d[i][k] + d[k][j];
                if (s < d[i][j]) d[i][j] = s;
            }

    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            if (d[i][j] >= UWB_SURVEY_INF) return 0;
    return 1;
}

/* 逐次三辺測量（ピボット付きコレスキー 3 段）による初期配置。
 *
 * ノード 0 を原点に取った Gram 行列
 *     G[i][j] = (d_0i² + d_0j² − d_ij²) / 2        (i, j >= 1)
 * は余弦定理そのもので、G = X Xᵀ（X は (n−1)×3 の座標行列）。したがって
 * 対角ピボット付きコレスキーを 3 段だけ回せば L の 3 列がそのまま座標になる:
 *   - ピボット 1 = 原点から最も遠いノード       → +X 軸上
 *   - ピボット 2 = その直線から最も遠いノード   → XY 平面、y > 0
 *   - ピボット 3 = その平面から最も遠いノード   → z > 0
 *   - 残りのノードは「原点 + ピボット 3 点への距離」から閉形式で決まる
 *     （= 4 球交差の三辺測量。L[i][k] = G[i][pk] / L[pk][k] がその式）
 * 古典的 MDS（二重中心化 + 8×8 Jacobi ≈ 20k flops + 400 sqrt）と違って
 * 固有分解が要らず、≈ 300 flops + sqrt 3 + 除算 3 で済む。
 *
 * 縮退判定は MDS の第3固有値と同じ意味の量で行う:
 *   - 3 段目のピボット値 = 平面から最も離れたノードの二乗距離（最大主軸の
 *     ピボット値との比で見る。スケール非依存）
 *   - 2 段後の残差対角の総和 = 各ノードの平面からの二乗距離の和なので、
 *     sqrt(Σ/n) が「第3軸方向の RMS 広がり」[m]（物理しきい値で見る）
 * ただし、この段の判定は **共通遅延 δ が乗った生の距離**で行うので、
 * 同一平面 + δ≠0 は (r = d + 2δ により三角形が膨らんで) 見逃す。
 * 最終判定は LM 収束後の shape_degenerate() が δ 除去済みの形状で行う（A-1）。
 *
 * 遅延 delta は初期値 0 として d = r をそのまま使う。全リンクが 2δ ぶん
 * 一様に伸びるだけなので、初期配置は少し膨らむが形は保たれ、LM が縮める。
 *
 * dim = 3 が通常。dim = 2 は 2 段で止めて XY 平面内の配置（z = 0）を作る
 * （shape_degenerate の平面フィットの第 2 初期値）。
 * 除算 dim（各段 1/L_kk）、sqrt dim。 */
static int trilat_init(survey_ctx *c, const uwb_real d[NMAX][NMAX], int dim, int *degenerate)
{
    int n = c->n, m = n - 1, i, j, k;
    uwb_real G[NMAX - 1][NMAX - 1];
    uwb_real L[NMAX - 1][3];
    uwb_real piv[3] = {(uwb_real)0, (uwb_real)0, (uwb_real)0};
    uwb_real resid2 = (uwb_real)0;
    int used[NMAX - 1], rank = 0;

    for (i = 0; i < m; ++i) {
        used[i] = 0;
        L[i][0] = L[i][1] = L[i][2] = (uwb_real)0;
        for (j = 0; j < m; ++j) {
            uwb_real a = d[0][i + 1], b = d[0][j + 1], e = d[i + 1][j + 1];
            G[i][j] = (uwb_real)0.5 * (a * a + b * b - e * e);
        }
    }

    for (k = 0; k < dim; ++k) {
        int pk = -1;
        uwb_real best = (uwb_real)0, inv;
        for (i = 0; i < m; ++i)
            if (!used[i] && G[i][i] > best) { best = G[i][i]; pk = i; }
        if (k == 2) {
            /* 2 段後の残差対角 = 各ノードの「平面からの二乗距離」。縮退判定用。
             * 丸めで負になったものは 0 扱い。 */
            for (i = 0; i < m; ++i)
                if (!used[i] && G[i][i] > (uwb_real)0) resid2 += G[i][i];
        }
        piv[k] = (pk >= 0) ? best : (uwb_real)0;
        if (pk < 0) break;
        if (k > 0 && !(best > piv[0] * UWB_MATH_RANK_TOL)) break;   /* ランク落ち */
        used[pk] = 1;
        rank     = k + 1;
        inv      = (uwb_real)1 / uwb_math_sqrt(best);
        for (i = 0; i < m; ++i) L[i][k] = G[i][pk] * inv;
        for (i = 0; i < m; ++i)
            for (j = 0; j < m; ++j) G[i][j] -= L[i][k] * L[j][k];
    }
    if (!(piv[0] > (uwb_real)0)) return 0;    /* 全点が同一点。距離がおかしい */

    /* 相対条件（スケール非依存）と物理条件（測距ばらつきより薄いか）の両方で見る。
     * sqrt(resid2/n) < EXTENT は両辺を二乗して resid2 < n·EXTENT² と同値。
     * dim=2（平面フィットの初期値）のときは 3 段目が無いので判定しない。 */
    if (dim >= 3)
        *degenerate = (rank < 3) ||
                      (piv[2] < UWB_SURVEY_DEGEN_RATIO * piv[0]) ||
                      (resid2 < (uwb_real)n * UWB_SURVEY_DEGEN_EXTENT * UWB_SURVEY_DEGEN_EXTENT);
    else
        *degenerate = (rank < 2);

    uwb_v3_zero(c->p[0]);
    for (i = 0; i < m; ++i) uwb_v3_copy(L[i], c->p[i + 1]);
    return 1;
}

/* =====================================================================
 * [3] Gauss-Newton (Levenberg-Marquardt)
 * ===================================================================== */

/* 【C の型規則】C には T(*)[N] から const T(*)[N] への暗黙変換が無い
 * （C++ には有る）。非 const の配列を const 引数へ渡すと、GCC の
 * -Wpedantic が "invalid use of pointers to arrays with different
 * qualifiers in ISO C before C2X" を出す。本リポジトリの `make strict` は
 * -Werror なのでビルドが落ちる（clang は許容するので macOS では気づけない。
 * GitHub Actions の GCC ビルドで発覚した）。
 * C2x で解消される規則だが、それまでは明示キャストで通す。 */
#define UWB_SURVEY_CP3(p)  ((const uwb_real (*)[3])(p))
#define UWB_SURVEY_CPN(d)  ((const uwb_real (*)[NMAX])(d))

/* 残差 f_k = |p_i - p_j| + 2*delta - r_k を作り、二乗和を返す。
 * 除算 0、sqrt はリンク本数ぶん。 */
static uwb_real cost_at(const survey_ctx *c, const uwb_real p[NMAX][3],
                        uwb_real delay, uwb_real *f)
{
    int k;
    uwb_real s = (uwb_real)0;
    for (k = 0; k < c->nlink; ++k) {
        uwb_real dv[3];
        f[k] = (uwb_real)0;
        if (!c->use[k]) continue;
        uwb_v3_sub(p[c->li[k]], p[c->lj[k]], dv);
        f[k] = uwb_v3_norm(dv) + (uwb_real)2 * delay - c->r[k];
        s += f[k] * f[k];
    }
    return s;
}

/* 正規方程式 A = JᵀJ（減衰前）と右辺 g = −Jᵀf を作る。
 *
 * ヤコビアンの 1 行は 7 個しか非零成分を持たない:
 *     df/dp_i = u,  df/dp_j = -u,  df/ddelta = 2,   u = (p_i-p_j)/|p_i-p_j|
 * ので、J そのものを持たずに外積を足し込む。A は 3×3 ブロックの下三角
 * パック（uwb_bchol）で、1 リンクの寄与は
 *     A_ii += u uᵀ,  A_jj += u uᵀ,  A_ij −= u uᵀ,  a_i += 2u, a_j −= 2u, add += 4
 * と u uᵀ の 6 要素を 1 回作れば済む（密の 7×7 外積 49 乗算 → 7 乗算）。
 *
 * 減衰（Marquardt スケーリング）は呼び出し側が uwb_bchol_damp_diag で掛ける。
 * c->planar のときはヤコビ行を平面へ射影する（shape_degenerate の平面フィット用）。
 * 除算 1/リンク（1/|p_i−p_j|）、sqrt 1/リンク。 */
static void build_normal(const survey_ctx *c, const uwb_real p[NMAX][3],
                         const uwb_real *f, uwb_bchol *A, uwb_real *g)
{
    int k, a, nx = 3 * c->n + (c->est_delay ? 1 : 0);

    uwb_bchol_zero(A, c->n, c->est_delay);
    for (a = 0; a < nx; ++a) g[a] = (uwb_real)0;

    for (k = 0; k < c->nlink; ++k) {
        int      i, j;
        uwb_real dv[3], dd, u[3], mf;

        if (!c->use[k]) continue;
        i = c->li[k];
        j = c->lj[k];
        uwb_v3_sub(p[i], p[j], dv);
        dd = uwb_v3_norm(dv);
        if (dd < UWB_MATH_TINY) {
            /* 2 点が重なった。方向が定義できないので任意の単位ベクトルを
             * 使う（次の反復で離れる）。 */
            u[0] = (uwb_real)1; u[1] = (uwb_real)0; u[2] = (uwb_real)0;
        } else {
            uwb_real inv = (uwb_real)1 / dd;
            u[0] = dv[0] * inv; u[1] = dv[1] * inv; u[2] = dv[2] * inv;
        }
        if (c->planar) {
            /* 平面拘束: ヤコビ行から法線成分を落とす。法線方向の正規方程式は
             * リッジだけ・右辺 0 になり、更新量は厳密に 0（面内に留まる） */
            uwb_v3_axpy(-uwb_v3_dot(u, c->plane_n), c->plane_n, u);
        }

        uwb_bchol_add_pair_outer(A, i, j, u, (uwb_real)2);

        mf = -f[k];                            /* g = −Jᵀf（A dx = g で dx が降下方向） */
        uwb_v3_axpy( mf, u, g + 3 * i);
        uwb_v3_axpy(-mf, u, g + 3 * j);
        if (c->est_delay) g[3 * c->n] += (uwb_real)2 * mf;
    }
}

/* LM 本体。
 *
 * ゲージの不定性（並進3 + 回転3）で J^T J は必ず 6 次元のヌル空間を持つ。
 * 対処は **リッジ（LM 減衰）** を選んだ。理由:
 *
 *   1. ゲージ拘束を式で足す方法（p_0 を固定、p_1 を x 軸に載せる 等）は、
 *      「拘束に使うノードが良い位置にある」ことを暗に仮定する。測量では
 *      どのノードがどこにあるか分からないので、たまたま p_0 と p_1 が
 *      近接していると拘束自体が悪条件になる
 *   2. リッジを入れれば A は厳密に正定値になり、S2 の要件どおり
 *      コレスキーだけで解ける（ピボットも特異値分解も要らない）
 *   3. ゲージ方向は残差を 1 ミリも変えない（剛体運動だから）。減衰付き
 *      最小ノルム更新はその方向へ動かないので、解は初期値のフレームの
 *      近くに留まる。どのみち [4][5] で剛体変換し直すので、フレームが
 *      どこにあるかは結果に影響しない
 *   4. LM のステップ制御は、欠測リンクを最短経路で埋めた「少し歪んだ」
 *      初期値から始めるときの発散よけにもなる（純 Gauss-Newton は
 *      ここで飛ぶことがある）
 *
 * 減衰は成功で 0.3 倍、失敗で 10 倍という定石。成功するたび lambda が
 * 下がるので、収束付近では実質ただの Gauss-Newton になり二次収束する。
 *
 * 正規方程式は試行のたびに組み直す（build_normal は 28 リンク × 10 乗算で
 * 安いうえ、減衰前の行列を退避するコピー（double で 3KB）をスタックに
 * 置かずに済む）。
 * 1 試行あたり: build_normal（除算 m、sqrt m）+ factor（除算 3n+1、sqrt 3n+1）
 * + solve（除算 0）+ cost_at（sqrt m）。
 * max_iter / tol は反復上限と収束判定（更新量ノルム [m]）。本解きは
 * UWB_SURVEY_MAX_ITER / UWB_SURVEY_TOL、leave-one-out は緩い値で呼ぶ。 */
static int lm_run(survey_ctx *c, uwb_real *f, int *iters, uwb_real *cost_out,
                  int max_iter, uwb_real tol)
{
    uwb_bchol A;
    uwb_real  g[XMAX], dx[XMAX];
    uwb_real  ptry[NMAX][3], ftry[LMAX];
    uwb_real  lambda = UWB_SURVEY_LM_INIT;
    uwb_real  cost;
    int       nx = 3 * c->n + (c->est_delay ? 1 : 0);
    int       it, a, i;

    *iters = 0;
    cost   = cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
    if (uwb_math_isnan(cost)) return 0;

    for (it = 0; it < max_iter; ++it) {
        int      accepted = 0;
        uwb_real step = (uwb_real)0;

        while (lambda <= UWB_SURVEY_LM_MAX) {
            uwb_real ntry, dtry, s2 = (uwb_real)0;

            build_normal(c, UWB_SURVEY_CP3(c->p), f, &A, g);
            uwb_bchol_damp_diag(&A, lambda, UWB_SURVEY_RIDGE_ABS);
            if (!uwb_bchol_factor(&A)) { lambda *= (uwb_real)10; continue; }
            uwb_bchol_solve(&A, g, dx);

            for (i = 0; i < c->n; ++i) uwb_v3_add(c->p[i], dx + 3 * i, ptry[i]);
            dtry = c->delay + (c->est_delay ? dx[3 * c->n] : (uwb_real)0);

            /* 試行の残差は ftry に入れる。f は「現在の座標での残差」で
             * なければならない（次の build_normal が使う）。 */
            ntry = cost_at(c, UWB_SURVEY_CP3(ptry), dtry, ftry);
            if (!uwb_math_isnan(ntry) && ntry < cost) {
                for (a = 0; a < nx; ++a) s2 += dx[a] * dx[a];
                step = uwb_math_sqrt(s2);
                for (i = 0; i < c->n; ++i) uwb_v3_copy(ptry[i], c->p[i]);
                for (a = 0; a < c->nlink; ++a) f[a] = ftry[a];
                c->delay = dtry;
                cost     = ntry;
                lambda  *= (uwb_real)0.3;
                if (lambda < UWB_SURVEY_LM_MIN) lambda = UWB_SURVEY_LM_MIN;
                accepted = 1;
                ++(*iters);
                break;
            }
            lambda *= (uwb_real)10;
        }

        /* 減衰をいくら上げても下がらない = 局所最小に着いた（または破綻）。
         * どちらにせよ残差で採否を判断してもらう。 */
        if (!accepted) break;
        if (step < tol) break;
    }

    /* f を最終座標のものに戻しておく（外れ値判定と残差 RMS が使う） */
    cost = cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
    if (uwb_math_isnan(cost)) return 0;
    for (i = 0; i < c->n; ++i)
        if (uwb_math_isnan(c->p[i][0]) || uwb_math_isnan(c->p[i][1]) || uwb_math_isnan(c->p[i][2]))
            return 0;
    *cost_out = cost;
    return 1;
}

/* --------------------------------------------- 局所解からの脱出（鏡映） */

/* ノード i の隣接点が作る最良近似平面（重心 cen と法線 nrm）を求める。
 * 法線は散布行列（3x3 対称）の最小固有値の固有ベクトル（= 主成分分析の
 * 第3軸）で、uwb_sym3_min_eigvec（閉形式の固有値 + 行外積）で取る。
 * 隣接点が 3 つ未満、または隣接点が一直線で法線が決まらない（λ1 ≈ λ2）
 * なら 0 を返す（鏡映する意味が無いので飛ばす）。
 * 除算 1（1/m）+ min_eigvec（sqrt 6、acos 1、cos 1、除算 5）。 */
static int neighbor_plane(const survey_ctx *c, int i, uwb_real *cen, uwb_real *nrm)
{
    uwb_real M[6], inv_m;
    int k, m = 0;

    uwb_v3_zero(cen);
    for (k = 0; k < c->nlink; ++k) {
        int o;
        if (!c->use[k]) continue;
        if (c->li[k] == i)      o = c->lj[k];
        else if (c->lj[k] == i) o = c->li[k];
        else continue;
        uwb_v3_add(cen, c->p[o], cen);
        ++m;
    }
    if (m < 3) return 0;
    inv_m = (uwb_real)1 / (uwb_real)m;
    uwb_v3_scale(inv_m, cen);

    uwb_sym3_zero(M);
    for (k = 0; k < c->nlink; ++k) {
        uwb_real q[3];
        int o;
        if (!c->use[k]) continue;
        if (c->li[k] == i)      o = c->lj[k];
        else if (c->lj[k] == i) o = c->li[k];
        else continue;
        uwb_v3_sub(c->p[o], cen, q);
        uwb_sym3_add_outer(M, q);
    }
    return uwb_sym3_min_eigvec(M, NULL, nrm);
}

/* 距離だけの最小二乗は **局所解を持つ**。とくに欠測リンクがあると、
 * 「あるノードだけが隣接点の作る平面の反対側に落ちた」形の偽の最小に
 * 捕まりやすい（分子構造決定でよく知られた flip ambiguity）。
 *
 * そこで LM が止まったあと、各ノードを隣接平面で鏡映した点から解き直し、
 * コストが下がったら採用する、というスイープを回す。ノード数が 8 以下なので
 * 全ノード×数スイープでも一瞬で終わる（測量は設置時に1回だけの処理）。
 *
 * 真の最小に着いていれば、どの鏡映もコストを下げられないので何も起きない。
 * max_iter / tol は鏡映後に回す LM のもの（lm_run と同じ意味）。 */
static int escape_local_minima(survey_ctx *c, uwb_real *f, int *iters, uwb_real *cost,
                               int max_iter, uwb_real tol)
{
    uwb_real bp[NMAX][3], bf[LMAX], bd;
    int      sweep, i, k, it;

    for (sweep = 0; sweep < 3; ++sweep) {
        int improved = 0;
        for (i = 0; i < c->n; ++i) {
            uwb_real cen[3], nrm[3], q[3], dn, trial;

            /* 現在の最良解を退避 */
            for (k = 0; k < c->n; ++k) uwb_v3_copy(c->p[k], bp[k]);
            for (k = 0; k < c->nlink; ++k) bf[k] = f[k];
            bd = c->delay;

            if (!neighbor_plane(c, i, cen, nrm)) continue;
            uwb_v3_sub(c->p[i], cen, q);
            dn = uwb_v3_dot(q, nrm);
            uwb_v3_axpy((uwb_real)-2 * dn, nrm, c->p[i]);

            it    = 0;
            trial = *cost;
            if (lm_run(c, f, &it, &trial, max_iter, tol) &&
                trial < *cost - (uwb_real)1e-12 * ((uwb_real)1 + *cost)) {
                *cost = trial;
                *iters += it;
                improved = 1;
                continue;                     /* 採用（退避は捨てる） */
            }
            /* 戻す */
            for (k = 0; k < c->n; ++k) uwb_v3_copy(bp[k], c->p[k]);
            for (k = 0; k < c->nlink; ++k) f[k] = bf[k];
            c->delay = bd;
        }
        if (!improved) break;
    }
    return 1;
}

/* --------------------------------------------------------- 外れ値リンク */

/* leave-one-out による外れ値リンクの棄却（レビュー A-2）。
 *
 * 以前は全リンク解の残差の MAD で判定していたが、残差は外れ値込みの最小二乗
 * の後の値なので、冗長度が低いと外れ値が座標と δ に吸収されて残差が小さく
 * なり（マスキング）、公称 6 台構成（15 本 vs 未知数 13）で +2m の外れ値を
 * 落とせなかった。ここでは採用中の各リンク k について
 *
 *   1. k を外し、収束済みの全リンク解（座標と δ）からウォームスタートで LM を
 *      回し、さらに escape_local_minima（各ノードを隣接平面で鏡映して解き直す）
 *      を掛けて cost_{−k}（k 抜きで到達できる最小コスト）を求める。
 *      鏡映が要るのは、大きな外れ値（2m）が全リンク解そのものを **別の局所解**
 *      （あるノードが隣接平面の反対側へ落ちた形。コストは 0.03〜0.25 と小さく、
 *      外れ値がほぼ吸収されている）へ連れて行くことがあるため。そこから k を
 *      外して LM を回すだけでは元の形に戻れず（n=8 の +2m で 1/28、n=6 で
 *      4/15 の位置: 真の外れ値を外しても cost 0.01〜0.12 で止まる）、無関係な
 *      リンクと見分けがつかない。鏡映を掛ければ 0 まで落ちる。k を欠測にした
 *      最短経路補完からのコールドスタートも試したが、全リンク解と同じ局所解に
 *      落ちるだけで（全ケースでウォームスタートと同値）効かなかった
 *   2. Δ_k = cost_full − cost_{−k}（k を外すことで説明できるようになる量）
 *
 * を作り、Δ_k が最大のリンクが UWB_SURVEY_OUTLIER_DELTA_MIN を超えていれば
 * 落とす。外れ値そのものを外せば残りは整合するので cost_{−k} ≈ ノイズ分、
 * 無関係なリンクを外しても外れ値は残るので cost_{−k} は大きいまま、という
 * 差で見分ける（残差の大小ではなく「外すと説明できるか」を直接試すので、
 * 吸収されていても捕まる）。「外れ値がちょうど 1 本」の仮説の下では
 * argmin_k cost_{−k} がその添字の最尤推定なので、これが最も筋の良い選び方。
 *
 * 限界:
 *   - **冗長度 2（6 台全リンク）では同定の余裕が薄い**: 1 本外すと冗長度 1
 *     になり、残る拘束は 1 本だけなので、無関係なリンクを外しても外れ値が
 *     数 mm の残差まで吸収される（cost_{−j} ≈ 0.0001〜0.004）。無雑音なら
 *     真の外れ値の cost_{−k} = 0 と区別できるが、5cm のばらつきの下では隣の
 *     候補と取り違える（実測: +2m を 200 試行で正答 94 / 取り違え 100）。
 *     次点の候補も測距ばらつきの範囲（DELTA_MIN）で説明できてしまうときは
 *     *ambiguous に 1 を立てる（= 冗長度を上げないと誰が外れ値か決まらない）
 *   - 同じノードに外れ値が 2 本乗ると（例: (0,1) と (1,5)）、そのノードを
 *     動かして両方を妥協させる方が 1 本ずつ外すより安く、無関係なリンクを
 *     外す方が Δ が大きく出ることがある。1 本ずつの greedy では捕まらない
 *     （総当たりの leave-two-out は 378 組 × 鏡映付き LM で高すぎる）
 *
 * 落とさない条件:
 *   - 落とすと冗長度が 0 になる（識別不能。残差 0 の別解に落ちるので、
 *     外れ値が混ざっていても落とさず、呼び出し側の冗長度警告に委ねる）
 *   - 落とすとどちらかの端点の次数が 3 未満になる
 *   - cost_full がしきい値以下（Δ_k ≤ cost_full なので調べるまでもない。
 *     外れ値の無い入力はほぼここで終わる）
 * 落としたら 1 を返し、c の座標・δ は最良候補の解（k 抜きで収束済み）に、
 * f はその残差にしてある（続く本解きはそこから始めればよい。元の全リンク解が
 * 別の局所解だった場合にそこへ戻らないため）。落とさなければ 0 を返し、
 * c の座標・δ・f は呼出前の状態に戻す。
 * コスト: 採用リンク本数 × （LM 1 本 + 鏡映 n 回 × LM）。n=8 で 1 掃引
 * ≈ 3000〜4000 回のコレスキー（設置時 1 回の処理。外れ値の無い入力では
 * cost_full のしきい値で飛ばされ、ほぼ 0）。 */
static int loo_drop_link(survey_ctx *c, uwb_real *f, uwb_real cost_full,
                         unsigned long *excluded, int *ambiguous, int *iters)
{
    uwb_real bp[NMAX][3], bd;                /* 全リンク解（ウォームスタートの起点） */
    uwb_real sp[NMAX][3], sd = (uwb_real)0;  /* これまでの最良候補の解 */
    uwb_real best_cost = UWB_SURVEY_INF, second_cost = UWB_SURVEY_INF;
    int      deg[NMAX];
    int      k, i, m = 0, best_k = -1, need;

    for (i = 0; i < c->n; ++i) deg[i] = 0;
    for (k = 0; k < c->nlink; ++k) {
        if (!c->use[k]) continue;
        ++m;
        ++deg[c->li[k]];
        ++deg[c->lj[k]];
    }

    need = 3 * c->n - 6 + (c->est_delay ? 1 : 0);
    if (m - 1 < need + 1) return 0;                     /* 落とすと冗長度が無くなる */
    if (!(cost_full > UWB_SURVEY_OUTLIER_DELTA_MIN)) return 0;

    for (i = 0; i < c->n; ++i) uwb_v3_copy(c->p[i], bp[i]);
    bd = c->delay;

    for (k = 0; k < c->nlink; ++k) {
        uwb_real cost_k;
        int      it = 0, ok;

        if (!c->use[k]) continue;
        if (deg[c->li[k]] <= 3 || deg[c->lj[k]] <= 3) continue;   /* 次数を割る */

        c->use[k] = 0;
        for (i = 0; i < c->n; ++i) uwb_v3_copy(bp[i], c->p[i]);
        c->delay = bd;
        ok = lm_run(c, f, &it, &cost_k, UWB_SURVEY_LOO_MAX_ITER, UWB_SURVEY_LOO_TOL) &&
             escape_local_minima(c, f, &it, &cost_k, UWB_SURVEY_LOO_MAX_ITER, UWB_SURVEY_LOO_TOL);
        c->use[k] = 1;
        *iters += it;
        if (!ok) continue;

        if (cost_k < best_cost) {
            second_cost = best_cost;
            best_cost   = cost_k;
            best_k      = k;
            for (i = 0; i < c->n; ++i) uwb_v3_copy(c->p[i], sp[i]);
            sd = c->delay;
        } else if (cost_k < second_cost) {
            second_cost = cost_k;
        }
    }

    if (best_k >= 0 && cost_full - best_cost > UWB_SURVEY_OUTLIER_DELTA_MIN) {
        c->use[best_k] = 0;
        if (c->lbit[best_k] >= 0 && c->lbit[best_k] < 32)
            *excluded |= (1UL << c->lbit[best_k]);
        if (second_cost <= UWB_SURVEY_OUTLIER_DELTA_MIN) *ambiguous = 1;
        for (i = 0; i < c->n; ++i) uwb_v3_copy(sp[i], c->p[i]);
        c->delay = sd;
        (void)cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
        return 1;
    }

    /* 落とさない: 全リンク解に戻す */
    for (i = 0; i < c->n; ++i) uwb_v3_copy(bp[i], c->p[i]);
    c->delay = bd;
    (void)cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
    return 0;
}

/* ------------------------------------------------- 最終形状の縮退判定 */

/* LM が収束した形状（δ 除去済み）で縮退（同一平面・直線）を判定する（A-1）。
 *
 * 初期値段階の判定は生の距離 r = d + 2δ で行うため、同一平面でも δ ≠ 0 なら
 * 三角形が一様に膨らんで「厚み」が見えてしまい、δ=0.02m ですでに見逃す
 * （6 台同一平面 + δ=0.15 + 5cm ノイズで 200/200 回 ok=1、最大 1.5m の
 * 誤差で z が裏返る、が実測されている）。LM は δ を同時推定するので、
 * 収束後の形状には δ が乗っていない。ここで判定すれば δ に依らない。
 *
 * 判定は 2 段:
 *   1. 散布行列 S = Σ (p − p̄)(p − p̄)ᵀ の固有値（閉形式）。λ2 は「第3主軸
 *      方向の二乗広がりの総和」なので sqrt(λ2/n) がその軸の RMS 厚み [m]。
 *      これが DEGEN_EXTENT 未満、または λ2/λ0 < DEGEN_RATIO なら縮退
 *      （無雑音の同一平面・直線はここで決まる）。
 *   2. ノイズがあると同一平面でも見かけの厚み sqrt(2dσ) ≈ 0.6m が出るので
 *      1. では捕まらない。そこで形状を最良近似平面（λ2 の固有ベクトル n̂ に
 *      直交）へ射影し、**面内だけ**で LM を解き直す（c->planar。2 次元の
 *      局所解に捕まらないよう、初期値を 2 つ（射影した 3 次元解 / 2 次元の
 *      逐次三辺測量）試して良い方を採る）。その距離
 *      残差 RMS が max(DEGEN_PLANAR_RMS, DEGEN_PLANAR_K × 3 次元の残差 RMS)
 *      未満なら「平面でも測距を同程度に説明できる = 第3次元は観測されて
 *      いない」として縮退。冗長度 0 ではこの段は行わない。
 *      n̂ が決まらない（λ1 ≈ λ2。立方体・正四面体のように等方な配置）
 *      ときは厚みが明らかにあるので縮退ではない。
 *
 * 平面フィットの LM は c の座標・残差・δ を退避して走らせ、戻してから返る。
 * 除算 1（1/n）+ min_eigvec + 平面 LM ×2（通常それぞれ 5〜30 反復。
 * コレスキー呼出は solve 全体の 1/3 程度）。
 * d は補完済み距離行列、rms3d は 3 次元フィットの距離残差 RMS、redundancy は
 * 冗長度。縮退なら 1。planar_rms に平面フィットの残差 RMS（行わなければ 0）
 * を返す。 */
static int shape_degenerate(survey_ctx *c, const uwb_real d[NMAX][NMAX], uwb_real *f,
                            uwb_real rms3d, int redundancy, uwb_real *planar_rms)
{
    uwb_real cen[3], S[6], lam[3], q[3], nvec[3];
    uwb_real bp[NMAX][3], bf[LMAX], bd, cost2, best = UWB_SURVEY_INF, thr;
    uwb_real inv_n = (uwb_real)1 / (uwb_real)c->n;
    int i, it, nused = 0, dummy, start;

    *planar_rms = (uwb_real)0;

    uwb_v3_zero(cen);
    for (i = 0; i < c->n; ++i) uwb_v3_add(cen, c->p[i], cen);
    uwb_v3_scale(inv_n, cen);

    uwb_sym3_zero(S);
    for (i = 0; i < c->n; ++i) {
        uwb_v3_sub(c->p[i], cen, q);
        uwb_sym3_add_outer(S, q);
    }
    if (!uwb_sym3_min_eigvec(S, lam, nvec)) {
        /* 三重縮退（全点が 1 点）か λ1 ≈ λ2（等方な 3 次元配置）。前者は
         * λ2 ≤ 0 で縮退、後者は縮退でない。 */
        return !(lam[2] > (uwb_real)0) ||
               (lam[2] < (uwb_real)c->n * UWB_SURVEY_DEGEN_EXTENT * UWB_SURVEY_DEGEN_EXTENT);
    }

    if (!(lam[2] > (uwb_real)0) ||
        (lam[2] < UWB_SURVEY_DEGEN_RATIO * lam[0]) ||
        (lam[2] < (uwb_real)c->n * UWB_SURVEY_DEGEN_EXTENT * UWB_SURVEY_DEGEN_EXTENT))
        return 1;

    if (redundancy <= 0) return 0;          /* 測距ばらつきの情報が無い（上の定数の説明参照） */

    /* 2. 面内だけで解き直す。2 次元の距離最小二乗は局所解が多い（5cm の
     *    ノイズでも数 % の試行で捕まる）ので、初期値を 2 つ試して良い方を採る:
     *      start 0: 3 次元解を最良近似平面へ射影したもの
     *      start 1: 生の距離からの 2 次元逐次三辺測量（trilat_init dim=2）
     *    escape_local_minima は面内では回さない（隣接平面による鏡映は 2 次元の
     *    局所解に合わず、本物の 3 次元配置の平面残差を下げて偽陽性を増やす
     *    だけだった: n=6・σ=5cm で偽陽性 2.4% → 0%、コレスキー呼出 4.8 倍 → 1.5 倍）。 */
    for (i = 0; i < c->n; ++i) uwb_v3_copy(c->p[i], bp[i]);
    for (i = 0; i < c->nlink; ++i) bf[i] = f[i];
    bd = c->delay;

    for (start = 0; start < 2; ++start) {
        int ok;
        if (start == 0) {
            for (i = 0; i < c->n; ++i) {
                uwb_v3_sub(c->p[i], cen, q);
                uwb_v3_axpy(-uwb_v3_dot(q, nvec), nvec, c->p[i]);
            }
            uwb_v3_copy(nvec, c->plane_n);
        } else {
            if (!trilat_init(c, d, 2, &dummy)) break;
            c->plane_n[0] = c->plane_n[1] = (uwb_real)0;
            c->plane_n[2] = (uwb_real)1;
            c->delay = (uwb_real)0;
        }
        c->planar = 1;
        it = 0;
        cost2 = (uwb_real)0;
        ok = lm_run(c, f, &it, &cost2, UWB_SURVEY_MAX_ITER, UWB_SURVEY_TOL);
        c->planar = 0;
        if (ok && cost2 < best) best = cost2;

        for (i = 0; i < c->n; ++i) uwb_v3_copy(bp[i], c->p[i]);
        for (i = 0; i < c->nlink; ++i) f[i] = bf[i];
        c->delay = bd;
    }

    if (!(best < UWB_SURVEY_INF)) return 0; /* 平面では解けない = 平面ではない */
    for (i = 0; i < c->nlink; ++i) if (c->use[i]) ++nused;
    *planar_rms = (nused > 0) ? uwb_math_sqrt(best / (uwb_real)nused) : (uwb_real)0;

    thr = UWB_SURVEY_DEGEN_PLANAR_K * rms3d;
    if (thr < UWB_SURVEY_DEGEN_PLANAR_RMS) thr = UWB_SURVEY_DEGEN_PLANAR_RMS;
    return *planar_rms < thr;
}

/* =====================================================================
 * [4] 実測高さによるゲージ固定
 * ===================================================================== */

/* 球面拘束付き最小二乗 min uᵀMu − 2bᵀu s.t. |u|=1（Moré–Sorensen の厳密解）、
 * ランク 2 のヌルベクトル（実測点が作る平面の法線）、ランク 1 の主方向
 * （最大ノルム行の正規化）は、いずれも uwb_math（uwb_sym3_solve_sphere /
 * uwb_sym3_null_vector / uwb_sym3_principal_axis）に委譲する。
 *
 * 以前ここにあった「u = M⁻¹b を正規化する」近似は厳密解でなく、高さに
 * 1cm のノイズが乗ると傾きが平均 0.14°・最大 0.55°（5m 先で ≈ 5cm）
 * ずれていた（A-6）。uwb_sym3_solve_sphere の線形部分は余因子ではなく
 * LDLᵀ（float で後退安定）。「hard case」（b が λ_min の固有ベクトルと
 * 直交し、ν → −λ_min でも |u| が 1 に届かない）で反復上限、または二分が
 * 極に達して LDLᵀ が正定値でなくなった時点に達したときは、直前に解けた
 * u を正規化して返す（0 は返さない。旧実装はこの場合に 0 を返して
 * 呼び出し側の u = ẑ 初期値に落ちていた）。
 *
 * uwb_sym3_null_vector は行を max|s| で正規化してから外積を取る点だけ
 * 旧実装（生の行）と違うが、選択規則・符号規約は同じ。
 * uwb_sym3_principal_axis は符号を「絶対値最大の成分が正」に正準化する
 * 点だけ違うが、fit_up() の使い方（coef = e0·b/λ0、w の e0 依存項が
 * すべて e0 の 2 次式）はどちらも符号不変なので u は変わらない。 */

/* 実測高さ h_k に最もよく合う「上」方向 u（単位ベクトル）と z オフセット c を
 * 求める。求めたい関係は
 *
 *     h_k ≈ u . p_k + c      (|u| = 1)
 *
 * c を消去（c = h_bar - u . p_bar）すると、中心化して
 *
 *     minimize  sum_k ( u . q_k - g_k )^2 ,  q_k = p_k - p_bar, g_k = h_k - h_bar
 *             = u^T M u - 2 u^T b + const,   M = sum q q^T, b = sum q g
 *
 * これを |u|=1 の下で解く。M のランク（閉形式の固有値から判定）で分岐する:
 *
 *   rank 3（高さ 4 点以上で同一平面でない）
 *     uwb_sym3_solve_sphere() の厳密解。データが剛体変換で厳密に説明できるなら
 *     無拘束解 M⁻¹b がそのまま |u|=1 を満たし 1 反復で終わる。
 *
 *   rank 2（**高さの実測が 3 点だけだと M は必ずランク 2**。中心化した
 *           3 点は sum q = 0 なので 2 次元しか張らない。4 点以上でも
 *           高さ実測点が同一平面なら同じ）
 *     ヌルベクトル n̂（= 実測点が作る平面の法線。行の外積で取る）を
 *     M' = M + λ0 n̂n̂ᵀ と足して正則化し、u_par = M'⁻¹b を解く。b は M の
 *     値域にある（b = Σ q g）ので u_par ⊥ n̂、つまりデータが決めるのは
 *     値域成分だけ。|u_par| < 1 なら
 *         u = u_par + sqrt(1 − |u_par|²) · (±n̂)
 *     で、**符号だけが未定**として残る。これがまさに鏡像の自由度で、
 *     高さ残差はどちらでも同じになる（uwb_survey.h 冒頭の説明を参照）。
 *     ここでは決定論的に「現フレームの +z に近い側」を選び、最終的な
 *     キラリティは [5] のあとの規約で潰す。|u_par| ≥ 1（高さがノイズで
 *     矛盾）なら M' 上で uwb_sym3_solve_sphere（n̂ 成分は 0 のまま）。
 *
 *   rank 1（高さ 2 点、または実測点が一直線）
 *     q の方向 e0（M の最大ノルム行）だけが決まる: u_par = (e0·b/λ0) e0。
 *     残りは e0 に直交する面内で +z に最も近い向き。
 *
 *   rank 0（高さ 0〜1 点）
 *     u = (0,0,1)、c だけ合わせる（nh=0 なら c=0）。
 *
 * つまり「高さが 3 点未満なら Lv1 相当へフォールバック」を同じ式で実現する。
 * rank_out には M のランク（0..3）を返す。3 なら傾きが一意に決まっている。
 * 除算: 1（1/nh）+ eigvals 1 + 分岐ごとに 1〜3（+ uwb_sym3_solve_sphere の反復）。 */
static void fit_up(const uwb_real p[NMAX][3], const uwb_real *h, const int *hm, int n,
                   uwb_real *u, uwb_real *c, uwb_real *hrms, int *rank_out)
{
    uwb_real M[6], b[3], lam[3];
    uwb_real pbar[3], hbar = (uwb_real)0, inv_nh;
    uwb_real res = (uwb_real)0;
    int i, nh = 0, rank = 0;

    u[0] = u[1] = (uwb_real)0;
    u[2] = (uwb_real)1;
    *c        = (uwb_real)0;
    *hrms     = (uwb_real)0;
    *rank_out = 0;

    for (i = 0; i < n; ++i) if (hm[i]) ++nh;
    if (nh == 0) return;

    uwb_v3_zero(pbar);
    for (i = 0; i < n; ++i) {
        if (!hm[i]) continue;
        uwb_v3_add(pbar, p[i], pbar);
        hbar += h[i];
    }
    inv_nh = (uwb_real)1 / (uwb_real)nh;
    uwb_v3_scale(inv_nh, pbar);
    hbar *= inv_nh;

    uwb_sym3_zero(M);
    uwb_v3_zero(b);
    for (i = 0; i < n; ++i) {
        uwb_real q[3];
        if (!hm[i]) continue;
        uwb_v3_sub(p[i], pbar, q);
        uwb_sym3_add_outer(M, q);
        uwb_v3_axpy(h[i] - hbar, q, b);
    }

    if (uwb_sym3_eigvals(M, lam))
        rank = uwb_sym3_rank(lam, UWB_MATH_RANK_TOL, (uwb_real)0);

    if (rank == 3) {
        uwb_sym3_solve_sphere(M, b, lam[2], u, NULL);   /* 失敗時は u = ẑ のまま */
    } else if (rank == 2) {
        uwb_real nvec[3], Mr[6], upar[3], n2;
        int k;
        if (uwb_sym3_null_vector(M, nvec)) {
            for (k = 0; k < 6; ++k) Mr[k] = M[k];
            uwb_sym3_add_scaled_outer(Mr, lam[0], nvec);
            if (uwb_sym3_solve(Mr, b, upar)) {
                n2 = uwb_v3_norm2(upar);
                if (n2 < (uwb_real)1) {
                    uwb_real rem = uwb_math_sqrt((uwb_real)1 - n2);
                    if (nvec[2] < (uwb_real)0) rem = -rem;    /* +z に近い側 */
                    uwb_v3_copy(upar, u);
                    uwb_v3_axpy(rem, nvec, u);
                } else {
                    uwb_sym3_solve_sphere(Mr, b, lam[1], u, NULL);   /* λ_min(M') = λ1 (λ0 ≥ λ1) */
                }
            }
        }
    } else if (rank == 1) {
        /* M ≈ λ0 e0 e0ᵀ: uwb_sym3_principal_axis が最大ノルム行を正規化して返す */
        uwb_real e0[3], w[3], coef, n2;
        if (uwb_sym3_principal_axis(M, e0)) {
            coef = uwb_v3_dot(e0, b) / lam[0];
            n2   = coef * coef;
            if (n2 < (uwb_real)1) {
                /* ヌル空間（e0 に直交する面）で +z に最も近い向き: ẑ の射影 */
                uwb_real e1[3], e2[3];
                w[0] = -e0[2] * e0[0];
                w[1] = -e0[2] * e0[1];
                w[2] = (uwb_real)1 - e0[2] * e0[2];
                if (!(uwb_v3_normalize(w) > (uwb_real)1e-3)) {
                    /* e0 ∥ ẑ。向きは決めようがないので直交基底の 1 本目 */
                    if (uwb_v3_perp_basis(e0, e1, e2)) uwb_v3_copy(e1, w);
                    else { w[0] = (uwb_real)1; w[1] = w[2] = (uwb_real)0; }
                    if (w[2] < (uwb_real)0) uwb_v3_scale((uwb_real)-1, w);
                }
                uwb_v3_copy(e0, u);
                uwb_v3_scale(coef, u);
                uwb_v3_axpy(uwb_math_sqrt((uwb_real)1 - n2), w, u);
            } else {
                uwb_v3_copy(e0, u);
                if (coef < (uwb_real)0) uwb_v3_scale((uwb_real)-1, u);
            }
        }
    }
    /* rank 0: u = (0,0,1) のまま */

    *c        = hbar - uwb_v3_dot(u, pbar);
    *rank_out = rank;

    for (i = 0; i < n; ++i) {
        uwb_real e;
        if (!hm[i]) continue;
        e = uwb_v3_dot(u, p[i]) + *c - h[i];
        res += e * e;
    }
    *hrms = uwb_math_sqrt(res * inv_nh);
}

/* u を +z に持ってくる回転 R（det=+1）と z 並進 c を全ノードに掛ける。
 *
 * R の第3行を u にすればよい。残り 2 行は u に直交する正規直交基底で、
 * uwb_v3_perp_basis が「u と最も揃っていない座標軸」から Gram-Schmidt で
 * 作る（u とほぼ平行な軸を選ぶと桁落ちするため）。e1 × e2 = u なので
 * 右手系（det=+1、鏡像を勝手に導入しない）。
 *
 * このあと残る自由度は z 軸まわりの回転 1 と XY 並進 2 で、それは [5] が潰す。
 * sqrt 1、除算 1。 */
static void apply_frame(uwb_real p[NMAX][3], int n, const uwb_real *u, uwb_real c)
{
    uwb_real e1[3], e2[3];
    int i;

    if (!uwb_v3_perp_basis(u, e1, e2)) {
        /* u が単位でない（起きないはず）。回さず z 並進だけ */
        e1[0] = (uwb_real)1; e1[1] = (uwb_real)0; e1[2] = (uwb_real)0;
        e2[0] = (uwb_real)0; e2[1] = (uwb_real)1; e2[2] = (uwb_real)0;
    }

    for (i = 0; i < n; ++i) {
        uwb_real q[3];
        uwb_v3_copy(p[i], q);
        p[i][0] = uwb_v3_dot(e1, q);
        p[i][1] = uwb_v3_dot(e2, q);
        p[i][2] = uwb_v3_dot(u, q) + c;
    }
}

/* =====================================================================
 * [5] XY 規約とキラリティ
 * ===================================================================== */

/* ノード0 を XY 原点へ、ノード1 を +X 軸上へ。z は高さ実測で決まって
 * いるので触らない。ノード1 がノード0 の真上にある（XY で重なる）と
 * ヨーが定義できないので、その場合は回さない。sqrt 1、除算 1。 */
static void apply_xy_convention(uwb_real p[NMAX][3], int n)
{
    uwb_real ox = p[0][0], oy = p[0][1], rr, inv, cs, sn;
    int i;

    for (i = 0; i < n; ++i) { p[i][0] -= ox; p[i][1] -= oy; }

    rr = uwb_math_sqrt(p[1][0] * p[1][0] + p[1][1] * p[1][1]);
    if (rr < UWB_MATH_TINY) return;
    inv = (uwb_real)1 / rr;
    cs  = p[1][0] * inv;
    sn  = p[1][1] * inv;
    for (i = 0; i < n; ++i) {
        uwb_real x = p[i][0], y = p[i][1];
        p[i][0] =  cs * x + sn * y;
        p[i][1] = -sn * x + cs * y;
    }
    p[1][0] = rr;     /* 丸め誤差の掃除 */
    p[1][1] = (uwb_real)0;
}

/* 鏡像の一意化。y を反転しても
 *   - すべての距離
 *   - すべての z（＝高さ残差）
 *   - ノード0 の XY 原点、ノード1 の +X 上
 * が保たれるので、[4][5] の結果を壊さずにキラリティだけ選び直せる。
 *
 * chirality != 0 のとき（レビュー A-4）: 呼び出し側の事前知識「上から見て
 * ノード 0→1→2 が反時計回り (+1) / 時計回り (−1)」に合わせる。判定量は
 * (p1−p0)×(p2−p0) の z 成分で、[5] の正規化後は p0 = 0、p1 = (x1, 0) なので
 * z = x1·y2、符号は y2 の符号そのもの。margin にはこれを |p1−p0| で割った
 * |y2|（= 上から見た 0–1 直線からノード 2 までの距離 [m]）を返す。測距
 * ばらつきと同程度以下なら 0,1,2 がほぼ一直線で、判定は信用できない。
 * 判定できた（y2 ≠ 0）ら 1 を返す。
 *
 * chirality == 0 のとき: 「|y| が最大のノード（同点なら添字が小さい方）の
 * y を正にする」規約。margin は |y| の 1 位と 2 位の差 [m]（これが測距
 * ばらつき以下 — ノード 0,1 を長方形の対角に置くと 2,3 が y=±w/2 で
 * 厳密に同点になる — だと試行ごとに裏返る）。常に 0 を返す（規約で潰した
 * だけで、左右が決まったわけではない）。 */
static int fix_chirality(uwb_real p[NMAX][3], int n, int chirality, uwb_real *margin)
{
    int i, big = -1;
    uwb_real bv = (uwb_real)0, sv = (uwb_real)0;   /* |y| の 1 位と 2 位 */

    *margin = (uwb_real)0;

    if (chirality != 0) {
        uwb_real y2 = p[2][1];
        *margin = uwb_math_abs(y2);
        if (!(*margin > (uwb_real)0) || !(p[1][0] > (uwb_real)0)) return 0;  /* 一直線 / ヨー未定義 */
        if ((chirality > 0) != (y2 > (uwb_real)0))
            for (i = 0; i < n; ++i) p[i][1] = -p[i][1];
        return 1;
    }

    for (i = 2; i < n; ++i) {
        uwb_real v = uwb_math_abs(p[i][1]);
        if (v > bv)      { sv = bv; bv = v; big = i; }
        else if (v > sv) { sv = v; }
    }
    if (big < 0 || bv < UWB_MATH_TINY) return 0;    /* XY で一直線。決めようがない */
    *margin = bv - sv;
    if (p[big][1] >= (uwb_real)0) return 0;
    for (i = 0; i < n; ++i) p[i][1] = -p[i][1];
    return 0;
}

/* =====================================================================
 * 本体
 * ===================================================================== */

int uwb_survey_solve(const uwb_survey_input *in, uwb_survey_result *out)
{
    survey_ctx c;
    uwb_real   d[NMAX][NMAX];
    uwb_real   f[LMAX];
    uwb_real   cost = (uwb_real)0, planar_rms;
    uwb_real   u[3], gz, hrms_a, hrms_b, margin;
    uwb_real   ua[3], ub[3], ca, cb;
    uwb_real   mirror[NMAX][3];
    int        i, t, nused, drops, degen = 0, iters = 0, sub = 0, rank_a, rank_b;

    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!in) return 0;
    if (in->n < UWB_SURVEY_MIN_NODES || in->n > UWB_SURVEY_MAX_NODES) return 0;

    for (i = 0; i < in->n; ++i) if (in->have_height[i]) ++out->n_heights;

    /* [1] 入力の取り込みと可解性 */
    if (!build_links(in, &c)) return 0;
    if (!complete_distances(&c, d)) return 0;

    /* [2] 逐次三辺測量による初期配置 */
    if (!trilat_init(&c, UWB_SURVEY_CPN(d), 3, &degen)) return 0;
    if (degen) {
        /* 縮退した配置は 3 次元の形が距離から決まらない。診断のため初期配置の
         * 生座標だけ返して失敗にする（仕様書 §5「配置を変えるよう促す」）。 */
        out->degenerate = 1;
        for (i = 0; i < c.n; ++i) uwb_v3_copy(c.p[i], out->pos[i]);
        return 0;
    }

    /* [3] LM + 外れ値リンクの棄却（leave-one-out。落としたら解き直す） */
    for (drops = 0; drops <= UWB_SURVEY_MAX_DROP; ++drops) {
        int it = 0;
        if (!lm_run(&c, f, &it, &cost, UWB_SURVEY_MAX_ITER, UWB_SURVEY_TOL)) return 0;
        iters += it;
        if (!escape_local_minima(&c, f, &iters, &cost, UWB_SURVEY_MAX_ITER, UWB_SURVEY_TOL)) return 0;
        if (drops == UWB_SURVEY_MAX_DROP) break;
        if (!loo_drop_link(&c, f, cost, &out->excluded, &out->outlier_ambiguous, &iters)) break;
    }

    nused = 0;
    for (i = 0; i < c.nlink; ++i) if (c.use[i]) ++nused;
    out->residual_rms   = (nused > 0) ? uwb_math_sqrt(cost / (uwb_real)nused) : (uwb_real)0;
    out->iterations     = iters;
    out->common_delay_m = c.est_delay ? c.delay : (uwb_real)0;
    out->redundancy     = nused - (3 * c.n - 6 + (c.est_delay ? 1 : 0));
    out->delay_suspect  = (c.est_delay && uwb_math_abs(c.delay) > UWB_SURVEY_DELAY_SUSPECT) ? 1 : 0;

    /* [3'] 収束した形状（δ 除去済み）で縮退判定をやり直す（A-1）。 */
    if (shape_degenerate(&c, UWB_SURVEY_CPN(d), f, out->residual_rms, out->redundancy, &planar_rms)) {
        out->degenerate = 1;
        for (i = 0; i < c.n; ++i) uwb_v3_copy(c.p[i], out->pos[i]);
        return 0;
    }

    /* [4] 高さによるゲージ固定。仕様どおり反転あり/なしの両方で高さ残差を
     * 出して比べる — ただし **理論上は必ず引き分けになる**
     * （uwb_survey.h 冒頭の証明）。有意差が出たときだけ mirror_resolved を
     * 立て、引き分けなら規約（fix_chirality）で一意化する。 */
    for (i = 0; i < c.n; ++i) {
        mirror[i][0] =  c.p[i][0];
        mirror[i][1] =  c.p[i][1];
        mirror[i][2] = -c.p[i][2];
    }
    fit_up(UWB_SURVEY_CP3(c.p),    in->height, in->have_height, c.n, ua, &ca, &hrms_a, &rank_a);
    fit_up(UWB_SURVEY_CP3(mirror), in->height, in->have_height, c.n, ub, &cb, &hrms_b, &rank_b);

    /* 「有意差」のしきい値。理論上この2つは必ず一致するので、差はすべて
     * 丸め誤差である。したがって絶対項は **計算機イプシロン基準**にする
     * （定数を決め打ちすると単精度ビルドで丸め誤差を有意差と誤認する）。 */
    {
        uwb_real hscale = (uwb_real)1;
        for (i = 0; i < c.n; ++i)
            if (in->have_height[i] && uwb_math_abs(in->height[i]) > hscale)
                hscale = uwb_math_abs(in->height[i]);
        margin = (uwb_real)1e-3 * ((hrms_a > hrms_b) ? hrms_a : hrms_b) +
                 hscale * (uwb_real)1e3 * UWB_MATH_EPS;
    }
    if (hrms_b < hrms_a - margin) {
        sub = 1;
        out->mirror_resolved = 1;
    } else if (hrms_a < hrms_b - margin) {
        out->mirror_resolved = 1;
    } else {
        out->mirror_resolved = 0;
    }

    if (sub) {
        for (i = 0; i < c.n; ++i) uwb_v3_copy(mirror[i], c.p[i]);
        uwb_v3_copy(ub, u);
        gz = cb;
        out->frame_determined = (rank_b == 3) ? 1 : 0;
    } else {
        uwb_v3_copy(ua, u);
        gz = ca;
        out->frame_determined = (rank_a == 3) ? 1 : 0;
    }

    apply_frame(c.p, c.n, u, gz);

    /* [5] XY 規約 → 鏡像の一意化（chirality 入力があればそれで、無ければ規約で） */
    apply_xy_convention(c.p, c.n);
    if (fix_chirality(c.p, c.n, in->chirality, &out->chirality_margin))
        out->mirror_resolved = 1;

    for (i = 0; i < c.n; ++i)
        for (t = 0; t < 3; ++t) {
            if (uwb_math_isnan(c.p[i][t])) return 0;
            out->pos[i][t] = c.p[i][t];
        }

    out->ok = 1;
    return 1;
}
