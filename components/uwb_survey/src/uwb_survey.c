/* uwb_survey — アンカー自動測量の計算本体（docs/SURVEY_SPEC.md §2 [2]〜[5]）。
 *
 * 数式の導出はそれぞれの節に書いてある。全体の流れは
 *
 *   build_links()        入力を「リンクの列」に直し、可解性を検査する
 *   complete_distances() 欠測リンクを最短経路で埋める（MDS の初期値づくり用）
 *   mds_init()           二重中心化 → 固有分解 → 上位3軸で初期配置
 *   lm_run()             Levenberg-Marquardt で距離残差を最小化
 *   escape_local_minima() ノードを隣接平面で鏡映して局所解から抜ける
 *   drop_worst_link()    外れ値リンクを1本ずつ落として解き直す
 *   fit_up()             実測高さに合う「上」方向 u と z オフセット c を求める
 *   apply_frame()        u を +z に持ってくる回転と z 並進を掛ける
 *   apply_xy_convention() ノード0 を XY 原点、ノード1 を +X 軸上へ
 *   fix_chirality()      鏡像を規約で一意化する
 */
#include "uwb_survey.h"

#include "uwb_survey_dense.h"

#include <float.h>
#include <math.h>
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
 *  流儀だが、測量は数値誤差レベル（1e-6 m）の復元を要求されるので厳しくする。 */
#ifndef UWB_SURVEY_TOL
#define UWB_SURVEY_TOL ((uwb_real)1e-10)
#endif

/** LM 減衰係数の初期値・下限・上限。 */
#ifndef UWB_SURVEY_LM_INIT
#define UWB_SURVEY_LM_INIT ((uwb_real)1e-3)
#endif
#define UWB_SURVEY_LM_MIN  ((uwb_real)1e-12)
#define UWB_SURVEY_LM_MAX  ((uwb_real)1e12)

/** 正規方程式の対角に必ず足す絶対リッジ。孤立した未知数（対角が 0）でも
 *  コレスキーが落ちないようにするための保険。 */
#define UWB_SURVEY_RIDGE_ABS ((uwb_real)1e-12)

/** 縮退判定: 第3固有値が最大固有値のこの割合を下回ったら同一平面とみなす。 */
#ifndef UWB_SURVEY_DEGEN_RATIO
#define UWB_SURVEY_DEGEN_RATIO ((uwb_real)1e-3)
#endif

/** 縮退判定: 第3主軸方向の RMS 広がり [m] がこれ未満なら縮退。
 *  UWB の測距ばらつき（数 cm）より小さい厚みは観測できない、という物理。 */
#ifndef UWB_SURVEY_DEGEN_EXTENT
#define UWB_SURVEY_DEGEN_EXTENT ((uwb_real)0.05)
#endif

/** 外れ値リンクの判定: |残差| > max(K*1.4826*MAD, FLOOR) で棄却。
 *  FLOOR は「測距の当たり外れ（数 cm）では絶対に落とさない」ための下限。 */
#ifndef UWB_SURVEY_OUTLIER_K
#define UWB_SURVEY_OUTLIER_K ((uwb_real)3.0)
#endif
#ifndef UWB_SURVEY_OUTLIER_FLOOR
#define UWB_SURVEY_OUTLIER_FLOOR ((uwb_real)0.30)
#endif

/** 落とすリンクの最大本数。 */
#define UWB_SURVEY_MAX_DROP 2

/** 最短経路補完で使う「無限大」。加算しても溢れない大きさにする。 */
#define UWB_SURVEY_INF ((uwb_real)1e18)

/** uwb_real の計算機イプシロン。 */
#if UWB_REAL_IS_FLOAT
#define UWB_SURVEY_EPS ((uwb_real)FLT_EPSILON)
#else
#define UWB_SURVEY_EPS ((uwb_real)DBL_EPSILON)
#endif

/** 固有値を「0 とみなす」相対しきい値（ランク判定用）。
 *  Jacobi 法の丸め誤差は最大固有値の eps 倍のオーダなので、その 1e4 倍を
 *  境にする。**必ず eps に比例させること** — 定数 1e-9 を決め打ちすると
 *  単精度ビルド（eps=1.2e-7）でランク落ちを検出できず、ほぼ 0 の固有値で
 *  割って解が壊れる。 */
#define UWB_SURVEY_RANK_TOL ((uwb_real)1e4 * UWB_SURVEY_EPS)

/* -------------------------------------------------------------- 小道具 */

static uwb_real sabs(uwb_real v) { return v < (uwb_real)0 ? -v : v; }
static uwb_real ssqrt(uwb_real v) { return (uwb_real)sqrt((double)v); }

static uwb_real dot3(const uwb_real *a, const uwb_real *b)
{
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

static uwb_real norm3(const uwb_real *a) { return ssqrt(dot3(a, a)); }

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

/* dist は対称のはずだが、両方向とも埋まっていれば平均を採る（往復2回の
 * 測距をそのまま入れられるようにするため）。片方だけならそれを使う。
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
            int hij = (in->have[i][j] != 0);
            int hji = (in->have[j][i] != 0);
            if (!hij && !hji) continue;
            if (hij && hji)      v = (uwb_real)0.5 * (in->dist[i][j] + in->dist[j][i]);
            else if (hij)        v = in->dist[i][j];
            else                 v = in->dist[j][i];
            if (!(v > (uwb_real)0)) continue;      /* 非正・NaN は欠測扱い */

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

    for (i = 0; i < c->n; ++i) if (deg[i] < 3) return 0;

    shape = 3 * c->n - 6;
    if (k < shape) return 0;
    c->est_delay = (k >= shape + 1) ? 1 : 0;
    return 1;
}

/* =====================================================================
 * [2] 欠測リンクの補完と古典的 MDS
 * ===================================================================== */

/* 欠測リンクの距離を **グラフ上の最短経路**で埋める（Floyd-Warshall）。
 *
 * なぜ最短経路か:
 *   - MDS は距離行列が全部埋まっていないと使えない。埋め方は
 *     「平均値で埋める」「反復付き重み付き MDS(SMACOF)」など色々あるが、
 *     最短経路（測地距離）は **三角不等式を必ず満たす**ので、二重中心化
 *     した Gram 行列が極端に非正定値になりにくい。ISOMAP で使われている
 *     のと同じ理屈
 *   - 直線見通しなら r_ij ≈ 実距離なので、2 ホップの和は真値の上界に
 *     なるだけで済む（少し伸びた初期値になるが、続く LM が直す）
 *   - n<=8 なので O(n^3)=512 回。malloc も反復も要らない
 *
 * 非連結（どこかへ辿り着けない）なら 0 を返す。 */
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

/* 古典的 MDS（Torgerson scaling）。
 *
 *   D2[i][j] = d_ij^2 を二重中心化して Gram 行列にする:
 *       B = -0.5 * J D2 J,   J = I - (1/n) 1 1^T
 *   これは「重心を原点に置いたときの内積行列」 B[i][j] = x_i . x_j に
 *   一致する（余弦定理 d_ij^2 = |x_i|^2 + |x_j|^2 - 2 x_i.x_j を
 *   中心化で解いたもの）。成分で書くと
 *       B[i][j] = -0.5 * (D2[i][j] - rowmean_i - rowmean_j + grandmean)
 *
 *   B = V L V^T と固有分解すれば X = V_3 L_3^{1/2} が求める座標。
 *   固有値は「各主軸方向の二乗広がりの総和」= sum_i (x_ik - mean)^2 なので、
 *   sqrt(lam_k / n) がその軸方向の RMS 広がり [m] になる。縮退判定に
 *   この物理量を使う。
 *
 * 遅延 delta は初期値 0 として D = r をそのまま使う（仕様書 §2[2] のとおり）。
 * 全リンクが 2*delta ぶん一様に伸びるだけなので、初期配置は少し膨らむが
 * 形は保たれ、LM が縮めてくれる。 */
static int mds_init(survey_ctx *c, const uwb_real d[NMAX][NMAX], int *degenerate)
{
    int i, j, k, n = c->n;
    uwb_real B[NMAX * NMAX], V[NMAX * NMAX], eig[NMAX];
    uwb_real row[NMAX], grand = (uwb_real)0;

    for (i = 0; i < n; ++i) {
        uwb_real s = (uwb_real)0;
        for (j = 0; j < n; ++j) s += d[i][j] * d[i][j];
        row[i] = s / (uwb_real)n;
        grand += s;
    }
    grand /= (uwb_real)n * (uwb_real)n;

    for (i = 0; i < n; ++i)
        for (j = 0; j < n; ++j)
            B[i * n + j] = (uwb_real)-0.5 * (d[i][j] * d[i][j] - row[i] - row[j] + grand);

    if (!uwb_survey_sym_eig(B, eig, V, n)) return 0;
    if (!(eig[0] > (uwb_real)0)) return 0;    /* 全点が同一点。距離がおかしい */

    /* 第3固有値が小さい = 3 次元目の広がりが無い = 同一平面（さらに小さければ直線）。
     * 相対条件（スケール非依存）と物理条件（測距ばらつきより薄いか）の両方で見る。 */
    *degenerate = (eig[2] <= (uwb_real)0) ||
                  (eig[2] < UWB_SURVEY_DEGEN_RATIO * eig[0]) ||
                  (ssqrt(eig[2] / (uwb_real)n) < UWB_SURVEY_DEGEN_EXTENT);

    for (i = 0; i < n; ++i)
        for (k = 0; k < 3; ++k) {
            uwb_real lam = (eig[k] > (uwb_real)0) ? eig[k] : (uwb_real)0;
            c->p[i][k] = ssqrt(lam) * V[i * n + k];
        }
    return 1;
}

/* =====================================================================
 * [3] Gauss-Newton (Levenberg-Marquardt)
 * ===================================================================== */

/* 残差 f_k = |p_i - p_j| + 2*delta - r_k を作り、二乗和を返す。 */
/* 【C の型規則】C には T(*)[N] から const T(*)[N] への暗黙変換が無い
 * （C++ には有る）。非 const の配列を const 引数へ渡すと、GCC の
 * -Wpedantic が "invalid use of pointers to arrays with different
 * qualifiers in ISO C before C2X" を出す。本リポジトリの `make strict` は
 * -Werror なのでビルドが落ちる（clang は許容するので macOS では気づけない。
 * GitHub Actions の GCC ビルドで発覚した）。
 * C2x で解消される規則だが、それまでは明示キャストで通す。 */
#define UWB_SURVEY_CP3(p)  ((const uwb_real (*)[3])(p))
#define UWB_SURVEY_CPN(d)  ((const uwb_real (*)[NMAX])(d))

static uwb_real cost_at(const survey_ctx *c, const uwb_real p[NMAX][3],
                        uwb_real delay, uwb_real *f)
{
    int k;
    uwb_real s = (uwb_real)0;
    for (k = 0; k < c->nlink; ++k) {
        uwb_real dv[3], dd;
        int i, j, t;
        f[k] = (uwb_real)0;
        if (!c->use[k]) continue;
        i = c->li[k];
        j = c->lj[k];
        for (t = 0; t < 3; ++t) dv[t] = p[i][t] - p[j][t];
        dd   = norm3(dv);
        f[k] = dd + (uwb_real)2 * delay - c->r[k];
        s += f[k] * f[k];
    }
    return s;
}

/* 正規方程式 A = J^T J（Marquardt 減衰込み）と g = J^T f を作る。
 *
 * ヤコビアンの 1 行は 7 個しか非零成分を持たない:
 *     df/dp_i = u,  df/dp_j = -u,  df/ddelta = 2,   u = (p_i-p_j)/|p_i-p_j|
 * ので、J そのものを持たずに外積を足し込む（28x25 の J を置かずに済み、
 * 演算量も O(m*49) で終わる）。
 *
 * 減衰は A_ii <- A_ii*(1+lambda) + eps の Marquardt スケーリング。
 * diag(J^T J) は連結グラフなら全て正なので、lambda>0 なら A は厳密に
 * 正定値になり、コレスキーが必ず通る。 */
static void build_normal(const survey_ctx *c, const uwb_real p[NMAX][3],
                         const uwb_real *f, uwb_real lambda,
                         uwb_real *A, uwb_real *g, int nx)
{
    int k, a, b, t;

    for (a = 0; a < nx * nx; ++a) A[a] = (uwb_real)0;
    for (a = 0; a < nx; ++a) g[a] = (uwb_real)0;

    for (k = 0; k < c->nlink; ++k) {
        int      idx[7];
        uwb_real jv[7];
        int      nz = 0, i, j;
        uwb_real dv[3], dd, u[3];

        if (!c->use[k]) continue;
        i = c->li[k];
        j = c->lj[k];
        for (t = 0; t < 3; ++t) dv[t] = p[i][t] - p[j][t];
        dd = norm3(dv);
        if (dd < (uwb_real)1e-12) {
            /* 2 点が重なった。方向が定義できないので任意の単位ベクトルを
             * 使う（次の反復で離れる）。 */
            u[0] = (uwb_real)1; u[1] = (uwb_real)0; u[2] = (uwb_real)0;
        } else {
            for (t = 0; t < 3; ++t) u[t] = dv[t] / dd;
        }

        for (t = 0; t < 3; ++t) { idx[nz] = 3 * i + t; jv[nz] =  u[t]; ++nz; }
        for (t = 0; t < 3; ++t) { idx[nz] = 3 * j + t; jv[nz] = -u[t]; ++nz; }
        if (c->est_delay)       { idx[nz] = 3 * c->n;  jv[nz] = (uwb_real)2; ++nz; }

        for (a = 0; a < nz; ++a) {
            g[idx[a]] += jv[a] * f[k];
            for (b = 0; b < nz; ++b)
                A[idx[a] * nx + idx[b]] += jv[a] * jv[b];
        }
    }

    for (a = 0; a < nx; ++a)
        A[a * nx + a] = A[a * nx + a] * ((uwb_real)1 + lambda) + UWB_SURVEY_RIDGE_ABS;
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
 *      最小ノルム更新はその方向へ動かないので、解は MDS のフレームの
 *      近くに留まる。どのみち [4][5] で剛体変換し直すので、フレームが
 *      どこにあるかは結果に影響しない
 *   4. LM のステップ制御は、欠測リンクを最短経路で埋めた「少し歪んだ」
 *      初期値から始めるときの発散よけにもなる（純 Gauss-Newton は
 *      ここで飛ぶことがある）
 *
 * 減衰は成功で 0.3 倍、失敗で 10 倍という定石。成功するたび lambda が
 * 下がるので、収束付近では実質ただの Gauss-Newton になり二次収束する。 */
static int lm_run(survey_ctx *c, uwb_real *f, int *iters, uwb_real *cost_out)
{
    uwb_real A[XMAX * XMAX], g[XMAX], dx[XMAX];
    uwb_real ptry[NMAX][3], ftry[LMAX];
    uwb_real lambda = UWB_SURVEY_LM_INIT;
    uwb_real cost;
    int      nx = 3 * c->n + (c->est_delay ? 1 : 0);
    int      it, a, i, t;

    *iters = 0;
    cost   = cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
    if (cost != cost) return 0;

    for (it = 0; it < UWB_SURVEY_MAX_ITER; ++it) {
        int      accepted = 0;
        uwb_real step = (uwb_real)0;

        while (lambda <= UWB_SURVEY_LM_MAX) {
            uwb_real ntry, dtry, s2 = (uwb_real)0;

            build_normal(c, UWB_SURVEY_CP3(c->p), f, lambda, A, g, nx);
            if (!uwb_survey_chol_solve(A, g, dx, nx)) { lambda *= (uwb_real)10; continue; }
            for (a = 0; a < nx; ++a) dx[a] = -dx[a];   /* A dx = -g */

            for (i = 0; i < c->n; ++i)
                for (t = 0; t < 3; ++t) ptry[i][t] = c->p[i][t] + dx[3 * i + t];
            dtry = c->delay + (c->est_delay ? dx[3 * c->n] : (uwb_real)0);

            /* 試行の残差は ftry に入れる。f は「現在の座標での残差」で
             * なければならない（次の build_normal が使う）。 */
            ntry = cost_at(c, UWB_SURVEY_CP3(ptry), dtry, ftry);
            if (ntry == ntry && ntry < cost) {
                for (a = 0; a < nx; ++a) s2 += dx[a] * dx[a];
                step = ssqrt(s2);
                for (i = 0; i < c->n; ++i)
                    for (t = 0; t < 3; ++t) c->p[i][t] = ptry[i][t];
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
        if (step < UWB_SURVEY_TOL) break;
    }

    /* f を最終座標のものに戻しておく（外れ値判定と残差 RMS が使う） */
    cost = cost_at(c, UWB_SURVEY_CP3(c->p), c->delay, f);
    if (cost != cost) return 0;
    for (i = 0; i < c->n; ++i)
        for (t = 0; t < 3; ++t)
            if (c->p[i][t] != c->p[i][t]) return 0;
    *cost_out = cost;
    return 1;
}

/* --------------------------------------------- 局所解からの脱出（鏡映） */

/* ノード i の隣接点が作る最良近似平面（重心 cen と法線 nrm）を求める。
 * 法線は共分散行列の最小固有値の固有ベクトル（＝主成分分析の第3軸）。
 * 隣接点が3つ未満なら平面が決まらないので 0 を返す。 */
static int neighbor_plane(const survey_ctx *c, int i, uwb_real *cen, uwb_real *nrm)
{
    uwb_real M[9], V[9], mu[3];
    int k, t, u, m = 0;

    for (t = 0; t < 3; ++t) cen[t] = (uwb_real)0;
    for (k = 0; k < c->nlink; ++k) {
        int o;
        if (!c->use[k]) continue;
        if (c->li[k] == i)      o = c->lj[k];
        else if (c->lj[k] == i) o = c->li[k];
        else continue;
        for (t = 0; t < 3; ++t) cen[t] += c->p[o][t];
        ++m;
    }
    if (m < 3) return 0;
    for (t = 0; t < 3; ++t) cen[t] /= (uwb_real)m;

    for (t = 0; t < 9; ++t) M[t] = (uwb_real)0;
    for (k = 0; k < c->nlink; ++k) {
        uwb_real q[3];
        int o;
        if (!c->use[k]) continue;
        if (c->li[k] == i)      o = c->lj[k];
        else if (c->lj[k] == i) o = c->li[k];
        else continue;
        for (t = 0; t < 3; ++t) q[t] = c->p[o][t] - cen[t];
        for (t = 0; t < 3; ++t)
            for (u = 0; u < 3; ++u) M[t * 3 + u] += q[t] * q[u];
    }
    if (!uwb_survey_sym_eig(M, mu, V, 3)) return 0;
    for (t = 0; t < 3; ++t) nrm[t] = V[t * 3 + 2];   /* 最小固有値の固有ベクトル */
    return 1;
}

/* 距離だけの最小二乗は **局所解を持つ**。とくに欠測リンクがあると、
 * 「あるノードだけが隣接点の作る平面の反対側に落ちた」形の偽の最小に
 * 捕まりやすい（分子構造決定でよく知られた flip ambiguity）。
 *
 * そこで LM が止まったあと、各ノードを隣接平面で鏡映した点から解き直し、
 * コストが下がったら採用する、というスイープを回す。ノード数が 8 以下なので
 * 全ノード×数スイープでも一瞬で終わる（測量は設置時に1回だけの処理）。
 *
 * 真の最小に着いていれば、どの鏡映もコストを下げられないので何も起きない。 */
static int escape_local_minima(survey_ctx *c, uwb_real *f, int *iters, uwb_real *cost)
{
    uwb_real bp[NMAX][3], bf[LMAX], bd;
    int      sweep, i, k, t, it;

    for (sweep = 0; sweep < 3; ++sweep) {
        int improved = 0;
        for (i = 0; i < c->n; ++i) {
            uwb_real cen[3], nrm[3], dn, trial;

            /* 現在の最良解を退避 */
            for (k = 0; k < c->n; ++k)
                for (t = 0; t < 3; ++t) bp[k][t] = c->p[k][t];
            for (k = 0; k < c->nlink; ++k) bf[k] = f[k];
            bd = c->delay;

            if (!neighbor_plane(c, i, cen, nrm)) continue;
            dn = (c->p[i][0] - cen[0]) * nrm[0] + (c->p[i][1] - cen[1]) * nrm[1] +
                 (c->p[i][2] - cen[2]) * nrm[2];
            for (t = 0; t < 3; ++t) c->p[i][t] -= (uwb_real)2 * dn * nrm[t];

            it    = 0;
            trial = *cost;
            if (lm_run(c, f, &it, &trial) &&
                trial < *cost - (uwb_real)1e-12 * ((uwb_real)1 + *cost)) {
                *cost = trial;
                *iters += it;
                improved = 1;
                continue;                     /* 採用（退避は捨てる） */
            }
            /* 戻す */
            for (k = 0; k < c->n; ++k)
                for (t = 0; t < 3; ++t) c->p[k][t] = bp[k][t];
            for (k = 0; k < c->nlink; ++k) f[k] = bf[k];
            c->delay = bd;
        }
        if (!improved) break;
    }
    return 1;
}

/* --------------------------------------------------------- 外れ値リンク */

/* 残差の MAD からロバストなばらつきを作り、突出した 1 本を落とす。
 *
 * 冗長度（式の本数 - 未知数）を 1 以上残す・どのノードも次数 3 以上を
 * 保つ、という 2 条件を満たせるときだけ落とす。UWB の測距ばらつき
 * （数 cm）では絶対に落とさないよう、しきい値に 30cm の下限を置く。
 * 落としたら 1、落とさなかったら 0。 */
static int drop_worst_link(survey_ctx *c, const uwb_real *f, unsigned long *excluded)
{
    /* ar は下のループで m 個だけ埋めてから ar[0..m-1] しか読まないが、
     * GCC はそれを証明できず -Werror=maybe-uninitialized で落ちる
     * (ESP-IDF のビルド設定。ホストの make strict では出ない)。
     * 設置時に1回だけ走る関数なので、素直にゼロ初期化する。 */
    uwb_real ar[LMAX] = {(uwb_real)0}, med, thr, worst = (uwb_real)-1;
    int      deg[NMAX];
    int      k, i, j, m = 0, wk = -1, need;

    for (i = 0; i < c->n; ++i) deg[i] = 0;
    for (k = 0; k < c->nlink; ++k) {
        if (!c->use[k]) continue;
        ar[m++] = sabs(f[k]);
        ++deg[c->li[k]];
        ++deg[c->lj[k]];
    }

    need = 3 * c->n - 6 + (c->est_delay ? 1 : 0);
    if (m - 1 < need + 1) return 0;      /* 落とすと冗長度が無くなる */

    /* 中央値（挿入ソート。m <= 28） */
    for (i = 1; i < m; ++i) {
        uwb_real v = ar[i];
        for (j = i - 1; j >= 0 && ar[j] > v; --j) ar[j + 1] = ar[j];
        ar[j + 1] = v;
    }
    med = (m & 1) ? ar[m / 2] : (uwb_real)0.5 * (ar[m / 2 - 1] + ar[m / 2]);

    thr = UWB_SURVEY_OUTLIER_K * (uwb_real)1.4826 * med;
    if (thr < UWB_SURVEY_OUTLIER_FLOOR) thr = UWB_SURVEY_OUTLIER_FLOOR;

    for (k = 0; k < c->nlink; ++k) {
        if (!c->use[k]) continue;
        if (deg[c->li[k]] <= 3 || deg[c->lj[k]] <= 3) continue;  /* 次数を割る */
        if (sabs(f[k]) > thr && sabs(f[k]) > worst) { worst = sabs(f[k]); wk = k; }
    }
    if (wk < 0) return 0;

    c->use[wk] = 0;
    if (c->lbit[wk] >= 0 && c->lbit[wk] < 32)
        *excluded |= (1UL << c->lbit[wk]);
    return 1;
}

/* =====================================================================
 * [4] 実測高さによるゲージ固定
 * ===================================================================== */

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
 * これを |u|=1 の下で解く。M が正則なら制約なしの解 u = M^{-1} b が
 * そのまま答えになる（データが厳密に剛体変換で説明できるなら |u|=1 が
 * 自動的に成り立つ）。
 *
 * ところが **高さの実測が 3 点だけだと M は必ずランク 2 になる**
 * （中心化した 3 点は sum q = 0 なので 2 次元しか張らない）。この場合
 *
 *     u = u_par + s * sqrt(1 - |u_par|^2) * n̂
 *
 * のように、データが決めるのは M の値域成分 u_par だけで、ヌル空間方向
 * n̂（= 3 点が作る平面の法線）の **符号 s だけが未定**として残る。
 * これがまさに鏡像の自由度で、高さ残差はどちらの s でも同じになる
 * （uwb_survey.h 冒頭の説明を参照）。ここでは決定論的に「現フレームの
 * +z に近い側」を選び、最終的なキラリティは [5] のあとの規約で潰す。
 *
 * 同じ式が nh=0,1,2 のフォールバックも自然に含む:
 *   nh=0 → M=0,b=0 → u=(0,0,1), c=0（何も固定できない）
 *   nh=1 → u=(0,0,1) のまま c だけ合わせる
 *   nh=2 → 2 点を結ぶ方向の傾き 1 自由度だけが決まる
 * つまり「高さが3点未満なら Lv1 相当へフォールバック」を分岐なしで実現する。 */
static void fit_up(const uwb_real p[NMAX][3], const uwb_real *h, const int *hm, int n,
                   uwb_real *u, uwb_real *c, uwb_real *hrms)
{
    uwb_real M[9], V[9], mu[3], b[3] = {0, 0, 0};
    uwb_real pbar[3] = {0, 0, 0}, hbar = (uwb_real)0;
    uwb_real upar[3] = {0, 0, 0}, nb[3][3], w[3];
    uwb_real tolm, n2, rem, res = (uwb_real)0;
    const uwb_real zhat[3] = {(uwb_real)0, (uwb_real)0, (uwb_real)1};
    int i, k, t, nh = 0, nnull = 0;

    u[0] = u[1] = (uwb_real)0;
    u[2] = (uwb_real)1;
    *c    = (uwb_real)0;
    *hrms = (uwb_real)0;

    for (i = 0; i < n; ++i) if (hm[i]) ++nh;
    if (nh == 0) return;

    for (i = 0; i < n; ++i) {
        if (!hm[i]) continue;
        for (t = 0; t < 3; ++t) pbar[t] += p[i][t];
        hbar += h[i];
    }
    for (t = 0; t < 3; ++t) pbar[t] /= (uwb_real)nh;
    hbar /= (uwb_real)nh;

    for (t = 0; t < 9; ++t) M[t] = (uwb_real)0;
    for (i = 0; i < n; ++i) {
        uwb_real q[3], gk;
        if (!hm[i]) continue;
        for (t = 0; t < 3; ++t) q[t] = p[i][t] - pbar[t];
        gk = h[i] - hbar;
        for (t = 0; t < 3; ++t) {
            b[t] += q[t] * gk;
            for (k = 0; k < 3; ++k) M[t * 3 + k] += q[t] * q[k];
        }
    }

    if (!uwb_survey_sym_eig(M, mu, V, 3)) return;

    tolm = mu[0] * UWB_SURVEY_RANK_TOL;
    if (!(tolm > (uwb_real)0)) tolm = (uwb_real)1e-30;

    for (k = 0; k < 3; ++k) {
        uwb_real e[3], coef;
        for (t = 0; t < 3; ++t) e[t] = V[t * 3 + k];
        if (mu[k] > tolm) {
            coef = dot3(e, b) / mu[k];
            for (t = 0; t < 3; ++t) upar[t] += coef * e[t];
        } else {
            for (t = 0; t < 3; ++t) nb[nnull][t] = e[t];
            ++nnull;
        }
    }

    n2 = dot3(upar, upar);
    if (nnull > 0 && n2 < (uwb_real)1) {
        uwb_real wn;
        rem = ssqrt((uwb_real)1 - n2);
        /* ヌル空間内では向きが決まらないので、決定論的に +z へ最も近い
         * 向きを選ぶ（射影して正規化）。射影が 0 なら基底の 1 本目を使う。 */
        for (t = 0; t < 3; ++t) w[t] = (uwb_real)0;
        for (k = 0; k < nnull; ++k) {
            uwb_real cf = dot3(nb[k], zhat);
            for (t = 0; t < 3; ++t) w[t] += cf * nb[k][t];
        }
        wn = norm3(w);
        if (wn < (uwb_real)1e-9) {
            for (t = 0; t < 3; ++t) w[t] = nb[0][t];
            if (w[2] < (uwb_real)0) for (t = 0; t < 3; ++t) w[t] = -w[t];
            wn = norm3(w);
        }
        if (wn > (uwb_real)0) for (t = 0; t < 3; ++t) w[t] /= wn;
        for (t = 0; t < 3; ++t) u[t] = upar[t] + rem * w[t];
    } else if (n2 > (uwb_real)1e-24) {
        uwb_real un = ssqrt(n2);
        for (t = 0; t < 3; ++t) u[t] = upar[t] / un;
    }
    /* どちらでもなければ u = (0,0,1) のまま */

    *c = hbar - dot3(u, pbar);

    for (i = 0; i < n; ++i) {
        uwb_real e;
        if (!hm[i]) continue;
        e = dot3(u, p[i]) + *c - h[i];
        res += e * e;
    }
    *hrms = ssqrt(res / (uwb_real)nh);
}

/* u を +z に持ってくる回転 R（det=+1）と z 並進 c を全ノードに掛ける。
 *
 * R の第3行を u にすればよい。残り 2 行は u に直交する任意の正規直交基底で、
 * 第1行 e1 は「u と最も揃っていない座標軸」から Gram-Schmidt で作る
 * （u とほぼ平行な軸を選ぶと桁落ちするため）。e2 = u × e1 とすれば
 * e1 × e2 = u となり右手系（det=+1、鏡像を勝手に導入しない）。
 *
 * このあと残る自由度は z 軸まわりの回転 1 と XY 並進 2 で、それは [5] が潰す。 */
static void apply_frame(uwb_real p[NMAX][3], int n, const uwb_real *u, uwb_real c)
{
    uwb_real e1[3], e2[3], ax[3] = {0, 0, 0};
    uwb_real d, len;
    int i, t, amin = 0;

    for (t = 1; t < 3; ++t) if (sabs(u[t]) < sabs(u[amin])) amin = t;
    ax[amin] = (uwb_real)1;

    d = dot3(ax, u);
    for (t = 0; t < 3; ++t) e1[t] = ax[t] - d * u[t];
    len = norm3(e1);
    if (len < (uwb_real)1e-12) { e1[0] = (uwb_real)1; e1[1] = e1[2] = (uwb_real)0; len = (uwb_real)1; }
    for (t = 0; t < 3; ++t) e1[t] /= len;

    e2[0] = u[1] * e1[2] - u[2] * e1[1];
    e2[1] = u[2] * e1[0] - u[0] * e1[2];
    e2[2] = u[0] * e1[1] - u[1] * e1[0];

    for (i = 0; i < n; ++i) {
        uwb_real q[3];
        for (t = 0; t < 3; ++t) q[t] = p[i][t];
        p[i][0] = dot3(e1, q);
        p[i][1] = dot3(e2, q);
        p[i][2] = dot3(u, q) + c;
    }
}

/* =====================================================================
 * [5] XY 規約とキラリティ
 * ===================================================================== */

/* ノード0 を XY 原点へ、ノード1 を +X 軸上へ。z は高さ実測で決まって
 * いるので触らない。ノード1 がノード0 の真上にある（XY で重なる）と
 * ヨーが定義できないので、その場合は回さない。 */
static void apply_xy_convention(uwb_real p[NMAX][3], int n)
{
    uwb_real ox = p[0][0], oy = p[0][1], rr, cs, sn;
    int i;

    for (i = 0; i < n; ++i) { p[i][0] -= ox; p[i][1] -= oy; }

    rr = ssqrt(p[1][0] * p[1][0] + p[1][1] * p[1][1]);
    if (rr < (uwb_real)1e-12) return;
    cs = p[1][0] / rr;
    sn = p[1][1] / rr;
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
 * 「|y| が最大のノード（同点なら添字が小さい方）の y を正にする」を規約と
 * する。最大値で決めるので、わずかなノイズで判定が裏返らない。 */
static void fix_chirality(uwb_real p[NMAX][3], int n)
{
    int i, big = -1;
    uwb_real bv = (uwb_real)0;

    for (i = 2; i < n; ++i) {
        uwb_real v = sabs(p[i][1]);
        if (v > bv) { bv = v; big = i; }
    }
    if (big < 0 || bv < (uwb_real)1e-12) return;    /* XY で一直線。決めようがない */
    if (p[big][1] >= (uwb_real)0) return;
    for (i = 0; i < n; ++i) p[i][1] = -p[i][1];
}

/* =====================================================================
 * 本体
 * ===================================================================== */

int uwb_survey_solve(const uwb_survey_input *in, uwb_survey_result *out)
{
    survey_ctx c;
    uwb_real   d[NMAX][NMAX];
    uwb_real   f[LMAX];
    uwb_real   cost = (uwb_real)0;
    uwb_real   u[3], gz, hrms_a, hrms_b, margin;
    uwb_real   ua[3], ub[3], ca, cb;
    uwb_real   mirror[NMAX][3];
    int        i, t, nused, drops, degen = 0, iters = 0, sub = 0;

    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    if (!in) return 0;
    if (in->n < UWB_SURVEY_MIN_NODES || in->n > UWB_SURVEY_MAX_NODES) return 0;

    /* [1] 入力の取り込みと可解性 */
    if (!build_links(in, &c)) return 0;
    if (!complete_distances(&c, d)) return 0;

    /* [2] 古典的 MDS */
    if (!mds_init(&c, UWB_SURVEY_CPN(d), &degen)) return 0;
    if (degen) {
        /* 縮退した配置は 3 次元の形が距離から決まらない。診断のため MDS の
         * 生座標だけ返して失敗にする（仕様書 §5「配置を変えるよう促す」）。 */
        out->degenerate = 1;
        for (i = 0; i < c.n; ++i)
            for (t = 0; t < 3; ++t) out->pos[i][t] = c.p[i][t];
        return 0;
    }

    /* [3] LM + 外れ値リンクの棄却 */
    for (drops = 0; drops <= UWB_SURVEY_MAX_DROP; ++drops) {
        int it = 0;
        if (!lm_run(&c, f, &it, &cost)) return 0;
        iters += it;
        if (!escape_local_minima(&c, f, &iters, &cost)) return 0;
        if (drops == UWB_SURVEY_MAX_DROP) break;
        if (!drop_worst_link(&c, f, &out->excluded)) break;
    }

    nused = 0;
    for (i = 0; i < c.nlink; ++i) if (c.use[i]) ++nused;
    out->residual_rms = (nused > 0) ? ssqrt(cost / (uwb_real)nused) : (uwb_real)0;
    out->iterations   = iters;
    out->common_delay_m = c.est_delay ? c.delay : (uwb_real)0;

    /* [4] 高さによるゲージ固定。仕様どおり反転あり/なしの両方で高さ残差を
     * 出して比べる — ただし **理論上は必ず引き分けになる**
     * （uwb_survey.h 冒頭の証明）。有意差が出たときだけ mirror_resolved を
     * 立て、引き分けなら規約（fix_chirality）で一意化する。 */
    for (i = 0; i < c.n; ++i) {
        mirror[i][0] =  c.p[i][0];
        mirror[i][1] =  c.p[i][1];
        mirror[i][2] = -c.p[i][2];
    }
    fit_up(UWB_SURVEY_CP3(c.p),    in->height, in->have_height, c.n, ua, &ca, &hrms_a);
    fit_up(UWB_SURVEY_CP3(mirror), in->height, in->have_height, c.n, ub, &cb, &hrms_b);

    /* 「有意差」のしきい値。理論上この2つは必ず一致するので、差はすべて
     * 丸め誤差である。したがって絶対項は **計算機イプシロン基準**にする
     * （定数を決め打ちすると単精度ビルドで丸め誤差を有意差と誤認する）。 */
    {
        uwb_real hscale = (uwb_real)1;
        for (i = 0; i < c.n; ++i)
            if (in->have_height[i] && sabs(in->height[i]) > hscale)
                hscale = sabs(in->height[i]);
        margin = (uwb_real)1e-3 * ((hrms_a > hrms_b) ? hrms_a : hrms_b) +
                 hscale * (uwb_real)1e3 * UWB_SURVEY_EPS;
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
        for (i = 0; i < c.n; ++i)
            for (t = 0; t < 3; ++t) c.p[i][t] = mirror[i][t];
        for (t = 0; t < 3; ++t) u[t] = ub[t];
        gz = cb;
    } else {
        for (t = 0; t < 3; ++t) u[t] = ua[t];
        gz = ca;
    }

    apply_frame(c.p, c.n, u, gz);

    /* [5] XY 規約 → 鏡像の一意化 */
    apply_xy_convention(c.p, c.n);
    fix_chirality(c.p, c.n);

    for (i = 0; i < c.n; ++i)
        for (t = 0; t < 3; ++t) {
            if (c.p[i][t] != c.p[i][t]) return 0;
            out->pos[i][t] = c.p[i][t];
        }

    out->ok = 1;
    return 1;
}
