/* 閉形式ソルバ — 反復も初期値も要らない解。
 * Python 版 uwb_loc/solvers/closed_form.py に対応する。 */
#include "uwb_internal.h"
#include "uwb_linalg.h"

#include <math.h>

/* 2 次元で解くときは、アンカーの高さの差を測距値から抜いて水平距離にする。 */
static void collect(const uwb_config *cfg, const uwb_meas *meas, int n,
                    uwb_real *pos, uwb_real *rng, uwb_real *w, int *out_n, int *d)
{
    int i, k = 0;
    *d = cfg->dim;
    for (i = 0; i < n && k < UWB_MAX_MEAS; ++i) {
        uwb_real r, s;
        uwb_meas mm;
        if (!uwb_meas_usable(cfg, &meas[i])) continue;
        r = uwb_corrected(cfg, &meas[i]);
        if (!(r > (uwb_real)0)) continue;
        mm = meas[i];
        s = uwb_sigma_of(cfg, &mm, r);

        if (*d == 2) {
            uwb_real dz = cfg->z_fixed - cfg->anchors[meas[i].anchor].p[2];
            uwb_real h2 = r * r - dz * dz;
            r = (uwb_real)sqrt((double)(h2 > (uwb_real)1e-4 ? h2 : (uwb_real)1e-4));
            pos[k * 2 + 0] = cfg->anchors[meas[i].anchor].p[0];
            pos[k * 2 + 1] = cfg->anchors[meas[i].anchor].p[1];
        } else {
            pos[k * 3 + 0] = cfg->anchors[meas[i].anchor].p[0];
            pos[k * 3 + 1] = cfg->anchors[meas[i].anchor].p[1];
            pos[k * 3 + 2] = cfg->anchors[meas[i].anchor].p[2];
        }
        rng[k] = r;
        w[k] = (uwb_real)1 / (s * s);
        ++k;
    }
    *out_n = k;
}

int uwb_lls_trilateration(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real amat[UWB_MAX_MEAS * 3], b[UWB_MAX_MEAS];
    uwb_real ata[9], atb[3], sol[3];
    int m = 0, d = 3, i, k, ref = 0, rows = 0;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;

    /* 基準は測距値が最小のもの。ふつう一番 S/N が良い。 */
    for (i = 1; i < m; ++i) if (rng[i] < rng[ref]) ref = i;

    {
        const uwb_real *aref = &pos[ref * d];
        uwb_real rref = rng[ref];
        uwb_real aa_ref = (uwb_real)0;
        for (k = 0; k < d; ++k) aa_ref += aref[k] * aref[k];

        for (i = 0; i < m; ++i) {
            const uwb_real *ai;
            uwb_real aa = (uwb_real)0, sw;
            if (i == ref) continue;
            ai = &pos[i * d];
            for (k = 0; k < d; ++k) aa += ai[k] * ai[k];
            /* ||p||^2 が差分で消えるので線形になる */
            for (k = 0; k < d; ++k) amat[rows * d + k] = (uwb_real)2 * (aref[k] - ai[k]);
            b[rows] = rng[i] * rng[i] - rref * rref - aa + aa_ref;
            /* 差分をとった行の重みは 2 本の合成。調和平均で近似する。 */
            sw = (uwb_real)sqrt((double)(w[i] * w[ref] / (w[i] + w[ref])));
            for (k = 0; k < d; ++k) amat[rows * d + k] *= sw;
            b[rows] *= sw;
            ++rows;
        }
    }
    if (rows < d) return 0;

    uwb_ata_weighted(amat, 0, rows, d, ata);
    uwb_atb_weighted(amat, 0, b, rows, d, atb);
    /* アンカーが同一平面 (または一直線) だと A^T A が階数落ちする。
     * numpy の lstsq はそのとき最小ノルム解を返すので、こちらも
     * ごく小さいリッジを入れて同じ振る舞いに寄せる。健全な配置では
     * 影響しない大きさ (対角平均の 1e-12 倍)。 */
    {
        uwb_real tr = (uwb_real)0;
        int kk;
        for (kk = 0; kk < d; ++kk) tr += ata[kk * d + kk];
        if (tr > (uwb_real)0) {
            uwb_real ridge = (tr / (uwb_real)d) * (uwb_real)1e-12;
            for (kk = 0; kk < d; ++kk) ata[kk * d + kk] += ridge;
        }
    }
    if (!uwb_solve_lin(ata, atb, sol, d)) return 0;

    for (k = 0; k < d; ++k) {
        if (sol[k] != sol[k]) return 0;
        out[k] = sol[k];
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}

/* ------------------------------------------------------------- Beck GTRS */

static uwb_real uwb_abs(uwb_real v) { return v < 0 ? -v : v; }

/* Beck 法の GTRS (Generalized Trust Region Subproblem) を、
 * (G + lam D) の構造を使って完全スカラー演算の二分法で解く。
 *
 * 導出 (前処理は uwb_beck_gtrs 側で1回だけ行う):
 *   G + lam D = L (I + lam S) L^T,   S = L^-1 D L^-T = U diag(mu) U^T
 *   c := U^T L^-1 h,  g := U^T L^-1 f   (f は index d だけ -0.5 の疎ベクトル)
 *   z_i(lam) = (c_i - lam g_i) / (1 + lam mu_i)
 *   phi(lam) = y^T D y + 2 f^T y = sum_i mu_i z_i^2 + 2 sum_i g_i z_i   (y を作らない)
 *   y(lam)   = L^-T U z(lam)                                          (収束後に1回だけ)
 *
 * 旧実装は毎回 (G + lam D) y = h - lam f を 4x4 LU で解いていたが、
 * G, D は lam に依存しないのでコレスキー・固有分解を使い回せる。
 * 典型 ~50 回呼ばれる二分法の内側が O(nq^2)=O(16) の LU から O(nq)=O(4) の
 * スカラー評価になる。 */
typedef struct {
    int d, nq;
    uwb_real mu[4];      /* S の固有値 (昇順)。mu[nq-1] = gamma (最大) */
    uwb_real cvec[4];    /* U^T L^-1 h */
    uwb_real gvec[4];    /* U^T L^-1 f */
    uwb_real tvec[4];    /* t_i = g_i + c_i*mu_i (dz_i/dlam の分子。lam に依存しない定数) */
    uwb_real u[16];      /* S の固有ベクトル (列 i が mu[i] に対応、mu と同じ昇順) */
    uwb_real inv_l[16];  /* L^-1 (下三角)。収束後の y = L^-T U z の復元に使う */
} beck_ctx;

/* phi(lam) = y^T D y + 2 f^T y をスカラー演算だけで評価する (y を作らない)。
 * 1 + lam*mu_i が 0 近傍のとき (= lam が極に当たったとき) は、
 * 旧実装で (G + lam D) が特異になって uwb_solve_lin が 0 を返していたのと
 * 同じ状況なので、同様に 0 を返す (UWB_EPS はこの関数内の他の特異判定と
 * 同じしきい値を使う)。 */
static int beck_phi(const beck_ctx *c, uwb_real lam, uwb_real *phi)
{
    uwb_real v = (uwb_real)0;
    int i;
    for (i = 0; i < c->nq; ++i) {
        uwb_real den = (uwb_real)1 + lam * c->mu[i];
        uwb_real z;
        if (uwb_abs(den) < UWB_EPS) return 0;
        z = (c->cvec[i] - lam * c->gvec[i]) / den;
        v += c->mu[i] * z * z + (uwb_real)2 * c->gvec[i] * z;
    }
    if (v != v) return 0;
    *phi = v;
    return 1;
}

/* phi(lam) と phi'(lam) を同時にスカラー演算だけで評価する (d_i = 1+lam*mu_i の
 * 再計算を避けるため、phi 単独版とは別関数にしてある)。
 *
 *   z_i(lam)  = (c_i - lam*g_i) / d_i
 *   z_i'(lam) = -t_i / d_i^2,           t_i := g_i + c_i*mu_i (lam に依存しない)
 *   phi'(lam) = 2 * sum_i z_i' * (mu_i*z_i + g_i)
 *
 * phi は 1 + lam*mu_i = 0 (= lam が S の固有値の逆数の符号反転、つまり極) で
 * 発散するので、根の近くでも phi' が非常に大きく/小さくなったり、極をまたぐと
 * 符号が反転したりする。そのため phi/phi' から得るニュートン候補をそのまま
 * 採用してはいけない (区間外に飛ぶ・振動する危険がある)。呼び出し側
 * (uwb_beck_gtrs) で必ず区間 (lo,hi) との整合性と収束速度を確認し、
 * 怪しいときは二分ステップに落とす「安全策付きニュートン法 (rtsafe)」にする。
 *
 * *scale には phi を構成する各項の絶対値の和 (最適化 E) を返す。
 * phi = sum_i (mu_i*z_i^2 + 2*g_i*z_i) は項同士の相殺で桁落ちするので、
 * 「これ以上詰めても意味が無い」丸め誤差の床は |phi| 自体ではなく
 * この scale に比例する (呼び出し側で UWB_REAL_EPS と比べて判定する)。 */
static int beck_phi_dphi(const beck_ctx *c, uwb_real lam, uwb_real *phi, uwb_real *dphi,
                         uwb_real *scale)
{
    uwb_real v = (uwb_real)0, dv = (uwb_real)0, sc = (uwb_real)0;
    int i;
    for (i = 0; i < c->nq; ++i) {
        uwb_real den = (uwb_real)1 + lam * c->mu[i];
        uwb_real inv, z, zp, t1, t2;
        if (uwb_abs(den) < UWB_EPS) return 0;
        inv = (uwb_real)1 / den;
        z = (c->cvec[i] - lam * c->gvec[i]) * inv;
        zp = -c->tvec[i] * inv * inv;
        t1 = c->mu[i] * z * z;
        t2 = (uwb_real)2 * c->gvec[i] * z;
        v += t1 + t2;
        sc += uwb_abs(t1) + uwb_abs(t2);
        dv += zp * (c->mu[i] * z + c->gvec[i]);
    }
    dv *= (uwb_real)2;
    if (v != v || dv != dv) return 0;
    *phi = v;
    *dphi = dv;
    *scale = sc;
    return 1;
}

/* 二分法収束後に1回だけ、z(lam) から y = L^-T U z(lam) を復元する。 */
static int beck_recover_y(const beck_ctx *c, uwb_real lam, uwb_real *out_y)
{
    uwb_real z[4], tmp[4];
    int i, r, col;
    for (i = 0; i < c->nq; ++i) {
        uwb_real den = (uwb_real)1 + lam * c->mu[i];
        if (uwb_abs(den) < UWB_EPS) return 0;
        z[i] = (c->cvec[i] - lam * c->gvec[i]) / den;
    }
    for (r = 0; r < c->nq; ++r) {                 /* tmp = U z */
        uwb_real acc = (uwb_real)0;
        for (i = 0; i < c->nq; ++i) acc += c->u[r * c->nq + i] * z[i];
        tmp[r] = acc;
    }
    for (r = 0; r < c->nq; ++r) {                 /* y = L^-T tmp (後退代入) */
        uwb_real acc = (uwb_real)0;
        for (col = r; col < c->nq; ++col) acc += c->inv_l[col * c->nq + r] * tmp[col];
        out_y[r] = acc;
    }
    return 1;
}

int uwb_beck_gtrs(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real amat[UWB_MAX_MEAS * 4], b[UWB_MAX_MEAS];
    uwb_real g[16], h[4];
    beck_ctx c;
    uwb_real lam_lo, lo, hi, span, phi, x, y[4];
    int m = 0, d = 3, i, k, it;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;
    c.d = d;
    c.nq = d + 1;

    /* A y = b,  y = [p; ||p||^2] */
    for (i = 0; i < m; ++i) {
        for (k = 0; k < d; ++k) amat[i * c.nq + k] = (uwb_real)-2 * pos[i * d + k];
        amat[i * c.nq + d] = (uwb_real)1;
        {
            uwb_real aa = (uwb_real)0;
            for (k = 0; k < d; ++k) aa += pos[i * d + k] * pos[i * d + k];
            b[i] = rng[i] * rng[i] - aa;
        }
    }
    uwb_ata_weighted(amat, w, m, c.nq, g);
    uwb_atb_weighted(amat, w, b, m, c.nq, h);

    /* lam の下限は一般化固有値 (D, G) の最大値の逆数の符号反転。
     * G = L L^T として S = L^-1 D L^-T は対称なので、その固有値・固有ベクトルを
     * Jacobi 法で取れる (G^-1 D と同じ固有値)。固有ベクトルは phi(lam) を
     * スカラー演算だけで評価するための前処理 (c_vec, g_vec) に使う。 */
    {
        uwb_real l[16], s[16];
        int r, col;
        /* アンカーが同一平面に乗っていると G はちょうど特異になる
         * ([x y z 1] の列に線形関係が立つため)。ここで解けないのは
         * 「その配置では Beck は使えない」という正しい合図なので、
         * 素直に失敗を返して呼び出し側 (LLS) に任せる。
         * Python 版の beck_gtrs も同じ条件で None を返す。 */
        for (i = 0; i < c.nq * c.nq; ++i) l[i] = g[i];
        if (!uwb_cholesky(l, c.nq)) return 0;

        /* L^-1 を前進代入で作る (下三角) */
        for (i = 0; i < c.nq * c.nq; ++i) c.inv_l[i] = (uwb_real)0;
        for (col = 0; col < c.nq; ++col) {
            c.inv_l[col * c.nq + col] = (uwb_real)1 / l[col * c.nq + col];
            for (r = col + 1; r < c.nq; ++r) {
                uwb_real acc = (uwb_real)0;
                for (k = col; k < r; ++k) acc += l[r * c.nq + k] * c.inv_l[k * c.nq + col];
                c.inv_l[r * c.nq + col] = -acc / l[r * c.nq + r];
            }
        }
        /* S = L^-1 D L^-T。D = diag(1..1, 0) なので最後の列を落とすだけ。 */
        for (r = 0; r < c.nq; ++r) {
            for (col = 0; col < c.nq; ++col) {
                uwb_real acc = (uwb_real)0;
                for (k = 0; k < d; ++k)          /* D の 1 が並ぶ範囲だけ */
                    acc += c.inv_l[r * c.nq + k] * c.inv_l[col * c.nq + k];
                s[r * c.nq + col] = acc;
            }
        }
        if (!uwb_sym_eig(s, c.mu, c.u, c.nq)) return 0;
        if (!(c.mu[c.nq - 1] > UWB_EPS)) return 0;
        lam_lo = (uwb_real)-1 / c.mu[c.nq - 1];

        /* c := U^T L^-1 h (定数ベクトル、1回だけ計算) */
        {
            uwb_real linv_h[4];
            for (r = 0; r < c.nq; ++r) {
                uwb_real acc = (uwb_real)0;
                for (col = 0; col <= r; ++col) acc += c.inv_l[r * c.nq + col] * h[col];
                linv_h[r] = acc;
            }
            for (i = 0; i < c.nq; ++i) {
                uwb_real acc = (uwb_real)0;
                for (r = 0; r < c.nq; ++r) acc += c.u[r * c.nq + i] * linv_h[r];
                c.cvec[i] = acc;
            }
        }
        /* g := U^T L^-1 f。f = (0,...,0,-0.5)^T (index d だけ非ゼロ) なので
         * L^-1 f は下三角の性質から最後の成分しか残らない
         * (L^-1 f)[d] = -0.5 / l[d][d]、他はゼロ。O(nq) で済む。 */
        {
            uwb_real linv_f_d = (uwb_real)-0.5 * c.inv_l[d * c.nq + d];
            for (i = 0; i < c.nq; ++i) c.gvec[i] = c.u[d * c.nq + i] * linv_f_d;
        }
        /* t_i = g_i + c_i*mu_i。dz_i/dlam = -t_i/(1+lam*mu_i)^2 の分子で、
         * lam に依存しないので前処理で1回だけ作っておく (beck_phi_dphi 側)。 */
        for (i = 0; i < c.nq; ++i) c.tvec[i] = c.gvec[i] + c.cvec[i] * c.mu[i];
    }

    /* 開区間なので下限のわずかに内側から始める。phi(lam_lo+) は +∞。 */
    span = lam_lo < 0 ? -lam_lo : lam_lo;
    if (span < (uwb_real)1) span = (uwb_real)1;
    lo = lam_lo + (uwb_real)1e-9 * span;
    for (it = 0; it < 60; ++it) {
        if (beck_phi(&c, lo, &phi) && phi > (uwb_real)0) break;
        lo = lam_lo + (lo - lam_lo) * (uwb_real)10;
    }
    if (it == 60) return 0;

    hi = lo + span;
    for (it = 0; it < 200; ++it) {
        if (beck_phi(&c, hi, &phi) && phi < (uwb_real)0) break;
        hi = lo + (hi - lo) * (uwb_real)2;
    }
    if (it == 200) return 0;

    /* 安全策付きニュートン法 (rtsafe)。phi は単調減少で lo 側が正・hi 側が負。
     * phi が閉形式の初等関数になった (最適化 C) ので導関数 phi' も解析的に
     * 書けるが、phi は 1+lam*mu_i=0 に極を持つため、根の付近でも純粋な
     * ニュートン法は候補が区間外へ飛んだり振動したりする危険がある。
     * そのため「ニュートン候補が区間 (lo,hi) の内側に留まり、かつ前回の
     * 更新幅の半分以下に縮む」ときだけ採用し、それ以外は二分ステップに
     * 落とすことで、二分法と同じ安全性を保ったまま収束を速める。 */
    {
        uwb_real prev_step = hi - lo;  /* 直前の更新幅。最初は括り出し区間全体 */
        uwb_real dphi, scale;

        x = (uwb_real)0.5 * (lo + hi);
        for (it = 0; it < 200; ++it) {
            uwb_real xn, step, scale_lam;
            int use_bisect;

            if (!beck_phi_dphi(&c, x, &phi, &dphi, &scale)) return 0;

            /* phi の符号で区間を更新 (phi は単調減少) */
            if (phi > (uwb_real)0) lo = x; else hi = x;

            /* 最適化 E-1: 丸め誤差の床。phi は項の相殺で桁落ちするので、
             * |phi| が評価誤差のスケール (scale, 各項の絶対値の和) に対して
             * machine epsilon 程度まで落ちたら、それ以上区間を詰めても
             * 数値的な意味がない (ノイズを追いかけるだけ)。安全率 8 は
             * nq<=4 項の丸め誤差の積み上がりに余裕を持たせるため。 */
            if (uwb_abs(phi) <= (uwb_real)8 * UWB_REAL_EPS * scale) break;

            /* 最適化 E-2: 区間幅判定を真の相対許容にする。旧判定
             * `hi > 1 ? hi : 1` は lam が 1 以下・負のとき常に 1 になり、
             * 実質「絶対許容 1e-14」という過剰に厳しい判定だった
             * (N=6 のように lam が負に収束する配置で二分が数十回余分に
             * 回っていた)。lo/hi のどちらが基準でも良いよう両方見る。 */
            scale_lam = uwb_abs(lo);
            if (uwb_abs(hi) > scale_lam) scale_lam = uwb_abs(hi);
            if (scale_lam < (uwb_real)1) scale_lam = (uwb_real)1;
            if (hi - lo < (uwb_real)1e-14 * scale_lam) break;

            /* ニュートン候補。区間外に出る／更新量が前回の半分以下に
             * 縮まない／|phi'| がゼロ近傍 (極の直近) なら二分に落とす。 */
            use_bisect = uwb_abs(dphi) < UWB_EPS;
            if (!use_bisect) {
                step = phi / dphi;
                xn = x - step;
                if (xn <= lo || xn >= hi ||
                    uwb_abs(step) > (uwb_real)0.5 * prev_step)
                    use_bisect = 1;
            }
            if (use_bisect) xn = (uwb_real)0.5 * (lo + hi);

            prev_step = uwb_abs(xn - x);
            x = xn;
        }
    }

    /* y の復元には最後に評価した x を使う (0.5*(lo+hi) ではない)。
     * rtsafe は毎回 phi(x) の符号で lo か hi の「片方だけ」を x に
     * 寄せるので、収束の途中では lo と hi の間にまだ大きな差が残る
     * ことがある (もう片方は括り出し時の粗い値のまま)。最適化 E で
     * 収束判定を早めた結果、この差が残ったまま抜けるケースが実際に
     * 起こる (安全策のニュートンで x 自体は根に十分近いのに、
     * 0.5*(lo+hi) は遠い方の境界に引っ張られて不正確になる)。
     * x は直前に phi/dphi を評価した点そのものなので、素直にこれを
     * 最終推定値として使う。 */
    if (!beck_recover_y(&c, x, y)) return 0;
    for (k = 0; k < d; ++k) {
        if (y[k] != y[k]) return 0;
        out[k] = y[k];
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}
