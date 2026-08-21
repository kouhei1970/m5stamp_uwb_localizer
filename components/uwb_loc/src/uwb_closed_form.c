/* 閉形式ソルバ — 反復も初期値も要らない解。
 * Python 版 uwb_loc/solvers/closed_form.py に対応する。
 *
 * 行列は dim×dim (2x2 / 3x3) の対称パック形式で持ち、uwb_math の LDLᵀ
 * (uwb_sym3_ldl_*、除算 3 回) と閉形式固有値で処理する。一般 LU / コレスキー /
 * Jacobi は無い。 */
#include "uwb_internal.h"

/* ------------------------------------------------- 2x2 / 3x3 の薄いラッパ */
/* Beck と LLS は dim=2 でも動くので、d を見て sym2 / sym3 を呼び分ける。 */

static void sc_zero(int d, uwb_real *s)
{
    if (d == 3) uwb_sym3_zero(s); else uwb_sym2_zero(s);
}

static void sc_add_scaled_outer(int d, uwb_real *s, uwb_real w, const uwb_real *u)
{
    if (d == 3) uwb_sym3_add_scaled_outer(s, w, u);
    else        uwb_sym2_add_scaled_outer(s, w, u);
}

static uwb_real sc_trace(int d, const uwb_real *s)
{
    return d == 3 ? uwb_sym3_trace(s) : uwb_sym2_trace(s);
}

/* LDLᵀ で S x = b (正定値でなければ 0)。除算 3 (2x2 は 2)。 */
static int sc_solve_pd(int d, const uwb_real *s, const uwb_real *b, uwb_real *x)
{
    if (d == 3) {
        uwb_sym3_ldl f;
        if (!uwb_sym3_ldl_factor(s, (uwb_real)0, 1, &f)) return 0;
        uwb_sym3_ldl_solve(&f, b, x);
    } else {
        uwb_sym2_ldl f;
        if (!uwb_sym2_ldl_factor(s, (uwb_real)0, 1, &f)) return 0;
        uwb_sym2_ldl_solve(&f, b, x);
    }
    return x[0] == x[0] && x[1] == x[1] && (d == 2 || x[2] == x[2]);
}

/* 固有値 (降順)。 */
static int sc_eigvals(int d, const uwb_real *s, uwb_real *lam)
{
    return d == 3 ? uwb_sym3_eigvals(s, lam) : uwb_sym2_eigvals(s, lam);
}

static uwb_real dotd(int d, const uwb_real *a, const uwb_real *b)
{
    return d == 3 ? uwb_v3_dot(a, b) : a[0] * b[0] + a[1] * b[1];
}

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
            r = uwb_math_sqrt(h2 > (uwb_real)1e-4 ? h2 : (uwb_real)1e-4);
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

/* ----------------------------------------------------------------- LLS */

/* 正規方程式 S x = b を解く。S が階数落ち (アンカーが同一平面 / 一直線) なら
 * numpy の lstsq と同じ**最小ノルム解**にする: 零空間の方向 n (最小固有値の
 * 固有ベクトル) に μ n nᵀ (μ = 対角平均) を足してから解く。b は S の値域に
 * あるので nᵀb ≈ 0 となり、解は S⁺b + O(eps)·n。
 * 旧実装は常に対角平均の 1e-12 倍のリッジを足して一般 LU に任せていたが、
 * それは条件数 κ の分だけ解を偏らせる (κ·1e-12。4 台配置で 5e-7 m を実測)
 * うえ、float では 1e-12 が eps 以下で効かない。階数落ちは明示的に扱い、
 * 健全な配置ではリッジ無しで解く。
 * 零空間が 2 次元以上 (一直線) なら解は決まらないので 0。 */
static int lls_solve(int d, const uwb_real *s, const uwb_real *b, uwb_real *x)
{
    uwb_real lam[3], nv[3], t[6], mu;

    /* LDLᵀ (後退安定、sqrt 無し、除算 3)。余因子解は条件数 κ の大きい配置
     * (4 台で行が 3 本、ほぼ同一平面など) で誤差が eps·κ·σ_max/σ_2 まで
     * 膨らみ、float で κ ≈ 1e5 の配置では解が数 m 狂った (旧 LU は正しかった)。
     * Lv0 はこの解をそのまま返すので精度を優先する (uwb_internal.h 参照)。 */
    if (sc_solve_pd(d, s, b, x)) return 1;

    /* 正定値でない = 階数落ち (または丸めでそう見える) → 最小ノルム解 */
    mu = sc_trace(d, s) / (uwb_real)d;
    if (!(mu > (uwb_real)0)) return 0;
    if (!sc_eigvals(d, s, lam)) return 0;
    if (d == 3) {
        if (!uwb_sym3_min_eigvec(s, 0, nv)) return 0;  /* 一直線: 法線が決まらない */
        t[0] = s[0]; t[1] = s[1]; t[2] = s[2]; t[3] = s[3]; t[4] = s[4]; t[5] = s[5];
        uwb_sym3_add_scaled_outer(t, mu, nv);
    } else {
        if (!uwb_sym2_eigvec(s, lam[1], nv)) return 0;
        t[0] = s[0]; t[1] = s[1]; t[2] = s[2];
        uwb_sym2_add_scaled_outer(t, mu, nv);
    }
    return sc_solve_pd(d, t, b, x);
}

int uwb_lls_trilateration(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real ata[6], atb[3], sol[3];
    int m = 0, d = 3, i, k, ref = 0, rows = 0;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;

    /* 基準は測距値が最小のもの。ふつう一番 S/N が良い。 */
    for (i = 1; i < m; ++i) if (rng[i] < rng[ref]) ref = i;

    sc_zero(d, ata);
    atb[0] = atb[1] = atb[2] = (uwb_real)0;
    {
        const uwb_real *aref = &pos[ref * d];
        uwb_real rref = rng[ref];
        uwb_real aa_ref = dotd(d, aref, aref);

        for (i = 0; i < m; ++i) {
            const uwb_real *ai;
            uwb_real a[3], b, aa, sw2;
            if (i == ref) continue;
            ai = &pos[i * d];
            aa = dotd(d, ai, ai);
            /* ||p||^2 が差分で消えるので線形になる */
            for (k = 0; k < d; ++k) a[k] = (uwb_real)2 * (aref[k] - ai[k]);
            b = rng[i] * rng[i] - rref * rref - aa + aa_ref;
            /* 差分をとった行の重みは 2 本の合成。調和平均で近似する。
             * 行を sqrt(sw2) 倍する代わりに正規方程式へ重み sw2 で足し込む
             * (同じ AᵀWA。sqrt が要らない)。 */
            sw2 = w[i] * w[ref] / (w[i] + w[ref]);
            sc_add_scaled_outer(d, ata, sw2, a);
            for (k = 0; k < d; ++k) atb[k] += sw2 * b * a[k];
            ++rows;
        }
    }
    if (rows < d) return 0;

    if (!lls_solve(d, ata, atb, sol)) return 0;

    for (k = 0; k < d; ++k) {
        if (sol[k] != sol[k]) return 0;
        out[k] = sol[k];
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}

/* ------------------------------------------------------------- Beck GTRS */

/* Beck 法の GTRS (Generalized Trust Region Subproblem)。
 *
 * 4x4 の (G + λD) y = h − λf を解く代わりに、‖p‖² に相当する未知数 α を
 * Schur 補元で先に消し、さらに座標を重み付き重心 p̄ に移して解く:
 *
 *   d_i = p_i − p̄,   q = p − p̄,   b'_i = r_i² − ‖d_i‖²
 *   Sc = 4 Σ w_i d_i d_iᵀ  (= 4 Σw · 重み付き散布行列。3x3 対称)
 *   r0 = −2 Σ w_i b'_i d_i,   W = Σ w_i,   b̄ = Σ w_i b'_i / W
 *
 *   (Sc + λI) q(λ) = r0                         (右辺は λ によらない)
 *   α(λ) = b̄ + λ/(2W)
 *   φ(λ) = ‖q(λ)‖² − α(λ)                        (= yᵀDy + 2fᵀy)
 *   φ'(λ) = −2 qᵀ (Sc + λI)⁻¹ q − 1/(2W) < 0     (狭義単調減少)
 *
 * 重心座標では g = −2Σw p_i が 0 になるので Schur 補元の式が最も簡単になり、
 * かつ Sc を中心化した和で作れる (桁落ちしない)。目的関数
 * Σ w (‖p − p_i‖² − r_i²)² は平行移動不変なので、解 p = p̄ + q は元の座標で
 * 解いたものと同じ (λ の値だけが変わる)。
 *
 * λ の定義域は Sc + λI ≻ 0、すなわち λ > λ_lo = −σ_min(Sc)。φ は λ_lo⁺ で
 * +∞、λ→∞ で −α → −∞ なので根がただ 1 つある。σ_min は閉形式の固有値で、
 * 固有ベクトルは使わない (正方形 / 立方体配置で縮退して不良条件になる)。
 *
 * 各反復は 3x3 (2 次元なら 2x2) の LDLᵀ 分解 1 回 (除算 3、sqrt 0) と前進
 * 後退代入 2 回 (q と φ' 用の (Sc+λI)⁻¹q)。余因子逆行列 (除算 1) も試したが、
 * 余因子展開は後退安定でなく、float では κ ≈ 200 の配置で解が 2e-4 m 狂い
 * (旧 4x4 の固有分解版は 3e-7 m)、σ_min/σ_max ≈ 1e-4 の配置では行列式が
 * 丸め誤差に埋もれて根の近傍が「解けない領域」になった。LDLᵀ なら
 * 誤差は eps·κ に収まる (同じ配置で 4e-6 m)。極の判定は σ_min + λ が固有値の
 * 精度 (4 eps σ_max) 以下か、LDLᵀ のピボットが非正か。
 *
 * float での堅さ: 旧実装は lo = λ_lo + 1e-9·|λ_lo| から探索を始めたが、
 * 1e-9 は float の eps (1.2e-7) より小さく lo == λ_lo になって極に当たり、
 * 一歩も動けずに失敗していた (FMA 縮約で偶然 1+λμ が非ゼロになるときだけ
 * 通っていた)。ここでは刻みを 64·eps·σ_max と eps に比例させ、極 (行列式が
 * 丸め誤差に埋もれる点) を踏んだら「φ = +∞ 側」とみなして区間を詰める。 */

typedef struct {
    int      d;          /* 2 か 3 */
    uwb_real sc[6];      /* Sc (パック) */
    uwb_real sig[3];     /* Sc の固有値 (降順) */
    uwb_real r0[3];
    uwb_real bbar;
    uwb_real inv_2w;     /* 1/(2W) */
} beck_ctx;

/* φ(λ), φ'(λ), 丸め誤差の床 scale (= 各項の絶対値の和) と q(λ)。
 * 極を踏んだ (σ_min + λ が固有値の精度以下、または LDLᵀ のピボットが非正)
 * なら 0 を返す。除算 3 (LDLᵀ の対角)。 */
static int beck_eval(const beck_ctx *c, uwb_real lam, uwb_real *phi, uwb_real *dphi,
                     uwb_real *scale, uwb_real *q)
{
    uwb_real t[3], n2, alpha, v, dv;
    uwb_real smin_l = c->sig[c->d - 1] + lam;

    if (!(smin_l > (uwb_real)4 * UWB_MATH_EPS * c->sig[0])) return 0;
    if (c->d == 3) {
        uwb_sym3_ldl f;
        if (!uwb_sym3_ldl_factor(c->sc, lam, 1, &f)) return 0;
        uwb_sym3_ldl_solve(&f, c->r0, q);        /* q = (Sc + λI)⁻¹ r0 */
        uwb_sym3_ldl_solve(&f, q, t);            /* t = (Sc + λI)⁻¹ q */
    } else {
        uwb_sym2_ldl f;
        if (!uwb_sym2_ldl_factor(c->sc, lam, 1, &f)) return 0;
        uwb_sym2_ldl_solve(&f, c->r0, q);
        uwb_sym2_ldl_solve(&f, q, t);
    }
    n2 = dotd(c->d, q, q);
    alpha = c->bbar + lam * c->inv_2w;
    v = n2 - alpha;
    dv = (uwb_real)-2 * dotd(c->d, q, t) - c->inv_2w;
    if (uwb_math_isnan(v) || uwb_math_isnan(dv)) return 0;
    *phi = v;
    *dphi = dv;
    *scale = n2 + uwb_math_abs(alpha);
    return 1;
}

int uwb_beck_gtrs(const uwb_config *cfg, const uwb_meas *meas, int n, uwb_real *out)
{
    uwb_real pos[UWB_MAX_MEAS * 3], rng[UWB_MAX_MEAS], w[UWB_MAX_MEAS];
    uwb_real pbar[3], q[3];
    uwb_real wsum, inv_w, smax, smin, lam_lo, delta, lo, hi, span;
    uwb_real phi, dphi, scale, x;
    beck_ctx c;
    int m = 0, d = 3, i, k, it;

    collect(cfg, meas, n, pos, rng, w, &m, &d);
    if (m < d + 1) return 0;
    c.d = d;

    /* 重み付き重心 */
    wsum = (uwb_real)0;
    pbar[0] = pbar[1] = pbar[2] = (uwb_real)0;
    for (i = 0; i < m; ++i) {
        wsum += w[i];
        for (k = 0; k < d; ++k) pbar[k] += w[i] * pos[i * d + k];
    }
    if (!(wsum > (uwb_real)0)) return 0;
    inv_w = (uwb_real)1 / wsum;
    for (k = 0; k < d; ++k) pbar[k] *= inv_w;
    c.inv_2w = (uwb_real)0.5 * inv_w;

    /* 中心化した正規方程式 */
    sc_zero(d, c.sc);
    c.r0[0] = c.r0[1] = c.r0[2] = (uwb_real)0;
    c.bbar = (uwb_real)0;
    for (i = 0; i < m; ++i) {
        uwb_real dd[3], bp, wb;
        for (k = 0; k < d; ++k) dd[k] = pos[i * d + k] - pbar[k];
        bp = rng[i] * rng[i] - dotd(d, dd, dd);
        wb = w[i] * bp;
        sc_add_scaled_outer(d, c.sc, (uwb_real)4 * w[i], dd);
        for (k = 0; k < d; ++k) c.r0[k] -= (uwb_real)2 * wb * dd[k];
        c.bbar += wb;
    }
    c.bbar *= inv_w;

    /* λ の下限 = −σ_min(Sc)。σ_min が σ_max の 64 eps 以下なら Sc は階数落ち
     * (アンカーが同一平面 / 一直線) で、Beck はこの配置では使えない。
     * 旧実装で G のコレスキーが失敗していたのと同じ合図なので、素直に失敗を
     * 返して呼び出し側 (LLS) に任せる。Python 版の beck_gtrs も None を返す。 */
    if (!sc_eigvals(d, c.sc, c.sig)) return 0;
    smax = c.sig[0];
    smin = c.sig[d - 1];
    if (!(smax > (uwb_real)0) || !(smin > UWB_MATH_SING_TOL * smax)) return 0;
    lam_lo = -smin;

    /* 開区間なので下限のわずかに内側から始める。φ(λ_lo⁺) は +∞。
     * 刻みは eps に比例 (float でも λ_lo + δ ≠ λ_lo)。固有値の丸めで
     * σ_min + δ が極の判定に掛かることがあるので、そのときは刻みを 10 倍。 */
    delta = (uwb_real)64 * UWB_MATH_EPS * smax;
    for (it = 0; it < 60; ++it) {
        lo = lam_lo + delta;
        if (beck_eval(&c, lo, &phi, &dphi, &scale, q)) {
            if (phi > (uwb_real)0) break;
            /* 正定値なのに φ ≤ 0: φ は単調減少なので右に根は無い (hard case:
             * r0 が最小固有ベクトルに直交し、解が一意に決まらない)。 */
            return 0;
        }
        delta *= (uwb_real)10;
    }
    if (it == 60) return 0;

    /* 上限: φ < 0 になるまで幅を倍々に。λ のスケールは Sc の最大固有値。 */
    span = smax;
    hi = lo + span;
    for (it = 0; it < 200; ++it) {
        if (beck_eval(&c, hi, &phi, &dphi, &scale, q) && phi < (uwb_real)0) break;
        hi = lo + (hi - lo) * (uwb_real)2;
    }
    if (it == 200) return 0;

    /* 安全策付きニュートン法 (rtsafe)。φ は単調減少で lo 側が正・hi 側が負。
     * ニュートン候補が区間 (lo,hi) の内側に留まり、かつ前回の更新幅の半分
     * 以下に縮むときだけ採用し、それ以外は二分ステップに落とす。 */
    {
        uwb_real prev_step = hi - lo;  /* 直前の更新幅。最初は括り出し区間全体 */
        int have_q = 0;                /* q が現在の x で評価済みか */

        x = (uwb_real)0.5 * (lo + hi);
        for (it = 0; it < 200; ++it) {
            uwb_real xn, step, scale_lam;
            int use_bisect;

            if (!beck_eval(&c, x, &phi, &dphi, &scale, q)) {
                /* 極を踏んだ (σ_min + x が固有値の精度以下) = 根より左。
                 * φ = +∞ 側として扱い、二分で右へ寄せる。 */
                lo = x;
                xn = (uwb_real)0.5 * (lo + hi);
                prev_step = uwb_math_abs(xn - x);
                x = xn;
                continue;
            }
            have_q = 1;

            /* φ の符号で区間を更新 (φ は単調減少) */
            if (phi > (uwb_real)0) lo = x; else hi = x;

            /* 丸め誤差の床。φ = ‖q‖² − α は項の相殺で桁落ちするので、|φ| が
             * 各項の絶対値の和 (scale) に対して machine epsilon 程度まで
             * 落ちたら、それ以上区間を詰めても数値的な意味がない。 */
            if (uwb_math_abs(phi) <= (uwb_real)8 * UWB_MATH_EPS * scale) break;

            /* 区間幅の相対許容 (eps に比例。旧実装の 1e-14 は double の
             * 64 eps に相当し、float ではどの幅でも満たせなかった)。 */
            scale_lam = uwb_math_abs(lo);
            if (uwb_math_abs(hi) > scale_lam) scale_lam = uwb_math_abs(hi);
            if (scale_lam < (uwb_real)1) scale_lam = (uwb_real)1;
            if (hi - lo < (uwb_real)64 * UWB_MATH_EPS * scale_lam) break;

            /* ニュートン候補。区間外に出る／更新量が前回の半分以下に
             * 縮まない／|φ'| がゼロ近傍なら二分に落とす。 */
            use_bisect = !(uwb_math_abs(dphi) > UWB_EPS);
            if (!use_bisect) {
                step = phi / dphi;
                xn = x - step;
                if (xn <= lo || xn >= hi ||
                    uwb_math_abs(step) > (uwb_real)0.5 * prev_step)
                    use_bisect = 1;
            }
            if (use_bisect) xn = (uwb_real)0.5 * (lo + hi);

            prev_step = uwb_math_abs(xn - x);
            x = xn;
            have_q = 0;
        }

        /* 最後に評価した x の q(x) をそのまま使う (0.5*(lo+hi) ではない。
         * rtsafe は片側だけを寄せるので、収束時点でも区間は広いことがある)。
         * 反復上限で抜けて x が未評価なら取り直す。 */
        if (!have_q && !beck_eval(&c, x, &phi, &dphi, &scale, q)) return 0;
    }
    for (k = 0; k < d; ++k) {
        uwb_real v = pbar[k] + q[k];
        if (v != v) return 0;
        out[k] = v;
    }
    if (d == 2) out[2] = cfg->z_fixed;
    return 1;
}
