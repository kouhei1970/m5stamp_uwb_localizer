/* uwb_survey_tape — メジャー実測値からのアンカー座標 閉形式計算の実装。
 *
 * 反復も固有分解も無い。全部代数式なので、関数を上から下まで読めば
 * そのまま計算の中身になっている（uwb_survey.c の trilat_init() が
 * n台×全対距離から座標を出すのに対し、こちらは基準2台からの距離だけで
 * 各ノードを1つずつ閉形式で置いていく）。
 */
#include "uwb_survey_tape.h"

#include <string.h>

#define TMAX UWB_SURVEY_TAPE_MAX_NODES

void uwb_survey_tape_input_init(uwb_survey_tape_input *in, int n)
{
    if (!in) return;
    memset(in, 0, sizeof(*in));
    in->n = n;
}

/* dist は両方向のどちらに入れてもよい（uwb_survey.c build_links() と同じ
 * 流儀）。両方あれば平均、片方だけなら片方を使う。i==j や未入力・非正の
 * 値は無効として 0 を返す。 */
static int fetch_dist(const uwb_survey_tape_input *in, int i, int j, uwb_real *out)
{
    int vij, vji;
    if (i == j) return 0;
    vij = (in->have[i][j] != 0) && (in->dist[i][j] > (uwb_real)0);
    vji = (in->have[j][i] != 0) && (in->dist[j][i] > (uwb_real)0);
    if (!vij && !vji) return 0;
    if (vij && vji) *out = (uwb_real)0.5 * (in->dist[i][j] + in->dist[j][i]);
    else if (vij)   *out = in->dist[i][j];
    else            *out = in->dist[j][i];
    return 1;
}

/* 根号の中身 r2 を検査する。
 *   r2 >= 0                              そのまま使える → 1
 *   -UWB_SURVEY_TAPE_CLAMP_TOL <= r2 < 0 メジャーの読み取り誤差とみなし
 *                                         0 に丸めて続行 → 1、*clamped=1
 *   それ以外（大きく負）                  測定同士が矛盾 → 0（*out は
 *                                         触らない） */
static int clamp_radicand(uwb_real r2, uwb_real *out, int *clamped)
{
    if (r2 >= (uwb_real)0) { *out = r2; return 1; }
    if (-r2 <= UWB_SURVEY_TAPE_CLAMP_TOL) { *out = (uwb_real)0; *clamped = 1; return 1; }
    return 0;
}

/* 必須の距離（d(0,1) と、各 k=2..n-1 の d(0,k)・d(1,k)）が全部揃っているか
 * だけを見る。欠けているペアを全部 missing に集める（1本見つかった時点で
 * 打ち切らない）。全部揃っていれば 1、1本でも欠けていれば 0。 */
static int check_required_present(const uwb_survey_tape_input *in, int n, unsigned long *missing)
{
    int k, ok = 1;
    uwb_real dummy;

    if (!fetch_dist(in, 0, 1, &dummy)) {
        *missing |= (1UL << (unsigned)uwb_survey_link_index(0, 1));
        ok = 0;
    }
    for (k = 2; k < n; ++k) {
        if (!fetch_dist(in, 0, k, &dummy)) {
            *missing |= (1UL << (unsigned)uwb_survey_link_index(0, k));
            ok = 0;
        }
        if (!fetch_dist(in, 1, k, &dummy)) {
            *missing |= (1UL << (unsigned)uwb_survey_link_index(1, k));
            ok = 0;
        }
    }
    return ok;
}

/* ペア (i,j) の水平距離の2乗 h_ij² = d_ij² − (z_i − z_j)² を求める。
 * 呼び出し時点で check_required_present()済み（distは必ず取れる）前提。
 * 根号の中身が大きく負なら out->status/err_i/err_j をその場で確定させて
 * 0 を返す（呼び出し側はそのまま uwb_survey_tape_solve() を打ち切る）。
 * わずかな負は 0 に丸めて out->clamped_pairs に記録する。 */
static int required_h2(const uwb_survey_tape_input *in, uwb_survey_tape_result *out,
                       int i, int j, uwb_real *h2)
{
    uwb_real d, dz, r2, hh;
    int clamped = 0;

    (void)fetch_dist(in, i, j, &d); /* 呼び出し前提により必ず成功する */
    dz = in->z[i] - in->z[j];
    r2 = d * d - dz * dz;

    if (!clamp_radicand(r2, &hh, &clamped)) {
        out->status = UWB_SURVEY_TAPE_ERR_INCONSISTENT;
        out->err_i  = i;
        out->err_j  = j;
        return 0;
    }
    if (clamped) out->clamped_pairs |= (1UL << (unsigned)uwb_survey_link_index(i, j));
    *h2 = hh;
    return 1;
}

int uwb_survey_tape_solve(const uwb_survey_tape_input *in, uwb_survey_tape_result *out)
{
    int n, k;
    uwb_real h01_2, h01;

    if (!out) return 0;
    memset(out, 0, sizeof(*out));
    out->err_i = -1;
    out->err_j = -1;
    if (!in) {
        /* in=NULL も「n が決まらない」と同じ扱いにする（ヘッダの契約どおり、
         * 呼び出し側が status だけ見ても NULL 引数を検出できるように）。 */
        out->status = UWB_SURVEY_TAPE_ERR_N_RANGE;
        return 0;
    }

    n = in->n;
    if (n < UWB_SURVEY_TAPE_MIN_NODES || n > UWB_SURVEY_TAPE_MAX_NODES) {
        out->status = UWB_SURVEY_TAPE_ERR_N_RANGE;
        return 0;
    }

    /* [1] 必須距離の充足チェック（全部集めてから返す）。 */
    if (!check_required_present(in, n, &out->missing)) {
        out->status = UWB_SURVEY_TAPE_ERR_MISSING;
        return 0;
    }

    /* [2] 基準線: index0 を原点、index1 を +X 軸上に置く。 */
    if (!required_h2(in, out, 0, 1, &h01_2)) return 0;
    if (!(h01_2 > (uwb_real)0)) {
        out->status = UWB_SURVEY_TAPE_ERR_BASELINE;
        out->err_i  = 0;
        out->err_j  = 1;
        return 0;
    }
    h01 = uwb_math_sqrt(h01_2);

    out->pos[0][0] = (uwb_real)0; out->pos[0][1] = (uwb_real)0; out->pos[0][2] = in->z[0];
    out->pos[1][0] = h01;         out->pos[1][1] = (uwb_real)0; out->pos[1][2] = in->z[1];

    /* [3][4] 残りの各ノードを、基準線からの2距離で閉形式に置き、
     * d(2,k) があれば y の符号を実測に合わせる。 */
    for (k = 2; k < n; ++k) {
        uwb_real h0k2, h1k2, x, y2, y2c, y;
        int clamped_y = 0;

        if (!required_h2(in, out, 0, k, &h0k2)) return 0;
        if (!required_h2(in, out, 1, k, &h1k2)) return 0;

        /* 余弦定理: index0=(0,0), index1=(h01,0) を頂点に持つ三角形で、
         * ノードkまでの2辺 h0k・h1k から x 座標を出す。
         *   h1k² = (x-h01)² + y²  、 h0k² = x² + y²
         * を引き算して y² を消すと x = (h01² + h0k² − h1k²) / (2 h01)。 */
        x  = (h01_2 + h0k2 - h1k2) / ((uwb_real)2 * h01);
        y2 = h0k2 - x * x;

        if (!clamp_radicand(y2, &y2c, &clamped_y)) {
            /* (0,1,k) の三角形が三角不等式に違反している（ペア単体の問題
             * ではないので err_i=-1 にして「ノード err_j 側」を示す）。 */
            out->status = UWB_SURVEY_TAPE_ERR_INCONSISTENT;
            out->err_i  = -1;
            out->err_j  = k;
            return 0;
        }
        if (clamped_y) out->clamped_nodes |= (1UL << (unsigned)k);
        y = uwb_math_sqrt(y2c);

        if (k == 2) {
            /* 規約: index2 は +y 側に固定。 */
            out->pos[2][0] = x;
            out->pos[2][1] = y;
            out->pos[2][2] = in->z[2];
            continue;
        }

        {
            uwb_real d2k;
            uwb_real dx  = x - out->pos[2][0];
            uwb_real dzk = in->z[k] - out->pos[2][2];
            if (fetch_dist(in, 2, k, &d2k)) {
                /* ±y のうち、index2 までの実測距離 d(2,k) に近い方を採る。 */
                uwb_real dyp        = y - out->pos[2][1];
                uwb_real dym        = -y - out->pos[2][1];
                uwb_real pred_plus  = uwb_math_sqrt(dx * dx + dyp * dyp + dzk * dzk);
                uwb_real pred_minus = uwb_math_sqrt(dx * dx + dym * dym + dzk * dzk);
                uwb_real err_plus   = uwb_math_abs(pred_plus  - d2k);
                uwb_real err_minus  = uwb_math_abs(pred_minus - d2k);
                if (err_minus < err_plus) y = -y;
            } else {
                /* 判定材料が無い。+y のまま置き、未確定として記録する。 */
                out->sign_unresolved |= (1UL << (unsigned)k);
            }
        }
        out->pos[k][0] = x;
        out->pos[k][1] = y;
        out->pos[k][2] = in->z[k];
    }

    for (k = n; k < TMAX; ++k) {
        out->pos[k][0] = out->pos[k][1] = out->pos[k][2] = (uwb_real)0;
    }

    out->status = UWB_SURVEY_TAPE_OK;
    out->ok     = 1;
    return 1;
}
