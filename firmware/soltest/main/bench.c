/**
 * @file bench.c
 * @brief 基本演算・メモリアクセスパターンのマイクロベンチマーク実装。
 *
 * 計測方針(要件を満たすための設計判断をまとめて説明する。各関数のコメントは
 * ここで説明した前提を踏まえたものとして簡潔に書く):
 *
 * --- サイクルカウンタ ---
 * esp_cpu_get_cycle_count() (esp_hw_support 提供、esp_cpu.h) を使う。
 * Xtensa の CCOUNT レジスタを読む薄いラッパで、esp_timer の us 分解能では
 * 拾えない1桁台のサイクル差を見るのに必要。戻り値は esp_cpu_cycle_count_t
 * (実体は uint32_t)。このカウンタは CPU コアごとに独立しているため、
 * 計測開始(t0)から終了(t1)までの間にタスクがコアを移動すると差分が
 * 無意味になる。呼び出し元(main.c)はベンチ実行タスクを
 * xTaskCreatePinnedToCore で特定コアへピン留めしている。
 *
 * --- レイテンシとスループットの作り分け ---
 * レイテンシ計測: 前回の結果を次回の入力にそのまま使う依存チェイン
 * (acc = acc OP step を1本、逐次実行)。CPU がその演算の結果を
 * 待ってから次の命令を出すしかない状況を再現する。
 * スループット計測: 独立したアキュムレータを8本並べ、同じ形の更新を
 * 8本分まとめて書く(ループ内でコンパイラのアンロールに頼らず、
 * 手でベタ書きする。逆アセンブルで8本分の命令列が実際に並んでいることを
 * 確認できるようにするため)。8本は互いに依存が無いので、パイプライン化
 * された命令であれば複数を重ねて実行でき、依存を待つ必要が無い場合の
 * 実効コストが分かる。ソフトウェア実装(除算・sqrtなど、実体はライブラリ
 * 関数呼び出し)は Xtensa LX7 がインオーダー単発issueのため、独立本数を
 * 増やしても劇的には縮まらない見込み(呼び出しの解決順序がプログラム順に
 * 縛られるため)だが、それ自体が意味のある実測結果になるので両方測る。
 *
 * --- 最適化で消されないようにする仕組み ---
 * 1. 初期値・係数はすべて実行時にしか分からない値(esp_cpu_get_cycle_count()
 *    の下位ビット)から作る。コンパイル時定数にしないことで、コンパイラが
 *    ループ全体を「計算済みの1個の定数」に畳み込むこと自体ができなくなる
 *    (コンパイル時に値が分からないので計算のしようがない)。
 * 2. 浮動小数点の加減乗除は、-ffast-math を付けない既定のビルドでは
 *    演算順序の入れ替え(結合則の利用)が許されない。したがって
 *    「同じ演算を N 回繰り返す」ことを「1回の演算に置き換える」ような
 *    畳み込みは、コンパイラ側にそもそも実行する権限が無い。
 * 3. 依存チェインの結果は、ループを抜けたあとに一度だけ volatile な
 *    シンク変数へ書く。ループの外で1回だけ、という要件どおりの形にして
 *    ある(ループ内で volatile アクセスすると、そのロード/ストア自体が
 *    計測対象に乗ってしまい、素の演算コストを歪めるため避ける)。
 *    この最終書き込みが「最後の1回の結果」を要求する以上、コンパイラは
 *    そこへ至る計算の連鎖(=ループ全体)を消せない。
 * 4. メモリアクセスパターン計測(9x9行列)だけは事情が逆で、「メモリへの
 *    ロード/ストアを実際に発生させること」自体が計測目的なので、対象の
 *    配列そのものを volatile にする。これはスカラー演算ベンチの方針
 *    (ループ内 volatile を避ける)とは矛盾しない。目的が違うので手段も
 *    変えている、という判断であることに注意。
 *
 * --- 発散・退化(値が0や1に張り付く)の防止 ---
 * 依存チェインを何十万回も回すので、値が無限大やゼロに飽和すると
 * その後の演算コストが変わってしまう懸念がある(特にソフトウェア実装は
 * 値によって内部の特殊経路(0・NaN・非正規化数など)を通る可能性を
 * 否定できない)。そのため各演算の刻み幅・初期値は「何十万回繰り返しても
 * 極端な値(0/無限大/非正規化数)に到達しない範囲」に収まるよう選んである。
 * 具体的な範囲の根拠は各 seed_* 関数のコメントに書く。
 *
 * --- ループ自体のオーバーヘッド ---
 * 対象演算を一切含まない、同じ形のループ(カウンタの更新・比較・分岐と、
 * ループが丸ごと消されないための最小限の副作用のみ)を測り、そこからの
 * 差分を「対象演算そのものの正味コスト」として報告する。
 * メモリアクセスベンチ(9x9のネストループ)については、フラットな
 * ループのオーバーヘッド値を要素あたりコストの近似として流用している
 * (ネスト構造の分だけ実際のオーバーヘッドよりわずかに小さく見積もる
 * 可能性があるが、その差は無視できる程度と考えられる。詳細は
 * bench_memory() のコメント)。
 */
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

#include "esp_cpu.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bench.h"

static const char *TAG = "uwb_bench";

/* ============================================================ 計測条件 */

/* レイテンシ計測1本あたりの依存チェインの長さ。要件(10万回以上)に対し
 * 余裕を持たせた値。 */
#define BENCH_ITERS 200000u

/* スループット計測で並べる独立アキュムレータの本数。 */
#define BENCH_LANES 8u

/* スループット計測の外側ループ回数。BENCH_LANES 本×この回数で
 * BENCH_ITERS と同じ総演算数になるようにし、レイテンシ計測と
 * 演算1回あたりのコストを対等に比較できるようにする。 */
#define BENCH_OUTER_TRIPS (BENCH_ITERS / BENCH_LANES)

/* 9x9行列(EKFの共分散P相当)の一辺。 */
#define MAT_N 9

/* メモリアクセスベンチで行列を何周なめるか。MAT_N*MAT_N*この回数が
 * 要件(10万要素アクセス以上)を満たすようにする
 * (9*9*2000 = 162,000 > 100,000)。 */
#define MEM_BENCH_REPEATS 2000u

/* ============================================================ 最適化除去防止シンク
 *
 * ループを抜けたあとに一度だけ書き込む。値そのものに意味は無く、
 * 「この値を最後に読む誰かがいる」という体裁だけがあればよい
 * (実際に読むのはこのファイルの外の誰でもないが、volatile
 * グローバル変数への書き込みはそれ自体が処理系にとって観測可能な
 * 副作用なので、そこへ至る計算をコンパイラは省略できない)。
 */
static volatile double   g_sink_d   = 0.0;
static volatile float    g_sink_f   = 0.0f;
static volatile uint32_t g_sink_u32 = 0;

/* ============================================================ 実行時seed生成
 *
 * esp_cpu_get_cycle_count() の下位ビットを種にする。呼び出しごとに
 * 違う値になる(コンパイル時には絶対に分からない)ことが目的であって、
 * 乱数としての品質は問わない。
 */
static inline uint32_t bench_seed_now(void)
{
    return (uint32_t)esp_cpu_get_cycle_count();
}

/* ---- 加減算チェイン用 ----
 * 初期値 1.0〜1.255、刻み幅は小さい正数。BENCH_ITERS 回加算しても
 * 高々数十〜数百程度にしか増えない(オーバーフローや桁落ちが起きない
 * 範囲に収める)。lane は8本のアキュムレータを別々の値にするためのオフセット。 */
static inline double seed_acc_d(uint32_t seed, uint32_t lane)
{
    return 1.0 + (double)((seed + lane * 97u) & 0xFFu) * 1e-3; /* 1.000 - 1.255 */
}
static inline double seed_addstep_d(uint32_t seed, uint32_t lane)
{
    return 1e-4 + (double)((seed + lane * 131u) & 0xFFu) * 1e-7; /* 1.0e-4 台 */
}
static inline float seed_acc_f(uint32_t seed, uint32_t lane)
{
    return 1.0f + (float)((seed + lane * 97u) & 0xFFu) * 1e-3f;
}
static inline float seed_addstep_f(uint32_t seed, uint32_t lane)
{
    /* float は有効桁が約7桁(epsilon ≈ 1.19e-7)。刻み幅を double 版より
     * 大きめの 1e-3 台にして、値が育ってきても丸めに埋もれないようにする。 */
    return 1e-3f + (float)((seed + lane * 131u) & 0xFFu) * 1e-6f;
}

/* ---- 乗除算チェイン用 ----
 * 刻み幅を「1に極めて近い値」にする。乗算チェインは acc *= step を
 * BENCH_ITERS 回繰り返すと acc は指数的に増減するが、step が
 * 1+ε(εが小さい)であれば全体の増減は e^(N*ε) 程度に収まり、
 * 何十万回繰り返しても発散・消滅しない。除算チェインも同じ step を
 * 使って acc /= step とすることで同様に有界にできる。 */
static inline double seed_mulstep_d(uint32_t seed, uint32_t lane)
{
    /* ε ≈ 1e-7〜1e-9台。double の epsilon(≈2.2e-16)に対して十分大きく
     * 丸めに消えない。N*ε ≈ 200000*1e-7 = 0.02 → 全体の増減は数%程度。 */
    return 1.0 + 1e-7 + (double)((seed + lane * 71u) & 0xFFu) * 1e-9;
}
static inline float seed_mulstep_f(uint32_t seed, uint32_t lane)
{
    /* float の epsilon(≈1.19e-7)より十分大きい相対幅(1e-5台)を使う。
     * N*ε ≈ 200000*1e-5 = 2 → 全体で e^2 ≈ 7.4倍程度の変動に収まる。 */
    return 1.0f + 1e-5f + (float)((seed + lane * 71u) & 0xFFu) * 1e-7f;
}

/* ---- sqrt チェイン用 ----
 * acc = sqrt(acc) を繰り返すと数回で 1.0 近辺へ収束する。ソフトウェア
 * sqrt(newlib の fdlibm 由来、仮数部ビット数固定のbit-by-bitアルゴリズム)
 * は特殊値(0・負・NaN・Inf)以外では入力値によらず同じ手順を踏むはずなので
 * 収束そのものは計測上の問題にならないと考えられるが、念のため収束先
 * ちょうどの値に張り付かせないよう、毎回ごく小さい値を加算して値を
 * わずかに動かし続ける(このtiny加算はハードウェア1命令のadd.sで
 * 極めて軽いため、sqrtのコストへの寄与は無視できる)。 */
static inline double seed_sqrt_acc_d(uint32_t seed, uint32_t lane)
{
    return 2.0 + (double)((seed + lane * 113u) & 0xFFu) * 1e-2; /* 2.0 - 4.55、常に正 */
}
static inline double seed_sqrt_nudge_d(uint32_t seed, uint32_t lane)
{
    return 1e-6 + (double)((seed + lane * 149u) & 0xFFu) * 1e-9;
}
static inline float seed_sqrt_acc_f(uint32_t seed, uint32_t lane)
{
    return 2.0f + (float)((seed + lane * 113u) & 0xFFu) * 1e-2f;
}
static inline float seed_sqrt_nudge_f(uint32_t seed, uint32_t lane)
{
    /* float の ULP(2〜4.5付近で ~2e-7)より十分大きい値にする。 */
    return 1e-5f + (float)((seed + lane * 149u) & 0xFFu) * 1e-8f;
}

/* ---- 積和(FMA, a = c*a + b)チェイン用 ----
 * acc_{n+1} = c*acc_n + b という線形漸化式(1次のIIRフィルタと同じ形)。
 * |c| < 1 であれば n→∞ で acc は b/(1-c) に収束する安定な漸化式になり、
 * 何十万回繰り返しても発散しない。b・c 自身はループ内で不変の値だが、
 * 乗算の片方の引数(acc)は毎回変わる依存チェインなので、コンパイラが
 * 「c*acc」をループ不変式としてループの外へくくり出すことはできない
 * (acc が毎回変わる以上、c*acc も毎回変わる)。 */
static inline double seed_fma_c_d(uint32_t seed, uint32_t lane)
{
    return 0.5 + (double)((seed + lane * 61u) & 0xFFu) * (0.49 / 255.0); /* 0.50 - 0.99 */
}
static inline double seed_fma_b_d(uint32_t seed, uint32_t lane)
{
    return 0.1 + (double)((seed + lane * 89u) & 0xFFu) * (1.9 / 255.0); /* 0.1 - 2.0 */
}
static inline float seed_fma_c_f(uint32_t seed, uint32_t lane)
{
    return 0.5f + (float)((seed + lane * 61u) & 0xFFu) * (0.49f / 255.0f);
}
static inline float seed_fma_b_f(uint32_t seed, uint32_t lane)
{
    return 0.1f + (float)((seed + lane * 89u) & 0xFFu) * (1.9f / 255.0f);
}

/* ---- uint32 乗除算(参考値)用 ----
 * mod 2^32 の世界での演算なのでオーバーフローは未定義動作にならない
 * (符号なし整数の巻き戻りは規格で定義された挙動)。乗算チェインは
 * step を奇数にすることで mod 2^32 上の可逆写像になり、特定の値に
 * 縮退しない。除算チェインは「大きい値を繰り返し割ると数十回で0近辺に
 * 縮退し、以後 0/divisor=0 を繰り返すだけになる」性質があるため、
 * 商に小さい奇数を1回加算して0への張り付きを防ぐ(整数加算は
 * ハードウェア1命令なのでコストへの寄与は無視できる)。 */
static inline uint32_t seed_u32_acc(uint32_t seed, uint32_t lane)
{
    return 0x2545F491u ^ (seed + lane * 2654435761u);
}
static inline uint32_t seed_u32_mulstep(uint32_t seed, uint32_t lane)
{
    return (seed + lane * 40503u) | 1u; /* 奇数 = mod 2^32 で可逆 */
}
static inline uint32_t seed_u32_divisor(uint32_t seed, uint32_t lane)
{
    return ((seed + lane * 6151u) % 65521u) | 1u; /* 65521は素数。奇数化含め非0を保証 */
}
static inline uint32_t seed_u32_increment(uint32_t seed, uint32_t lane)
{
    return (seed + lane * 104729u) | 1u;
}

/* ============================================================ ループオーバーヘッド
 *
 * 対象演算を含まない、同じ形のループを測る。ループ内には
 * 「i を絡めた最小限の副作用(XOR)」だけを置く。中身が完全に空だと
 * コンパイラがループそのものを丸ごと削除できてしまう(空ループは
 * トリップカウントが実行時にしか分からなくても、観測可能な副作用が
 * 無ければ削除してよいというのが最適化として正当なため)。i を絡めた
 * XOR は 0..N-1 の並びに対する単純な閉形式が無い(コンパイラが
 * ループを畳み込む定型パターンとして持っていない)ので、削除も畳み込みも
 * されない。XOR 自体は加算と同程度に軽い1命令なので、この分だけ
 * 「本当のループ制御コスト」よりわずかに大きい値を測っている可能性があるが、
 * 無視できる程度(1サイクル前後)と考えられる。 */
static uint32_t bench_loop_overhead(uint32_t seed)
{
    uint32_t dummy = seed;
    uint32_t t0    = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        dummy ^= i;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_u32  = dummy;
    return t1 - t0;
}

/* ============================================================ 二項演算(+,-,*,/) の生成マクロ
 *
 * float/double の加減乗除は形が完全に同じ(TYPEと演算子だけが違う)ので
 * マクロで生成する。手で20本近く書き写すよりコピペミスのリスクが低い。
 * 生成される中身は素直な for ループ1本なので、逆アセンブルで見比べる
 * うえでも困らない。
 */
#define DEFINE_LATENCY_BINOP(FNNAME, TYPE, OP, ACCFN, STEPFN, SINK)                                  \
    static uint32_t FNNAME(uint32_t seed)                                                            \
    {                                                                                                 \
        TYPE     acc  = ACCFN(seed, 0);                                                               \
        TYPE     step = STEPFN(seed, 0);                                                              \
        uint32_t t0   = (uint32_t)esp_cpu_get_cycle_count();                                          \
        for (uint32_t i = 0; i < BENCH_ITERS; ++i) {                                                  \
            acc = acc OP step;                                                                        \
        }                                                                                              \
        uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();                                            \
        SINK        = acc;                                                                             \
        (void)t1;                                                                                      \
        return t1 - t0;                                                                                \
    }

#define DEFINE_THROUGHPUT_BINOP(FNNAME, TYPE, OP, ACCFN, STEPFN, SINK)                               \
    static uint32_t FNNAME(uint32_t seed)                                                            \
    {                                                                                                 \
        TYPE a0 = ACCFN(seed, 0), a1 = ACCFN(seed, 1), a2 = ACCFN(seed, 2), a3 = ACCFN(seed, 3);      \
        TYPE a4 = ACCFN(seed, 4), a5 = ACCFN(seed, 5), a6 = ACCFN(seed, 6), a7 = ACCFN(seed, 7);      \
        TYPE s0 = STEPFN(seed, 0), s1 = STEPFN(seed, 1), s2 = STEPFN(seed, 2), s3 = STEPFN(seed, 3);  \
        TYPE s4 = STEPFN(seed, 4), s5 = STEPFN(seed, 5), s6 = STEPFN(seed, 6), s7 = STEPFN(seed, 7);  \
        uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();                                            \
        for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {                                            \
            a0 = a0 OP s0;                                                                             \
            a1 = a1 OP s1;                                                                             \
            a2 = a2 OP s2;                                                                             \
            a3 = a3 OP s3;                                                                             \
            a4 = a4 OP s4;                                                                             \
            a5 = a5 OP s5;                                                                             \
            a6 = a6 OP s6;                                                                             \
            a7 = a7 OP s7;                                                                             \
        }                                                                                              \
        uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();                                            \
        SINK        = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;                                           \
        (void)t1;                                                                                      \
        return t1 - t0;                                                                                \
    }

/* float: 加減乗除 */
DEFINE_LATENCY_BINOP(bench_lat_add_f, float, +, seed_acc_f, seed_addstep_f, g_sink_f)
DEFINE_THROUGHPUT_BINOP(bench_thr_add_f, float, +, seed_acc_f, seed_addstep_f, g_sink_f)
DEFINE_LATENCY_BINOP(bench_lat_sub_f, float, -, seed_acc_f, seed_addstep_f, g_sink_f)
DEFINE_THROUGHPUT_BINOP(bench_thr_sub_f, float, -, seed_acc_f, seed_addstep_f, g_sink_f)
DEFINE_LATENCY_BINOP(bench_lat_mul_f, float, *, seed_acc_f, seed_mulstep_f, g_sink_f)
DEFINE_THROUGHPUT_BINOP(bench_thr_mul_f, float, *, seed_acc_f, seed_mulstep_f, g_sink_f)
DEFINE_LATENCY_BINOP(bench_lat_div_f, float, /, seed_acc_f, seed_mulstep_f, g_sink_f)
DEFINE_THROUGHPUT_BINOP(bench_thr_div_f, float, /, seed_acc_f, seed_mulstep_f, g_sink_f)

/* double: 加減乗除 */
DEFINE_LATENCY_BINOP(bench_lat_add_d, double, +, seed_acc_d, seed_addstep_d, g_sink_d)
DEFINE_THROUGHPUT_BINOP(bench_thr_add_d, double, +, seed_acc_d, seed_addstep_d, g_sink_d)
DEFINE_LATENCY_BINOP(bench_lat_sub_d, double, -, seed_acc_d, seed_addstep_d, g_sink_d)
DEFINE_THROUGHPUT_BINOP(bench_thr_sub_d, double, -, seed_acc_d, seed_addstep_d, g_sink_d)
DEFINE_LATENCY_BINOP(bench_lat_mul_d, double, *, seed_acc_d, seed_mulstep_d, g_sink_d)
DEFINE_THROUGHPUT_BINOP(bench_thr_mul_d, double, *, seed_acc_d, seed_mulstep_d, g_sink_d)
DEFINE_LATENCY_BINOP(bench_lat_div_d, double, /, seed_acc_d, seed_mulstep_d, g_sink_d)
DEFINE_THROUGHPUT_BINOP(bench_thr_div_d, double, /, seed_acc_d, seed_mulstep_d, g_sink_d)

/* uint32: 乗算(参考値)。除算は0への縮退対策が要るので下で個別に書く。 */
DEFINE_LATENCY_BINOP(bench_lat_mul_u32, uint32_t, *, seed_u32_acc, seed_u32_mulstep, g_sink_u32)
DEFINE_THROUGHPUT_BINOP(bench_thr_mul_u32, uint32_t, *, seed_u32_acc, seed_u32_mulstep, g_sink_u32)

#undef DEFINE_LATENCY_BINOP
#undef DEFINE_THROUGHPUT_BINOP

/* ============================================================ sqrt (特殊形: 関数呼び出し) */

static uint32_t bench_lat_sqrt_f(uint32_t seed)
{
    float    acc   = seed_sqrt_acc_f(seed, 0);
    float    nudge = seed_sqrt_nudge_f(seed, 0);
    uint32_t t0    = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        acc = sqrtf(acc) + nudge;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_f    = acc;
    return t1 - t0;
}

static uint32_t bench_thr_sqrt_f(uint32_t seed)
{
    float a0 = seed_sqrt_acc_f(seed, 0), a1 = seed_sqrt_acc_f(seed, 1);
    float a2 = seed_sqrt_acc_f(seed, 2), a3 = seed_sqrt_acc_f(seed, 3);
    float a4 = seed_sqrt_acc_f(seed, 4), a5 = seed_sqrt_acc_f(seed, 5);
    float a6 = seed_sqrt_acc_f(seed, 6), a7 = seed_sqrt_acc_f(seed, 7);
    float n0 = seed_sqrt_nudge_f(seed, 0), n1 = seed_sqrt_nudge_f(seed, 1);
    float n2 = seed_sqrt_nudge_f(seed, 2), n3 = seed_sqrt_nudge_f(seed, 3);
    float n4 = seed_sqrt_nudge_f(seed, 4), n5 = seed_sqrt_nudge_f(seed, 5);
    float n6 = seed_sqrt_nudge_f(seed, 6), n7 = seed_sqrt_nudge_f(seed, 7);
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {
        a0 = sqrtf(a0) + n0;
        a1 = sqrtf(a1) + n1;
        a2 = sqrtf(a2) + n2;
        a3 = sqrtf(a3) + n3;
        a4 = sqrtf(a4) + n4;
        a5 = sqrtf(a5) + n5;
        a6 = sqrtf(a6) + n6;
        a7 = sqrtf(a7) + n7;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_f    = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
    return t1 - t0;
}

static uint32_t bench_lat_sqrt_d(uint32_t seed)
{
    double   acc   = seed_sqrt_acc_d(seed, 0);
    double   nudge = seed_sqrt_nudge_d(seed, 0);
    uint32_t t0    = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        acc = sqrt(acc) + nudge;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_d    = acc;
    return t1 - t0;
}

static uint32_t bench_thr_sqrt_d(uint32_t seed)
{
    double a0 = seed_sqrt_acc_d(seed, 0), a1 = seed_sqrt_acc_d(seed, 1);
    double a2 = seed_sqrt_acc_d(seed, 2), a3 = seed_sqrt_acc_d(seed, 3);
    double a4 = seed_sqrt_acc_d(seed, 4), a5 = seed_sqrt_acc_d(seed, 5);
    double a6 = seed_sqrt_acc_d(seed, 6), a7 = seed_sqrt_acc_d(seed, 7);
    double n0 = seed_sqrt_nudge_d(seed, 0), n1 = seed_sqrt_nudge_d(seed, 1);
    double n2 = seed_sqrt_nudge_d(seed, 2), n3 = seed_sqrt_nudge_d(seed, 3);
    double n4 = seed_sqrt_nudge_d(seed, 4), n5 = seed_sqrt_nudge_d(seed, 5);
    double n6 = seed_sqrt_nudge_d(seed, 6), n7 = seed_sqrt_nudge_d(seed, 7);
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {
        a0 = sqrt(a0) + n0;
        a1 = sqrt(a1) + n1;
        a2 = sqrt(a2) + n2;
        a3 = sqrt(a3) + n3;
        a4 = sqrt(a4) + n4;
        a5 = sqrt(a5) + n5;
        a6 = sqrt(a6) + n6;
        a7 = sqrt(a7) + n7;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_d    = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
    return t1 - t0;
}

/* ============================================================ 積和 FMA (a = c*a + b) */

static uint32_t bench_lat_fma_f(uint32_t seed)
{
    float    acc = seed_acc_f(seed, 0);
    float    c   = seed_fma_c_f(seed, 0);
    float    b   = seed_fma_b_f(seed, 0);
    uint32_t t0  = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        acc = c * acc + b; /* -ffp-contract=fast (既定) なら madd.s 1命令になる想定。objdumpで確認する */
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_f    = acc;
    return t1 - t0;
}

static uint32_t bench_thr_fma_f(uint32_t seed)
{
    float a0 = seed_acc_f(seed, 0), a1 = seed_acc_f(seed, 1), a2 = seed_acc_f(seed, 2), a3 = seed_acc_f(seed, 3);
    float a4 = seed_acc_f(seed, 4), a5 = seed_acc_f(seed, 5), a6 = seed_acc_f(seed, 6), a7 = seed_acc_f(seed, 7);
    float c0 = seed_fma_c_f(seed, 0), c1 = seed_fma_c_f(seed, 1), c2 = seed_fma_c_f(seed, 2), c3 = seed_fma_c_f(seed, 3);
    float c4 = seed_fma_c_f(seed, 4), c5 = seed_fma_c_f(seed, 5), c6 = seed_fma_c_f(seed, 6), c7 = seed_fma_c_f(seed, 7);
    float b0 = seed_fma_b_f(seed, 0), b1 = seed_fma_b_f(seed, 1), b2 = seed_fma_b_f(seed, 2), b3 = seed_fma_b_f(seed, 3);
    float b4 = seed_fma_b_f(seed, 4), b5 = seed_fma_b_f(seed, 5), b6 = seed_fma_b_f(seed, 6), b7 = seed_fma_b_f(seed, 7);
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {
        a0 = c0 * a0 + b0;
        a1 = c1 * a1 + b1;
        a2 = c2 * a2 + b2;
        a3 = c3 * a3 + b3;
        a4 = c4 * a4 + b4;
        a5 = c5 * a5 + b5;
        a6 = c6 * a6 + b6;
        a7 = c7 * a7 + b7;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_f    = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
    return t1 - t0;
}

static uint32_t bench_lat_fma_d(uint32_t seed)
{
    double   acc = seed_acc_d(seed, 0);
    double   c   = seed_fma_c_d(seed, 0);
    double   b   = seed_fma_b_d(seed, 0);
    uint32_t t0  = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        acc = c * acc + b; /* madd.d は無いので mul(__muldf3)+add(__adddf3) の2回呼び出しになる想定 */
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_d    = acc;
    return t1 - t0;
}

static uint32_t bench_thr_fma_d(uint32_t seed)
{
    double a0 = seed_acc_d(seed, 0), a1 = seed_acc_d(seed, 1), a2 = seed_acc_d(seed, 2), a3 = seed_acc_d(seed, 3);
    double a4 = seed_acc_d(seed, 4), a5 = seed_acc_d(seed, 5), a6 = seed_acc_d(seed, 6), a7 = seed_acc_d(seed, 7);
    double c0 = seed_fma_c_d(seed, 0), c1 = seed_fma_c_d(seed, 1), c2 = seed_fma_c_d(seed, 2), c3 = seed_fma_c_d(seed, 3);
    double c4 = seed_fma_c_d(seed, 4), c5 = seed_fma_c_d(seed, 5), c6 = seed_fma_c_d(seed, 6), c7 = seed_fma_c_d(seed, 7);
    double b0 = seed_fma_b_d(seed, 0), b1 = seed_fma_b_d(seed, 1), b2 = seed_fma_b_d(seed, 2), b3 = seed_fma_b_d(seed, 3);
    double b4 = seed_fma_b_d(seed, 4), b5 = seed_fma_b_d(seed, 5), b6 = seed_fma_b_d(seed, 6), b7 = seed_fma_b_d(seed, 7);
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {
        a0 = c0 * a0 + b0;
        a1 = c1 * a1 + b1;
        a2 = c2 * a2 + b2;
        a3 = c3 * a3 + b3;
        a4 = c4 * a4 + b4;
        a5 = c5 * a5 + b5;
        a6 = c6 * a6 + b6;
        a7 = c7 * a7 + b7;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_d    = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
    return t1 - t0;
}

/* ============================================================ uint32 除算(参考値、特殊形) */

static uint32_t bench_lat_div_u32(uint32_t seed)
{
    uint32_t acc     = seed_u32_acc(seed, 0);
    uint32_t divisor = seed_u32_divisor(seed, 0);
    uint32_t inc     = seed_u32_increment(seed, 0);
    uint32_t t0      = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_ITERS; ++i) {
        acc = acc / divisor + inc; /* +inc は 0 への縮退防止(コメント冒頭参照) */
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_u32  = acc;
    return t1 - t0;
}

static uint32_t bench_thr_div_u32(uint32_t seed)
{
    uint32_t a0 = seed_u32_acc(seed, 0), a1 = seed_u32_acc(seed, 1), a2 = seed_u32_acc(seed, 2), a3 = seed_u32_acc(seed, 3);
    uint32_t a4 = seed_u32_acc(seed, 4), a5 = seed_u32_acc(seed, 5), a6 = seed_u32_acc(seed, 6), a7 = seed_u32_acc(seed, 7);
    uint32_t d0 = seed_u32_divisor(seed, 0), d1 = seed_u32_divisor(seed, 1), d2 = seed_u32_divisor(seed, 2), d3 = seed_u32_divisor(seed, 3);
    uint32_t d4 = seed_u32_divisor(seed, 4), d5 = seed_u32_divisor(seed, 5), d6 = seed_u32_divisor(seed, 6), d7 = seed_u32_divisor(seed, 7);
    uint32_t n0 = seed_u32_increment(seed, 0), n1 = seed_u32_increment(seed, 1), n2 = seed_u32_increment(seed, 2), n3 = seed_u32_increment(seed, 3);
    uint32_t n4 = seed_u32_increment(seed, 4), n5 = seed_u32_increment(seed, 5), n6 = seed_u32_increment(seed, 6), n7 = seed_u32_increment(seed, 7);
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t i = 0; i < BENCH_OUTER_TRIPS; ++i) {
        a0 = a0 / d0 + n0;
        a1 = a1 / d1 + n1;
        a2 = a2 / d2 + n2;
        a3 = a3 / d3 + n3;
        a4 = a4 / d4 + n4;
        a5 = a5 / d5 + n5;
        a6 = a6 / d6 + n6;
        a7 = a7 / d7 + n7;
    }
    uint32_t t1 = (uint32_t)esp_cpu_get_cycle_count();
    g_sink_u32  = a0 + a1 + a2 + a3 + a4 + a5 + a6 + a7;
    return t1 - t0;
}

/* ============================================================ 集計・表示補助 */

/* 計測生値(cycles)からループオーバーヘッドを差し引き、対象演算1回あたりの
 * 正味サイクル数を返す。trips はこの計測が回したループのトリップ数
 * (レイテンシなら BENCH_ITERS、スループットなら BENCH_OUTER_TRIPS)、
 * ops は実行された対象演算の総数(レイテンシなら trips と同じ、
 * スループットなら trips*BENCH_LANES)。 */
static double net_cycles_per_op(uint32_t raw_cycles, uint32_t trips, uint32_t ops, double overhead_per_trip)
{
    double overhead_total = overhead_per_trip * (double)trips;
    double net            = (double)raw_cycles - overhead_total;
    if (net < 0.0) {
        /* オーバーヘッド推定値の誤差で理論上ごくまれに負になり得る。
         * 「対象演算そのもののコストはゼロ未満にはならない」という
         * 物理的な制約に合わせて0に丸める。 */
        net = 0.0;
    }
    return net / (double)ops;
}

typedef struct {
    const char *label;
    double      lat_cycles; /* レイテンシ: 演算1回あたりの正味サイクル数 */
    double      thr_cycles; /* スループット: 演算1回あたりの正味サイクル数 */
} bench_row_t;

static void report_row(bench_row_t *row, const char *label, uint32_t raw_lat, uint32_t raw_thr,
                        double overhead_per_trip, double ns_per_cycle)
{
    row->label      = label;
    row->lat_cycles = net_cycles_per_op(raw_lat, BENCH_ITERS, BENCH_ITERS, overhead_per_trip);
    row->thr_cycles = net_cycles_per_op(raw_thr, BENCH_OUTER_TRIPS, BENCH_ITERS, overhead_per_trip);
    ESP_LOGI(TAG, "  %-12s latency %8.2f cyc (%7.1f ns)   throughput %8.2f cyc (%7.1f ns)", label,
             row->lat_cycles, row->lat_cycles * ns_per_cycle, row->thr_cycles, row->thr_cycles * ns_per_cycle);
    /* ベンチ関数の呼び出し1本ごとに一呼吸置き、Task Watchdog を踏まないよう
     * アイドルタスクへ制御を返す(bench.h のコメント参照)。 */
    vTaskDelay(pdMS_TO_TICKS(1));
}

/* ============================================================ 1. 基本演算ベンチ */

static void bench_scalar_ops(double overhead_per_trip, double ns_per_cycle)
{
    uint32_t seed = bench_seed_now();

    ESP_LOGI(TAG, "--- 1. 基本演算のサイクル数 (%u回, スループットは%u本x%u周) ---", BENCH_ITERS,
             BENCH_LANES, BENCH_OUTER_TRIPS);
    ESP_LOGI(TAG, "  ループ制御オーバーヘッド: %.3f cyc/回 (以下の数値から差し引き済み)",
             overhead_per_trip);

    bench_row_t r_add_f, r_sub_f, r_mul_f, r_fma_f, r_div_f, r_sqrt_f;
    bench_row_t r_add_d, r_sub_d, r_mul_d, r_fma_d, r_div_d, r_sqrt_d;
    bench_row_t r_mul_u32, r_div_u32;

    ESP_LOGI(TAG, " [ハードウェア命令になる演算群: float の加減乗算・積和]");
    report_row(&r_add_f, "float add", bench_lat_add_f(seed), bench_thr_add_f(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_sub_f, "float sub", bench_lat_sub_f(seed), bench_thr_sub_f(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_mul_f, "float mul", bench_lat_mul_f(seed), bench_thr_mul_f(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_fma_f, "float fma", bench_lat_fma_f(seed), bench_thr_fma_f(seed), overhead_per_trip,
               ns_per_cycle);

    ESP_LOGI(TAG, " [ソフトウェア実装になる演算群: float除算・sqrt、double全演算]");
    report_row(&r_div_f, "float div", bench_lat_div_f(seed), bench_thr_div_f(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_sqrt_f, "float sqrt", bench_lat_sqrt_f(seed), bench_thr_sqrt_f(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_add_d, "double add", bench_lat_add_d(seed), bench_thr_add_d(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_sub_d, "double sub", bench_lat_sub_d(seed), bench_thr_sub_d(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_mul_d, "double mul", bench_lat_mul_d(seed), bench_thr_mul_d(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_fma_d, "double fma", bench_lat_fma_d(seed), bench_thr_fma_d(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_div_d, "double div", bench_lat_div_d(seed), bench_thr_div_d(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_sqrt_d, "double sqrt", bench_lat_sqrt_d(seed), bench_thr_sqrt_d(seed), overhead_per_trip,
               ns_per_cycle);

    ESP_LOGI(TAG, " [参考値: uint32 (soft/hardは未確定。数値だけ並べる)]");
    report_row(&r_mul_u32, "u32 mul", bench_lat_mul_u32(seed), bench_thr_mul_u32(seed), overhead_per_trip,
               ns_per_cycle);
    report_row(&r_div_u32, "u32 div", bench_lat_div_u32(seed), bench_thr_div_u32(seed), overhead_per_trip,
               ns_per_cycle);

    ESP_LOGI(TAG, "--- 1b. double/float 比 (レイテンシ基準) ---");
    ESP_LOGI(TAG, "  add: %.2fx   sub: %.2fx   mul: %.2fx   fma: %.2fx   div: %.2fx   sqrt: %.2fx",
             r_add_d.lat_cycles / r_add_f.lat_cycles, r_sub_d.lat_cycles / r_sub_f.lat_cycles,
             r_mul_d.lat_cycles / r_mul_f.lat_cycles, r_fma_d.lat_cycles / r_fma_f.lat_cycles,
             r_div_d.lat_cycles / r_div_f.lat_cycles, r_sqrt_d.lat_cycles / r_sqrt_f.lat_cycles);

    ESP_LOGI(TAG, "--- 1c. この結果から機械的に言えること ---");
    /* 「加減乗算+FMA はハード、除算・sqrt はソフト」という事前情報
     * (docs/PERF_ANALYSIS.md, docs/PLATFORM_TUNING.md)を、実測比率で
     * 裏付けられるかを閾値判定で機械的に出す。閾値は「大きな差」を
     * 大まかに切り分けるための目安であり、精密な統計検定ではない。 */
    double hw_ratio_avg = (r_add_d.lat_cycles / r_add_f.lat_cycles + r_mul_d.lat_cycles / r_mul_f.lat_cycles +
                            r_fma_d.lat_cycles / r_fma_f.lat_cycles) /
                           3.0;
    double sw_ratio_avg = (r_div_d.lat_cycles / r_div_f.lat_cycles + r_sqrt_d.lat_cycles / r_sqrt_f.lat_cycles) / 2.0;

    if (hw_ratio_avg > 3.0) {
        ESP_LOGI(TAG,
                 "  加減乗算+FMAのdouble/float比が%.1fxと大きい → EKFのP更新(加減乗算のみの経路)は"
                 "UWB_USE_FLOATで大きく高速化する見込み",
                 hw_ratio_avg);
    } else {
        ESP_LOGW(TAG, "  加減乗算+FMAのdouble/float比が%.1fxと小さい → 想定(ハード化で大差)と食い違う。要再確認",
                 hw_ratio_avg);
    }
    if (sw_ratio_avg < hw_ratio_avg) {
        ESP_LOGI(TAG,
                 "  除算/sqrtのdouble/float比(%.1fx)は加減乗算+FMA(%.1fx)より小さい → "
                 "Jacobi固有分解やuwb_evaluateのsqrtパスへのUWB_USE_FLOATの効果は加減乗算経路より限定的な見込み",
                 sw_ratio_avg, hw_ratio_avg);
    } else {
        ESP_LOGW(TAG, "  除算/sqrtのdouble/float比(%.1fx)が加減乗算+FMA(%.1fx)以上 → 想定と食い違う。要再確認",
                 sw_ratio_avg, hw_ratio_avg);
    }
}

/* ============================================================ 2. メモリアクセスベンチ
 *
 * 9x9行列(EKFの共分散P相当、double版648byte/float版324byte)への
 * read-modify-write を、連続アクセス(行方向、stride=sizeof(要素))と
 * ストライドアクセス(列方向、stride=9*sizeof(要素))で比較する。
 *
 * ここでは「メモリへのロード/ストアを本当に発生させること」自体が計測の
 * 目的なので、配列そのものを volatile にする(スカラー演算ベンチとは
 * 目的が逆であることに注意。ファイル冒頭のコメント参照)。 */

static volatile double g_mat_d[MAT_N][MAT_N];
static volatile float  g_mat_f[MAT_N][MAT_N];

static uint32_t bench_mem_contig_d(void)
{
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t rep = 0; rep < MEM_BENCH_REPEATS; ++rep) {
        for (int r = 0; r < MAT_N; ++r) {
            for (int c = 0; c < MAT_N; ++c) {
                g_mat_d[r][c] = g_mat_d[r][c] + 1.0;
            }
        }
    }
    return (uint32_t)esp_cpu_get_cycle_count() - t0;
}

static uint32_t bench_mem_stride_d(void)
{
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t rep = 0; rep < MEM_BENCH_REPEATS; ++rep) {
        for (int c = 0; c < MAT_N; ++c) {
            for (int r = 0; r < MAT_N; ++r) {
                g_mat_d[r][c] = g_mat_d[r][c] + 1.0;
            }
        }
    }
    return (uint32_t)esp_cpu_get_cycle_count() - t0;
}

static uint32_t bench_mem_contig_f(void)
{
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t rep = 0; rep < MEM_BENCH_REPEATS; ++rep) {
        for (int r = 0; r < MAT_N; ++r) {
            for (int c = 0; c < MAT_N; ++c) {
                g_mat_f[r][c] = g_mat_f[r][c] + 1.0f;
            }
        }
    }
    return (uint32_t)esp_cpu_get_cycle_count() - t0;
}

static uint32_t bench_mem_stride_f(void)
{
    uint32_t t0 = (uint32_t)esp_cpu_get_cycle_count();
    for (uint32_t rep = 0; rep < MEM_BENCH_REPEATS; ++rep) {
        for (int c = 0; c < MAT_N; ++c) {
            for (int r = 0; r < MAT_N; ++r) {
                g_mat_f[r][c] = g_mat_f[r][c] + 1.0f;
            }
        }
    }
    return (uint32_t)esp_cpu_get_cycle_count() - t0;
}

static void bench_memory(double overhead_per_trip, double ns_per_cycle)
{
    const uint32_t elements = MEM_BENCH_REPEATS * MAT_N * MAT_N;

    ESP_LOGI(TAG, "--- 2. メモリアクセスパターン (9x9行列, %u要素x%u周=%u要素アクセス) ---", MAT_N * MAT_N,
             MEM_BENCH_REPEATS, elements);
    ESP_LOGI(TAG,
             "  (注記: オーバーヘッドは1.の空ループ値を要素数で割った近似値を流用。"
             "9x9の二重ループぶん実際よりわずかに過小評価している可能性があるが誤差は僅少)");

    double contig_d = net_cycles_per_op(bench_mem_contig_d(), elements, elements, overhead_per_trip);
    vTaskDelay(pdMS_TO_TICKS(1));
    double stride_d = net_cycles_per_op(bench_mem_stride_d(), elements, elements, overhead_per_trip);
    vTaskDelay(pdMS_TO_TICKS(1));
    double contig_f = net_cycles_per_op(bench_mem_contig_f(), elements, elements, overhead_per_trip);
    vTaskDelay(pdMS_TO_TICKS(1));
    double stride_f = net_cycles_per_op(bench_mem_stride_f(), elements, elements, overhead_per_trip);
    vTaskDelay(pdMS_TO_TICKS(1));

    ESP_LOGI(TAG, "  double 648byte: 連続 %6.2f cyc/要素 (%6.1f ns)   ストライド72byte %6.2f cyc/要素 (%6.1f ns)   比 %.2fx",
             contig_d, contig_d * ns_per_cycle, stride_d, stride_d * ns_per_cycle, stride_d / contig_d);
    ESP_LOGI(TAG, "  float  324byte: 連続 %6.2f cyc/要素 (%6.1f ns)   ストライド36byte %6.2f cyc/要素 (%6.1f ns)   比 %.2fx",
             contig_f, contig_f * ns_per_cycle, stride_f, stride_f * ns_per_cycle, stride_f / contig_f);
}

/* ============================================================ エントリポイント */

void bench_run(void)
{
    uint32_t ticks_per_us = esp_rom_get_cpu_ticks_per_us();
    double   ns_per_cycle = 1000.0 / (double)ticks_per_us;

    ESP_LOGI(TAG, "=== 基本演算・メモリアクセス マイクロベンチ ===");
    ESP_LOGI(TAG, "CPU: %lu MHz (%.4f ns/cycle)  [esp_rom_get_cpu_ticks_per_us() より]",
             (unsigned long)ticks_per_us, ns_per_cycle);

    /* ループ制御オーバーヘッド(1トリップあたりのサイクル数)は
     * bench_scalar_ops() と bench_memory() の両方で使うので、ここで
     * 1回だけ測って両方へ同じ値を渡す。 */
    double overhead_per_trip = (double)bench_loop_overhead(bench_seed_now()) / (double)BENCH_ITERS;

    bench_scalar_ops(overhead_per_trip, ns_per_cycle);
    vTaskDelay(pdMS_TO_TICKS(1));
    bench_memory(overhead_per_trip, ns_per_cycle);

    ESP_LOGI(TAG, "=== マイクロベンチ完了 ===");
}
