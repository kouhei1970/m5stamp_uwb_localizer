/**
 * @file bench.h
 * @brief 基本演算・メモリアクセスパターンのマイクロベンチマーク(サイクル計測)。
 *
 * docs/PERF_ANALYSIS.md の追記2・追記3、docs/PLATFORM_TUNING.md の指摘を受けて、
 * 実機到着後すぐに「float と double、加減乗除・積和(FMA)・sqrt それぞれの
 * 実コストが何サイクルか」「9x9行列(EKFの共分散P相当)への連続アクセスと
 * ストライドアクセスの差」を取れるようにしたもの。合成データのソルバ計測
 * (main.c の scenario_timing 等)とは独立に、より低レベルの基本演算そのものを見る。
 *
 * 計測方法(オーバーヘッド差し引き、依存チェイン(レイテンシ)と独立演算の
 * 並び(スループット)の作り分け、最適化で消されないようにする仕掛け)の
 * 詳細は bench.c 冒頭のコメントを参照。
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief マイクロベンチマーク一式を実行して結果を ESP_LOGI で出力する。
 *
 * 内訳:
 *   1. 基本演算(float/double の加減乗除・積和・sqrt、参考として uint32 の乗除算)の
 *      レイテンシ(依存チェイン)とスループット(8本の独立アキュムレータ)を
 *      サイクル数と ns の両方で出す。ハードウェア命令になる演算群
 *      (float の加減乗算・積和)とソフトウェア実装になる演算群
 *      (float の除算・sqrt、double の全演算)を分けて表示する。
 *   2. double/float の比率表と、そこから機械的に導ける結論
 *      (UWB_USE_FLOAT がどの経路に効きやすいか)。
 *   3. 9x9行列(EKFの共分散P相当)への連続アクセス/ストライドアクセスの
 *      要素あたりコスト(double版・float版の両方)。
 *
 * 数十万回のループを何十本も回すため実行に数秒かかる。呼び出し元は
 * ESP_TASK_WDT に引っかからないよう配慮すること(このファイルの実装は
 * 各計測の合間に vTaskDelay を挟んでアイドルタスクへ制御を返す。
 * ただし esp_cpu_get_cycle_count() はコアごとに独立したカウンタなので、
 * 呼び出し元のタスクは特定のコアにピン留めしておくこと。
 * firmware/soltest/main/main.c の xTaskCreatePinnedToCore 呼び出し参照)。
 */
void bench_run(void);

#ifdef __cplusplus
}
#endif
