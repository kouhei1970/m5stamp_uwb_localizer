/**
 * @file uwb_ranging_service.hpp
 * @brief タグの測距・測位を独立タスクとして走らせる uwb::RangingService
 * （docs/ARCHITECTURE_V2.md §3.1）。
 *
 * Runs ranging + positioning as a single dedicated FreeRTOS task, so a host
 * app (firmware/tag, and later a StampFly flight-control task) can consume
 * results through a mailbox/queue instead of driving the radio inline.
 *
 * ハード依存（uwb_ranging_scheduler.hpp 経由で uwb::Qm33120 / FreeRTOS を使う）
 * なので、uwb_ranging_anchor_table.hpp / uwb_ranging_pipeline.hpp と異なり
 * ホストビルド（tests/host/pipeline）のコンパイル対象には**含めない**
 * （Makefile が uwb_ranging_anchor_table.cpp / uwb_ranging_pipeline.cpp を
 * 名指しで列挙しているだけなので、本ファイルを src/ に置いても自動的に
 * 拾われることはない。それでも取り違えを防ぐため、本ファイル自体は
 * ESP-IDF 専用ヘッダにしか依存しないようにしてある）。
 *
 * ------------------------------------------------------------------
 * タスク・キュー構成（docs/ARCHITECTURE_V2.md §3.1〜§3.2）
 * ------------------------------------------------------------------
 * このクラスの start() が xTaskCreatePinnedToCore() で 1 本の「電波（チップ）
 * を触るタスク」を作る。そのタスクだけが RangingScheduler::runCycle()
 * （TWR フレームのやりとり）と PositioningPipeline::solve()/updateEkf()
 * （測位ソルバ・EKF）を呼ぶ。1 周期ごとに:
 *
 *   1. tableMutex() を取る（アンカー表の読み取り中はコンソールからの
 *      table.set()/update() を止める。詳しくは tableMutex() のコメント）
 *   2. runCycle() → solve(Lv0) → solve(Lv2)
 *   3. アンカーごとの成功率統計（scheduler の内部状態）をこの周期の
 *      スナップショットへコピーする（stats() 用。tableMutex 保持中に行う
 *      ことで scheduler 側の書き込みと競合しないようにする）
 *   4. tableMutex() を離す
 *   5. CycleResult を組み立て、latestQueue()（長さ1、xQueueOverwrite）へ
 *      上書き・resultQueue()（長さ4、満杯なら最古を捨てて再送）へ投入・
 *      getLatest() 用のメールボックスへコピー
 *   6. cycleIntervalMs > 0 なら vTaskDelayUntil() で周期を刻む
 *
 * **EKF（Lv3）は上の手順2にはもう出てこない。** enableEkf のときは
 * RangingScheduler::setSampleHook() で登録した onRangingSample() が、
 * runCycle() の内側・測距1本が成功するたびに（rangeOne() の直後、同じ
 * タスク文脈のまま同期的に）呼ばれ、その場で
 * PositioningPipeline::updateEkf(sample.t_us由来の秒, &sample, 1) を実行する
 * （測距1本ごとの逐次観測更新）。手順1でtableMutex()を取ってから手順2の
 * runCycle()を呼ぶので、onRangingSample()もその内側＝**tableMutex()保持中に
 * 実行される**（新しいロックは増やしていない。手順2の一部として同じ規律に
 * 収まる）。周期末尾では、その周期中に得た最後のEKF結果（lastEkfFix_）と
 * 逐次更新の合計所要時間（ekfUsThisCycle_）をCycleResult::lv3/solveUsLv3へ
 * 詰めるだけで、この専用タスクからのupdateEkf()呼び出し自体は手順2の中
 * （runCycle()の内側）で完結している。旧実装（周期末尾で全サンプルを
 * まとめて1回updateEkf()する方式）は廃止した — 同じ測距をuwb_ekfへ二重に
 * 入れないための変更（詳しくはonRangingSample()のコメント参照）。
 *
 * この処理タスクは **ホットループ中に ESP_LOG を一切呼ばない**（エラー
 * 経路を除く。docs/ARCHITECTURE_V2.md §1「電波を触るタスクは1つ」の精神を
 * 測位側にも適用したもの）。ログ・JSON出力は呼び出し側（firmware/tag の
 * uwb_log タスク等）の仕事。
 *
 * ------------------------------------------------------------------
 * tableMutex() の優先度逆転（実機で確認・修正済み）
 * ------------------------------------------------------------------
 * 上の手順4で離した tableMutex_ を、手順1（次の周期の先頭）で即座に
 * 取り直す。cycleIntervalMs==0（連続実行）だと、この「離す→即取り直す」の
 * 間隔は数マイクロ秒しかない。FreeRTOS のミューテックスは**待ち行列が
 * FIFO ではない**（xSemaphoreGive() で目覚めた待ち手が必ず次に取れる
 * 保証はなく、たまたま先に xSemaphoreTake() を呼んだ側が取る）ため、
 * core 1・優先度18で回り続けるこの処理タスクは、core 0 の低優先度タスク
 * （コンソール REPL 優先度2、uwb_net_wscmd 優先度4、uwb_net_tcp 優先度5）
 * を相手にほぼ確実に競り勝ち続け、それらは `xSemaphoreTake(tableMutex_,
 * portMAX_DELAY)` で**無期限にブロックする**（実機で `info` / `anchor
 * list` コマンドが固まる形で確認済み。ロガータスク（core 0・優先度10）は
 * resultQueue() への送信で同じ瞬間に起こされるため大抵勝ててしまい、
 * JSON 出力自体は流れ続けるので気づきにくい）。
 *
 * 対策として `tableWaiters_`（std::atomic<uint32_t>）で「今 tableMutex_
 * 待ちの他タスクが何本いるか」を数え、`lockTable()`/`unlockTable()` を
 * 経由するすべての取得・解放でこれを増減させる。処理タスクは手順4で
 * tableMutex_ を離した直後に `tableWaiters_` を見て、非ゼロなら
 * `vTaskDelay(1)` を挟む（CONFIG_FREERTOS_HZ=1000 の下で最低1ms、
 * ノーウェイト時のコストはゼロ）。これで手順1の再取得の前に他コアの
 * 待ち手がスケジューラに拾われる猶予ができ、飢餓を避けられる。
 * コンソールから編集する側（firmware/tag/main/tag_console.cpp）や
 * resetStats()/reinitEkf() は、素の `tableMutex()` ではなく必ず
 * `lockTable()`/`unlockTable()` を使うこと。ロガータスク（firmware/tag/
 * main/main.cpp の uwb_log）は resultQueue() の受信で毎周期起こされ、
 * この処理タスクが手順1へ戻る前に大抵先着できるため飢餓のリスクが低く、
 * 素の `tableMutex()` のままにしてある（変更不要）。
 *
 * Priority inversion on tableMutex_ (confirmed and fixed on real hardware):
 * step 4 releases tableMutex_ and step 1 of the very next cycle re-acquires
 * it, only microseconds apart when cycleIntervalMs==0. FreeRTOS mutexes are
 * NOT FIFO-fair - whichever task calls xSemaphoreTake() first wins, not
 * whichever was woken first - so this task (core 1, priority 18, looping
 * tightly) reliably out-races the low-priority waiters on core 0 (console
 * REPL prio 2, uwb_net_wscmd prio 4, uwb_net_tcp prio 5), which then block
 * on xSemaphoreTake(tableMutex_, portMAX_DELAY) forever (reproduced on
 * hardware: `info` / `anchor list` hang). The logger task (core 0, prio 10)
 * mostly wins the race because it's woken by the same resultQueue() send
 * that follows the give, which is why JSON output keeps flowing and masks
 * the starvation. Fix: `tableWaiters_` (std::atomic<uint32_t>) counts
 * outstanding waiters; every acquire/release through `lockTable()`/
 * `unlockTable()` updates it. Right after releasing tableMutex_ in step 4,
 * the task checks tableWaiters_ and, if nonzero, calls vTaskDelay(1)
 * (>=1 ms at CONFIG_FREERTOS_HZ=1000, free when nobody is waiting) so a
 * waiter on the other core gets a chance to run before step 1 re-acquires.
 * Console edits and resetStats()/reinitEkf() must go through lockTable()/
 * unlockTable(), not the raw tableMutex(). The logger task keeps using the
 * raw tableMutex() unchanged - it isn't at starvation risk for the reason
 * above.
 *
 * Task/queue layout: one dedicated task owns the radio + solver every
 * cycle; results are handed off via a length-4 "log" queue (drop-oldest on
 * overflow) and a length-1 "latest" queue (xQueueOverwrite) plus a
 * mutex-guarded mailbox for getLatest(). The task never logs except on
 * error - logging is the consumer's job (see the uwb_log task in
 * firmware/tag/main/main.cpp).
 */
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "uwb_qm33120.hpp"
#include "uwb_ranging_anchor_table.hpp"
#include "uwb_ranging_pipeline.hpp"
#include "uwb_ranging_scheduler.hpp"
#include "uwb_ranging_types.hpp"

namespace uwb {

/**
 * @brief 1 周期ぶんの測距・測位結果。ポインタを持たない単純なコピー可能構造体
 * （docs/ARCHITECTURE_V2.md §3.1 の指定どおり）。キュー（値渡し）で運ぶための形。
 */
struct CycleResult {
    /** 周期番号。1 から単調増加する（0 は「まだ1周も終わっていない」の意味。
     *  getLatest() の呼び出し側はこの値の変化で新しい周期が来たかを判定する
     *  想定 — ARCHITECTURE_V2.md 原文の "seq で更新判定" の意味）。 */
    uint32_t seq = 0;

    /** 周期開始時刻。esp_timer_get_time() の生値（RangingSample::t_us と同じ
     *  基準）。起動からの経過秒に直すには呼び出し側で基準時刻を引くこと
     *  （firmware/tag/main/main.cpp の bootUs と同じ考え方）。 */
    int64_t tUs = 0;

    /** この周期の測距ポーリングに要した時間 [ms]（RangingScheduler::lastCycleMs()）。 */
    uint32_t cycleMs = 0;

    /** samples の有効数（0 〜 kMaxAnchors）。 */
    size_t n = 0;

    /** 各アンカーへの測距結果。samples[0..n) だけが意味を持つ。 */
    RangingSample samples[kMaxAnchors];

    PositionResult lv0; //!< 比較用の簡易ソルバ（閉形式三辺測量）
    PositionResult lv2; //!< 本番ソルバ（Beck厳密解 + Huberロバスト化）

    bool haveLv3 = false; //!< true なら lv3 が意味を持つ（ServiceConfig::enableEkf）
    PositionResult lv3;    //!< EKF（Lv3）の結果。haveLv3==false のときは未使用

    uint32_t solveUsLv0 = 0; //!< 各ソルバの計算時間 [us]（esp_timer_get_time() の差分）
    uint32_t solveUsLv2 = 0;
    uint32_t solveUsLv3 = 0; //!< haveLv3==false のときは 0
};

/**
 * @brief RangingService の動作設定（docs/ARCHITECTURE_V2.md §3.1）。
 */
struct ServiceConfig {
    SchedulerConfig scheduler; //!< 方式・再試行・アンカー間隔（後述の cycleIntervalMs との
                                //!< 二重刻み注意点は下のフィールドコメント参照）

    /**
     * NOTE(deviation): ARCHITECTURE_V2.md §3.1 はこのフィールドの型を
     * `PipelineConfig` と書いているが、uwb_ranging_pipeline.hpp が実際に
     * 定義している型は `PositioningConfig`（uwb_ranging_types.hpp）。
     * アーキ文書の型名の書き誤りとみなし、実在する型をそのまま使う。
     *
     * The architecture doc names this field's type `PipelineConfig`, but
     * the real type in uwb_ranging_pipeline.hpp is `PositioningConfig`.
     * Treated as a naming slip in the doc; using the real type.
     */
    PositioningConfig pipeline;

    /**
     * Lv3（EKF、拡張カルマンフィルタ）も毎周期呼ぶか。
     *
     * ARCHITECTURE_V2.md §3.1 の CycleResult は lv3/haveLv3/solveUsLv3 を
     * 直接持っている＝EKF の呼び出しはこのサービスの内部（1本の処理タスク）
     * で完結させる設計だと読める。uwb_ekf はスレッド安全でないため
     * （uwb_ranging_pipeline.hpp の PositioningPipeline 冒頭コメント）、
     * 呼び出し元は必ずこのタスク1本に統一する必要があり、ロガー側
     * （消費側タスク）で EKF を呼ぶ選択肢は取れない。よってこのサービスが
     * PositioningPipeline を所有し、その内部で initEkf()/updateEkf() を
     * 呼ぶ構成にした（firmware/tag の CONFIG_UWB_TAG_ENABLE_EKF はこの
     * フラグへ変換されるだけで、挙動は従来と同じ）。
     *
     * The doc's CycleResult already carries lv3/haveLv3/solveUsLv3, which
     * implies EKF must be evaluated inside this service's single task
     * (uwb_ekf is not thread-safe - see PositioningPipeline's header
     * comment - so it cannot be called from a separate consumer task).
     * CONFIG_UWB_TAG_ENABLE_EKF in firmware/tag just becomes this flag;
     * behavior is unchanged from before.
     */
    bool enableEkf = false;

    /**
     * 1周期の最小間隔 [ms]。0 なら連続実行（前の周期が終わり次第すぐ次へ）。
     * vTaskDelayUntil() で刻む。
     *
     * **二重刻みに注意**: scheduler.cycleIntervalMs にも同じ役割のフィールドが
     * ある（RangingScheduler が runCycle() の先頭で待つ）。両方を同時に
     * 使うと待ち時間が二重にかかる。本サービスを使う場合は
     * **scheduler.cycleIntervalMs は 0 のままにし、周期の刻みはこちらの
     * cycleIntervalMs だけで行うこと**（firmware/tag/main/main.cpp で
     * 実際にそうしている。start() 自身は cfg.scheduler.cycleIntervalMs を
     * 強制的に書き換えたりはしない — 呼び出し側の設定をそのまま尊重する
     * 方針なので、二重設定に気づけるよう start() が両方非ゼロなら1回だけ
     * ESP_LOGW を出す）。
     *
     * Avoid double pacing: SchedulerConfig has its own cycleIntervalMs
     * (RangingScheduler waits on it at the top of runCycle()). Using both
     * doubles the wait. When using this service, leave
     * scheduler.cycleIntervalMs at 0 and pace only via this field
     * (firmware/tag/main/main.cpp does this). start() does not silently
     * override scheduler.cycleIntervalMs - it logs a one-time warning if
     * both are non-zero instead, since respecting what the caller
     * explicitly configured is more predictable than magic overrides.
     */
    uint32_t cycleIntervalMs = 0;

    int core        = 1;    //!< xTaskCreatePinnedToCore() のコア番号
    int prio         = 18;   //!< タスク優先度
    uint32_t stack   = 8192; //!< タスクスタックサイズ [bytes]
};

/**
 * @brief 測距・測位を1本の専用タスクで回すサービス（docs/ARCHITECTURE_V2.md §3.1）。
 *
 * start() が渡された Qm33120（電波）と AnchorTable（アンカー登録テーブル。
 * 呼び出し側が所有し続ける生きた参照）を使って内部に RangingScheduler と
 * PositioningPipeline を作り、専用タスクで周期的に回す。
 *
 * AnchorTable はコンソール等から実行時に編集され得るので、編集する側は
 * 必ず tableMutex() を取ってから table.set()/update() を呼ぶこと
 * （tableMutex() のコメント参照）。
 */
class RangingService {
public:
    RangingService()  = default;
    ~RangingService();
    RangingService(const RangingService&)            = delete;
    RangingService& operator=(const RangingService&) = delete;

    /**
     * @brief 専用タスクを起こし、測距・測位を開始する。
     *
     * @param radio 電波（生きている間ずっと有効なオブジェクトへの参照。
     *              呼び出し側が begin() 済みであること）
     * @param table アンカー登録テーブル（同上。以後この参照を通じて
     *              読み書きされる。コンソールから編集する場合は
     *              tableMutex() を取ってから触ること）
     * @return タスク・キュー・ミューテックスの生成にすべて成功したら true。
     *         二重 start() も false を返す（先に stop() すること）。
     */
    bool start(Qm33120& radio, AnchorTable& table, const ServiceConfig& cfg);

    /**
     * @brief タスクを止め、キュー・ミューテックスを解放する。
     *
     * 現在実行中の1周期が終わるまで待ってから止める（無線交換の途中で
     * 強制終了してチップを半端な状態に残すことを避けるため）。
     */
    void stop();

    /**
     * @brief 最新の周期結果をコピーで取り出す。
     *
     * 呼び出し側は前回取得した out.seq と比較して、新しい周期が来たかを
     * 判定する想定（ARCHITECTURE_V2.md §3.1 原文の「seq で更新判定」）。
     * start() 前 / stop() 後は false を返し、out は変更しない。
     *
     * @return start() 済みなら true（まだ1周も終わっていなくても、
     *         out.seq==0 の初期値がコピーされて true を返す）。
     */
    bool getLatest(CycleResult& out) const;

    /** 長さ4のキュー（値は CycleResult）。ログ用途。満杯なら最古の1件を
     *  捨てて投入し直す（drop-oldest。処理タスクは絶対にブロックしない）。
     *  start() 前は nullptr。 */
    QueueHandle_t resultQueue() const { return resultQueue_; }

    /** 長さ1のキュー（xQueueOverwrite で常に最新値だけを保持）。制御ループ用途。
     *  start() 前は nullptr。 */
    QueueHandle_t latestQueue() const { return latestQueue_; }

    /**
     * @brief anchorIndex 番目のアンカーの測距成功率統計のコピーを返す。
     *
     * NOTE(deviation): ARCHITECTURE_V2.md §3.1 の宣言は
     * `const AnchorStats& stats(size_t i) const;`（参照）だが、同じ行の
     * コメントは「コピーを返す」と書いてあり矛盾している。処理タスクが
     * 毎周期この統計を書き換えている以上、別タスクから参照をそのまま返すと
     * データ競合（複数フィールドの torn read）になるため、コメントの意図
     * （コピー）を採用し、戻り値を値渡し（AnchorStats）にした。
     *
     * The doc's declaration returns a reference, but its own comment says
     * "returns a copy" - contradictory. Since the service task mutates
     * this every cycle, handing out a live reference to another task would
     * be a data race (torn multi-field reads). Took the comment's intent
     * (copy) and return by value instead.
     */
    AnchorStats stats(size_t anchorIndex) const;

    /**
     * @brief 全アンカーの統計をゼロに戻す。
     *
     * 内部で tableMutex() を取る（処理タスクの周期の合間にだけリセットが
     * 起きるようにするため）。**呼び出し側が既に tableMutex() を保持した
     * 状態でこれを呼ぶとデッドロックする**（通常の mutex は再帰不可）。
     * アンカー表を編集するコンソールは、tableMutex() を離してから
     * resetStats() を呼ぶこと（firmware/tag/main/tag_console.cpp 参照）。
     */
    void resetStats();

    /**
     * @brief EKF（Lv3）の内部状態を組み直す。
     *
     * NOTE(addition): ARCHITECTURE_V2.md §3.1 のメンバ一覧には無いメソッド。
     * EKF がこのサービスの内部（PositioningPipeline）にカプセル化された
     * ことで、アンカー表の構成（台数・座標）が変わったときに観測モデルを
     * 組み直す処理（旧 main.cpp が `pipeline.initEkf()` を再度呼んでいた
     * 箇所）を呼び出し側から行う手段が無くなった。そのための最小限の
     * 追加 API。ServiceConfig::enableEkf==false のときは no-op。
     * resetStats() 同様、内部で tableMutex() を取るので、呼び出し側は
     * tableMutex() を離してから呼ぶこと。
     *
     * Not in ARCHITECTURE_V2.md's member list. Needed so a caller that
     * just changed the anchor table's structure can ask the service to
     * rebuild its EKF's internal state, mirroring what main.cpp used to do
     * inline before the EKF moved inside this service. No-op when
     * ServiceConfig::enableEkf is false. Takes tableMutex() internally, so
     * release it first if you're already holding it.
     */
    void reinitEkf();

    /**
     * @brief アンカー登録テーブルを保護するミューテックス。
     *
     * **処理タスクは1周期の間（runCycle() の開始から solve() の終了まで、
     * enableEkf のときはさらに逐次EKF更新も含めて）ずっとこれを保持する。**
     * AnchorTable::set() は「ソルバから読まれている最中に呼んではいけない」
     * （uwb_ranging_anchor_table.hpp の同クラス冒頭コメント）ため、周期の
     * 一部だけ保持する方式だと table.set() との競合を防げない。コンソール等
     * がアンカー表を編集するときは、必ずこれを取ってから table.set()/
     * update() を呼び、編集が終わったら離すこと（tag_console.cpp 参照）。
     *
     * **逐次EKF更新のロック規律**: enableEkf のときに呼ばれる
     * onRangingSample()（RangingScheduler::setSampleHook() で登録）は
     * runCycle() の内側から同期的に呼ばれるので、新しいロックを増やさず
     * このtableMutex_の保持区間にそのまま収まる（table_->modeDecision() の
     * 参照も pipeline_->updateEkf() の呼び出しも、他タスクによる
     * table.set()/update() と競合しない。uwb_ranging_service.hpp 冒頭の
     * タスク・キュー構成コメント参照）。
     *
     * この設計の代償: table.set()/update() を伴う編集コマンドは、
     * 実行中の1周期（DS-TWR・再試行込みで最大で数百ms程度）ぶん待たされる
     * ことがある。設定作業はホットパスではないため許容している。
     *
     * **このハンドルへ直接 xSemaphoreTake()/xSemaphoreGive() するのは
     * ロガータスク（firmware/tag/main/main.cpp の uwb_log）専用と
     * 考えること。** それ以外の呼び出し側（コンソール、resetStats()、
     * reinitEkf()）は下の lockTable()/unlockTable() を使う — 素の
     * ハンドルのままだと、このサービス処理タスクが手放した直後に
     * 自分自身へ取り直してしまい、低優先度の待ち手が無期限に飢餓する
     * （本ヘッダ冒頭「tableMutex() の優先度逆転」参照。実機で確認・
     * 修正済み）。ロガータスクは resultQueue() の受信で処理タスクの
     * 各周期の直後に起こされ、この飢餓の対象にならないため素のハンドル
     * のままにしてある。
     *
     * The service task holds this for the ENTIRE cycle (runCycle() through
     * solve()/updateEkf()), not just briefly at the head, because
     * AnchorTable::set() must not run concurrently with a solve reading
     * the table. A console editing the table must take this before
     * calling table.set()/update() and release it afterward. Trade-off:
     * table edits can block for up to one in-flight cycle (worst case a
     * few hundred ms with DS-TWR + retries) - acceptable since console
     * edits are not on the hot path.
     *
     * **Treat direct xSemaphoreTake()/xSemaphoreGive() on this raw handle
     * as reserved for the logger task** (uwb_log in firmware/tag/main/
     * main.cpp). Every other caller (console, resetStats(), reinitEkf())
     * must use lockTable()/unlockTable() below - using the raw handle lets
     * this service's own task re-acquire it right after releasing it,
     * starving low-priority waiters indefinitely (see "priority inversion
     * on tableMutex_" at the top of this header; confirmed and fixed on
     * hardware). The logger task is woken by resultQueue() right after
     * each cycle, so it isn't exposed to that starvation and can keep
     * using the raw handle.
     *
     * start() 前は nullptr。
     */
    SemaphoreHandle_t tableMutex() const { return tableMutex_; }

    /**
     * @brief tableMutex() を、飢餓を避けるプロトコル付きで取る。
     *
     * `tableWaiters_` を先にインクリメントしてから
     * `xSemaphoreTake(tableMutex_, portMAX_DELAY)` し、取れたら
     * デクリメントする。処理タスクは周期の終わりに tableMutex_ を
     * 離した直後、`tableWaiters_` が非ゼロなら `vTaskDelay(1)` を挟んで
     * 他コアの待ち手にスケジューラの隙を空ける（本ヘッダ冒頭の
     * 「tableMutex() の優先度逆転」参照）。**コンソール・resetStats()・
     * reinitEkf() など、処理タスク以外からアンカー表を編集・参照する側は
     * 必ずこちらを使うこと**（素の tableMutex() を直接 take しない）。
     *
     * Takes tableMutex() with the starvation-avoidance protocol: increments
     * `tableWaiters_` before xSemaphoreTake(tableMutex_, portMAX_DELAY),
     * decrements it once acquired. The service task checks tableWaiters_
     * right after releasing tableMutex_ at the end of each cycle and yields
     * via vTaskDelay(1) when nonzero, giving a waiter on the other core a
     * chance to run (see "priority inversion on tableMutex_" above). Every
     * caller other than the service task itself (console, resetStats(),
     * reinitEkf()) must use this instead of taking the raw tableMutex().
     *
     * start() 前 / stop() 後に呼ぶと tableMutex_==nullptr のまま
     * xSemaphoreTake(nullptr, ...) を呼ぶことになる点は tableMutex() 同様
     * （呼び出し側が start() 済みであることを保証すること）。
     */
    void lockTable();

    /**
     * @brief lockTable() で取ったロックを離す。
     *
     * 中身は `xSemaphoreGive(tableMutex_)` のみ（tableWaiters_ は
     * lockTable() 側で既にデクリメント済み）。lockTable() と必ず対で
     * 呼ぶこと。
     *
     * Releases the lock taken by lockTable() (plain
     * xSemaphoreGive(tableMutex_); tableWaiters_ was already decremented
     * inside lockTable()). Always pair with lockTable().
     */
    void unlockTable();

    /** 処理タスクのハンドル。
     *
     * NOTE(addition): ARCHITECTURE_V2.md §3.1 のメンバ一覧には無い。
     * firmware/tag/main/main.cpp の起動後5秒でのスタック残量ログ
     * （uxTaskGetStackHighWaterMark()）に外側から渡す先が必要だったための
     * 追加。start() 前 / stop() 後は nullptr。
     *
     * Not in the doc's member list. Added so main.cpp's "5 seconds after
     * boot" stack high-water-mark log (uxTaskGetStackHighWaterMark()) has
     * something to pass in from outside. nullptr before start() / after
     * stop(). */
    TaskHandle_t taskHandle() const { return taskHandle_; }

private:
    static void taskTrampoline(void* arg);
    void taskMain();

    /**
     * @brief RangingScheduler::setSampleHook() へ渡すトランポリン。
     *
     * uwb::RangingScheduler は uwb::RangingService（本クラス）を知らない
     * （ハード依存/ハード非依存の分離を保つため）ので、C形式の関数ポインタ
     * から this を復元して onRangingSample() へ渡すだけの薄い橋渡し。
     * taskTrampoline() と同じ作法。
     */
    static void sampleHookTrampoline(const RangingSample& sample, void* user);

    /**
     * @brief 測距1本ごとの逐次EKF観測更新（本体）。
     *
     * RangingScheduler::runCycle() の内側、rangeOne() が成功を返した直後に
     * **同じタスク文脈で同期的に**呼ばれる（taskMain() が tableMutex_ を
     * 保持したまま runCycle() を呼んでいる区間の内側 = tableMutex_ 保持中。
     * uwb_ranging_service.hpp 冒頭のタスク・キュー構成コメントと
     * tableMutex() のコメント参照）。
     *
     * enableEkf==false、または現在の測位モードが RangingOnly
     * （有効アンカー2台以下）のときは何もしない（78d7cfa で入った測位モード
     * 自動切替の規律をここでも守る。taskMain() の rangingOnly 判定と
     * 同じ条件）。
     *
     * サンプル1本ぶんだけを渡す（updateEkf(tS, &sample, 1)）ことが本改修の
     * 要— サンプルごとの実際の測距時刻（RangingSample::t_us）で処理される
     * ようにするため、周期末尾でまとめて渡す旧実装を廃止した。周期末尾で
     * まとめて呼ぶ旧実装と違い、**同じ測距を二重にEKFへ入れないよう**、
     * taskMain() 側は周期末尾で改めて updateEkf() を呼ばない（このメソッドの
     * 呼び出しだけが唯一の入口になる）。結果は lastEkfFix_ に保持し、
     * ekfUpdatedThisCycle_/ekfUsThisCycle_ で「この周期に更新があったか」
     * 「逐次更新の合計所要時間」を taskMain() へ伝える（taskMain() が
     * runCycle() 呼び出し前にこの3つをリセットする）。
     */
    void onRangingSample(const RangingSample& sample);

    Qm33120* radio_       = nullptr;
    AnchorTable* table_    = nullptr;
    ServiceConfig cfg_;

    // PImpl 的に heap へ置く（開始時にしかサイズが決まらない上、
    // components/uwb_qm33120/src/uwb_qm33120.cpp の Impl と同じ流儀
    // - new/delete による所有）。
    RangingScheduler* scheduler_  = nullptr;
    PositioningPipeline* pipeline_ = nullptr;

    /** 逐次EKF更新（onRangingSample()）の状態。処理タスクからしか
     *  触らない（他タスクと共有しないので排他不要。CycleResult へ詰める
     *  ためだけの一時的な集計置き場）。
     *
     *  lastEkfFix_: 直近に成功したupdateEkf(...,1)の結果。この周期に1本も
     *  成功しなかった場合、taskMain()は前の周期のこの値をそのまま使い、
     *  ok=falseだけ上書きして返す（Lv0/Lv2が欠測時にsolvable=false,ok=false
     *  を返すのと意味論を揃えるため。onRangingSample()と対になるコメント
     *  参照）。
     *  ekfUpdatedThisCycle_/ekfUsThisCycle_: 今の周期でonRangingSample()が
     *  実際にupdateEkf()を呼んだか、その合計所要時間[us]。taskMain()が
     *  runCycle()を呼ぶ直前に両方リセットする。 */
    PositionResult lastEkfFix_{};
    bool ekfUpdatedThisCycle_ = false;
    uint32_t ekfUsThisCycle_    = 0;

    TaskHandle_t taskHandle_ = nullptr;
    volatile bool running_    = false;

    QueueHandle_t resultQueue_ = nullptr; //!< 長さ4、drop-oldest
    QueueHandle_t latestQueue_ = nullptr; //!< 長さ1、xQueueOverwrite

    SemaphoreHandle_t tableMutex_  = nullptr; //!< AnchorTable 保護（tableMutex() 参照）

    /** lockTable() 待ち（xSemaphoreTake() 呼び出し中）の他タスク本数。
     *  処理タスクが周期の終わりに tableMutex_ を離した直後にこれを見て、
     *  非ゼロなら vTaskDelay(1) して他コアの待ち手へスケジューラの隙を
     *  空ける（tableMutex() の優先度逆転対策。本ヘッダ冒頭コメント参照）。
     *
     *  Count of other tasks currently blocked inside lockTable()'s
     *  xSemaphoreTake(). The service task checks this right after
     *  releasing tableMutex_ at the end of each cycle and yields via
     *  vTaskDelay(1) when nonzero (starvation fix - see header comment). */
    std::atomic<uint32_t> tableWaiters_{0};

    SemaphoreHandle_t latestMutex_ = nullptr; //!< latest_ メールボックス保護
    SemaphoreHandle_t statsMutex_  = nullptr; //!< statsSnapshot_ 保護
    SemaphoreHandle_t stoppedSem_  = nullptr; //!< タスク終了通知（stop() が待つ）

    CycleResult latest_{};                        //!< getLatest() 用メールボックス
    AnchorStats statsSnapshot_[kMaxAnchors]{};      //!< stats() 用スナップショット
};

} // namespace uwb
