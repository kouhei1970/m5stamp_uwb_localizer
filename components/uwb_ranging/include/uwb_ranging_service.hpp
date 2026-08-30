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
 *   2. runCycle() → solve(Lv0) → solve(Lv2) → (enableEkf なら) updateEkf()
 *   3. アンカーごとの成功率統計（scheduler の内部状態）をこの周期の
 *      スナップショットへコピーする（stats() 用。tableMutex 保持中に行う
 *      ことで scheduler 側の書き込みと競合しないようにする）
 *   4. tableMutex() を離す
 *   5. CycleResult を組み立て、latestQueue()（長さ1、xQueueOverwrite）へ
 *      上書き・resultQueue()（長さ4、満杯なら最古を捨てて再送）へ投入・
 *      getLatest() 用のメールボックスへコピー
 *   6. cycleIntervalMs > 0 なら vTaskDelayUntil() で周期を刻む
 *
 * この処理タスクは **ホットループ中に ESP_LOG を一切呼ばない**（エラー
 * 経路を除く。docs/ARCHITECTURE_V2.md §1「電波を触るタスクは1つ」の精神を
 * 測位側にも適用したもの）。ログ・JSON出力は呼び出し側（firmware/tag の
 * uwb_log タスク等）の仕事。
 *
 * Task/queue layout: one dedicated task owns the radio + solver every
 * cycle; results are handed off via a length-4 "log" queue (drop-oldest on
 * overflow) and a length-1 "latest" queue (xQueueOverwrite) plus a
 * mutex-guarded mailbox for getLatest(). The task never logs except on
 * error - logging is the consumer's job (see the uwb_log task in
 * firmware/tag/main/main.cpp).
 */
#pragma once

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
     * **処理タスクは1周期の間（runCycle() の開始から solve()/updateEkf()
     * の終了まで）ずっとこれを保持する。** AnchorTable::set() は「ソルバから
     * 読まれている最中に呼んではいけない」（uwb_ranging_anchor_table.hpp
     * の同クラス冒頭コメント）ため、周期の一部だけ保持する方式だと
     * table.set() との競合を防げない。コンソール等がアンカー表を編集する
     * ときは、必ずこれを取ってから table.set()/update() を呼び、
     * 編集が終わったら離すこと（tag_console.cpp 参照）。
     *
     * この設計の代償: table.set()/update() を伴う編集コマンドは、
     * 実行中の1周期（DS-TWR・再試行込みで最大で数百ms程度）ぶん待たされる
     * ことがある。設定作業はホットパスではないため許容している。
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
     * start() 前は nullptr。
     */
    SemaphoreHandle_t tableMutex() const { return tableMutex_; }

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

    Qm33120* radio_       = nullptr;
    AnchorTable* table_    = nullptr;
    ServiceConfig cfg_;

    // PImpl 的に heap へ置く（開始時にしかサイズが決まらない上、
    // components/uwb_qm33120/src/uwb_qm33120.cpp の Impl と同じ流儀
    // - new/delete による所有）。
    RangingScheduler* scheduler_  = nullptr;
    PositioningPipeline* pipeline_ = nullptr;

    TaskHandle_t taskHandle_ = nullptr;
    volatile bool running_    = false;

    QueueHandle_t resultQueue_ = nullptr; //!< 長さ4、drop-oldest
    QueueHandle_t latestQueue_ = nullptr; //!< 長さ1、xQueueOverwrite

    SemaphoreHandle_t tableMutex_  = nullptr; //!< AnchorTable 保護（tableMutex() 参照）
    SemaphoreHandle_t latestMutex_ = nullptr; //!< latest_ メールボックス保護
    SemaphoreHandle_t statsMutex_  = nullptr; //!< statsSnapshot_ 保護
    SemaphoreHandle_t stoppedSem_  = nullptr; //!< タスク終了通知（stop() が待つ）

    CycleResult latest_{};                        //!< getLatest() 用メールボックス
    AnchorStats statsSnapshot_[kMaxAnchors]{};      //!< stats() 用スナップショット
};

} // namespace uwb
