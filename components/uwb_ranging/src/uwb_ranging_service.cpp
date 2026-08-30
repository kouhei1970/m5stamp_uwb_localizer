/**
 * @file uwb_ranging_service.cpp
 * @brief uwb::RangingService の実装（ハード依存、ESP-IDF/FreeRTOS を使う）。
 */
#include "uwb_ranging_service.hpp"

#include "esp_log.h"
#include "esp_timer.h"

namespace uwb {

namespace {
const char* kLogTag = "uwb_ranging_svc";
} // namespace

RangingService::~RangingService()
{
    stop();
}

bool RangingService::start(Qm33120& radio, AnchorTable& table, const ServiceConfig& cfg)
{
    if (taskHandle_ != nullptr) {
        // 二重 start()。先に stop() すること。
        ESP_LOGE(kLogTag, "start() called while already running");
        return false;
    }

    radio_ = &radio;
    table_  = &table;
    cfg_    = cfg;

    // 二重刻みの取り違えに気づけるよう、両方のペーシングが同時に有効なら
    // 1回だけ警告する（値は自動で変えない。ServiceConfig::cycleIntervalMs
    // のフィールドコメント参照）。
    if (cfg_.cycleIntervalMs > 0 && cfg_.scheduler.cycleIntervalMs > 0) {
        ESP_LOGW(kLogTag,
                 "ServiceConfig::cycleIntervalMs(%lu) と scheduler.cycleIntervalMs(%lu) が"
                 "両方非ゼロです。周期が二重に刻まれます。scheduler.cycleIntervalMs は0にしてください",
                 static_cast<unsigned long>(cfg_.cycleIntervalMs),
                 static_cast<unsigned long>(cfg_.scheduler.cycleIntervalMs));
    }

    tableMutex_   = xSemaphoreCreateMutex();
    latestMutex_  = xSemaphoreCreateMutex();
    statsMutex_   = xSemaphoreCreateMutex();
    stoppedSem_    = xSemaphoreCreateBinary();
    resultQueue_  = xQueueCreate(4, sizeof(CycleResult));
    latestQueue_  = xQueueCreate(1, sizeof(CycleResult));

    if (tableMutex_ == nullptr || latestMutex_ == nullptr || statsMutex_ == nullptr ||
        stoppedSem_ == nullptr || resultQueue_ == nullptr || latestQueue_ == nullptr) {
        ESP_LOGE(kLogTag, "failed to allocate mutex/queue objects");
        // running_ はまだ false のまま。stop() は生成済み分だけ後始末する。
        stop();
        return false;
    }

    running_ = true;
    const BaseType_t ok = xTaskCreatePinnedToCore(&RangingService::taskTrampoline, "uwb_ranging_svc",
                                                   cfg_.stack, this, cfg_.prio, &taskHandle_, cfg_.core);
    if (ok != pdPASS) {
        ESP_LOGE(kLogTag, "xTaskCreatePinnedToCore() failed");
        running_    = false;
        taskHandle_ = nullptr;
        stop();
        return false;
    }
    return true;
}

void RangingService::stop()
{
    if (taskHandle_ != nullptr) {
        running_ = false;
        // 実行中の1周期が終わって処理タスクが自ら終了するまで待つ
        // （無線交換の途中で強制終了しない。taskMain() 末尾で give される）。
        if (stoppedSem_ != nullptr) {
            xSemaphoreTake(stoppedSem_, portMAX_DELAY);
        }
        taskHandle_ = nullptr;
    }

    if (resultQueue_ != nullptr) {
        vQueueDelete(resultQueue_);
        resultQueue_ = nullptr;
    }
    if (latestQueue_ != nullptr) {
        vQueueDelete(latestQueue_);
        latestQueue_ = nullptr;
    }
    if (tableMutex_ != nullptr) {
        vSemaphoreDelete(tableMutex_);
        tableMutex_ = nullptr;
    }
    if (latestMutex_ != nullptr) {
        vSemaphoreDelete(latestMutex_);
        latestMutex_ = nullptr;
    }
    if (statsMutex_ != nullptr) {
        vSemaphoreDelete(statsMutex_);
        statsMutex_ = nullptr;
    }
    if (stoppedSem_ != nullptr) {
        vSemaphoreDelete(stoppedSem_);
        stoppedSem_ = nullptr;
    }
}

bool RangingService::getLatest(CycleResult& out) const
{
    if (latestMutex_ == nullptr) {
        return false; // start() 前 / stop() 後
    }
    xSemaphoreTake(latestMutex_, portMAX_DELAY);
    out = latest_;
    xSemaphoreGive(latestMutex_);
    return true;
}

AnchorStats RangingService::stats(size_t anchorIndex) const
{
    AnchorStats result{};
    if (statsMutex_ == nullptr) {
        return result;
    }
    const size_t idx = (anchorIndex >= kMaxAnchors) ? (kMaxAnchors - 1) : anchorIndex;
    xSemaphoreTake(statsMutex_, portMAX_DELAY);
    result = statsSnapshot_[idx];
    xSemaphoreGive(statsMutex_);
    return result;
}

void RangingService::resetStats()
{
    if (tableMutex_ == nullptr) {
        return;
    }
    // scheduler_ の書き込み（処理タスクが周期の中で行う）と競合しないよう
    // tableMutex() を取ってから触る（tableMutex() のフィールドコメント参照:
    // 処理タスクは1周期の間ずっとこれを保持しているため、ここを取れた時点で
    // 「今この瞬間は周期の合間」であることが保証される）。
    xSemaphoreTake(tableMutex_, portMAX_DELAY);
    if (scheduler_ != nullptr) {
        scheduler_->resetStats();
    }
    xSemaphoreGive(tableMutex_);

    if (statsMutex_ != nullptr) {
        xSemaphoreTake(statsMutex_, portMAX_DELAY);
        for (size_t i = 0; i < kMaxAnchors; ++i) {
            statsSnapshot_[i] = AnchorStats{};
        }
        xSemaphoreGive(statsMutex_);
    }
}

void RangingService::reinitEkf()
{
    if (!cfg_.enableEkf || tableMutex_ == nullptr) {
        return;
    }
    xSemaphoreTake(tableMutex_, portMAX_DELAY);
    if (pipeline_ != nullptr) {
        pipeline_->initEkf();
    }
    xSemaphoreGive(tableMutex_);
}

void RangingService::taskTrampoline(void* arg)
{
    static_cast<RangingService*>(arg)->taskMain();
}

void RangingService::taskMain()
{
    // タスク自身のスタック上ではなく heap に置く（components/uwb_qm33120/src/
    // uwb_qm33120.cpp の Impl と同じ new/delete による PImpl 的所有）。
    scheduler_ = new RangingScheduler(*radio_, *table_, cfg_.scheduler);
    pipeline_   = new PositioningPipeline(*table_, cfg_.pipeline);

    if (cfg_.enableEkf) {
        // 起動直後の1回だけ組む。以後はアンカー表の構成が変わるたびに
        // reinitEkf() を呼び出し側（コンソール等）から呼んでもらう
        // （旧main.cppが `pipeline.initEkf()` を再度呼んでいたのと同じ考え方）。
        pipeline_->initEkf();
    }

    uint32_t seq              = 0;
    TickType_t lastWakeTick   = xTaskGetTickCount();

    while (running_) {
        CycleResult result{};

        // --- アンカー表を保護しながら1周期ぶん測距・測位する ---
        // tableMutex() のフィールドコメントのとおり、runCycle() の無線
        // 交換の間ずっと保持する（table.set() との競合を避けるため）。
        xSemaphoreTake(tableMutex_, portMAX_DELAY);

        result.tUs      = esp_timer_get_time();
        result.n         = scheduler_->runCycle(result.samples, kMaxAnchors);
        result.cycleMs  = scheduler_->lastCycleMs();

        const int64_t t0 = esp_timer_get_time();
        result.lv0        = pipeline_->solve(result.samples, result.n, SolverLevel::Lv0);
        const int64_t t1 = esp_timer_get_time();
        result.solveUsLv0 = static_cast<uint32_t>(t1 - t0);

        result.lv2        = pipeline_->solve(result.samples, result.n, SolverLevel::Lv2);
        const int64_t t2 = esp_timer_get_time();
        result.solveUsLv2 = static_cast<uint32_t>(t2 - t1);

        result.haveLv3 = cfg_.enableEkf;
        if (cfg_.enableEkf) {
            // updateEkf() の tS は「単調増加の秒」であればよい
            // （uwb_ranging_pipeline.hpp のコメント）。esp_timer の us値を
            // そのまま秒に直したものを使う（旧main.cppの t と同じ量）。
            const double tS = static_cast<double>(result.tUs) / 1e6;
            result.lv3        = pipeline_->updateEkf(tS, result.samples, result.n);
            const int64_t t3 = esp_timer_get_time();
            result.solveUsLv3 = static_cast<uint32_t>(t3 - t2);
        }

        // stats() 用のスナップショットをこの周期のうちに更新する
        // （tableMutex 保持中 = scheduler_ の書き込みと競合しないタイミング）。
        if (statsMutex_ != nullptr) {
            xSemaphoreTake(statsMutex_, portMAX_DELAY);
            for (size_t i = 0; i < kMaxAnchors; ++i) {
                statsSnapshot_[i] = scheduler_->stats(i);
            }
            xSemaphoreGive(statsMutex_);
        }

        xSemaphoreGive(tableMutex_);

        ++seq;
        result.seq = seq;

        // --- 結果の配布（このタスクは絶対にブロックしない） ---
        xSemaphoreTake(latestMutex_, portMAX_DELAY);
        latest_ = result;
        xSemaphoreGive(latestMutex_);

        xQueueOverwrite(latestQueue_, &result);

        if (xQueueSend(resultQueue_, &result, 0) != pdTRUE) {
            // 満杯: 最古の1件を捨てて入れ直す（drop-oldest）。
            CycleResult dropped;
            (void)xQueueReceive(resultQueue_, &dropped, 0);
            (void)xQueueSend(resultQueue_, &result, 0);
        }

        if (cfg_.cycleIntervalMs > 0) {
            vTaskDelayUntil(&lastWakeTick, pdMS_TO_TICKS(cfg_.cycleIntervalMs));
        } else {
            // ペーシング無し。次の周期の起点として使う lastWakeTick だけは
            // 更新しておく（cycleIntervalMs を後から変える運用は想定して
            // いないため、実際にはここは通らない経路だが安全側に倒す）。
            lastWakeTick = xTaskGetTickCount();
        }
    }

    delete pipeline_;
    pipeline_ = nullptr;
    delete scheduler_;
    scheduler_ = nullptr;

    if (stoppedSem_ != nullptr) {
        xSemaphoreGive(stoppedSem_);
    }
    vTaskDelete(nullptr);
}

} // namespace uwb
