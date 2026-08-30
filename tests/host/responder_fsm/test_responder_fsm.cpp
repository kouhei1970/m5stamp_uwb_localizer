/**
 * @file test_responder_fsm.cpp
 * @brief components/uwb_qm33120/include/uwb_qm33120_responder_fsm.hpp
 * (`uwb::decide()`, docs/ARCHITECTURE_V2.md §2.2/§2.3) を実機なしでホスト上で
 * 直接検算するテスト。tests/host/pipeline/test_pipeline.cpp と同じ CHECK()
 * マクロの流儀（PASS/FAILをその場でカウントし、失敗時だけ内容を表示する）。
 *
 * §2.2 の遷移表の行を1つずつ、decide() への直接呼び出しでなぞる。
 * responder_fsm.hpp 自身のヘッダコメント（"Why State stays a bare 2-value
 * enum" / "Why uwb::TwrMethod, not uwb::RangingMethod"）が説明する設計に
 * 沿って、"Final from wrong peer or wrong seq" は
 * FrameSummary::matchesTrackedPeer=false（呼び出し側が既に判定済みという
 * 想定）で表現し、"Poll from other pan" は FrameSummary::panId を
 * cfg.panId と変えることで表現する。
 */
#include <cstdio>
#include <initializer_list>

#include "uwb_qm33120_responder_fsm.hpp"

using namespace uwb;

static int g_run  = 0;
static int g_fail = 0;

#define CHECK(cond, ...)                                             \
    do {                                                             \
        ++g_run;                                                     \
        if (!(cond)) {                                               \
            ++g_fail;                                                \
            std::printf("  NG  %s:%d  ", __FILE__, __LINE__);        \
            std::printf(__VA_ARGS__);                                \
            std::printf("\n");                                       \
        }                                                            \
    } while (0)

namespace {

const char* actionName(Action a)
{
    switch (a) {
    case Action::Ignore:
        return "Ignore";
    case Action::CheckIdleAndRearm:
        return "CheckIdleAndRearm";
    case Action::RespondSs:
        return "RespondSs";
    case Action::RespondDs:
        return "RespondDs";
    case Action::ComputeAndRespond:
        return "ComputeAndRespond";
    case Action::Abandon:
        return "Abandon";
    default:
        return "?";
    }
}

/** 既定値: panId=0xDECA, shortAddr=0x0002, method=DS, restartOnForeignPoll=true. */
ResponderConfig makeCfg(TwrMethod method, bool restartOnForeignPoll = true)
{
    ResponderConfig cfg;
    cfg.method              = method;
    cfg.panId                = 0xDECA;
    cfg.shortAddr           = 0x0002;
    cfg.restartOnForeignPoll = restartOnForeignPoll;
    return cfg;
}

/** 正しくアドレス指定された Poll（cfg 宛て、cfg.method と一致）。 */
FrameSummary makePoll(const ResponderConfig& cfg, uint16_t src, bool matchesTrackedPeer = true)
{
    FrameSummary f;
    f.kind                = FrameKind::Poll;
    f.src                  = src;
    f.dst                   = cfg.shortAddr;
    f.panId                  = cfg.panId;
    f.seq                     = 7;
    f.method                   = cfg.method;
    f.matchesTrackedPeer        = matchesTrackedPeer;
    return f;
}

/** 呼び出し側が既に peer/seq 一致を確認済みとして分類した Final。 */
FrameSummary makeFinal(const ResponderConfig& cfg, uint16_t src)
{
    FrameSummary f;
    f.kind                = FrameKind::Final;
    f.src                  = src;
    f.dst                   = cfg.shortAddr;
    f.panId                  = cfg.panId;
    f.seq                     = 7;
    f.matchesTrackedPeer        = true;
    return f;
}

FrameSummary makeOther()
{
    FrameSummary f;
    f.kind = FrameKind::Other;
    return f;
}

} // namespace

/* ------------------------------------------------------------------------
 * Listen 状態
 * ---------------------------------------------------------------------- */

static void listen_tick_checks_idle()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::Listen, Event::Tick, FrameSummary{}, cfg);
    CHECK(a == Action::CheckIdleAndRearm, "Listen+Tick は CheckIdleAndRearm のはず (got %s)", actionName(a));
}

static void listen_valid_poll_ss_responds()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::SS);
    const FrameSummary poll   = makePoll(cfg, 0x0001);
    const Action a            = decide(State::Listen, Event::RxFrame, poll, cfg);
    CHECK(a == Action::RespondSs, "Listen+Poll(SS,method一致) は RespondSs のはず (got %s)", actionName(a));
}

static void listen_valid_poll_ds_responds()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const FrameSummary poll   = makePoll(cfg, 0x0001);
    const Action a            = decide(State::Listen, Event::RxFrame, poll, cfg);
    CHECK(a == Action::RespondDs, "Listen+Poll(DS,method一致) は RespondDs のはず (got %s)", actionName(a));
}

static void listen_poll_wrong_pan_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary poll         = makePoll(cfg, 0x0001);
    poll.panId                = static_cast<uint16_t>(cfg.panId ^ 0x0001); // 他PAN。
    const Action a            = decide(State::Listen, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "Listen+Poll(他PAN) は Ignore のはず (got %s) - 'Poll from other pan ignored'",
          actionName(a));
}

static void listen_poll_wrong_dst_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary poll         = makePoll(cfg, 0x0001);
    poll.dst                  = static_cast<uint16_t>(cfg.shortAddr + 1); // 他アドレス宛て。
    const Action a            = decide(State::Listen, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "Listen+Poll(他アドレス宛て) は Ignore のはず (got %s)", actionName(a));
}

static void listen_poll_wrong_method_ignored()
{
    // アンカーは DS で待っているが、届いた Poll が SS のタグ用（method タグ不一致）。
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary poll         = makePoll(cfg, 0x0001);
    poll.method                = TwrMethod::SS;
    const Action a            = decide(State::Listen, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "Listen+Poll(method不一致) は Ignore のはず (got %s)", actionName(a));
}

static void listen_other_frame_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::Listen, Event::RxFrame, makeOther(), cfg);
    CHECK(a == Action::Ignore, "Listen+Other は Ignore のはず (got %s)", actionName(a));
}

static void listen_stray_final_ignored()
{
    // 通常あり得ない(Finalが来るのはWaitFinal中のはず)が、防御的に Ignore を期待する。
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const FrameSummary final_ = makeFinal(cfg, 0x0001);
    const Action a            = decide(State::Listen, Event::RxFrame, final_, cfg);
    CHECK(a == Action::Ignore, "Listen+Final(迷子) は Ignore のはず (got %s)", actionName(a));
}

static void listen_rxerror_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::Listen, Event::RxError, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "Listen+RxError は Ignore のはず (got %s)", actionName(a));
}

static void listen_rxtimeout_ignored()
{
    // RXタイムアウト自体は Listen では想定されない(rxtimeout=0で無期限)が、
    // 防御的に Ignore を期待する。
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::Listen, Event::RxTimeout, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "Listen+RxTimeout は Ignore のはず (got %s)", actionName(a));
}

static void listen_txdone_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::Listen, Event::TxDone, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "Listen+TxDone は Ignore のはず (got %s)", actionName(a));
}

/* ------------------------------------------------------------------------
 * WaitFinal 状態
 * ---------------------------------------------------------------------- */

static void waitfinal_matching_final_computes()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const FrameSummary final_ = makeFinal(cfg, 0x0001);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, final_, cfg);
    CHECK(a == Action::ComputeAndRespond, "WaitFinal+Final(一致) は ComputeAndRespond のはず (got %s)",
          actionName(a));
}

static void waitfinal_final_wrong_peer_or_seq_ignored()
{
    // 呼び出し側が peer/seq 不一致を検出した場合、matchesTrackedPeer=false で
    // 渡す想定（uwb_qm33120_responder_fsm.hpp ヘッダコメント参照）。
    // "Final from wrong peer or wrong seq ignored" を表現する。
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary final_       = makeFinal(cfg, 0x0001);
    final_.matchesTrackedPeer  = false;
    const Action a            = decide(State::WaitFinal, Event::RxFrame, final_, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Final(peer/seq不一致) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_final_wrong_pan_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary final_       = makeFinal(cfg, 0x0001);
    final_.panId               = static_cast<uint16_t>(cfg.panId ^ 0x0001);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, final_, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Final(他PAN) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_final_wrong_dst_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    FrameSummary final_       = makeFinal(cfg, 0x0001);
    final_.dst                 = static_cast<uint16_t>(cfg.shortAddr + 1);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, final_, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Final(他アドレス宛て) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_same_peer_poll_always_restarts_ss()
{
    // 同じ相手からの再送 Poll は restartOnForeignPoll に関わらず常に応じる。
    for (bool restartFlag : {true, false}) {
        const ResponderConfig cfg = makeCfg(TwrMethod::SS, restartFlag);
        const FrameSummary poll   = makePoll(cfg, 0x0001, /*matchesTrackedPeer=*/true);
        const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
        CHECK(a == Action::RespondSs,
              "WaitFinal+Poll(同一peer,restartOnForeignPoll=%d,SS) は RespondSs のはず (got %s)", (int)restartFlag,
              actionName(a));
    }
}

static void waitfinal_same_peer_poll_always_restarts_ds()
{
    for (bool restartFlag : {true, false}) {
        const ResponderConfig cfg = makeCfg(TwrMethod::DS, restartFlag);
        const FrameSummary poll   = makePoll(cfg, 0x0001, /*matchesTrackedPeer=*/true);
        const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
        CHECK(a == Action::RespondDs,
              "WaitFinal+Poll(同一peer,restartOnForeignPoll=%d,DS) は RespondDs のはず (got %s)", (int)restartFlag,
              actionName(a));
    }
}

static void waitfinal_foreign_poll_restart_true_responds()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS, /*restartOnForeignPoll=*/true);
    const FrameSummary poll   = makePoll(cfg, 0x0003, /*matchesTrackedPeer=*/false); // 別のタグ。
    const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
    CHECK(a == Action::RespondDs, "WaitFinal+Poll(他タグ,restartOnForeignPoll=true) は RespondDs のはず (got %s)",
          actionName(a));
}

static void waitfinal_foreign_poll_restart_false_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS, /*restartOnForeignPoll=*/false);
    const FrameSummary poll   = makePoll(cfg, 0x0003, /*matchesTrackedPeer=*/false); // 別のタグ。
    const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Poll(他タグ,restartOnForeignPoll=false) は Ignore のはず (got %s)",
          actionName(a));
}

static void waitfinal_poll_wrong_pan_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS, /*restartOnForeignPoll=*/true);
    FrameSummary poll         = makePoll(cfg, 0x0001, /*matchesTrackedPeer=*/true);
    poll.panId                = static_cast<uint16_t>(cfg.panId ^ 0x0001);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Poll(他PAN) は Ignore のはず (got %s) - 'Poll from other pan ignored'",
          actionName(a));
}

static void waitfinal_poll_wrong_dst_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS, /*restartOnForeignPoll=*/true);
    FrameSummary poll         = makePoll(cfg, 0x0001, /*matchesTrackedPeer=*/true);
    poll.dst                  = static_cast<uint16_t>(cfg.shortAddr + 1);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Poll(他アドレス宛て) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_poll_wrong_method_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS, /*restartOnForeignPoll=*/true);
    FrameSummary poll         = makePoll(cfg, 0x0001, /*matchesTrackedPeer=*/true);
    poll.method                = TwrMethod::SS;
    const Action a            = decide(State::WaitFinal, Event::RxFrame, poll, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Poll(method不一致) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_other_frame_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::WaitFinal, Event::RxFrame, makeOther(), cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Other は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_rxerror_ignored_keeps_waiting()
{
    // 呼び出し側は期限内であることを確認済みで Event::RxError を渡す想定
    // (期限超過は呼び出し側が Event::RxTimeout に丸める、ヘッダコメント参照)。
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::WaitFinal, Event::RxError, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+RxError(期限内) は Ignore(待ち継続) のはず (got %s)", actionName(a));
}

static void waitfinal_rxtimeout_abandons()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::WaitFinal, Event::RxTimeout, FrameSummary{}, cfg);
    CHECK(a == Action::Abandon, "WaitFinal+RxTimeout は Abandon のはず (got %s)", actionName(a));
}

static void waitfinal_tick_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::WaitFinal, Event::Tick, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+Tick(期限内) は Ignore のはず (got %s)", actionName(a));
}

static void waitfinal_txdone_ignored()
{
    const ResponderConfig cfg = makeCfg(TwrMethod::DS);
    const Action a            = decide(State::WaitFinal, Event::TxDone, FrameSummary{}, cfg);
    CHECK(a == Action::Ignore, "WaitFinal+TxDone は Ignore のはず (got %s)", actionName(a));
}

/* ------------------------------------------------------------------------
 * その他: 既定値・型のとりあえずの健全性
 * ---------------------------------------------------------------------- */

static void defaults_match_spec()
{
    const ResponderConfig cfg;
    CHECK(cfg.method == TwrMethod::DS, "ResponderConfig::method の既定は DS のはず");
    CHECK(cfg.panId == 0xDECA, "ResponderConfig::panId の既定は 0xDECA のはず (got 0x%04X)", cfg.panId);
    CHECK(cfg.shortAddr == 0x0002, "ResponderConfig::shortAddr の既定は 0x0002 のはず (got 0x%04X)",
          cfg.shortAddr);
    CHECK(cfg.idleTickMs == 20, "ResponderConfig::idleTickMs の既定は 20 のはず (docs/ARCHITECTURE_V2.md §1 "
                                 "2026-08-30改定, got %u)",
          cfg.idleTickMs);
    CHECK(cfg.restartOnForeignPoll == true, "ResponderConfig::restartOnForeignPoll の既定は true のはず");

    const ResponderStats stats;
    CHECK(stats.polls == 0 && stats.responses == 0 && stats.finals == 0 && stats.results == 0 &&
              stats.restarts == 0 && stats.finalTimeouts == 0 && stats.rxErrors == 0 && stats.txFailures == 0 &&
              stats.rearms == 0 && stats.other == 0 && stats.eventDrops == 0 && stats.lastRxStatus == 0,
          "ResponderStats の既定は全カウンタ0のはず");
    CHECK(stats.distance.count == 0, "ResponderStats::distance の既定は count=0 のはず");
}

static void state_and_action_enums_are_distinct()
{
    CHECK(State::Listen != State::WaitFinal, "State の2値は異なるはず");
    CHECK(Action::Ignore != Action::CheckIdleAndRearm, "Action の値は異なるはず");
    CHECK(Action::RespondSs != Action::RespondDs, "Action::RespondSs/RespondDs は異なるはず");
    CHECK(TwrMethod::SS != TwrMethod::DS, "TwrMethod の2値は異なるはず");
    CHECK(FrameKind::None != FrameKind::Poll && FrameKind::Poll != FrameKind::Final &&
              FrameKind::Final != FrameKind::Other,
          "FrameKind の4値は異なるはず");
}

int main()
{
    std::printf("=== tests/host/responder_fsm: uwb::decide() 遷移表 検証 (docs/ARCHITECTURE_V2.md §2.2) ===\n\n");

    listen_tick_checks_idle();
    listen_valid_poll_ss_responds();
    listen_valid_poll_ds_responds();
    listen_poll_wrong_pan_ignored();
    listen_poll_wrong_dst_ignored();
    listen_poll_wrong_method_ignored();
    listen_other_frame_ignored();
    listen_stray_final_ignored();
    listen_rxerror_ignored();
    listen_rxtimeout_ignored();
    listen_txdone_ignored();

    waitfinal_matching_final_computes();
    waitfinal_final_wrong_peer_or_seq_ignored();
    waitfinal_final_wrong_pan_ignored();
    waitfinal_final_wrong_dst_ignored();
    waitfinal_same_peer_poll_always_restarts_ss();
    waitfinal_same_peer_poll_always_restarts_ds();
    waitfinal_foreign_poll_restart_true_responds();
    waitfinal_foreign_poll_restart_false_ignored();
    waitfinal_poll_wrong_pan_ignored();
    waitfinal_poll_wrong_dst_ignored();
    waitfinal_poll_wrong_method_ignored();
    waitfinal_other_frame_ignored();
    waitfinal_rxerror_ignored_keeps_waiting();
    waitfinal_rxtimeout_abandons();
    waitfinal_tick_ignored();
    waitfinal_txdone_ignored();

    defaults_match_spec();
    state_and_action_enums_are_distinct();

    std::printf("\n=== %d 件中 %d 件失敗 ===\n", g_run, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
