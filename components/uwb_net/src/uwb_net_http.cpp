/**
 * @file uwb_net_http.cpp
 * @brief HTTP サーバ + WebSocket ダッシュボード (uwb_net_internal.hpp の分担: uwb_net_http.cpp)。
 *        HTTP server + WebSocket dashboard endpoint.
 *
 *   GET  /          ページ本体 (EMBED_FILES で埋め込んだ web/index.html)
 *                   The dashboard page, embedded at link time via EMBED_FILES.
 *   GET  /api/info  現在の "node" 行と同じ内容を JSON で返す
 *                   Same content as the periodic "node" line, as plain JSON.
 *   /ws             WebSocket。サーバ→ブラウザは JSON Lines のプッシュ配信 (httpBroadcast)、
 *                   ブラウザ→サーバは {"cmd":"..."} 1 個につき "con" 行 1 個で返す。
 *                   Server pushes JSON Lines; browser sends {"cmd":"..."} and gets one "con" line back.
 *
 * 詳細仕様は scratchpad/NET_SPEC.md §6 を参照。
 *
 * 【レビュー指摘 2026-08-31・重要】以前は handleWsCommand() が httpd タスク上で
 * runCommandCaptured() を同期実行していた。"net probe"（応答しないホスト相手）や
 * "wifi scan"（約4秒）のような重いコマンドを実行すると、その間 httpd タスクが
 * 丸ごと塞がり、httpBroadcast() の作業（同じ httpd タスクへ httpd_queue_work()
 * される）も含め全HTTPクライアントが止まってしまっていた。
 * 対策: 専用ワーカータスク "uwb_net_wscmd"（コア0・優先度4・スタック8192）を
 * 起こし、長さ1のキューでコマンドを1件だけ受け付ける。handleWsCommand() は
 * パースしてキューへ積むだけ（キューが埋まっていれば「busy」の "con" 行を
 * httpd タスク上でその場で返す＝重い処理は一切しない）。ワーカーが
 * runCommandCaptured() を実行して "con" 行を組み立てたら、
 * httpd_queue_work() で httpd タスクへ渡し、そちらが
 * httpd_ws_send_frame_async() で送る（fdが既に消えていれば送信失敗するだけ、
 * 無視してよい）。httpBroadcast() 自身の pending フラグ（gBroadcastPending）は
 * この仕組みとは完全に独立（元々のとおり）。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <atomic>
#include <cstdlib>
#include <cstring>

#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

namespace uwb::net::internal {

namespace {

constexpr const char* kTag = "uwb_net_http";

/** httpd_config_t.max_open_sockets に渡す値と揃える（httpd_get_client_list の配列サイズもこれで足りる）。
 *  Kept in sync with httpd_config_t.max_open_sockets; also sized the fd array used for broadcast/count. */
constexpr uint16_t kMaxOpenSockets = 6;

/** WebSocket 受信 1 フレームの上限（コマンド行）。exceeding this length means "not a command", ignore it. */
constexpr size_t kWsRecvMaxLen = 256;

/** runCommandCaptured の出力を溜めるバッファの容量。 Accumulator capacity for captured command output. */
constexpr size_t kCmdAccCap = 6144;

/** サーバハンドル。起動前/停止後は nullptr。 Server handle; nullptr before start / after stop. */
httpd_handle_t gServer = nullptr;

// EMBED_FILES (components/uwb_net/CMakeLists.txt) が用意するシンボル。
// Symbols provided by the CMake EMBED_FILES mechanism for web/index.html.
extern "C" const uint8_t index_html_start[] asm("_binary_index_html_start");
extern "C" const uint8_t index_html_end[] asm("_binary_index_html_end");

// ---- httpBroadcast 用の状態。1 個の作業だけを httpd タスクに預ける（溜め込まない）。
// ---- State for httpBroadcast: at most one outstanding work item is queued at a time (never backs up).
// wscmd（コマンド返信）用のキュー・ペンディング状態とは完全に独立している。
// Entirely independent of the wscmd (command reply) queue/pending state below.
std::atomic<bool> gBroadcastPending{false};
uint32_t gBroadcastCount = 0;      // デバッグ用累計。 Debug-only running counts.
uint32_t gBroadcastFailCount = 0;  // httpd_queue_work 失敗など。 Counts httpd_queue_work failures etc.
uint32_t gBroadcastSendFailCount = 0;  // 個々のクライアントへの送信失敗（切断済みなど）。 Per-client send failures (e.g. already gone).

/** httpBroadcast() が malloc するペイロード。可変長データを後ろに続ける。
 *  Payload malloc'd by httpBroadcast(); the JSON bytes follow the header in the same allocation. */
struct BroadcastWork {
    size_t len;
    char data[];  // NOLINT: 可変長メンバ（C99 flexible array member 相当）。flexible-array-like tail.
};

// ---- "uwb_net_wscmd" ワーカー用の状態 ----
/** ワーカーへ渡す1コマンドぶん。キュー長は1（同時に1コマンドしか処理しない）。 */
struct WsCmdRequest {
    int fd;
    char cmd[161];  // uwb_net_cmd.cpp 側の 160 バイト制限に合わせる。
};

QueueHandle_t gWsCmdQueue        = nullptr;
TaskHandle_t gWsCmdTaskHandle      = nullptr;

/** ワーカータスクが組み立てた "con" 行を httpd タスクへ渡すための作業項目
 *  （httpd_queue_work 経由。可変長データを後ろに続ける）。 */
struct WsReplyWork {
    int fd;
    size_t len;
    char data[];  // NOLINT
};

/** {"cmd":"..."} をコマンド行としてアンエスケープしながら取り出す。
 *  Extract the command string from {"cmd":"..."} , unescaping \" \\ \/ \n \t as it copies.
 *
 * @param json  受信した WebSocket テキスト（NUL 終端済み）。NUL-terminated received text.
 * @param out   コピー先。 Destination buffer.
 * @param outCap outの容量（NUL 込み）。 Capacity of out, including the terminating NUL.
 * @return true ならコマンド文字列が見つかった。 true if a "cmd" string value was found.
 */
bool parseCmdField(const char* json, char* out, size_t outCap) {
    const char* key = strstr(json, "\"cmd\"");
    if (key == nullptr || outCap == 0) {
        return false;
    }
    const char* colon = strchr(key, ':');
    if (colon == nullptr) {
        return false;
    }
    const char* p = colon + 1;
    while (*p == ' ' || *p == '\t') {
        ++p;
    }
    if (*p != '"') {
        return false;
    }
    ++p;
    size_t outLen = 0;
    while (*p != '\0' && *p != '"') {
        char c = *p;
        if (c == '\\' && p[1] != '\0') {
            char esc = p[1];
            switch (esc) {
                case '"': c = '"'; break;
                case '\\': c = '\\'; break;
                case '/': c = '/'; break;
                case 'n': c = '\n'; break;
                case 't': c = '\t'; break;
                default: c = esc; break;  // 未知のエスケープはそのまま通す。Unknown escape: pass through.
            }
            p += 2;
        } else {
            ++p;
        }
        if (outLen + 1 < outCap) {
            out[outLen++] = c;
        }
        // outCap を超えた分は捨てる（コマンド長は 160 バイト制約が下流にある）。
        // Overflow is silently dropped; runCommandCaptured enforces its own 160-byte line limit downstream.
    }
    out[outLen] = '\0';
    return *p == '"';  // 閉じクォートまで来ていること。Require a proper closing quote.
}

// jsonEscapeAppendByte()/jsonEscapeAppend()/appendLiteral() は uwb_net_cmd.cpp
// で定義され、uwb::net::internal 名前空間（このファイルと同じ）で公開されている
// ので、ここでは #include "uwb_net_internal.hpp" だけで無限定名で呼べる
// （レビュー指摘 2026-08-31: 元々ここにあった同名のローカル実装を、
// uwb_net_sink.cpp とも共有するために移した）。

/** "con" 行（末尾 '\n' 込み）をヒープに組み立てる。呼び出し側が free() すること。
 *  cmd は JSON エスケープしてコピー、text は生バイト列を JSON エスケープして
 *  コピーする。malloc 失敗時は nullptr（*lenOut は変更しない）。
 *  handleWsCommand()（busy応答・httpdタスク上）と wsCmdTask()（実行結果・
 *  ワーカータスク上）の両方から呼ぶ共有ヘルパー。 */
char* buildConLine(const char* cmd, int ret, const char* text, size_t textLen, size_t* lenOut) {
    const size_t replyCap = textLen * 6 + strlen(cmd) * 6 + 64;
    char* reply = static_cast<char*>(malloc(replyCap));
    if (reply == nullptr) {
        return nullptr;
    }
    size_t off = 0;
    appendLiteral(reply, off, replyCap, "{\"v\":1,\"type\":\"con\",\"cmd\":\"");
    jsonEscapeAppend(reply, off, replyCap, cmd);
    appendLiteral(reply, off, replyCap, "\",\"ret\":");
    char retBuf[16];
    snprintf(retBuf, sizeof(retBuf), "%d", ret);  // 固定長のローカルバッファなので安全。Fixed-size local buffer; always safe.
    appendLiteral(reply, off, replyCap, retBuf);
    appendLiteral(reply, off, replyCap, ",\"text\":\"");
    // 末尾の `"}\n` を必ず書けるよう、本文はそこまでの余白でだけ書く。
    // Reserve room for the trailing `"}\n` so it always fits even if text fills the rest.
    constexpr size_t kCloserLen = 3;  // '"' '}' '\n'
    const size_t textLimit = (replyCap > kCloserLen) ? (replyCap - kCloserLen) : off;
    for (size_t i = 0; i < textLen && off < textLimit; ++i) {
        jsonEscapeAppendByte(reply, off, textLimit, static_cast<uint8_t>(text[i]));
    }
    appendLiteral(reply, off, replyCap, "\"}\n");
    *lenOut = off;
    return reply;
}

/** runCommandCaptured の出力を溜め込む先。溢れたら残りを捨てて truncated を立てる。
 *  Accumulator for runCommandCaptured() output; drops the remainder and sets truncated on overflow. */
struct CmdAccumulator {
    char* buf;
    size_t cap;
    size_t len = 0;
    bool truncated = false;
};

/** CmdOutFn 実装。CmdOutFn callback implementation: appends into a CmdAccumulator. */
void accumulateCmdOut(const char* data, size_t len, void* user) {
    auto* acc = static_cast<CmdAccumulator*>(user);
    if (acc->truncated) {
        return;
    }
    size_t room = (acc->cap > acc->len) ? (acc->cap - acc->len) : 0;
    if (len > room) {
        len = room;
        acc->truncated = true;
    }
    if (len > 0) {
        memcpy(acc->buf + acc->len, data, len);
        acc->len += len;
    }
    if (acc->truncated) {
        // 切り詰めた印を末尾へ収める余地があれば付ける（無ければ諦める。呼び出し側で cap に
        // 予め余白を残しておく想定）。
        // Append a truncation marker if there is still room (caller should leave headroom in cap).
        static const char kMark[] = "...\n";
        size_t markLen = sizeof(kMark) - 1;
        size_t markRoom = (acc->cap > acc->len) ? (acc->cap - acc->len) : 0;
        size_t n = markLen < markRoom ? markLen : markRoom;
        memcpy(acc->buf + acc->len, kMark, n);
        acc->len += n;
    }
}

// ---------------------------------------------------------------------------
// URI handlers
// ---------------------------------------------------------------------------

/** GET / : 埋め込みページを返す。 Serve the embedded dashboard page. */
esp_err_t handleRoot(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    const size_t len = static_cast<size_t>(index_html_end - index_html_start);
    return httpd_resp_send(req, reinterpret_cast<const char*>(index_html_start), static_cast<ssize_t>(len));
}

/** GET /api/info : 現在の "node" 行相当を JSON で返す。 Serve the current node-line snapshot as JSON. */
esp_err_t handleApiInfo(httpd_req_t* req) {
    char buf[1024];
    size_t len = buildNodeLine(buf, sizeof(buf));
    if (len > sizeof(buf)) {
        len = sizeof(buf);  // buildNodeLine は cap 超過時に切り詰め済みだが念のため。Defensive clamp.
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, static_cast<ssize_t>(len));
}

/** "uwb_net_wscmd" ワーカーが組み立てた返信を、httpd タスク上で実際に送る
 *  作業本体（httpd_queue_work 経由）。fd が既に切断済みなら送信が失敗する
 *  だけなので、それは無視する（コーディネータ指示）。 */
void wsReplyWork(void* arg) {
    auto* work = static_cast<WsReplyWork*>(arg);
    if (gServer != nullptr) {
        httpd_ws_frame_t frame;
        memset(&frame, 0, sizeof(frame));
        frame.type    = HTTPD_WS_TYPE_TEXT;
        frame.payload = reinterpret_cast<uint8_t*>(work->data);
        frame.len     = work->len;
        frame.final   = true;
        const esp_err_t sendErr = httpd_ws_send_frame_async(gServer, work->fd, &frame);
        ESP_LOGD(kTag, "ws: reply send fd=%d len=%u -> %s", work->fd, static_cast<unsigned>(work->len), esp_err_to_name(sendErr));
    }
    free(work);
}

/** "uwb_net_wscmd" タスク本体。キューから1件ずつ受け取り、
 *  runCommandCaptured() を（httpd タスクとは別のこのタスク上で）実行してから
 *  "con" 行を組み立て、httpd_queue_work() で httpd タスクへ返信を渡す。
 *  ブロッキング/時間のかかるコマンド（"wifi scan" の数秒・"net probe" の
 *  応答しないホスト相手の数秒など）を実行しても、httpd タスク（ひいては
 *  他のHTTP/WebSocketクライアント）を止めないための専用タスク
 *  （レビュー指摘 2026-08-31）。 */
void wsCmdTask(void* /*arg*/) {
    WsCmdRequest req;
    while (true) {
        if (xQueueReceive(gWsCmdQueue, &req, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        char* accBuf              = static_cast<char*>(malloc(kCmdAccCap));
        int cmdRet                  = 1;
        const char* textPtr       = "";
        size_t textLen               = 0;
        CmdAccumulator acc{accBuf, kCmdAccCap};
        if (accBuf != nullptr) {
            ESP_LOGI(kTag, "wscmd: run fd=%d cmd=\"%s\"", req.fd, req.cmd);
            cmdRet  = runCommandCaptured(req.cmd, accumulateCmdOut, &acc);
            textPtr = accBuf;
            textLen = acc.len;
            ESP_LOGI(kTag, "wscmd: done ret=%d text_len=%u", cmdRet, static_cast<unsigned>(acc.len));
        } else {
            ESP_LOGW(kTag, "wscmd: コマンド出力バッファのメモリ不足");
        }

        size_t replyLen = 0;
        char* reply       = buildConLine(req.cmd, cmdRet, textPtr, textLen, &replyLen);
        if (accBuf != nullptr) {
            free(accBuf);
        }
        if (reply == nullptr) {
            ESP_LOGW(kTag, "wscmd: 返信バッファのメモリ不足。この応答は諦めます");
            continue;
        }

        auto* work = static_cast<WsReplyWork*>(malloc(sizeof(WsReplyWork) + replyLen));
        if (work == nullptr) {
            free(reply);
            continue;
        }
        work->fd  = req.fd;
        work->len = replyLen;
        memcpy(work->data, reply, replyLen);
        free(reply);

        if (gServer == nullptr || httpd_queue_work(gServer, wsReplyWork, work) != ESP_OK) {
            ESP_LOGW(kTag, "wscmd: queue_work failed");
            free(work);
        } else {
            ESP_LOGD(kTag, "wscmd: reply queued len=%u", static_cast<unsigned>(replyLen));
        }
    }
}

/** WebSocket ハンドシェイク成立後、1 コマンド分の受信を行い、ワーカータスクへ
 *  渡す（実行はしない＝httpd タスクはここで長居しない）。
 *  After the WebSocket handshake, receive one command and hand it to the
 *  worker task; this handler itself never runs the command. */
esp_err_t handleWsCommand(httpd_req_t* req) {
    httpd_ws_frame_t pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.type = HTTPD_WS_TYPE_TEXT;

    // まず長さだけ取得する（max_len=0）。First call with max_len=0 just returns the frame length.
    esp_err_t ret = httpd_ws_recv_frame(req, &pkt, 0);
    if (ret != ESP_OK) {
        // 読めないソケットは閉じる。ESP_FAIL を返すと httpd がこのセッションを削除する
        // （httpd_process_session -> httpd_sess_delete）。
        // 以前は「切断扱いにしない」ために ESP_OK を返していたが、それだと相手が
        // RST で消えた接続（ブラウザの再読み込み後、配信送信が失敗した直後の状態）が
        // 永遠に残る。ESP-IDF v5.5.2 の httpd_ws_get_frame_type() は recv() の負の
        // 戻り値を size_t と比較して切断を見逃すため、ハンドラ側で閉じるしかない
        // （2026-09-02 実機で確認: 切れた接続に対し recv エラー 104/128 を毎周期
        // 繰り返す busy loop になり、配信が途切れ・生きているブラウザが LRU で追い出された）。
        // Close unreadable sockets: returning ESP_FAIL makes httpd delete the session.
        // Returning ESP_OK here left RST'd peers alive forever (IDF's
        // httpd_ws_get_frame_type() compares the negative recv() result against a
        // size_t and never flags the close), spinning the httpd task and evicting
        // live browsers through the LRU purge.
        ESP_LOGW(kTag, "ws recv (len probe) failed: %d, closing fd %d", ret, httpd_req_to_sockfd(req));
        return ESP_FAIL;
    }
    if (pkt.type != HTTPD_WS_TYPE_TEXT || pkt.len == 0 || pkt.len > kWsRecvMaxLen) {
        // 制御フレームや大きすぎる/空のフレームは無視する。Ignore control frames and oversized/empty frames.
        return ESP_OK;
    }

    char raw[kWsRecvMaxLen + 1];
    pkt.payload = reinterpret_cast<uint8_t*>(raw);
    ret = httpd_ws_recv_frame(req, &pkt, pkt.len);
    if (ret != ESP_OK) {
        // 同上。ヘッダは読めたのに本体が読めないのも壊れた接続なので閉じる。
        // Same as above: a frame whose payload cannot be read means a broken socket.
        ESP_LOGW(kTag, "ws recv (payload) failed: %d, closing fd %d", ret, httpd_req_to_sockfd(req));
        return ESP_FAIL;
    }
    raw[pkt.len] = '\0';

    WsCmdRequest cmdReq;
    cmdReq.fd = httpd_req_to_sockfd(req);
    if (!parseCmdField(raw, cmdReq.cmd, sizeof(cmdReq.cmd))) {
        ESP_LOGD(kTag, "ws frame without a usable \"cmd\" field, ignored");
        return ESP_OK;
    }

    if (gWsCmdQueue != nullptr && xQueueSend(gWsCmdQueue, &cmdReq, 0) == pdTRUE) {
        return ESP_OK;  // ワーカーが処理する。返信は wsCmdTask() -> wsReplyWork() 経由。
    }

    // キューが埋まっている（長さ1なので＝別のコマンドを処理中）。
    // ここ（httpd タスク）から軽量な "busy" 応答だけ即座に返す
    // （このリクエストのソケットへの同期送信なので安全・低コスト）。
    static const char kBusyText[] = "busy: another command is running\n";
    size_t replyLen                = 0;
    char* reply                      = buildConLine(cmdReq.cmd, -1, kBusyText, sizeof(kBusyText) - 1, &replyLen);
    if (reply != nullptr) {
        httpd_ws_frame_t out;
        memset(&out, 0, sizeof(out));
        out.type    = HTTPD_WS_TYPE_TEXT;
        out.payload = reinterpret_cast<uint8_t*>(reply);
        out.len     = replyLen;
        out.final   = true;
        if (httpd_ws_send_frame(req, &out) != ESP_OK) {
            ESP_LOGD(kTag, "ws send (busy reply) failed");
        }
        free(reply);
    }
    return ESP_OK;
}

/** GET/データフレーム両方を受ける /ws のエントリポイント。
 *  Single entry point for /ws: handshake (GET) returns immediately, data frames go to handleWsCommand. */
esp_err_t handleWs(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        // ハンドシェイクのみ。ここで新規クライアントへ何かを送る必要はない
        // （次の配信タスクの周期で自然に流れ始める）。
        // Handshake only; no need to push anything here — the next broadcast tick will reach it.
        ESP_LOGD(kTag, "ws: new client fd=%d", httpd_req_to_sockfd(req));
        return ESP_OK;
    }
    return handleWsCommand(req);
}

/** httpBroadcast() が httpd タスクへ積む作業本体。全 WebSocket クライアントへ配る。
 *  Work item queued by httpBroadcast(): fan the payload out to every connected WebSocket client. */
void broadcastWork(void* arg) {
    auto* work = static_cast<BroadcastWork*>(arg);
    if (gServer != nullptr) {
        int fdList[kMaxOpenSockets];
        size_t fdCount = kMaxOpenSockets;
        if (httpd_get_client_list(gServer, &fdCount, fdList) == ESP_OK) {
            httpd_ws_frame_t frame;
            memset(&frame, 0, sizeof(frame));
            frame.type = HTTPD_WS_TYPE_TEXT;
            frame.payload = reinterpret_cast<uint8_t*>(work->data);
            frame.len = work->len;
            frame.final = true;
            for (size_t i = 0; i < fdCount; ++i) {
                int fd = fdList[i];
                if (httpd_ws_get_fd_info(gServer, fd) != HTTPD_WS_CLIENT_WEBSOCKET) {
                    continue;
                }
                if (httpd_ws_send_frame_async(gServer, fd, &frame) != ESP_OK) {
                    // 送れない相手は閉じる。放置すると httpd の唯一のタスクが毎周期
                    // send_wait_timeout（2 秒）だけ待たされ、他の全クライアントへの配信が止まる。
                    // Close peers we cannot send to; otherwise the single httpd task
                    // blocks for send_wait_timeout on every tick and starves everyone else.
                    ++gBroadcastSendFailCount;
                    httpd_sess_trigger_close(gServer, fd);
                } else {
                    // 受信専用のブラウザは httpd の LRU では「未使用」のままになり、
                    // 枠が埋まると真っ先に追い出される。配信できた接続を使用中として記録する。
                    // A receive-only dashboard never touches the LRU counter and becomes the
                    // purge victim; count a successful push as activity.
                    httpd_sess_update_lru_counter(gServer, fd);
                }
            }
        }
    }
    free(work);
    gBroadcastPending.store(false, std::memory_order_release);
}

const httpd_uri_t kUriRoot = {
    .uri = "/",
    .method = HTTP_GET,
    .handler = handleRoot,
    .user_ctx = nullptr,
};

const httpd_uri_t kUriApiInfo = {
    .uri = "/api/info",
    .method = HTTP_GET,
    .handler = handleApiInfo,
    .user_ctx = nullptr,
};

const httpd_uri_t kUriWs = {
    .uri = "/ws",
    .method = HTTP_GET,
    .handler = handleWs,
    .user_ctx = nullptr,
    .is_websocket = true,
    .handle_ws_control_frames = false,
};

}  // namespace

/** HTTP サーバを起動し、"/" "/api/info" "/ws" を登録する。 Start the HTTP server and register the three URIs. */
esp_err_t httpStart() {
    if (gServer != nullptr) {
        return ESP_OK;  // 二重起動はしない。Already started; no-op.
    }

    // "uwb_net_wscmd" ワーカー（コマンド実行本体）を先に用意する。長さ1の
    // キューなので、2件目以降は handleWsCommand() が busy 応答を返すだけ
    // （このファイル冒頭コメント参照）。
    gWsCmdQueue = xQueueCreate(1, sizeof(WsCmdRequest));
    if (gWsCmdQueue == nullptr) {
        ESP_LOGE(kTag, "wscmd 用キューを作れませんでした");
        return ESP_ERR_NO_MEM;
    }
    const BaseType_t wsCmdTaskOk =
        xTaskCreatePinnedToCore(&wsCmdTask, "uwb_net_wscmd", 8192, nullptr, 4, &gWsCmdTaskHandle, 0);
    if (wsCmdTaskOk != pdPASS) {
        ESP_LOGE(kTag, "uwb_net_wscmd タスクの起動に失敗しました");
        vQueueDelete(gWsCmdQueue);
        gWsCmdQueue = nullptr;
        return ESP_FAIL;
    }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.core_id = 0;
    cfg.task_priority = 5;
    // "/ws" ハンドラ自体はもう重い処理をしない（パースしてキューへ積むだけ、
    // またはbusy応答をその場で組み立てて送るだけ）が、他のURIハンドラや
    // WebSocketハンドシェイク処理の分の余裕として既存値のまま維持する。
    cfg.stack_size = 10240;
    cfg.max_open_sockets = kMaxOpenSockets;
    cfg.lru_purge_enable = true;
    cfg.send_wait_timeout = 2;
    cfg.recv_wait_timeout = 5;
    // 通信が消えた相手（スリープした端末・圏外のスマホ）を TCP キープアライブで検出する。
    // 5 秒無通信 → 5 秒間隔で 3 回 → 約 20 秒で切断。無効だと相手が消えても永遠に枠を占める。
    // TCP keep-alive: detect peers that vanished without FIN in ~20 s instead of never.
    cfg.keep_alive_enable   = true;
    cfg.keep_alive_idle     = 5;
    cfg.keep_alive_interval = 5;
    cfg.keep_alive_count    = 3;
    cfg.server_port = config().httpPort;

    esp_err_t ret = httpd_start(&gServer, &cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(kTag, "httpd_start failed: %d", ret);
        gServer = nullptr;
        return ret;
    }

    httpd_register_uri_handler(gServer, &kUriRoot);
    httpd_register_uri_handler(gServer, &kUriApiInfo);
    httpd_register_uri_handler(gServer, &kUriWs);

    ESP_LOGI(kTag, "http: started on port %u", static_cast<unsigned>(cfg.server_port));
    return ESP_OK;
}

/** 配信タスクから呼ぶ。前の塊が送信待ちなら諦めて false を返す（キューを溜めない）。
 *  Called from the tx task; refuses (returns false) while a previous chunk is still queued. */
bool httpBroadcast(const char* data, size_t len) {
    if (gServer == nullptr || data == nullptr || len == 0) {
        return false;
    }
    bool expected = false;
    if (!gBroadcastPending.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
        return false;  // 前の作業がまだ httpd タスクで処理待ち。Previous work item hasn't drained yet.
    }

    auto* work = static_cast<BroadcastWork*>(malloc(sizeof(BroadcastWork) + len));
    if (work == nullptr) {
        gBroadcastPending.store(false, std::memory_order_release);
        return false;
    }
    work->len = len;
    memcpy(work->data, data, len);

    if (httpd_queue_work(gServer, broadcastWork, work) != ESP_OK) {
        free(work);
        gBroadcastPending.store(false, std::memory_order_release);
        ++gBroadcastFailCount;
        return false;
    }
    ++gBroadcastCount;
    ESP_LOGD(kTag, "http: broadcast #%u queued (%u bytes), fail=%u sendfail=%u",
              static_cast<unsigned>(gBroadcastCount), static_cast<unsigned>(len),
              static_cast<unsigned>(gBroadcastFailCount), static_cast<unsigned>(gBroadcastSendFailCount));
    return true;
}

/** 現在つながっている WebSocket クライアント数。 Number of currently connected WebSocket clients. */
int httpWsClientCount() {
    if (gServer == nullptr) {
        return 0;
    }
    int fdList[kMaxOpenSockets];
    size_t fdCount = kMaxOpenSockets;
    if (httpd_get_client_list(gServer, &fdCount, fdList) != ESP_OK) {
        return 0;
    }
    int n = 0;
    for (size_t i = 0; i < fdCount; ++i) {
        if (httpd_ws_get_fd_info(gServer, fdList[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
            ++n;
        }
    }
    return n;
}

TaskHandle_t wsCmdTaskHandle() {
    return gWsCmdTaskHandle;
}

}  // namespace uwb::net::internal

#else  // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t httpStart() { return ESP_OK; }
bool httpBroadcast(const char*, size_t) { return false; }
int httpWsClientCount() { return 0; }
TaskHandle_t wsCmdTaskHandle() { return nullptr; }

}  // namespace uwb::net::internal

#endif  // CONFIG_UWB_NET_ENABLE
