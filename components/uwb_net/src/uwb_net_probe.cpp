/**
 * @file uwb_net_probe.cpp
 * @brief "net probe" コンソールコマンド（uwb_net_internal.hpp の分担:
 *        uwb_net_probe.cpp）。他機の HTTP / TCPコンソール / WebSocket を
 *        素のソケットで叩く自己診断クライアント。
 *        A self-test client that pokes another device's HTTP / TCP console /
 *        WebSocket endpoints with raw sockets, printing results via printf
 *        so they show on the calling device's USB console.
 *
 * 使い方: net probe <host> [http_port] [console_port]
 *   例: アンカー機のUSBコンソールで `net probe 192.168.4.1` を打つと、
 *       タグ機（SoftAP時のIPが192.168.4.1）のダッシュボード一式を
 *       アンカー側から叩いて動作確認できる（コーディネータ指示）。
 *
 * 方針: 1ステップの失敗でコマンド全体を中断しない。各ステップは自分の
 * 結果を "probe http: ..." 等の行で必ず何か出す。ソケットは毎回
 * SO_RCVTIMEO=3秒を既定にし、使い終わったら必ず close する。バッファは
 * 極力ヒープに置く（呼び出し元のコンソールタスクのスタックを圧迫しない
 * ため。USBコンソールのタスクスタックは6144バイトで、他のコマンドの
 * 呼び出しフレームとも共有している）。
 */
#include "uwb_net_internal.hpp"

#include "sdkconfig.h"

#if CONFIG_UWB_NET_ENABLE

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "esp_console.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "lwip/sockets.h"

namespace uwb::net::internal {

namespace {

/* ------------------------------------------------------------ 共通部品 --- */

/** host:port へTCP接続し、SO_RCVTIMEO/SO_SNDTIMEOをtimeoutMsに設定する。
 *  失敗したら-1を返す。host は数値IPv4文字列限定（"192.168.4.1"等。
 *  probeはローカルネット内の既知IPに対して使う想定なのでDNS解決はしない）。
 *
 * 【レビュー指摘 2026-08-31】SO_RCVTIMEO/SO_SNDTIMEO は lwIP では connect() 自体を
 * 縛らない（ハンドシェイク後の recv/send にしか効かない）。応答しない
 * ホスト相手だと TCP の規定リトライ（数十秒）までブロックしていた。
 * ソケットを O_NONBLOCK にしてから connect() し、select() の書き込みセットで
 * timeoutMs だけ待ち、SO_ERROR で結果を確認してから O_NONBLOCK を解除する。 */
int connectWithTimeout(const char* host, uint16_t port, int timeoutMs)
{
    struct sockaddr_in addr;
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port    = htons(port);
    if (inet_pton(AF_INET, host, &addr.sin_addr) != 1) {
        return -1;
    }

    const int sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        return -1;
    }

    // ハンドシェイク後（recv/send）のタイムアウトは通常どおり設定しておく。
    struct timeval tv;
    tv.tv_sec  = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(sock, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    const int origFlags = fcntl(sock, F_GETFL, 0);
    fcntl(sock, F_SETFL, origFlags | O_NONBLOCK);

    const int connErr = connect(sock, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr));
    if (connErr != 0 && errno != EINPROGRESS) {
        close(sock);
        return -1;
    }

    if (connErr != 0) {  // EINPROGRESS: select() で接続完了（または失敗）を待つ
        fd_set writeSet;
        FD_ZERO(&writeSet);
        FD_SET(sock, &writeSet);
        struct timeval selTv;
        selTv.tv_sec  = timeoutMs / 1000;
        selTv.tv_usec = (timeoutMs % 1000) * 1000;
        const int selRet = select(sock + 1, nullptr, &writeSet, nullptr, &selTv);
        if (selRet <= 0) {
            std::printf("probe: connect timeout (host=%s port=%u)\n", host, static_cast<unsigned>(port));
            close(sock);
            return -1;
        }
        int soErr           = 0;
        socklen_t soErrLen = sizeof(soErr);
        if (getsockopt(sock, SOL_SOCKET, SO_ERROR, &soErr, &soErrLen) != 0 || soErr != 0) {
            close(sock);
            return -1;
        }
    }

    fcntl(sock, F_SETFL, origFlags);  // O_NONBLOCK を解除し、以降は通常のブロッキングI/Oに戻す
    return sock;
}

/** そのソケットの受信タイムアウトだけ変更する（TCPコンソールの応答待ち等、
 *  ステップごとに待ち時間を変えたい場合に使う）。 */
void setRecvTimeout(int sock, int timeoutMs)
{
    struct timeval tv;
    tv.tv_sec  = timeoutMs / 1000;
    tv.tv_usec = (timeoutMs % 1000) * 1000;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

/* ------------------------------------------------------------- 1) HTTP --- */

/** buf(NUL終端済み)からHTTPステータス行（先頭の"HTTP/1.x ..." の1行）を
 *  outへコピーする。見つからなければ"(no response)"にする。 */
void extractStatusLine(const char* buf, char* out, size_t outCap)
{
    const char* end = strstr(buf, "\r\n");
    if (end == nullptr) {
        std::snprintf(out, outCap, "(no response)");
        return;
    }
    size_t len = static_cast<size_t>(end - buf);
    if (len >= outCap) {
        len = outCap - 1;
    }
    std::memcpy(out, buf, len);
    out[len] = '\0';
}

/** GET /api/info を1回打ち、ステータス行と本文冒頭200バイトを表示する。 */
void probeHttpApiInfo(const char* host, uint16_t port)
{
    const int sock = connectWithTimeout(host, port, 3000);
    if (sock < 0) {
        std::printf("probe http: /api/info への接続に失敗しました (host=%s port=%u)\n", host,
                    static_cast<unsigned>(port));
        return;
    }

    char req[160];
    const int reqLen = std::snprintf(req, sizeof(req), "GET /api/info HTTP/1.0\r\nHost: %s\r\n\r\n", host);
    send(sock, req, static_cast<size_t>(reqLen), 0);

    constexpr size_t kCap = 2048;
    char* buf              = static_cast<char*>(std::malloc(kCap + 1));
    if (buf == nullptr) {
        std::printf("probe http: メモリ不足です\n");
        close(sock);
        return;
    }
    size_t total = 0;
    while (total < kCap) {
        const int n = recv(sock, buf + total, kCap - total, 0);
        if (n <= 0) {
            break;
        }
        total += static_cast<size_t>(n);
    }
    buf[total] = '\0';
    close(sock);

    char statusLine[128];
    extractStatusLine(buf, statusLine, sizeof(statusLine));

    char bodyPreview[201] = "";
    const char* body        = strstr(buf, "\r\n\r\n");
    if (body != nullptr) {
        body += 4;
        std::snprintf(bodyPreview, sizeof(bodyPreview), "%.200s", body);
    }

    std::printf("probe http: GET /api/info -> %s\n", statusLine);
    std::printf("probe http:   body[0:200]=%s\n", bodyPreview);
    std::free(buf);
}

/** GET / を1回打ち、埋め込みページが最後まで丸ごと届くかを確認する
 *  （ステータス行・Content-Length・実受信バイト数を比較表示する）。 */
void probeHttpRoot(const char* host, uint16_t port)
{
    const int sock = connectWithTimeout(host, port, 3000);
    if (sock < 0) {
        std::printf("probe http: / への接続に失敗しました (host=%s port=%u)\n", host,
                    static_cast<unsigned>(port));
        return;
    }

    char req[128];
    const int reqLen = std::snprintf(req, sizeof(req), "GET / HTTP/1.0\r\nHost: %s\r\n\r\n", host);
    send(sock, req, static_cast<size_t>(reqLen), 0);

    // ヘッダ部分（先頭 <=2048 バイト）を集め、"\r\n\r\n" を境にヘッダと本文の
    // 先頭断片を切り分ける。以後の本文は捨てながら総バイト数だけ数える。
    constexpr size_t kHeadCap = 2048;
    char* head                 = static_cast<char*>(std::malloc(kHeadCap + 1));
    if (head == nullptr) {
        std::printf("probe http: メモリ不足です\n");
        close(sock);
        return;
    }
    size_t headLen        = 0;
    const char* bodyStart = nullptr;
    while (headLen < kHeadCap) {
        const int n = recv(sock, head + headLen, kHeadCap - headLen, 0);
        if (n <= 0) {
            break;
        }
        headLen += static_cast<size_t>(n);
        head[headLen] = '\0';
        bodyStart       = strstr(head, "\r\n\r\n");
        if (bodyStart != nullptr) {
            break;
        }
    }
    head[headLen] = '\0';

    char statusLine[128];
    extractStatusLine(head, statusLine, sizeof(statusLine));

    long contentLength = -1;
    const char* clHdr    = strstr(head, "Content-Length:");
    if (clHdr != nullptr) {
        contentLength = std::strtol(clHdr + std::strlen("Content-Length:"), nullptr, 10);
    }

    size_t totalBody = 0;
    if (bodyStart != nullptr) {
        totalBody = headLen - static_cast<size_t>((bodyStart + 4) - head);
    }
    std::free(head);

    // 残りの本文は捨てながら読み続ける（合計200KBで打ち切り。埋め込み
    // ページは高々数十KB程度なので十分な上限）。
    constexpr size_t kBodyCap = 200 * 1024;
    char discardBuf[512];
    while (totalBody < kBodyCap) {
        const int n = recv(sock, discardBuf, sizeof(discardBuf), 0);
        if (n <= 0) {
            break;
        }
        totalBody += static_cast<size_t>(n);
    }
    close(sock);

    std::printf("probe http: GET / -> %s\n", statusLine);
    if (contentLength >= 0) {
        std::printf("probe http:   Content-Length=%ld body_bytes_received=%u %s\n", contentLength,
                    static_cast<unsigned>(totalBody),
                    (static_cast<long>(totalBody) == contentLength) ? "(一致)" : "(不一致!)");
    } else {
        std::printf("probe http:   Content-Length ヘッダなし body_bytes_received=%u\n",
                    static_cast<unsigned>(totalBody));
    }
}

void probeHttp(const char* host, uint16_t port)
{
    probeHttpApiInfo(host, port);
    probeHttpRoot(host, port);
}

/* --------------------------------------------------------- 2) TCPコンソール --- */

void probeTcpConsole(const char* host, uint16_t port, const char* cmd)
{
    const int sock = connectWithTimeout(host, port, 3000);
    if (sock < 0) {
        std::printf("probe tcp: 接続に失敗しました (host=%s port=%u)\n", host, static_cast<unsigned>(port));
        return;
    }

    constexpr size_t kCap = 512;
    char* buf               = static_cast<char*>(std::malloc(kCap));
    if (buf == nullptr) {
        std::printf("probe tcp: メモリ不足です\n");
        close(sock);
        return;
    }

    // バナー（1回分）を受信して表示する。
    int n = recv(sock, buf, kCap - 1, 0);
    if (n > 0) {
        buf[n] = '\0';
        std::printf("probe tcp: banner: %s", buf); // バナー自体に改行が含まれる
    } else {
        std::printf("probe tcp: バナーを受信できませんでした\n");
    }

    // ws_cmd と同じコマンドを TCP コンソールにも送る（経路の切り分け用）。
    char cmdLine[176];
    std::snprintf(cmdLine, sizeof(cmdLine), "%s\n", cmd);
    send(sock, cmdLine, std::strlen(cmdLine), 0);

    // "wifi" コマンドの応答は短時間で終わるはずなので、この待ちだけ短く
    // (1.5秒) する。
    setRecvTimeout(sock, 1500);
    size_t total = 0;
    while (total < 400) {
        n = recv(sock, buf + total, kCap - 1 - total, 0);
        if (n <= 0) {
            break;
        }
        total += static_cast<size_t>(n);
        if (total >= 400) {
            break;
        }
    }
    buf[(total < kCap - 1) ? total : (kCap - 1)] = '\0';
    std::printf("probe tcp: 応答[0:400]=\n%s\n", buf);

    const char* exitCmd = "exit\n";
    send(sock, exitCmd, std::strlen(exitCmd), 0);

    std::free(buf);
    close(sock);
}

/* ---------------------------------------------------- 3) WebSocket --- */

/** recv() 前置バッファ付きソケット。HTTPハンドシェイク応答の読み取りで
 *  "\r\n\r\n" の直後（＝最初のWSフレームの先頭）まで一緒に読み込んで
 *  しまった分を、捨てずにWSフレーム解析側へ回すために使う。 */
struct BufSock {
    int sock;
    uint8_t pending[256];
    size_t pendingLen = 0;
    size_t pendingPos = 0;
};

int bsRecv(BufSock& bs, void* buf, size_t n)
{
    if (bs.pendingPos < bs.pendingLen) {
        const size_t avail = bs.pendingLen - bs.pendingPos;
        const size_t take    = (n < avail) ? n : avail;
        std::memcpy(buf, bs.pending + bs.pendingPos, take);
        bs.pendingPos += take;
        return static_cast<int>(take);
    }
    return recv(bs.sock, buf, n, 0);
}

bool bsRecvFull(BufSock& bs, uint8_t* buf, size_t n)
{
    size_t got = 0;
    while (got < n) {
        const int r = bsRecv(bs, buf + got, n - got);
        if (r <= 0) {
            return false;
        }
        got += static_cast<size_t>(r);
    }
    return true;
}

struct WsFrameInfo {
    uint8_t opcode  = 0;
    bool fin          = false;
    uint64_t payloadLen = 0;
};

/** 1フレームぶんのヘッダ（サーバ→クライアントなのでmaskは付かない想定だが、
 *  一応読める形にしておく）を読む。 */
bool readWsFrameHeader(BufSock& bs, WsFrameInfo& info)
{
    uint8_t hdr[2];
    if (!bsRecvFull(bs, hdr, 2)) {
        return false;
    }
    info.fin      = (hdr[0] & 0x80) != 0;
    info.opcode  = hdr[0] & 0x0F;
    const bool masked = (hdr[1] & 0x80) != 0; // サーバフレームは0のはず
    uint64_t len         = hdr[1] & 0x7F;
    if (len == 126) {
        uint8_t ext[2];
        if (!bsRecvFull(bs, ext, 2)) {
            return false;
        }
        len = (static_cast<uint64_t>(ext[0]) << 8) | ext[1];
    } else if (len == 127) {
        uint8_t ext[8];
        if (!bsRecvFull(bs, ext, 8)) {
            return false;
        }
        len = 0;
        for (int i = 0; i < 8; ++i) {
            len = (len << 8) | ext[i];
        }
    }
    info.payloadLen = len;
    if (masked) {
        // 本来サーバフレームには付かないが、万一付いていたら読み捨てて
        // 整合を保つ（プロトコル違反を検出しても落ちないようにする）。
        uint8_t maskKey[4];
        if (!bsRecvFull(bs, maskKey, 4)) {
            return false;
        }
    }
    return true;
}

/** ペイロードを最大 capCap バイトだけ capBuf へコピーしつつ、残りは
 *  読み捨てる（"discard beyond"）。実際に読んだバイト数(=payloadLen)は
 *  呼び出し側が info.payloadLen を見ればよいので、ここでは capturedLen
 *  （capBufへ書いた分）だけ返す。 */
bool readWsPayloadCapped(BufSock& bs, uint64_t payloadLen, char* capBuf, size_t capCap, size_t& capturedLen)
{
    capturedLen               = 0;
    uint64_t remaining = payloadLen;
    uint8_t tmp[256];
    while (remaining > 0) {
        const size_t want = (remaining < sizeof(tmp)) ? static_cast<size_t>(remaining) : sizeof(tmp);
        const int r          = bsRecv(bs, tmp, want);
        if (r <= 0) {
            return false;
        }
        if (capturedLen < capCap) {
            const size_t room  = capCap - capturedLen;
            const size_t copyN = (static_cast<size_t>(r) < room) ? static_cast<size_t>(r) : room;
            std::memcpy(capBuf + capturedLen, tmp, copyN);
            capturedLen += copyN;
        }
        remaining -= static_cast<uint64_t>(r);
    }
    return true;
}

constexpr int kMaxTypeNames                 = 12;
constexpr size_t kTypeNameCap             = 24;

struct TypeCounts {
    char names[kMaxTypeNames][kTypeNameCap];
    uint32_t counts[kMaxTypeNames] = {};
    int n                              = 0;
};

/** payload(capturedLen バイト、NUL終端はされていない可能性がある)から
 *  "\"type\":\"..."" を総当たりで探し、値ごとに件数を数える。 */
void countTypes(TypeCounts& tc, const char* payload, size_t len)
{
    static constexpr char kNeedle[]  = "\"type\":\"";
    static constexpr size_t kNeedleLen = sizeof(kNeedle) - 1;
    if (len < kNeedleLen) {
        return;
    }
    for (size_t i = 0; i + kNeedleLen <= len; ++i) {
        if (std::memcmp(payload + i, kNeedle, kNeedleLen) != 0) {
            continue;
        }
        size_t start = i + kNeedleLen;
        size_t end     = start;
        while (end < len && payload[end] != '"') {
            ++end;
        }
        char tok[kTypeNameCap];
        size_t tokLen = end - start;
        if (tokLen >= kTypeNameCap) {
            tokLen = kTypeNameCap - 1;
        }
        std::memcpy(tok, payload + start, tokLen);
        tok[tokLen] = '\0';

        bool matched = false;
        for (int j = 0; j < tc.n; ++j) {
            if (std::strcmp(tc.names[j], tok) == 0) {
                ++tc.counts[j];
                matched = true;
                break;
            }
        }
        if (!matched && tc.n < kMaxTypeNames) {
            std::snprintf(tc.names[tc.n], kTypeNameCap, "%s", tok);
            tc.counts[tc.n] = 1;
            ++tc.n;
        }
        i = end; // 同じ一致を数え直さないよう、値の終わりまで進める
    }
}

void probeWebSocket(const char* host, uint16_t port, const char* wsCmd)
{
    const int rawSock = connectWithTimeout(host, port, 3000);
    if (rawSock < 0) {
        std::printf("probe ws: 接続に失敗しました (host=%s port=%u)\n", host, static_cast<unsigned>(port));
        return;
    }

    char req[320];
    const int reqLen = std::snprintf(req, sizeof(req),
                                       "GET /ws HTTP/1.1\r\n"
                                       "Host: %s\r\n"
                                       "Upgrade: websocket\r\n"
                                       "Connection: Upgrade\r\n"
                                       "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                                       "Sec-WebSocket-Version: 13\r\n"
                                       "\r\n",
                                       host);
    send(rawSock, req, static_cast<size_t>(reqLen), 0);

    // ハンドシェイク応答（ヘッダのみ、<=512バイト想定）を読む。"\r\n\r\n"の
    // 直後まで一気に読めてしまった分はBufSock::pendingへ回し、WSフレーム
    // 解析側で使う（読み捨てない）。
    constexpr size_t kHeadCap = 512;
    char* headBuf               = static_cast<char*>(std::malloc(kHeadCap + 1));
    if (headBuf == nullptr) {
        std::printf("probe ws: メモリ不足です\n");
        close(rawSock);
        return;
    }
    size_t headLen        = 0;
    const char* bodyStart = nullptr;
    while (headLen < kHeadCap) {
        const int n = recv(rawSock, headBuf + headLen, kHeadCap - headLen, 0);
        if (n <= 0) {
            break;
        }
        headLen += static_cast<size_t>(n);
        headBuf[headLen] = '\0';
        bodyStart          = strstr(headBuf, "\r\n\r\n");
        if (bodyStart != nullptr) {
            break;
        }
    }
    headBuf[headLen] = '\0';

    char statusLine[128];
    extractStatusLine(headBuf, statusLine, sizeof(statusLine));
    std::printf("probe ws: %s\n", statusLine);

    BufSock bs;
    bs.sock = rawSock;
    if (bodyStart != nullptr) {
        const char* leftover  = bodyStart + 4;
        size_t leftoverLen      = headLen - static_cast<size_t>(leftover - headBuf);
        if (leftoverLen > sizeof(bs.pending)) {
            leftoverLen = sizeof(bs.pending); // 通常は数バイト〜数十バイト程度で収まる
        }
        std::memcpy(bs.pending, leftover, leftoverLen);
        bs.pendingLen = leftoverLen;
    }
    std::free(headBuf);

    if (strstr(statusLine, "101") == nullptr) {
        std::printf("probe ws: ハンドシェイクが101以外だったため、これ以上は行いません\n");
        close(rawSock);
        return;
    }

    // --- 3秒間、サーバからのフレームを数える ---
    constexpr size_t kFramePayloadCap = 4096;
    char* frameBuf                      = static_cast<char*>(std::malloc(kFramePayloadCap));
    if (frameBuf == nullptr) {
        std::printf("probe ws: メモリ不足です\n");
        close(rawSock);
        return;
    }

    TypeCounts tc;
    uint32_t frameCount = 0;
    uint64_t byteCount    = 0;
    const int64_t listenDeadlineUs = esp_timer_get_time() + 3LL * 1000 * 1000;
    while (esp_timer_get_time() < listenDeadlineUs) {
        WsFrameInfo info;
        if (!readWsFrameHeader(bs, info)) {
            break; // タイムアウト or 切断
        }
        size_t captured = 0;
        if (!readWsPayloadCapped(bs, info.payloadLen, frameBuf, kFramePayloadCap, captured)) {
            break;
        }
        ++frameCount;
        byteCount += info.payloadLen;
        if (info.opcode == 0x1) { // text frame のときだけ type を数える
            countTypes(tc, frameBuf, captured);
        }
    }

    std::printf("probe ws: frames=%lu bytes=%llu types:", static_cast<unsigned long>(frameCount),
                static_cast<unsigned long long>(byteCount));
    if (tc.n == 0) {
        std::printf(" (なし)");
    }
    for (int i = 0; i < tc.n; ++i) {
        std::printf(" %s=%lu", tc.names[i], static_cast<unsigned long>(tc.counts[i]));
    }
    std::printf("\n");

    // --- {"cmd":"<wsCmd>"} をマスク付きテキストフレームで送る（クライアント
    // ->サーバは必ずマスクする。RFC6455）。wsCmd はコマンドライン第4引数
    // （省略時 "wifi"）。JSON エスケープは共有ヘルパーで行う（saturating
    // なのでバッファを超えて書くことは無い）。 */
    {
        char payload[256];
        size_t payloadOff = 0;
        appendLiteral(payload, payloadOff, sizeof(payload), "{\"cmd\":\"");
        jsonEscapeAppend(payload, payloadOff, sizeof(payload), wsCmd);
        appendLiteral(payload, payloadOff, sizeof(payload), "\"}");
        const size_t payloadLen = payloadOff;

        uint8_t maskKey[4];
        const uint32_t r = esp_random();
        maskKey[0]          = static_cast<uint8_t>(r >> 24);
        maskKey[1]          = static_cast<uint8_t>(r >> 16);
        maskKey[2]          = static_cast<uint8_t>(r >> 8);
        maskKey[3]          = static_cast<uint8_t>(r);

        // ヘッダ: 126バイト未満は7bit長を直値で、以上は126固定+16bit拡張長で。
        uint8_t frameHdr[8];
        size_t hdrLen;
        if (payloadLen < 126) {
            frameHdr[0] = 0x80 | 0x01; // FIN + opcode=text
            frameHdr[1] = 0x80 | static_cast<uint8_t>(payloadLen); // mask + 7bit長
            std::memcpy(frameHdr + 2, maskKey, 4);
            hdrLen = 6;
        } else {
            frameHdr[0] = 0x80 | 0x01;
            frameHdr[1] = 0x80 | 126;
            frameHdr[2] = static_cast<uint8_t>((payloadLen >> 8) & 0xFF);
            frameHdr[3] = static_cast<uint8_t>(payloadLen & 0xFF);
            std::memcpy(frameHdr + 4, maskKey, 4);
            hdrLen = 8;
        }
        send(rawSock, frameHdr, hdrLen, 0);

        uint8_t masked[sizeof(payload)];
        for (size_t i = 0; i < payloadLen; ++i) {
            masked[i] = static_cast<uint8_t>(payload[i]) ^ maskKey[i % 4];
        }
        send(rawSock, masked, payloadLen, 0);
    }

    // --- "con" 応答を最大10秒待つ（wifi scan のような数秒かかるコマンドにも対応） ---
    bool foundCon                     = false;
    const int64_t conDeadlineUs = esp_timer_get_time() + 10LL * 1000 * 1000;
    while (!foundCon && esp_timer_get_time() < conDeadlineUs) {
        WsFrameInfo info;
        if (!readWsFrameHeader(bs, info)) {
            break;
        }
        size_t captured = 0;
        if (!readWsPayloadCapped(bs, info.payloadLen, frameBuf, kFramePayloadCap, captured)) {
            break;
        }
        if (info.opcode == 0x1 &&
            memmem(frameBuf, captured, "\"type\":\"con\"", std::strlen("\"type\":\"con\"")) != nullptr) {
            char preview[301];
            const size_t n = (captured < sizeof(preview) - 1) ? captured : (sizeof(preview) - 1);
            std::memcpy(preview, frameBuf, n);
            preview[n] = '\0';
            std::printf("probe ws: con応答[0:300]=%s\n", preview);
            foundCon = true;
        }
    }
    if (!foundCon) {
        std::printf("probe ws: \"con\" 応答が10秒以内に来ませんでした\n");
    }
    std::free(frameBuf);

    // --- close frame (opcode 8, masked, 空ペイロード) を送って閉じる ---
    {
        uint8_t maskKey[4] = {0x00, 0x00, 0x00, 0x00}; // 空ペイロードなので値は無関係
        uint8_t frameHdr[6];
        frameHdr[0] = 0x88; // FIN + opcode=close
        frameHdr[1] = 0x80; // mask + len=0
        std::memcpy(frameHdr + 2, maskKey, 4);
        send(rawSock, frameHdr, sizeof(frameHdr), 0);
    }
    close(rawSock);
}

/* --------------------------------------------------------- "net" コマンド --- */

void printNetUsage()
{
    std::printf("使い方: net probe <host> [http_port] [console_port] [ws_cmd]\n");
    std::printf("  例: net probe 192.168.4.1\n");
    std::printf("  例: net probe 192.168.4.1 80 23 \"wifi scan\"\n");
    std::printf("  他機のHTTP(/api/info, /)・TCPコンソール・WebSocket(/ws)を\n");
    std::printf("  素のソケットで叩いて自己診断する（結果はこのUSBコンソールへ表示）。\n");
    std::printf("  ws_cmd はWebSocket経由で送るコマンド（省略時 \"wifi\"）。\n");
}

bool parsePort(const char* text, uint16_t* out)
{
    if (text == nullptr || text[0] == '\0') {
        return false;
    }
    char* end                 = nullptr;
    const unsigned long v = std::strtoul(text, &end, 10);
    if (end == text || *end != '\0' || v == 0 || v > 0xFFFFul) {
        return false;
    }
    *out = static_cast<uint16_t>(v);
    return true;
}

int cmdNetProbe(int argc, char** argv)
{
    if (argc < 1 || argc > 4) {
        printNetUsage();
        return 1;
    }
    const char* host      = argv[0];
    uint16_t httpPort    = 80;
    uint16_t consolePort = 23;
    // esp_console は引用符で囲まれた引数を1個の argv へまとめてから渡してくる
    // ので、"wifi scan" のように空白を含むコマンドもそのまま argv[3] に入る
    // （コーディネータ指示の但し書きどおり）。
    const char* wsCmd = (argc >= 4) ? argv[3] : "wifi";
    if (argc >= 2 && !parsePort(argv[1], &httpPort)) {
        std::printf("http_port が不正です: %s\n", argv[1]);
        return 1;
    }
    if (argc >= 3 && !parsePort(argv[2], &consolePort)) {
        std::printf("console_port が不正です: %s\n", argv[2]);
        return 1;
    }

    std::printf("net probe: host=%s http_port=%u console_port=%u ws_cmd=%s\n", host,
                static_cast<unsigned>(httpPort), static_cast<unsigned>(consolePort), wsCmd);

    probeHttp(host, httpPort);
    probeTcpConsole(host, consolePort, wsCmd);
    probeWebSocket(host, httpPort, wsCmd);

    std::printf("net probe: 完了\n");
    return 0;
}

int cmdNet(int argc, char** argv)
{
    if (argc < 2) {
        printNetUsage();
        return 1;
    }
    const char* sub = argv[1];
    if (std::strcmp(sub, "probe") == 0) {
        return cmdNetProbe(argc - 2, argv + 2);
    }
    std::printf("不明なサブコマンド: %s\n", sub);
    printNetUsage();
    return 1;
}

} // namespace

/* ==================================================================== *
 * uwb_net_internal.hpp で宣言された関数
 * ==================================================================== */

esp_err_t probeRegisterConsoleCommands()
{
    const esp_console_cmd_t cmd = {
        .command  = "net",
        .help     = "他機のHTTP/TCPコンソール/WebSocketを叩く自己診断（net probe <host> [http_port] "
                    "[console_port]）",
        .hint     = " probe <host> [http_port] [console_port]",
        .func     = &cmdNet,
        .argtable = nullptr,
        .func_w_context = nullptr,
        .context        = nullptr,
    };
    return esp_console_cmd_register(&cmd);
}

} // namespace uwb::net::internal

#else // !CONFIG_UWB_NET_ENABLE

namespace uwb::net::internal {

esp_err_t probeRegisterConsoleCommands()
{
    return ESP_OK;
}

} // namespace uwb::net::internal

#endif // CONFIG_UWB_NET_ENABLE
