/**
 * @file uwb_net_internal.hpp
 * @brief uwb_net コンポーネント内部の分担境界（ファイル間の契約）。公開 API ではない。
 *
 *   uwb_net_wifi.cpp   : Wi-Fi 起動・NVS の認証情報・mDNS・"wifi" コンソールコマンド
 *   uwb_net_sink.cpp   : 行のリングバッファ・間引き・配信タスク (uwb_net_tx)・"node" 行の生成
 *   uwb_net_udp.cpp    : UDP ブロードキャスト送信 (forward) / 受信して集約 (aggregate)
 *   uwb_net_cmd.cpp    : esp_console_run を stdout 差し替えで実行し出力を取り出す共通処理
 *   uwb_net_tcp.cpp    : TCP テキストコンソール (port 23)
 *   uwb_net_http.cpp   : HTTP サーバ、"/" のページ、"/api/info"、"/ws" の WebSocket
 *   uwb_net_probe.cpp  : "net probe <host> ..." コンソールコマンド（他機のHTTP/TCP
 *                        コンソール/WebSocketを叩く自己診断クライアント）
 */
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h" // TaskHandle_t（各タスクのスタック残量を
                                // uwb_net_tx から横断して見るための accessor 群で使う）
#include "freertos/task.h"
#include "uwb_net.hpp"

namespace uwb::net::internal {

/** start() でコピーした設定 */
const Config& config();

// ---- uwb_net_wifi.cpp ----
struct WifiStatus {
    char mode[6];    // "sta" / "ap" / "off"
    char ssid[33];   // 接続中（STA）または自分の SSID（AP）
    char ip[16];
    int rssi;        // STA のとき。AP なら 0
    bool connected;  // STA: GOT_IP 済み / AP: 起動済み
    uint8_t apClients;
};
esp_err_t wifiStart();
void wifiStatus(WifiStatus& out);
esp_err_t wifiRegisterConsoleCommands();
/** 保存済みSSIDの再スキャンを行う "uwb_net_wifi" タスクのハンドル。
 *  uwb_net_tx の起動時スタック残量ログ専用。 */
TaskHandle_t wifiRescanTaskHandle();

// ---- uwb_net_probe.cpp ----
/** "net" コンソールコマンド（現状のサブコマンドは "probe" のみ）を登録する。 */
esp_err_t probeRegisterConsoleCommands();

// ---- uwb_net_sink.cpp ----
esp_err_t sinkStart();
/** 間引きを通さずに積む（他機から UDP で受けた行・"node" 行用）。複数行まとめて可 */
void sinkPushRaw(const char* data, size_t len);
uint32_t sinkDrops();
/** "node" 行（末尾 '\n' 込み）を組み立てる。戻り値は長さ（cap 超過なら切り詰め） */
size_t buildNodeLine(char* buf, size_t cap);

// ---- uwb_net_udp.cpp ----
esp_err_t udpStart();
/** 行の塊（'\n' 区切り）を ≤1400 バイトのデータグラムに分割して送る（forward=true のときだけ有効） */
void udpSend(const char* data, size_t len);
uint32_t udpTxCount();
uint32_t udpRxCount();
/** 受信タスクのハンドル（aggregate=false 等で作っていなければ nullptr）。
 *  uwb_net_tx の起動時スタック残量ログ専用。 */
TaskHandle_t udpRxTaskHandle();

// ---- uwb_net_cmd.cpp ----
/** コマンド実行用ミューテックスを1回だけ作る。uwb::net::start() の先頭で呼ぶ
 *  （runCommandCaptured() 側の遅延生成はTOCTOUレースがあるため廃止した）。 */
esp_err_t cmdInit();

using CmdOutFn = void (*)(const char* data, size_t len, void* user);
/**
 * 1 行のコマンドを esp_console_run で実行し、その間の stdout/stderr 出力を out へ渡す。
 * 呼び出しタスクの stdout を fopencookie で作った FILE に差し替え、終了後に戻す。
 * 同時実行は内部ミューテックスで直列化する。戻り値はコマンドの戻り値
 * （未知のコマンドなら out に "Unrecognized command\n" を出して 1 を返す）。
 * cmdInit() を呼ぶ前に呼ばれた場合はエラーメッセージを out へ渡して 1 を返す。
 */
int runCommandCaptured(const char* line, CmdOutFn out, void* user);

/** JSON文字列としてエスケープしながら1バイト追記する（'"' '\\' '\n' '\r' '\t' と
 *  0x20未満の制御文字を \u00XX、0x80以上はUTF-8継続バイトとしてそのまま）。
 *  cap を超えたら何もしない（saturating）。uwb_net_http.cpp・uwb_net_sink.cpp の
 *  両方から使う共有ヘルパー。 */
void jsonEscapeAppendByte(char* dst, size_t& off, size_t cap, uint8_t byte);
/** NUL終端文字列を jsonEscapeAppendByte() でエスケープしながら追記する。 */
void jsonEscapeAppend(char* dst, size_t& off, size_t cap, const char* src);
/** NUL終端の生文字列をそのまま（エスケープせず）cap までで追記する（saturating）。 */
void appendLiteral(char* dst, size_t& off, size_t cap, const char* src);

// ---- uwb_net_tcp.cpp ----
esp_err_t tcpConsoleStart();
bool tcpConsoleClientConnected();
/** accept ループタスクのハンドル。uwb_net_tx の起動時スタック残量ログ専用。 */
TaskHandle_t tcpConsoleTaskHandle();

// ---- uwb_net_http.cpp ----
esp_err_t httpStart();
/**
 * 配信タスクから呼ぶ。data（'\n' 区切りの JSON 行の塊）を全 WebSocket クライアントへ
 * テキストフレーム 1 つとして送る。内部でコピーし httpd_queue_work で httpd タスクに渡す。
 * 前の塊がまだ送信待ちなら捨てて false を返す（メモリを増やさない）。
 */
bool httpBroadcast(const char* data, size_t len);
int httpWsClientCount();
/** "net"/"wifi" 等の重いコマンドを実行するワーカータスク "uwb_net_wscmd" のハンドル。
 *  uwb_net_tx の起動時スタック残量ログ専用。 */
TaskHandle_t wsCmdTaskHandle();

}  // namespace uwb::net::internal
