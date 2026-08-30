# components/uwb_net の CMakeLists.txt
#
# 分担は src/uwb_net_internal.hpp の冒頭コメント参照:
#   uwb_net.cpp      : 公開 API (start/publishLine/isStarted/ipString/
#                       registerConsoleCommands) と internal::config()
#   uwb_net_wifi.cpp : Wi-Fi 起動・NVS の認証情報・mDNS・"wifi" コンソールコマンド
#   uwb_net_sink.cpp : 行のリングバッファ・配信タスク (uwb_net_tx)・"node" 行の生成
#   uwb_net_udp.cpp  : UDP ブロードキャスト送信 (forward) / 受信して集約 (aggregate)
#   uwb_net_cmd.cpp  : esp_console_run を stdout 差し替えで実行し出力を取り出す共通処理
#   uwb_net_tcp.cpp  : TCP テキストコンソール (port 23)
#   uwb_net_http.cpp : HTTP サーバ、"/" のページ、"/api/info"、"/ws" の WebSocket
#   uwb_net_probe.cpp: "net probe <host> ..." コンソールコマンド（自己診断クライアント）
#
# CONFIG_UWB_NET_ENABLE=n でもこのコンポーネント自体は常にビルド対象に入れる
# （各 .cpp が自分の中で #if CONFIG_UWB_NET_ENABLE により本体/no-opスタブを
# 切り替える。呼び出し側の firmware/tag, firmware/anchor の CMakeLists.txt を
# Kconfig で分岐させずに済むようにするため）。
idf_component_register(
    SRCS
        "src/uwb_net.cpp"
        "src/uwb_net_wifi.cpp"
        "src/uwb_net_sink.cpp"
        "src/uwb_net_udp.cpp"
        "src/uwb_net_cmd.cpp"
        "src/uwb_net_tcp.cpp"
        "src/uwb_net_http.cpp"
        "src/uwb_net_probe.cpp"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        "src"
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        esp_http_server
        nvs_flash
        console
        lwip
        esp_timer
        # esp_random()（"net probe" のWebSocketクライアントマスクキー生成用）。
        esp_hw_support
        # マネージド・コンポーネント（idf_component.yml の espressif/mdns）。
        # Managed component (see idf_component.yml's espressif/mdns entry).
        mdns
    EMBED_FILES
        "web/index.html"
)
