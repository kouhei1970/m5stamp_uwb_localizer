# シリアルコンソール（tag_console.cpp）は CONFIG_UWB_TAG_CONSOLE が有効な
# ときだけビルド対象に入れる。
#
# REQUIRES の方を条件分岐**しない**のは ESP-IDF のビルド手順の都合。
# コンポーネントの CMakeLists.txt は 2 回評価され、依存関係を集める 1 回目
# （early expansion）では CONFIG_* がまだ定義されていない。そのため
# REQUIRES を if(CONFIG_...) で囲むと、SRCS の方だけが 2 回目で有効になり
# 「ソースはあるのに console への依存が無い」状態になってビルドが失敗する。
# console を常に REQUIRES へ入れておいても、参照されないアーカイブメンバは
# リンクされないのでバイナリサイズには影響しない。
set(tag_srcs "main.cpp")
if(CONFIG_UWB_TAG_CONSOLE)
    list(APPEND tag_srcs "tag_console.cpp")
endif()

idf_component_register(
    SRCS
        ${tag_srcs}
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/tag/main/, so that
        # #include "boards/stamps3.h" / "boards/stampfly.h"
        # resolves (same convention as firmware/probe, firmware/devtest,
        # firmware/twr).
        "../../.."
    REQUIRES
        # 役割別ハートビート LED (タグ=緑 / アンカー=赤)
        # Role heartbeat LED (tag = green, anchor = red)
        uwb_status_led
        uwb_ranging
        uwb_qm33120
        uwb_port
        qm33120w_sdk
        uwb_loc
        esp_timer
        # NVS への設定永続化（アンカー登録テーブル）。コンソールを無効にしても
        # 「NVS に保存済みのテーブルで起動する」機能は残るので、常に必要。
        uwb_cfgstore
        # メジャー実測値からのアンカー座標 閉形式計算 (survey dist/z/show/
        # apply、tag_console.cpp のみが使う)。CONFIG_UWB_TAG_CONSOLE=n でも
        # REQUIRES を条件分岐しない理由は上の console と同じ（early
        # expansion の都合）。
        uwb_survey
        # シリアルコンソール（linenoise + argtable3）。上のコメント参照。
        console
        # Wi-Fi + ブラウザダッシュボード + 無線コンソール（scratchpad/NET_SPEC.md）。
        uwb_net
        # IP_EVENT/IP_EVENT_STA_GOT_IP + esp_event_handler_register()。
        # 起動パンくずのstage4 (got_ip) 用ハンドラ登録（main.cpp）。
        # uwb_net 経由でも透過的に付いてくるが、直接使うので明示しておく。
        esp_event
        esp_netif
)
