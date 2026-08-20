# シリアルコンソール（anchor_console.cpp）は CONFIG_UWB_ANCHOR_CONSOLE が
# 有効なときだけビルド対象に入れる。
#
# REQUIRES の方を条件分岐**しない**のは ESP-IDF のビルド手順の都合。
# コンポーネントの CMakeLists.txt は 2 回評価され、依存関係を集める 1 回目
# （early expansion）では CONFIG_* がまだ定義されていない。そのため
# REQUIRES を if(CONFIG_...) で囲むと、SRCS の方だけが 2 回目で有効になり
# 「ソースはあるのに console への依存が無い」状態になってビルドが失敗する。
# console を常に REQUIRES へ入れておいても、参照されないアーカイブメンバは
# リンクされないのでバイナリサイズには影響しない。
set(anchor_srcs "main.cpp")
if(CONFIG_UWB_ANCHOR_CONSOLE)
    list(APPEND anchor_srcs "anchor_console.cpp")
endif()

idf_component_register(
    SRCS
        ${anchor_srcs}
    INCLUDE_DIRS
        "."
        # Repo root, three levels up from firmware/anchor/main/, so that
        # #include "boards/stamps3.h" / "boards/atoms3.h" resolves (same
        # convention as firmware/twr, firmware/probe, firmware/devtest).
        "../../.."
    REQUIRES
        uwb_qm33120
        uwb_port
        qm33120w_sdk
        # NVS への設定永続化（ショートアドレス）。コンソールを無効にしても
        # 「NVS に保存済みの値で起動する」機能は残るので、常に必要。
        uwb_cfgstore
        # シリアルコンソール（linenoise + argtable3）。上のコメント参照。
        console
)
