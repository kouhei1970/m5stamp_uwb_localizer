# uwb_ranging — アンカー登録テーブル + 測距スケジューラ + 測位パイプライン
# + RangingService（タグの測距・測位を1本の専用タスクにまとめたサービス、
# docs/ARCHITECTURE_V2.md §3.1）。
#
# ハード非依存部分（uwb_ranging_anchor_table.cpp / uwb_ranging_pipeline.cpp、
# uwb_loc.h にしか依存しない）と、ハード依存部分（uwb_ranging_scheduler.cpp /
# uwb_ranging_service.cpp、uwb_qm33120・FreeRTOS 経由でTWRを叩く）に
# 分かれている。ハード非依存部分は tests/host/pipeline がホストで直接
# コンパイルする対象でもある（そちらは本 CMakeLists を使わず、Makefile が
# 同じ2ファイルを名指しで指定する。uwb_ranging_scheduler.cpp /
# uwb_ranging_service.cpp はその Makefile には**含めない** — ホストには
# UWBチップもESP-IDF/FreeRTOSも無いため）。

idf_component_register(
    SRCS
        "src/uwb_ranging_anchor_table.cpp"
        "src/uwb_ranging_mode.cpp"
        "src/uwb_ranging_pipeline.cpp"
        "src/uwb_ranging_scheduler.cpp"
        "src/uwb_ranging_service.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        # uwb_ranging_anchor_table.hpp / uwb_ranging_pipeline.hpp が
        # uwb_loc.h を公開ヘッダとして include するため PUBLIC。
        "uwb_loc"
        # uwb_ranging_scheduler.hpp が uwb_qm33120.hpp を公開ヘッダとして
        # include するため PUBLIC（uwb_qm33120 自体が qm33120w_sdk/uwb_port/
        # driver/esp_timer を PUBLIC REQUIRES しているので、それらも
        # 連鎖的に見える）。
        "uwb_qm33120"
        # uwb_ranging_scheduler.cpp が測距間隔の管理に esp_timer_get_time() を、
        # 待ち合わせに FreeRTOS の vTaskDelay() を使う。
        "esp_timer"
        "freertos"
)
