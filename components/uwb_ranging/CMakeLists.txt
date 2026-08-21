# uwb_ranging — アンカー登録テーブル + 測距スケジューラ + 測位パイプライン。
#
# ハード非依存部分（uwb_ranging_anchor_table.cpp / uwb_ranging_pipeline.cpp、
# uwb_loc.h にしか依存しない）と、ハード依存部分（uwb_ranging_scheduler.cpp、
# uwb_qm33120 経由でTWRを叩く）に分かれている。ハード非依存部分は
# tests/host/pipeline がホストで直接コンパイルする対象でもある
# （そちらは本 CMakeLists を使わず、Makefile が同じ2ファイルを直接指定する）。

idf_component_register(
    SRCS
        "src/uwb_ranging_anchor_table.cpp"
        "src/uwb_ranging_pipeline.cpp"
        "src/uwb_ranging_scheduler.cpp"
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
