idf_component_register(
    SRCS
        "main.c"
        "bench.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        uwb_loc
        # esp_timer_get_time() でソルバの実行時間を計測するため。
        esp_timer
        # esp_cpu_get_cycle_count() (bench.c の基本演算マイクロベンチ) のため。
        # ESP-IDF の全コンポーネントに暗黙のPRIV_REQUIRESとして付くが、
        # 直接ヘッダを include しているので明示しておく。
        esp_hw_support
        # esp_rom_get_cpu_ticks_per_us() (CPU周波数の取得) のため。理由は上と同じ。
        esp_rom
)
