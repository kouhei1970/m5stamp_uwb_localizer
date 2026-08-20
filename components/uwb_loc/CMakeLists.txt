# uwb_loc — third_party/uwb_localizer/c の測位ソルバをそのまま vendoring した
# コンポーネント。ロジックは無改造（このファイルの追加のみ）。
#
# コンパイル時パラメータ（UWB_MAX_ANCHORS / UWB_MAX_MEAS / UWB_ID_LEN /
# UWB_USE_FLOAT）は uwb_loc.h 本体を書き換えず、Kconfig の値を
# target_compile_definitions で -D 上書きすることで反映する（uwb_loc.h 側は
# #ifndef で既定値を用意しているので、これだけで上書きできる）。

idf_component_register(
    SRCS
        "src/uwb_linalg.c"
        "src/uwb_model.c"
        "src/uwb_closed_form.c"
        "src/uwb_nls.c"
        "src/uwb_ekf.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        # uwb_internal.h / uwb_linalg.h はライブラリ内部専用（公開 API では
        # ない）ので、他コンポーネントからは見えない PRIV_INCLUDE_DIRS に置く。
        "src"
    # 依存は libm の sqrt/fabs と string.h の strncmp のみ。ESP-IDF は
    # libm を既定でリンクするため REQUIRES は不要（Kconfig 側にも注記あり）。
)

target_compile_definitions(${COMPONENT_LIB} PUBLIC
    "UWB_MAX_ANCHORS=${CONFIG_UWB_LOC_MAX_ANCHORS}"
    "UWB_MAX_MEAS=${CONFIG_UWB_LOC_MAX_MEAS}"
    "UWB_ID_LEN=${CONFIG_UWB_LOC_ID_LEN}"
)

if(CONFIG_UWB_LOC_USE_FLOAT)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "UWB_USE_FLOAT=1")
endif()
