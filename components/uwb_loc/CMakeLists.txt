# uwb_loc — 測位ソルバ (上流 uwb_localizer を 2026-08-21 に凍結して取り込み、
# 以後は本リポジトリで独立して開発しているコンポーネント)。
#
# 線形代数は components/uwb_math (2x2 / 3x3 のスカラー展開) に寄せてあり、
# 一般の LU / コレスキー / Jacobi はここには無い。uwb_loc.h が uwb_math.h を
# include する (uwb_real を共有する) ので、依存は PUBLIC な REQUIRES にする。
#
# コンパイル時パラメータ（UWB_MAX_ANCHORS / UWB_MAX_MEAS / UWB_ID_LEN /
# UWB_USE_FLOAT）は uwb_loc.h 本体を書き換えず、Kconfig の値を
# target_compile_definitions で -D 上書きすることで反映する（uwb_loc.h 側は
# #ifndef で既定値を用意しているので、これだけで上書きできる）。
# UWB_USE_FLOAT は uwb_math 側にも同じ値が PUBLIC で付く (uwb_math/CMakeLists.txt
# が CONFIG_UWB_LOC_USE_FLOAT を見る) ので、uwb_real の幅は両者で一致する。

idf_component_register(
    SRCS
        "src/uwb_model.c"
        "src/uwb_closed_form.c"
        "src/uwb_nls.c"
        "src/uwb_ekf.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        # uwb_internal.h はライブラリ内部専用（公開 API ではない）ので、
        # 他コンポーネントからは見えない PRIV_INCLUDE_DIRS に置く。
        "src"
    REQUIRES
        uwb_math
    # それ以外の依存は libm の sqrt と string.h の strncmp のみ。ESP-IDF は
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
