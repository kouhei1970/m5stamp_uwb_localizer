# uwb_math — 小次元（2x2 / 3x3 / 3x3 ブロック）の線形代数をスカラー展開した
# 純粋な C99 のコンポーネント。ESP-IDF にも FreeRTOS にも依存せず、
# tests/host/math/ がホストで**同じソースを直接コンパイルして**検証する。
#
# 依存は libm (sqrt / acos / cos) のみ。ESP-IDF は libm を既定でリンクする
# ので REQUIRES は不要。
#
# 実数型 uwb_real の float/double 切り替えは uwb_loc と同じ流儀で、
# ヘッダ本体を書き換えずに -DUWB_USE_FLOAT を PUBLIC に付けて反映する。
# Kconfig は持たない (uwb_loc の CONFIG_UWB_LOC_USE_FLOAT に追従する:
# 両コンポーネントで uwb_real の幅が食い違うと typedef が衝突するため、
# 切り替え元を 1 箇所に保つ)。

idf_component_register(
    SRCS
        "src/uwb_math.c"
    INCLUDE_DIRS
        "include"
)

if(CONFIG_UWB_LOC_USE_FLOAT)
    target_compile_definitions(${COMPONENT_LIB} PUBLIC "UWB_USE_FLOAT=1")
endif()
