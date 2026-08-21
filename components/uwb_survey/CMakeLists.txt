# uwb_survey — アンカー自動測量の計算本体（docs/SURVEY_SPEC.md の S1/S2）。
#
# ESP-IDF にも FreeRTOS にも UWB チップにも依存しない純粋な C99 で、
# tests/host/survey/ がホストで**同じソースを直接コンパイルして**検証する。
# ESP-NOW での収集（S3〜S5）や ToF での高さ取得（S7）は別コンポーネント。
#
# 線形代数は components/uwb_math（スカラー展開した 3x3 / 3x3 ブロックの
# 対称ソルバ・閉形式固有値・ブロックコレスキー）に全部任せる。uwb_real
# （倍精度 or 単精度）の typedef もそこから借りる。uwb_survey.h が公開
# ヘッダとして uwb_math.h を include するので REQUIRES（PUBLIC 相当）に置く。
# uwb_loc には依存しない（uwb_real の幅は uwb_math 側が
# CONFIG_UWB_LOC_USE_FLOAT を見て uwb_loc と揃える）。

idf_component_register(
    SRCS
        "src/uwb_survey.c"
    INCLUDE_DIRS
        "include"
    REQUIRES
        "uwb_math"
)
