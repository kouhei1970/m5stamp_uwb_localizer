# uwb_survey — アンカー自動測量の計算本体（docs/SURVEY_SPEC.md の S1/S2）。
#
# ESP-IDF にも FreeRTOS にも UWB チップにも依存しない純粋な C99 で、
# tools/test_survey/ がホストで**同じソースを直接コンパイルして**検証する。
# ESP-NOW での収集（S3〜S5）や ToF での高さ取得（S7）は別コンポーネント。
#
# uwb_loc への依存は uwb_real（倍精度 or 単精度の切り替え）を共有するため
# だけ。uwb_survey.h が公開ヘッダとして uwb_loc.h を include するので
# REQUIRES（PUBLIC 相当）に置く。uwb_loc 側の関数は 1 つも呼んでいない
# （uwb_linalg は 9x9 が上限で 25x25 の正規方程式が入らないため、
#  src/uwb_survey_dense.c に測量専用の密行列ソルバを別に持っている）。

idf_component_register(
    SRCS
        "src/uwb_survey_dense.c"
        "src/uwb_survey.c"
    INCLUDE_DIRS
        "include"
    PRIV_INCLUDE_DIRS
        # uwb_survey_dense.h は内部専用（公開 API ではない）。
        "src"
    REQUIRES
        "uwb_loc"
)
