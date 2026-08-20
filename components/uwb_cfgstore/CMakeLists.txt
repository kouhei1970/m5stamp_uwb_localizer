# uwb_cfgstore — NVS への設定永続化（タグ／アンカー共用）。
#
# 2 層に分かれている。
#
#   src/uwb_cfgstore_blob.cpp  ... バイト列の組み立てと検査。ESP-IDF に
#       一切依存しない（uwb_ranging_types.hpp と <cmath>/<cstring> のみ）。
#       tools/test_pipeline がホストで**同じソースを直接コンパイルして**
#       往復・境界値・破損データの検算を行う（実機が無いので、検証できる
#       のはこの層まで、という切り分け）。
#   src/uwb_cfgstore.cpp       ... NVS 入出力。nvs_flash に依存する。
#
# uwb_ranging への依存は uwb::AnchorEntry / uwb::kMaxAnchors を使うため
# （uwb_ranging_types.hpp はヘッダのみで uwb_loc.h にしか依存しない）。
# アンカー用ファームは測位を行わないが、テーブル定義を共有する都合で
# 同じ依存になる。未参照のオブジェクトはアーカイブから取り込まれないので
# バイナリサイズへの影響は無い。

idf_component_register(
    SRCS
        "src/uwb_cfgstore_blob.cpp"
        "src/uwb_cfgstore.cpp"
    INCLUDE_DIRS
        "include"
    REQUIRES
        # uwb_cfgstore_blob.hpp が公開ヘッダとして uwb_ranging_types.hpp を include する。
        "uwb_ranging"
        # uwb_cfgstore.cpp が nvs_flash.h / nvs.h を使う。
        "nvs_flash"
)
