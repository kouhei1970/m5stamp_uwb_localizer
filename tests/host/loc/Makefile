# uwb_loc (components/uwb_loc) のホストテスト。
#
# 上流 uwb_localizer は 2026-08-21 に凍結し、最終状態を components/uwb_loc/
# へ取り込んだ。以後 components/uwb_loc/ は本リポジトリで独立して開発する
# ので、ここは third_party や上流クローンに依存せず本リポジトリ内だけで
# 完結する (旧 tools/test_uwb_loc の後継)。
#
# ref/ には凍結時点 (commit 4298c08) の components/uwb_loc をそのまま
# 写した「参照実装」を置いてあり (test_regress.c が新旧比較に使う)、
# ref_prefix.h で全シンボルに ref_ 接頭辞を付けて新実装とのリンク衝突を
# 避けている。ref/ の中身は本体の変更を追わない (詳しくは各ファイル先頭
# のコメントと ref/ref_bridge.h を参照)。
#
#   make            既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8。
#                   Kconfig の既定と同じ) でビルド
#   make test       同上でビルドして test_uwb (77件) → test_regress
#                   (新旧比較回帰テスト) の順に実行
#   make regress    test_regress だけをビルドして実行
#   make test-upstream-defaults
#                   取り込み元の既定値 (16/32) でテスト実行。参考用
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#                   (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8)
#   make float      float 版で作り直して試す (マイコン想定)
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -g
WARN     = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS   = -lm

ROOT         = ../../..
UWB_LOC_DIR  = $(ROOT)/components/uwb_loc
UWB_MATH_DIR = $(ROOT)/components/uwb_math
TEST_SRC          = test_uwb.c
TEST_REGRESS_SRC  = test_regress.c

INCLUDES = -I$(UWB_LOC_DIR)/include -I$(UWB_LOC_DIR)/src -I$(UWB_MATH_DIR)/include

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
# アンカー/観測数は4〜8台運用なので8で十分 (docs/PLAN.md「運用構成」参照)。
ANCHORS8_DEFS = -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8

# uwb_linalg.c は uwb_math への書き換えで削除される予定 (作業中はまだ
# 存在する)。$(wildcard ...) で「そのとき実在するものだけ」を拾うので、
# 削除前後どちらの状態でも SRC の変更なしに通る。
SRC = $(UWB_LOC_DIR)/src/uwb_model.c \
      $(UWB_LOC_DIR)/src/uwb_closed_form.c \
      $(UWB_LOC_DIR)/src/uwb_nls.c \
      $(UWB_LOC_DIR)/src/uwb_ekf.c \
      $(wildcard $(UWB_LOC_DIR)/src/uwb_linalg.c) \
      $(wildcard $(UWB_MATH_DIR)/src/uwb_math.c)

BUILD = build
# VPATH で uwb_loc/src と uwb_math/src の両方から %.c を探させる
# (ファイル名が一意なので衝突しない)。SRC がどちらのディレクトリの
# ものを含んでいても $(BUILD)/%.o: %.c の 1 本のパターンルールで済む。
VPATH = $(UWB_LOC_DIR)/src:$(UWB_MATH_DIR)/src

OBJ   = $(addprefix $(BUILD)/,$(notdir $(SRC:.c=.o)))
LIB   = $(BUILD)/libuwbloc_components.a
TEST_BIN         = $(BUILD)/test_uwb
TEST_REGRESS_BIN = $(BUILD)/test_regress

# --------------------------------------------------------- 参照実装 (ref/)
REF_DIR      = ref
REF_SRC      = $(wildcard $(REF_DIR)/*.c)
REF_OBJ      = $(addprefix $(BUILD)/ref/,$(notdir $(REF_SRC:.c=.o)))
REF_LIB      = $(BUILD)/libuwbloc_ref.a
REF_INCLUDES = -I$(REF_DIR)
# 上流を無改変で写したものなので同じ WARN でまず試す。通らなくなったら
# ここだけ緩める (その場合は理由をここに書くこと)。今のところ緩めていない。
REF_WARN ?= $(WARN)

DEFS ?= $(ANCHORS8_DEFS)

.PHONY: all test regress test-upstream-defaults strict float clean

all: $(LIB) $(REF_LIB)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/ref:
	mkdir -p $(BUILD)/ref

# 静的パターンルール ($(OBJ) に列挙された target だけに適用) にしてある。
# 素の "$(BUILD)/%.o: %.c" だと stem に "ref/..." も一致してしまい、
# 下の ref 用ルールと衝突する (どちらが選ばれるか make の実装依存になる)。
$(OBJ): $(BUILD)/%.o: %.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)

# ref/*.c は無改変の上流ソースを -include ref_prefix.h でコンパイルし、
# uwb_solve_lv2 のような全シンボルを ref_uwb_solve_lv2 に付け替える
# (詳しくは ref/ref_prefix.h)。これで新実装 (LIB) と衝突せず 1 つの
# テストバイナリにリンクできる。こちらも静的パターンルールにして
# 上の $(OBJ) 用ルールと衝突しないようにしてある。
$(REF_OBJ): $(BUILD)/ref/%.o: $(REF_DIR)/%.c $(REF_DIR)/ref_prefix.h | $(BUILD)/ref
	$(CC) $(CFLAGS) $(REF_WARN) $(DEFS) $(REF_INCLUDES) -std=c99 -include $(REF_DIR)/ref_prefix.h -c $< -o $@

$(REF_LIB): $(REF_OBJ)
	$(AR) rcs $@ $(REF_OBJ)

$(TEST_BIN): $(TEST_SRC) $(LIB) $(REF_LIB)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 $(TEST_SRC) $(LIB) $(REF_LIB) $(LDLIBS) -o $@

$(TEST_REGRESS_BIN): $(TEST_REGRESS_SRC) $(LIB) $(REF_LIB)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 $(TEST_REGRESS_SRC) $(LIB) $(REF_LIB) $(LDLIBS) -o $@

# test_uwb (77件) → test_regress (新旧比較) の順。tests/Makefile は
# このターゲットの標準出力の末尾行だけを拾うので、最後に走る方
# (test_regress) の OK/NG 行がそこに出る。
test: $(TEST_BIN) $(TEST_REGRESS_BIN)
	./$(TEST_BIN)
	./$(TEST_REGRESS_BIN)

# test_regress だけを単独で走らせたいとき用。
regress: $(TEST_REGRESS_BIN)
	./$(TEST_REGRESS_BIN)

# 取り込み元の既定 (UWB_MAX_ANCHORS=16 / UWB_MAX_MEAS=32) でも通るかの参考実行。
# components/uwb_loc の実運用は ANCHORS8_DEFS (Kconfig 既定) を使う。
test-upstream-defaults:
	$(MAKE) clean
	$(MAKE) DEFS="" test

# 警告をすべてエラーにして通す (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8)。
strict:
	$(MAKE) clean
	$(MAKE) WARN="-Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -Wno-sign-conversion" test

# 単精度でも通るか。マイコンでは float の方が速い/小さいことが多い。
# -Wconversion を外すのは、テストのアンカー座標が double リテラルで
# 書いてあるため。ライブラリ本体は float でも無警告で通る。
float:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O2 -g -DUWB_USE_FLOAT" WARN="-Wall -Wextra -pedantic -Wshadow" test

clean:
	rm -rf $(BUILD)
