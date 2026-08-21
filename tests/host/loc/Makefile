# uwb_loc (components/uwb_loc) のホストテスト。
#
# 上流 uwb_localizer は 2026-08-21 に凍結し、最終状態を components/uwb_loc/
# へ取り込んだ。以後 components/uwb_loc/ は本リポジトリで独立して開発する
# ので、ここは third_party や上流クローンに依存せず本リポジトリ内だけで
# 完結する (旧 tools/test_uwb_loc の後継)。
#
#   make            既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8。
#                   Kconfig の既定と同じ) でビルド
#   make test       同上でビルドしてテスト実行 (77件)
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

ROOT        = ../../..
UWB_LOC_DIR = $(ROOT)/components/uwb_loc
TEST_SRC    = test_uwb.c

INCLUDES = -I$(UWB_LOC_DIR)/include -I$(UWB_LOC_DIR)/src

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
# アンカー/観測数は4〜8台運用なので8で十分 (docs/PLAN.md「運用構成」参照)。
ANCHORS8_DEFS = -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8

SRC = $(UWB_LOC_DIR)/src/uwb_linalg.c \
      $(UWB_LOC_DIR)/src/uwb_model.c \
      $(UWB_LOC_DIR)/src/uwb_closed_form.c \
      $(UWB_LOC_DIR)/src/uwb_nls.c \
      $(UWB_LOC_DIR)/src/uwb_ekf.c

BUILD = build
OBJ   = $(patsubst $(UWB_LOC_DIR)/src/%.c,$(BUILD)/%.o,$(SRC))
LIB   = $(BUILD)/libuwbloc_components.a
TEST_BIN = $(BUILD)/test_uwb

DEFS ?= $(ANCHORS8_DEFS)

.PHONY: all test test-upstream-defaults strict float clean

all: $(LIB)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(UWB_LOC_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)

$(TEST_BIN): $(TEST_SRC) $(LIB)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 $(TEST_SRC) $(LIB) $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

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
