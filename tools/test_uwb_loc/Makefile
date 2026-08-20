# components/uwb_loc/ (ESP-IDF コンポーネントとして vendoring したコピー) が
# third_party/uwb_localizer/c/tests/test_uwb.c (53件) に対して元と同じ結果を
# 返すことをホストで確認するための Makefile。
#
# テストコード自体は third_party/uwb_localizer/c/tests/test_uwb.c をそのまま
# 参照する（third_party は読み取り専用なのでコピーしない）。ビルドするのは
# components/uwb_loc/src/*.c、つまり本番でビルドされるのと同じソース。
#
#   make            既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8。
#                   Kconfig の既定と同じ) でビルド
#   make test       同上でビルドしてテスト実行 (53件)
#   make test-upstream-defaults
#                   取り込み元の既定値 (16/32) でテスト実行。参考用
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#                   (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8)
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -g
WARN     = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS   = -lm

ROOT        = ../..
UWB_LOC_DIR = $(ROOT)/components/uwb_loc
TEST_SRC    = $(ROOT)/third_party/uwb_localizer/c/tests/test_uwb.c

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

.PHONY: all test test-upstream-defaults strict clean

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

clean:
	rm -rf $(BUILD)
