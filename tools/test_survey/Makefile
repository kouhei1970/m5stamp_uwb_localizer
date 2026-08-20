# components/uwb_survey（アンカー自動測量の計算本体）を、実機なしで
# ホスト上で直接ビルド・実行して検証するための Makefile。
#
# tools/test_pipeline/Makefile と同じ考え方（third_party は読み取り専用、
# ビルドするのは本番と同じ components/ 配下のソース）。uwb_survey は
# 全部 C99 なので、こちらは cc だけで済む。
#
# uwb_loc からは uwb_real の typedef（ヘッダ）しか使っていないので、
# uwb_loc の .c は 1 本もリンクしない。
#
#   make            既定値 (UWB_MAX_ANCHORS=8) でビルド
#   make test       ビルドしてテスト実行
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#   make float      -DUWB_USE_FLOAT（単精度）でビルドして実行
#   make clean

CC     ?= cc
CFLAGS ?= -O2 -g
WARN    = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS  = -lm

ROOT        = ../..
UWB_LOC_DIR = $(ROOT)/components/uwb_loc
SURVEY_DIR  = $(ROOT)/components/uwb_survey

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
DEFS ?= -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8

INCLUDES = -I$(UWB_LOC_DIR)/include -I$(SURVEY_DIR)/include -I$(SURVEY_DIR)/src

SRC = $(SURVEY_DIR)/src/uwb_survey_dense.c \
      $(SURVEY_DIR)/src/uwb_survey.c

BUILD    = build
OBJ      = $(patsubst $(SURVEY_DIR)/src/%.c,$(BUILD)/%.o,$(SRC))
TEST_OBJ = $(BUILD)/test_survey.o
TEST_BIN = $(BUILD)/test_survey

.PHONY: all test strict float clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(SURVEY_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(TEST_OBJ): test_survey.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(TEST_BIN): $(OBJ) $(TEST_OBJ)
	$(CC) $(CFLAGS) $^ $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

# 警告をすべてエラーにして通すか見る。uwb_survey は全部自前実装なので
# uwb_loc のような third_party 由来の既知警告の除外は要らない。
strict:
	$(MAKE) clean
	$(MAKE) WARN="-Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -Wno-sign-conversion" test

# 単精度（uwb_real=float）でも破綻しないかを見る。復元精度の判定値は
# テスト側が UWB_REAL_IS_FLOAT を見て緩めている。
#
# 前後で clean するのは、DEFS を変えても .o のタイムスタンプは新しいままで
# 再コンパイルされず、単精度の .o と倍精度の .o が混ざると uwb_real の
# サイズ違いで壊れるため（デバッグに時間を溶かす類の罠）。
float:
	$(MAKE) clean
	$(MAKE) DEFS="-DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8 -DUWB_USE_FLOAT=1" test
	$(MAKE) clean

clean:
	rm -rf $(BUILD)
