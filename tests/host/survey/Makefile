# components/uwb_survey（アンカー自動測量の計算本体）を、実機なしで
# ホスト上で直接ビルド・実行して検証するための Makefile。
#
# tests/host/pipeline/Makefile と同じ考え方（third_party は読み取り専用、
# ビルドするのは本番と同じ components/ 配下のソース）。uwb_survey は
# 全部 C99 なので、こちらは cc だけで済む（旧 tools/test_survey の後継）。
#
# 線形代数は components/uwb_math に依存するので、その .c も一緒に
# コンパイルする（uwb_real の typedef もそこから来る。uwb_loc は不要）。
#
#   make            既定値 (UWB_MAX_ANCHORS=8) でビルド
#   make test       ビルドしてテスト実行
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#   make float      -DUWB_USE_FLOAT（単精度）でビルドして実行
#   make sanitize   ASan/UBSan 付きでビルドして実行
#   make clean

CC     ?= cc
CFLAGS ?= -O2 -g
WARN    = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS  = -lm

ROOT         = ../../..
UWB_MATH_DIR = $(ROOT)/components/uwb_math
SURVEY_DIR   = $(ROOT)/components/uwb_survey

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値（uwb_survey 自体は
# 参照しないが、uwb_loc.h を同居させる将来の変更に備えて残してある）。
DEFS ?= -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8

INCLUDES = -I$(UWB_MATH_DIR)/include -I$(SURVEY_DIR)/include

SRC = $(SURVEY_DIR)/src/uwb_survey.c \
      $(UWB_MATH_DIR)/src/uwb_math.c

BUILD    = build
OBJ      = $(BUILD)/uwb_survey.o $(BUILD)/uwb_math.o
TEST_OBJ = $(BUILD)/test_survey.o
TEST_BIN = $(BUILD)/test_survey

.PHONY: all test strict float sanitize clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/uwb_survey.o: $(SURVEY_DIR)/src/uwb_survey.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(BUILD)/uwb_math.o: $(UWB_MATH_DIR)/src/uwb_math.c | $(BUILD)
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

# ASan/UBSan で未定義動作・メモリ破壊を検出する。CFLAGS はコンパイルと
# リンクの両方の recipe に載るので、-fsanitize は自動的に両方に付く。
sanitize:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" test
	$(MAKE) clean

clean:
	rm -rf $(BUILD)
