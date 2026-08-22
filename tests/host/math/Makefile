# uwb_math (components/uwb_math) のホストテスト。
#
# uwb_math は BLAS/LAPACK/malloc に依存しない純粋な C99 なので、
# third_party や上流クローンを介さず、コンポーネントのソースを
# そのままホストでコンパイルして検証する (tests/host/loc の後継流儀)。
#
#   make            既定値でビルドのみ (build/test_math を作る。実行はしない)
#   make test       ビルドしてテスト実行
#   make strict     -Werror を足して警告ゼロで通るか見る
#   make float      float 版 (-DUWB_USE_FLOAT) で作り直して試す (マイコン想定)
#   make sanitize   ASan/UBSan を足して未定義動作・メモリ破壊を検出する
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -g -MMD -MP
WARN     = -Wall -Wextra -pedantic -Wshadow -Wconversion
LDLIBS   = -lm

ROOT         = ../../..
UWB_MATH_DIR = $(ROOT)/components/uwb_math
TEST_SRC     = test_math.c

INCLUDES = -I$(UWB_MATH_DIR)/include

BUILD    = build
OBJ      = $(BUILD)/uwb_math.o
TEST_BIN = $(BUILD)/test_math

# -MMD -MP (CFLAGS) が $(BUILD)/*.d にヘッダ依存を書き出す。ヘッダだけ
# 直してもここで拾って再コンパイルされるので、古い .o のまま
# 「通ってはいけないものが通る」事故を防ぐ（初回ビルドでは .d が無いので
# 空リストになるだけで問題ない）。
-include $(wildcard $(BUILD)/*.d)

.PHONY: all test strict float sanitize clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/uwb_math.o: $(UWB_MATH_DIR)/src/uwb_math.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(INCLUDES) -std=c99 -c $< -o $@

$(TEST_BIN): $(TEST_SRC) $(OBJ)
	$(CC) $(CFLAGS) $(WARN) $(INCLUDES) -std=c99 $(TEST_SRC) $(OBJ) $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

# 警告をすべてエラーにして通すか見る。
strict:
	$(MAKE) clean
	$(MAKE) WARN="-Wall -Wextra -pedantic -Wshadow -Wconversion -Werror" test

# 単精度でも通るか。マイコンでは float の方が速い/小さいことが多い。
# テスト側は uwb_real に入れるリテラルを必ず (uwb_real) キャストしてあるので
# WARN はそのまま (-Wconversion を外さない)。
float:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O2 -g -DUWB_USE_FLOAT" test

# ASan/UBSan で未定義動作・メモリ破壊を検出する。CFLAGS はコンパイルと
# リンクの両方の recipe に載るので、-fsanitize は自動的に両方に付く。
sanitize:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" test

clean:
	rm -rf $(BUILD)
