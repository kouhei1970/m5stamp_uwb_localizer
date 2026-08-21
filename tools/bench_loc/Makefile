# uwb_loc (components/uwb_loc) のマイクロベンチマーク。
#
# 上流 uwb_localizer の c/bench/bench_uwb.c を 2026-08-21 の最終同期で
# 移設したもの。tests/host/loc と同じ考え方で、third_party や上流クローン
# には依存せず本リポジトリ内だけで完結する。
#
#   make              既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8。
#                     Kconfig の既定と同じ) でビルド
#   make bench        同上でビルドして実行 (結果は表 + bench_result.tsv)
#   make bench-float  float 版 (-DUWB_USE_FLOAT) で同上
#   make bench-build  コンパイルのみ (実行はしない)
#   make clean

CC      ?= cc
CFLAGS  ?= -O2 -g
WARN     = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
LDLIBS   = -lm

ROOT        = ../..
UWB_LOC_DIR  = $(ROOT)/components/uwb_loc
UWB_MATH_DIR = $(ROOT)/components/uwb_math
BENCH_SRC    = bench_uwb.c

# uwb_internal.h (uwb_gdop_from_jac を直接計るため) と uwb_math.h も見せる。
INCLUDES = -I$(UWB_LOC_DIR)/include -I$(UWB_LOC_DIR)/src -I$(UWB_MATH_DIR)/include

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
ANCHORS8_DEFS = -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8

# 線形代数は components/uwb_math に移った (uwb_linalg.c は削除済み)。
SRC = $(UWB_LOC_DIR)/src/uwb_model.c \
      $(UWB_LOC_DIR)/src/uwb_closed_form.c \
      $(UWB_LOC_DIR)/src/uwb_nls.c \
      $(UWB_LOC_DIR)/src/uwb_ekf.c \
      $(UWB_MATH_DIR)/src/uwb_math.c

BUILD = build
OBJ   = $(patsubst %.c,$(BUILD)/%.o,$(notdir $(SRC)))
LIB   = $(BUILD)/libuwbloc_components.a
BENCH_BIN = $(BUILD)/bench_uwb

DEFS ?= $(ANCHORS8_DEFS)

.PHONY: all bench bench-float bench-build clean

all: $(LIB)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(UWB_LOC_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(BUILD)/%.o: $(UWB_MATH_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(LIB): $(OBJ)
	$(AR) rcs $@ $(OBJ)

$(BENCH_BIN): $(BENCH_SRC) $(LIB)
	$(CC) $(CFLAGS) $(WARN) $(DEFS) $(INCLUDES) -std=c99 $(BENCH_SRC) $(LIB) $(LDLIBS) -o $@

bench-build: $(BENCH_BIN)

bench: $(BENCH_BIN)
	./$(BENCH_BIN)

# 単精度版 (マイコン想定)。-Wconversion を外すのはテストと同じ理由
# (アンカー座標が double リテラル)。
bench-float:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O2 -g -DUWB_USE_FLOAT" WARN="-Wall -Wextra -pedantic -Wshadow" bench

clean:
	rm -rf $(BUILD) bench_result.tsv
