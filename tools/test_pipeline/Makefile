# components/uwb_ranging のハード非依存部分（uwb_ranging_anchor_table.cpp /
# uwb_ranging_pipeline.cpp）+ components/uwb_loc を、実機なしでホスト上で
# 直接ビルド・実行して検証するための Makefile。
#
# tools/test_uwb_loc/Makefile と同じ考え方（third_party は読み取り専用、
# ビルドするのは本番と同じ components/ 配下のソース）。ただし今回は
# C（uwb_loc）と C++（uwb_ranging）の混在なので、.c は cc、.cpp は c++ で
# それぞれコンパイルしてから link する。
#
# ハード依存部分（components/uwb_ranging/src/uwb_ranging_scheduler.cpp、
# components/uwb_qm33120 経由でTWRを叩く方）は意図的にビルド対象に**含めない**。
# ホストにはUWBチップもESP-IDFも無いので、これは正しい切り分け
# （PLAN.md/委譲指示にある「ハード依存を切り離せる設計」の実体）。
#
# 例外: components/uwb_qm33120/include/uwb_qm33120_units.hpp
# (uwb::detail::usToUus() / sfdTimeoutFromPhy()) はESP-IDF非依存のヘッダ
# のみ（<cstdint>のみ）なので、include pathに足してこのバイナリからも
# 直接検算する（docs/REIMPL_PLAN.md R1/R8）。
#
#   make            既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8) でビルド
#   make test       同上でビルドしてテスト実行（uwb_ranging 6シナリオ +
#                    uwb_qm33120_units R1/R8 検算）
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#   make clean

CC       ?= cc
CXX      ?= c++
CFLAGS   ?= -O2 -g
CXXFLAGS ?= -O2 -g -std=c++17
WARN_C    = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
WARN_CXX  = -Wall -Wextra -Wshadow
LDLIBS    = -lm

ROOT         = ../..
UWB_LOC_DIR  = $(ROOT)/components/uwb_loc
RANGING_DIR  = $(ROOT)/components/uwb_ranging
QM33120_DIR  = $(ROOT)/components/uwb_qm33120

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
ANCHORS8_DEFS = -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8
DEFS ?= $(ANCHORS8_DEFS)

INCLUDES = -I$(UWB_LOC_DIR)/include -I$(UWB_LOC_DIR)/src -I$(RANGING_DIR)/include -I$(QM33120_DIR)/include

C_SRC = $(UWB_LOC_DIR)/src/uwb_linalg.c \
        $(UWB_LOC_DIR)/src/uwb_model.c \
        $(UWB_LOC_DIR)/src/uwb_closed_form.c \
        $(UWB_LOC_DIR)/src/uwb_nls.c \
        $(UWB_LOC_DIR)/src/uwb_ekf.c

CXX_SRC = $(RANGING_DIR)/src/uwb_ranging_anchor_table.cpp \
          $(RANGING_DIR)/src/uwb_ranging_pipeline.cpp

BUILD = build
C_OBJ   = $(patsubst $(UWB_LOC_DIR)/src/%.c,$(BUILD)/%.o,$(C_SRC))
CXX_OBJ = $(patsubst $(RANGING_DIR)/src/%.cpp,$(BUILD)/%.o,$(CXX_SRC))
TEST_OBJ = $(BUILD)/test_pipeline.o
TEST_BIN = $(BUILD)/test_pipeline

.PHONY: all test strict clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(UWB_LOC_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN_C) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(BUILD)/%.o: $(RANGING_DIR)/src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(DEFS) $(INCLUDES) -c $< -o $@

$(TEST_OBJ): test_pipeline.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(DEFS) $(INCLUDES) -c $< -o $@

$(TEST_BIN): $(C_OBJ) $(CXX_OBJ) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

# 警告をすべてエラーにして通すか見る。uwb_loc 側は third_party 由来の
# 既知の未使用変数 (uwb_nls.c:342, PROGRESS.md 記載の上流バグ) があるため
# -Werror は uwb_ranging 側 (自前実装分) にだけ掛ける。
strict:
	$(MAKE) clean
	$(MAKE) WARN_CXX="-Wall -Wextra -Wshadow -Werror" test

clean:
	rm -rf $(BUILD)
