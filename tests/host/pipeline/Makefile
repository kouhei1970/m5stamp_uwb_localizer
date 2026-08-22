# components/uwb_ranging のハード非依存部分（uwb_ranging_anchor_table.cpp /
# uwb_ranging_pipeline.cpp）+ components/uwb_loc を、実機なしでホスト上で
# 直接ビルド・実行して検証するための Makefile。
#
# tests/host/loc/Makefile と同じ考え方（third_party には依存せず、
# ビルドするのは本番と同じ components/ 配下のソース）。ただし今回は
# C（uwb_loc）と C++（uwb_ranging）の混在なので、.c は cc、.cpp は c++ で
# それぞれコンパイルしてから link する（旧 tools/test_pipeline の後継）。
#
# ハード依存部分（components/uwb_ranging/src/uwb_ranging_scheduler.cpp、
# components/uwb_qm33120 経由でTWRを叩く方）は意図的にビルド対象に**含めない**。
# ホストにはUWBチップもESP-IDFも無いので、これは正しい切り分け
# （PLAN.md/委譲指示にある「ハード依存を切り離せる設計」の実体）。
#
# 例外: components/uwb_qm33120/include/uwb_qm33120_units.hpp
# (uwb::detail::usToUus() / sfdTimeoutFromPhy()) はESP-IDF非依存のヘッダ
# のみ（<cstdint>のみ）なので、include pathに足してこのバイナリからも
# 直接検算する（docs/archive/REIMPL_PLAN.md R1/R8）。
#
#   make            既定値 (UWB_MAX_ANCHORS=8 / UWB_MAX_MEAS=8) でビルド
#   make test       同上でビルドしてテスト実行（uwb_ranging 6シナリオ +
#                    uwb_qm33120_units R1/R8 検算 + uwb_cfgstore の
#                    シリアライズ往復・境界値・破損データ検算）
#   make strict     -Wall -Wextra -Werror -pedantic で通るか見る
#   make sanitize   ASan/UBSan 付きでビルドして実行
#   make clean

CC       ?= cc
CXX      ?= c++
CFLAGS   ?= -O2 -g -MMD -MP
CXXFLAGS ?= -O2 -g -std=c++17 -MMD -MP
WARN_C    = -Wall -Wextra -pedantic -Wshadow -Wconversion -Wno-sign-conversion
WARN_CXX  = -Wall -Wextra -Wshadow
LDLIBS    = -lm

ROOT         = ../../..
UWB_LOC_DIR  = $(ROOT)/components/uwb_loc
UWB_MATH_DIR = $(ROOT)/components/uwb_math
RANGING_DIR  = $(ROOT)/components/uwb_ranging
QM33120_DIR  = $(ROOT)/components/uwb_qm33120
CFGSTORE_DIR = $(ROOT)/components/uwb_cfgstore

# Kconfig の既定 (components/uwb_loc/Kconfig) と揃えた値。
ANCHORS8_DEFS = -DUWB_MAX_ANCHORS=8 -DUWB_MAX_MEAS=8
DEFS ?= $(ANCHORS8_DEFS)

INCLUDES = -I$(UWB_LOC_DIR)/include -I$(UWB_LOC_DIR)/src -I$(UWB_MATH_DIR)/include \
           -I$(RANGING_DIR)/include -I$(QM33120_DIR)/include -I$(CFGSTORE_DIR)/include

# uwb_loc の線形代数は components/uwb_math (uwb_loc.h が uwb_math.h を include
# する) に移ったので、その .c も一緒にビルドする。
C_SRC = $(UWB_LOC_DIR)/src/uwb_model.c \
        $(UWB_LOC_DIR)/src/uwb_closed_form.c \
        $(UWB_LOC_DIR)/src/uwb_nls.c \
        $(UWB_LOC_DIR)/src/uwb_ekf.c \
        $(UWB_MATH_DIR)/src/uwb_math.c

CXX_SRC = $(RANGING_DIR)/src/uwb_ranging_anchor_table.cpp \
          $(RANGING_DIR)/src/uwb_ranging_pipeline.cpp

# components/uwb_cfgstore のハード非依存部分（設定のシリアライズ/デシリアライズ）。
# NVS 入出力（src/uwb_cfgstore.cpp）は ESP-IDF 依存なので**含めない** —
# ここで検証したいのはバイト列の往復・境界値・壊れたデータの扱いであって、
# フラッシュの読み書きそのものではない（実機が無い段階で検証できるのはここまで）。
CFGSTORE_SRC = $(CFGSTORE_DIR)/src/uwb_cfgstore_blob.cpp

BUILD = build
C_OBJ   = $(patsubst %.c,$(BUILD)/%.o,$(notdir $(C_SRC)))
CXX_OBJ = $(patsubst $(RANGING_DIR)/src/%.cpp,$(BUILD)/%.o,$(CXX_SRC))
CFGSTORE_OBJ = $(patsubst $(CFGSTORE_DIR)/src/%.cpp,$(BUILD)/%.o,$(CFGSTORE_SRC))
TEST_OBJ = $(BUILD)/test_pipeline.o
TEST_BIN = $(BUILD)/test_pipeline

# -MMD -MP (CFLAGS/CXXFLAGS) が $(BUILD)/*.d にヘッダ依存を書き出す。
# ヘッダだけ直してもここで拾って再コンパイルされるので、古い .o のまま
# 「通ってはいけないものが通る」事故を防ぐ（初回ビルドでは .d が無いので
# 空リストになるだけで問題ない）。
-include $(wildcard $(BUILD)/*.d)

.PHONY: all test strict sanitize clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(BUILD)/%.o: $(UWB_LOC_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN_C) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(BUILD)/%.o: $(UWB_MATH_DIR)/src/%.c | $(BUILD)
	$(CC) $(CFLAGS) $(WARN_C) $(DEFS) $(INCLUDES) -std=c99 -c $< -o $@

$(BUILD)/%.o: $(RANGING_DIR)/src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(DEFS) $(INCLUDES) -c $< -o $@

$(BUILD)/%.o: $(CFGSTORE_DIR)/src/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(DEFS) $(INCLUDES) -c $< -o $@

$(TEST_OBJ): test_pipeline.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(DEFS) $(INCLUDES) -c $< -o $@

$(TEST_BIN): $(C_OBJ) $(CXX_OBJ) $(CFGSTORE_OBJ) $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) $^ $(LDLIBS) -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

# 警告をすべてエラーにして通すか見る。uwb_loc 側は third_party 由来の
# 既知の未使用変数 (uwb_nls.c:342, docs/archive/PROGRESS.md 記載の上流バグ)
# があるため -Werror は uwb_ranging 側 (自前実装分) にだけ掛ける。
strict:
	$(MAKE) clean
	$(MAKE) WARN_CXX="-Wall -Wextra -Wshadow -Werror" test

# ASan/UBSan で未定義動作・メモリ破壊を検出する。CFLAGS/CXXFLAGS はコンパイルと
# リンクの両方の recipe に載るので、-fsanitize は自動的に両方（C/C++ とも）に付く。
sanitize:
	$(MAKE) clean
	$(MAKE) CFLAGS="-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer" \
	        CXXFLAGS="-O1 -g -std=c++17 -fsanitize=address,undefined -fno-omit-frame-pointer" test
	$(MAKE) clean

clean:
	rm -rf $(BUILD)
