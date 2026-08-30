# components/uwb_qm33120/include/uwb_qm33120_responder_fsm.hpp
# （uwb::decide() の遷移表、docs/ARCHITECTURE_V2.md §2.2/§2.3）を実機なしで
# ホスト上で直接ビルド・実行して検証するための Makefile。
#
# tests/host/pipeline/Makefile と同じ考え方。responder_fsm.hpp はヘッダのみ
# （ESP-IDF/Qorvo SDK非依存、<cstdint> + uwb_qm33120_twr_config.hpp +
# uwb_qm33120_distance_stats.hpp のみに依存）なので、コンパイル対象は
# test_responder_fsm.cpp 1ファイルだけで済む。
#
#   make test     ビルドしてテスト実行
#   make strict   -Wall -Wextra -Werror -pedantic で通るか見る
#   make clean

CXX      ?= c++
CXXFLAGS ?= -O2 -g -std=c++17 -MMD -MP
WARN_CXX  = -Wall -Wextra -Wshadow

ROOT        = ../../..
QM33120_DIR = $(ROOT)/components/uwb_qm33120

INCLUDES = -I$(QM33120_DIR)/include

BUILD    = build
TEST_OBJ = $(BUILD)/test_responder_fsm.o
TEST_BIN = $(BUILD)/test_responder_fsm

-include $(wildcard $(BUILD)/*.d)

.PHONY: all test strict clean

all: $(TEST_BIN)

$(BUILD):
	mkdir -p $(BUILD)

$(TEST_OBJ): test_responder_fsm.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) $(WARN_CXX) $(INCLUDES) -c $< -o $@

$(TEST_BIN): $(TEST_OBJ)
	$(CXX) $(CXXFLAGS) $^ -o $@

test: $(TEST_BIN)
	./$(TEST_BIN)

strict:
	$(MAKE) clean
	$(MAKE) WARN_CXX="-Wall -Wextra -Wshadow -Wpedantic -Werror" test

clean:
	rm -rf $(BUILD)
