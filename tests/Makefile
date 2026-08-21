# ホスト側テスト（tests/host/*/）を一括で回すための Makefile。
#
# tests/host/*/Makefile を自動検出するので、新しいサブディレクトリ
# （例: tests/host/math/）が増えてもこのファイル自体の変更は不要。
# 各サブディレクトリの Makefile はどれも `test` / `strict` / `clean` を
# 持つ前提（tests/host/loc・tests/host/pipeline・tests/host/survey で確認済み）。
# `float`（単精度ビルド）は持たないサブディレクトリもあるので、
# `grep -q '^float:' <dir>/Makefile` で有無を確認してから実行する。
#
#   make test     全サブディレクトリで `make test`
#   make strict   全サブディレクトリで `make strict`（-Werror 系）
#   make float    `float` ターゲットを持つサブディレクトリだけで `make float`
#   make clean    全サブディレクトリで `make clean`
#   make all      test → strict → float の順で実行
#
# 1つでも失敗したら非ゼロで終了する（set -e + pipefail）。
#
# 各テストの出力形式は揃っていない
# （pipeline/survey は「=== N 件中 M 件失敗 ===」、loc は
#   「OK  N 件すべて通った」/「NG  M / N 件が失敗」）ので、
# 件数を横断集計する代わりに、`test` の最後に各サブディレクトリの
# 末尾行を並べて表示する。

SHELL := /bin/bash

DIRS := $(patsubst host/%/Makefile,%,$(wildcard host/*/Makefile))

.PHONY: all test strict float clean

test:
	@set -euo pipefail; \
	logdir=$$(mktemp -d); \
	for d in $(DIRS); do \
		echo "=== test: $$d ==="; \
		$(MAKE) -C host/$$d test 2>&1 | tee "$$logdir/$$d.log"; \
		echo; \
	done; \
	echo "=== summary ==="; \
	for d in $(DIRS); do \
		printf '%-10s %s\n' "$$d" "$$(tail -n1 "$$logdir/$$d.log")"; \
	done; \
	rm -rf "$$logdir"

strict:
	@set -euo pipefail; \
	for d in $(DIRS); do \
		echo "=== strict: $$d ==="; \
		$(MAKE) -C host/$$d strict; \
	done

float:
	@set -euo pipefail; \
	for d in $(DIRS); do \
		if grep -q '^float:' host/$$d/Makefile; then \
			echo "=== float: $$d ==="; \
			$(MAKE) -C host/$$d float; \
		else \
			echo "=== float: $$d (float ターゲット無し。skip) ==="; \
		fi; \
	done

clean:
	@set -euo pipefail; \
	for d in $(DIRS); do \
		$(MAKE) -C host/$$d clean; \
	done

all: test strict float
