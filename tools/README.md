# tools/

ベンチマークと補助スクリプトの置き場です。**ホスト側テストは
[`tests/`](../tests/) を見てください**（`tests/host/loc` / `tests/host/pipeline` /
`tests/host/survey`、`make -C tests test strict float` で一括実行）。

| ディレクトリ | 内容 |
|---|---|
| [`bench_loc/`](bench_loc/) | 測位ソルバ（`components/uwb_loc`）のマイクロベンチマーク（`make bench`） |
| [`serial/`](serial/) | 実機のシリアルログ採取・集計スクリプト（`capture.py` / `capture_reenum.py` / `twr_stats.py`） |
