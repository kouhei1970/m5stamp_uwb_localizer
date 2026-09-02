# tools/

ベンチマークと補助スクリプトの置き場です。**ホスト側テストは
[`tests/`](../tests/) を見てください**（`tests/host/loc` / `tests/host/pipeline` /
`tests/host/survey`、`make -C tests test strict float` で一括実行）。

| ディレクトリ | 内容 |
|---|---|
| [`bench_loc/`](bench_loc/) | 測位ソルバ（`components/uwb_loc`）のマイクロベンチマーク（`make bench`） |
| [`serial/`](serial/) | 実機のシリアルログ採取・集計スクリプト（`capture.py` / `capture_reenum.py` / `twr_stats.py`） |
| [`ekf_sweep.py`](ekf_sweep.py) | タグの EKF（`components/uwb_loc/src/uwb_ekf.c`）を PC 上で再現し、記録済みの WebSocket キャプチャに対して Q（`sigma_a`）・R（`sigma_r`）・棄却ゲートを掃引する。`python3 tools/ekf_sweep.py <raw> --sigma-a 0.2,0.5 --sigma-r 0.05,0.1 --gate 3,4`、`--validate` で実機の lv3 出力と突き合わせ |
