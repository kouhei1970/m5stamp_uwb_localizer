# tools/serial/

実機のシリアルログを採取・集計するための補助スクリプト置き場です。
ESP32-S3 のファーム（`firmware/probe` / `firmware/twr` など）のシリアル
出力を録って、あとから成功率などを機械的に集計するために使います。

## スクリプト

| スクリプト | 用途 |
|---|---|
| `capture.py` | ESP-IDF ファーム（USB-Serial/JTAG コンソール）用。DTR/RTS でリセットしてポートを開いたまま採取し、`PROBE SUMMARY` 行が出たら早めに打ち切る |
| `capture_reenum.py` | Arduino HWCDC（Hardware CDC）ファーム用。リセットで USB が再列挙されるため、いったんポートを閉じて再接続してから採取する |
| `twr_stats.py` | `firmware/twr` タグ側ログを要約する（成功率 / `rsl_dbm`・`fp_dbm` 中央値 / `clock_ppm` / 失敗種別内訳 / 測距値） |

## 使い方

```sh
# ESP-IDF ファーム（probe / twr など）。60 秒採取、PROBE SUMMARY が出たら早期終了
python3 capture.py /dev/cu.usbmodem101 out.log 60

# リセットせずにそのまま採取したい場合
python3 capture.py /dev/cu.usbmodem101 out.log 60 noreset

# Arduino HWCDC ファーム（例: 純正 M5Stack ライブラリの examples）。90 秒採取
python3 capture_reenum.py /dev/cu.usbmodem101 out.log 90

# twr タグログの要約
python3 twr_stats.py out.log
```

## 注意点（実機で踏んだ落とし穴）

- **DTR/RTS リセット + ポートを開いたまま**という `capture.py` の方式は
  ESP-IDF ファームではほぼ動くが、USB-Serial/JTAG コンソールが
  再オープンしたハンドルへの出力配信をまれに 1 分ほど止めることがある
  （見た目はハングだが、ボード自体は正常に動いている）。出力が来ない
  ときはまず数十秒待ってから疑うこと。
- **Arduino の HWCDC ファームはリセットで USB デバイスが再列挙される。**
  `capture.py` のようにポートを開いたまま待つ方式では追随できないため、
  必ず `capture_reenum.py`（いったん閉じて再オープンする）を使うこと。
- 採取した生ログはリポジトリに含めない一時ファイルとして扱うこと。
  分析結果の要点は `docs/HANDOFF.md` 側に書き写して残す運用にしている。
