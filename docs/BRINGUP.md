# 立ち上げ手順 (Phase 1: SPI疎通確認)

## 目的
`firmware/probe` を書き込み、Stamp UWB-F の Device ID **`0xDECA0314`** が読めることを確認する。
これが通れば、以降の TWR 移植（Phase 2）はソフトウェアだけの話になる。

## 用意するもの
- M5Stamp S3（推奨）または AtomS3
- M5Stack **Stamp UWB-F**（FPCコネクタ実装済みの `S017-F`。ケーブル同梱）
- 12P 0.5mm FPC → 配線 の変換手段（ブレークアウト基板 or 直半田）
- **3.3V 電源**（モジュールは 3.3V 単一供給。5V は入れないこと）

## 配線

### モジュール側 12P FPC（固定）
| Pin | 信号 | Pin | 信号 |
|---|---|---|---|
| 1 | GND | 7 | DW_CDO (MISO) |
| 2 | **VCC_3V3** | 8 | GND |
| 3 | DW_WAKEUP | 9 | DW_CDI (MOSI) |
| 4 | DW_IRQ | 10 | DW_CSn |
| 5 | DW_GP7 | 11 | DW_CLK (SCK) |
| 6 | DW_RSTn | 12 | GND |

最小配線は **GND / VCC_3V3 / CLK / CDI / CDO / CSn の6本**。
ドライバはポーリング方式なので IRQ は無くても動く。

### ホスト側ピン（**暫定値。実配線前に必ず現物と照合すること**）
| | SCK | MOSI | MISO | CS | RST | IRQ | WAKEUP | GP7 |
|---|---|---|---|---|---|---|---|---|
| Stamp S3 | G12 | G11 | G13 | G10 | G6 | G7 | G8 | G9 |
| AtomS3 | G7 | G6 | G5 | G8 | G1 | G2 | 未配線 | 未配線 |

定義は `boards/stamps3.h` / `boards/atoms3.h`。変えたらここも直すこと。

**注意**
- `DW_RSTn` はホストが能動的に High へ駆動しない（モジュール側プルアップ前提）。
  リセット時のみ Low に引き、それ以外は入力（ハイインピーダンス）にしている
- 配線が長い/ジャンパ線だと 16MHz で信号品質が問題になり得る。
  疑わしければ `boards/*.h` の `spi_fast_hz` を 8MHz や 4MHz に落として切り分ける

## ビルドと書き込み
```sh
. ~/esp/esp-idf/export.sh
cd firmware/probe

# ボード選択（既定は Stamp S3）
idf.py menuconfig      # → UWB Probe Configuration → Target board

idf.py set-target esp32s3
idf.py build
idf.py -p /dev/tty.usbmodem* flash monitor
```

## 期待される出力
```
I (xxx) uwb_probe: L1: raw SPI DEV_ID = 0xDECA0314  OK
I (xxx) uwb_probe: L2: dwt_probe OK, dwt_readdevid = 0xDECA0314  OK
```
以降1秒周期で L1 の読み出しを繰り返し、値が安定していることを確認する。

## 切り分け

| 症状 | 疑うところ |
|---|---|
| `0x00000000` が返る | MISO が繋がっていない / CS が効いていない / 電源が来ていない |
| `0xFFFFFFFF` が返る | MISO がプルアップだけで浮いている / モジュール未給電 |
| 値が毎回バラつく | 配線長・SPIクロック（`spi_fast_hz` を落とす）・GND の共通化 |
| L1 は OK だが L2 が失敗 | `dwt_probe()` 周り。SDK 側のリトライ・WAKEUP シーケンスを確認 |
| 全く反応しない | 3.3V の実測、FPC コネクタの挿し込み、ピン番号の照合 |

## 確認できたら記録すること
- 実際に使ったピン番号（`boards/*.h` を実配線に合わせて更新）
- 動作した `spi_fast_hz`
- モジュールの消費電流（データシート値: スリープ 75.9uA / アンカー 5.23mA / タグ 58.0mA @3.3V）
- 結果を `PROGRESS.md` へ

→ ここまで通ったら **Phase 2（SS-TWR / DS-TWR の移植）** に進む。
