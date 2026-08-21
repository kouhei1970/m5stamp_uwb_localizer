# 立ち上げ手順 (Phase 1: SPI 疎通の受入確認)

## この文書の位置づけ

![公式ピンマップ](../assets/S017_Stamp_UWB_pinmap.jpg)

**半田パッドと FPC ケーブルは信号の並びが違う。** 側面ラベルがパッド、下部の帯が FPC。


**`firmware/probe` を書き込み、M5Stamp UWB Module（通称 M5Stamp UWB）の Device ID `0xDECA0314` が読めることを
確認する** ——ここだけに絞った受入確認手順です。
これが通れば、以降の TWR 移植（Phase 2）はソフトウェアだけの話になります。

| 知りたいこと | 読むもの |
|---|---|
| **買ってから測位が出るまでの通し手順**（配線・5 台への書き込み・座標入力・測位・校正） | **[`GETTING_STARTED.md`](GETTING_STARTED.md)** |
| **半田パッドの寸法・向きの確定・半田付け手順・アンテナ禁止領域** | [`SOLDER_PADS.md`](SOLDER_PADS.md) |
| モジュールのハードウェア仕様 | [`archive/SURVEY_m5stamp_uwb_module.md`](archive/SURVEY_m5stamp_uwb_module.md) |
| **Phase 1 の受入確認そのもの** | **本書** |

---

## 用意するもの

- **M5StampS3A** または **M5 AtomS3**（ESP32-S3 ホストボード）
- **M5Stamp UWB Module (QM33120W)**（SKU `S017`）<br>  または **M5Stamp UWB Module with FPC (QM33120W)**（SKU `S017-F`）。**どちらでも可**
- 接続手段（下記）
- **3.3V 電源**（モジュールは 3.3V 単一供給。**5V は入れないこと**）
- ESP-IDF **v5.5.2**

---

## 接続方法は 2 通りある

モジュールには **12P FPC コネクタ**（`S017-F` のみ）と、
**基板側面の半田付けパッド（キャステレーション 12 個）**（両方にある）が出ています。
信号は同一で、**パッド番号 = FPC ピン番号**です。

| | A. FPC を使う | B. 半田パッドを使う |
|---|---|---|
| 対象 | `S017-F`（コネクタ実装済み） | `S017` / `S017-F` の両方 |
| 相手側 | 0.5mm 12P FPC ブレークアウト基板が要る | 線を直付け |
| 着脱 | 可 | 不可（付け外しは半田ごて） |
| 実験段階 | 変換基板が手元にあるなら楽 | **手元の部品だけで始められる。本プロジェクトはこちら** |

**B を選ぶ場合**（寸法・**向きの判定**・半田付け手順・アンテナ禁止領域）:
→ [`SOLDER_PADS.md`](SOLDER_PADS.md)、手順としての展開は
[`GETTING_STARTED.md` §3](GETTING_STARTED.md#wiring)

> **⚠️ 配線前に必ず向きを確定すること。**
> 半田パッドの pin 1 がどちら側かは**実物で確認が必要**です（公式 PINMAP 画像を
> 入手できていないため）。テスターによる導通試験の手順が
> [`GETTING_STARTED.md` §3.1](GETTING_STARTED.md#orientation) にあります。
> **間違えると電源逆接でモジュールが壊れます。**

---

## 配線（信号名は A / B 共通）

### モジュール側 12P（FPC ピン番号 = 半田パッド番号。固定）

| Pin | 信号 | Pin | 信号 |
|---|---|---|---|
| 1 | GND | 7 | DW_CDO (MISO) |
| 2 | **VCC_3V3** | 8 | GND |
| 3 | DW_WAKEUP | 9 | DW_CDI (MOSI) |
| 4 | DW_IRQ | 10 | DW_CSn |
| 5 | DW_GP7 | 11 | DW_CLK (SCK) |
| 6 | DW_RSTn | 12 | GND |

**Phase 1 の最小配線は 6 本**（GND / VCC_3V3 / CLK / CDI / CDO / CSn）。
ドライバはポーリング方式なので IRQ は無くても動きます。
**`DW_RSTn`（pin 6）は配線を強く推奨**（`uwb_port_hard_reset()` が使えるようになる）。

半田パッドの場合、**SPI 4 本 + GND 2 本がすべて右列（pin 7〜12）にまとまっている**ので、
右列 6 本 + 左列の 3V3 (pin 2) + RSTn (pin 6) の**計 8 本**が実用上の最小構成です。

### ホスト側ピン（**暫定値。実配線前に必ず現物と照合すること**）

| | SCK | MOSI | MISO | CS | RST | IRQ | WAKEUP | GP7 |
|---|---|---|---|---|---|---|---|---|
| M5StampS3A | G12 | G11 | G13 | G10 | G6 | G7 | G8 | 未配線 |
| AtomS3 | G7 | G6 | G5 | G8 | G1 | G2 | 未配線 | 未配線 |

定義は [`boards/stamps3.h`](../boards/stamps3.h) / [`boards/atoms3.h`](../boards/atoms3.h)。
**実配線を変えたらこの 2 ファイルも本表も直すこと。**

**注意**

- `DW_RSTn` はホストが能動的に High へ駆動しません（**モジュール側プルアップ前提**）。
  リセット時のみ Low に引き、それ以外は入力（ハイインピーダンス）にしています。
  モジュール側プルアップの有無は**未確認**なので、疎通が取れないときは
  ハーネス上で 10kΩ プルアップ（pin 6 ↔ pin 2）を入れて切り分けてください。
- 配線が長い／ジャンパ線だと 16MHz で信号品質が問題になり得ます。
  疑わしければ `boards/*.h` の `spi_fast_hz` を 8MHz や 4MHz に落として切り分けます。
  **ただし `firmware/probe` は `dwt_probe()`/`dwt_readdevid()` までしか行わず
  `Qm33120::begin()` を呼ばないため、`spi_fast_hz` へは一度も切り替わらず
  常に `spi_slow_hz`（既定 2MHz）のままです。** `spi_fast_hz` を変えても
  `firmware/probe` の挙動は変わりません。16MHz での実挙動を切り分けたい
  場合は `firmware/twr`（または `firmware/tag`/`firmware/anchor`）を使い、
  `begin()` 成功時のログ `spi: slow=... Hz fast=... Hz active=... Hz`
  （`components/uwb_qm33120/src/uwb_qm33120.cpp` `Qm33120::begin()`）で
  `active` が `spi_fast_hz` と一致しているか確認してください。
- **アンテナ側（pin 1 / pin 12 のある端。角が 2mm 面取りされている側）の端から
  3.5mm には金属・グランド・配線を置かないこと。**
  公式フットプリントに表裏とも銅箔禁止領域として定義されています
  （[`SOLDER_PADS.md`](SOLDER_PADS.md) §1.5）。
  疎通確認には効きませんが**測距距離に効きます**。

---

## ビルドと書き込み

```sh
. ~/esp/esp-idf/export.sh
cd firmware/probe

idf.py set-target esp32s3

# ボード選択（既定は M5StampS3A）
idf.py menuconfig      # → UWB Probe Configuration → Target host board

idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor    # macOS。Linux は /dev/ttyACM*
```

`monitor` を抜けるには `Ctrl-]`。

---

## 期待される出力（受入基準）

```
I (xxx) uwb_probe: Phase 1 UWB probe acceptance test, board=M5StampS3A
I (xxx) uwb_probe: L1: raw DEV_ID = 0xDECA0314 (expect 0xDECA0314) -> OK
I (xxx) uwb_probe: L2: dwt_probe + dwt_readdevid = 0xDECA0314 (expect 0xDECA0314) -> OK
I (xxx) uwb_probe: === L1: PASS / L2: PASS ===
I (xxx) uwb_probe: L1 (periodic): raw DEV_ID = 0xDECA0314
```

| チェック | 内容 |
|---|---|
| **L1** | 生の SPI で DEV_ID レジスタを読む（配線が合っているか） |
| **L2** | Qorvo SDK の `dwt_probe()` + `dwt_readdevid()`（ドライバが初期化できるか） |

**受入基準: L1 / L2 とも PASS し、その後 1 秒周期で繰り返される L1 の値が
`0xDECA0314` のまま安定していること**（1 分程度は観察する）。

---

## 切り分け

| 症状 | 疑うところ |
|---|---|
| `0x00000000` が返る | MISO が繋がっていない / CS が効いていない / 電源が来ていない |
| `0xFFFFFFFF` が返る | MISO がプルアップだけで浮いている / モジュール未給電 |
| 値が毎回バラつく | 配線長・GND の共通化。`spi_fast_hz` は `firmware/probe` には効かない（上の「注意」参照）ので、SPI クロックを疑うなら `spi_slow_hz` を落とすか `firmware/twr` で `active` ログを見る |
| L1 は OK だが L2 が失敗 | `dwt_probe()` 周り。SDK 側のリトライ・WAKEUP シーケンス。RSTn のプルアップを追加してみる |
| 全く反応しない | 3.3V の実測、FPC コネクタの挿し込み／半田付けの浮き、ピン番号の照合 |
| 隣の信号が連動して動く | （半田パッド時）ブリッジ。ピッチ 1.27mm なので 9-10-11-12 を重点確認 |
| 触ると値が変わる | （半田パッド時）パッド根元の剥がれ。[`SOLDER_PADS.md`](SOLDER_PADS.md) §4.6 |
| 電源投入直後にモジュールが熱い | **電源逆接**。ただちに外す |

さらに詳しい症状別の表は
[`GETTING_STARTED.md` §10](GETTING_STARTED.md#troubleshooting)。

---

## 確認できたら記録すること

- 実際に使ったピン番号（`boards/*.h` を実配線に合わせて更新する）
- 実際に使った配線長。`spi_fast_hz`（16MHz）が通ったかは
  `firmware/probe` では確認できない（上の「注意」参照）。`firmware/twr` の
  `begin()` ログの `active` 値で記録すること
- モジュールの消費電流
  （データシート値: スリープ 75.9µA / アンカー 5.23mA / タグ 58.0mA @3.3V）
- **半田パッドの向き**（pin 1 の位置、基板シルクにピン番号の印刷があるか）
  → [`SOLDER_PADS.md`](SOLDER_PADS.md) §5 の未確認リストを埋める
- 結果を [`archive/PROGRESS.md`](archive/PROGRESS.md) へ

---

## 次

**Phase 2（SS-TWR / DS-TWR による 1 対 1 の測距）**
→ [`GETTING_STARTED.md` §5](GETTING_STARTED.md#twr)

そのあとアンカー 5 台の準備・設置・測位へ続きます
（[`GETTING_STARTED.md` §6〜§9](GETTING_STARTED.md#anchors5)）。
