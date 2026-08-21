# ビルド済みバイナリを使う（ESP-IDF なしで試す）

GitHub Actions が全ファームをビルドし、**そのまま書き込める `merged-firmware.bin`**
を作っています。ESP-IDF（インストールに 30 分ほどかかります）を入れずに実機を試せます。

---

## ⚠ 先に読んでください — 動かない可能性がある条件

> **ピン定義（`boards/*.h`）は実機未検証の暫定値です。**
> ビルド済みバイナリはその値を**焼き込んだ**ものなので、
> **あなたの配線が [`GETTING_STARTED.md` §3](GETTING_STARTED.md#wiring) と違えば動きません。**

配線を変えたい場合は、`boards/*.h` を直して**自分でビルドしてください**
（[`GETTING_STARTED.md` §2](GETTING_STARTED.md#setup)）。

また、**このリポジトリのコードは実機で一度も動作確認していません。**
ビルド済みバイナリは「ビルドが通ったもの」であって「動作確認済みのもの」ではありません。

---

## どれを取ればいいか

| やりたいこと | 取るもの | 対応する実験 |
|---|---|---|
| **まず SPI が通じるか見る** | `probe-*`（ボードに合うもの） | [実験1](EXPERIMENT_PLAN.md) |
| 2台で距離を測る | `twr-tag-ss` + `twr-anchor-ss` | 実験2 |
| 同上（本番方式） | `twr-tag-ds` + `twr-anchor-ds` | 実験3 |
| **5台のアンカー + タグで測位** | `anchor-*-ds` ×1（5台に同じものを書く）+ `tag-*-ds` | 実験5・6 |
| 速くする（59 Hz） | `anchor-*-ds-fast` + `tag-*-ds-fast` | 実験7・8 |

### ボードの選び方

| 手元のボード | 選ぶ variant |
|---|---|
| M5StampS3A | `*-stamps3` |
| **AtomS3R（現行）** | `*-atoms3-pinoutB` |
| 無印 AtomS3（在庫限り） | `*-atoms3-pinoutA` |
| StampFly に載せる | `*-stampfly` |

構成 A / B の違いは [`IRQ_POLICY.md`](IRQ_POLICY.md) を参照。
**AtomS3R は G38/G39 が空いているので構成 B（ToF を Grove に挿すだけ）が綺麗**です。

### `-fast` は必ずペアで使う

`anchor-*-ds-fast` と `tag-*-ds-fast` は**必ず両方**書き込んでください。
**遅延プリセットはタグとアンカーで一致していないと測距が成立しません**
（[`TIMING_PRESETS.md`](TIMING_PRESETS.md)）。
片方だけ焼くと起動ログに不一致の警告が出ます。

---

## 取得方法

| 経路 | 用途 |
|---|---|
| **Releases** | タグが打たれた版。安定して同じものが取れる |
| **Actions の artifact** | 最新の `main` の版。90 日で消える。GitHub のログインが要る |

Actions からの取り方: リポジトリの **Actions** タブ → `build` ワークフローの成功した実行を開く
→ ページ下部の **Artifacts** から variant 名の zip をダウンロード。

---

## 書き込み方

`merged-firmware.bin` は**ブートローダ・パーティションテーブル・アプリを1つに結合した
イメージ**です。オフセット `0x0` に書くだけで完結します。

### 方法 A: `esptool`（コマンドライン）

```sh
pip install esptool

# ポートを調べる（macOS）
ls /dev/cu.usbmodem*        # Linux は ls /dev/ttyACM*

esptool.py --chip esp32s3 -p /dev/cu.usbmodemXXXX write_flash 0x0 merged-firmware.bin
```

### 方法 B: ブラウザ（インストール不要）

<https://espressif.github.io/esptool-js/> を Chrome / Edge で開き、

1. **Connect** でポートを選ぶ
2. ファイルに `merged-firmware.bin`、**Flash Address に `0x0`** を指定
3. **Program**

> **書き込みモードに入れないとき**: M5StampS3A は中央のボタンを押しながら USB を挿す、
> AtomS3 は側面のリセットボタンを 2 秒ほど長押しします
> （**この操作は本リポジトリでは実機確認していません**）。

### 出力を見る

```sh
# esptool で書いた場合、モニタは別途必要
pip install esp-idf-monitor
esp-idf-monitor -p /dev/cu.usbmodemXXXX
```
`screen /dev/cu.usbmodemXXXX 115200` でも読めます（抜けるのは `Ctrl-A` `K`）。

期待される出力は [`BRINGUP.md`](BRINGUP.md) と [`GETTING_STARTED.md`](GETTING_STARTED.md) を参照。

---

## ビルド済みでも**実行時に変えられる**もの

ここが重要です。**アンカー5台に同じバイナリを書いて構いません。**
個体差はシリアルコンソールで設定し、NVS に保存されます。

| 設定 | コマンド | ファーム |
|---|---|---|
| アンカーのショートアドレス | `addr set 0x0003` → `save` | `anchor` |
| アンカーの座標 | `anchor set <idx> <hex_addr> <x> <y> <z>` → `save` | `tag` |
| アンテナ遅延 | `anchor delay <idx> <meters>` → `save` | `tag` |
| アンカーの有効/無効・台数 | `anchor enable/disable <idx>` / `anchor count <n>` | `tag` |
| JSON 出力の一時停止 | `output off` / `output on` | `tag` |
| 設定の確認 | `info` / `anchor list` | 両方 |

詳細は [`GETTING_STARTED.md` §6.2](GETTING_STARTED.md#anchors5-console) と §7。

## ビルド済みでは**変えられない**もの

再ビルドが必要です。

- **ピン割り当て**（`boards/*.h`）
- ボードの種類、SS-TWR / DS-TWR の別
- **IRQ の有効/無効、遅延プリセット**（`-fast` 版として別に用意してあります）
- SPI クロック、EKF の有効化、2D フォールバックの挙動

これらを触るときは [`GETTING_STARTED.md` §2](GETTING_STARTED.md#setup) の手順で
ESP-IDF を入れて自分でビルドしてください。

---

## CI が何をしているか

`.github/workflows/build.yml`:

1. **ホスト側テスト**（`test_pipeline` / `test_survey` / `test_uwb_loc`）を実行
   ※ `test_uwb_loc` は上流 `uwb_localizer` のテストコード本体を参照するので、CI が別途 clone します
2. **14 通りのファーム**を ESP-IDF v5.5.2 でビルドし、`idf.py merge-bin` で結合
3. artifact として保存し、**タグを打った時は Release に添付**

各 artifact には `kconfig-used.txt`（そのバイナリに焼き込まれた設定）と
`README.md`（書き込み手順）が同梱されます。**どの設定でビルドされたものか必ず確認できます。**

---

## 関連文書
- [`EXPERIMENT_PLAN.md`](EXPERIMENT_PLAN.md) — どの順で試すか
- [`GETTING_STARTED.md`](GETTING_STARTED.md) — 配線から測位までの完全手順
- [`BRINGUP.md`](BRINGUP.md) — 実験1（SPI 疎通）の受入基準
- [`TIMING_PRESETS.md`](TIMING_PRESETS.md) — `-fast` 版で何が変わるか
