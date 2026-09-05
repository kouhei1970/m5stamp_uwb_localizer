# ビルド済みバイナリを使う（ESP-IDF なしで試す）

GitHub Actions が全ファームをビルドし、**そのまま書き込める `merged-firmware.bin`**
を作っています。ESP-IDF（インストールに 30 分ほどかかります）を入れずに実機を試せます。

---

## ⚠ 先に読んでください — 動かない可能性がある条件

> **実機確認済みのピン定義は M5StampS3A（`boards/stamps3.h`）のみです**
> （SPI 4 本・RST・IRQ〈Interrupt ReQuest、割り込み要求〉まで実機確認済み。
> WAKEUP と StampFly 用の定義は
> 未検証の暫定値）。ビルド済みバイナリはその値を**焼き込んだ**ものなので、
> **あなたの配線が [`GETTING_STARTED.md` §3](GETTING_STARTED.md#wiring) と違えば動きません。**

配線を変えたい場合は、`boards/*.h` を直して**自分でビルドしてください**
（[`GETTING_STARTED.md` §2](GETTING_STARTED.md#setup)）。

また、**アンカー 4 台以上での 3D 測位も含めて実機で動作確認済み**です
（1 対 1 測距・ブラウザダッシュボードに加え、2D 測位〈アンカー3台、2026-09-02〉・
3D 測位〈アンカー4台、2026-09-04〜05〉まで。詳細は [`HANDOFF.md`](HANDOFF.md) §1）。
**ただし、これらの実機確認はすべてローカルビルド（`idf.py build`）で行っており、
ビルド済みバイナリそのものでの動作確認はまだしていません。**

さらに、**利用条件（ライセンス）**: 本リポジトリのコードは MIT ですが、zip に
組み込まれている Qorvo QM33120W/DW3720 用ドライバは `LicenseRef-QORVO-2` で、
**Qorvo 製 IC（本モジュール = M5Stamp UWB Module に搭載された QM33120W/DW3720）
と共に使う場合に限り利用可能**です。他ベンダーのチップへの転用はできません。
このため各 zip には `LICENSE` / `THIRD_PARTY_LICENSES.md` /
`LICENSES/LicenseRef-QORVO-2.txt` を同梱しています（詳細は
[`../THIRD_PARTY_LICENSES.md`](../THIRD_PARTY_LICENSES.md)）。

---

## どれを取ればいいか

**Releases に添付しているのは、目的別の2つの zip だけです。**

| 何をしたいか | 取る zip | 書き込むファイル |
|---|---|---|
| 普通に測位したい（M5StampS3A のタグ 1 台 + アンカー N 台） | `1-positioning-M5StampS3A-tag-and-anchor.zip` | タグ → `tag-stamps3-ds/merged-firmware.bin`、アンカー全台 → `anchor-stamps3-ds/merged-firmware.bin`（全台同じ） |
| 配線した後に UWB モジュールと通信できるか確認したい | `2-wiring-check-probe-M5StampS3A.zip` | `probe-stamps3/merged-firmware.bin`（書き込んでシリアル端末を見る。合格したら上の行の zip へ） |

**まず bundle 2（`probe-stamps3`）で配線を疎通確認し、合格してから bundle 1
（`tag-stamps3-ds` / `anchor-stamps3-ds`）で測位に進む**ことを推奨します。
bundle 2 の合格基準・出力内容は [`GETTING_STARTED.md` §4.2](GETTING_STARTED.md#probe)
と同じです。

StampFly をタグにしたい、IRQ が使えない個体を切り分けたい、1対1測距だけを
診断したい、といった用途は Release の zip には含まれていません。
→ [開発者向け（Actions の artifact のみ）](#dev-only)を参照してください。

### 既定は IRQ 有効 + 遅延プリセット `PollingBoth`（約 31 Hz）

**`tag-stamps3-ds` / `anchor-stamps3-ds`（bundle 1 の中身）は、
IRQ を受信待ちの起床信号として使う設定
（`UWB_ENABLE_IRQ` 既定 `y`）と、遅延プリセット `PollingBoth`（`UWB_TIMING_PROFILE`
既定値、両側ポーリング相当の遅延。約 31 Hz）の組み合わせで焼いてあります。**
この組み合わせは 850 kbps・プリアンブル 256 と合わせて実機で確認済みで
（DS-TWR 周期成功率 99.5〜99.6%、2026-08-30）、**IRQ の極性・動作も
2026-08-29〜30 に実機で確認済みです**（IRQ 自己診断 `L5=PASS(irq=active)` を
2台とも確認、IRQ 有効のままの測距がポーリングと同等の成功率だったことを確認。
詳細は [`IRQ_POLICY.md`](IRQ_POLICY.md)・[`HANDOFF.md`](HANDOFF.md) §1）。

測距が全く出ない場合は、まず起動ログの `irq=` 行と `timing profile=` 行
（[`IRQ_POLICY.md`](IRQ_POLICY.md) の切り分け表）を確認してください。IRQ が
使えない個体を切り分けたい場合の polling 版は
[開発者向け](#dev-only)から取得できます。

---

<a id="dev-only"></a>

## 開発者向け（Actions の artifact のみ）

> ここから先は Release の2つの zip には**入っていない**構成の説明です。
> **Actions の artifact（12 通りの variant）からのみ**取得できます
> （[取得方法](#取得方法)参照）。普段タグ1台＋アンカーで測位するだけなら
> 読み飛ばして構いません。

### ボードの選び方

| 手元のボード | 選ぶ variant |
|---|---|
| **M5StampS3A + StampS3 BreakOut（標準構成）** | `*-stamps3` |
| StampFly に載せる | `*-stampfly` |

> **例外: `twr-anchor-ss` / `twr-anchor-ds` はボード名を含みません。**
> この2つは **M5StampS3A 用にビルドされたもの**です（標準構成が
> M5StampS3A + StampS3 BreakOut になったため）。

### `firmware/twr`（1対1診断）だけ既定が `BothIrq`

**遅延プリセットが旧既定の `BothIrq`（約 90 Hz）になることはありません**
（上記の bundle 1 は常に `PollingBoth`）。`BothIrq` は実機での素の成功率が
低く（6.8 Mbps 構成で 1.0%）、2026-08-30 に本番既定から外されました
（[`HANDOFF.md`](HANDOFF.md) §0-D「6.8 Mbps の切り分けと本番既定の決定」）。
`BothIrq` を今も既定に使うのは 1 対 1 診断用の `firmware/twr`＝
`twr-tag-*` / `twr-anchor-*` だけです。

### `-polling` 版が違うのは「IRQ を使うかどうか」だけ

`anchor-stamps3-ds-polling` / `tag-stamps3-ds-polling` / `tag-stampfly-ds-polling`
は、上の既定構成から**`UWB_ENABLE_IRQ` だけを `n`（無効）にした**ものです。
遅延プリセットはどれも `PollingBoth` のままなので、タグ・アンカーで IRQ の
有効/無効が食い違っていても遅延プリセットの不一致は起きません
（すべて `PollingBoth`）。`tag-stamps3-ds-polling` は M5StampS3A タグ用の
polling 版で、`anchor-stamps3-ds-polling` と組んで使います。

用意してある理由は、**個体によっては IRQ ピンが未配線・接触不良のことがあり得る**
ためです（IRQ が使えない場合は自動でポーリングへフォールバックしますが、
問題を切り分けたいときは明示的に無効化した版を使うと変数が減ります）。
「IRQ の極性が未検証だから」ではありません。

StampFly をタグにする場合は `tag-stampfly-ds`（既定・IRQ 有効）または
`tag-stampfly-ds-polling`（IRQ が使えない個体向け）を Actions artifact から
取得してください。アンカー側は上記の `anchor-stamps3-ds`（または
`anchor-stamps3-ds-polling`）を使います。

---

## 取得方法

| 経路 | 用途 | 単位 |
|---|---|---|
| **Releases** | タグが打たれた版。安定して同じものが取れる | **上記の2つの bundle だけ**（`1-positioning-M5StampS3A-tag-and-anchor.zip` / `2-wiring-check-probe-M5StampS3A.zip`） |
| **Actions の artifact** | 最新の `main` の版。90 日で消える。GitHub のログインが要る | **variant 単位**（`tag-stamps3-ds` 等、12 個。[開発者向け](#dev-only)の variant を含む全部） |

**Releases の2つの zip（bundle）は普段の利用に絞ったもの、Actions の artifact は
今まで通り variant 単位のまま**です。bundle の中の各サブフォルダは
Actions artifact の中身と同じ（`merged-firmware.bin` / `README.md` 等）なので、
Actions から取る場合は上表の「書き込むファイル」列の variant 名（例:
`tag-stamps3-ds`）と同じ名前の artifact を選んでください。StampFly 用・
polling 版・1対1診断用（`twr-*`）は Releases には無いので、
[開発者向け](#dev-only)の節にある variant 名を Actions artifact から探してください。

Actions からの取り方: リポジトリの **Actions** タブ → `build` ワークフローの成功した実行を開く
→ ページ下部の **Artifacts** から variant 名の zip をダウンロード。

---

## 書き込み方

`merged-firmware.bin` は**ブートローダ・パーティションテーブル・アプリを1つに結合した
イメージ**です。フラッシュ設定（フラッシュサイズ・モード・周波数）もこのファイルに
埋め込み済みなので、**オフセット `0x0` に書くだけで完結します**
（M5StampS3A は物理的に 8MB フラッシュを搭載。`anchor-*` / `tag-*` は Wi-Fi
ダッシュボード用のコードが増えた分アプリ領域を 1MB→3MB に広げており、収まる
ようビルド時に 8MB 設定を明示しています。`probe-*` / `twr-*` は ESP-IDF の
既定設定のままですが、どちらも書き込みコマンド自体は同じです）。

### ポートを調べる

- **macOS**: `ls /dev/cu.usbmodem*`
- **Linux**: `ls /dev/ttyACM*`（権限が無い場合は `sudo usermod -aG dialout $USER` 後に再ログイン）
- **Windows**: デバイスマネージャーの「ポート (COM と LPT)」を開き、M5StampS3A を
  挿す前後で増えた `COMxx` を探す（ESP32-S3 の USB-Serial/JTAG は Windows 10/11 の
  標準ドライバで認識される想定。**本リポジトリでは Windows 実機での動作確認はしていません**）

### 方法 A: `esptool`（コマンドライン）

```sh
pip install esptool

esptool.py --chip esp32s3 -p /dev/cu.usbmodemXXXX write_flash 0x0 merged-firmware.bin
# Windows の場合はポートを COM ポート名に置き換える: -p COM5 など
```

`esptool.py` は USB のリセット信号（DTR/RTS）で**通常は自動的に**書き込みモード
（ダウンロードモード）に入ります。何らかの理由でタイムアウトする場合は
`--before default_reset --after hard_reset` を明示するか、下記「書き込みモードに
手動で入れる」を試してください。

書き込みエラー（サイズやチェックサムの不一致）が出た場合だけ、念のため
`--flash_size 8MB`（または自動検出の `--flash_size detect`）を明示的に付けてみてください
（通常は不要です。フラッシュ設定は `merged-firmware.bin` 側に既に入っています）。

### 方法 B: ブラウザ（インストール不要）

<https://espressif.github.io/esptool-js/>（Espressif 公式の esptool-js。同種のツールに
[ESP Web Tools](https://esphome.github.io/esp-web-tools/) や
[Adafruit WebSerial ESPTool](https://adafruit.github.io/Adafruit_WebSerial_ESPTool/) もあります）
を **Chrome または Edge**（Web Serial API に対応したブラウザ）で開き、

1. **Connect** でポートを選ぶ
2. ファイルに `merged-firmware.bin`、**Flash Address に `0x0`** を指定
3. **Program**

### 書き込みモードに手動で入れる

自動で書き込みモードに入らない場合は、次のどちらか（**ボードの種類による**）で
手動でダウンロードモードに入れます。

- **StampS3 BreakOut を使っている場合（標準構成）**: BreakOut 上の **G0 ボタンを
  押したまま EN ボタンを押して離し、その後 G0 を離します**（ESP32 系でよく使われる
  二ボタン方式）。
- **BreakOut を使わず M5StampS3A 単体の場合**: 中央のボタン（G0）を押しながら
  USB を挿します。

> **この2つの手動手順そのものは、本リポジトリでは実機確認していません**
> （通常は esptool の自動リセットで書き込めるため、手動手順が必要になった
> 実例が無い）。M5Stack の各製品ページも参照してください。

### 出力を見る（シリアルコンソール）

**`idf.py monitor` は ESP-IDF が入っていないと使えません。** 代わりに、115200bps に
対応した一般的なシリアルターミナルを使います。

- **macOS / Linux**: `screen /dev/cu.usbmodemXXXX 115200`（抜けるのは `Ctrl-A` `K`）
- **Windows**: [Tera Term](https://ttssh2.osdn.jp/) または [PuTTY](https://www.putty.org/) で
  COM ポートを 115200bps・8N1 で開く（**本リポジトリでは Windows での動作確認はしていません**）
- **どの OS でも（Python があれば）**: `pip install esp-idf-monitor` の後
  `esp-idf-monitor -p /dev/cu.usbmodemXXXX`（`idf.py monitor` と同じ表示。ただし
  キー入力の一部ショートカットは ESP-IDF 環境が前提）

**このシリアルターミナルが、後述の「一気通貫の手順」で `addr` / `anchor set` /
`wifi set` 等を打つコンソールそのものです。** 115200bps 以外の設定にすると
文字化けするので注意してください。

期待される起動ログの内容は [`GETTING_STARTED.md`](GETTING_STARTED.md#probe) §4.2
（`firmware/probe`）・[§6.2](GETTING_STARTED.md#anchors5-console)（`firmware/anchor`
のコンソール）・§8.2（`firmware/tag` の起動ログ）を参照。

---

## 一気通貫の手順（タグ1台＋アンカー数台 → ブラウザのダッシュボード表示まで）

ESP-IDF を入れずに、書き込みから設定・ダッシュボード表示までを通しで行う手順です。
各コマンドの詳細は [`GETTING_STARTED.md`](GETTING_STARTED.md) の該当節に譲るので、
ここでは**手順の流れ**だけを示します。

0. **（推奨）配線を疎通確認する。** `2-wiring-check-probe-M5StampS3A.zip` の
   `probe-stamps3/merged-firmware.bin` を書き込んで、Device ID `0xDECA0314` が
   読めることを確認してから先に進みます（詳細は上の「どれを取ればいいか」と
   [`GETTING_STARTED.md` §4.2](GETTING_STARTED.md#probe)）。
1. **zip を取得する**（各1回ずつでよい。同じ zip を全アンカーに使い回す）。
   タグも M5StampS3A 単体の場合は `1-positioning-M5StampS3A-tag-and-anchor.zip`
   の1つでよい（中の `tag-stamps3-ds/` と `anchor-stamps3-ds/` を使う）。
   タグを StampFly に載せたい場合は
   [開発者向け](#dev-only)の `tag-stampfly-ds`（Actions artifact）を
   別途取得してください。
2. **アンカーを1台ずつ書き込む**。`1-positioning-M5StampS3A-tag-and-anchor.zip`
   の中の `anchor-stamps3-ds/merged-firmware.bin` を、台数分すべて同じ内容のまま
   上記の「書き込み方」（`esptool` または方法B）でオフセット `0x0` に書き込む
   （**ビルドは1回、書き込みは台数分**）。
3. **アンカーごとにシリアルコンソール（115200bps）でアドレスを設定する。**
   1 台ずつ USB を挿し、上記「出力を見る（シリアルコンソール）」の方法でコンソールを開いて:
   ```
   uwb-anchor> addr
   uwb-anchor> addr set 0x0003
   uwb-anchor> save
   uwb-anchor> reboot
   ```
   1台目は既定値 `0x0002` のままでよいので `save` だけでも構いません。2台目以降は
   `0x0003`・`0x0004`…と重複しないアドレスに変えます（`0xFFFF` とタグの `0x0001` は
   使えません）。**設定したアドレスをボードにテープで貼るなどして物理的に判別できる
   ようにしてください**（次の座標登録で必要になります）。詳細は
   [`GETTING_STARTED.md` §6.2](GETTING_STARTED.md#anchors5-console)。
4. **タグを書き込む。** 手順は 2 と同じ（`1-positioning-M5StampS3A-tag-and-anchor.zip`
   の `tag-stamps3-ds/merged-firmware.bin`、または StampFly の場合は
   [開発者向け](#dev-only)の `tag-stampfly-ds` の `merged-firmware.bin` を
   オフセット `0x0` に書き込む）。
5. **タグのコンソールでアンカー座標を登録する。** 巻尺で実測した各アンカーの
   アンテナ位置を、アドレスと対応づけて登録します。
   ```
   uwb-tag> anchor set 0 0x0002 0.000 0.000 2.400
   uwb-tag> anchor set 1 0x0003 5.000 0.000 0.200
   ...
   uwb-tag> save
   ```
   3点間の距離だけで座標を計算したい場合は `survey dist` / `survey z` /
   `survey apply` も使えます（[`GETTING_STARTED.md` §7.3](GETTING_STARTED.md#survey)）。
   **2D 運用**（アンカーを同一平面にしか置けない場合）はタグの高さを
   `height <メートル>` → `save` で設定してください。
6. **Wi-Fi を設定する。** タグのコンソールで
   ```
   uwb-tag> wifi set <ルーターのSSID> <パスワード>
   ```
   を実行すると、起動ログ（または再起動後）に開くべき URL が
   `url=http://<IP>/ mdns=http://uwb-tag.local/` の形で出ます。この URL
   （または mDNS〈`<名前>.local` で機器を見つける仕組み〉が使える環境なら
   `http://uwb-tag.local/`）をブラウザで開くと、タグと全アンカーの数値・グラフ・
   測位図・無線コンソールが1画面で見られます。詳細は
   [`GETTING_STARTED.md` §8.7](GETTING_STARTED.md#net-dashboard) と
   [`NET_DASHBOARD.md`](NET_DASHBOARD.md)。アンカー側にも Wi-Fi を設定すると、
   各アンカー単体の簡易ページも見られます。
7. **アンカーの配置ルールを必ず守る。** 最低1台（できれば複数）は他のアンカーと
   高さを 1m 以上変えてください。同一平面に並べると3D測位が原理的に不安定になり、
   実機でも誤った解（鏡像解）が選ばれた例があります。根拠と実測は
   [`ANCHOR_PLACEMENT.md`](ANCHOR_PLACEMENT.md)。
8. **ファームを書き直した後は、ブラウザを強制再読み込みしてください**
   （macOS: `Shift+Cmd+R`、Windows/Linux: `Ctrl+Shift+R`）。古いページが
   接続し直したまま残ってしまうことがあります。

---

## ビルド済みでも**実行時に変えられる**もの

ここが重要です。**アンカー5台に同じバイナリを書いて構いません。**
個体差はシリアルコンソールで設定し、NVS〈ESP32 の不揮発設定ストレージ〉に保存されます。

| 設定 | コマンド | ファーム |
|---|---|---|
| アンカーのショートアドレス | `addr set 0x0003` → `save` | `anchor` |
| アンカーの座標 | `anchor set <idx> <hex_addr> <x> <y> <z>` → `save` | `tag` |
| アンテナ遅延 | `anchor delay <idx> <meters>` → `save` | `tag` |
| アンカーの有効/無効・台数 | `anchor enable/disable <idx>` / `anchor count <n>` | `tag` |
| 測位モード（有効台数・配置で自動切替。手動強制も可） | `mode` （表示）/ `mode auto\|2d\|3d` → `save` | `tag` |
| 2D測位の固定高さ | `height <meters>` → `save` | `tag` |
| EKF（拡張カルマンフィルタ）のQ・R・ゲート | `ekf q\|r\|gate\|model` （表示は `ekf` のみ）→ `save` | `tag` |
| JSON 出力の一時停止 | `output off` / `output on` | `tag` |
| 設定の確認 | `info` / `anchor list` | 両方 |

詳細は [`GETTING_STARTED.md` §6.2](GETTING_STARTED.md#anchors5-console) と §7。

## ビルド済みでは**変えられない**もの

再ビルドが必要です。

- **ピン割り当て**（`boards/*.h`）
- ボードの種類、SS-TWR / DS-TWR の別
- **IRQ の有効/無効、遅延プリセット**（`anchor-*-ds` / `tag-*-ds` の既定は
  IRQ 有効 + `PollingBoth`。IRQ だけを無効化した `*-polling` 版は
  [開発者向け](#dev-only)の Actions artifact から取得できます）
- SPI クロック、EKF の有効化（on/off そのもの。有効化後のQ/R/ゲートの
  調整は上表の `ekf` コマンドで実行時に変えられます）、2D フォールバックの挙動

これらを触るときは [`GETTING_STARTED.md` §2](GETTING_STARTED.md#setup) の手順で
ESP-IDF を入れて自分でビルドしてください。

---

## CI が何をしているか

`.github/workflows/build.yml`:

1. **ホスト側テスト**（`test_pipeline` / `test_survey` / `tests/host/loc`）を実行
   ※ 上流 `uwb_localizer` は凍結・最終取り込み済み。CI は上流を
   clone せず、本リポジトリ内のソースだけでテストします
2. **12 通りのファーム**を ESP-IDF v5.5.2 でビルドし、`idf.py merge-bin` で結合
3. **variant 単位で artifact として保存**（Actions タブから常にこの単位で取れる。
   12 通り全部、[開発者向け](#dev-only)の variant も含む）
4. **タグを打った時だけ**、普段使う2つの variant の組だけを**2つの bundle zip
   へまとめ直して** Release に添付（上表参照。残り10 variant は Release には
   載らず、Actions artifact のみで入手可能）

各 variant のフォルダ（Actions artifact ではそのまま、Release の bundle
では中のサブフォルダ）には `kconfig-used.txt`（そのバイナリに焼き込まれた設定）と
`README.md`（書き込み手順）が同梱されます。**どの設定でビルドされたものか必ず確認できます。**
Release の bundle にはさらに、書き込むファイルの対応を示す**トップレベルの
`README.md`** が付きます。

variant フォルダの中身一覧:

| ファイル / ディレクトリ | 内容 |
|---|---|
| `merged-firmware.bin` | 書き込み用の結合済みイメージ（オフセット `0x0`） |
| `bootloader/bootloader.bin`, `partition_table/partition-table.bin`, `flasher_args.json` | 個別書き込み用（`esptool.py write_flash @flasher_args.json`） |
| `kconfig-used.txt` | そのバイナリに焼き込まれた Kconfig 設定 |
| `README.md` | 書き込み手順とライセンスの要約 |
| `LICENSE` | 本リポジトリのライセンス本文（MIT） |
| `THIRD_PARTY_LICENSES.md` | サードパーティ ライセンスの一覧 |
| `LICENSES/LicenseRef-QORVO-2.txt` | Qorvo ドライバのライセンス全文（**Qorvo 製 IC 限定**、上記参照） |

---

## 関連文書
- [`EXPERIMENT_PLAN.md`](EXPERIMENT_PLAN.md) — どの順で試すか
- [`GETTING_STARTED.md`](GETTING_STARTED.md) — 配線から測位までの完全手順（ESP-IDF を使う場合）
- [`GETTING_STARTED.md` §4.2.1](GETTING_STARTED.md#probe-result) — 実験1（SPI 疎通）の受入基準・実機結果
- [`ANCHOR_PLACEMENT.md`](ANCHOR_PLACEMENT.md) — アンカー配置のルールと実機での根拠
- [`NET_DASHBOARD.md`](NET_DASHBOARD.md) — Wi-Fi ダッシュボードの詳細（トポロジ・`wifi` コマンド・各パネルの意味）
- [`TIMING_PRESETS.md`](TIMING_PRESETS.md) — 遅延プリセットで何が変わるか
- [`IRQ_POLICY.md`](IRQ_POLICY.md) — IRQ の使用方針と実機での検証結果
- [`HANDOFF.md`](HANDOFF.md) §1 — 実機での検証済み・未検証の一覧
