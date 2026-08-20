# m5stack_uwb

**M5Stack Stamp UWB-F**（Qorvo QM33120W / DW3720 搭載の UWB モジュール）を
**ESP32-S3 ホストボード**から ESP-IDF で使うための、**測距 + 測位スタック**です。

タグ 1 台とアンカー 4 台以上で、**屋内の 3 次元位置**を求めて JSON Lines で吐きます。
StampFly（マルチコプター機体）には依存しない汎用のスタックで、StampFly への統合は
本リポジトリの成果物を使う下流の作業と位置づけています。

---

## ⚠️ 現状（先に読んでください）

**このリポジトリのコードは、実機で一度も動作確認していません。**

| | 状況 |
|---|---|
| ビルド | 全ファーム **警告 0・エラー 0**（ESP-IDF v5.5.2 / ESP32-S3） |
| 測位パイプライン | ホスト（PC）上の合成データで検証済み（**53 件のチェック全通過**） |
| 測位ソルバ（uwb_loc） | 上流 [uwb_localizer](https://github.com/kouhei1970/uwb_localizer) のホストテストで検証済み |
| ソルバの計算時間 | ESP32-S3 実機用ベンチを実装済み（**実行は未**） |
| **実機での SPI 疎通（Device ID 読み出し）** | **未確認** |
| **実機での測距・測位** | **未確認** |

- 製品自体が新しく、コミュニティの作例もほとんどありません。
- **半田付けパッドの向き（pin 1 がどちら側か）は実物で確認が必要**です。公式の
  PINMAP 画像が配信サーバから取得できず、設計データからは左右を確定できていません。
  → [`docs/GETTING_STARTED.md` §3.1](docs/GETTING_STARTED.md#orientation) の
  **テスターによる導通試験を必ず実施してください**（間違えると電源逆接で壊れます）。
- 未確認事項の一覧は [`docs/SOLDER_PADS.md`](docs/SOLDER_PADS.md) §5 と
  [`docs/GETTING_STARTED.md` §11](docs/GETTING_STARTED.md#limitations) にあります。

つまりこれは「動作実績のあるライブラリ」ではなく、**一次資料から積み上げた実装と、
その検証手順の一式**です。実機での最初の 1 件が取れたら、そこがこのプロジェクトの
本当のスタート地点になります。

---

## 何ができるか

| 機能 | 状態 |
|---|---|
| **SS-TWR / DS-TWR による 2 点間の測距** | 実装済み（実機未検証） |
| **アンカー 4 台以上での 3D 測位**（台数は登録テーブルの長さで決まる。上限 8） | 実装済み（ホスト検証済み） |
| 測位ソルバ 3 段（Lv0 閉形式三辺測量 / **Lv2 Beck 厳密解 + Huber ロバスト化** / Lv3 EKF） | 実装済み |
| **外れ値アンカーの自動棄却**（Huber / χ² ゲート） | 実装済み |
| 欠測・タイムアウトへの耐性（有効測距 4 件以上あれば解く） | 実装済み |
| 同一平面配置の検出と 2D フォールバック | 実装済み |
| **JSON Lines 出力**（測距値・位置・GDOP・残差・棄却情報・周期時間） | 実装済み |
| アンテナ遅延の補正（値はアンカーごとに設定） | 実装済み（**校正手順は手動**） |
| IRQ 駆動 / ch9 PLL 再校正 / NLOS 判定 | **未実装**（[§既知の制約](docs/GETTING_STARTED.md#limitations)） |

---

## 必要なもの

### ハードウェア（検証構成: タグ 1 + アンカー 5）

| 品目 | 数量 | 役割 |
|---|---:|---|
| [M5Stack Stamp UWB-F](https://docs.m5stack.com/en/stamp/Stamp_UWB_F)（SKU `S017-F` / 無印 `S017` も可） | **6** | UWB モジュール |
| [M5Stamp S3](https://docs.m5stack.com/en/core/StampS3) | **1** | タグ（移動体）のホスト |
| [M5 AtomS3](https://docs.m5stack.com/en/core/AtomS3) | **5** | アンカー（固定局）のホスト |

**モジュールとホストは半田付けで接続します**（1.27mm ピッチのキャステレーションホール）。
FPC コネクタ経由でも構いませんが、0.5mm 12P の変換基板が別途必要です。

### 工具・材料

**AWG30 相当の細線**、**1.6mm C 型以下のこて先**、フラックス、拡大鏡、
**テスター（向きの確認に必須）**、線の根元を固定する接着剤、
**巻尺（アンカー座標の実測用）**、USB-C ケーブル。

> 詳細な BOM は [`docs/GETTING_STARTED.md` §1](docs/GETTING_STARTED.md#bom)。

### ソフトウェア

**ESP-IDF v5.5.2**

---

## 5 分クイックスタート

**実機がなくてもここまでできます。**

```sh
# 1) ESP-IDF v5.5.2（未導入なら）
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.2 --recursive https://github.com/espressif/esp-idf.git
cd ~/esp/esp-idf && ./install.sh esp32s3

# 2) 環境変数（ターミナルを開くたびに必要）
. ~/esp/esp-idf/export.sh

# 3) このリポジトリ
git clone <このリポジトリの URL> m5stack_uwb
cd m5stack_uwb

# 4) 測位計算が正しく動くことを PC 上で確認（ESP-IDF 不要）
cd tools/test_pipeline && make test
#    → "=== 53 件中 0 件失敗 ===" が出れば OK
cd ../..

# 5) 疎通確認ファームをビルド
cd firmware/probe
idf.py set-target esp32s3
idf.py build
```

**実機がある場合**は、配線を済ませてから（**必ず
[§3.1 向きの確定](docs/GETTING_STARTED.md#orientation) を先に**）:

```sh
ls /dev/cu.usbmodem*                                  # macOS。Linux は /dev/ttyACM*
idf.py -p /dev/cu.usbmodemXXXX flash monitor
```

期待される出力:

```
I (xxx) uwb_probe: L1: raw DEV_ID = 0xDECA0314 (expect 0xDECA0314) -> OK
I (xxx) uwb_probe: L2: dwt_probe + dwt_readdevid = 0xDECA0314 (expect 0xDECA0314) -> OK
I (xxx) uwb_probe: === L1: PASS / L2: PASS ===
```

**`0xDECA0314` が読めれば最初の関門は突破**です。
読めないときの切り分けは
[`docs/GETTING_STARTED.md` §4.3](docs/GETTING_STARTED.md#probe-troubleshoot)。

> **M5 AtomS3 で動かす場合**は `idf.py menuconfig` →
> `UWB Probe Configuration` → `Target host board` → `M5 AtomS3` に切り替えてください
> （`firmware/probe` の既定は M5Stamp S3）。

**→ ここから先の完全な手順は [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md) へ。**
配線・5 台への書き込み・座標入力・測位・アンテナ遅延の校正まで通しで書いてあります。

---

## ディレクトリ構成

```
m5stack_uwb/
├── README.md                このファイル
├── LICENSE                  リポジトリ既定のライセンス（MIT）
├── THIRD_PARTY_LICENSES.md  ライセンス構成の詳細
├── PROGRESS.md              開発進捗ログ（何がどこまで検証済みか）
│
├── docs/
│   ├── GETTING_STARTED.md   ★ 買ってから測位が出るまでの完全手順
│   ├── SOLDER_PADS.md       半田パッドの寸法・配線・アンテナ禁止領域
│   ├── BRINGUP.md           Phase 1（SPI 疎通）の受入確認手順
│   ├── ANCHOR_PLACEMENT.md  アンカー配置ルール（実測にもとづく）
│   ├── PLAN.md              全体設計・フェーズ計画
│   ├── REIMPL_PLAN.md       TWR 層の課題一覧（R1〜R12）
│   ├── CRITICAL_REVIEW.md   移植元コードの問題点の詳細分析
│   ├── PERF_ANALYSIS.md     測位ソルバの性能分析
│   ├── PLATFORM_TUNING.md   ESP32-S3 固有の最適化調査
│   └── SURVEY_*.md          事前調査資料
│
├── boards/                  ホストボードのピン定義（※ 暫定値。実配線で要検証）
│   ├── stamps3.h            M5Stamp S3
│   └── atoms3.h             M5 AtomS3
│
├── components/
│   ├── qm33120w_sdk/        Qorvo 提供の QM33120W/DW3720 チップドライバ SDK
│   ├── uwb_port/            ESP-IDF 向けプラットフォーム抽象層（SPI/GPIO/時刻/排他）
│   ├── uwb_qm33120/         C++ ラッパ（初期化・PHY 設定・SS/DS-TWR）
│   ├── uwb_ranging/         アンカー登録テーブル / スケジューラ / 測位パイプライン
│   └── uwb_loc/             測位ソルバ（Lv0 / Lv2 / EKF）
│
├── firmware/
│   ├── probe/               ★ SPI 疎通確認（Device ID を読む）
│   ├── devtest/             フレーム送受信の疎通確認（sender / receiver）
│   ├── twr/                 ★ 1 対 1 の測距評価（ロール・方式を Kconfig で切替）
│   ├── anchor/              ★ 本番用アンカー（常時レスポンダ。アドレスを Kconfig で設定）
│   ├── tag/                 ★ 本番用タグ（測距 → 測位 → JSON Lines 出力）
│   └── soltest/             ソルバの実機ベンチ（UWB ハード不要）
│
├── tools/
│   ├── test_pipeline/       ★ 測位パイプラインのホスト検証（実機不要、`make test`）
│   └── test_uwb_loc/        測位ソルバのホスト検証（上流クローンが別途必要）
│
└── third_party/             上流リポジトリの参照クローン（gitignore 済み・ビルド対象外）
```

`docs/refs/`（ベンダ資料）と `third_party/` は `.gitignore` されています。
clone しただけの状態で全ファームがビルドできます。

### 依存関係

```
firmware/tag ──┬─ uwb_ranging ─┬─ uwb_loc          （ハード非依存。PC でもビルドできる）
               │               └─ uwb_qm33120 ── uwb_port ── qm33120w_sdk ── ESP-IDF
               └─ boards/*.h
```

ハードウェア依存は **`components/uwb_port/` 1 枚に閉じています**。
`uwb_ranging` の測位側と `uwb_loc` は ESP-IDF に一切依存せず、
`tools/test_pipeline` でそのまま PC 上でビルド・検証できます。

---

## 主なハードウェア仕様（Stamp UWB-F）

| 項目 | 値 |
|---|---|
| チップ | Qorvo **QM33120W**（DW3720 系。Device ID `0xDECA0314`） |
| ホスト I/F | **SPI 一択**（mode0 / MSB first、初期化 2MHz → 通常 16MHz、チップ上限 32MHz） |
| **電源** | **DC 3.3V 単一。5V は不可（壊れます）** |
| 消費電流 | スリープ 75.9µA / アンカー動作 5.23mA / **タグ動作 58.0mA** @3.3V |
| チャネル | **ch9 固定**（中心周波数 7987.2MHz） |
| 接続 | 0.5mm 12P FPC コネクタ（`S017-F` のみ） / **1.27mm キャステレーションホール 12 個**（両方） |
| 外形 | 11.5 × 12.0 × 1.6mm（`S017`）/ × 2.8mm（`S017-F`） |

> **M5Stack の別製品「Unit UWB」(SKU U100) とは別物です。**
> あちらは UART + AT コマンド方式（DW1000 ベース）で、本リポジトリのコードは使えません。

詳細は [`docs/SURVEY_stamp_uwb_f.md`](docs/SURVEY_stamp_uwb_f.md)。

---

## ライセンス

**このリポジトリは複数のライセンスが混在しています。**

| 対象 | ライセンス |
|---|---|
| 本リポジトリ独自のコード（`components/uwb_port/`, `components/uwb_ranging/`, `firmware/*` 等） | **MIT** — Copyright (c) 2026 Kouhei Ito |
| `components/qm33120w_sdk/`（Qorvo 提供 SDK の vendoring） | **LicenseRef-QORVO-2** |
| `components/uwb_loc/`（[uwb_localizer](https://github.com/kouhei1970/uwb_localizer) 由来） | **MIT** — Copyright (c) 2026 Kouhei Ito |
| `components/uwb_qm33120/`（[m5stack/M5Stamp-UWB](https://github.com/m5stack/M5Stamp-UWB) の Arduino ラッパを C++ へ移植） | **MIT** — Copyright (c) 2026 M5Stack Technology CO LTD |

> ### ⚠️ `LicenseRef-QORVO-2` の重要な制約
>
> **一般的なパーミッシブライセンスではありません。**
> 条件 3 により、**Qorvo 製の集積回路（＝本モジュールが搭載する QM33120W/DW3720、
> またはそれを内蔵するモジュール）とともに使用する場合に限り**利用が許可されます。
> **他ベンダーのチップ向けに転用することはできません。**

全文と詳細は [`LICENSE`](LICENSE) と
[`THIRD_PARTY_LICENSES.md`](THIRD_PARTY_LICENSES.md) を参照してください。

---

## 開発計画・進捗

- **手順書**: [`docs/GETTING_STARTED.md`](docs/GETTING_STARTED.md)
- 設計方針・フェーズ計画: [`docs/PLAN.md`](docs/PLAN.md)
- 事前調査資料: [`docs/`](docs/) 配下の `SURVEY_*.md`
- 進捗ログ: [`PROGRESS.md`](PROGRESS.md)
- 既知の課題: [`docs/REIMPL_PLAN.md`](docs/REIMPL_PLAN.md) / [`docs/CRITICAL_REVIEW.md`](docs/CRITICAL_REVIEW.md)
