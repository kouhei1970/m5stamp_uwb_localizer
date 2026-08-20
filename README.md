# m5stack_uwb

StampFly（マルチコプター機体）に依存しない、汎用の UWB 測位ポーティングプロジェクトです。
Qorvo **QM33120W (DW3720)** を搭載した **M5Stack Stamp UWB-F** モジュールを、汎用の
**ESP32-S3 ホストボード**上で ESP-IDF から使えるようにすることを目的としています。

コア（レンジング処理・測位演算）は StampFly から独立して完結させ、StampFly への統合は
本リポジトリの成果物を利用する下流の作業と位置づけます。詳細な設計方針・フェーズ計画は
[docs/PLAN.md](docs/PLAN.md) を参照してください。

## 対応ハードウェア

- **UWB モジュール**: [M5Stack Stamp UWB-F](https://docs.m5stack.com/en/stamp/Stamp_UWB_F)
  （Qorvo QM33120W / DW3720 チップ搭載、SPI 接続、0.5mm 12P FPC コネクタ、3.3V 単一電源）
- **ホストボード**（いずれも ESP32-S3、`idf.py set-target esp32s3`）:
  - **M5Stamp S3**
  - **M5 AtomS3**

汎用の ESP32-S3 DevKit ではなく、この2機種で動作することを前提に設計・検証しています。

## ディレクトリ構成

現時点（Phase 1）で実際に存在するもの／このフェーズ直後に追加される予定のものです。
将来フェーズで追加されるコンポーネントについては [docs/PLAN.md](docs/PLAN.md) の
「ディレクトリ構成（予定）」を参照してください。

```
m5stack_uwb/
├── README.md                 このファイル
├── LICENSE                    リポジトリ全体の既定ライセンス（MIT）
├── THIRD_PARTY_LICENSES.md    ライセンス構成の詳細
├── PROGRESS.md                開発進捗ログ
├── docs/                      調査資料（SURVEY_*.md）と開発計画（PLAN.md）
├── components/
│   ├── qm33120w_sdk/          Qorvo 提供の QM33120W/DW3720 チップドライバ SDK
│   │                          （m5stack/M5Stamp-UWB から vendoring。SPDX ヘッダ保持のまま）
│   └── uwb_port/              ESP-IDF 向けプラットフォーム抽象層（SPI/GPIO/時刻/排他）
├── boards/                    ホストボードごとのピン定義ヘッダ（stamps3.h / atoms3.h）
│                              ※ ピン番号は暫定値。実機配線での検証が必要（各ヘッダの冒頭コメント参照）
├── firmware/
│   └── probe/                 Phase 1 の疎通確認アプリ（DW3720 の Device ID を読み出す）
│                              ホストボードは Kconfig で選択（後述）
└── third_party/                読み取り専用の上流参照クローン（ビルド対象外、gitignore 済み）
    └── M5Stamp-UWB/            m5stack/M5Stamp-UWB のクローン（qm33120w_sdk の vendoring 元）
```

`components/qm33120w_sdk/`、`components/uwb_port/`、`boards/`、`firmware/probe/` は
いずれも Phase 1 で配置済みです（ビルド検証済み。実機での動作確認は未実施 - 詳細は
[PROGRESS.md](PROGRESS.md) 参照）。

さらに後続フェーズでは `components/uwb_qm33120/`（M5Stamp-UWB wrapper の C++ 移植）、
`components/uwb_twr/`（TWR 状態機械）、`components/uwb_loc/`（測位ソルバ、
[uwb_localizer](https://github.com/kouhei1970/uwb_localizer) から vendoring）、
`firmware/tag/`、`firmware/anchor/` などが追加される予定です。詳細は
[docs/PLAN.md](docs/PLAN.md) を参照してください。

## ビルド方法

前提: **ESP-IDF v5.5.2**

```sh
. ~/esp/esp-idf/export.sh
cd firmware/probe
idf.py set-target esp32s3
idf.py build
```

### ホストボードの選択

`firmware/probe` はデフォルトで **M5Stamp S3**（`boards/stamps3.h`）向けにビルドされます。
**M5 AtomS3**（`boards/atoms3.h`）を使う場合は `idf.py menuconfig` の
`UWB Probe Configuration` → `Target host board` で切り替えてください
（`sdkconfig` に `CONFIG_UWB_PROBE_BOARD_ATOMS3=y` を設定するのと同義です）。

フラッシュ書き込み・実機動作確認は Phase 1 の範囲外です（実機書き込み不可のため未実施。
実機がある場合は別途 `idf.py -p <PORT> flash monitor` を実行してください）。

## ライセンス

このリポジトリは複数のライセンスが混在しています。

- 本リポジトリ独自のコード（例: `components/uwb_port/`）: **MIT**、Copyright (c) 2026 Kouhei Ito
  （[LICENSE](LICENSE) 参照）
- `components/qm33120w_sdk/`（Qorvo 提供の SDK を vendoring）: **LicenseRef-QORVO-2**。
  **Qorvo 製 IC（本モジュールが搭載する QM33120W/DW3720）専用**という重要な制約があり、
  一般的なパーミッシブライセンスではありません。

詳細は [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md) を参照してください。

## 開発計画・進捗

- 設計方針・フェーズ計画: [docs/PLAN.md](docs/PLAN.md)
- 事前調査資料: [docs/](docs/) 配下の `SURVEY_*.md`
- 進捗ログ: [PROGRESS.md](PROGRESS.md)
