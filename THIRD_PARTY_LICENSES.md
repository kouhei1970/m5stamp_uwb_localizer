# サードパーティ ライセンス

このリポジトリは複数のライセンスが混在します。ディレクトリごとの適用ライセンスは以下の通りです。

## `components/qm33120w_sdk/` — `LicenseRef-QORVO-2`

Copyright (c) 2024 Qorvo US, Inc.

Qorvo QM33120W (DW3720) 用チップドライバ SDK を、上流 [m5stack/M5Stamp-UWB](https://github.com/m5stack/M5Stamp-UWB)
の `src/qm33120w_sdk/` からそのまま vendoring したものです。全文は
[`components/qm33120w_sdk/LICENSES/LicenseRef-QORVO-2.txt`](components/qm33120w_sdk/LICENSES/LicenseRef-QORVO-2.txt)
を参照してください。

**重要な制約**: このライセンスは一般的なパーミッシブライセンスでは *ありません*。
条件3により、**本ソフトウェアは Qorvo 製の集積回路（本モジュール = M5Stack Stamp UWB-F に
搭載された Qorvo QM33120W/DW3720、またはそれを内蔵するモジュール）とともに使用する場合に限り**
利用が許可されています。他ベンダーのチップ向けに転用することはできません。

`components/qm33120w_sdk/` 配下の全ソースファイルは、冒頭の SPDX ヘッダ
（`SPDX-FileCopyrightText: Copyright (c) 2024 Qorvo US, Inc.` /
`SPDX-License-Identifier: LicenseRef-QORVO-2`）を改変せずそのまま保持しています。

## `components/uwb_loc/` — MIT

MIT License, Copyright (c) 2026 Kouhei Ito

[kouhei1970/uwb_localizer](https://github.com/kouhei1970/uwb_localizer) の測位ソルバ部分
（`c/include/uwb_loc.h`、`c/src/{uwb_model,uwb_closed_form,uwb_nls,uwb_ekf,uwb_linalg}.c`、
`c/src/{uwb_internal,uwb_linalg}.h`）を、`third_party/uwb_localizer/`
（コミット `8d0edc057ed05cf6b4af91df329999fe2343f515`）からそのまま vendoring したものです。
ロジックは無改造で、ESP-IDF 用の `CMakeLists.txt` / `Kconfig` を追加しただけです。
全文は取り込み元の [`third_party/uwb_localizer/LICENSE`](third_party/uwb_localizer/LICENSE)
（本リポジトリの [`LICENSE`](LICENSE) と同一の MIT）を参照してください。

## M5Stack 由来のポーティングコード（将来追加予定、例: `components/uwb_qm33120/`）

MIT License, Copyright (c) 2026 M5Stack Technology CO LTD

[m5stack/M5Stamp-UWB](https://github.com/m5stack/M5Stamp-UWB) の Arduino ラッパー
（`src/M5Stamp_UWB.cpp` 等）を ESP-IDF / C++ に移植して取り込む場合、このライセンス表記に従います。
本フェーズ（Phase 1）ではまだ該当コードは追加されていません。

## それ以外（本リポジトリ独自に書かれたコード）— MIT

MIT License, Copyright (c) 2026 Kouhei Ito

`components/uwb_port/` など、本リポジトリのために新規に書かれたコードは、リポジトリ直下の
[`LICENSE`](LICENSE) と同一の MIT ライセンスです。
