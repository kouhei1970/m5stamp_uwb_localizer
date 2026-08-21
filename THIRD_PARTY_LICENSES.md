# サードパーティ ライセンス

このリポジトリは複数のライセンスが混在します。ディレクトリごとの適用ライセンスは以下の通りです。

## `components/qm33120w_sdk/` — `LicenseRef-QORVO-2`

Copyright (c) 2024 Qorvo US, Inc.

Qorvo QM33120W (DW3720) 用チップドライバ SDK を、上流 [m5stack/M5Stamp-UWB](https://github.com/m5stack/M5Stamp-UWB)
の `src/qm33120w_sdk/` からそのまま vendoring したものです。全文は
[`components/qm33120w_sdk/LICENSES/LicenseRef-QORVO-2.txt`](components/qm33120w_sdk/LICENSES/LicenseRef-QORVO-2.txt)
を参照してください。

**重要な制約**: このライセンスは一般的なパーミッシブライセンスでは *ありません*。
条件3により、**本ソフトウェアは Qorvo 製の集積回路（本モジュール = M5Stamp UWB Module に
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
全文は上流の [`LICENSE`](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/LICENSE)（本リポジトリの [`LICENSE`](LICENSE) と同一の MIT）を
参照してください。**`third_party/` は `.gitignore` されているので、clone しただけでは
ローカルには存在しません**（参照用クローンを手元に置いている場合のみ
`third_party/uwb_localizer/LICENSE` にあります）。

## `docs/UWB_ALGORITHMS.md` — MIT（文書の移植）

MIT License, Copyright (c) 2026 Kouhei Ito

[kouhei1970/uwb_localizer](https://github.com/kouhei1970/uwb_localizer) の
[`docs/UWB_ALGORITHMS.md`](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/docs/UWB_ALGORITHMS.md)
を本リポジトリ向けに改訂して移植したものです。
**数式・導出・章立ては原典のまま**で、追加したのは本リポジトリの実装への対応づけ、
本リポジトリでは未使用の節（TDoA / AoA）の注記、`SolverLevel` に `Lv1` が無いことの
注記、および相対リンクの張り替えです。文書冒頭にも同じ出典を明記しています。

## `components/uwb_qm33120/` — MIT（M5Stack 由来の移植コードを含む）

```
MIT License

Copyright (c) 2026 M5Stack Technology CO LTD
Copyright (c) 2026 Kouhei Ito

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

このコンポーネントは [m5stack/M5Stamp-UWB](https://github.com/m5stack/M5Stamp-UWB)
の Arduino ラッパー（`src/M5Stamp_UWB.cpp` / `M5Stamp_UWB.h` / `M5Stamp_UWB_Types.h`、MIT）を
ESP-IDF / C++ へ移植したものを含みます。各ファイル冒頭に移植元の関数名と行番号を記載しています。

ただし**制御フローの一部は Qorvo 公式 TWR リファレンス実装に合わせて書き直しており、
M5Stack 版とは挙動が異なります**（詳細は [`docs/CRITICAL_REVIEW.md`](docs/CRITICAL_REVIEW.md)
と [`docs/REIMPL_PLAN.md`](docs/REIMPL_PLAN.md)）。移植元と同じ動作を期待しないでください。

## `components/uwb_ranging/` — MIT（本リポジトリ独自）

MIT License, Copyright (c) 2026 Kouhei Ito

アンカー登録テーブル・測距スケジューラ・測位パイプライン。M5Stack 由来のコードは含みません。

## それ以外（本リポジトリ独自に書かれたコード）— MIT

MIT License, Copyright (c) 2026 Kouhei Ito

`components/uwb_port/` など、本リポジトリのために新規に書かれたコードは、リポジトリ直下の
[`LICENSE`](LICENSE) と同一の MIT ライセンスです。
