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
`c/src/{uwb_internal,uwb_linalg}.h`）を取り込んだものです。取り込み元は上流ブランチ
`perf/exploit-structure`（コミット `ab23b33`、2026-08-21 に上流 `main` へマージ）の
最終状態で、ESP-IDF 用の `CMakeLists.txt` / `Kconfig` を追加しています。

**2026-08-21 に上流 `uwb_localizer` を凍結し、この最終状態を取り込んだ。以後
`components/uwb_loc/` は本リポジトリで独立して開発する（上流はもう追わない）。**
このため以後の変更は本リポジトリのコミット履歴を参照してください。ライセンスは
引き続き MIT（表記は変更なし）です。全文は上流の
[`LICENSE`](https://github.com/kouhei1970/uwb_localizer/blob/HEAD/LICENSE)（本リポジトリの
[`LICENSE`](LICENSE) と同一の MIT）を参照してください。

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
M5Stack 版とは挙動が異なります**（詳細は [`docs/archive/CRITICAL_REVIEW.md`](docs/archive/CRITICAL_REVIEW.md)
と [`docs/archive/REIMPL_PLAN.md`](docs/archive/REIMPL_PLAN.md)）。移植元と同じ動作を期待しないでください。

## `components/uwb_ranging/` — MIT（本リポジトリ独自）

MIT License, Copyright (c) 2026 Kouhei Ito

アンカー登録テーブル・測距スケジューラ・測位パイプライン。M5Stack 由来のコードは含みません。

## それ以外（本リポジトリ独自に書かれたコード）— MIT

MIT License, Copyright (c) 2026 Kouhei Ito

`components/uwb_port/` など、本リポジトリのために新規に書かれたコードは、リポジトリ直下の
[`LICENSE`](LICENSE) と同一の MIT ライセンスです。

## `assets/` 内の M5Stack 由来ファイル — MIT の対象外（引用・参照目的）

以下のファイルは M5Stack Technology CO., LTD. が公式ドキュメントサイト
（docs.m5stack.com）の Stamp UWB 製品ページで配布している画像・回路図です。
著作権は M5Stack Technology CO., LTD. に帰属し、本リポジトリの MIT ライセンスの
対象**外**です。本リポジトリでは、配線・電源仕様の検証といった**引用・参照目的**で
同梱しています。

| ファイル | 内容 | 出典 |
|---|---|---|
| `assets/S017_Stamp_UWB_pinmap.jpg` | 公式ピンマップ画像（パッド / FPC のピン配置図） | M5Stack 公式ドキュメント（[Stamp UWB 製品ページ](https://docs.m5stack.com/en/stamp/Stamp_UWB)） |
| `assets/S017_Stamp_UWB_main_pictures_01.webp` | 製品写真 | 同上 |
| `assets/SCH_UWB_MODULE_SCH_main_V0.2_20251128_2026_06_01_11_54_35.pdf` | 公式回路図（V0.2、2025-11-28） | 同上 |

再配布・改変を行う場合は、各ファイルの著作権が M5Stack Technology CO., LTD. に
帰属することを別途確認してください。

## 配布バイナリ（Release zip）について

GitHub Release / Actions artifact で配布している各 zip（`probe-stamps3.zip` 等）には、
`merged-firmware.bin` 等のバイナリに加えて次のライセンス文書を同梱しています。

- `LICENSE` — 本リポジトリのライセンス本文（MIT）
- `THIRD_PARTY_LICENSES.md` — 本ファイル
- `LICENSES/LicenseRef-QORVO-2.txt` — Qorvo ドライバのライセンス全文

これは上記 `LicenseRef-QORVO-2` の条件2（バイナリ再配布時は上記の著作権表示・
条件・免責事項を配布物に含めること）を満たすためです。**条件3により、この
バイナリは Qorvo 製 IC（本モジュール = M5Stamp UWB Module に搭載された
QM33120W/DW3720、またはそれを内蔵するモジュール）と共に使う場合に限り
利用が許可されており、他ベンダーのチップへ転用することはできません。**
詳細は [`docs/PREBUILT_BINARIES.md`](docs/PREBUILT_BINARIES.md) を参照してください。
