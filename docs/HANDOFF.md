# 次セッションへの申し送り (2026-08-21 時点)

**このファイルを最初に読むこと。** 全体像・決定事項・落とし穴・次の一手をまとめてある。

---

## 0. 次セッションの任務

> **実機検証の前に追加で読む文書は無い。** `docs/REVIEW_2026-08-21.md` の Critical 1・High 6 は
> 本日中に**全件対応済み**（§0 の表参照。#6 外れ値棄却・#7 ライセンス同梱も本セッションで解消した）。
> 残っているのは実機でしか確認できない項目だけである。

### 【追記 2026-08-22】ブランチ `feat/stamps3-fpc-migration` での変更

前回 HANDOFF 時点（HEAD `dffcde5`、main）から本ブランチで6コミット
（`git log --oneline main..HEAD` で確認可能。一覧は §1）を積み、ハードウェア構成の
既定を切り替えた:

- **アンカー5台＋据置タグ1台を M5StampS3A + StampS3 BreakOut に統一**（`UWB_ANCHOR_BOARD_STAMPS3` /
  `UWB_TAG_BOARD_STAMPS3` が既定）。**AtomS3(R) は削除せず代替として残す。**
  UWB モジュールは S017-F を 0.5mm 12P FPC + FPC→DIP 変換基板で接続する（`docs/WIRING.md` 経路A）
- **StampFly 搭載タグは M5StampS3A 背面の 12P FPC 経由に切り替え**（`UWB_TAG_BOARD_STAMPFLY`、
  `boards/stampfly.h` 全面改訂）。**旧 GROVE 2系統4本構成は廃案**（RST/IRQ/WAKEUP が取れず、
  GROVE が電池電圧直結・満充電約4.35V で絶対最大定格4.0Vを超えるため LDO が必須だった。
  新経路は背面 FPC の VDD_3V3 に直結できるため LDO 不要）
- RST/IRQ/WAKEUP が全構成で取れるようになり、**`uwb::TimingProfile::BothIrq`（90Hz）が
  初めて成立する**。CI に `anchor-stamps3-ds-bothirq` / `tag-stampfly-ds-bothirq` を追加済み。
  **既定は引き続き `PollingBoth` のまま**（実機で Phase 1〜2 が通ってから段階的に上げる方針は不変）
- ドキュメント構成を整理: `SOLDER_PADS.md`→`WIRING.md`、`BRINGUP.md`→`GETTING_STARTED.md` §3〜§4
  に統合、`UNITS.md`→`GLOSSARY.md` に統合、`PLATFORM_TUNING.md`→`PERF_ANALYSIS.md` の付録に統合、
  `MATH_AUDIT_2026-08-21.md`→`docs/archive/` へ移動

**このブランチはまだ `main` に未マージ・未 push。** 次セッションはまず差分をレビューし、
問題なければ `main` へマージすること。

**実機が届いたらこの順で進める**（詳細・判断基準は
**[`docs/EXPERIMENT_PLAN.md`](EXPERIMENT_PLAN.md)**）:

1. `firmware/probe` — Device ID `0xDECA0314` が読めるか（最初の関門）
2. `firmware/twr` — **SS-TWR から**。DS-TWR は Critical バグ（PRETOC 誤設定）を修正済みだが
   実機では未検証なので後回しにする
3. `firmware/anchor` × 5 + `firmware/tag` × 1 — フル測位

**起動直後に必ず確認するログ3行**（`docs/EXPERIMENT_PLAN.md` §10 #12 にも記載。
出なければ即座に切り分けへ）:

1. `... spi: slow=... fast=... active=16000000` — SPI が 2MHz に張り付いていないか
   （レビュー H-2 の修正確認）
2. `... main task stack high-water mark: ... bytes` — メインタスクを 12288B に
   引き上げた効果の確認（レビュー H-1。残量が小さければ専用タスクへの分離を検討）
3. `INIT_FAILED` が **出ないこと**（`dwt_checkidlerc()` 待ちタイムアウト。レビュー M-2 の対策確認）

**実機到着前にできることはまだ残っている。** 旧版の「すぐ着手できる」項目 A〜F、レビュー #6
（外れ値棄却）・#7（ライセンス同梱）は前々回・前回セッションで完了・コミット・push 済み
（`dffcde5`、main、CI green）。その後 `feat/stamps3-fpc-migration` ブランチでハードウェア構成の
既定を切り替えたことで、**実機を待たずに進められる作業が新たに生まれている**:

- **FPC→DIP 変換基板の型番選定と接点面（同面／異面）の確認**（`docs/EXPERIMENT_PLAN.md` §10 #13）
- **M5StampS3A 背面 12P FPC コネクタの入手**（HDGC/0.5K-HX-12PWB。出荷時は未実装で後付けが要る）
- **StampS3 BreakOut の PinMap 確認**（3V3/GND のヘッダ位置。同 §10 #15）

配線図（`docs/WIRING.md`）とピン定義（`boards/stamps3.h` / `boards/stampfly.h`）はすでに
確定しているので、部材さえ揃えばこれらは実機（UWB モジュール本体）の到着を待たずに進められる。
それ以外に強いてやるなら以下の任意項目のみ:

- `uwb_math` の `uwb_sym3_solve_sphere` と Beck のλ探索の共通化（反復の形が同型。任意）
- `v0.2.0` を切るかどうかの判断（§1 Release 参照。全件コミット済みなので
  `git tag v0.2.0 && git push --tags` だけで CI が zip を作る）

---

## 1. 現在地

**実機未着。すべてビルド通過とホスト検証まで。実機で Device ID すら読めていない。**
製品自体が新しく、コミュニティの実績も無い（ユーザ談）。

| リポジトリ | 状態 |
|---|---|
| **m5stamp_uwb_localizer**（本体） | GitHub 公開済み `kouhei1970/m5stamp_uwb_localizer`（public）。`main` の HEAD は `dffcde5`。**現在の作業ブランチ `feat/stamps3-fpc-migration`（HEAD `9730d66`、main から6コミット、未マージ・未 push）で作業中**。ワークツリーには本ブランチの文書更新に伴う未コミットの差分がある |
| **uwb_localizer**（上流） | **2026-08-21 に凍結・独立**（`42daea9`。上流 `667551e` を取り込み）。以後 `components/uwb_loc/` は本リポジトリで独立して開発する。**上流は見ない** |
| stampfly_ecosystem | `third_party/stampfly_ecosystem` に読み取り専用クローンあり。**書き込み禁止** |
| 一次資料（PDF/公式API） | `docs/refs/`。**`.gitignore` 済み**（再配布禁止文書を含む） |

### 前回 HANDOFF（`d9a7dbc`）以降のコミット（HEAD `dffcde5`。`42daea9` のみ CI 失敗）

`42daea9` は float ビルドの loc テストが落ちた（gcc の FMA 縮約に関する事故。§3「float の検証」の由来）。
`98385fe` で修正済み。他はすべて CI 緑。

| コミット | 内容 |
|---|---|
| `bce1096` | 実機投入前の最終レビューと Critical 1・High 3 修正（DS-TWR の PRETOC、SPI 16MHz 切替、タグのメインタスクスタック、`decamutexon`） |
| `42daea9` | 上流 `uwb_localizer` を凍結（上流 `667551e`）・最終状態に同期・独立宣言（**CI 失敗**。float の loc テストが FMA 縮約事故で落ちた） |
| `4298c08` | `tests/host/` へホストテストを統合、`docs/archive/` を新設、線形代数コンポーネント `uwb_math` を新設 |
| `98385fe` | `uwb_loc` / `uwb_survey` を `uwb_math` ベースのスカラー展開に書き換え、一般 LU / Jacobi を全廃（`42daea9` の float ビルド失敗を修正） |
| `4de9c68` | 外部の実機報告（GOROman 氏 gist）を反映、`begin()` にソフトリセットを追加 |
| `4ab98e2` | 公式回路図で電源系統・FPC/パッド対応を確定、電源電圧の記述を裏付け直す |
| `dffcde5` | 測量の leave-one-out 外れ値棄却・キラリティ入力、`uwb_math` へ LDLᵀ / `solve_sphere` / `null_vector` 統合、Release zip へのライセンス同梱、`firmware/soltest/.cache/clangd/index/*.idx` の untrack |

### `dffcde5` でコミットした内容（前回 HANDOFF 時点では未コミットだったもの）

1. **Release zip へのライセンス同梱**（レビュー #7 対応）
   `.github/workflows/build.yml` / `README.md` / `THIRD_PARTY_LICENSES.md` / `docs/PREBUILT_BINARIES.md`。
   各 zip に `LICENSE` / `THIRD_PARTY_LICENSES.md` / `LICENSES/LicenseRef-QORVO-2.txt` を同梱し、
   Release 本文と `PREBUILT_BINARIES.md` に `LicenseRef-QORVO-2` 条件3（Qorvo 製 IC 限定）を明記した。
2. **測量の外れ値 leave-one-out 棄却（A-2）とキラリティ入力（A-4）**（レビュー #6 対応）
   `components/uwb_survey/{include,src}` / `docs/SURVEY_SPEC.md` / `tests/host/survey/test_survey.c`。
   `uwb_survey_input.chirality`、`uwb_survey_result.outlier_ambiguous` / `chirality_margin` を追加。
3. **`uwb_math` へ LDLᵀ・`solve_sphere`・`null_vector` 等を統合**
   `components/uwb_math/{include,src}` / `components/uwb_loc/src/{uwb_closed_form.c,uwb_internal.h,uwb_nls.c}` /
   `docs/archive/MATH_AUDIT_2026-08-21.md` / `tests/host/math/test_math.c`。
   `uwb_loc` 側・`uwb_survey` 側とも新 API への差し替えが完了している。

### `feat/stamps3-fpc-migration` ブランチのコミット（`main`=`dffcde5` から6コミット、HEAD `9730d66`。未 push）

| コミット | 内容 |
|---|---|
| `bc92494` | StampS3A 用アンカーバイナリを配布（CI に `anchor-stamps3-ds` / `-fast` を追加）、旧 `BRINGUP.md` の FPC 番号誤記を修正 |
| `5044f46` | アンカーを StampS3A 既定にし、StampFly を背面 12P FPC 接続へ切替（`boards/stampfly.h` 全面改訂、CI variant 14→18）。旧 GROVE 4線構成を廃案に |
| `2f9d8df` | M5StampS3A の公式回路図（`assets/Sch_StampS3_v0.3.3.pdf`）を追加。`boards/stampfly.h` の一次資料 |
| `6578065` | `BRINGUP.md` を `GETTING_STARTED.md` §3〜§4 へ統合 |
| `7f84e2d` | `PLATFORM_TUNING.md` / `UNITS.md` を統合、`MATH_AUDIT_2026-08-21.md` を `docs/archive/` へ移動 |
| `9730d66` | `SOLDER_PADS.md` を `WIRING.md` へ改組し配線の正本を1本化（経路A: FPC→DIP変換基板 / 経路B: 半田パッド直付け / 経路C: StampS3A背面12P FPC の新設を含む） |

詳細は各コミットメッセージおよび `docs/EXPERIMENT_PLAN.md` / `docs/STAMPFLY_INTEGRATION.md` /
`docs/WIRING.md` を参照。

### ホストテスト（`make -C tests all` = test / strict / float を再実行して確認。float ビルドの回帰は 591,184 件）

| スイート | 件数 | 失敗 |
|---|---:|---:|
| loc（`test_uwb.c`） | 77 | 0 |
| loc（`test_regress.c`、新旧比較回帰） | 591,418 | 0 |
| math | 10,781 | 0 |
| pipeline | 188 | 0 |
| survey | 561 | 0 |

（math は前回の 3,423〜3,439 件から `uwb_math` への LDLᵀ / `solve_sphere` / `null_vector` 追加で
10,781 件に増えた。survey は前回の 365 件から A-2 / A-4 のケース追加で 561 件に増えた。）

### ファームウェア（6種）

`firmware/{probe,devtest,twr,soltest,tag,anchor}`。

- 直近 CI（`dffcde5`, 3m6s, green）で 6 種ともカバー済み。
- `soltest` / `tag` / `anchor` は `uwb_loc` 経由で `uwb_math` を使うため、本セッションで
  `idf.py fullclean && idf.py build` により再確認した。**3種とも警告 0・エラー 0**
  （`soltest` は `uwb_survey` も直接リンクしており、A-2/A-4 込みで確認できている）。

### CI

直近 push（`dffcde5`）は green。未 push の差分は無い。

### Release

`v0.1.0`（タグは `9be9000`、2026-08-21 13:17 作成）は**古い**。以後 10 コミット
（レビューの Critical/High 修正、上流凍結、`uwb_math` 化、GOROman 報告反映、回路図確認、
ライセンス同梱を含む）が乗っていない。**特に DS-TWR の Critical バグ修正
（`bce1096`）より前なので、`v0.1.0` の DS-TWR ビルドは実機で確実に失敗する。**
`v0.2.0` を切るなら `git tag v0.2.0 && git push --tags` するだけで CI が
14 通りの zip をライセンス同梱込みで作る。

---

## 2. 何を作ったか

```
components/
  qm33120w_sdk/   Qorvo ドライバ (vendoring, SPDX 保持)              ← 一次資料。信頼してよい
  uwb_port/       dwt_spi_s / deca_sleep / mutex / GPIO              ← 自作
  uwb_qm33120/    デバイス層 + SS/DS-TWR                              ← M5Stack 由来を大幅に修正
  uwb_math/       対称3x3/2x2の閉形式・LDLᵀ・ブロックコレスキー      ← 新設。一般LU/Jacobiは置かない
  uwb_loc/        測位ソルバ Lv0-Lv3（uwb_math ベースに書き換え済み） ← 上流凍結後は本リポジトリで独立開発
  uwb_ranging/    アンカーテーブル + スケジューラ + パイプライン
  uwb_cfgstore/   NVS 永続化 + シリアルコンソール
  uwb_survey/     MDS(逐次三辺測量) + Gauss-Newton + ゲージ固定       ← uwb_math ベース。leave-one-out棄却・キラリティ入力を追加
firmware/         probe / devtest / twr / soltest / tag / anchor
tests/host/       loc（77+回帰59万） / math（10781） / pipeline（188） / survey（561）
tools/            bench_loc（測位ソルバのマイクロベンチ）
docs/archive/     経緯文書（PROGRESS / REIMPL_PLAN / CRITICAL_REVIEW / SURVEY_* など計8本）
```

**ハード依存は `uwb_port` と `uwb_ranging` のスケジューラ部分の2箇所だけ**に隔離。
測位パイプラインと測量計算はホストで検証できる。**線形代数はすべて `uwb_math` に集約**
されており、一般次元の LU 分解・Jacobi 法などは存在しない（設計根拠:
`docs/archive/MATH_AUDIT_2026-08-21.md`）。

---

## 3. 確定した仕様・決定事項

| # | 決定 | 文書 |
|---|---|---|
| 対象 | **ESP32-S3 + M5Stamp UWB Module 専用**。プラットフォーム最適化してよい。ただし StampFly には非依存 | `docs/PLAN.md` |
| **ハード方針** | **StampFly 非依存。ただしタグの配線だけは StampFly 互換を維持する**（GROVE 2系統4本で成立 ＝ IRQ/RST 不要）。想定利用者は本リポジトリを単体で試す人 | `docs/PLAN.md` §1 |
| 役割 | **タグ = M5StampS3A ×1（既定は据置＝StampS3 BreakOut 経由。StampFly 搭載時は機体の M5StampS3A を背面 12P FPC 経由で流用） / アンカー = M5StampS3A + StampS3 BreakOut ×5（既定、`UWB_ANCHOR_BOARD_STAMPS3`）。AtomS3(R) は代替として残る** | `docs/archive/PROGRESS.md`、本ブランチ `feat/stamps3-fpc-migration`（§0・§1） |
| 接続 | **FPC ではなく半田パッド**（1.27mm キャステレーション）。J1（FPC 用番号）と PINMAP（パッド用番号）は**別の番号体系**で両方正しい | `docs/WIRING.md` §5.5 |
| 電源 | パッド2（VCC_3V3）は QM33120W の VDD1/VDD2 に直結。動作上限 3.6V・絶対最大 4.0V。5V や StampFly GROVE（満充電 ~4.35V）は不可 | `docs/WIRING.md` §5.4 |
| **IRQ** | **アンカーは積極使用。タグは不使用。StampFly の別配線可能性は残す** | **`docs/IRQ_POLICY.md`** |
| 資料 | **一次資料 = Qorvo SDK/UM/APS。M5Stack ラッパは二次資料で信頼しない** | **`docs/PLAN.md` §5** |
| 測量 | 高さのみ実測(4点以上) + キラリティ1ビット入力（`uwb_survey_input.chirality`）。タグも6台目として参加。外れ値リンクは leave-one-out で棄却。計算は実機上（呼び出し元はまだ無い） | `docs/SURVEY_SPEC.md` |
| StampFly統合 | **案B-2 疎結合**: Lv2 で位置を出し `EskfCore::vectorUpdate3()` で POS_X/Y 観測 | `docs/STAMPFLY_INTEGRATION.md` |
| **数値計算方針** | **一般次元の行列計算（LU/Jacobi/一般コレスキー）は置かない。** 対称3x3/2x2の閉形式と nb≤8 のブロックコレスキーに集約し `uwb_math` に一本化する。ESP32-S3 特化（スカラー展開・単精度FPU前提の最適化）は OK | `docs/archive/MATH_AUDIT_2026-08-21.md` |
| **上流の扱い** | `uwb_localizer` は 2026-08-21 に凍結・独立（`42daea9`）。以後 `components/uwb_loc/` は本リポジトリで独立開発する。**上流は見ない**（孵化器は役目を終えた） | — |
| **float の検証** | clang に加え **gcc-16 と `-ffp-contract=off` でも**通すこと（FMA 縮約に助けられて clang だけ通っていた事故があった） | `tests/Makefile` |
| **ブラウザ自動化** | 使わない（§4 参照） | — |

---

## 4. 過去の誤りと運用ルール

**同じ失敗を繰り返さないために記録がある。**
パターンは1つに集約される:
**「二次的な指標を、直接証拠より優先した」「確認せずに断定した」。**

- **誤り 15 件の一覧と詳細** → [`archive/DESIGN_HISTORY.md` §3](archive/DESIGN_HISTORY.md)
- **調査の運用ルール**（直接証拠を優先／矛盾したら前提を疑う／`grep` 0 件はファイルを疑う／
  資料はハッシュ照合／**ブラウザ自動化は使わない**）→ [`PLAN.md` §5](PLAN.md)

## 5. 文書の地図

現役 22 本（+ 経緯を記録した archive 8 本）。索引は **[`docs/README.md`](README.md)**。

```
README.md                    購入者の入口（リポジトリ直下）
docs/README.md                ★ ドキュメント索引（ここから探す）
docs/HANDOFF.md               ← このファイル
docs/GLOSSARY.md              用語集（略語の正式名称と意味）。分からない略語はここ
docs/UWB_PRIMER.md            UWB 入門。なぜ電波で cm が測れるのか（最初に読む）
docs/UWB_ALGORITHMS.md        測位アルゴリズムの導出（上流からの移植・改訂版）
docs/EXPERIMENT_PLAN.md       実機到着後の実験計画とフラグ有効化の順序。実機でしか潰せない前提12件
docs/IRQ_POLICY.md            IRQ 方針（確定版）
docs/GLOSSARY.md                 UUS/DTU/実µs の単位リファレンス（遅延値を触る前に必読）
docs/TIMING_PRESETS.md        遅延プリセットとバージョン不一致検出（設計）
docs/SURVEY_SPEC.md           自動測量の仕様（外れ値 leave-one-out・キラリティ入力を追記）
docs/STAMPFLY_INTEGRATION.md  StampFly 位置制御への統合（1127行）
docs/PERF_ANALYSIS.md         測位計算の性能分析と上流最適化の結果
docs/PERF_ANALYSIS.md       ESP32-S3 の浮動小数点・コンパイル設定
docs/archive/MATH_AUDIT_2026-08-21.md 行列計算の残存箇所の監査とスカラー化の設計根拠。uwb_math の仕様の元
docs/ANCHOR_PLACEMENT.md      アンカー配置ルール
docs/WIRING.md           パッド仕様と配線（公式回路図でのネットリスト確認込み）
docs/GETTING_STARTED.md               Phase 1 受入確認
docs/PLAN.md                  全体設計・フェーズ計画
docs/GETTING_STARTED.md       BOM から測位まで11章
docs/PREBUILT_BINARIES.md     ビルド済みバイナリの入手と書き込み。ライセンス条件込み
docs/REVIEW_2026-08-21.md     実機投入前の最終レビュー。Critical 1・High 6、全件対応済み
docs/refs/README.md           一次資料の索引
docs/archive/                 経緯文書（現役8本 + archive自身のREADME.md）
  CRITICAL_REVIEW.md            M5Stack ラッパの批判的レビュー
  REIMPL_PLAN.md                R1-R12。R1/R2/R3-1/R4/R7/R8/R9 は実装済み
  PROGRESS.md                   開発進捗ログ
  SURVEY_*.md（5本）            設計当時の事前調査資料
```

---

## 6. 次にやること

### 実機が要る
| # | 内容 |
|---|---|
| **Phase 1 受入** | `firmware/probe` で Device ID `0xDECA0314`。**ピン配線が未検証なのでここが最初の関門** |
| R5 | 遅延値の追い込み（公式手順: `dwt_starttx()` が `DWT_ERROR` を返すまで下げる） |
| R6 | **アンカー側の** IRQ 駆動化 — **コードは実装済み**（`docs/IRQ_POLICY.md`「実装状況」節）。残るのは実機での極性検証（`gpio_config()`のアクティブHIGH前提が正しいか）だけ |
| R10-R12 | ch9 の PLL 再校正(20°Cごと)、アンテナ遅延校正、診断情報→測位重み |
| S3-S5 | ESP-NOW、測量モード、コーディネータ |
| S7 | I2C ToF（高さ自動計測） |
| **float 既定化** | 実機 `soltest` で double/float を比較し `CONFIG_UWB_LOC_USE_FLOAT` の既定値を判断する |

### 任意（実機不要、優先度低）
| # | 内容 |
|---|---|
| `uwb_math` の共通化 | `uwb_sym3_solve_sphere` と Beck のλ探索の共通化（反復の形が同型） |
| 測量 leave-one-out の計算量 | n=8 の外れ値入りで最大 ~5000 回のコレスキー/solve（S3 の double で数秒の可能性）。設置時1回として許容しているが、実機で時間を測るとなお良い |
| 測量の2本外れ値（同一ノード共有） | greedy leave-one-out の既知の限界（総当たりの leave-two-out は組合せ数が多すぎる）。記録のみで未対応 |
| SPDX ヘッダ | `components/qm33120w_sdk/` 以外に SPDX ヘッダ・著作権表示が無い。レビュー §5（Low-Medium） |

---

## 7. 落とし穴（踏むと時間を失う）

1. **Qorvo 公式サンプルの `.c` は latin-1。`grep -a` を使うこと**
2. **`UUS_TO_DWT_TIME` は DW1000=65536 / DW3000公式=63898。**
   Qorvo の `*_UUS` 定数は実は実マイクロ秒。そのまま流用すると2.5%ずれる
   → **`docs/GLOSSARY.md`**
3. **ESP-IDF ビルドとホスト `make strict` は警告設定が違う。**
   `uwb_survey.c` が `-Werror=maybe-uninitialized` で落ちた実例あり。
   **新規コンポーネントは必ず `idf.py build` も通すこと**
4. **シェルの cwd が持続する。** `cd firmware/anchor` した後に `cat >> PROGRESS.md` して
   迷子ファイルを作った実例あり。**絶対パスを使うこと**
5. **`sdkconfig` は `.gitignore` 済み。** `sed` で書き換える手順を書いてはいけない。
   `-D SDKCONFIG=build/xxx/sdkconfig` 方式を使う（`GETTING_STARTED.md` に検証済みの手順あり）
6. **`components/uwb_loc/` はもう「上流と byte 一致」ではない。**
   上流 `uwb_localizer` を凍結・最終取り込みした後は、`components/uwb_loc/` は本リポジトリで
   独立して開発する方針に転換済み（2026-08-21）。ESP32-S3 向けの最適化（`uwb_math` ベースの
   スカラー展開など）をソースに直接入れてよい
7. サブエージェントに Web 調査を任せるときは**取得手段を明示的に限定する**
8. **複数セッションが同じファイルを同時に編集していることがある。**
   本セッション中、`components/uwb_survey/src/uwb_survey.c` が別セッションの編集途中で
   一瞬ビルドエラーになる瞬間を観測した。ビルド失敗を見たら即座に「壊れた」と判断せず、
   数分おいて `git diff` / 再ビルドで再確認すること

---

## 8. 数字の要約（実機で最初に検証すべきもの）

| 項目 | 値 | 出典 |
|---|---|---|
| 測位レート（現状・5アンカー） | **31.3 Hz**（1周 31.9ms） | `STAMPFLY_INTEGRATION.md` |
| 同（アンカーのみ IRQ） | **59.4 Hz** | 同上 |
| StampFly 位置制御の実効帯域 | **約 0.064 Hz**（`pos.kp=0.4`） | `params.cpp:771` |
| アンカー座標の推定誤差（σ=5cm時） | RMS 5cm / 最悪 25cm | `test_survey` |
| 共通アンテナ遅延の推定誤差 | 平均 1.5cm | 同上 |
| 外れ値棄却の誤棄却率（σ=5cm ノイズのみ） | **≤0.35%**（2000試行実測、n=8） | `test_survey`（A-2） |
| **無校正のアンテナ遅延バイアス** | **Δ1ns = 30cm** | APS014 |
| 電源電圧の絶対最大定格 | **4.0V**（動作上限 3.6V。5V/StampFly GROVE 4.35V は不可） | `QM33120W_DS.txt` / 公式回路図 |
| ESP32-S3 の float | 加減乗と `madd.s` はハード、**除算と sqrt はソフト** | 逆アセンブルで確認 |

**StampFly の ESKF には POS_X/POS_Y を直接観測する update 関数が1つも無く、
水平位置は速度の積分だけ（実質デッドレコニング）。UWB が埋めるのはそこ。**
