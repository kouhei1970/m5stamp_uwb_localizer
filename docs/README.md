# ドキュメント索引

現役 21 本・約 9,000 行あります（他に、設計当時の調査・経緯を記録した
[archive](archive/) が 8 本）。**全部読む必要はありません。**
下の「あなたはどれですか」から入ってください。

---

## あなたはどれですか

### 🔰 A. UWB を知らない。原理から勉強したい

```
UWB_PRIMER.md  →  UWB_ALGORITHMS.md  →  ANCHOR_PLACEMENT.md
   原理             測位の数理            配置で精度が決まる話
```

| # | 文書 | 何が分かるか |
|---|---|---|
| 1 | **[UWB_PRIMER.md](UWB_PRIMER.md)** | **ここから。** なぜ電波で cm が測れるのか。1 ns = 30 cm、帯域と時間分解能、TWR がなぜ時計同期を要らなくするか、アンテナ遅延、NLOS |
| 2 | [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | 距離から位置を解く数理。Lv0 閉形式 → Beck 厳密解 → ロバスト化 → EKF まで式を追える（**1,700 行。重い**） |
| 3 | [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | アンカーをどう置くか。同一平面の罠と GDOP |

**分からない略語が出たら [GLOSSARY.md](GLOSSARY.md)、単位で混乱したら [GLOSSARY.md](GLOSSARY.md)。**

### 🔧 B. 買った。動かしたい

```
GETTING_STARTED.md  →  EXPERIMENT_PLAN.md  →  GETTING_STARTED.md
   通しの手順            進める順序と判断        最初の関門の詳細
```

| # | 文書 | 何が分かるか |
|---|---|---|
| 1 | **[GETTING_STARTED.md](GETTING_STARTED.md)** | **ここから。** BOM から測位まで 11 章の完全手順（**1,200 行**） |
| 2 | **[EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md)** | **何を・どの順で確かめ、どのフラグをいつ有効にするか。** 実機でしか潰せない前提12件つき |
| 3 | [PREBUILT_BINARIES.md](PREBUILT_BINARIES.md) | **ESP-IDF を入れずに**書き込んで試す |
| 4 | [GETTING_STARTED.md](GETTING_STARTED.md) | 実験1（SPI 疎通で Device ID を読む）の受入基準と切り分け |
| 5 | [WIRING.md](WIRING.md) | 半田パッドの寸法・**向きの確定**・アンテナ禁止領域 |
| 6 | [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | アンカーの置き方 |

> **⚠ 配線前に [WIRING.md](WIRING.md) と
> [GETTING_STARTED.md §3.1](GETTING_STARTED.md#orientation) を必ず読んでください。**
> パッドの向きは実機で確認が必要で、間違えると電源逆接で壊れます。

### 🛠 C. 中身を読みたい・改造したい

```
PLAN.md  →  各仕様書  →  archive/REIMPL_PLAN.md / archive/CRITICAL_REVIEW.md
 全体設計    決まりごと      なぜこうなっているかの経緯
```

| 文書 | 何が分かるか |
|---|---|
| **[PLAN.md](PLAN.md)** | **ここから。** 全体設計・フェーズ計画・成果物の定義。**§1 に「StampFly 非依存だがタグのハードは互換に保つ」方針** |
| [IRQ_POLICY.md](IRQ_POLICY.md) | IRQ を使うかどうかの方針。**ポーリングを第一級で残す理由** |
| [TIMING_PRESETS.md](TIMING_PRESETS.md) | TWR 遅延値の**導出**と、タグ/アンカー間の版不一致検出 |
| [SURVEY_SPEC.md](SURVEY_SPEC.md) | アンカー座標の自動測量（MDS + Gauss-Newton + ゲージ固定） |
| [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md) | StampFly の位置制御へ載せる設計検討（**1,100 行**） |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | ESP32-S3 固有の最適化調査 |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | 測位ソルバの性能分析 |
| [MATH_AUDIT_2026-08-21.md](archive/MATH_AUDIT_2026-08-21.md) | 行列計算の残存箇所の監査とスカラー化の設計根拠。`components/uwb_math/` の仕様の元 |
| [REVIEW_2026-08-21.md](REVIEW_2026-08-21.md) | 実機投入前の最終レビュー（2026-08-21）。Critical 1・High 6 を含む全指摘と根拠、対応状況、着手順 |
| [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md) | 【経緯】移植元の課題一覧 R1〜R12 と、それぞれの決着 |
| [archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) | 【経緯】移植元コードの批判的レビュー（**訂正ボックス入り**） |

### 📋 D. 引き継ぐ / 続きをやる

| 文書 | 何が分かるか |
|---|---|
| **[HANDOFF.md](HANDOFF.md)** | **最初に読む。** 現在地・確定事項・落とし穴・次の一手 |
| [REVIEW_2026-08-21.md](REVIEW_2026-08-21.md) | 実機投入前の最終レビュー（2026-08-21）。Critical 1・High 6 を含む全指摘と根拠、対応状況、着手順 |
| **[SOURCE_POLICY.md](SOURCE_POLICY.md)** | **資料の格付けと、これまでに犯した誤りの記録。** 同じ失敗を繰り返さないために |
| [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) | 実機が来てからやること |

---

## 目的別インデックス

| やりたいこと | 読むもの |
|---|---|
| 略語の意味を知りたい | [GLOSSARY.md](GLOSSARY.md) |
| `*Uus` の単位で混乱した | [GLOSSARY.md](GLOSSARY.md) |
| **とにかく実機に書き込みたい** | [PREBUILT_BINARIES.md](PREBUILT_BINARIES.md) |
| Device ID が読めない | [GETTING_STARTED.md](GETTING_STARTED.md) §切り分け、[GETTING_STARTED.md §4.3](GETTING_STARTED.md#probe-troubleshoot) |
| 距離が一定量ずれる | [GETTING_STARTED.md §9](GETTING_STARTED.md#antenna-delay)（アンテナ遅延の校正） |
| 位置が出ない / 飛ぶ | [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md)、[GETTING_STARTED.md §10](GETTING_STARTED.md#troubleshooting) |
| 測位を速くしたい | [IRQ_POLICY.md](IRQ_POLICY.md) → [TIMING_PRESETS.md](TIMING_PRESETS.md) → [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) §7・§8 |
| アンカー座標を測るのが面倒 | [SURVEY_SPEC.md](SURVEY_SPEC.md) |
| 半田付けの前に確認したい | [WIRING.md](WIRING.md) |
| StampFly に載せたい | [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md)、[PLAN.md §1](PLAN.md) |
| なぜこの実装なのか知りたい（経緯） | [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md)、[archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) |
| 何が検証済みで何が未検証か | [HANDOFF.md](HANDOFF.md) §1、[GETTING_STARTED.md §11](GETTING_STARTED.md#limitations) |

---

## 全文書一覧

### 入門・リファレンス
| 文書 | 行 | 内容 |
|---|---:|---|
| [UWB_PRIMER.md](UWB_PRIMER.md) | 278 | UWB の原理。なぜ電波で cm が測れるのか |
| [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | 1796 | 測位アルゴリズムの導出（上流 uwb_localizer からの移植・改訂版） |
| [GLOSSARY.md](GLOSSARY.md) | 191 | 用語集。**§9 に「紛らわしい語」**（ToF の二義など） |
| [GLOSSARY.md](GLOSSARY.md) | 157 | UUS / DTU / 実 µs。**遅延値を触る前に必読** |

### 実践
| 文書 | 行 | 内容 |
|---|---:|---|
| [GETTING_STARTED.md](GETTING_STARTED.md) | 1228 | BOM から測位までの完全手順（11 章） |
| [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) | 359 | 実機到着後の実験計画とフラグ有効化の順序 |
| [PREBUILT_BINARIES.md](PREBUILT_BINARIES.md) | 175 | ビルド済みバイナリの入手と書き込み。ライセンス条件込み |
| [GETTING_STARTED.md](GETTING_STARTED.md) | 193 | Phase 1（SPI 疎通）の受入確認 |
| [WIRING.md](WIRING.md) | 731 | 半田パッドの仕様・配線・向きの確定 |
| [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | 59 | アンカー配置ルール（実測にもとづく） |

### 設計・仕様
| 文書 | 行 | 内容 |
|---|---:|---|
| [PLAN.md](PLAN.md) | 317 | 全体設計・フェーズ計画・**リポジトリの方針** |
| [IRQ_POLICY.md](IRQ_POLICY.md) | 200 | IRQ の使用方針（確定版） |
| [TIMING_PRESETS.md](TIMING_PRESETS.md) | 295 | 遅延プリセットの導出と版不一致検出 |
| [SURVEY_SPEC.md](SURVEY_SPEC.md) | 432 | アンカー座標の自動測量の仕様（外れ値 leave-one-out・キラリティ入力を追記） |
| [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md) | 1128 | StampFly 位置制御への統合検討 |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | 126 | ESP32-S3 の浮動小数点・コンパイル設定 |
| [MATH_AUDIT_2026-08-21.md](archive/MATH_AUDIT_2026-08-21.md) | 303 | 行列計算の残存箇所の監査とスカラー化の設計根拠。`components/uwb_math/` の仕様の元 |

### 経緯・分析
| 文書 | 行 | 内容 |
|---|---:|---|
| [REVIEW_2026-08-21.md](REVIEW_2026-08-21.md) | 282 | 実機投入前の最終レビュー。Critical 1・High 6、**全件対応済み**（実機確認のみ残る） |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | 321 | 測位ソルバの性能分析と上流最適化の結果 |
| [SOURCE_POLICY.md](SOURCE_POLICY.md) | 136 | **資料の格付けと、犯した誤りの記録** |

### 引き継ぎ
| 文書 | 行 | 内容 |
|---|---:|---|
| [HANDOFF.md](HANDOFF.md) | 303 | 次セッションへの申し送り。現在地・確定事項・誤りと教訓・文書の地図・次の一手 |

---

## アーカイブ（経緯）

上記はすべて**現役の文書**です。それとは別に、[`archive/`](archive/) に
**設計当時の調査・検討の記録**（経緯文書）を 8 本まとめてあります。
プロジェクトの背景や「なぜ今の実装になったか」を追いたいときに読んでください。
**現役文書と矛盾する場合は、常に現役文書が正しい。** 索引は [`archive/README.md`](archive/README.md)。

| 文書 | 内容 |
|---|---|
| [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md) | 移植元の課題一覧 R1〜R12 とその決着 |
| [archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) | 移植元コードの批判的レビュー |
| [archive/SURVEY_m5stamp_uwb_module.md](archive/SURVEY_m5stamp_uwb_module.md) | モジュールのハードウェア仕様の事前調査 |
| [archive/SURVEY_m5stamp_uwb_port.md](archive/SURVEY_m5stamp_uwb_port.md) | Arduino → ESP-IDF 移植の対応表 |
| [archive/SURVEY_stampfly_grove.md](archive/SURVEY_stampfly_grove.md) | StampFly の GROVE 端子と GPIO 空き状況の事前調査 |
| [archive/SURVEY_stampfly_ecosystem.md](archive/SURVEY_stampfly_ecosystem.md) | StampFly 側のソフト構成の事前調査 |
| [archive/SURVEY_uwb_localizer.md](archive/SURVEY_uwb_localizer.md) | 上流測位ライブラリの調査 |
| [archive/PROGRESS.md](archive/PROGRESS.md) | 開発進捗ログ（著者の作業記録） |

---

## この文書群の約束ごと

書き足すときは以下に従ってください。**過去に何度も踏んだ失敗から作られた規則です。**

1. **略語は各文書の初出で「正式名称（英語）＝日本語の意味」を添える**（[GLOSSARY.md](GLOSSARY.md) 冒頭）。
   文書をまたぐときは各文書で改めて展開する
2. **断定には出典を `ファイル:行` で添える**（[SOURCE_POLICY.md](SOURCE_POLICY.md)）。
   一次資料は Qorvo の SDK / UM / APS。M5Stack のラッパは二次資料として扱う
3. **フラグやメタデータより、直接観測できる事実を優先する。**
   矛盾する証拠が出たら辻褄を合わせず前提を疑う
4. **実機で確認していないことは「未検証」と明記する。** このリポジトリは
   実機で一度も動作確認していないので、これを省くと読者を誤解させる
5. **`.gitignore` されているディレクトリ（`third_party/` / `docs/refs/`）へ
   Markdown リンクを張らない。** clone した人には切れる
6. **訂正は消さずに残す。** 何を間違えたかが次の人の役に立つ
   （[archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) の訂正ボックスが例）
