# ドキュメント索引

現役 17 本あります。**全部読む必要はありません。**
下の「あなたはどれですか」から入ってください。

**現役文書には「いま正しいこと」だけが書いてあります。**
廃案・訂正・方針変更の経緯は
[archive/DESIGN_HISTORY.md](archive/DESIGN_HISTORY.md) に退避してあるので、
「なぜこうなっているのか」を知りたいときだけそちらを開いてください。

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
| 2 | [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | 距離から位置を解く数理。Lv0 閉形式 → Beck 厳密解 → ロバスト化 → EKF まで式を追える（**長い**） |
| 3 | [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | アンカーをどう置くか。同一平面の罠と GDOP |

**分からない略語や単位で迷ったら [GLOSSARY.md](GLOSSARY.md)**（用語集と単位リファレンスの2部構成）。

### 🔧 B. 買った。動かしたい

```
GETTING_STARTED.md  →  EXPERIMENT_PLAN.md
   通しの手順            進める順序と判断
```

| # | 文書 | 何が分かるか |
|---|---|---|
| 1 | **[GETTING_STARTED.md](GETTING_STARTED.md)** | **ここから。** BOM から測位まで 11 章の完全手順 |
| 2 | **[EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md)** | **何を・どの順で確かめ、どのフラグをいつ有効にするか。** 実機でしか潰せない前提12件つき |
| 3 | [PREBUILT_BINARIES.md](PREBUILT_BINARIES.md) | **ESP-IDF を入れずに**書き込んで試す |
| 4 | [WIRING.md](WIRING.md) | **配線の正本。接続経路は3通り**（FPC→DIP 変換基板／半田パッド直付け／StampS3A 背面 FPC）。**§0.1 でまず自分の経路を選ぶ** |
| 5 | [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | アンカーの置き方 |
| 6 | [NET_DASHBOARD.md](NET_DASHBOARD.md) | 測位が動いたら次に読む。**Wi-Fi のブラウザダッシュボード + 無線コンソール**（USB を外してモバイルバッテリ運用する手順込み） |

> 実験1（SPI 疎通で Device ID を読む）の受入基準と切り分けは、
> [`GETTING_STARTED.md` §4](GETTING_STARTED.md#probe) を参照。

> **⚠ 配線前に [WIRING.md §1](WIRING.md#route) で自分の接続経路を選び、
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
| [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md) | StampFly の位置制御へ載せる設計検討 |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | 測位ソルバの性能分析と ESP32-S3 固有の最適化調査 |
| [ARCHITECTURE_V2.md](ARCHITECTURE_V2.md) | UWB 測距ファームウェアの v2 アーキテクチャ（アンカー受信常時 ON のステートマシン化） |
| [REVIEW_2026-08-21.md](archive/REVIEW_2026-08-21.md) | 実機投入前の最終レビュー。Critical 1・High 6 を含む全指摘と根拠、対応状況、着手順 |
| [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md) | 【経緯】移植元の課題一覧 R1〜R12 と、それぞれの決着 |
| [archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) | 【経緯】移植元コードの批判的レビュー（**訂正ボックス入り**） |
| [archive/MATH_AUDIT_2026-08-21.md](archive/MATH_AUDIT_2026-08-21.md) | 【経緯】行列計算の残存箇所の監査とスカラー化の設計根拠。`components/uwb_math/` の仕様の元（指摘は全件対応済み） |

### 📋 D. 引き継ぐ / 続きをやる

| 文書 | 何が分かるか |
|---|---|
| **[HANDOFF.md](HANDOFF.md)** | **最初に読む。** 現在地・確定事項・落とし穴・次の一手 |
| [REVIEW_2026-08-21.md](archive/REVIEW_2026-08-21.md) | 実機投入前の最終レビュー。Critical 1・High 6 を含む全指摘と根拠、対応状況、着手順 |
| [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) | 実機が来てからやること |
| **[archive/DESIGN_HISTORY.md](archive/DESIGN_HISTORY.md)** | **なぜこうなっているのか。** 廃案・訂正 15 件・方針変更の経緯。**同じ失敗を繰り返さないための記録** |

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
| 配線前に確認したい | [WIRING.md](WIRING.md) |
| StampFly に載せたい | [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md)、[PLAN.md §1](PLAN.md) |
| USB を外して無線でブラウザから見たい | [NET_DASHBOARD.md](NET_DASHBOARD.md) |
| Wi-Fi が繋がらない（`reason=201`/`202` 等） | [NET_DASHBOARD.md §3](NET_DASHBOARD.md) の起動ログ・`reason` の表 |
| なぜこの実装なのか知りたい（経緯） | [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md)、[archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) |
| 何が検証済みで何が未検証か | [HANDOFF.md](HANDOFF.md) §1、[GETTING_STARTED.md §11](GETTING_STARTED.md#limitations) |

---

## 全文書一覧

### 入門・リファレンス
| 文書 | 内容 |
|---|---|
| [UWB_PRIMER.md](UWB_PRIMER.md) | UWB の原理。なぜ電波で cm が測れるのか |
| [UWB_ALGORITHMS.md](UWB_ALGORITHMS.md) | 測位アルゴリズムの導出（上流 uwb_localizer からの移植・改訂版） |
| [GLOSSARY.md](GLOSSARY.md) | 用語集。**§9 に「紛らわしい語」**（ToF の二義など）。UUS / DTU / 実 µs の単位リファレンスも含む（**遅延値を触る前に必読**） |

### 実践
| 文書 | 内容 |
|---|---|
| [GETTING_STARTED.md](GETTING_STARTED.md) | BOM から測位までの完全手順（11 章）。Phase 1（SPI 疎通）の受入確認を含む |
| [EXPERIMENT_PLAN.md](EXPERIMENT_PLAN.md) | 実機到着後の実験計画とフラグ有効化の順序 |
| [PREBUILT_BINARIES.md](PREBUILT_BINARIES.md) | ビルド済みバイナリの入手と書き込み。ライセンス条件込み |
| [WIRING.md](WIRING.md) | 配線の正本。3つの接続経路とピン対応・向きの確定 |
| [ANCHOR_PLACEMENT.md](ANCHOR_PLACEMENT.md) | アンカー配置ルール（実測にもとづく） |
| [NET_DASHBOARD.md](NET_DASHBOARD.md) | Wi-Fi ブラウザダッシュボード・無線コンソール（`uwb_net`）の使い方。トポロジ・`wifi`/`net probe` コマンド・プロトコル・実測結果 |

### 設計・仕様
| 文書 | 内容 |
|---|---|
| [PLAN.md](PLAN.md) | 全体設計・フェーズ計画・**リポジトリの方針** |
| [PLAN_STAMPC5.md](PLAN_STAMPC5.md) | M5Stamp C5（ESP32-C5）をホストボードとして追加する計画。FPC 直結の配線案、シングルコア化などファーム側の変更点、実機検証の順番（2026-09-05、机上検討まで） |
| [IRQ_POLICY.md](IRQ_POLICY.md) | IRQ の使用方針（確定版） |
| [TIMING_PRESETS.md](TIMING_PRESETS.md) | 遅延プリセットの導出と版不一致検出 |
| [SURVEY_SPEC.md](SURVEY_SPEC.md) | アンカー座標の自動測量の仕様（外れ値 leave-one-out・キラリティ入力） |
| [STAMPFLY_INTEGRATION.md](STAMPFLY_INTEGRATION.md) | StampFly 位置制御への統合検討 |
| [PERF_ANALYSIS.md](PERF_ANALYSIS.md) | 測位ソルバの計算コスト分析と ESP32-S3 固有の最適化調査（浮動小数点・コンパイル設定） |
| [ARCHITECTURE_V2.md](ARCHITECTURE_V2.md) | UWB 測距ファームウェアの v2 アーキテクチャ設計。アンカー受信常時 ON のステートマシン化・DS-TWR 不安定の原因特定 |

### 引き継ぎ
| 文書 | 内容 |
|---|---|
| [HANDOFF.md](HANDOFF.md) | 次セッションへの申し送り。現在地・確定事項・誤りと教訓・文書の地図・次の一手 |

---

## アーカイブ（経緯）

上記はすべて**現役の文書**です。それとは別に、[`archive/`](archive/) に
**設計当時の調査・検討の記録**（経緯文書）を 11 本まとめてあります。
プロジェクトの背景や「なぜ今の実装になったか」を追いたいときに読んでください。
**現役文書と矛盾する場合は、常に現役文書が正しい。** 索引は [`archive/README.md`](archive/README.md)。

| 文書 | 内容 |
|---|---|
| **[archive/DESIGN_HISTORY.md](archive/DESIGN_HISTORY.md)** | **廃案・訂正・方針変更の経緯を集約。現役文書から外したものはここにある** |
| [archive/REVIEW_2026-08-21.md](archive/REVIEW_2026-08-21.md) | 実機投入前の最終レビュー。Critical 1・High 6、全件対応済み |
| [archive/REIMPL_PLAN.md](archive/REIMPL_PLAN.md) | 移植元の課題一覧 R1〜R12 とその決着 |
| [archive/CRITICAL_REVIEW.md](archive/CRITICAL_REVIEW.md) | 移植元コードの批判的レビュー |
| [archive/SURVEY_m5stamp_uwb_module.md](archive/SURVEY_m5stamp_uwb_module.md) | モジュールのハードウェア仕様の事前調査 |
| [archive/SURVEY_m5stamp_uwb_port.md](archive/SURVEY_m5stamp_uwb_port.md) | Arduino → ESP-IDF 移植の対応表 |
| [archive/SURVEY_stampfly_grove.md](archive/SURVEY_stampfly_grove.md) | StampFly の GROVE 端子と GPIO 空き状況の事前調査 |
| [archive/SURVEY_stampfly_ecosystem.md](archive/SURVEY_stampfly_ecosystem.md) | StampFly 側のソフト構成の事前調査 |
| [archive/SURVEY_uwb_localizer.md](archive/SURVEY_uwb_localizer.md) | 上流測位ライブラリの調査 |
| [archive/PROGRESS.md](archive/PROGRESS.md) | 開発進捗ログ（著者の作業記録） |
| [archive/MATH_AUDIT_2026-08-21.md](archive/MATH_AUDIT_2026-08-21.md) | 行列計算の残存箇所の監査とスカラー化の設計根拠（`components/uwb_math/` の仕様の元。指摘は全件対応済み） |

---

## この文書群の約束ごと

書き足すときは以下に従ってください。**過去に何度も踏んだ失敗から作られた規則です。**

1. **略語は各文書の初出で「正式名称（英語）＝日本語の意味」を添える**（[GLOSSARY.md](GLOSSARY.md) 冒頭）。
   文書をまたぐときは各文書で改めて展開する
2. **断定には出典を `ファイル:行` で添える**
   （一次資料は Qorvo の SDK / UM / APS。M5Stack のラッパは二次資料として扱う）
3. **フラグやメタデータより、直接観測できる事実を優先する。**
   矛盾する証拠が出たら辻褄を合わせず前提を疑う
4. **実機で確認していないことは「未検証」と明記する。** このリポジトリは
   タグ 1 + アンカー 1 の測距（2026-08-29〜31、SS-TWR 99.95% / DS-TWR
   99.5〜99.6%）に加え、2D 測位（アンカー3台、2026-09-02）・3D 測位（アンカー
   4台、2026-09-04〜05）まで実機確認済みだが、アンテナ遅延の校正や自動測量の
   無線側（アンカー間の自動測距）はまだ未検証。これを省くと読者を誤解させる。
   **逆に、実機で確認が取れた項目は「未検証」の記述を消して確定に書き換える**
5. **`.gitignore` されているディレクトリ（`third_party/` / `docs/refs/`）へ
   Markdown リンクを張らない。** clone した人には切れる
6. **現に正しい内容だけを書く。訂正の経緯は残さない。** 読む量を減らすため、
   最新の正しい結論だけを地の文に記す
7. **直訳語・業界スラング・擬人化を使わない。** 装置やソフトウェアが主語のときは、
   比喩ではなく実際の動作を書く（統計の「回帰」〈回帰分析・回帰直線〉と「期待値」は対象外）。
   下記は本文書群での言い換え辞書

   | 使わない語 | 言い換え |
   |---|---|
   | 回帰（ソフトウェアの regression） | 退行テスト・退行 |
   | 発火 | 作動する／割り込みが入る／条件が成立する |
   | 機会 | 場合・タイミング・時点 |
   | 露出（IO が） | 引き出されている／使える |
   | 律速 | ボトルネック／時間の大半を占める処理 |
   | 家事 | 補助処理・付随処理 |
   | 叩く | 呼び出す／直接操作する／送る |
   | 吐く | 出力する |
   | 食わせる | 入力する／渡す |
   | ラップする | 桁あふれで 0 に戻る／折り返す |
   | 走る・走らせる | 動く・実行する・動かす |
   | 殺す | 停止させる／終了させる |
   | 一発で | 一度で／一回で |
   | ゾンビ接続 | 残留接続／切断後も残った接続（初出時のみ説明を添えて使用可） |
   | 気づく | 検出する／検知する／分かる |
   | 諦める | 打ち切る／断念する／〜へ切り替える |
   | 期待する | 想定する／見込む／前提とする |
   | 知っている・知らない | 情報を持っている・持っていない／判別できる・できない |
   | 信じる | 重みを置く／信頼度を高く扱う／観測を重視する |
   | 死んでいる・死ぬ／生きている | 故障している・応答しない・停止している／動作している・応答している |
   | 黙る・黙っている | 出力が止まる／応答が無い |
   | 喋る | 送信する／通信する |
   | 眠る・目覚める・起こす | スリープする／待機する／復帰する／起動する |
   | 素直に | そのまま／無理なく／単純に |
   | 正直な | 実態に合った／妥当な |
   | 迷子（ファイル） | 残存した／取り残された |
   | 飢える・飢餓 | 処理が回ってこない状態（スタベーション、初出時のみ説明を添える）／処理が回ってこない |
   | 頑張る・嫌がる・喜ぶ・機嫌 | 状況に即して平易に言い換える |
   | 面倒を見る・お守り | 管理する／扱う |
   | 相手（装置を人のように） | 相手局／相手側／通信相手 |
   | 自分（装置を人のように） | 自局／自分自身の |
   | 〜してくれる・〜してしまう（装置が主語） | 〜する／〜になる |
