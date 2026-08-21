# 次セッションへの申し送り (2026-08-21 時点)

**このファイルを最初に読むこと。** 全体像・決定事項・落とし穴・次の一手をまとめてある。

---

## 0. 次セッションの任務

> **実機検証の前に [`docs/REVIEW_2026-08-21.md`](REVIEW_2026-08-21.md) §0 の表を読むこと。**
> #5（測量の同一平面検出 × アンテナ遅延）は同日夕方のリファクタで対応済み。**#6（外れ値リンク棄却）と #7（Release のライセンス同梱）は未対応。**
>
> **2026-08-21 夕方: 上流 `uwb_localizer` を凍結して独立し、線形代数を `components/uwb_math/`（スカラー展開）に一本化した。** `uwb_loc` / `uwb_survey` から一般 LU / Jacobi / 汎用コレスキーは消えている。設計根拠と実施状況は `docs/MATH_AUDIT_2026-08-21.md`。ホストテストは `make -C tests test strict float`（loc 77 + 新旧回帰 59 万 / math 3439 / pipeline 188 / survey 365）。float は clang だけでなく **gcc-16 と `-ffp-contract=off` でも**通すこと（FMA 縮約に助けられて clang だけ通る事故があった）。次は float 既定化（`CONFIG_UWB_LOC_USE_FLOAT`）の判断と実機 `soltest`。

> 前セッション（2026-08-21）で、**§5 の「すぐ着手できる」項目 A〜F はすべて完了した。**
> 残っているのは**実機を要する検証**と、下の 5 / 6 である。

### 2026-08-21 に完了した仕様と実装

| # | 仕様 | 実装状況 |
|---|---|---|
| **1** | `docs/IRQ_POLICY.md`（IRQ 方針） | **実装済み**。IRQ は起床信号としてのみ使い、判読はポーリングと共通。`pin_irq` 未配線なら自動フォールバック。**極性は実機未検証** |
| **2** | 同上（遅延値の扱い） | **実装済み**。`docs/TIMING_PRESETS.md` に3プリセット。版番号と種別をフレームに載せ、不一致を警告 |
| **3** | 役割の確定（タグ=StampFly） | **実装済み**。`boards/stampfly.h` 作成。GROVE 2系統4本=SPI、SPI3_HOST、`pin_irq` 既定 UNUSED。**HANDOFF が挙げていた「別配線候補 G6/G8/G11」は誤りだった**（ToF INT ×2 と BMI270 INT1 に配線済みでコネクタに出ていない） |
| **4** | アンカーのピン構成A/B | **実装済み**。`boards/atoms3.h` に `UWB_BOARD_ATOMS3_PINOUT` の Kconfig 切替 |

### 残っている仕様

| # | 仕様 | 実装状況 | やること |
|---|---|---|---|
| 5 | `docs/SURVEY_SPEC.md` の訂正 | 計算部分のみ実装 | `survey chirality` / 冗長度表示 は S3-S5（ESP-NOW）と同時 |
| 6 | `docs/STAMPFLY_INTEGRATION.md` 案B-2 | 未着手 | 実機で測位が出てから |

**次にやるべきことは実機での検証である。** 手順と順序は
**[`docs/EXPERIMENT_PLAN.md`](EXPERIMENT_PLAN.md)** にまとめてある
（何をどの順で確かめ、どのフラグをいつ有効にするか、実機でしか潰せない前提10件）。

---

## 1. 現在地

**実機未着。すべてビルド通過とホスト検証まで。実機で Device ID すら読めていない。**
製品自体が新しく、コミュニティの実績も無い（ユーザ談）。

| リポジトリ | 場所 | 状態 |
|---|---|---|
| **m5stamp_uwb_localizer**（本体） | `/Users/kouhei/tmp/github/m5stamp_uwb_localizer` | **GitHub 公開済み** `kouhei1970/m5stamp_uwb_localizer` (public)。全コミット push 済み |
| **uwb_localizer**（上流） | `/Users/kouhei/tmp/github/uwb_localizer` | **解決済 (2026-08-21)**: `perf/exploit-structure` を `main` にマージして凍結。最終状態（`ab23b33`）を `components/uwb_loc/` に取り込み済み。以後 `components/uwb_loc/` は本リポジトリで独立して開発する（上流は見ない） |
| stampfly_ecosystem | `third_party/stampfly_ecosystem` | 2026-08-19 の読み取り専用クローン。**書き込み禁止** |
| M5Stamp-UWB / uwb_localizer 参照用 | `third_party/` | 読み取り専用 |
| 一次資料（PDF/公式API） | `docs/refs/` | **`.gitignore` 済み**（再配布禁止文書を含む） |

### ビルド・テストの現況（すべて通る）
```
firmware/{probe,devtest,twr,tag,anchor,soltest}   全て警告0・エラー0
tests/host/pipeline    188件    tests/host/survey  281件    tests/host/loc  77件
```
**GitHub Actions でも同じものが回っている**（`.github/workflows/build.yml`）。
ホストテスト3種 + strict、ファーム14通り、タグ `v*` で Release 添付。
**Release `v0.1.0` 公開済み**（14個の zip。`docs/PREBUILT_BINARIES.md`）。

### 最終セッション（2026-08-21 午後）の終了時点での宿題
| # | 内容 | 誰が |
|---|---|---|
| 1 | **GitHub の Social preview が壊れている。** 3回アップロードしたが画像本体が配信されない（`og:image` の URL が 404）。DevTools で送信は 204 成功、読み戻しだけ失敗 → **GitHub 側の不具合**。クライアント側（拡張/ブラウザ/ネットワーク）は調査済みで無実 | ユーザ。**数日後に再試行。それまで Settings で Remove image しておく**（壊れた画像より自動生成カードの方がまし） |
| 2 | `assets/social_card.{png,jpg}` は 1280x640 で作り直し済み・コミット済み | — |
| 3 | 上流 `uwb_localizer` へ報告: `uwb_nls.c:342` の `wsum` が GCC `-Werror=unused-but-set-variable` で落ちる（clang は通す） | **解決済 (2026-08-21)**: 上流ブランチ側で `wsum` は削除済み。マージ・取り込み後は本リポジトリ側でも再発しない |
| 4 | 上流 `perf/exploit-structure` をマージするか | **解決済 (2026-08-21)**: マージ・凍結し、最終状態を本リポジトリへ取り込んだ。以後独立開発 |
| 5 | エディタの clangd が旧ディレクトリ名のパスを見ている | どれか1つで `idf.py fullclean && idf.py build` |
| 6 | **最終レビュー**（ユーザが「最終レビューの前に一度セッションを閉じる」と言って終了） | **完了 (2026-08-21)**。結果は `docs/REVIEW_2026-08-21.md`。§0 の表 #1〜#4（DS-TWR の PRETOC、SPI 16MHz 切替、タグのメインタスクスタック、decamutex）は同日に修正済み。残りの指摘は同報告書 §7 の着手順に従う |

---

## 2. 何を作ったか

```
components/
  qm33120w_sdk/   Qorvo ドライバ (vendoring, SPDX 保持)     ← 一次資料。信頼してよい
  uwb_port/       dwt_spi_s / deca_sleep / mutex / GPIO     ← 自作
  uwb_qm33120/    デバイス層 + SS/DS-TWR                     ← M5Stack 由来を大幅に修正
  uwb_loc/        測位ソルバ Lv0-Lv3 (uwb_localizer 凍結時点を取り込み)  ← 以後は本リポジトリで独立開発
  uwb_ranging/    アンカーテーブル + スケジューラ + パイプライン
  uwb_cfgstore/   NVS 永続化 + シリアルコンソール
  uwb_survey/     MDS + Gauss-Newton + ゲージ固定（自動測量の計算部分）
firmware/         probe / devtest / twr / soltest / tag / anchor
tests/host/       loc (uwb_loc のホスト検証)
tools/            test_pipeline / test_survey / bench_loc
```

**ハード依存は `uwb_port` と `uwb_ranging_scheduler` の2箇所だけ**に隔離。
測位パイプラインと測量計算はホストで検証できる。

---

## 3. 確定した仕様・決定事項

| # | 決定 | 文書 |
|---|---|---|
| 対象 | **ESP32-S3 + M5Stamp UWB Module 専用**。プラットフォーム最適化してよい。ただし StampFly には非依存 | `docs/PLAN.md` |
| **ハード方針** | **StampFly 非依存。ただしタグの配線だけは StampFly 互換を維持する**（GROVE 2系統4本で成立 ＝ IRQ/RST 不要）。想定利用者は本リポジトリを単体で試す人 | `docs/PLAN.md` §1 |
| 役割 | **タグ = M5StampS3A ×1 / アンカー = AtomS3(R) ×5** | `docs/archive/PROGRESS.md` |
| 接続 | **FPC ではなく半田パッド**（1.27mm キャステレーション） | `docs/SOLDER_PADS.md` |
| **IRQ** | **アンカーは積極使用。タグは不使用。StampFly の別配線可能性は残す** | **`docs/IRQ_POLICY.md`** |
| 資料 | **一次資料 = Qorvo SDK/UM/APS。M5Stack ラッパは二次資料で信頼しない** | **`docs/SOURCE_POLICY.md`** |
| 測量 | 高さのみ実測(4点以上) + キラリティ1ビット。タグも6台目として参加。計算は実機上 | `docs/SURVEY_SPEC.md` |
| StampFly統合 | **案B-2 疎結合**: Lv2 で位置を出し `EskfCore::vectorUpdate3()` で POS_X/Y 観測 | `docs/STAMPFLY_INTEGRATION.md` |

### 文書の地図
```
README.md                    購入者の入口
docs/GETTING_STARTED.md      BOM から測位まで11章
docs/HANDOFF.md              ← このファイル
docs/GLOSSARY.md             用語集（略語の正式名称と意味）。分からない略語はここ
docs/UWB_PRIMER.md           UWB 入門。なぜ電波で cm が測れるのか（最初に読む）
docs/UWB_ALGORITHMS.md       測位アルゴリズムの導出（上流からの移植・改訂版）
docs/EXPERIMENT_PLAN.md      実機到着後の実験計画とフラグ有効化の順序
docs/SOURCE_POLICY.md        資料の格付けと、過去の誤りの記録
docs/IRQ_POLICY.md           IRQ 方針（確定版）
docs/UNITS.md                UUS/DTU/実µs の単位リファレンス（遅延値を触る前に必読）
docs/TIMING_PRESETS.md       遅延プリセットとバージョン不一致検出（設計）
docs/archive/CRITICAL_REVIEW.md      M5Stack ラッパの批判的レビュー
docs/archive/REIMPL_PLAN.md          R1-R12。R1/R2/R3-1/R4/R7/R8/R9 は実装済み
docs/SURVEY_SPEC.md          自動測量の仕様（訂正4件入り）
docs/STAMPFLY_INTEGRATION.md StampFly 位置制御への統合（1127行）
docs/PERF_ANALYSIS.md        測位計算の性能分析と上流最適化の結果
docs/PLATFORM_TUNING.md      ESP32-S3 の浮動小数点・コンパイル設定
docs/ANCHOR_PLACEMENT.md     アンカー配置ルール
docs/SOLDER_PADS.md          パッド仕様と配線
docs/BRINGUP.md              Phase 1 受入確認
docs/refs/README.md          一次資料の索引
```

---

## 4. 【最重要】私が犯した誤りと、そこから得た教訓

**同じ失敗を繰り返さないこと。パターンは1つに集約される:
「二次的な指標を、直接証拠より優先した」「確認せずに断定した」。**

| # | 誤り | 正解 | 原因 |
|---|---|---|---|
| 1 | フレームフィルタ未設定は欠陥 | **Qorvo 公式 TWR 4例も使っていない** | 公式サンプルを見ずに断定 |
| 2 | アンテナ遅延 16385 は EVB からのコピー | **APS014 が定める正規の初期値 (~513ns)** | 同上 |
| 3 | `PGcount=0` は欠陥 | **Qorvo 公式も 0** | 同上 |
| 4 | 操作対象はこの Mac の Chrome | **リモート接続元の別マシンだった** | API の `isLocal:true` を検証せず信用 |
| 5 | 「同期された履歴が見えているだけ」 | 実際に別マシンへダウンロードされていた | **矛盾する直接証拠（ファイルが無い）を握りながら辻褄合わせ** |
| 6 | StampFly の空きGPIO は G5/G10/G41/G42 | **4本ともモータPWM**（`vehicle/main/config.hpp:63-66`） | 古い M5StampFly を見て現行 vehicle を見なかった |
| 7 | 高さ実測で鏡像が決まる | **原理的に不可能**（線形汎関数はキラリティを検出できない） | 数学を検証せず仕様に書いた |
| 8 | `uwb_sym_eig()` が使える | **未マージ上流にしか無い** | 自分が書いた別ブランチの成果と混同 |
| 9 | 高さ3点で傾きが決まる | **4点以上必要** | 同上 |
| 10 | 単位を `Uus`→`Us` にリネームすべき | **M5Stack の単位系は正しかった。誤称は Qorvo 側** | SDK の記述を読む前に計画を書いた |
| 11 | 公式ピンマップと文書が8箇所違う | **パッドと FPC で並びが違うだけ。両方正しい** | 番号付きの帯をパッド番号と決めつけた |

### 運用ルール（`docs/SOURCE_POLICY.md` に詳細）
1. **フラグ・メタデータより直接観測できる事実を優先する**
2. **矛盾する証拠が出たら辻褄を合わせず前提を疑う**
3. **`grep` が0件なら、まずファイルが読めているか疑う**
   （**Qorvo 公式サンプルの `.c` は非UTF-8。`grep -a` か `iconv` 必須**。この罠で誤結論を出しかけた）
4. **「SDK に API があるのに使っていない」は、それだけでは欠陥の根拠にならない**
5. **入手資料は独立経路で再取得してハッシュ照合する**（実施済み、主要6件一致）

### ブラウザ自動化は使わない
セッション中、`mcp__claude-in-chrome__*` がユーザの意図しない**リモート接続元マシンの
Chrome** を操作する事故を起こした。**本プロジェクトでは以後一切使わない。**
Web 取得は `WebFetch` / `curl` に限定し、サブエージェントにも明示的に禁止すること。

---

## 5. 次にやること

### すぐ着手できる（実機不要）
| # | 内容 | 備考 |
|---|---|---|
| ~~**A**~~ | ~~**`boards/stampfly.h` の作成**~~ | **完了 (2026-08-21)**。GROVE 4本(G13/G15/G1/G2)=SPI、`pin_irq` は既定 UNUSED。**当初挙げていた「別配線候補 G6/G8/G11」は誤りだったので採用していない**（§0 参照）。代わりに現実的な経路（HW-2）をコメントに書いた |
| ~~B~~ | ~~`boards/atoms3.h` に構成A/B の Kconfig 切替~~ | **完了 (2026-08-21)**。ToF を Grove に挿すか底面に手配線するか |
| ~~C~~ | ~~遅延プリセット化 + バージョン不一致検出~~ | **完了 (2026-08-21)**。タグとアンカーで遅延値が食い違うと測距が壊れる。実運用で必ず踏む |
| ~~D~~ | ~~`firmware/twr` の `spi_fast_hz` / アンカーアドレスを Kconfig 化~~ | **完了 (2026-08-21)**。`UWB_TWR_TAG_ADDR`/`UWB_TWR_ANCHOR_ADDR`（既定は元の固定値のまま）と `UWB_SPI_FAST_HZ`（既定0=`boards/*.h`のまま。`firmware/probe`にも同じ形で追加）。切り分けの最頻出項目 |
| ~~E~~ | ~~`RangingScheduler::stats()` を tag の JSON に接続~~ | **完了 (2026-08-21)**。毎周期の`fix`行とは別に`"type":"stats"`行を`UWB_TAG_STATS_INTERVAL_MS`（既定1000ms、0で無出力）ごとに出す。「どのアンカーが悪いか」は最初に知りたい情報 |
| ~~F~~ | ~~`RangingSample` に絶対タイムスタンプ追加~~ | **完了 (2026-08-21)**。`t_us`（測距開始直前に`esp_timer_get_time()`で記録）を追加し、`fix`行`anchors[]`各要素にも`t`（`fix`の`t`と同じ基準の相対秒）として出力。周内スミア対策（1周32ms → ±16ms、1m/s で 16mm） |

### 実機が要る
| # | 内容 |
|---|---|
| **Phase 1 受入** | `firmware/probe` で Device ID `0xDECA0314`。**ピン配線が未検証なのでここが最初の関門** |
| R5 | 遅延値の追い込み（公式手順: `dwt_starttx()` が `DWT_ERROR` を返すまで下げる） |
| R6 | **アンカー側の** IRQ 駆動化 — **コードは実装済み**（`docs/IRQ_POLICY.md`「実装状況」節）。残るのは実機での極性検証（`gpio_config()`のアクティブHIGH前提が正しいか）だけ |
| R10-R12 | ch9 の PLL 再校正(20°Cごと)、アンテナ遅延校正、診断情報→測位重み |
| S3-S5 | ESP-NOW、測量モード、コーディネータ |
| S7 | I2C ToF（高さ自動計測） |

### ユーザ判断待ち
- ~~**`uwb_localizer` の `perf/exploit-structure` をマージするか**~~
  → **解決済 (2026-08-21)**: 上流をマージ・凍結し、最終状態（`ab23b33`）を
  `components/uwb_loc/` へ取り込んだ。測位計算が3〜5倍速い最適化版に更新済み。
  以後 `components/uwb_loc/` は本リポジトリで独立して開発する（上流は見ない）
- ~~**上流 `uwb_loc` が GCC の `-Werror=unused-but-set-variable` で落ちる**~~
  → **解決済 (2026-08-21)**: マージ元ブランチで `wsum`（`uwb_nls.c:342`）は
  削除済み。取り込み後の `components/uwb_loc/` にも `wsum` は存在せず、
  CI の `strict` 除外は解除した（`tests/host/loc` の `test`/`strict`/`float`
  をすべて実行している）
- 上流の pytest 1件失敗（`test_self_survey_with_noise_and_missing_links[1]`）
  → テストの基準アンカーが同一平面。テスト側の問題
- ~~デフォルトブランチが `master`。`main` に変えるか~~
  → **解決済 (2026-08-21): `main` に統一した。** リモートの `master` は削除済み。
  既存クローンがある場合は `git branch -m master main && git fetch origin &&
  git branch -u origin/main main && git remote set-head origin -a` を実行すること

---

## 6. 落とし穴（踏むと時間を失う）

1. **Qorvo 公式サンプルの `.c` は latin-1。`grep -a` を使うこと**
2. **`UUS_TO_DWT_TIME` は DW1000=65536 / DW3000公式=63898。**
   Qorvo の `*_UUS` 定数は実は実マイクロ秒。そのまま流用すると2.5%ずれる
   → **`docs/UNITS.md`**
3. **ESP-IDF ビルドとホスト `make strict` は警告設定が違う。**
   `uwb_survey.c` が `-Werror=maybe-uninitialized` で落ちた実例あり。
   **新規コンポーネントは必ず `idf.py build` も通すこと**
4. **シェルの cwd が持続する。** `cd firmware/anchor` した後に `cat >> PROGRESS.md` して
   迷子ファイルを作った実例あり。**絶対パスを使うこと**
5. **`sdkconfig` は `.gitignore` 済み。** `sed` で書き換える手順を書いてはいけない。
   `-D SDKCONFIG=build/xxx/sdkconfig` 方式を使う（`GETTING_STARTED.md` に検証済みの手順あり）
6. ~~`components/uwb_loc/` は上流と byte 一致を保っている。直接編集しないこと~~
   → **2026-08-21 に方針転換**: 上流 `uwb_localizer` を凍結・最終取り込みした後は、
   `components/uwb_loc/` は本リポジトリで独立して開発する。ESP32-S3 向けの
   最適化（float 化・スカラー展開など）をソースに直接入れてよい
7. サブエージェントに Web 調査を任せるときは**取得手段を明示的に限定する**

---

## 7. 数字の要約（実機で最初に検証すべきもの）

| 項目 | 値 | 出典 |
|---|---|---|
| 測位レート（現状・5アンカー） | **31.3 Hz**（1周 31.9ms） | `STAMPFLY_INTEGRATION.md` |
| 同（アンカーのみ IRQ） | **59.4 Hz** | 同上 |
| StampFly 位置制御の実効帯域 | **約 0.064 Hz**（`pos.kp=0.4`） | `params.cpp:771` |
| アンカー座標の推定誤差（σ=5cm時） | RMS 5cm / 最悪 25cm | `test_survey` |
| 共通アンテナ遅延の推定誤差 | 平均 1.5cm | 同上 |
| **無校正のアンテナ遅延バイアス** | **Δ1ns = 30cm** | APS014 |
| ESP32-S3 の float | 加減乗と `madd.s` はハード、**除算と sqrt はソフト** | 逆アセンブルで確認 |

**StampFly の ESKF には POS_X/POS_Y を直接観測する update 関数が1つも無く、
水平位置は速度の積分だけ（実質デッドレコニング）。UWB が埋めるのはそこ。**
