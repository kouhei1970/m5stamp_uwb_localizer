# m5stamp_uwb_localizer 開発計画 (2026-08-19 策定、2026-08-21 進捗反映)

## 0. Phase 0 調査の結論

> **本文で使う主な略語**: **UWB**（Ultra-Wideband、超広帯域無線）／**TWR**（Two-Way Ranging、双方向測距）／
> **SS-TWR**（Single-Sided TWR、片側二方向測距）／**DS-TWR**（Double-Sided TWR、両側二方向測距）／
> **I2C**（Inter-Integrated Circuit）／**SPI**（Serial Peripheral Interface）／
> **GPIO**（General Purpose Input/Output、汎用入出力）／**IRQ**（Interrupt ReQuest、割り込み要求）。
> 全一覧は `docs/GLOSSARY.md`。

### 実現可能性の判定
| 論点 | 結論 |
|---|---|
| M5Stamp UWB Module（通称 M5Stamp UWB）を SPI で使えるか | **使える。というより SPI しか無い**（Qorvo QM33120W 直結、AT コマンド無し） |
| StampFly の GROVE 1系統で SPI を張れるか | **不可**（信号2本 vs SPI 最低4本） |
| GROVE 2系統併用なら | **可能**（G13,G15,G1,G2 で SCK/MOSI/MISO/CS）。ただし IRQ 無し・カスタム配線・電源要降圧 |
| uwb_localizer をそのまま使えるか | **測位層(c/)はほぼ無改造で使える。レンジング層は存在しないので新規** |

### 想定外だった重要事実（計画の前提が変わった点）
1. **uwb_localizer にはチップドライバも測距シーケンスも無い。**
   対応済み実機は REYAX RYUW122（UART+AT）のみで、その HAL は Python にしかない。
   → M5Stamp UWB Module 用のレンジング層は**まるごと新規**。ただしゼロからではなく
     **M5Stack 公式 Arduino ライブラリ (m5stack/M5Stamp-UWB, MIT + Qorvo SDK) を
     ESP-IDF へ移植**するのが現実解。
2. **M5Stamp UWB Module のコネクタは GROVE ではなく 0.5mm 12P FPC（Flexible Printed Circuit、フレキシブル基板）。電源は 3.3V 単一（チップ直結、絶対最大 4.0V）。**
   GROVE（M5Stack 一般の GROVE は 5V 出力のものが多いが、**StampFly の GROVE は
   電池電圧そのもの＝満充電で ~4.35V あり、これもチップの絶対最大 4.0V を超える**）
   から直接は挿さらない。**FPC→配線の変換と、3.3V への降圧（LDO 必須）が必須。**
3. StampFly の GROVE は**2系統ある**（赤=I2C G13/G15、黒=UART G1/G2）。
   どちらも内蔵バスと非共有・ファームウェア未使用。

---

## 1. 成果物の定義

**本リポジトリは StampFly から独立した「UWB（Ultra-Wideband、超広帯域無線）測位一式」として完結させる。**
StampFly への統合はその成果物を利用する下流作業。

### 方針: StampFly 非依存。ただし**タグのハードだけは StampFly 互換**に保つ（2026-08-21 ユーザ指示）

この2つは矛盾しない。**独立**なのはソフトウェアと想定利用者、**互換**にするのはタグの配線である。

| | 方針 |
|---|---|
| **想定利用者** | **このリポジトリを手元の ESP32-S3 + M5Stamp UWB Module で実際に試す人**。ドキュメントも実装もこの人に向けて書く。**StampFly を持っていることは前提にしない** |
| **タグのハード** | **StampFly 互換を維持する**（意図的な制約）。本スタックの成果をそのまま StampFly の位置制御へ移植したいため（`docs/STAMPFLY_INTEGRATION.md` 案B-2） |
| **アンカーのハード** | **この制約は無い**。据置きなので AtomS3(R) の IRQ を積極利用してよい（`docs/IRQ_POLICY.md`） |

「タグのハードを StampFly 互換に保つ」の具体的な中身:

1. **タグは GROVE 2系統4本 (G13/G15/G1/G2) だけで動くこと。** StampFly で外部に出せる
   信号線はこれだけなので、これを超える配線を前提にした実装をタグ側に入れない
   （`boards/stampfly.h`、`docs/STAMPFLY_INTEGRATION.md` §5.2）。
2. **したがって IRQ（Interrupt ReQuest、割り込み要求）/ RST を必要としないポーリング経路を、常に第一級の実装として保つ**
   （`docs/IRQ_POLICY.md`）。タグ単体構成の M5StampS3A では IRQ (G7) が取れるが、
   **それを前提にした実装にはしない**。IRQ はあくまで「使えるなら使う」加点要素。
3. **SPI（Serial Peripheral Interface）ホストは飛行系(SPI2_HOST)と分離できること**（`boards/stampfly.h` は SPI3_HOST）。

**この制約があるからタグは 31 Hz 止まりで、アンカーだけ IRQ 化しても 59 Hz が上限になる**
（`docs/TIMING_PRESETS.md`）。それでも StampFly の位置制御の実効帯域は約 0.064 Hz なので
実用上の問題は無い、というのが `docs/STAMPFLY_INTEGRATION.md` §3.1 の結論である。

### 提供するもの
| # | 成果物 | 内容 | 状態（2026-08-21） |
|---|---|---|---|
| D1 | ESP-IDF コンポーネント群 | 他プロジェクトへ丸ごと持ち込める。StampFly 依存ゼロ | **実装済み**（`components/` 8個） |
| D2 | タグ側ファームウェア | M5StampS3A / AtomS3 + M5Stamp UWB Module で測距→測位まで完結 | **実装済み**（`firmware/tag`、CI ビルド済み、実機未検証） |
| D3 | アンカー側ファームウェア | 同上ハードでレスポンダ動作。アドレス/座標設定可 | **実装済み**（`firmware/anchor`、同上） |
| D4 | ホスト側ツール | 測位結果の可視化・ログ・アンテナ遅延キャリブレーション | **一部**（`tools/bench_loc` のみ。JSON Lines は上流 Python 可視化と互換。校正ツールは未） |
| D5 | ドキュメント | 配線図、ボード別ピン定義、立ち上げ手順、キャリブレーション手順 | **実装済み**（`docs/` 22本。`WIRING.md` / `GETTING_STARTED.md` / `GETTING_STARTED.md` / `EXPERIMENT_PLAN.md`） |
| D6 | StampFly 統合 | `sf_hal_uwb_qm33120` として stampfly_ecosystem へ | **未着手**（設計のみ `STAMPFLY_INTEGRATION.md`） |

### 運用構成（2026-08-19 確定）
**アンカー数は固定しない。4台以上の任意台数に対応する。**
手持ちは M5Stamp UWB Module ×6 なので、当面の検証構成は**タグ1 + アンカー5**。

#### 台数に関する設計要件
- **アンカー台数はコンパイル時に決め打ちしない**。アンカー登録テーブル
  （ショートアドレス + 3D座標 + 有効フラグ）を持ち、**台数はテーブル長で決まる**
  → **実装済み**（`components/uwb_ranging/include/uwb_ranging_anchor_table.hpp`）
- 必要台数の下限:
  - **3D 測位: 4台**（未知数 x,y,z の3 + 幾何的な曖昧性の解消）
  - 2D 測位（高さを既知として固定）: 3台
  - `uwb_loc` には `uwb_anchors_coplanar()` があり、**同一平面配置だと3Dが縮退する**ことを
    検出できる。天井/床にアンカーを平面配置すると鏡像解が出るので、この判定を必ず使う
    → **実装済み**（`uwb_ranging_anchor_table.cpp:90` で `coplanar` フラグを返し、呼び出し側が警告）
- 上限は `uwb_loc` の `UWB_MAX_ANCHORS`（既定16）。RAM 節約のため8へ下げる余地あり
  → **未実施**（既定16のまま）
- **1周で全アンカーから応答が返るとは限らない。有効な測距が4件以上あれば解く**という
  作りにする（欠測・タイムアウトを常態として扱う）。有効数が足りない周期は解を出さず
  「測位不能」を返し、EKF（Extended Kalman Filter、拡張カルマンフィルタ）(Lv3)側で予測のみ進める
  → **実装済み**（`uwb_ranging` パイプライン。ホストテスト pipeline に「5台中2台欠測」「1台欠測」のケースあり）
- 台数が多いほど:
  - 冗長性が上がり、**外れ値を棄却しても解が残る**（Lv2 の Huber/χ²ゲートが効かせられる。
    4台ちょうどだと1台弾いた時点で解けなくなるため、実質ゲートを効かせられない）
  - GDOP（Geometric Dilution of Precision、幾何学的精度低下率）が改善し、アンカー配置の自由度も上がる
  - **1周の所要時間が伸びる**（→ 更新レートとのトレードオフ。R6）
- **更新レートの制約が最大の設計課題になる**: DS-TWR（Double-Sided TWR、両側二方向測距）は1リンクあたり
  Poll→Resp→Final→(結果返送) の往復が必要で、それを5アンカー分**逐次**回す。
  原本サンプルの `RANGE_INTERVAL_MS = 200` をそのまま使うと1周1秒＝1Hz にしかならない。
  飛行制御に使うには全く足りないので、**1リンクあたりの所要時間の実測**と
  インターバル短縮が Phase 4-5 の主要課題（→ R6）
  → **試算 31.3 Hz**（5アンカー DS-TWR）、**アンカー IRQ 化で 59.4 Hz** 試算
  （`docs/TIMING_PRESETS.md`, `docs/STAMPFLY_INTEGRATION.md`）。実測は実機待ち
- ~~ホストボードが6台あるかは未確認。不足する場合はアンカー台数を減らして段階的に検証する~~
  **確定構成（`docs/archive/PROGRESS.md`）: タグ = M5StampS3A ×1、アンカー = AtomS3(R) ×5**

### 対応ホストボード
- **M5StampS3A**（ESP32-S3、GPIO 露出多い＝本命の開発ボード）
- **M5 AtomS3**（ESP32-S3、GROVE 1系統 + 内部で LCD/ボタンが GPIO 消費）
- （下流）StampFly = M5StampS3A

---

## 2. アーキテクチャ

```
+---------------------------------------------------------------+
|  firmware/tag  |  firmware/anchor     (アプリ層・ボード非依存)  |
+---------------------------------------------------------------+
|  uwb_localizer : 測距スケジューラ + 測位パイプライン           |  ← 新規
+------------------------------+--------------------------------+
|  uwb_twr : SS/DS-TWR 状態機械 |  uwb_loc : 測位ソルバ Lv0-Lv3  |
|            ノード/アドレス管理 |            (uwb_localizer c/) |  ← vendoring
+------------------------------+--------------------------------+
|  uwb_qm33120 : チップドライバ（レジスタ/PHY/フレーム/TS）      |  ← M5Stamp-UWB 移植
+---------------------------------------------------------------+
|  uwb_port : プラットフォーム抽象 (SPI / GPIO / 時刻 / ログ)     |  ← 新規
|             uwb_port_espidf.c  |  uwb_port_arduino.c (任意)    |
+---------------------------------------------------------------+
|  boards/ : stamps3.h  atoms3.h  stampfly.h  (ピン定義のみ)      |
+---------------------------------------------------------------+
```

**実装後の実名対応（2026-08-21）**: `uwb_twr` → `uwb_qm33120` に統合。
app 層 `uwb_localizer` → 実名 `uwb_ranging`。新設: `uwb_cfgstore`（NVS 永続化 + コンソール）、
`uwb_math`（線形代数）、`uwb_survey`（アンカー自動測量、計画外）。

### スコープ（2026-08-20 ユーザ指示で確定）
**本リポジトリは ESP32-S3 + M5Stamp UWB Module 専用。プラットフォーム最適化を積極的に行ってよい。**

| 対象 | 方針 |
|---|---|
| `m5stamp_uwb_localizer`（本リポジトリ） | **ESP32-S3 専用に最適化してよい**。-O2 / 240MHz / float / IRAM / esp-dsp など |
| `uwb_localizer`（上流） | ~~移植性を維持。他プロジェクトでも使うライブラリなので ESP32 依存を持ち込まない~~ → **2026-08-21 に上流を凍結し、最終状態（`ab23b33`）を `components/uwb_loc/` に取り込んだ。以後 `components/uwb_loc/` は本リポジトリで独立して開発する（上流はもう追わない）** |
| StampFly | 引き続き**非依存**。StampFly はあくまで本成果物の利用者の一つ |

「StampFly 非依存」と「プラットフォーム非依存」は別物である点に注意。
前者は維持し、後者は放棄する。

#### 適用済み
- 全ファームの `sdkconfig.defaults` に
  `CONFIG_COMPILER_OPTIMIZATION_PERF=y`（-O2。既定は -Og だった）と
  `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`（既定は 160MHz）
- 結果: バイナリはむしろ縮んだ（tag: 0x4a6c0 → 0x48430、-8.8KB）

#### 評価予定（実機測定後に判断）
- `UWB_USE_FLOAT`（`madd.s` が効くので期待大。ただし精度検証が先）
  → **2026-08-21 追記**: ホストでは float ビルド全通過（clang / gcc-16 / `-ffp-contract=off`、
  `CONFIG_UWB_LOC_USE_FLOAT`）。既定化は実機 `soltest` 待ち
- ホット関数の IRAM 配置
- esp-dsp の適用（ただし最適化後のホットパスは行列積ではないため期待値は低い）
- 詳細は `docs/PERF_ANALYSIS.md`

### 設計原則
- **言語方針（2026-08-19 改訂）**: 当初「コアは C99」としたが、調査5で
  M5Stamp-UWB の wrapper が C++ (PImpl) 1,946行だと判明したため方針変更。
  機械的に C へ書き換えるのは純粋な追加工数で見返りが無い。
  | 層 | 言語 | 理由 |
  |---|---|---|
  | `qm33120w_sdk` | C（原本のまま） | Arduino依存ゼロ。無改造コピー |
  | `uwb_port` | C | Qorvo SDK の関数ポインタ/`extern "C"` に直接バインドするため |
  | `uwb_qm33120` / `uwb_twr` | **C++** | M5Stamp-UWB 由来。構造を保って移植＝最小工数。**`uwb_twr` は `uwb_qm33120` に統合された（2026-08-19）** |
  | `uwb_loc` | C99 | **2026-08-21 に上流 `uwb_localizer` を凍結し、最終状態（`ab23b33`）を取り込んだ。以後 `components/uwb_loc/` は本リポジトリで独立して開発する。** ESP32-S3 向けの最適化（float 化・スカラー展開など）をソースに直接入れてよい |
  | `uwb_localizer`(app層) | C++ | 上位ロジック。**実名 `uwb_ranging`** |
  移植性は「ハード依存が `uwb_port` 1枚に閉じている」ことで担保され、言語では担保しない。
  stampfly_ecosystem は元々 C++ なので統合も素直（`namespace stampfly` ラッパを被せる）
- **ハード依存は uwb_port の関数テーブル1枚に隔離**。ESP-IDF 以外へも移植可能に
- **ボード差はヘッダのピン定義だけ**。コードは共通
- **SPI バスは二重初期化耐性**を持たせる（stampfly_ecosystem の既存流儀。
  単独利用時は自分で `spi_bus_initialize`、`ESP_ERR_INVALID_STATE` は許容してスキップ）
- **IRQ はオプショナル**。ポーリング動作でも成立させる
  （StampFly の GROVE 2系統案では IRQ 線が取れないため。TWR の時間критиカルな部分は
    チップ内の遅延送信で処理されるので、ホストのポーリング遅延は主に更新レートに効き、
    測距精度には直結しない）

### ディレクトリ構成（実績、2026-08-21）
```
m5stamp_uwb_localizer/
├── docs/                 現役22本: WIRING.md / GETTING_STARTED.md / GETTING_STARTED.md /
│                         EXPERIMENT_PLAN.md / PLAN.md / TIMING_PRESETS.md /
│                         STAMPFLY_INTEGRATION.md / SURVEY_SPEC.md / GLOSSARY.md ...
│   └── archive/          PROGRESS.md、SURVEY_*.md ほか旧文書
├── components/           (8個)
│   ├── qm33120w_sdk/     Qorvo SDK 原本
│   ├── uwb_port/         プラットフォーム抽象 (SPI/GPIO/時刻/ログ)
│   ├── uwb_qm33120/      チップドライバ + SS/DS-TWR（旧 uwb_twr を統合）
│   ├── uwb_math/         対称3x3/2x2 閉形式・LDLᵀ・ブロックコレスキー（新設）
│   ├── uwb_loc/          測位ソルバ Lv0-Lv3
│   ├── uwb_ranging/      アンカーテーブル + スケジューラ + 測位パイプライン
│   │                     （計画の app 層 `uwb_localizer` に相当）
│   ├── uwb_cfgstore/     NVS 永続化 + シリアルコンソール
│   └── uwb_survey/       アンカー自動測量（計画外で追加。docs/SURVEY_SPEC.md）
├── firmware/             (6種、全部 CI でビルド済み・警告0。実機は全て未検証)
│   ├── probe/            Device ID 読み出し
│   ├── devtest/
│   ├── twr/              2台の SS/DS-TWR
│   ├── soltest/          測位ソルバの実機ベンチ
│   ├── tag/
│   └── anchor/
├── tests/host/           ホストテスト4スイート (loc / math / pipeline / survey、全通過)
├── tools/
│   └── bench_loc/        測位ソルバのマイクロベンチ（可視化・校正ツールは無し）
├── boards/               stamps3.h  atoms3.h  stampfly.h
└── third_party/          M5Stamp-UWB / uwb_localizer / stampfly_ecosystem（読み取り専用）
                          ※ Qorvo SDK は components/qm33120w_sdk/ に vendoring
```

---

## 3. 配線設計

**【2026-08-21 更新】** 接続は FPC ではなく半田パッドで確定（`docs/WIRING.md`）。
以下の 12P 信号一覧は有効。

### モジュール側（12P FPC、固定）
1:GND 2:VCC_3V3 3:DW_WAKEUP 4:DW_IRQ 5:DW_GP7 6:DW_RSTn
7:DW_CDO(MISO) 8:GND 9:DW_CDI(MOSI) 10:DW_CSn 11:DW_CLK 12:GND

**必須: VCC_3V3, GND, CLK, CDI, CDO, CSn（6本）**
**推奨: +IRQ, +RSTn（8本）／省電力運用なら +WAKEUP**

### 開発機（M5StampS3A / AtomS3）
GPIO に余裕があるので **SPI4線 + IRQ + RSTn をフル配線**して開発する。
ピン番号は `boards/*.h` で定義。3.3V はボードから直接取れる。

### StampFly（下流・Phase 6）
| 案 | 配線 | 長所 | 短所 |
|---|---|---|---|
| **A: GROVE 2系統併用** | G13,G15,G1,G2 → SCK/MOSI/MISO/CS | 半田付け無しに近い | IRQ/RST 無し（ポーリング）、I2C/UART 拡張を両方潰す、カスタムケーブル必須、**StampFly の GROVE は電池電圧（満充電 ~4.35V、チップの絶対最大 4.0V 超）→ LDO で 3.3V が必要** |
| ~~B: 空きGPIO 直付け~~ | ~~G5,G10,G41,G42~~ | — | **【訂正 2026-08-21】成立しない。** この4本は現行ファームで**モータPWM**（`vehicle/main/config.hpp:63-66`）。当時の調査が古いリポジトリを見てモータピンを見落としていた |
| C: 内蔵SPI2 相乗り | G14/G43/G44 + 新規CS | 線が3本節約 | これらはコネクタに出ていない＝直付け必須、BMI270 の 10MHz バスと競合 |

→ **A を前提に設計**（IRQ 不要な作りにしておく）。~~B は実機のパッド有無を確認して判断。~~
  **B は成立しない（R3 参照）。**
→ **補強材料(調査5)**: M5Stack 公式ライブラリは `attachInterrupt` を一切使わない
  **完全ポーリング方式**（`dwt_readsysstatuslo()` ループ監視）。
  つまり **IRQ 線が無くても元の実装がそのまま動く**。案 A の最大の懸念が消えた。
→ いずれも **StampFly の GROVE は電池電圧（満充電 ~4.35V、チップの絶対最大 4.0V 超）→
   LDO（Low Dropout regulator、低損失レギュレータ）で 3.3V を作る変換（60mA 以上）を
   載せた小さな変換基板が必要**。FPC 12P → 配線の変換も同基板でやるのが素直。

### 電力
モジュールはタグ動作 58.0mA @3.3V。StampFly の GROVE は電池電圧（満充電 ~4.35V、
チップの絶対最大 4.0V 超のため LDO 必須）で、供給能力および LDO 後の 3.3V での
供給能力は**未公開**。
→ 実機で電流実測して確認する（Phase 6 の受入条件）。

---

## 4. フェーズ計画

### Phase 1: 基盤とSPI疎通 【最優先・ここが通れば全体の目処が立つ】
**実装済み（2026-08-19）。受入（Device ID 読み出し）は実機待ち**
- リポジトリ雛形、ESP-IDF プロジェクト、`uwb_port` 抽象定義
- `uwb_qm33120` の最小実装（レジスタ read/write、ソフトリセット）
- **受入条件: M5StampS3A で Device ID `0xDECA0314` を読み出せる**
- 併せて AtomS3 でも同じことを確認（ピン定義の差し替えだけで通ること）

### Phase 2: 単一リンクの測距
**実装済み（2026-08-19）。受入は実機待ち**（SS/DS-TWR は `uwb_qm33120_twr.cpp`、`firmware/twr`）
- M5Stamp-UWB の PHY（physical layer、物理層）設定・フレーム送受信・タイムスタンプ取得を移植
- `uwb_twr`: SS-TWR（Single-Sided TWR、片側二方向測距）→ DS-TWR の状態機械
- **受入条件: 2台（タグ1 + アンカー1）で距離値が安定して出る**
- **必要ハード: M5Stamp UWB Module ×2 + ホストボード ×2**

### Phase 3: アンテナ遅延キャリブレーション
**一部実装。NVS 保存とコンソール設定は済み、校正ツール（tools/）は未、受入は実機待ち**
（手順は `GETTING_STARTED.md` §9 / `EXPERIMENT_PLAN.md` 実験 4）
- 既知距離での遅延推定、値の NVS（Non-Volatile Storage、ESP-IDF の不揮発設定ストレージ）保存
- **受入条件: 既知距離 1m/3m/5m で誤差が仕様値（±0.14m）内に収まる**
- ホスト側キャリブレーションツール（tools/）

### Phase 4: マルチアンカー測距 + 測位
**実装済み（2026-08-20）。受入と「1周あたりの所要時間の実測」は実機待ち**
- ~~複数アンカーへの測距スケジューリング（TDMA 的な順次ポーリング）~~
  **実装済み**（`uwb_ranging_scheduler.hpp`）
- ~~`uwb_loc` の vendoring と `uwb_meas[]` への接続~~ **実装済み**（`components/uwb_loc/`、`uwb_ranging_pipeline.hpp`）
- ~~Lv0（三辺測量）→ Lv2（Beck+Huber）で 3D 位置を出す~~ **実装済み**
- ~~アンカー座標の設定手段（コンパイル時定数 → NVS → シリアルCLI）~~ **実装済み**（`components/uwb_cfgstore/` + 各ファームのコンソール）
- ~~**アンカー登録テーブル駆動**のスケジューラ（台数はテーブル長で決まる。4台以上の任意台数）。
  各アンカーに固有のショートアドレスを割り当てる
  （`firmware/anchor` に Kconfig/NVS でアドレスを設定できるようにする）~~
  **実装済み**（`components/uwb_ranging/include/uwb_ranging_anchor_table.hpp`）
- ~~**欠測を常態として扱う**: 応答が返らないアンカーはスキップし、
  その周期の有効測距数が4件以上なら解く。4件未満なら「測位不能」を返す~~
  **実装済み**（`uwb_ranging` パイプライン。ホストテスト pipeline に「5台中2台欠測」「1台欠測」のケースあり）
- 1周あたりの所要時間を実測し、インターバルを詰める
- ~~N点測距 → Lv0(三辺測量) で初期値 → Lv2(Beck+Huber) で本解~~ **実装済み**（`uwb_ranging_pipeline.hpp`）
- ~~`uwb_anchors_coplanar()` で同一平面配置を検出し、縮退時は警告を出す~~
  **実装済み**（`uwb_ranging_anchor_table.cpp:90`、coplanar フラグを返し呼び出し側が警告）
- **受入条件: アンカー4台の既知配置で3D位置が出ること、
  かつ5台に増やしたとき1台を強制的に外しても解が維持されること**
  （静止位置の誤差 0.3m 程度を目標）
- **必要ハード: M5Stamp UWB Module ×6（確保済み） + ホストボード（台数要確認。
  4台あれば受入条件の下限は検証できる）**

### Phase 5: 動体追従と出力整備
**一部実装。Lv3 EKF 統合と JSON Lines 出力は済み、UDP は未、float は実機待ち、受入は実機待ち**
- Lv3 EKF（`uwb_ekf`）の統合。まず double、検証後に `UWB_USE_FLOAT` を検討
- 出力 I/F: シリアル（JSON Lines）／UDP。uwb_localizer の Python 可視化と接続
- 更新レート・レイテンシの実測
- **受入条件: 歩行程度の移動でトラッキングが破綻しない**

### Phase 6: StampFly 統合
**未着手（設計のみ `STAMPFLY_INTEGRATION.md` 案 B-2）**
- 変換基板（FPC + LDO）の設計・製作
- `sf_hal_uwb_qm33120` として C++ ラッパを作り stampfly_ecosystem へ
- `tasks/uwb_task.cpp`、`sf_core/include/topics.hpp` に `Topic<UwbPosData, Queue, N>` 追加
- `sf_estimator` / ESKF（Error-State Kalman Filter、誤差状態カルマンフィルタ）への供給
- **受入条件: 機体に載せて飛行中に位置が出る。GROVE（StampFly は電池電圧 ~4.35V。
  LDO 後の 3.3V）の電流も実測確認**

**計画外で追加したもの**: `uwb_survey`（アンカー自動測量、`docs/SURVEY_SPEC.md`）、
`uwb_math`（対称3x3/2x2 閉形式・LDLᵀ・ブロックコレスキーの線形代数コンポーネント）、
`firmware/devtest`、`firmware/soltest`（測位ソルバの実機ベンチ）、
CI/Release 整備（`docs/PREBUILT_BINARIES.md`、Release zip への LICENSE 同梱）。

---

## 5. リスクと未確認事項

> **⚠ 番号の衝突に注意。** 本節の R1〜R10 は**この表の中だけの通し番号**であり、
> `docs/archive/REIMPL_PLAN.md` の R1〜R12（TWR 層の再実装項目）とは**別物**です。
> 例えば本節の R6 は「更新レート」、`archive/REIMPL_PLAN.md` の R6 は「IRQ 駆動化」を指します。
> 改番すると既存の相互参照が壊れるので、そのままにしてあります。

| # | リスク | 影響 | 対処 |
|---|---|---|---|
| ~~R1~~ | ~~Qorvo SDK のライセンス~~ | — | **解決済(2026-08-19)**: 改変込みのソース再配布は許可。条件=著作権/SPDX表示の保持＋**Qorvo製IC限定**。vendoring 可。詳細は archive/SURVEY_m5stamp_uwb_port.md |
| ~~R2~~ | ~~Arduino→ESP-IDF 移植の手間（SPI/GPIO/時刻APIの差）~~ | — | **解決済(2026-08-19)**: `uwb_port` 実装、Phase 1-2 完了 |
| ~~R3~~ | ~~StampFly に G5/G10/G41/G42 のパッドが無い~~ | — | **解決済(2026-08-21): 案B は成立しない。** 4本ともモータPWM。**案A（GROVE 2系統・IRQ無し）で確定** |
| R4 | StampFly GROVE（電池電圧、満充電 ~4.35V）の供給能力、または LDO 後 3.3V の供給能力不足 | Phase 6 で電源設計やり直し | 電流実測。最悪は機体のバッテリから直接取る |
| R5 | 公式ライブラリが TWR のみで TDoA 未実装 | 多数タグ運用は困難 | 当面 TWR 前提。TDoA は将来課題 |
| R6 | **更新レート**。DS-TWR を N アンカー逐次で回すと1周が長い（N に比例） | 飛行制御に使うには不足の可能性が高い | **Phase 2 完了時点で1リンクの所要時間を実測**。不足なら (a) SS-TWR へ切替 (b) インターバル短縮 (c) アンカーを間引いて交互にポーリング (d) EKF で補間、を検討。**N を増やすと直接効く**ので台数と更新レートはトレードオフ。**試算 31.3 Hz**（アンカー IRQ 化で 59.4 Hz、`TIMING_PRESETS.md`）。実測は実機待ち。SS/DS は `RangingMethod` で切替可 |
| ~~R7~~ | ~~ハードウェアの入手数~~ | — | **解決済(2026-08-19)**: 4台以上を保有。Phase 4 まで一気に検証可能 |
| R9 | Qorvo IC 限定条件（ライセンス条件3） | 別チップ転用不可 | 本リポジトリは QM33120W/DW3720 専用と明記する。SPDXヘッダ・LICENSES/ を必ず同梱。Release zip に LICENSES 同梱・IC 限定明記は済み（`dffcde5`）。自前コードへの SPDX ヘッダは未 |
| ~~R10~~ | ~~DS-TWR の距離計算が Responder(Anchor) 側にある実装~~ | — | **解決済(2026-08-19)**: Anchor 計算 → DWD フレームで Tag へ返送のまま採用（`uwb_qm33120_twr.cpp` `requestDSRange`）。SS-TWR は Tag 側で完結 |
| R8 | M5StampS3A の PSRAM 有無が資料間で矛盾 | メモリ設計 | 実機で確認。`boards/*.h` は ESP32-S3FN8（PSRAM 無し）前提で設計。実機確認は未 |

---

## 6. 直近の着手順

1. ~~R1（Qorvo SDK ライセンス）の確認~~ → **完了・クリア**
2. ~~M5Stamp-UWB の移植量見積もり~~ → **完了（実質 wrapper 1,946行のみ）**
3. ~~リポジトリ雛形 + ESP-IDF プロジェクト骨格 + `uwb_port` I/F 設計~~ → **完了（2026-08-19）**
4. ~~Phase 1 実装（M5StampS3A / AtomS3 で Device ID `0xDECA0314` 読み出し）~~ → **実装 2026-08-19・実機受入は未**
5. 実機検証（`docs/EXPERIMENT_PLAN.md` の順で probe → twr → anchor×5 + tag）← **いまここ**

### 確定した前提（ユーザ回答 2026-08-19）
- 保有ハード: **M5Stamp UWB Module / ホストボード ともに4台以上** → Phase 4（3D測位）まで実機検証可能
- StampFly 統合先: **`firmware/vehicle`**（C++ ラッパはその流儀に合わせる）
  - ※ユーザ指摘により訂正: 旧 `vehicle_new` が現 `vehicle` にリネーム済み。
    旧 `vehicle` は `vehicle_old` になった。
    ~~ローカルクローンは 2026-05-09 時点で古い（Phase 6 着手前に更新が必要）~~
    **`third_party/stampfly_ecosystem` は 2026-08-19 時点のクローン（`e52cc04`）に更新済み**
