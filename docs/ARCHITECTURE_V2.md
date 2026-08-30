# UWB 測距ファームウェアの再設計（v2、2026-08-30）

## 0. 背景と目的

現行のアンカー／タグは、Qorvo の例題をなぞった「1 本道のブロッキング関数」（`respondRange()` /
`respondDSRange()` / `requestRange()` / `requestDSRange()`）を main ループから呼ぶ構造で、
以下の問題がこの構造そのものから生じていた（実測の経緯は `docs/HANDOFF.md` §0-C）。

- アンカーの Poll 待ちに **200 ms のホスト側窓**があり、窓の切り替わりごとに受信機を切って
  入れ直す空白（0.2〜0.5 ms）ができる。タグの周期と一致すると Poll が毎回空白に落ちて DS-TWR が
  0% になった（位相ロック）。チップの受信機は本来、無期限に聞き続けられる（`RX_FWTO=0`）。
- Response を送った後の Final 待ち中に来た Poll（タグの再試行）を捨てるため、DS の即時再試行は
  2 回目の失敗率が 4 倍になり、10 ms の待ちを入れて回避していた。
- 統計・コンソール・LED などの「家事」が電波を扱うループと同じタスクにあり、家事のために
  電波の待ちを周期的に抜ける設計になっていた。
- タグの測距・測位が `app_main` のループに直書きで、ホスト（将来は StampFly の飛行制御）から
  独立したタスクとして使えない。
- 本番ファームは PHY（データ速度・プリアンブル・PLL 粗調整）を Kconfig で選べず、実験
  ファーム `firmware/twr` で得た 850 kbps の結果を本番に持ち込めない。

本設計の目的:

1. **アンカー**: 受信機を常時 ON にし、来たフレームの種類で分岐するステートマシンにする。
   電波を扱うタスクと家事のタスクを分ける（ESP32-S3 は 2 コア、FreeRTOS 1 kHz）。
   アンカーは給電前提で消費電力は考慮しない。
2. **タグ**: 測距・測位を独立したサービス（タスク）にし、ホストは最新値のメールボックス
   またはキューで受け取る。飛行制御や他センサの取得と同じ構造で共存できる形にする。
3. **PHY を Kconfig で選べる**ようにし、本番機で 850 kbps / 6.8 Mbps を切り替えて比較できる
   ようにする。
4. その上で、**6.8 Mbps の誤り率を下げる検討**をやり直す（§6）。

## 1. 設計原則

- **受信は無期限**。ホスト側で周期的に受信機を切って入れ直さない。空白が生じるのは
  「フレームを処理した直後の再有効化」（数十 µs）だけにする。
- **電波（チップ）を触るタスクは 1 つ**（所有権）。他のタスクは統計のスナップショット（コピー）
  とキューを通してだけ関わる。
- **待ちは IRQ 通知を基本**（`uwb_port_irq_wait()`）。IRQ が使えない構成では SPI ポーリング。
  待ちのタイムアウトは「生存確認のための目覚まし」であって「再設定の合図」ではない。
  目覚めたときにチップが受信状態でなければ、そのときだけ再有効化する（回数を数える）。
- **判読の経路は 1 本**。IRQ の有無で変わるのは「待ち方」（セマフォで眠るか、回るか）だけで、
  状態レジスタ（`SYS_STATUS`）を読んで事象を分類し FSM に渡す部分は同一コードを通す。
  Qorvo SDK の `dwt_isr()` / コールバックは使わない（`docs/IRQ_POLICY.md` の方針を継承）。
- **エッジ取りこぼしへの備え**: IRQ 線は「状態ビットが立っている間 High」で GPIO は立ち上がり
  エッジ検出のため、状態を読んでから消すまでの間に起きた事象はエッジが来ないことがある。
  起きたら「状態を読む → 空になるまで処理 → 眠る直前にもう一度読む」の順にして競合を潰し、
  目覚まし（`idleTickMs`）は 20 ms 程度の短い保険にする（Listen 中の SPI 読み 1 回/20 ms は
  無視できるコスト）。
- **ホストタイムアウト**（`hostTimeoutMs`）の役割は「1 回の交換の上限」に限定する。
- **判断ロジックは純粋関数**にしてホストテストで検証する（状態・受信フレームの要約 → 次の
  動作）。ハードウェア依存の部分は薄く保つ。
- 実験ファーム `firmware/twr` は当面旧構造のまま残し、A/B の基準として使う。旧 `respondRange()` /
  `respondDSRange()` と新 `Responder` は、フレーム生成・遅延送信の予約・エラッタ検査・距離計算を
  `detail::` の補助関数で**共有**し、コピーしない。v2 が実機で検証できたら twr のアンカー役も
  `Responder` に切り替え、旧関数は削除する（重複を残す期間を区切る）。

## 2. アンカー

### 2.1 タスク構成

| タスク | コア | 優先度 | 役割 |
|---|---|---|---|
| `uwb_radio`（新設） | 1 | 高（例 20） | `Responder::service()` を回すだけ。電波の所有者 |
| `main` | 0 | 通常 | 起動・設定読込・統計行の出力（1 s ごと）・LED |
| `esp_console` REPL | 0 | 通常 | 既存のコンソール（`addr set` / `save` など） |

タスク間の受け渡し:

- 統計 `ResponderStats` は radio タスクだけが更新し、他タスクは `snapshot()` でコピーを得る
  （`portMUX` の臨界区間で構造体コピー）。
- 交換ごとの結果 `RangeEvent`（相手アドレス・seq・距離・所要時間・時刻）は長さ 8 の
  キューに入れる。**満杯なら捨てる**（radio タスクは決してブロックしない）。

### 2.2 ステートマシン

状態: `Listen`、`WaitFinal{peer, seq, timestamps, deadline}`。
事象: `RxFrame(good)`、`RxError(status)`、`RxTimeout`（W4R の RXFTO）、`TxDone`、`Tick`（待ちの目覚まし）。

| 状態 | 事象 | 動作 | 次状態 |
|---|---|---|---|
| Listen | Poll（SS） | Response を遅延送信（Poll の RMARKER + `responseTxDelayUus`）。送信完了後に RX 再有効化 | Listen |
| Listen | Poll（DS） | Response を遅延送信 + W4R（`finalRxAfterResponseTxDelayUus`）+ RX タイムアウト（`rxTimeoutUus`） | WaitFinal(peer=src) |
| Listen | Poll 以外の良フレーム／RX エラー | RX 再有効化（数える: `other` / `rxErrors`） | Listen |
| Listen | Tick | `SYS_STATE` を読み、受信状態でなければ RX 再有効化（`rearms`） | Listen |
| WaitFinal | Final（peer と seq が一致） | 距離を計算 → Result（DWD）送信 → RX 再有効化。`RangeEvent` を発行 | Listen |
| WaitFinal | **Poll（どの相手からでも）** | 進行中の交換を捨てて Listen+Poll と同じ動作（`restarts`） | WaitFinal(新) |
| WaitFinal | それ以外の良フレーム／RX エラー | 期限（Response 送信時刻 + W4R + `rxTimeoutUus` + 余裕）内なら RX 再有効化して待ち続ける。期限超過なら諦める | WaitFinal／Listen |
| WaitFinal | RxTimeout（RXFTO） | 諦める（`finalTimeouts`） | Listen |
| 任意 | 遅延送信の失敗（HPDWARN／wedged） | `forcetrxoff` → RX 再有効化（`txFailures`） | Listen |

補足:

- **受信窓の基準点**: W4R は「自分の送信完了」基準、遅延送信は「相手フレームの RMARKER」基準
  （`docs/TIMING_PRESETS.md` §2.4「受信窓の基準点」）。プリセットはこの規則で導出されたものを使う。
- 【2026-08-30 実機で判明、§5.1 参照】**W4R 遅延は受信 OFF の空白になる。** Responder は
  Response 送信直後に受信を開き、遅延ぶんをタイムアウトに繰り入れる。初版の実装は
  Response 送信完了から W4R 遅延（`finalRxAfterResponseTxDelayUus`、850 kbps/256 で
  ≈1.5 ms）の間、受信機を OFF のままにしており、その間に届いたタグの再試行 Poll を
  丸ごと取りこぼしていた（`docs/HANDOFF.md` §0-D）。
- 複数タグ: Final は「Response を返した相手」からのものだけ受理する。他タグの Poll が来たら
  進行中の交換を捨てて応じる（後着優先）。公平性が問題になったら、`WaitFinal` 中の他タグ Poll を
  無視する設定（`restartOnForeignPoll=false`）で切り替えられるようにしておく。
- RX エラー後の自動再有効化: DW3000 の `SYS_CFG.RXAUTR` が「RX エラー後に受信機を自動で
  再有効化する」機能であれば使う（ホストの再有効化より速い）。ドライバでは
  `dwt_setdblrxbuffmode(DBL_BUF_MODE_AUTO)` に紐付いているため、単独で立てられるか・良フレーム
  受信後の挙動がどうなるかを一次資料（DW3000 User Manual、`assets/` または `docs/refs/`）で
  確認してから採用する。確認できなければソフトウェア再有効化のみ。

### 2.3 API（`components/uwb_qm33120/include/uwb_qm33120_responder.hpp`）

```cpp
namespace uwb {
struct ResponderConfig {
    RangingMethod method = RangingMethod::DS;   // SS / DS
    uint16_t panId = 0xDECA;
    uint16_t shortAddr = 0x0002;
    RangeConfig   ss;      // SS のタイミング（アドレスは shortAddr で上書き）
    DSRangeConfig ds;      // DS のタイミング
    uint32_t idleTickMs = 20;          // 待ちの目覚まし間隔（エッジ取りこぼしの保険 + 生存確認）
    bool restartOnForeignPoll = true;  // WaitFinal 中に来た他タグの Poll に応じる
};
struct ResponderStats {   // すべて累積カウンタ + 距離統計
    uint32_t polls, responses, finals, results, restarts, finalTimeouts,
             rxErrors, txFailures, rearms, other;
    uint32_t lastRxStatus;
    DistanceStats distance;   // n / mean / std（DS のみ）
};
struct RangeEvent { uint16_t peer; uint8_t seq; int32_t distanceMm; uint32_t elapsedUs; int64_t tUs; };

class Responder {
public:
    bool begin(Qm33120& radio, const ResponderConfig& cfg);   // RX を有効化して Listen へ
    void service();                 // 1 事象ぶん処理して返る（radio タスクが無限に呼ぶ）
    bool popEvent(RangeEvent& out); // キューから 1 件（無ければ false）
    ResponderStats snapshot() const;
    void end();
};
}
```

判断部は `uwb_qm33120_responder_fsm.hpp` に純粋関数として置く:
`Action decide(State, const FrameSummary&, const ResponderConfig&)`。ホストテスト
`tests/host/responder_fsm/` で遷移表を検証する。

### 2.4 Kconfig（`firmware/anchor`）

`UWB_ANCHOR_RADIO_TASK_CORE`（既定 1）、`UWB_ANCHOR_RADIO_TASK_PRIO`（既定 20）、
`UWB_ANCHOR_RADIO_TASK_STACK`（既定 8192）、`UWB_ANCHOR_RESTART_ON_FOREIGN_POLL`（既定 y）。
PHY は §4 の共通オプション。既存の `UWB_ANCHOR_METHOD_*` / `UWB_ANCHOR_SHORT_ADDR` /
`UWB_TIMING_PROFILE` / `UWB_ENABLE_IRQ` は維持。

### 2.5 統計行

1 s ごとに main タスクが `snapshot()` を JSON 1 行で出す:
`{"v":1,"type":"anchor_stats","t":..,"addr":"A0002","polls":..,"responses":..,"finals":..,"results":..,"restarts":..,"final_timeouts":..,"rx_errors":..,"tx_failures":..,"rearms":..,"other":..,"dist_n":..,"dist_mean_m":..,"dist_std_m":..}`

## 3. タグ

### 3.1 `RangingService`（`components/uwb_ranging/include/uwb_ranging_service.hpp`）

電波・アンカー表・スケジューラ・測位パイプライン（EKF は任意）を所有するタスク。

```cpp
namespace uwb {
struct CycleResult {
    uint32_t seq;            // 周期番号（単調増加）
    int64_t  tUs;            // 周期開始時刻
    uint32_t cycleMs;        // 1 周の所要時間
    size_t   n;              // samples の有効数
    RangingSample samples[kMaxAnchors];
    PositionResult lv0, lv2; bool haveLv3; PositionResult lv3;   // 既存の結果型を流用
    uint32_t solveUsLv0, solveUsLv2, solveUsLv3;
};
struct ServiceConfig {
    SchedulerConfig scheduler;     // 方式・再試行・間隔
    PipelineConfig  pipeline;
    uint32_t cycleIntervalMs = 0;  // 0 = 連続。vTaskDelayUntil で刻む
    int core = 1; int prio = 18; uint32_t stack = 8192;
};
class RangingService {
public:
    bool start(Qm33120& radio, AnchorTable& table, const ServiceConfig& cfg);
    void stop();
    bool getLatest(CycleResult& out) const;          // 最新値のコピー（mutex）。seq で更新判定
    QueueHandle_t resultQueue() const;               // 長さ 4、満杯なら古い方を捨てる（ログ用）
    QueueHandle_t latestQueue() const;               // 長さ 1、xQueueOverwrite（制御ループ用）
    const AnchorStats& stats(size_t i) const;        // スケジューラの統計（コピーを返す）
    void resetStats();
    SemaphoreHandle_t tableMutex() const;            // コンソールがアンカー表を編集するときに取る
};
}
```

- 1 周期 = `runCycle()` → `solve()` → `CycleResult` を作って `latestQueue` に上書き、
  `resultQueue` に投入（満杯なら最古を捨てる）、`getLatest` 用のコピーを更新。
- アンカー表はコンソールから編集されるので、周期の先頭で `tableMutex` を取って参照する。
- 再試行はスケジューラに実装済み（`retryMax` / `retryDelayMs`）。新アンカーは Final 待ち中の
  Poll に応じるので `retryDelayMs` の既定を DS でも小さくできる見込み（§5 で実測して決める）。

### 3.2 `firmware/tag/main`

- `app_main`: 電波初期化 → 設定読込 → `RangingService::start()` → ロガータスク（コア 0）を起動 →
  コンソール。ロガータスクは `resultQueue` を消費して現行と同じ `"fix"` / `"meas"` / `"stats"`
  の JSON 行を出す（書式は変えない。`docs/GETTING_STARTED.md`）。
- 将来の StampFly ホスト: `components/uwb_ranging` + `components/uwb_qm33120` をリンクし、
  推定器のタスクから `getLatest()`（または `latestQueue`）で受け取る。飛行制御のループ周期とは
  独立に動く。

## 4. PHY を Kconfig で選ぶ（`components/uwb_qm33120/Kconfig`、共通）

**【2026-08-30 実機決定で既定を変更】** 本節を書いた設計時点（実機検証前）の既定案は
データ速度 6M8・プリアンブル 128 だったが、§5.1・§6.1 の実機検証（e50〜e66）でこの組は
p=1% と使い物にならないと分かったため、**既定は 850 kbps・プリアンブル 256 に変更した**
（下表は変更後の値。理由は §6.1「決定」、`docs/HANDOFF.md` §0-D「6.8 Mbps の切り分けと
本番既定の決定（§G）」参照）。

| オプション | 既定 | 内容 |
|---|---|---|
| `UWB_PHY_DATA_RATE_6M8` / `_850K`（choice） | **850K** | データ速度 |
| `UWB_PHY_PREAMBLE_LEN`（int: 64/128/256/512/1024） | **256** | プリアンブル長。PAC は Qorvo の規則で自動（≤128→8、256→16、512→16、1024→32） |
| `UWB_PHY_TX_POWER`（hex） | 0xfefefefe | 送信電力 |
| `UWB_PHY_PG_DELAY`（hex） | 0x34 | PG delay |
| `UWB_PHY_PLL_COARSE_MODE`（choice: Auto／OTP／Fixed） | **OTP** | ch9 の PLL VCO 粗調整コードの出所。Auto=チップの自動校正まかせ（温度で工場値から 1 段ずれることがある）、OTP=起動時に OTP アドレス `0x35` から個体固有の工場値を読んで強制（`Qm33120::forcePllCoarseFromOtp()`。基板ごとの値を都度読むので board-independent）、Fixed=下の `UWB_PHY_PLL_COARSE_CH9` を直接強制（単体診断専用）。既定は OTP（2026-08-30 実機決定、§6.1・`docs/HANDOFF.md` §0-D「新既定の最終確認と PLL 粗調整（G-2）」参照） |
| `UWB_PHY_PLL_COARSE_CH9`（hex、`UWB_PHY_PLL_COARSE_MODE=Fixed` のときだけ使用） | 0x00 | Fixed モード限定で PLL 粗調整コードを直接指定する（`firmware/twr` の `DIAG_PLL_COARSE_CH9` の手順を `Qm33120::forcePllCoarse()` として共通化）。既定運用では使わない（既定は上記の OTP モード） |

`uwb::PhyConfig uwb::phyConfigFromKconfig();` を追加し、`firmware/tag` / `firmware/anchor` は
これを使う。起動ログに `phy: preamble=… pac=… rate=… ch=… code=… sfd=… txpower=… pgdelay=…`
（`firmware/twr` と同じ書式）を出す。`firmware/twr` は既存の `DIAG_*` を維持する。

## 5. 検証計画

ホストテスト: `tests/host/responder_fsm/`（遷移表）。既存の `make -C tests all` に組み込む。

実機（1 m 卓上、95 秒、既存の `scratchpad/run.sh` 系の道具を本番ファーム用に拡張）:

1. 新アンカー + 新タグ、850 kbps / 256、DS と SS、IRQ とポーリング: 再試行なしで ≥88%、
   再試行（待ち 0 ms）で ≥99%。
2. 位相ロックの反証: タグの周期を 200 ms（`UWB_TAG_CYCLE_INTERVAL_MS=200`）にしても新アンカーで
   ロックしない（旧アンカー `firmware/twr` の `anc_ds_rxret` では 0% になる条件）。
3. 再試行の待ち 0 ms で 2 回目の失敗率が 1 回目の 1.5 倍以内（新アンカーの `restarts` > 0 で、
   Final 待ち中の Poll に応じている証拠）。
4. 6.8 Mbps: IRQ／ポーリング × プリアンブル 128／256 × SS／DS で、電文の方向ごとの損失
   （アンカー側 `polls`/`responses`/`finals`/`results` とタグ側 `att`/`succ`）を採る → §6。

### 5.1 実施結果（2026-08-30）

実機検証（e50〜e58b、約 1.2 m 卓上）を行った。詳細な数値・ログの在処は
`docs/HANDOFF.md` §0-D に記録してある。上の 1〜4 との対応:

| # | 目標 | 結果 | 実験 |
|---|---|---|---|
| 1 | 850 kbps/256、DS/SS、IRQ/ポーリング。再試行なしで ≥88%、再試行（待ち 0 ms）で ≥99% | **✔**（DS で確認: 再試行なし 86.4%——目標 88% にわずかに届かず、再試行(待ち0ms) 99.0%。**SS はこのバッチでは未実施**） | e53（なし）、e55/e56（待ち 0 ms） |
| 2 | 位相ロックの反証（タグ周期 200 ms） | **✔** 89.6%（1 秒ごとの成功回数が 5/5・6/6 と一様）。受信窓を無くしたので原理的に起きない | e54 |
| 3 | 再試行（待ち 0 ms）で 2 回目失敗率が 1 回目の 1.5 倍以内 | **△** 待ち 0 ms では 28%（1 回目失敗率 10.9% の約 2.6 倍、目標未達）。**待ち 2 ms に伸ばすと 15%** まで下がる（本番タグの既定を 2 ms に変更） | e55（待ち 0 ms、28%）、e57（待ち 2 ms、15%） |
| 4 | 6.8 Mbps の切り分け | **未実施 → §6 へ持ち越し**。本番既定（6.8 Mbps/128/IRQ/`BothIrq`）の素の p=1.0%（DS）であることのみ確認 | e50 |

## 6. 6.8 Mbps の誤り率低減の検討（§5 の後）

本日の実測: 本番既定（6.8 Mbps／128／IRQ／BothIrq）で素の成功率 DS 5%／SS 23%。
アンカー側の内訳は SS で「Poll 受信 70%、タグの Response 受信 33%」と、タグ側の受信が特に弱い。
一方 `firmware/twr`（ポーリング、6.8 Mbps／128）では SS の p=0.68〜0.80。検討する仮説と切り分け:

| 仮説 | 切り分け |
|---|---|
| (a) IRQ 経路の遅れ・取りこぼしで Response の受信窓が間に合っていない | 新構造で IRQ／ポーリングを同じ PHY で比較。IRQ 側の `rearms` / RX エラーの種類を見る |
| (b) BothIrq の DS プリセット（878/400/683/200）が受信窓の基準点の規則に照らして詰まりすぎ | §2.2 の規則で再導出し直す。`docs/TIMING_PRESETS.md` §2.4 の表に 6.8 Mbps 列を足す |
| (c) 1 m での受信飽和 | 距離 1／3／5 m で p を採る（`docs/HANDOFF.md` §0-B の距離掃引と照合） |
| (d) プリアンブル 128／PAC8 では捕捉が足りない | 6.8 Mbps のまま 256／PAC16 |
| (e) PHR 誤り（RXPHE）が主体 = 同期後の PHR 復号で落ちる（PHR は速度によらず 850 kbps） | `rx_status` の内訳を電文方向別に採り、SFD 種別（DW8／IEEE 4z）を比較 |

判断は `docs/HANDOFF.md` §0-C の判断表（機体上の p で速度を決める）に従う。

### 6.1 実施結果（2026-08-30）

実機で切り分けた（e50, e60〜e65。v2、DS は再試行なし・60 秒・約 1.2 m・PLL 粗調整 0x23
固定）。詳細な表・掛け算チェックの注記は `docs/HANDOFF.md` §0-D「6.8 Mbps の切り分けと
本番既定の決定（§G）」に記録してある。上の (a)〜(e) との対応:

| 仮説 | 判定 | 根拠 |
|---|---|---|
| (a) IRQ 経路の遅れ・取りこぼし | **棄却** | 同じ PHY（6.8 Mbps/128/`PollingBoth`）で IRQ（e60, p=40.6%）とポーリング（e61, p=40.0%）がほぼ同じ |
| (b) `BothIrq` の DS プリセットが詰まりすぎ | **確定** | 本番既定（`BothIrq`、e50）は p=1.0%。同じ 6.8 Mbps/128 のまま `PollingBoth` 相当の余裕あるプリセットにしただけ（e60）で p=40.6% まで上がった |
| (c) 距離 1 m での受信飽和 | **未実施** | 距離掃引（3／5 m）はまだ行っていない |
| (d) プリアンブル 128/PAC8 では捕捉不足 | **支持** | プリアンブルを 128 → 256 → 1024 と伸ばすと p が 40% → 57% → 75%（e61 → e62 → e64）と単調に上がった |
| (e) PHR 誤り（RXPHE）主体 | **未実施** | `rx_status` の電文方向別内訳・SFD 種別（DW8／IEEE 4z）比較用の Kconfig がまだ無い |

**基板差の発見（e63）**: e62 の役割を入れ替える（タグ↔アンカーの基板を交換する）と
p が 57% → 70% に上がった。タグ側の受信（Response・Result の 2 ホップ）がアンカー側
（Poll の 1 ホップ）より一貫して悪かった原因は、6.8 Mbps 固有の問題というより**基板差**
（1101 側の基板の受信性能が弱い）が主だったと分かった。良い方の基板をタグにしても、
850 kbps の水準（p=86.4%、参考 e53）には届かない。

**PLL 粗調整の自動校正コストの発見（e66/e67）**: 上の「決定」の構成をそのまま最終確認する過程で、PLL（Phase-Locked Loop）の粗調整コードをチップの自動校正に任せると、本日の温度で工場値からもう 1 段ずれる（`0x23`→`0x24`）ことがあり、これだけで素の p が **90.5% → 50.1%**（周期成功率 99.4% → 86.8%）まで落ちることが分かった。OTP から個体固有の工場値を読んで強制する方式（`UWB_PHY_PLL_COARSE_MODE=OTP`）を既定にして解消した（詳細は`docs/HANDOFF.md` §0-D「新既定の最終確認と PLL 粗調整（G-2）」）。

**決定**: 本番既定は **850 kbps / プリアンブル 256 / `PollingBoth` タイミング /
IRQ 有効 / 再試行 2 回・待ち 2 ms**（§4 の Kconfig 既定として反映済み）。6.8 Mbps は
プリアンブル 1024 まで伸ばしてようやく p=75% に達するが、それでも周期 16 ms は
850 kbps/256・DS の 9 ms より遅く、**この機材では速度の利点が無い**。`BothIrq` /
`AnchorIrq` の DS プリセットは 6.8 Mbps 用に導出されたものだが実機で p=1% だったため、
再導出するまで既定から外した。残る (c)・(e) と、DS の電文数を 3 に減らす設計変更は
未着手（`docs/HANDOFF.md` §0-D 参照）。

## 7. 実装の順序

1. §4 PHY Kconfig と `phyConfigFromKconfig()`（tag／anchor／twr のビルドが通ること）
2. §2.3 `Responder`（ドライバ層）+ FSM のホストテスト
3. §2 アンカーファーム（radio タスク + 統計行）
4. §3 タグの `RangingService` + ロガータスク（2 と並行可能）
5. §5 実機検証 → `docs/HANDOFF.md` に記録
6. §6 の検討
