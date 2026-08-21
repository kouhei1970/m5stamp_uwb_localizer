# 単位のリファレンス: UUS / DTU / 実マイクロ秒

**TWR（Two-Way Ranging、双方向測距）の遅延値を読み書きする前に必ずここを読むこと。**
この単位を取り違えると測距が成立しない。しかも症状は「距離が出ない」だけで、
どこがずれているのかログからは分からない。

本リポジトリで実際に踏みかけた罠であり、`docs/HANDOFF.md` の
「落とし穴 2」および「誤り 10」として記録されている。

---

## 1. 三つの単位

| 単位 | 定義 | 値 |
|---|---|---|
| **DTU** (device time unit) | `1 / (499.2 MHz × 128)` | **≒ 15.65 ps** |
| **UUS** (UWB microsecond) | 499.2 MHz の **512 周期** | **512/499.2 µs = 1.02564… µs** |
| 実マイクロ秒 | 普通の µs | 1 µs |

499.2 MHz は DW3000 の UWB（Ultra-Wideband、超広帯域無線）基本クロック（UWB チャネルの中心周波数もこの整数倍）。
チップ内のタイムスタンプ用カウンタ（40bit）は DTU で刻まれている。

### なぜ「1 µs ちょうど」にしなかったのか

```
1 UUS = 512 × 128 = 65536 DTU = 2^16 DTU
```

**UUS は DTU のちょうど 16bit シフト**である。ハードのカウンタから見て
桁合わせがシフト1回で済む単位を選んだ結果、実 µs から 2.56% ずれた。

これが `components/uwb_qm33120/src/uwb_qm33120_internal.hpp:192` の

```cpp
static constexpr uint64_t kUusToDwtTime = 65536ULL;
```

の正体である。

参考: 実マイクロ秒 → DTU は `1e-6 / 15.65e-12 = 63897.6 ≒ 63898`。
**65536 / 63898 = 1.02564** ＝ 1 UUS の実 µs 換算そのもの。

---

## 2. どの API がどの単位を取るか

| API | 単位 | 出典 |
|---|---|---|
| `dwt_setrxaftertxdelay()` | **UUS** | `components/qm33120w_sdk/deca_device_api.h:2681`「The delay is in **UWB microseconds**, 20-bit value」 |
| `dwt_setrxtimeout()` | **UUS** | 同 :2360「in **1.0256 us** (512/499.2MHz) units」 |
| `dwt_setdelayedtrxtime()` | **DTU >> 8**（40bit カウンタの上位 32bit） | ラッパ側で `× kUusToDwtTime` してから渡す |

### 本プロジェクトのフィールドの内訳

`RangeConfig` / `DSRangeConfig`（`components/uwb_qm33120/include/uwb_qm33120_types.hpp`）の
`*Uus` フィールドは**すべて UUS**。ただし SDK へ渡るまでの経路が 2 通りある。

| フィールド | 経路 |
|---|---|
| `responseRxAfterTxDelayUus` | UUS のまま `dwt_setrxaftertxdelay()` へ |
| `finalRxAfterResponseTxDelayUus` | 同上 |
| `resultRxAfterFinalTxDelayUus` | 同上 |
| `rxTimeoutUus` | UUS のまま `dwt_setrxtimeout()` へ |
| **`responseTxDelayUus`** | **× 65536 して DTU 化 → `dwt_setdelayedtrxtime()`** |
| **`finalTxDelayUus`** | 同上 |

`hostTimeoutMs` だけは **ms**（ホスト側ポーリングループの上限。SDK API には渡らない）。

---

## 3. 【罠】Qorvo 公式サンプルの `*_UUS` 定数は実マイクロ秒

`docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/shared_data/shared_defines.h:37`

```c
#define UUS_TO_DWT_TIME 63898
```

63898 は **「実 µs → DTU」の係数**である。つまり Qorvo 公式サンプルが
`*_UUS` という名前で定義している定数は、**名前に反して実マイクロ秒**で書かれている。

| | 係数 | 意味 |
|---|---|---|
| Qorvo 公式 `shared_defines.h:37` | `UUS_TO_DWT_TIME 63898` | **実マイクロ秒** → DTU |
| M5Stack / 本プロジェクト | `kUusToDwtTime 65536` | **真の UUS** → DTU |

**⇒ Qorvo の値を `*Uus` フィールドへそのまま代入すると、意図した遅延より約 2.5% 長くなる。**

> **命名が誤っているのは Qorvo 側であり、M5Stack 由来の `Uus` という命名は正しい。**
> 当初は「`Uus` → `Us` にリネームすべき」と判断したが誤りだった
> （経緯は `docs/REIMPL_PLAN.md` R1、`docs/HANDOFF.md` の誤り 10）。

### 変換ヘルパを必ず通すこと

`components/uwb_qm33120/include/uwb_qm33120_units.hpp`

```cpp
constexpr uint32_t usToUus(uint32_t us);   // 実µs -> UUS。四捨五入
```

ESP-IDF / Qorvo SDK のヘッダに依存しないので、ホスト側テスト
（`tools/test_pipeline`）からそのまま include して検算できる。

---

## 4. 実例: Qorvo `ex_05b_ds_twr_resp` の値を持ってくる

Qorvo 公式の DS-TWR（Double-Sided TWR、両側二方向測距）responder サンプル（本プロジェクトのアンカーに相当）:

```c
/* docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/examples/ex_05b_ds_twr_resp/ds_twr_responder.c */
#define POLL_RX_TO_RESP_TX_DLY_UUS  900   /* :85 */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500   /* :87 */
#define FINAL_RX_TIMEOUT_UUS        220   /* :89 */
```

この 900 は**実 900 µs** であって 900 UUS ではない。本プロジェクトへ持ってくるなら

```
usToUus(900) = 878   ->  DSRangeConfig::responseTxDelayUus = 878
usToUus(500) = 488   ->  DSRangeConfig::finalRxAfterResponseTxDelayUus = 488
```

`900` と書いてしまうと 923 µs、つまり **23 µs（2.56%）長くなる**。

`docs/TIMING_PRESETS.md` の `AnchorIrq` プリセットが `responseTxDelayUus = 878` に
なっているのはこの換算の結果であり、**Qorvo 公式の DS-TWR responder と同じ実時間**である。

> **⚠ `.c` ファイルは非 UTF-8（latin-1）。`grep` が 0 件を返したらまずこれを疑うこと。
> `grep -a` か `iconv` を使う**（`docs/HANDOFF.md` 運用ルール 3）。

---

## 5. 早見表

| 実 µs | UUS | 用途 |
|---:|---:|---|
| 205 | 200 | `resultRxAfterFinalTxDelayUus`（PollingBoth） |
| 513 | 500 | `finalRxAfterResponseTxDelayUus` |
| 700 | 683 | `finalTxDelayUus`（BothIrq） |
| 900 | 878 | `responseTxDelayUus`（IRQ プリセット。Qorvo 公式と同値） |
| 1231 | 1200 | `rxTimeoutUus`（IRQ プリセット） |
| 1400 | 1365 | `finalTxDelayUus`（AnchorIrq） |
| 1846 | 1800 | `finalTxDelayUus`（PollingBoth、現行既定） |
| 3077 | 3000 | `responseTxDelayUus`（PollingBoth、現行既定） |

```
UUS -> 実µs :  x * 1.02564        （x * 512 / 499.2）
実µs -> UUS :  x * 0.975          （x * 4992 / 5120、usToUus() が四捨五入で実装）
UUS -> DTU  :  x * 65536
```

## 関連文書
- `docs/GLOSSARY.md` — 略語の一覧（UUS 以外の用語はこちら）
- `docs/TIMING_PRESETS.md` — 遅延プリセットの実際の値と、その導出
- `docs/REIMPL_PLAN.md` R1 — この単位問題を発見・訂正した経緯
- `docs/SOURCE_POLICY.md` — 「一次資料 = Qorvo SDK/UM/APS」の格付け
