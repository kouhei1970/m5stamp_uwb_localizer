# StampFly への UWB 測位統合 設計検討 (2026-08-21)

本リポジトリ（`m5stamp_uwb_localizer`）の UWB 測位を、StampFly の位置制御に使うための設計検討。

## この文書の前提（最初に読むこと）

| 項目 | 状態 |
|---|---|
| 本リポジトリの UWB スタック | **実機未検証**。Device ID 読み出しすら未達（`PROGRESS.md:411`）。数値はすべて静的解析による見積もり |
| アンテナ遅延校正 | **未実施**。無校正では数十cm〜1m超の定常バイアスが乗る（`docs/archive/CRITICAL_REVIEW.md:130`） |
| StampFly 側 | `third_party/stampfly_ecosystem/`（2026-08-19 取得）は**読み取り専用**。本検討で一切変更していない |
| StampFly の UWB 対応 | リポジトリ全体を `uwb` で grep して**0件**（`docs/archive/SURVEY_stampfly_ecosystem.md:32`）。完全新規 |
| 本文書の位置づけ | **実装計画ではなく設計検討**。実機が来て Phase 1（Device ID 読み出し）が通るまで、下記はすべて仮説 |

### 引用の記法

- `SF/` は `third_party/stampfly_ecosystem/firmware/vehicle/` の略。例: `SF/main/config.hpp:83`
- それ以外のパスは本リポジトリのルートからの相対パス
- **「事実」= 実コードの file:line を引いたもの。「推測」と明記したものはコードから直接は確認できていない**

---

# 1. StampFly 現行ファームの制御構造（実コード確認）

## 1.1 タスク構成

コア割り当ては `SF/tasks/tasks.cpp:42-103` の `xTaskCreatePinnedToCore` 実引数、優先度・スタックは `SF/main/config.hpp`、周期は各タスク実装のコード内定数から確認。

| タスク | 優先度 | スタック[B] | 周期 | コア | 周期の根拠 |
|---|---:|---:|---|---:|---|
| **ImuTask** | 24 (`config.hpp:83`) | 16384 (`:107`) | **400 Hz**（esp_timer 2500µs） | **1** | `config.hpp:132`, `tasks/imu_task.cpp:752-766` |
| **ControlTask** | 23 (`:84`) | 8192 (`:108`) | **400 Hz**（ImuTask 通知同期） | **1** | `tasks/control_task.cpp:224`, `config.hpp:147` |
| StateTask | 22 (`:85`) | 4096 (`:109`) | イベント駆動 + 20ms ポーリング | 0 | `tasks/state_task.cpp:311` |
| FlowTask | 20 (`:86`) | 8192 (`:110`) | 100 Hz | 0 | `config.hpp:133`, `tasks/flow_task.cpp:63` |
| MagTask | 18 (`:87`) | 8192 (`:111`) | 25 Hz | 0 | `config.hpp:134`, `tasks/mag_task.cpp:59` |
| BaroTask | 16 (`:88`) | 8192 (`:112`) | 50 Hz（実変換は 13〜16Hz） | 0 | `config.hpp:135`, `tasks/baro_task.cpp:47-57` |
| CommTask | 15 (`:89`) | 4096 (`:113`) | 50 Hz | 0 | `tasks/comm_task.cpp:60` |
| TofTask | 14 (`:90`) | 8192 (`:114`) | 30 Hz | 0 | `config.hpp:136`, `tasks/tof_task.cpp:53` |
| TelemetryTask | 13 (`:91`) | 4096 (`:115`) | 50 Hz | 0 | `tasks/telemetry_task.cpp:51` |
| PowerTask | 12 (`:92`) | 4096 (`:116`) | 10 Hz | 0 | `tasks/power_task.cpp:158` |
| ButtonTask | 10 (`:93`) | 4096 (`:117`) | 50 Hz | 0 | `tasks/button_task.cpp:92` |
| NotifyTask | 8 (`:94`) | 4096 (`:118`) | ~30 Hz | 0 | `tasks/notify_task.cpp:59` |
| ApiTask | 6 (`:97`) | 6144 (`:121`) | 20ms ポーリング | 0 | `tasks/api_task.cpp:82,1251` |
| TelloStateTask | 6 (`:98`) | 4096 (`:122`) | 10 Hz | 0 | `tasks/api_task.cpp:1310` |
| CLITask | 5 (`:95`) | 8192 (`:119`) | 非周期（REPL ブロッキング） | 0 | `tasks/cli_task.cpp:1166-1206` |
| LogTask | 5 (`:96`) | 4096 (`:120`) | 100 Hz（`vTaskDelay`） | 0 | `tasks/log_task.cpp:45,83` |

**コア配置は非対称**: コア1 は ImuTask + ControlTask の2つのみ。残り14タスクはすべてコア0
（`SF/tasks/tasks.cpp:44-57` のコメントに、コア1へ優先度22を同居させた際に数十秒の飢餓が
発生した実機バグの記録あり）。

→ **UWB タスクをどのコア・どの優先度に置くかは、この非対称配置の制約を受ける**（§5.4）。

## 1.2 姿勢制御と位置制御の階層 — **分離されていない**

**重要な事実: 位置ループと姿勢ループは別タスクでも別周期でもない。**

- `ControlTask` が呼ぶのは `controller.compute(state, setpoint, config::IMU_DT)` の**1回だけ**
  （`SF/tasks/control_task.cpp:351`）。`dt = IMU_DT = 0.0025f`（`SF/main/config.hpp:131`）
- カスケード4段は `PidController::compute()` の**内部で毎サイクル逐次実行**される:

| 段 | PID | 入力 → 出力 | 実装 |
|---|---|---|---|
| ① 位置（水平, outer） | `pos_x_`, `pos_y_` | 位置誤差(NED) → 目標水平速度 | `SF/components/sf_controller_pid/pid_controller.cpp:1326-1327` |
| ② 速度 | `vel_x_`, `vel_y_` | 速度誤差 → 目標水平加速度 | 同 `:1332-1333` |
| ②→③ 変換 | — | `pitch_sp = clamp(-ax_body/g)`, `roll_sp = clamp(ay_body/g)` | 同 `:1336-1341` |
| ③ 姿勢（角度, inner） | `att_roll_`, `att_pitch_` | 傾き誤差 → 目標角速度 | 同 `:455-456` |
| ④ レート（最内） | `rate_roll_/pitch_/yaw_` | 角速度誤差 → トルク | 同 `:774-776` |

高度（Z）は別カスケード `alt_pos_` → `alt_vel_`（同 `:527,540,628,631`）で、`ALT_HOLD` / `POS_HOLD`
の両方で使われる。

### 位置ループの「実効帯域」— ここが UWB のレート要求を決める

制御タスク自体は 400 Hz だが、**位置ループのゲインは非常に低い**:

| パラメータ | 既定値 | 出典 |
|---|---:|---|
| `position.pos.kp` | **0.4** | `SF/components/sf_core/params.cpp:771` |
| `position.pos.ti` | 5.0 | 同 `:772` |
| `position.vel.kp` | **3.0** | 同 `:773` |
| `position.vel.ti` | 2.0 | 同 `:774` |
| 位置ループ出力上限 `max_pos_vel_` | 1.0 m/s | `SF/components/sf_controller_pid/pid_controller.cpp:180-181` |

P ゲインをそのままクロスオーバ周波数の目安とすると:

- 位置ループ: **≈ 0.4 rad/s ≈ 0.064 Hz**
- 速度ループ: **≈ 3.0 rad/s ≈ 0.48 Hz**

`params.cpp:765-770` のコメントに、`pos.kp` を 1.0 → 0.4 に**下げた**経緯（外ループを遅くして
内ループとの時間スケール分離を回復させた）が記録されている。**位置ループは意図的に遅い。**

## 1.3 状態推定 — **位置は現状どう出しているか**

### 推定器の選択

既定は **ESKF**（`estimator.type` 既定 `0.0f`、`SF/components/sf_core/params.cpp:708`。
factory は `SF/tasks/imu_task.cpp:115-127` で `type==1` のときだけ相補フィルタ）。

`sf_estimator_complementary` は**水平位置・速度を推定しない**
（`SF/components/sf_estimator_complementary/include/complementary_estimator.hpp:22-23` に明記、
`getState()` が `position[0]/[1]` をゼロのまま返す: 同 `.cpp:202-213`）。
→ **位置制御を使う構成では実質 ESKF 一択。**

### ESKF の状態ベクトル（15次元 error-state）

`SF/components/sf_estimator_eskf/include/eskf_core.hpp:35-41`:

```
POS_X=0, POS_Y=1, POS_Z=2,   VEL_X=3, VEL_Y=4, VEL_Z=5,
ATT_X=6, ATT_Y=7, ATT_Z=8,   BG_X=9..11 (gyro bias),  BA_X=12..14 (accel bias)
```
公称状態は `pos_`/`vel_`/`q_`/`bg_`/`ba_`（同 `:260-266`）。共分散は `SymMat15 P_`（同 `:269`）。**単精度 float**。

### センサ融合の対応表

| 段 | 関数 | 拘束する状態 | 実装 |
|---|---|---|---|
| 予測 | `predict(accel, gyro, dt)` @400Hz | 全状態 | `eskf_core.cpp:169`（`pos_ += vel_*dt` は `:195`） |
| ToF | `updateToF(distance)` | **POS_Z のみ**（`H[POS_Z]=1`） | `eskf_core.cpp:530-556` |
| 気圧 | `updateBaro(altitude)` | POS_Z | `eskf_core.cpp:598-611`。**既定 OFF**（`params.cpp:800` `eskf.use_baro=0`） |
| 地磁気 | `updateMag(mag)` | 姿勢のみ | `eskf_core.cpp:612-644`。**既定 OFF**（`params.cpp:801` `eskf.use_mag=0`） |
| 加速度姿勢 | `updateAccelAttitude(accel)` | 姿勢 + accel bias | `eskf_core.cpp:645`。predict 毎に自動実行（`eskf_estimator.cpp:120`） |
| フロー | `updateFlowRaw(dx,dy,height,dt,gx,gy,squal)` | **VEL_X, VEL_Y のみ** | `eskf_core.cpp:729-784` |

### **結論: 水平位置 (x, y) には絶対観測が一切ない**

- **POS_Z**: ToF が直接拘束する絶対観測（`eskf_core.cpp:544-545`）。傾き補正あり
  （`height = distance·cos(roll)·cos(pitch)`、`eskf_core.cpp:539`）、傾き閾値超過で棄却（同 `:536-537`）
- **POS_X, POS_Y**: **直接観測する update 関数が存在しない。**
  フローは速度（VEL_X/VEL_Y）だけを拘束し、位置は predict の速度積分でしか動かない
  → **実質デッドレコニング。長時間では必ずドリフトする**

これが **UWB を入れる構造上の空白**である（推測ではなく、`eskf_core.cpp` に POS_X/POS_Y を
観測する `H` が1つも無いという事実）。

### 実際に達成できている性能（比較の基準として重要）

`SF/docs/poshold_journey.md:5,180`:

> 37g の超小型ドローン StampFly が、GPS もモーションキャプチャも使わず、
> オプティカルフロー＋ToF＋IMU だけで **手放し定点ホバリング（±6〜7cm, RMS 16mm）**

同 `:184`: 「**±6〜7cm がこの37g機のトルク効きでの実用限界**」。姿勢ループのチューニングに
伸びしろは無く、これ以上はハード（モータ/プロペラ）の話、と結論されている。

→ **UWB の意義は「定点保持精度の改善」ではない**（§4.0 で詳述）。

## 1.4 センサデータの受け渡し（`sf_core` Pub-Sub）

`SF/components/sf_core/include/topic.hpp` の3方式:

| 方式 | 実装 | 特性 | 用途（コメント） | 実例 |
|---|---|---|---|---|
| `Latest` | `topic.hpp:73-126` | mutex 保護、最新値のみ、`updated()` フラグ | 推定→制御、推定→テレメトリ | `estimate_state`（`topics.hpp:51`） |
| `RingBuffer` | `topic.hpp:149-268` | ロックフリー SPSC、満杯で最古ドロップ + overflow カウント | IMU→推定、全データ→ログ | `sensor_imu`（`RingBuffer,8`） |
| `Queue` | `topic.hpp:278-327` | FreeRTOS Queue、非ブロッキング publish | **低レートセンサ（ToF/Flow/Mag/Baro）** | `sensor_tof/flow/mag/baro`（すべて `Queue,2`、`topics.hpp:39-42`） |

→ **UWB は `Queue,2` が既存の低レートセンサと同じ流儀**（`topic.hpp:61-62` のコメントが明示）。

### 観測の注入点（ここが統合の実装箇所）

`SF/tasks/imu_task.cpp` の 400 Hz メインループ:

```
Step 2: g_estimator->predict(imu, IMU_DT);        (imu_task.cpp:857)
Step 3: processAsyncSensors();                    (imu_task.cpp:864、実体は :248)
          └ while (sensor_tof.read(tof))   g_estimator->updateTof(tof);    (:273-274)
          └ while (sensor_flow.read(flow)) g_estimator->updateFlow(flow);  (:292-293)
          └ while (sensor_mag.read(mag))   g_estimator->updateMag(mag);    (:307-309)
          └ while (sensor_baro.read(baro)) g_estimator->updateBaro(baro);  (:316-317)
Step 3.5: applyVerticalGroundHandoff();
Step 4: estimate_state.publish(g_estimator->getState());  (imu_task.cpp:931)
Step 5: ControlTask へ xTaskNotifyGive
```

**UWB の観測は `processAsyncSensors()` に1本足すだけで、既存の4センサと完全に同じ形で入る。**

## 1.5 タイムスタンプと遅延の扱い

### タイムスタンプは全データ型に付いている

`uint32_t timestamp`、単位は**マイクロ秒 [µs]**（`SF/components/sf_core/include/data_types.hpp:43,52,61,69,78,135,177`）。
`SF/components/sf_estimator_eskf/eskf_estimator.cpp:62-68` に「uint32_t は約71.6分でラップする」旨の記述あり。

### **しかし遅延補償の機構は存在しない**

- `TopicLatest` は履歴を持たない。`TopicRing` は単純 FIFO
- ESKF の各 `update*()` は**観測タイムスタンプを一切参照せず**、常に「現在の `pos_/vel_/q_`」に即座に補正をかける
  （`updateFlowRaw` はフロー自身のサンプル間隔 dt を計算するが、IMU 予測との時刻整合は取らない）
- PX4/EKF2 的なバッファ巻き戻し（OOSM 処理）は無い

→ **UWB のように「測距開始から結果が出るまで数ms〜数十ms」かかるセンサを入れるとき、
この遅延補償の欠如が設計上の考慮点になる**（§3.3 で定量評価）。

---

# 2. UWB 測位が出せる更新レート

## 2.1 計算モデルと定数

### 使う定数（すべて出典あり）

| 定数 | 値 | 出典 |
|---|---:|---|
| 1 UUS | **1.025641 µs** (= 512/499.2) | `components/qm33120w_sdk/deca_device_api.h:2360,2681` |
| SHR（preamble128 + SFD8 = 136 sym × 1017.63 ns） | **138.4 µs** | `docs/archive/CRITICAL_REVIEW.md:226-229` |
| フレーム全長 Poll / Resp / Final / DWD | 174 / 184 / 188 / **179** µs | `docs/archive/CRITICAL_REVIEW.md:234` |
| RMARKER→フレーム終端 Poll / Resp / Final / DWD | 35.6 / 45.6 / **49.6** / 40.6 µs | 同 `:235`（= 全長 − SHR） |
| FreeRTOS tick | **1000 Hz** → `vTaskDelay(pdMS_TO_TICKS(1))` = (0, 1] ms | `firmware/tag/sdkconfig.defaults` の `CONFIG_FREERTOS_HZ=1000` |
| ホスト SPI 固定オーバヘッド（推定） | **≈ 100 µs** | 16MHz SPI で約9トランザクション。**未実測** |

### タイムライン（DS-TWR、Poll の RMARKER を t=0 とする）

```
t=0                Poll RMARKER (Tag送信)
t=+35.6            Poll フレーム終端 → Anchor の RX 完了
t=+D1              Response RMARKER (Anchor)     D1 = responseTxDelayUus
t=+D1+45.6         Response 終端 → Tag の RX 完了
t=+D1+D2           Final RMARKER (Tag)           D2 = finalTxDelayUus
t=+D1+D2+49.6      Final 終端 → Anchor の RX 完了
t=+…+L_a           Anchor がホスト処理して DWD を即時送信開始（L_a = Anchor 折返し時間）
t=+…+L_a+179       DWD 終端 → Tag の RX 完了
t=+…+Q_t           Tag のポーリングループが検出（Q_t = Tag 検出遅延）
```

**検算**: 既定値 D1=3000 UUS=3076.9 µs, D2=1800 UUS=1846.2 µs で
`138.4 + 3076.9 + 1846.2 + 49.6 = 5111.1`、Final 終端は Poll RMARKER から
`3076.9 + 1846.2 + 49.6 = 4972.7 µs`。
`docs/archive/CRITICAL_REVIEW.md:95` の独立に導出された「DWF (final) 終端 +4,973 µs」と一致。
同 `:96` の「DWD 受信ウィンドウ +5,486 〜 +8,563 µs」も
`4973 + 500UUS(512.8) = 5486`、`5486 + 3000UUS(3077) = 8563` で一致する。
**→ 本モデルは既存文書と整合している。**

### 1リンク所要時間の式

```
T_link = 100 (SPI固定) + 138.4 (Poll SHR) + D1 + D2 + 49.6 (Final尾部)
         + L_a (Anchor折返し) + 179 (DWD air time) + Q_t (Tag検出遅延)      [µs]
```

`RangingScheduler::runCycle()` は `perAnchorIntervalMs=0`（既定、`uwb_ranging_scheduler.hpp:32`）
なのでアンカー間の追加待ちは無い。

```
T_cycle(5アンカー) = 5 × T_link      レート = 1000 / T_cycle(ms)  [Hz]
```

### 遅延送信の締切（どこまで詰められるかを決める制約）

`dwt_setdelayedtrxtime()` の DX_TIME は RMARKER 相当。プリアンブルはその **138.4 µs 前**から
送出が始まるので、ホストはそれまでに `dwt_starttx()` を打ち終える必要がある
（`docs/archive/REIMPL_PLAN.md:138`「DX_TIME − (プリアンブル+SFD の air time)」）。

| 側 | 折返し予算 | 計算 |
|---|---:|---|
| Anchor（D1 に対して） | D1 − 35.6 − 138.4 | Poll 終端から Response プリアンブル開始まで |
| Tag（D2 に対して） | D2 − 45.6 − 138.4 | Response 終端から Final プリアンブル開始まで |

| D1 / D2 | 予算 | 1msポーリング（検出0〜1000µs + SPI 100µs）で足りるか |
|---:|---:|---|
| 3076.9 µs（現状 D1） | 2902.9 µs | **余裕** |
| 1846.2 µs（現状 D2） | 1662.2 µs | **足りる**（最悪 1100 µs 使用、余裕 562 µs） |
| 1400 µs | 1226.0 µs | **ぎりぎり足りる**（余裕 126 µs） |
| 900.5 µs（Qorvo 公式 D1） | 726.5 µs | **足りない**。検出だけで最悪 1000 µs |
| 700.5 µs（Qorvo 公式 D2） | 516.5 µs | **足りない** |
| 450 µs | 276.0 µs | IRQ 駆動（折返し 60〜150µs）でのみ成立 |

→ **`docs/archive/REIMPL_PLAN.md:145`「R6 は R5 の前提条件」が数値で裏付けられた。**
Qorvo 公式値をポーリングのまま入れると、Anchor 側の折返しが間に合わず
`dwt_starttx()` が失敗する確率が **約 40%**（予算 726.5 µs に対し検出遅延が一様 (0,1000] と
仮定し、SPI 100 µs を引いた 626.5 µs を超える確率）になる。

## 2.2 段階別の数値

`L_a` / `Q_t` の仮定:
- **ポーリング**: 一様 (0, 1000] µs（tick=1000Hz）。typ = 500 µs / max = 1000 µs
- **IRQ 駆動**: `docs/archive/CRITICAL_REVIEW.md:240`「折返し 60〜150µs」より
  L_a: typ 100 / max 150 µs、Q_t（受信検出のみ、SPI が軽い）: typ 50 / max 100 µs

| # | 構成 | D1 [µs] | D2 [µs] | 1リンク typ/max [ms] | 5アンカー1周 typ/max [ms] | **測位レート typ/max [Hz]** |
|---|---|---:|---:|---:|---:|---:|
| **(a)** | **現状（R3-1 適用後、DS-TWR、両側ポーリング）** | 3076.9 | 1846.2 | **6.39 / 7.39** | **31.9 / 37.0** | **31.3 / 27.1** |
| (b1) | **R5：Anchor だけ IRQ 化（StampFly の現実解）** | 900.5 | 1400 | 3.37 / 3.92 | 16.8 / 19.6 | **59.4 / 51.1** |
| (b2) | R5：両側ポーリングのまま安全に詰めた上限 | 1400 | 1400 | 4.27 / 5.27 | 21.3 / 26.3 | 46.9 / 38.0 |
| (b3) | 参考：Qorvo 公式値をポーリングで入れた場合 | 900.5 | 700.5 | 3.07 / 4.07 | 15.3 / 20.3 | 65.2 / 49.2 ※**約40%失敗するので成立しない** |
| **(c1)** | **R5+R6：両側 IRQ、Qorvo 公式値** | 900.5 | 700.5 | **2.22 / 2.32** | **11.1 / 11.6** | **90.2 / 86.3** |
| (c2) | R5+R6：両側 IRQ、`CPU_PROCESSING_TIME` 相当まで攻める | 450 | 450 | 1.52 / 1.62 | 7.6 / 8.1 | 131.8 / 123.7 |
| — | 【別軸】SS-TWR 現状（D1=3000UUS、Tag ポーリング） | 3076.9 | — | 3.86 / 4.36 | 19.3 / 21.8 | 51.8 / 45.9 |
| — | 【別軸】SS-TWR + Anchor IRQ（Qorvo 公式 650µs、Tag ポーリング） | 650.3 | — | 1.43 / 1.93 | 7.2 / 9.7 | **139.5 / 103.4** |
| — | 【別軸】SS-TWR + 両側 IRQ | 650.3 | — | 0.98 / 1.03 | 4.9 / 5.2 | 203.3 / 193.4 |

### 計算過程の明示（(a) を例に）

```
D1 = 3000 UUS × 1.025641 µs/UUS = 3076.9 µs      (uwb_qm33120_types.hpp:357  responseTxDelayUus)
D2 = 1800 UUS × 1.025641 µs/UUS = 1846.2 µs      (uwb_qm33120_types.hpp:361  finalTxDelayUus)

T_link(typ) = 100    (SPI 固定オーバヘッド、推定)
            + 138.4  (Poll の SHR)
            + 3076.9 (D1)
            + 1846.2 (D2)
            +   49.6 (Final RMARKER→終端)
            +  500   (L_a: Anchor 折返し、1ms ポーリングの平均)
            +  179   (DWD フレーム全長)
            +  500   (Q_t: Tag 検出遅延、1ms ポーリングの平均)
            = 6390.1 µs = 6.39 ms

T_cycle = 5 × 6.39 = 31.95 ms  →  1000/31.95 = 31.3 Hz
```

max 版は L_a / Q_t を 1000 µs にするだけ（= 7.39 ms / 36.95 ms / 27.1 Hz）。

### 参照した設定値の出典

| 値 | 出典 |
|---|---|
| `responseTxDelayUus = 3000` | `components/uwb_qm33120/include/uwb_qm33120_types.hpp:357` |
| `finalTxDelayUus = 1800` | 同 `:361` |
| `resultRxAfterFinalTxDelayUus = 200`（R3-1 適用済み） | 同 `:389` |
| `resultRepeatCount = 1`（R3-1 適用済み） | 同 `:417` |
| `rxTimeoutUus = 3000` | 同 `:393` |
| `hostTimeoutMs = 10`（R9 適用済み） | 同 `:400` |
| Qorvo 公式 `POLL_RX_TO_RESP_TX_DLY_UUS 900`（DS 応答側、実µs） | `docs/refs/qorvo_ex_ds_twr_responder.c:106` |
| Qorvo 公式 `RESP_RX_TO_FINAL_TX_DLY_UUS 700`（DS 起動側、実µs） | `docs/refs/qorvo_ex_ds_twr_initiator.c:109` |
| Qorvo 公式 `POLL_RX_TO_RESP_TX_DLY_UUS 650`（SS 応答側、実µs） | `docs/refs/qorvo_ex_ss_twr_responder.c:88` |
| `CPU_PROCESSING_TIME 400` | `docs/refs/qorvo_api/DW3XXX_API_rev9p3/API/Src/config_options.h:57` |
| 実µs → UUS 変換 `usToUus()` | `components/uwb_qm33120/include/uwb_qm33120_units.hpp:48-52` |

**罠**: Qorvo 公式の `*_UUS` 定数は**実マイクロ秒**であり、本 API の UUS（1.0256 µs）ではない
（`docs/archive/REIMPL_PLAN.md:56-63`）。900 実µs → `usToUus(900) = 878 UUS` → 実時間 900.5 µs。
上表はすべてこの換算を通してある。

## 2.3 測位ソルバの計算時間（無視できるか）

`docs/PERF_ANALYSIS.md` のホスト実測（macOS）から:

| ソルバ | ホスト実測 | ESP32-S3 推定 | 根拠 |
|---|---:|---:|---|
| Lv2（Beck + Huber）N=5 | 約 5.4 µs（最適化前） | **0.2〜2 ms** | `PERF_ANALYSIS.md:146-147`。ESP32-S3 は double がソフトエミュレーション（同 `:11`） |
| Lv3 EKF update（5観測） | 約 2.7 µs（最適化前） | 0.1〜1 ms | 同 `:141` |

**注意**: `components/uwb_loc/` は上流の**最適化前**バージョンと byte 一致
（`PERF_ANALYSIS.md:186-191`）。上流の最適化（Lv2 で 4.3倍）はまだ取り込まれていない。

いずれにせよ **1周 32 ms に対して 1 ms 以下**なので、測位計算はレートのボトルネックではない。
**ただし実機未測定**（`firmware/soltest` に計測を仕込んであるが実機が無い）。

## 2.4 この数字の信頼性についての注意

- **すべて静的解析。実測はゼロ。**
- (a) の 31 Hz は「**全リンクが成功した場合**」の値。失敗すると `hostTimeoutMs = 10 ms` まで待つ
  ので、1台失敗するごとに +3.6 ms（≒ 28 Hz に低下）。
  `docs/archive/CRITICAL_REVIEW.md:272` は「5アンカーで全アンカーから安定して取れる確率」を
  R2 適用前で 15〜25% と見積もっている。R2/R3-1 適用後の実測値は未取得
- SPI 固定オーバヘッド 100 µs は**推定値**。実測で 2〜3倍あれば (c2) の数字は崩れる
- `L_a` の一様分布仮定は `docs/archive/CRITICAL_REVIEW.md:101` と同じ仮定であり、実測ではない

---

# 3. 位置制御に必要なレートとの突き合わせ

## 3.1 レートは足りるか → **足りる。それも大幅に。**

| 比較対象 | 周波数 | 出典 |
|---|---:|---|
| ControlTask（カスケード全段） | 400 Hz | `SF/tasks/control_task.cpp:224`, `SF/main/config.hpp:131` |
| **位置ループの実効帯域** | **≈ 0.064 Hz** | `position.pos.kp = 0.4`（`SF/components/sf_core/params.cpp:771`） |
| 速度ループの実効帯域 | ≈ 0.48 Hz | `position.vel.kp = 3.0`（同 `:773`） |
| 既存の高度観測（ToF） | 30 Hz | `SF/main/config.hpp:136` |
| 既存の速度観測（フロー） | 100 Hz | `SF/main/config.hpp:133` |
| **UWB (a) 現状** | **31 Hz** | §2.2 |

- **UWB 31 Hz は位置ループ帯域 0.064 Hz の約 480倍。** ナイキストの観点で全く問題ない
- **既に成立している高度ループ（ToF 30 Hz）とほぼ同じレート。**
  30 Hz の ToF で `±6cm` の高度保持が成立している以上、31 Hz の水平位置観測でレートが不足する
  理由は無い
- (b)(c) への高速化は**位置制御の要求からは不要**。高速化が効くのは
  「高速飛行時の周内スミア低減」（§3.3）と「欠測に対する冗長性」だけ

**→ レートは統合の律速ではない。律速は精度とバイアス。**

## 3.2 400 Hz との差はどう埋まるか → **既存の仕組みがそのまま使える**

UWB 31 Hz と制御 400 Hz の間は、**ESKF の predict ステップ（IMU 400 Hz）が既に埋めている**。
ToF 30 Hz / フロー 100 Hz と全く同じ構図:

```
400Hz: predict(imu, 2.5ms)   ← IMU で常に前進（デッドレコニング）
 31Hz:   ↳ updateUwb(...)    ← 観測が来たときだけ補正
400Hz: estimate_state.publish → ControlTask は常に 400Hz の推定値を受け取る
```

**新しい補間機構は要らない。** `processAsyncSensors()` に1本足すだけ（§1.4）。

## 3.3 レイテンシ — 制御にどう効くか

### 遅延の内訳（(a) 現状構成）

| 要素 | 値 | 根拠 |
|---|---:|---|
| ① 周内スミア（1周 32 ms の中央からの偏差） | **±16 ms** | 5リンクが 32 ms に分散。ソルバは「同時刻」と仮定する |
| ② 測位計算 | 0.2〜2 ms（推定） | §2.3 |
| ③ Topic → `processAsyncSensors()` | 0〜2.5 ms | ImuTask 400 Hz が次に回るまで |
| ④ ESKF 適用 → publish → ControlTask | 0〜2.5 ms | ImuTask 内で完結後、通知 |
| **合計（周中央基準）** | **≈ 16〜23 ms** | |

### 制御への影響

**(i) 位置ループへの位相遅れ → 無視できる**

位置ループのクロスオーバ ω ≈ 0.4 rad/s、遅延 20 ms とすると
位相遅れ = 0.4 × 0.020 = 0.008 rad = **0.46°**。位相余裕への影響は皆無。

**(ii) 周内スミアによる測距誤差 → 速度が上がると効く**

タグが速度 v で動くと、周の端で取った測距は最大 16 ms 分の移動量だけずれる:

| 速度 | 16 ms の移動量 | 評価 |
|---:|---:|---|
| 0.1 m/s（ホバリング時のふらつき） | 1.6 mm | 無視できる |
| 1.0 m/s（`max_pos_vel_` 上限） | **16 mm** | ホールド精度 RMS 16mm と同オーダー。無視できない |
| 3.0 m/s | 48 mm | 明確に効く |

対処（優先順）:
1. **各測距に個別のタイムスタンプを付け、周の中央時刻を fix のタイムスタンプにする**
   → 平均的なバイアスは消え、残差はランダム化される
2. ESKF の速度推定でスミアを補正してから解く
3. **密結合（案A）にして各測距を自分の時刻で融合する** → 原理的に解決（§4.3）

**(iii) UWB 復帰時のジャンプ → これが最大のリスク**

- ESKF に遅延補償が無い（§1.5）ため、20 ms 古い観測が「今の値」として入る
- 通常運転では上記の通り無害だが、**UWB が数秒落ちて復帰したときが危険**:
  その間フロー由来の水平位置がドリフトしているので、復帰時のイノベーションが数十cm〜数mになる
- **`PidController` の POS_HOLD 目標値 `pos_setpoint_x_/y_` は、モード進入時に
  `state.position[0]/[1]` から捕捉されたきり動かない**
  （`SF/components/sf_controller_pid/pid_controller.cpp:1317-1323`）。
  推定位置だけがジャンプすると、機体は「位置誤差ができた」と判断して
  **最大 1.0 m/s（`max_pos_vel_`）で飛び出す**

→ 対策は §4.5 と §7 に記載。**この1点は必ず設計で潰すこと。**

---

# 4. 統合方式の検討

## 4.0 まず：UWB を入れる目的を取り違えないこと

`SF/docs/poshold_journey.md:5,180-184` の事実:

| 現状（フロー + ToF + IMU） | 値 |
|---|---|
| 定点保持精度 | **±6〜7 cm、RMS 16 mm** |
| ドリフト RMS | 16 mm |
| 結論 | 「±6〜7cm がこの37g機のトルク効きでの実用限界」 |

一方 UWB の精度:

| 指標 | 値 | 出典 |
|---|---|---|
| M5Stack 自社測定（DS-TWR） | 約 **0.14 m** | `docs/archive/SURVEY_m5stamp_uwb_module.md:109` |
| **無校正時の定常バイアス** | **数十cm〜1m超** | `docs/archive/CRITICAL_REVIEW.md:130-132` |
| アンテナ遅延校正後（APS014 実績） | 3σ ≈ 4.5 cm | `docs/archive/REIMPL_PLAN.md:199` |

**→ UWB を「そのまま位置ループに入れれば精度が上がる」のは誤り。校正前は確実に悪化する。**

UWB の価値は別のところにある:

| 得られるもの | 現状の限界 |
|---|---|
| **絶対座標（部屋座標系）での位置** | フローは相対。原点も向きも無い |
| **時間無制限の無ドリフト** | フローは速度観測のみ → x,y は積分でドリフト（§1.3） |
| **4 m を超える高度** | ToF は DistanceMode=LONG で最大 4 m（`SF/components/sf_hal_vl53l3cx/include/vl53l3cx_wrapper.hpp:112`） |
| **床が無地・暗所でも動く** | PMW3901 は床のテクスチャと明るさに依存（SQUAL ゲート `eskf.gate.flow_squal=10`、`params.cpp:808`） |
| **ウェイポイント飛行** | `GuidanceTarget`（`data_types.hpp:699-716`）は既に用意されているが絶対座標が無い |

**この整理が、以下の案の評価軸を決める。**

## 4.1 前提：`uwb_loc` の Lv2 と Lv3 の違い（二重フィルタ問題の核心）

| レベル | 性質 | API |
|---|---|---|
| **Lv0** | 閉形式三辺測量。無状態 | `uwb_solve_lv0()`（`components/uwb_loc/include/uwb_loc.h:154`） |
| **Lv2** | Beck GTRS 厳密解 + Huber ロバスト化。**無状態の純関数**（1周のスナップショットだけで解く） | `uwb_solve_lv2()`（同 `:163`） |
| **Lv3** | **密結合 EKF。内部に状態 x/P を持つ** | `uwb_ekf_init/predict/update`（同 `:208,214,218`）。状態は `UWB_MOTION_CV`(6次元) または `UWB_MOTION_CA`(9次元)（同 `:175-177`） |

**二重フィルタ問題は「Lv3 を使ったときだけ」発生する。**
Lv3 は自前の運動モデル（CV/CA）で位置・速度・加速度を平滑化するので、
その出力を StampFly の ESKF に入れると:

- **同じ観測が2回平滑化される** → 出力は「実際より滑らかで、実際より古い」
- **誤差が時間相関を持つ** → ESKF の白色雑音仮定が破れ、共分散が過信になる
- 2つの運動モデル（Lv3 の CV/CA と ESKF の IMU 予測）が競合し、どちらのプロセスノイズを
  触ればいいのか分からなくなる

**→ Lv2（無状態）を使えば、この問題は原理的に発生しない。**
Lv2 は「1周分の測距 → 位置」の写像に過ぎず、フィルタではない。

## 4.2 案B：UWB 側で位置まで出し、StampFly には位置だけ渡す（疎結合）

### B-1（Lv3 を使う） — **不採用**

上記の二重フィルタ問題が全部乗る。加えて `PositioningPipeline` のコメントが
「**uwb_ekf はスレッド安全でない**。必ず単一タスクから呼ぶこと」
（`components/uwb_ranging/include/uwb_ranging_pipeline.hpp:52-55`）と明記しており、
StampFly のマルチタスク環境では制約が増える。

### B-2（Lv2 を使う） — **推奨の出発点**

```
[UwbTask, core0]                            [ImuTask, core1, 400Hz]
 RangingScheduler::runCycle()  32ms
   → RangingSample[5]
   → PositioningPipeline::solve(..., Lv2)   ── 無状態
   → PositionFix{p[3], sigma, gdop, nUsed, t_us}
   → sf::sensor_uwb.publish()  (Queue,2)  ──→ processAsyncSensors()
                                                → g_estimator->updateUwb(fix)
                                                  → EskfCore::updateUwbPosition()
                                                    → vectorUpdate3()  (POS_X/Y[/Z])
```

| 評価軸 | 評価 |
|---|---|
| 実装量 | **小**。`uwb_loc`/`uwb_ranging` は無改造。ESKF に関数1本、Topic 1本、タスク1本 |
| 既存コードへの侵襲度 | **小**。`processAsyncSensors()` に4行、`IEstimator` に純粋仮想1本、`EskfCore` に約25行 |
| 精度 | **中**。Lv2 は全アンカーの幾何を使った最適解。ただし「4本以上揃った周」でしか解が出ない |
| レイテンシ | 16〜23 ms（§3.3） |
| 失敗時の挙動 | **最良**。観測が来ないだけ → **現在のフロー単独動作にそのまま縮退する** |
| 二重フィルタ | **無し**（Lv2 は無状態） |
| 部分観測の活用 | **できない**。3本しか取れなかった周は丸ごと捨てる（`PositioningConfig::minValid3d=4`、`uwb_ranging_types.hpp:78`） |
| 周内スミア | 残る（§3.3-ii） |

### 既存の受け皿がぴったり合う点

`EskfCore::vectorUpdate3(const float H[3][N], const float innov[3], float R, float chi2_gate)`
（`SF/components/sf_estimator_eskf/include/eskf_core.hpp:330`、実装 `eskf_core.cpp:380`）は

- **H の列疎性を利用する実装**（`eskf_core.cpp:384-393` のコメント）。
  UWB 位置観測の H は POS_X/Y/Z の3列だけが非ゼロ → まさに想定されている使い方
- **χ² ゲートを内蔵**している → 外れ値の除去が最初から付いてくる

つまり **UWB 3D 位置観測は、既存 API に何も足さずそのまま入る形をしている。**

## 4.3 案A：UWB を ESKF の観測として直接入れる（密結合）

### A-1：位置を観測にする

→ これは実質 **案B-2 と同じ**（Lv2 で位置を作ってから入れる）。「密結合」と呼ぶ必要はない。

### A-2：各アンカーまでの距離をそのまま観測にする（真の密結合）

`EskfCore` に以下を足す:

```cpp
// H = [ (p̂ - a)/‖p̂ - a‖ , 0 ... 0 ]   （先頭3成分のみ非ゼロ）
// innovation = r_meas - ‖p̂ - a‖
void EskfCore::updateUwbRange(const Vec3& anchor, float range, float sigma);
   → scalarUpdate(H, innovation, sigma*sigma);
```

`scalarUpdate(const float H[N], float innovation, float R)`
（`eskf_core.hpp:327`、実装 `eskf_core.cpp:319`）は
`updateToF` が使っているのと同じ関数（`eskf_core.cpp:556`）。
**コード量は `updateToF`（`eskf_core.cpp:530-556`、27行）とほぼ同じ。**

| 評価軸 | 評価 |
|---|---|
| 実装量 | **中**。ESKF は小さいが、**機体側にアンカー座標テーブル + アンテナ遅延校正値の永続化**が必要 |
| 既存コードへの侵襲度 | 中。ESKF 本体 + NVS パラメータ + CLI コマンド |
| 精度 | **最良**。1本でも観測が入れば拘束になる（3本以下の周も使える）。各測距を自分の時刻で融合できるので**周内スミアが原理的に消える** |
| レイテンシ | **最小**（測距ごとに即融合すれば 2.5〜5 ms） |
| 失敗時の挙動 | 良い（観測が来ないだけ） |
| 二重フィルタ | 無し（`uwb_loc` を通さない） |
| **危険** | **初期値依存**。距離のみの EKF は初期位置が悪いと**鏡像解に収束しうる**。Lv2/Beck が持つ大域最適性（閉形式）を捨てることになる |
| **危険** | NLOS/マルチパスの外れ値除去が ESKF の χ² ゲート任せになる。**`SF/docs/chi2_latchup_finding.md` 等3本の文書が示す通り、このプロジェクトは χ² ゲートのラッチアップ問題を既に経験している** |
| **危険** | GDOP・冗長度・同一平面判定（`AnchorTable::checkPlacement()`、`uwb_ranging_anchor_table.hpp:98`）という `uwb_loc` 側の安全機構が全部使われなくなる |

**→ 精度は最良だが、飛行安全に直結するコードに最初から入れるには危険が多すぎる。**

## 4.4 案C：UWB を別タスクで回し、既存の推定器に「補正」として注入

`IEstimator` には位置を外から書き換える口が `resetPositionVelocity()`（`estimator.hpp:135`）
しか無い。これを定期的に呼ぶ、あるいは `pos_` を直接叩くという方式。

| 評価軸 | 評価 |
|---|---|
| 実装量 | 小 |
| 既存コードへの侵襲度 | 小（見かけ上） |
| 精度 | **悪い**。共分散を経由しないので、UWB の不確かさと IMU/フローの不確かさが重み付けされない |
| レイテンシ | 小 |
| 失敗時の挙動 | **最悪**。補正が「ジャンプ」として入り、§3.3-iii のフライアウェイ機構をそのまま踏む |
| 一貫性 | **破綻**。`P_` と `pos_` が整合しなくなり、以降の ESKF 更新の重みが全部おかしくなる |

**→ 不採用。** ESKF がある環境でフィルタの外から状態を書き換えるのは、
共分散を持つ推定器の意味を失わせる。

## 4.5 比較まとめと推奨

| | 案A-2（距離を密結合） | **案B-2（Lv2 → 位置観測）** | 案B-1（Lv3 → 位置観測） | 案C（外から補正） |
|---|---|---|---|---|
| 実装量 | 中 | **小** | 小 | 小 |
| 侵襲度 | 中 | **小** | 小 | 小 |
| 精度 | ◎ | ○ | △（二重平滑化） | × |
| レイテンシ | ◎ 2.5〜5ms | ○ 16〜23ms | △ さらに悪化 | ○ |
| 周内スミア | **無し** | 残る | 残る | 残る |
| 部分観測（<4本）の活用 | **可** | 不可 | 不可 | 不可 |
| 二重フィルタ | 無し | **無し** | **有り** | — |
| 大域最適性（鏡像解回避） | **無し** | 有り（Beck） | 有り | — |
| 外れ値除去 | ESKF の χ² のみ | Huber + χ² + GDOP | 同左 | 無し |
| **UWB 喪失時** | 縮退 | **現状動作に完全縮退** | 縮退 | 危険 |
| **初回導入の安全性** | △ | **◎** | ○ | × |

### 推奨：**案B-2 から始め、実測に基づいて案A-2 へ段階移行する**

**Phase I（推奨の出発点）: 案B-2**

理由:
1. **失敗時に現状へ完全縮退する。** UWB が落ちても、機体は「今まさに ±6〜7cm で飛べている
   フロー + ToF + IMU 構成」そのものに戻る。飛行安全上、これが決定的
2. **`uwb_loc` / `uwb_ranging` を一切改造しない。** 上流（`kouhei1970/uwb_localizer`）への
   追従性が保たれる（`docs/PERF_ANALYSIS.md:186-191` が再 vendoring の必要性を指摘している）
3. **Lv2 は無状態なので二重フィルタにならない。** Lv3 は使わない
4. **既存の `vectorUpdate3()` にそのまま乗る。** χ² ゲートも列疎性最適化も既にある
5. Lv2 の `gdop` / `sigma` / `nUsed` / `excluded`（`uwb_ranging_types.hpp:110-131`）を
   そのまま観測ノイズ R に反映できる → **品質に応じた重み付けが最初から入る**
6. **レートは §3.1 の通り (a) の 31 Hz で十分すぎる。** 高速化（R5/R6）を待つ必要がない

**Phase II（Phase I が飛んでから）: 案A-2 へ移行**

移行の判断材料になるのは:
- 周内スミア（§3.3-ii）が実測で問題になったか（＝高速飛行をしたいか）
- 「4本揃わない周」の割合が実測でどれだけあったか（＝部屋の遮蔽が厳しいか）

移行時の設計:
- `updateUwbRange()`（scalarUpdate）を主経路にする
- **Lv2 は捨てず、初期化・再捕捉のときだけ使う**（大域最適な初期値を ESKF に種付けする）
- **Lv3（`uwb_ekf`）は最後まで使わない**

### R（観測ノイズ）の設定方針 — ここを間違えると悪化する

**UWB の σ ≈ 0.10〜0.15 m に対し、現状のフロー由来の短期精度は RMS 16 mm。**
R を小さく設定すると、ESKF が UWB を信用して**短期精度が悪化する**。

- `eskf.obs.uwb_noise` は**保守的に（≥ 0.15 m、校正前は ≥ 0.5 m）**設定する
- 既存の観測ノイズと比較して桁を合わせること:
  `eskf.obs.tof_noise = 0.01`, `eskf.obs.flow_noise = 0.30`（`SF/components/sf_core/params.cpp:790-791`）
- 狙いは「**UWB が長周期の DC ドリフトだけを直し、短周期のダイナミクスはフロー + IMU に任せる**」

### 役割分担（重要）

**UWB は既存センサを置き換えない。**

| 状態 | 主観測 | UWB の役割 |
|---|---|---|
| POS_Z（高度） | **ToF（30 Hz, σ=0.01 m）を維持** | 高度 4 m 超のときだけ補助。**同一平面アンカー配置では UWB の z は GDOP が悪い**（`docs/ANCHOR_PLACEMENT.md`、`AnchorTable::checkPlacement()`） |
| VEL_X/VEL_Y | **フロー（100 Hz）を維持** | 関与しない |
| **POS_X/POS_Y** | **無し（デッドレコニング）** | **← ここを埋めるのが UWB の仕事** |

初期実装では **UWB は POS_X / POS_Y のみを拘束**し、z は使わないことを推奨する
（`H` の POS_Z 行を落とせば `vectorUpdate3` ではなく 2回の `scalarUpdate` でも実装できる）。

## 4.6 実装に必要な変更点（案B-2の場合）

### 本リポジトリ側（`m5stamp_uwb_localizer`）— 現状の欠落

| # | 欠落 | 該当 | 対処 |
|---|---|---|---|
| 1 | ~~**`RangingSample` に絶対タイムスタンプが無い**~~ | **解決済 (2026-08-21)**: `int64_t t_us` を追加（`components/uwb_ranging/include/uwb_ranging_types.hpp:62`）。測距開始の直前に `esp_timer_get_time()` を刻む | — |
| 2 | **`PositionResult` にタイムスタンプが無い** | 同 `:110-131` | 周の中央時刻を持たせる |
| 3 | `Config::port_already_initialized` は既にある | `components/uwb_qm33120/include/uwb_qm33120_types.hpp:93` | **StampFly 統合を見越して用意済み**。`sf_board` が SPI を初期化する構成にそのまま乗る |
| 4 | ~~IRQ 未使用（R6 未実装）~~ | **解決済 (2026-08-21)**: アンカー側の IRQ 経路は実装済み（`UWB_ENABLE_IRQ`、既定は無効・極性は実機未検証）。**タグ側は方針として IRQ 非依存のまま**（`docs/IRQ_POLICY.md`） | ポーリング経路は第一級のまま残してある |

### StampFly 側（`firmware/vehicle`）— 追加が必要なもの

| # | ファイル | 変更 |
|---|---|---|
| 1 | `components/sf_core/include/data_types.hpp` | `struct UwbFix { float position[3]; float sigma; float gdop; uint8_t n_used; uint32_t timestamp; }` |
| 2 | `components/sf_core/include/topics.hpp` | `extern Topic<UwbFix, Queue, 2> sensor_uwb;`（`:39-42` の低レートセンサと同じ流儀） |
| 3 | `components/sf_estimator/include/estimator.hpp` | `virtual void updateUwb(const UwbFix&) {}` — **純粋仮想にせず既定 no-op にする**（相補フィルタ側を触らずに済む） |
| 4 | `components/sf_estimator_eskf/include/eskf_core.hpp` + `.cpp` | `updateUwbPosition(const Vec3& p, float sigma)` → `vectorUpdate3()`。`updateToF`（`eskf_core.cpp:530-556`）と同型 |
| 5 | `components/sf_core/params.cpp` | `eskf.use_uwb`(既定 0), `eskf.obs.uwb_noise`, `eskf.gate.uwb_innov` を `:790-808` の並びに追加 |
| 6 | `tasks/imu_task.cpp` | `processAsyncSensors()`（`:248`）に `while (sf::sensor_uwb.read(fix)) g_estimator->updateUwb(fix);` |
| 7 | `components/sf_hal_uwb_qm33120/`（新規） | 本リポジトリの `uwb_*` コンポーネントを取り込む C++ ラッパ。命名は `sf_hal_<chip>` 流儀（`docs/archive/SURVEY_stampfly_ecosystem.md:39`） |
| 8 | `tasks/uwb_task.cpp`（新規）+ `tasks/tasks.hpp` / `tasks.cpp` | 測距ループ。コア0（§5.4） |
| 9 | `main/config.hpp` | `PRIORITY_UWB` / `STACK_UWB` / GPIO 定義 |

**⚠ `third_party/` は読み取り専用。上記は「統合するならこうなる」という設計であり、
本検討では一切実装していない。**

---

# 5. ハードウェア構成

## 5.1 【重要な訂正】`docs/archive/SURVEY_stampfly_grove.md` の「空きGPIO」は誤り

`docs/archive/SURVEY_stampfly_grove.md:25-29` は **G5 / G10 / G41 / G42 を「未使用（＝空き）」**と
記載し、`docs/PLAN.md:253` の R3 と `docs/archive/SURVEY_stampfly_grove.md:70` の代替案2 が
これに依拠している。

**現行ファームの実コードでは、この4本はすべてモータ PWM 出力である:**

```
SF/main/config.hpp:63:  GPIO_MOTOR_M1 = 42;  // FR, CCW
SF/main/config.hpp:64:  GPIO_MOTOR_M2 = 41;  // RR, CW
SF/main/config.hpp:65:  GPIO_MOTOR_M3 = 10;  // RL, CCW
SF/main/config.hpp:66:  GPIO_MOTOR_M4 = 5;   // FL, CW
```

**→ `archive/SURVEY_stampfly_grove.md` の「代替案2（空きGPIOを使う）」は成立しない。**
旧調査はモータのピン割当を見ていなかった（旧調査の対象は `M5StampFly` / `stampfly_hal` であり、
`firmware/vehicle` ではない）。

## 5.2 現行ファームでの GPIO 使用状況（実コード確認済み）

`SF/main/config.hpp:45-73` から:

| GPIO | 用途 | file:line |
|---:|---|---|
| 0 | ボタン | `config.hpp:73` |
| 3 / 4 | I2C SDA / SCL | `config.hpp:53-54` |
| **5** | **モータ M4** | `config.hpp:66` |
| 7 | ToF XSHUT（底面） | `config.hpp:58` |
| 9 | ToF XSHUT（前方、恒久リセット保持） | `config.hpp:59`, `SF/tasks/tof_task.cpp:96-105` |
| **10** | **モータ M3** | `config.hpp:65` |
| 12 | PMW3901 CS | `config.hpp:49` |
| 14 / 43 / 44 | SPI MOSI / MISO / SCK | `config.hpp:45-47` |
| 21 | M5StampS3A 内蔵 LED | `config.hpp:70` |
| 39 | 機体 LED | `config.hpp:71` |
| 40 | ブザー | `config.hpp:72` |
| **41 / 42** | **モータ M2 / M1** | `config.hpp:64,63` |
| 46 | BMI270 CS | `config.hpp:48` |

**現行ファームで未使用の GPIO**（`GPIO_NUM_n` を `main/` `components/` `tasks/` で grep して0件）:

| GPIO | 物理的な素性 | 使えるか |
|---:|---|---|
| **1 / 2** | **GROVE(黒/UART) RX/TX** | **○ コネクタに出ている** |
| **13 / 15** | **GROVE(赤/I2C) SDA/SCL** | **○ コネクタに出ている** |
| 6 / 8 | ToF INT（底面/前方） | **× VL53L3CX IC へ配線済み。コネクタに出ていない** |
| 11 | BMI270 INT1 | **× BMI270 へ配線済み** |

出典: GROVE / ToF INT / BMI270 INT の配線は
`SF/components/sf_hal_bmi270/docs/M5StamFly_spec_ja.md:112-140`（同一ファイルが
`sf_hal_vl53l3cx/docs/` にも重複配置）。

**→ 外部に取り出せる信号線は GROVE 2系統の 4本（G13, G15, G1, G2）だけ。**

## 5.3 接続案の比較

M5Stamp UWB Module のホスト側必要信号:
**最低4本（SCK/MOSI/MISO/CS）、推奨 +2（IRQ/RSTn）、省電力なら +1（WAKEUP）**
（`docs/archive/SURVEY_m5stamp_uwb_module.md:91-92`）。

### HW-1: GROVE 2系統を4本すべて SPI に使う（`archive/SURVEY_stampfly_grove.md:64` の代替案1）【廃案】

**2026-08-22 廃案。** RST/IRQ/WAKEUP が一切取れず、GROVE がバッテリ電圧直結（満充電約4.35V >
QM33120W の絶対最大定格 4.0V）で LDO が必須、2系統にまたがるためカスタムケーブルが必須、
GROVE を使い切ってしまう。**HW-4（背面 12P FPC。§5.3 末尾）はこれらを全て解消する**
（電源は背面 FPC の VDD_3V3 から直接取れるため降圧回路そのものが不要）。以下は当時の
検討記録として残す（`docs/README.md` 規則6「訂正は消さずに残す」）。

```
G13 → SCK,  G15 → MOSI,  G1 → MISO,  G2 → CS    （SPI3_HOST、GPIO マトリクス経由）
IRQ / RSTn / WAKEUP  → 無し
```

| 項目 | 評価 |
|---|---|
| StampFly への半田付け | **不要**（カスタムケーブルのみ） |
| SPI ホスト | **SPI3_HOST が空いている**（`docs/archive/SURVEY_stampfly_ecosystem.md:65`）→ 飛行系の SPI2 と完全分離 |
| GPIO マトリクス | 16MHz なら問題なし（`archive/SURVEY_stampfly_grove.md:56-58`） |
| **IRQ** | **取れない → R6 不可 → §2.2 の (c) は到達不能。Tag 側は永久にポーリング** |
| **RSTn** | 取れない → `hard_reset_on_begin`（`uwb_qm33120_types.hpp:83`）が使えない。ソフトリセット (`dwt_softreset`) だけで復旧できるか**要検証** |
| GROVE 拡張性 | **I2C 拡張・UART 拡張の両方を潰す** |
| 配線 | 2コネクタが基板上の別位置 → 標準ケーブル1本では不可。**カスタムケーブル必須** |
| 電源 | StampFly の GROVE は電池直結（満充電約 4.35V、プロジェクト設計者による実機確認 2026-08-21）で **QM33120W の絶対最大定格 4.0V を超える**。パッド 2 はチップ直結（`docs/WIRING.md` §5.4）のため → **LDO 必須**（または基板の 3.3V レールから取る） |

### HW-2: SPI2_HOST に相乗り + GROVE を CS / IRQ / RST に使う【廃案】

**2026-08-22 廃案。** 飛行制御の生命線である BMI270 と SPI バスを共有するリスクが残っていた
（下表「リスク」行）。**HW-4（背面 12P FPC。§5.3 末尾）は G16/G17/G18/G33-G37 という
StampFly が1本も使っていない専有線を使うため、バス共有が無い。** 以下は当時の検討記録
として残す（`docs/README.md` 規則6「訂正は消さずに残す」）。

```
SCK/MOSI/MISO → M5StampS3A のパッド or 基板上の SPI2 配線から分岐（G44/G14/G43）
G13 → CS,  G15 → IRQ,  G1 → RSTn,  G2 → WAKEUP（予備）
```

| 項目 | 評価 |
|---|---|
| **IRQ** | **取れる → R6 が Tag 側でも効く → §2.2 の (c1) 90 Hz が射程に入る** |
| RSTn / WAKEUP | 取れる |
| StampFly への半田付け | **必要**（M5StampS3A のキャステレーション or 基板上の配線に飛び線） |
| SPI バス共有 | BMI270 (10MHz, `SF/components/sf_board/board.cpp` 経由) と PMW3901 (2MHz) と同居 |
| バス共有のレイテンシ影響 | **問題にならない見込み**。BMI270 の 1バースト ≈ 16µs、PMW3901 ≈ 48µs（推定）に対し TWR の折返し予算は最小でも 276 µs（§2.1） |
| バス初期化 | `sf_board` が `spi_bus_initialize()` を1回だけ実行し、各 HAL は `spi_bus_add_device()` のみ（`SF/components/sf_board/board.cpp:285-296`）。**本リポジトリの `Config::port_already_initialized`（`uwb_qm33120_types.hpp:93`）がまさにこの構成用に用意されている** |
| **リスク** | **飛行制御の生命線（IMU）と同じバスを共有する。** UWB ドライバがバスをハングさせると IMU も止まる |
| 実現性 | **M5StampS3A のパッドに物理的にアクセスできるか未確認**（StampFly 基板に実装済みのモジュールの側面キャステレーションに半田付けできるか） |

### HW-3: 空き GPIO を使う → **不可能**（§5.1）

### HW-4: M5StampS3A 背面の 12P FPC を使う（2026-08-22 決定・採用）

**HW-1・HW-2 に代わって採用された経路。** M5StampS3A は背面に TFT 用の FPC インタフェースを
予約しており、この 12P コネクタ（出荷時は未実装、後付けが要る）に UWB モジュールの SPI4本 +
RSTn + IRQ + WAKEUP + 電源をすべて割り当てる。根拠の全文は `boards/stampfly.h` 冒頭コメント
（一次資料: M5Stack 公式回路図 `assets/Sch_StampS3_v0.3.3.pdf`）。要約:

```
SCK    → G36（背面FPC 位置10 / DISP_SCK）
MOSI   → G35（位置9 / DISP_MOSI）
MISO   → G34（位置8 / DISP_RS を転用）
CS     → G37（位置12 / DISP_CS。ソフトウェア制御）
RSTn   → G33（位置7 / DISP_RST）
IRQ    → G16（位置4）
WAKEUP → G17（位置3）
電源   → VDD_3V3（位置11）。BL_3V3（位置5）は使わない
spi_host = SPI3_HOST（飛行系の SPI2_HOST とは別バス）
```

| 項目 | 評価 |
|---|---|
| StampFly への半田付け | **必要**（背面の 0.5mm 12P コネクタ HDGC/0.5K-HX-12PWB を後付け。8P 版では GPIO が5本 (G33-G37) しか取れないため 12P 版が必須） |
| SPI ホスト | **SPI3_HOST**。飛行系(BMI270/PMW3901)が使う SPI2_HOST(G14/G43/G44/G46)とは別バスで、**一切共有しない** |
| **IRQ** | **取れる → R6 が Tag 側でも効く → `uwb::TimingProfile::BothIrq`（90Hz）が初めて成立する** |
| **RSTn / WAKEUP** | **両方取れる**。`hard_reset_on_begin` が使える |
| GPIO 本数 | **8本**（G16/G17/G18/G33-G37）。うち G16/G17/G18 は M5StampS3A 側面23ピンに出ておらず、この背面FPCでしか掴めない完全な空きGPIOで、StampFly の飛行制御ファームも使っていない |
| GROVE への影響 | **一切使わない**。GROVE 2系統は丸ごと空く |
| 電源 | **VDD_3V3（位置11）から直接取れる。降圧回路そのものが不要**（BL_3V3(位置5)はロードスイッチ AW35122FDR(U2)の出力で G38 の状態に従属するため使わない） |
| バス共有リスク | **無い**（G16/G17/G18/G33-G37 は StampFly が1本も使っていない専有線） |
| 1番パッドの特定 | 実物でテスターにより一度だけ確定する（目的のパッドと M5StampS3A 側面の 5V パッド／3V3 パッドとの導通で VIN_5V 端・VDD_3V3 端が一意に決まる） |

### 推奨

**HW-4 を採用する（2026-08-22 決定）。** 地上検証・飛行統合を問わず HW-4 で統一する
（HW-1 のような「地上検証だけ半田付け不要」という使い分けは無くなった。背面 FPC コネクタの
後付け半田付け自体は地上検証の段階から必要）。

RSTn/IRQ/WAKEUP が最初から全て取れるため、(a) 31Hz → `TimingProfile::BothIrq` 適用で
(c1) 90Hz が初めて成立する。**ただし §3.1 の通りレート面の必然性は無い。** IRQ の本当の
価値は「折返し時間が縮む → SS-TWR のクロックオフセット誤差が減る」「タイミングマージンが
増えて成功率が上がる」の方である（既定は `PollingBoth` のままで、実機で Phase 1〜2 が通って
から `AnchorIrq` → `BothIrq` と段階的に上げる。`docs/EXPERIMENT_PLAN.md`）。

**⚠ IRQ が取れるようになっても、ポーリング経路は必ず残すこと**
（`docs/archive/REIMPL_PLAN.md:157`、`docs/PLAN.md:138`）。理由が「StampFly が IRQ を
取れないから」から「IRQ の極性が実機で未検証だから」に変わっただけで、結論は変わらない
（`boards/stampfly.h`）。

## 5.4 タスク配置（`sf_board` / FreeRTOS の制約）

### コアの選択 → **コア0 一択**

- コア1 は ImuTask (400Hz, prio 24) + ControlTask (prio 23) の2つだけ（`SF/tasks/tasks.cpp:44-63`）
- コア1 に置くと、2.5 ms ごとに IMU が割り込んで **TWR の遅延送信締切（最小 276 µs、
  ポーリングでも 1226 µs）を踏み抜く**
- コア0 には既に14タスクがいる（§1.1）

### 優先度の選択 → **低優先度から始めて実測で上げる**

**鍵になる事実: TWR の締切を逃しても壊れない。**
`dwt_starttx(DWT_START_TX_DELAYED)` が `DWT_ERROR` を返し、その1リンクが失敗するだけ
（`components/uwb_qm33120/src/uwb_qm33120_twr.cpp:469-474`）。
→ 高優先度化は「成功率のため」であって「安全のため」ではない。

| 優先度 | 影響 | 判断 |
|---:|---|---|
| 24〜23 | コア1 専用帯。使わない | — |
| 22 | StateTask（安全 FSM）と同格 | **禁止**。安全 FSM を飢餓させうる |
| **21** | OPTFLOW(20) 以下すべてを preempt できる | 成功率は上がるが FlowTask(100Hz) を圧迫。**最後の手段** |
| 20〜13 | フロー/地磁気/気圧/通信/ToF と競合 | 中間 |
| **11** | Power(12) より下、Button(10) より上 | **ここから始める** |

**手順**: `PRIORITY_UWB = 11` で始め、`RangingScheduler::stats()`（`uwb_ranging_scheduler.hpp`）の
成功率と `lastCycleMs()` を実測し、不足なら段階的に上げる。**22 は超えない。**

補足: 測距ループは `vTaskDelay(pdMS_TO_TICKS(1))` で必ず yield する
（`uwb_qm33120_twr.cpp:429,516` など）ので、32 ms 間 CPU を占有し続けることはない。

### スタック

`uwb_loc` はスタック上限 6.6 KB（`docs/PERF_ANALYSIS.md:152`。最適化済み上流を取り込めば
約 3.1 KB 削減）。**`STACK_UWB = 8192` 以上**を推奨（`SF/main/config.hpp:107-122` の並びに合わせる）。

## 5.5 電源

| 項目 | 値 | 出典 |
|---|---|---|
| モジュール電源 | **公式仕様 3.3V**。パッド 2 は QM33120W の電源レールに直結（動作 2.4〜3.6V、絶対最大定格 4.0V）。モジュール上の DC-DC（`JW5712`、入力 2.7〜5.5V）は内部 1.8V 系用で入力上限は広げない。**5V 不可** | `docs/refs/QM33120W_DS.txt:553,567-569`、実物確認（2026-08-21） |
| 消費電流（タグ動作） | **58.0 mA @3.3V** | `docs/archive/SURVEY_m5stamp_uwb_module.md:100` |
| 消費電流（アンカー動作 / スリープ） | 5.23 mA / 75.9 µA | 同 `:100` |
| TX 瞬時ピーク | **未公開** | 同 `:101` |
| GROVE の供給 | **StampFly は電池直結**（満充電約 4.35V。絶対最大定格 4.0V を超えるため直結不可）。M5Stack 製品一般の GROVE は 5V | プロジェクト設計者による実機確認（2026-08-21）、`docs/archive/SURVEY_stampfly_grove.md:47`（一般 GROVE の 5V 部分） |
| GROVE の供給電流定格 | **未公開** | 同 `:49` |
| StampFly の 3.3V レギュレータ余裕 | **文書に一切記載なし**（`stampfly_ecosystem` 全体を検索して該当なし） | 未解決 |
| バッテリ低電圧閾値 | 3.4 V（`safety.battery.low_v`） | `SF/components/sf_core/params.cpp:825` |
| 電圧監視 | INA3221 ch1、PowerTask 10 Hz | `SF/main/config.hpp:302-311`, `SF/tasks/power_task.cpp:158` |

**未解決の課題:**
1. StampFly の GROVE（電池電圧、満充電約 4.35V）は QM33120W の絶対最大定格 4.0V を超えるため
   **直結不可・LDO 必須**（重量・スペース増）。基板上の 3.3V レールから取れば LDO は不要だが、
   **その電流余裕が文書化されていない**
2. **TX 瞬時ピーク電流が不明。** 3.3V レールが瞬間的に落ちると **IMU / ToF まで巻き添えになる**
   → デカップリング（数十〜100 µF）を UWB モジュール直近に入れることを推奨
3. 58 mA は 4モータの消費（アンペアオーダー）に比べれば小さいが、**飛行時間には効く**

## 5.6 重量・搭載位置

| 項目 | 値 | 出典 |
|---|---:|---|
| StampFly 全備重量 | **37 g** | `SF/docs/poshold_journey.md:5` |
| M5Stamp UWB Module (S017) | **0.5 g** | `docs/WIRING.md:25` |
| M5Stamp UWB Module (S017-F、FPC コネクタ実装済) | 0.6 g | 同 |
| モジュール外形 | 11.5 × 12.0 × 1.6 mm（S017）/ ×2.8 mm（S017-F） | 同 `:24` |
| 配線 + LDO + 固定具（推定） | **+1.5〜3.5 g** | 未実測 |
| **追加重量合計（推定）** | **2〜4 g = 全備の 5〜11%** | |

**【2026-08-22 追記】接続経路が HW-4（背面 12P FPC + 専用基板方式）に変わったため、
上表の内訳はもう前提が合わない。** LDO は不要になった（VDD_3V3 直結のため）一方、
モジュールを固定・配線するための専用基板が新たに要る。**専用基板の設計は未着手**であり、
重量を推測で書くと根拠のない数字が独り歩きするため、**現時点では再見積もりしない**。
上表の「2〜4g」は HW-1/HW-2（LDO 前提）当時の推定値として、参考のためそのまま残す。

### 重量増が引き起こす問題 — **見落としやすい**

`SF/docs/poshold_journey.md:180-184`:
> 実機モータの権限不足…**実機同定でプラントを作り直して設計し直す**ことで、定点保持 ±6〜7cm を達成
> 「±6〜7cm がこの37g機のトルク効きでの実用限界」

**現在の PID ゲイン（`position.pos.kp=0.4` / `vel.kp=3.0` 等）は 37 g の機体を
システム同定して設計されたもの。** 5〜11% の質量増は:

- 推力余裕を削る（既にトルク権限が限界と明記されている）
- 慣性モーメントを変える（搭載位置次第で無視できない）
- **同定済みプラントモデルを無効にする** → 位置ループの再同定が必要になりうる

**→ UWB 搭載後は、まず ALT_HOLD / POS_HOLD が現状通り成立するかを確認してから
UWB 統合に進むこと**（§6 の Step 0）。

### アンテナ禁止領域（`docs/WIRING.md:189-215`）

- 基板のアンテナ側（**2.0mm 面取りがある角の側**）の端から **3.556 mm、基板全幅**にわたる帯が
  KiCad フットプリントで F.Cu / B.Cu 両面 keepout に指定されている
- M5Stack 公式も「**FPC ケーブルをモジュールの PCB アンテナの下に通すな**」と明記（同 `:204-206`）

**StampFly での実務上の含意:**

| 守ること | 理由 |
|---|---|
| 面取り側 3.5 mm 以上に**金属・グランド・配線・バッテリ・カーボンを置かない** | パターンアンテナの特性が崩れる |
| 引き出し配線は **pin 6 / pin 7 側（面取りの反対端）へ逃がす** | 同上 |
| **バッテリの真上・真横を避ける** | LiPo は大きな導体。StampFly は 37g 機で余地が少ない |
| **モータ / プロペラの回転面から離す** | 遮蔽と振動 |
| **アンテナ側を上向き or 外向きに** | アンカーは通常、部屋の上方に置く（`docs/ANCHOR_PLACEMENT.md`） |
| S017-F は背面に FPC コネクタがあり**背面が平らでない**（厚さ 2.8mm） | ベタ貼り不可（`docs/WIRING.md:235-237`） |

**実験段階は半田パッド（S017 / キャステレーション）を使う方針が確定済み**
（`PROGRESS.md` ハードウェア構成の節）。

---

# 6. 段階的な導入計画

**原則: いきなり飛ばさない。各ステップに「次へ進む条件」を置く。**

## Step 0: UWB 抜きで現状を再確認（重量増の影響切り分け）

| やること | 合格条件 |
|---|---|
| ダミーウェイト（UWB モジュール + 配線 + LDO 相当、2〜4 g）を搭載位置に付けて ALT_HOLD / POS_HOLD | 現状（±6〜7cm）と同等に飛ぶ。悪化するなら**先に PID を再調整** |
| 電流測定（`motor sweep` CLI が既にある：`SF/main/config.hpp:327-397`） | 推力余裕とホバリング電流の悪化幅を把握 |

**→ ここを飛ばすと、後で「UWB のせいで飛ばない」のか「重いから飛ばない」のか切り分けられない。**

## Step 1: 地上、機体から独立して UWB スタックを立ち上げる

本リポジトリの `firmware/tag`（M5StampS3A）+ `firmware/anchor`（AtomS3 ×5）で完結。**StampFly は触らない。**

| # | やること | 合格条件 |
|---|---|---|
| 1.1 | Device ID `0xDECA0314` の読み出し | **`PROGRESS.md:411` が「最初の関門」と書いている箇所。ここが通らない限り先は無い** |
| 1.2 | 1対1 DS-TWR で距離が出る | 既知距離で ±1m 以内 |
| 1.3 | **アンテナ遅延校正**（R11、推奨校正距離 5.0 m） | `docs/archive/REIMPL_PLAN.md:195-205`。**3σ ≈ 30cm → 4.5cm。これをやらないと以降は全部無意味** |
| 1.4 | 5アンカー round-robin で全台から取れる | 各アンカー成功率 > 90%（`RangingScheduler::stats()`） |
| 1.5 | **1周の実測時間**（`lastCycleMs()`） | §2.2 (a) の予測 32 ms と突き合わせる。**ここで初めて §2 の数字が検証される** |
| 1.6 | アンカー自動測量（`docs/SURVEY_SPEC.md`） | 冗長度 > 0 の警告に従う（同 `:49-66`） |
| 1.7 | 静止測位（Lv2） | 既知点で bias < 10 cm、σ < 10 cm |

## Step 2: 手で動かして追従を見る（まだ飛ばさない）

| # | やること | 合格条件 |
|---|---|---|
| 2.1 | タグを手で持って部屋を歩く。Lv2 の軌跡をログ | 経路が破綻しない。飛び値が Huber で弾かれている |
| 2.2 | **姿勢を変える**（傾ける・裏返す・体で遮る） | どの姿勢で成功率が落ちるかを記録（§7-1） |
| 2.3 | 移動速度を上げる | 周内スミア（§3.3-ii）の影響を実測 |
| 2.4 | **モータを回した状態で測距**（機体に載せ、プロペラ外して地上でスロットル） | **成功率がモータノイズで落ちないか**（§7-2）。落ちるなら統合以前の問題 |

## Step 3: StampFly に統合するが、推定には使わない（観測のみ）

| # | やること | 合格条件 |
|---|---|---|
| 3.1 | `sf_hal_uwb_qm33120` + `uwb_task` を追加、`sensor_uwb` に publish | **`eskf.use_uwb = 0`（既定 OFF）のまま** |
| 3.2 | ログ / テレメトリに UWB 位置と ESKF 位置を**両方**出す | 400Hz ループのジッタが増えていない（`SF/tasks/imu_task.cpp` の busy 統計） |
| 3.3 | **通常飛行（ALT_HOLD / POS_HOLD）を UWB タスク動作中に実施** | 飛行性能が Step 0 と同等。**UWB タスクが飛行を邪魔しないことの証明** |
| 3.4 | ログを解析：UWB 位置 vs ESKF 位置（フロー由来）の差 | フローのドリフトが UWB で見える。**この差が「UWB を入れると何が直るか」の実測値** |
| 3.5 | 測距成功率を**飛行中に**測る | Step 2.4 の地上値と比較。悪化幅を把握 |

**この Step 3 が最も重要。** 実飛行データを取りながら一切のリスクを負わない。

## Step 4: 高度のみ UWB（ToF が届かない領域だけ）

| # | やること | 合格条件 |
|---|---|---|
| 4.1 | `updateUwbPosition()` を POS_Z のみ有効化。**ToF が有効な間（< 3.5 m）は UWB z を使わない** | 高度が暴れない |
| 4.2 | ToF レンジ外（> 4 m）へ上昇 | 高度制御が破綻しない |

**⚠ 同一平面アンカー配置では UWB の z は GDOP が悪い**（`AnchorTable::checkPlacement()` の
`coplanar` / `originWarning`）。**このステップは飛ばして Step 5 へ行ってもよい。**
実際、ToF の σ=0.01 m に対し UWB z は桁違いに悪いので、4 m 以下では使う理由がない。

## Step 5: 水平のみ UWB（本命）

| # | やること | 合格条件 |
|---|---|---|
| 5.1 | `updateUwbPosition()` を **POS_X / POS_Y のみ**有効化。`eskf.obs.uwb_noise` は保守的に（≥ 0.15 m） | 推定位置がジャンプしない |
| 5.2 | 紐付き（テザー）で ALT_HOLD ホバリング | ESKF 位置が UWB 座標に緩やかに収束する。振動しない |
| 5.3 | POS_HOLD で長時間（> 3分）ホバリング | **フロー単独ではドリフトしていた分が止まる**。これが UWB を入れた目的の直接的な証明 |
| 5.4 | 保持精度を測る | **±6〜7cm から悪化していない**こと。悪化するなら `uwb_noise` を大きくする |
| 5.5 | **UWB を意図的に落とす**（アンカーの電源を切る / 遮蔽する） | 現状動作へ滑らかに縮退する。**ジャンプもフライアウェイも起きない**（§3.3-iii） |
| 5.6 | **UWB を復帰させる** | **ここが最大の危険。** 復帰時に飛び出さないこと |

## Step 6: 完全統合

| # | やること |
|---|---|
| 6.1 | `GuidanceTarget`（`SF/components/sf_core/include/data_types.hpp:699-716`、`command_target` トピックは `RESERVED (M4+)`：`topics.hpp:137`）で絶対座標ウェイポイント飛行 |
| 6.2 | 4 m 超の高度で POS_HOLD（ToF レンジ外、z も UWB） |
| 6.3 | 暗所 / 無地の床（フローが効かない条件）で POS_HOLD |
| 6.4 | 必要なら案A-2（密結合）へ移行し、周内スミアと部分観測の問題を解消 |

---

# 7. リスクと未解決事項

## 7.1 アンテナ指向性 — 姿勢変化の影響

| 事項 | 状態 |
|---|---|
| M5Stamp UWB Module は**基板パターンアンテナ** | `docs/WIRING.md:201-206` |
| 指向性パターン | **データシートに記載なし。未調査** |
| StampFly の飛行姿勢 | 位置制御中は最大 ±? 度傾く（`clamp` 上限は `pid_controller.cpp:1336-1341`）。ドリフト補正時に実測 6.4° の傾きが報告されている（`SF/docs/poshold_journey.md:68`） |

**懸念**: パターンアンテナはヌル方向を持つ。機体が傾くと、特定のアンカーへの利得が
急に落ちる可能性がある。**測距が「傾いたときだけ」失敗すると、位置ループが傾ける →
測距が落ちる → 位置が悪化する → もっと傾ける、という正のフィードバックになりうる。**

**Step 2.2 で必ず実測すること。**

## 7.2 モータノイズ・機体による遮蔽とマルチパス

| 事項 | 状態 |
|---|---|
| ブラシモータの電気ノイズ | **未評価**。UWB は 7987.2 MHz（ch9）なのでモータの基本波とは離れているが、電源経由の混入は別問題 |
| 機体の導体（バッテリ / PCB / モータ） | 特定方向のアンカーを遮蔽しうる |
| プロペラの回転による周期的遮蔽 | **未評価**。プロペラは樹脂だが、回転面がアンテナの視野を横切る配置なら影響あり |
| マルチパス | 屋内は必ず起きる。**Lv2 は Huber + χ² ゲートを持つ**（`uwb_loc.h:115`）ので単発の飛び値には強い |
| **NLOS 判定材料が未取得** | `docs/archive/REIMPL_PLAN.md:207-219` の R12。`rsl_calculate_signal_power()` / `rsl_calculate_first_path_power()` が SDK にありビルド対象にも入っているのに一度も呼ばれていない。`RSL − FP_RSL > 6dB` が Qorvo 標準の見通し外指標 |

**R12（診断情報の取得）は `docs/archive/REIMPL_PLAN.md:207` が「費用対効果が最も高い追加」と
位置づけている。** 飛行体では機体自身が遮蔽物になるので、地上固定タグより重要度が高い。
→ **Step 2.4 / 3.5 の前に R12 を入れておく価値がある。**

## 7.3 UWB 喪失時のフェイルセーフ

### 良い知らせ: 縮退先が既に飛行実証済み

案B-2 では **UWB の観測が来なくなるだけ**で、ESKF は自動的に
「フロー + ToF + IMU」＝ **現在 ±6〜7cm で飛べている構成**に戻る。
新しいフェイルセーフ状態を作る必要がない。

### 悪い知らせ: 「落ちるとき」より「戻るとき」が危険

§3.3-iii の再掲:

1. UWB が数秒落ちる → その間フロー由来の x,y がドリフト（例: 50 cm）
2. UWB 復帰 → イノベーション 50 cm が一発で入る
3. ESKF の `pos_` がジャンプ
4. **`PidController` の `pos_setpoint_x_/y_` は POS_HOLD 進入時から動いていない**
   （`SF/components/sf_controller_pid/pid_controller.cpp:1317-1323`）
5. → 50 cm の「位置誤差」が発生したと解釈され、**最大 1.0 m/s で飛び出す**

**必須の対策（設計に入れること）:**

| # | 対策 |
|---|---|
| 1 | **イノベーションゲート**。`eskf.gate.tof_innov = 0.5`（`SF/components/sf_core/params.cpp:805`）の前例に倣い `eskf.gate.uwb_innov` を設ける |
| 2 | **段階的な再収束**。ゲートを超え続けたら一発で飛ばさず、`inflateCovariance(POS_X\|POS_Y)`（`eskf_core.hpp:173`、`eskf_core.cpp:123-160`）で共分散だけ膨らませ、ESKF に自然に引き寄せさせる |
| 3 | **推定位置がジャンプしたら POS_HOLD 目標も同時に付け替える**。さもないと (4) の構図になる |
| 4 | **陳腐化ウォッチドッグ**。`SENSOR_HEALTH_STALE_US = 500000`（`SF/main/config.hpp:295`）の前例に倣い、UWB 停止を `sensor_health` に載せて CLI / テレメトリから見えるようにする |
| 5 | **`eskf.use_uwb` を実行時に切れる**ようにする（`params.cpp:798-801` の `eskf.use_*` と同じ流儀）。異常時に CLI で即座に UWB を切り離せること |

### 既知の関連リスク: χ² ゲートのラッチアップ

`SF/docs/` に `chi2_latchup_explained.md` / `chi2_latchup_finding.md` /
`chi2_latchup_x_thread.md` の**3本**が存在する。
**このプロジェクトは χ² ゲートが閉じっぱなしになる問題を既に経験している。**
UWB という「たまに大きく外れる観測」をゲート付きで追加するのは、同じ罠を踏みやすい。
→ **Step 5 に入る前にこれら3本を読むこと。**

## 7.4 二重フィルタ（案B-1 を選んだ場合）

§4.1 の通り。**Lv2 を使う限り発生しない。**
「Lv3 の方が高機能そうだから」という理由で Lv3 を選ばないこと。

## 7.5 実機未検証であること — 一覧

| 未検証項目 | 影響範囲 |
|---|---|
| **Device ID の読み出し（Phase 1）** | **すべての前提**（`PROGRESS.md:411`） |
| §2 の全数値 | 静的解析のみ。SPI オーバヘッド 100 µs、`L_a` の一様分布仮定は未実測 |
| 測位ソルバの実行時間 | ESP32-S3 上で未測定（`docs/PERF_ANALYSIS.md:122`, `:250`） |
| アンテナ遅延の実値 | 無校正。**Δ=1ns で 30cm の定常バイアス**（`docs/archive/CRITICAL_REVIEW.md:130`） |
| 5アンカー構成の成功率 | R2/R3-1 適用後の実測なし |
| NVS の実読み書き / USB-Serial JTAG REPL | `PROGRESS.md:903-906` |
| **背面の 0.5mm 12P パッドに後付けでコネクタを半田付けできるか**（HW-4。実装済み StampFly 機体上での作業性） | 未確認 |
| **GROVE の供給電流定格** | 未公開（`docs/archive/SURVEY_stampfly_grove.md:49`） |
| **StampFly の 3.3V レギュレータ余裕** | `stampfly_ecosystem` に**記載が一切ない** |
| **UWB モジュールの TX 瞬時ピーク電流** | 未公開（`docs/archive/SURVEY_m5stamp_uwb_module.md:101`） |
| **UWB アンテナの指向性パターン** | 未調査 |
| 重量増（2〜4 g）の飛行性能への影響 | 未評価（Step 0） |
| ch9 の PLL 温度再校正（R10） | 未実装。**ch9 では 20°C 変化で再校正が必要**（`docs/archive/REIMPL_PLAN.md:189-193`） |

## 7.6 その他の未解決事項

| # | 事項 |
|---|---|
| 1 | **座標系の対応付け。** StampFly の `StateEstimate.position` は **NED**（`SF/components/sf_core/include/data_types.hpp:128`、ESKF の z は下向き：`eskf_core.cpp:549` の `innovation = -height - pos_.z`）。`uwb_loc` のワールド座標系は測量で決まる（`docs/SURVEY_SPEC.md` の「XY の規約」）。**変換を明示的に定義し、テストすること** |
| 2 | **ヨー基準。** UWB は位置しか与えない。部屋座標系での機首方位は地磁気（既定 OFF: `eskf.use_mag=0`）か、UWB 位置の履歴からの推定に頼ることになる。ウェイポイント飛行（Step 6.1）にはヨーの絶対基準が要る |
| 3 | **アンカー座標の機体側での保持。** 案A-2 に移行するときは機体の NVS にアンカーテーブルが要る。案B-2 なら UWB タスク内で完結する（これも案B-2 の利点） |
| 4 | `components/uwb_loc/` は上流の最適化前バージョンと byte 一致（`docs/PERF_ANALYSIS.md:186-191`）。上流の最適化（Lv2 で 4.3倍、スタック 3.1KB 減）は**取り込み待ち**。組込みではスタック削減の方が効く可能性 |
| 5 | **測量モードと運用モードが排他**（`docs/SURVEY_SPEC.md:14`、WiFi スタックのジッタが遅延送信を乱すため）。StampFly は ESP-NOW / WiFi を常用する（CommTask, ApiTask, TelemetryTask）。**飛行中の WiFi ジッタが TWR の遅延送信締切に影響しないか、Step 3.5 で実測が要る** |
| 6 | **`hard_reset_on_begin`（`uwb_qm33120_types.hpp:83`）は既定 true。** HW-1（RST 線なし）では成立しない。ソフトリセット経路の検証が要る |
| 7 | `IEstimator` への `updateUwb()` 追加は **`ComplementaryEstimator` にも波及する**（純粋仮想にした場合）。既定 no-op の非純粋仮想にすれば回避できる（§4.6-3） |

---

# 付録: 主要な数字の早見表

| 項目 | 値 | 出典 |
|---|---:|---|
| StampFly 制御ループ | 400 Hz | `SF/main/config.hpp:131-132` |
| **位置ループ実効帯域** | **≈ 0.064 Hz** | `position.pos.kp = 0.4`（`SF/components/sf_core/params.cpp:771`） |
| 現在の定点保持精度 | **±6〜7 cm, RMS 16 mm** | `SF/docs/poshold_journey.md:180` |
| 現在の水平位置の絶対観測 | **無し**（デッドレコニング） | `eskf_core.cpp` に POS_X/POS_Y の H が存在しない |
| ToF | 30 Hz, σ=0.01 m, 最大 4 m | `SF/main/config.hpp:136`, `params.cpp:790`, `vl53l3cx_wrapper.hpp:112` |
| フロー | 100 Hz, VEL_X/Y のみ | `SF/main/config.hpp:133`, `eskf_core.cpp:766-784` |
| **UWB (a) 現状** | **31.3 Hz**（1リンク 6.39 ms） | §2.2 |
| UWB (b1) R5 Anchor IRQ | 59.4 Hz | §2.2 |
| UWB (c1) R5+R6 両側 IRQ | 90.2 Hz | §2.2 |
| UWB レイテンシ（周中央基準） | 16〜23 ms | §3.3 |
| UWB 精度（M5Stack 測定） | 0.14 m | `docs/archive/SURVEY_m5stamp_uwb_module.md:109` |
| UWB 精度（無校正） | **数十cm〜1m超のバイアス** | `docs/archive/CRITICAL_REVIEW.md:130` |
| モジュール消費 | 58.0 mA @3.3V（タグ） | `docs/archive/SURVEY_m5stamp_uwb_module.md:100` |
| モジュール重量 | 0.5 g / 機体 37 g | `docs/WIRING.md:25`, `SF/docs/poshold_journey.md:5` |
| アンテナ禁止領域 | 面取り側から 3.556 mm × 全幅 | `docs/WIRING.md:194-197` |
| 外部に出せる GPIO | **GROVE 4本のみ**（G13/G15/G1/G2） | §5.2 |

---

## 結論（1段落）

**UWB の意義は「定点保持精度の改善」ではなく「絶対座標と無ドリフト化」である。**
StampFly は既にフロー + ToF + IMU で ±6〜7cm を達成しており、無校正の UWB を入れれば
確実に悪化する。一方、水平位置には絶対観測が構造的に存在せず（`eskf_core.cpp` に
POS_X/POS_Y の H が1つも無い）、そこが UWB の埋めるべき空白である。
統合方式は **`uwb_loc` の Lv2（無状態）で位置を出し、`vectorUpdate3()` で ESKF の
POS_X/POS_Y 観測として入れる疎結合（案B-2）** を推奨する。理由は、二重フィルタを
原理的に回避でき、既存 API にそのまま乗り、そして何より **UWB が落ちたときの縮退先が
「現在飛行実証済みの構成そのもの」** だからである。更新レートは現状の 31 Hz でも
位置ループ帯域の約480倍あり、R5/R6 による高速化は統合の前提条件ではない。
ただし **すべては「アンテナ遅延校正」と「実機で Device ID が読める」ことの後の話**であり、
現時点では本文書のすべてが検証待ちの仮説である。
