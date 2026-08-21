# 調査1: uwb_localizer の解析結果 (2026-08-19)

## 結論（要点）
uwb_localizer は **「測距値 → 位置」の測位演算ライブラリ**であり、
**UWBチップの制御（SPI/レジスタ叩き）と測距シーケンス（DS-TWRのフレーム往復・
タイムスタンプ処理）は明確にスコープ外**（README/BRINGUP.md に「やらないこと」と明記）。

## 構成
- `uwb_loc/` : Python本体（HAL抽象 + ソルバ + pipeline + CLI + ブラウザUI）。依存は numpy のみ
- `c/`       : **測位コアのみ C99 移植版**。ここが組込みへの移植対象
- `docs/`    : TUTORIAL / REFERENCE / UWB_PROTOCOL / UWB_ALGORITHMS / DESIGN / RYUW122 / BRINGUP
- `tools/crossval.py` : Python版とC版の数値突合（実測 1e-11 m 一致）

## 実機対応済みモジュール
- **REYAX RYUW122 / RYUW122_Lite** のみ（UART + ATコマンド, 115200bps）
  - `AT+ANCHOR_SEND` で測距要求 → `+ANCHOR_RCV=<addr>,<len>,<data>,<dist> cm[,RSSI]` をパース
  - 専用HAL `uwb_loc/hal/ryuw122.py` (711行) は **Pythonにしか存在しない**
- DW1000/DW3000/SR150 等は「距離が出れば同じコードが動く」と述べるのみで専用実装なし
- `BU01` の語はリポジトリ内に一切出現しない

## HAL抽象
- Python側のみ。`UwbHal` ABC = `anchors` プロパティ + `poll(timeout)` の2つだけの薄いI/F
- 実装: `TextHal`(正規表現) / `JsonLinesHal` / `PushHal`(コールバック) / `Ryuw122Hal`
- **C版にはHAL層が存在しない**。距離配列 `uwb_meas[]` を受け取るだけの純粋計算ライブラリ

## C実装 (c/) — 約1,900行
| ファイル | 行 | 責務 |
|---|---|---|
| include/uwb_loc.h | 245 | 公開API・型定義 |
| src/uwb_model.c | 354 | 設定初期化、観測モデル(残差・ヤコビアン)、GDOP/CRLB、同一平面判定、鏡像解 |
| src/uwb_closed_form.c | 234 | Lv0 LLS三辺測量、Beck法 GTRS 厳密解 |
| src/uwb_nls.c | 416 | Lv1/Lv2 重み付きNLS + Huber-IRLS + 物理/χ²ゲート (Gauss-Newton) |
| src/uwb_ekf.c | 387 | Lv3 密結合EKF (CV/CAモデル) |
| src/uwb_linalg.c/.h | 197/40 | 最大9x9 密行列演算 (LU/コレスキー/Jacobi) |

主要API: `uwb_config_init`, `uwb_solve_lv0/lv1/lv2`, `uwb_beck_gtrs`,
`uwb_lls_trilateration`, `uwb_ekf_init/reset/predict/update`, `uwb_gdop_at`,
`uwb_crlb_at`, `uwb_anchors_coplanar`, `uwb_version`

## 移植性（極めて良好）
- malloc/free 不使用、printf 不使用、OS/RTOS 依存なし、ブロッキングdelayなし
- 依存は `<math.h>` (sqrt/fabs) と `<string.h>` (strncmp) のみ
- C99準拠、`-Wall -Wextra -Werror -pedantic` 無警告
- `uwb_real` 型: 既定 double、`-DUWB_USE_FLOAT` で float 切替可
- スタック最大 6.6KB(double) 〜 1.5KB(float+縮小)。ESP32-S3 なら余裕
- 静的固定サイズ: UWB_MAX_ANCHORS=16 / UWB_MAX_MEAS=32 / UWB_ID_LEN=16

### 注意点
- `uwb_ekf` は状態保持のためスレッドセーフでない（タグごとに1インスタンス）
- `uwb_config.anchors` は借用（コピーしない）。ライフタイム管理は呼び出し側責務
- float化は Beck法の二分法と共分散で桁落ちしやすい。**まず double で検証してから**

## 測位アルゴリズム
- 観測方式は **DS-TWR（距離）を第一級**として設計。TDoA/AoA も残差の形が同じで同ソルバ対応
- Chan法 TDoA 閉形式解は Python 側に実装済み
- レベル: Lv0 閉形式LLS → Lv1 重み付きNLS+χ²ゲート → Lv2 Beck厳密解+Huber-IRLS(NLOS既定) → Lv3 密結合EKF(移動体)
- RANSAC は Python 版のみ（C版は組込み負荷を考慮し非搭載）

### RYUW122 の役割逆転（重要）
RYUW122 では AT コマンド上の役割名が一般定義と逆。README推奨構成は
**移動側 = ATの "ANCHOR" モード、固定点 = "TAG" モード**
（距離が ANCHOR 側 UART にしか出ないため）。
ライブラリの `Anchor.id` には RYUW122 的な "TAG" のアドレスを入れる。

## ビルド/テスト
- C版: 素の Makefile。`make` / `make test`(53件) / `make float` / `make strict` / `make size`
- Python版: pytest 177件。valgrind/ASan/UBSan 検証済み

---

## ESP32-S3 (ESP-IDF) 移植時の切り分け

### そのまま使える
- `c/include/uwb_loc.h` + `c/src/*.c` 一式 → ほぼ無改造でコンパイル可能
- Lv0〜Lv3 の測位アルゴリズム全部
- ビルド設定の考え方（マクロで ROM/RAM 調整）

### 書き直しが必要
- **HAL層全体**（Pythonにしかない）: UART AT 送受信・レスポンスパースを C で実装
- **測距シーケンスそのもの**: DW1000/DW3000 等を直接叩く場合は SPI制御 + DS-TWR を**ゼロから実装**
- ビルドシステム: Makefile → ESP-IDF `CMakeLists.txt` (`idf_component_register`)
- EKF の排他制御（FreeRTOS タスク間共有時）
- float 運用への切替（先に double で検証）
