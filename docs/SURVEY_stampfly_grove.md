# 調査4: StampFly GROVE 端子と GPIO 空き状況 (2026-08-19)

## 結論
**GROVE 1系統（信号2本）では SPI は物理的に不可能。** 本数不足であり、
ピン機能制約（strapping 等）の問題ではない。

## メインMCU
- **ESP32-S3FN8**（Xtensa LX7 デュアル 240MHz、内蔵Flash 8MB）@ M5StampS3A
- M5StampS3A が露出する GPIO は 23本:
  G0,G1,G2,G3,G4,G5,G6,G7,G8,G9,G10,G11,G12,G13,G14,G15,G39,G40,G41,G42,G43,G44,G46
- ※ローカル文書の「PSRAM 2MB」記載は ESP32-S3**FN8** の命名則と矛盾（PSRAM非搭載の疑い）。要実機確認

## StampFly の GPIO 使用状況（ソース実証）
| GPIO | 用途 | 出典 |
|---|---|---|
| G0 | BOOTボタン | M5StampFly/src/flight_control.hpp:36, button.cpp:11 |
| G3/G4 | 内蔵I2C SDA/SCL | M5StampFly/src/sensor.hpp:41-42, stampfly_hal/.../i2c.cpp:234-235 |
| G6/G7/G8/G9 | ToF 底面INT/XSHUT, 前方INT/XSHUT | M5StampFly/src/tof.hpp:33-36 |
| G14/G43/G44/G46 | SPI MOSI/MISO/CLK/CS(BMI270) | M5StampFly/src/sensor.hpp:43-46 |
| G39 | WS2812 RGB LED | M5StampFly/src/led.hpp:41 |
| G11 | BMI270 INT1 | stampfly_ecosystem HW仕様書 |
| G12 | PMW3901 CS2 | 同上 |
| G40 | ブザー | 同上 |

### ~~未使用（＝空き）GPIO~~ → **【重大な訂正 2026-08-21】誤りだった**

当初「**G5, G10, G41, G42 の4本が空き**」と記載したが、**これは誤り**。

**現行ファーム `stampfly_ecosystem/firmware/vehicle/main/config.hpp:63-66` で
この4本はすべてモータ PWM に割り当てられている**:
```c
inline constexpr int GPIO_MOTOR_M1 = 42;  // FR, CCW
inline constexpr int GPIO_MOTOR_M2 = 41;  // RR, CW
inline constexpr int GPIO_MOTOR_M3 = 10;  // RL, CCW
inline constexpr int GPIO_MOTOR_M4 = 5;   // FL, CW
```
**機体が飛ぶために必須の信号**であり、転用は不可能。

**誤りの原因**: 当時の調査は古い `M5StampFly`（Arduino版）と `stampfly_hal` を
対象にしており、**現行の `firmware/vehicle` を見ていなかったためモータピンを見落とした**。

### 結果として何が変わるか
- **本文書の「代替案2: 空きGPIO (G5/G10/G41/G42) を使う」は成立しない**
- `docs/PLAN.md` の配線案B（空きGPIO 直付け）も**成立しない**
- **外部に出せるのは GROVE の4本（G13 / G15 / G1 / G2）のみ**
- したがって StampFly では **SPI 4線で使い切り、IRQ 線が取れない**。
  R6（IRQ 駆動化）は StampFly では効かないことになる
  （アンカー側の AtomS3 では IRQ を確保できているので、そちらには効く）
- 残る選択肢は「GROVE 2系統で SPI 4線」または
  「内蔵 SPI2_HOST に相乗り（要パッド直付け、IMU と同バス）」

詳細は `docs/STAMPFLY_INTEGRATION.md` を参照。

## GROVE 端子（HY2.0-4P）— 物理的に2系統ある
| コネクタ | 色 | 用途 | 信号→GPIO |
|---|---|---|---|
| GROVE (RED) | 赤 | I2C | SDA=**G13**, SCL=**G15** |
| GROVE (BLACK) | 黒 | UART | RX=**G1**, TX=**G2** |

出典: stampfly_ecosystem `.../sf_hal_bmi270/docs/M5StamFly_spec_ja.md:125-130`、
docs.m5stack.com/en/app/Stamp%20Fly、同 StampFly_v1.1（**初代/v1.1で GPIO 番号は同一**）

### 内蔵バスとの共有: **無し（完全独立）**
- 内蔵 I2C は G3/G4（INA3221, BMM150, BMP280, VL53L3×2）
- 内蔵 SPI は G14/G43/G44 + CS G46/G12
- GROVE I2C(G13/G15) / GROVE UART(G1/G2) はいずれも**別配線**
- GROVE UART は現状ファームウェアで**未使用**（開放されている）

### 電源
- M5Stack の GROVE(HY2.0-4P) は一般に **5V** 出力（黒=GND, 赤=5V, 信号2本）
  出典: docs.m5stack.com/en/learn/interface/grove
- **StampFly の GROVE 供給電流定格は公式ドキュメント・ローカル資料いずれにも記載なし**
- 推測: 300mAh級 LiPo 単セルでモータ4基と共有のため、大電流は期待できない

## ESP32-S3 の SPI 割当自由度
- G13/G15/G1/G2 いずれも **strapping ピンでも USB ピンでもない**（strapping は G0,G3,G45,G46）
- **SPI2(FSPI)/SPI3 は GPIO マトリクス経由で任意ピンに配置可能**
  出典: ESP-IDF SPI Master ドキュメント
- 制約: マトリクス経由は 40MHz 超で入力遅延（約25ns）により不安定になり得る
  → 本件は 16MHz 運用なので**問題にならない**

## GROVE で SPI を賄えるか → **賄えない**
SPI 最小構成は SCK/MOSI/MISO/CS の**4信号線**（IRQ 併用なら5本）。
GROVE は 1系統あたり信号2本のみ。**本数が足りない。**

### 代替案
1. **GROVE 2系統を両方使い 4本確保**（G13, G15, G1, G2 → SCK/MOSI/MISO/CS）
   - IRQ 用のピンが残らない → **ポーリング動作前提**
   - 2コネクタは基板上の別位置。標準 GROVE ケーブル1本では配線不可
     → **カスタムケーブル/半田付けが必要**
   - I2C拡張・UART拡張の両方を潰すトレードオフ
   - GPIOマトリクス上の割当自体は問題なし
2. **空きGPIO (G5/G10/G41/G42) を使う** — SPI4線+IRQ を最もクリーンに確保できる可能性。
   ただし基板上にアクセス可能な形で出ているか**未確認**。基板直付け前提
3. UART/I2C ブリッジ付き別モジュールに置き換え（M5Stamp UWB Module は SPI 専用なので該当せず）

## 未確認事項
- GROVE の供給電流定格
- G5/G10/G41/G42 が物理パッドとして存在するか
- M5StampS3A の PSRAM 有無
