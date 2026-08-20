# ESP32-S3 のプラットフォーム側高速化手段（調査結果 2026-08-20）

**これまでアルゴリズム最適化ばかり見ていて、SDK/プラットフォームが提供する手段を
調べていなかった。その抜けを埋めた記録。** すべて実際にツールチェーンで検証した事実。

## 1. 【最重要】ファームウェアが `-Og` でビルドされていた

```
CONFIG_COMPILER_OPTIMIZATION_DEBUG=y
→ 実際のコンパイルフラグ: -Og
```
（`build/compile_commands.json` で確認）

ホスト側のベンチはすべて `-O2`。**このまま実機で測っても比較にならない。**

- 対処: `CONFIG_COMPILER_OPTIMIZATION_PERF=y`（`-O2`）
- あわせて `CONFIG_COMPILER_OPTIMIZATION_ASSERTIONS_ENABLE` も見直す（現状 y、レベル2）

## 2. CPU が 160MHz（240MHz にできる）

```
CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ=160
```
ESP32-S3 は 240MHz 対応。**そのまま 1.5倍。**
- 対処: `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y`
- 消費電力とのトレードオフ。StampFly のような電池駆動では要検討

## 3. 単精度 FMA `madd.s` / `msub.s` がハードウェアにある

**以前の調査で add/mul/div/sqrt しか見ておらず、FMA を見落としていた。**

| 命令 | 有無 |
|---|---|
| `madd.s` (float 積和) | **あり** |
| `msub.s` (float 積差) | **あり** |
| `madd.d` (double 積和) | **無し** |

しかも **GCC が `a + b*c` から自動的に `madd.s` を生成する**（-O2、ループ内でも
ハードウェア `loop` 命令と組み合わせて出る）。逆アセンブルで確認済み。

→ **`UWB_USE_FLOAT` の価値は、以前の見立てよりさらに高い。**
   EKF の P 更新や Gauss-Newton の JᵀWJ は積和の塊なので、
   float 化するだけで乗算+加算が1命令になる。

### 改めて整理: ESP32-S3 の浮動小数点（すべて逆アセンブルで確認）
| 演算 | float | double |
|---|---|---|
| 加算 `add.s` / 乗算 `mul.s` | **ハード1命令** | ソフト (`__adddf3`/`__muldf3`) |
| **積和 `madd.s`** | **ハード1命令** | **無し** |
| 除算 | ソフト (`__divsf3`) | ソフト (`__divdf3`) |
| sqrt | ソフト (`sqrtf`) | ソフト (`sqrt`) |

`-ffast-math` でも除算・sqrt はハード命令にならない（ISA に無い）。

## 4. esp-dsp（Espressif 公式 DSP ライブラリ、v1.8.2）

`idf.py add-dependency "espressif/esp-dsp"` で取得できる managed component。

### 関係しそうなもの
- **float 行列積の Xtensa アセンブリ最適化版があり、3x3 / 4x4 の専用実装まである**
  - `dspm_mult_3x3x3_f32_ae32.S`, `dspm_mult_4x4x4_f32_ae32.S`
  - `dspm_mult_3x3x1_f32_ae32.S`, `dspm_mult_4x4x1_f32_ae32.S`
  - `dspm_mult_f32_aes3.S`（S3 専用。`madd.s` を多用）
- float 内積: `dsps_dotprod_f32_aes3.S`
- 行列の加減・定数倍: `dspm_add/sub/addc/mulc_f32_ae32.S`
- **EKF モジュール**: `modules/kalman/ekf`（汎用EKF基底クラス、C++）
- sqrt: `dsps_sqrtf_f32_ansi()` — **ANSI 実装のみでアセンブリ版は無い**。
  近似による高速版の可能性があるが未評価

### ただし適用範囲は限定的（重要な留保）
**我々の最適化後のホットパスは「単純な行列積」ではない**:
- EKF の P 更新 → ランク1更新（外積）。行列積ではない
- Gauss-Newton の JᵀWJ → 重み付き。素の行列積ではない
- Beck → 4x4 LU は既に廃止済み。いまはスカラー演算
- 3x3 の逆行列・連立解 → 行列積ではない

**皮肉だが、esp-dsp の行列積が最も効いたのは「最適化前の O(nx³) コード」**。
アルゴリズムを直した後では出番が減っている。
それでも `dspm_mult_3x3x3` / `4x4x4` が使える箇所がないか、実機測定後に再評価する価値はある。

### 設計上の論点
`uwb_loc` は**依存ゼロの移植性重視 C99**（malloc も OS 依存も無い）。
esp-dsp を本体に入れるとその設計が壊れる。取るなら:
- マクロで切り替える optional backend にする、または
- `uwb_loc` は純粋なまま保ち、`m5stack_uwb` 側のラッパで置き換える

## 5. ベクタ命令 (PIE) は測位計算に使えない

ESP32-S3 にはベクタ命令があるが**8/16bit 固定小数点専用**。
アセンブラで実際に確認:

| 命令 | 結果 |
|---|---|
| `ee.vmul.s8` / `ee.vmul.s16` | **あり** |
| `ee.vmulas.s16.accx` (積和) | **あり** |
| `ee.vld.128.ip` (128bitロード) | **あり** |
| `ee.vmul.s32` | **無し** |
| float 版 (`ee.vfmul.s` 等) | **無し** |

AI 推論向けの固定小数点 SIMD であり、浮動小数の測位計算には直接使えない。
使うなら測位計算全体を固定小数点化する必要があるが、
上流 README が float ですら桁落ちを警告している数値特性を考えると現実的でない。

## 6. RVfplib は使えない

`CONFIG_COMPILER_FLOAT_LIB_FROM_RVFPLIB` は `depends on ESP_ROM_HAS_RVFPLIB` で
**RISC-V 専用**。Xtensa の ESP32-S3 では選択できない。

## 7. その他の未評価項目
- **IRAM 配置** (`IRAM_ATTR`): ホット関数を flash から IRAM へ。
  flash キャッシュミスの影響を排除できる。命令キャッシュは現状 16KB/8way
- 命令キャッシュサイズの変更（現状 `CONFIG_ESP32S3_INSTRUCTION_CACHE_16KB`）
- flash 速度は既に 80MHz/DIO。QIO にできるかはボード依存

---

## 直近の対処（優先順）

| # | 項目 | 効果 | 手間 |
|---|---|---|---|
| 1 | `CONFIG_COMPILER_OPTIMIZATION_PERF=y` (-O2) | 大 | 1行 |
| 2 | `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y` | 1.5倍 | 1行 |
| 3 | ベンチに `madd.s` の測定を追加 | 判断材料 | 小 |
| 4 | `UWB_USE_FLOAT` の実機評価（精度と速度を並べて） | 大 | 中 |
| 5 | IRAM 配置の効果測定 | 未知 | 中 |
| 6 | esp-dsp の適用可否を実機測定後に再評価 | 限定的 | 大 |
