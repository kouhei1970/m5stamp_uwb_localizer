# M5Stamp C5 対応計画

M5Stack の **M5Stamp C5**（以下 Stamp-C5。Espressif ESP32-C5 搭載）を、タグ・アンカーの
ホストボードとして M5StampS3A に**追加**する計画。M5StampS3A は引き続き既定のホストで、
StampFly は本計画の対象外。

- 作成: 2026-09-05。根拠は Stamp-C5 回路図 V0.2（2026-02-07）・公式ピン図・ESP32-C5
  データシート・ESP-IDF v5.5.2（§7 出典）
- 状態: **机上検討と試しビルドまで**。実機は未入手・未検証
- 関連: [`PLAN.md`](PLAN.md)（全体計画）、[`WIRING.md`](WIRING.md)（配線）、
  [`ARCHITECTURE_V2.md`](ARCHITECTURE_V2.md)（2 コア前提のタスク設計）

## 0. 結論（先に要点）

| 項目 | 結論 |
|---|---|
| 配線 | **UWB モジュールと Stamp-C5 を 0.5 mm 12P の FPC ケーブル 1 本で直結できる見込み**。電源・GND の位置が両者で完全に一致し、信号も 1 対 1 で対応する（§2）。BreakOut も手配線も不要 |
| ビルド | 現行ソースのまま `idf.py set-target esp32c5` で**タグ・アンカーともビルドは通る**（2026-09-05 確認、アンカー 1.24 MB・タグ 1.34 MB）。ただしピン定義とコア固定が S3 用のままなので、そのままでは動かない |
| 最大の技術リスク | ESP32-C5 は**シングルコア**。現行設計は「電波を扱うタスクをコア 1 に隔離し、Wi-Fi・ダッシュボードをコア 0 に置く」前提（§3.3） |
| 利点 | 電池入力（3.7 V、充電回路内蔵）でアンカーの電池駆動が素直にできる。5 GHz Wi-Fi が使える。配線ゼロ |
| 見積もり | 机上・ビルド 1〜2 日、実機 probe/twr 1 日、anchor/tag 実運用 1〜2 日、文書 半日（§6） |

## 1. Stamp-C5 の要点

| 項目 | 内容 | 本計画への意味 |
|---|---|---|
| SoC | ESP32-C5HF4: RISC-V **単一コア** 240 MHz、SRAM 384 KB、フラッシュ **4 MB** 内蔵、PSRAM なし | コア固定の見直し（§3.3）、フラッシュ 8 MB → 4 MB（§3.4） |
| 無線 | Wi-Fi 6 の 2.4 / 5 GHz 両対応、BLE 5、IEEE 802.15.4 | ダッシュボード用の Wi-Fi を 5 GHz へ逃がせる（§3.8） |
| アンテナ | **基板内蔵アンテナ無し**。IPEX コネクタに付属の FPC アンテナ（2.4/5 GHz 共用、50 mm）を挿す | UWB モジュールのアンテナ禁止領域（WIRING.md §5.2）との配置を決める |
| 電源 | USB-C、VBAT 端子（3.7 V リチウム電池。充電 IC SGM40567、充電電流 200 mA）、3.3 V レギュレータ JW5712（最大 600 mA） | アンカーの電池駆動。UWB モジュール（タグ 58 mA）と Wi-Fi 送信ピーク（数百 mA）の合計は 600 mA 以内の見込み。要実測 |
| USB | ESP32-C5 内蔵の USB-Serial/JTAG に直結（USB-UART ブリッジ無し） | M5StampS3A と同じ運用。`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y` をそのまま使える |
| 外部端子 | 2.54 mm パッド 11 本（G1〜G10、G28）。背面 FPC 0.5 mm 12P: 3V3×2、G23、G0、G24、G25、G26、G27、G11(TXD)、G12(RXD)、GND×2 | FPC 側だけで UWB の 7 信号 + 電源が足りる（§2） |
| LED | 青 LED = G28（BOOT ストラップ兼用、単色）、赤 LED = 充電状態 | 現行の WS2812 用ドライバは使えない（§3.5） |
| 書き込みモード | G28 を GND に落として電源投入 | 通常は USB-Serial/JTAG 経由で不要（§5） |
| 寸法 | 17.6 × 19.1 × 3.4 mm、1.8 g | **StampS3 BreakOut には載らない**（パッド配置が違う）。FPC 直結なら BreakOut 自体が不要 |
| 価格 | Stamp-C5 6.50 USD、Stamp-C5 DIP（ヘッダ実装済み）7.95 USD | FPC 直結なら無印で足りる |

## 2. 接続案: FPC 直結（本命）

M5Stamp UWB Module（S017-F、FPC コネクタ J1）と Stamp-C5 の背面 FPC（JP3）は、
どちらも 0.5 mm ピッチ 12 ピンで、**1・2 番が 3.3 V、7・11 番が GND** という並びまで同じ。
残りの信号を番号どおりに突き合わせると次のようになる。

| FPC 番号 | UWB モジュール側（J1） | Stamp-C5 側（JP3） | ESP32-C5 の事情 | 判定 |
|---:|---|---|---|---|
| 1, 2 | VCC_3V3 | 3V3（レギュレータ出力 VSYS_3V3） | — | 一致 |
| 3 | DW_GP7（未使用） | G23 | 特殊機能なし | 配線されるが未使用 |
| 4 | DW_IRQ（モジュール → ホスト、10 kΩ プルダウン付き） | G0 | 32 kHz 水晶用の兼用ピン。GPIO 割り込み可 | 可 |
| 5 | DW_WAKEUP（ホスト → モジュール） | G24 | 特殊機能なし | 可 |
| 6 | DW_RSTn（ホスト → モジュール、モジュール側 10 kΩ プルアップ） | G25 | **ストラップピン**（SDIO のタイミング設定用）。起動時はモジュールのプルアップで High | 影響なしの見込み |
| 7 | GND | GND | — | 一致 |
| 8 | DW_CDO = MISO（モジュール → ホスト） | G26 | **ストラップピン**（G28=0 のときのダウンロードモード判定に関与）。MISO は CS=High のとき Hi-Z で、C5 側は内部プル無し | §5 の要確認事項 |
| 9 | DW_CDI = MOSI（ホスト → モジュール） | G27 | **ストラップピン**（ROM メッセージ出力の有無・ブートモード）。内部プルアップあり。モジュール側は入力なので起動時は High のまま | 影響なし |
| 10 | DW_CSn（ホスト → モジュール） | G11 = UART0 TXD | チップと端子の間に **499 Ω の直列抵抗（R5）**。起動時に ROM・ブートローダのログがこの線に出る | §5 の要確認事項 |
| 11 | GND | GND | — | 一致 |
| 12 | DW_CLK = SCK（ホスト → モジュール） | G12 = UART0 RXD | 直列抵抗なし。起動時は入力（プルアップ） | 可 |

- SPI は ESP32-C5 の **SPI2_HOST**（汎用 SPI はこれ 1 系統だけ）を GPIO マトリクス経由で使う。
  80 MHz 以下なら IOMUX 直結と同等なので、本リポジトリの 16 MHz は問題ない。
- CS はソフトウェア制御の GPIO（`uwb_port.c` の方針そのまま）。
- **FPC ケーブルの向き（同面接点／異面接点）は実物で確定する。** 逆向きだと 1・2 番（3.3 V）が
  11・12 番（GND・G12）に当たり、モジュールに電源が入らないうえ G12 に 3.3 V を押し込む。
  手順は WIRING.md §2.2 と同じで、「両端の 1・2 番だけが互いに短絡している（3V3）」性質を
  テスターで確認してから通電する。
- ケーブルは短いもの（付属品）を使い、UWB モジュールの PCB アンテナの下を通さない
  （WIRING.md §2.4）。Stamp-C5 の Wi-Fi アンテナも UWB モジュールの禁止領域の外に置く。

代替案（FPC 直結が成立しない場合）: Stamp-C5 DIP の 2.54 mm パッド（G1〜G10）へ
FPC→DIP 変換基板から手配線する。現行の経路 A（WIRING.md §2）と同じ作業になる。

## 3. ファームウェアの変更点

2026-09-05 の監査で見つかった「ESP32-S3 前提」の箇所を、必要な変更に落とし込んだもの。
行番号は当日時点。

### 3.1 ボード定義（新規 `boards/stampc5.h`）

| 項目 | 値 | 根拠 |
|---|---|---|
| spi_host | SPI2_HOST | ESP32-C5 の汎用 SPI は SPI2 のみ |
| pin_sck / mosi / miso / cs | 12 / 27 / 26 / 11 | §2 の FPC 対応表 |
| pin_rst / irq / wakeup | 25 / 0 / 24 | 同上 |
| pin_gp7 | UWB_PORT_PIN_UNUSED（配線上は G23） | 現行と同じく未使用 |
| spi_slow_hz / spi_fast_hz | 2 MHz / 16 MHz | 現行と同じ |
| status LED | G28（単色・青） | §3.5 |

ヘッダの冒頭コメントには、現行 `boards/stamps3.h` と同じ体裁で「実機での確認状況」欄を
設け、最初は全項目「未確認」と書く。

### 3.2 ボード選択の配線（Kconfig と main.cpp）

| 場所 | 変更 |
|---|---|
| `firmware/{tag,twr,probe,devtest}/main/Kconfig.projbuild` の `choice UWB_*_BOARD` | `UWB_*_BOARD_STAMPC5` を追加 |
| `firmware/anchor/main/Kconfig.projbuild` | choice が STAMPS3 のみ。STAMPC5 を追加 |
| `firmware/anchor/main/main.cpp:89-92` | `boards/stamps3.h` を無条件 include している。`#if` 分岐にする |
| `firmware/{tag,twr,probe,devtest}/main/main.cpp` の `#if CONFIG_UWB_*_BOARD_STAMPFLY / #else` | STAMPC5 の分岐を追加（`BOARD_UWB_PORT_CONFIG` / `BOARD_NAME` / `BOARD_STATUS_LED_GPIO`） |
| `tools/docs_check/verify_docs.py:42,84,121` | ボードヘッダの一覧が 3 か所にハードコード。`boards/stampc5.h` を追加しないと文書検査から黙って外れる |

### 3.3 シングルコア対応（最大の変更点）

現行設計はコア 1 に電波（`uwb_radio` / `RangingService`）、コア 0 に Wi-Fi・lwIP・HTTP・
ログを固定している。ESP32-C5 では `CONFIG_FREERTOS_UNICORE=y`・`CONFIG_SOC_CPU_CORES_NUM=1`
となり、**コア 1 を指定した `xTaskCreatePinnedToCore()` は失敗して起動が止まる**
（例: `firmware/anchor/main/main.cpp:723-725` は失敗時に return する）。

| 場所 | 現状 | 変更 |
|---|---|---|
| `components/uwb_ranging/include/uwb_ranging_service.hpp:237` | `int core = 1` | 既定値を「2 コアなら 1、1 コアなら 0」にする |
| `components/uwb_ranging/src/uwb_ranging_service.cpp:69` | `xTaskCreatePinnedToCore(..., cfg_.core)` | 共通ヘルパ経由に |
| `firmware/anchor/main/main.cpp:720-722`、`Kconfig.projbuild:141-150`（`UWB_ANCHOR_RADIO_TASK_CORE`、既定 1） | 同上 | Kconfig の既定を `SOC_CPU_CORES_NUM > 1 ? 1 : 0` に。range も 0..(コア数−1) |
| `firmware/tag/main/main.cpp:1134`、`Kconfig.projbuild:132-140`（`UWB_TAG_SERVICE_TASK_CORE`） | 同上 | 同上 |
| `firmware/tag/main/main.cpp:1245`（ログタスク、コア 0 固定） | 0 固定 | 1 コアでも 0 なので変更不要だが、ヘルパ経由に統一 |
| `components/uwb_net/src/{uwb_net_sink.cpp:244, uwb_net_tcp.cpp:239, uwb_net_http.cpp:499, uwb_net_udp.cpp:125, uwb_net_wifi.cpp:1102}` | コア 0 固定 | 同上（1 コアでは 0 のままで動く） |
| `firmware/{tag,anchor}/sdkconfig.defaults` の `CONFIG_LWIP_TCPIP_TASK_AFFINITY_CPU0` / `CONFIG_ESP_WIFI_TASK_PINNED_TO_CORE_0` | S3 向けの記述 | C5 では意味を持たない（試しビルドでは警告なく無視された）。ターゲット別ファイルへ移す（§3.4） |

共通ヘルパの案（`components/uwb_port` か新規の小さなヘッダ）:

```c
/* 要求したコア番号をこのチップで使える範囲に丸める / clamp a wanted core id to what this SoC has */
static inline int uwb_task_core(int wanted) {
#if CONFIG_FREERTOS_UNICORE
    (void)wanted; return 0;
#else
    return wanted;
#endif
}
```

**実時間性の懸念**: 1 コアでは電波タスク（優先度 20）が Wi-Fi ドライバのタスク（ESP-IDF
既定で優先度 23）・lwIP・HTTP サーバと CPU を奪い合う。DS-TWR は遅延送信の締切（ms 単位）
を守る必要があり、Wi-Fi の送受信中に電波タスクが数 ms 待たされると `tx_failures` や
`final_timeouts` が増える。M5StampS3A（2 コア）でも Wi-Fi 併用時に周期成功率が
99.5〜99.6% → 99.3〜99.4% へ下がっており（HANDOFF.md §1）、1 コアではこれより悪化しうる。
対策の候補:

1. 電波タスクの優先度を Wi-Fi ドライバより上（`configMAX_PRIORITIES-1`）にする
2. IRQ 駆動（現行既定）を維持し、ポーリング区間を最小にする
3. ダッシュボード配信（WebSocket 50 ms 周期）の負荷を下げる、または送信をまとめる
4. 効果は §4 の Step 3・5 で「Wi-Fi なし／あり」の周期成功率を比べて判断する

### 3.4 ターゲット別の sdkconfig とフラッシュ 4 MB

ESP-IDF は `sdkconfig.defaults` に加えて **`sdkconfig.defaults.<target>`** を自動で読む。
S3 専用・C5 専用の設定をそれぞれのファイルへ分ける。

| ファイル | 内容 |
|---|---|
| `sdkconfig.defaults`（共通） | `CONFIG_IDF_TARGET` 行を削除（`set-target` で決める）、FreeRTOS 1 kHz、`-O2`、USB-Serial/JTAG コンソール、パーティション、HTTP/lwIP 設定、EKF 既定 |
| `sdkconfig.defaults.esp32s3` | `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`、lwIP・Wi-Fi のコア 0 固定 |
| `sdkconfig.defaults.esp32c5` | `CONFIG_ESPTOOLPY_FLASHSIZE_4MB=y`、`CONFIG_UWB_*_BOARD_STAMPC5=y` |

- `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_240=y` は C5 でも同じ名前で有効（試しビルドで確認）。共通に残す。
- パーティション表（`partitions.csv`: nvs 24 KB、factory 3 MB）は 4 MB に収まる
  （0x10000 + 3 MB = 0x310000 < 0x400000）。変更不要。アプリ実サイズはアンカー 1.24 MB、タグ 1.34 MB。
- 試しビルド（2026-09-05、`idf.py -B build_c5 -D SDKCONFIG=build_c5/sdkconfig set-target esp32c5 build`）
  は上記の分離をせずに通ったが、生成された sdkconfig は 8 MB のままなので**このバイナリを
  書き込んではいけない**。

### 3.5 ステータス LED

`components/uwb_status_led` は WS2812（RMT 送信）専用。Stamp-C5 の青 LED は G28 に
1 kΩ を介して 3.3 V へ繋がる単色 LED（Low 点灯）で、しかも BOOT ストラップ兼用（10 kΩ プルアップ）。
起動後に出力にするぶんには M5Stack 自身の設計どおりで問題ない。

- 変更: `uwb_status_led` に「単色 GPIO」バックエンド（Kconfig で WS2812 / 単色 / なし を選択）を追加する
- 最小案: C5 ではハートビートを無効化して先に進み、LED は後回し

### 3.6 その他の見直し箇所

| 場所 | 内容 |
|---|---|
| `components/uwb_port/src/uwb_port.c:505-521` | IRQ ピンの内部プルダウン値を ESP32-S3 の約 45 kΩ で論じている。C5 の値は別。モジュール側の 10 kΩ プルダウン（R2）があるので結論は変わらない見込みだが、probe の L5（IRQ 自己診断）で確認する |
| `components/uwb_net/src/uwb_net_sink.cpp:41` | 送信バッファ 12 KB を S3 の内蔵 SRAM 前提で正当化。C5 は SRAM 384 KB（S3 は 512 KB）で、空きヒープを `info` で確認する |
| `components/uwb_survey/include/uwb_survey.h:93` | スタック使用量の実測値が xtensa の gcc のもの。riscv で `-fstack-usage` を取り直す |
| ブートローダ/ROM の UART0 出力 | G11（= CS）に起動ログが出る。SCK が止まっていれば UWB 側は無視するはずだが、`CONFIG_BOOTLOADER_LOG_LEVEL_NONE` にする選択肢を残す |
| `uwb_math`・`uwb_survey` の「単精度 FPU のみ」前提 | ESP32-C5 も単精度 FPU のみなので設計は変えない。コメントの表記を「ESP32-S3 / C5」に直す |

### 3.7 CI と配布物

| 場所 | 変更 |
|---|---|
| `.github/workflows/build.yml:150-151` | `target: esp32s3` が全変種共通。matrix に `target` 軸を足し `${{ matrix.target }}` にする |
| 同 `:97-129` の変種名 | `probe-stampc5`、`anchor-stampc5-ds`、`tag-stampc5-ds` を追加（`<app>-<board>[-<variant>]` の規則どおり） |
| 同 `:190,199,265,268` | `esptool.py --chip esp32s3` が固定。matrix の値へ |
| `docs/PREBUILT_BINARIES.md` | `*-stampc5` の行と `--chip esp32c5` の書き込み例 |

### 3.8 Wi-Fi の 5 GHz

`components/uwb_net` には周波数帯を選ぶコードが無く（`esp_wifi_set_mode` のみ）、
C5 では ESP-IDF の既定（2.4/5 GHz を SSID で自動選択）で動く。初期スコープでは
**2.4 GHz の現行運用のまま**とし、`wifi` コマンドの表示に「帯域・チャネル」を追加するだけにする。
5 GHz の明示選択（`wifi band 5g` 等）は Step 7 の任意項目。ESP-IDF v5.5.2 の
ESP32-C5 対応は「初期対応」で、5 GHz のチャネル・国コード設定はまだ整備途中の項目が残る（§7）。

## 4. 実機での確認手順（受入検査の順番）

| Step | 内容 | 合格条件 |
|---:|---|---|
| 0 | 入手: Stamp-C5 ×2 以上（無印でよい）、UWB モジュール付属の 0.5 mm 12P FPC ケーブル、付属 Wi-Fi アンテナ。`esptool.py --chip esp32c5 chip_id` でチップリビジョンを記録 | ESP-IDF v5.5.2 が対応するリビジョンであること |
| 1 | FPC の向き確認（§2 の 3V3 短絡法）→ 通電 → 消費電流 | UWB モジュールの 3.3 V が出ている。過電流なし |
| 2 | `firmware/probe` を Stamp-C5 用にビルド・書き込み、受入検査 L1〜L11 | Device ID 0xDECA0314、L4（RSTn）・L5（IRQ）PASS、L6（16 MHz SPI 1,000 回）不一致 0 |
| 3 | `firmware/twr` で 1 対 1 測距。組み合わせ: C5 アンカー × S3A タグ、C5 × C5。Wi-Fi なし／あり | 周期成功率 ≥ 99%（現行 S3A: 99.5〜99.6%）。`elapsed_ms` が S3A と同等 |
| 4 | `firmware/anchor`（C5）を既存の S3A アンカー 4 台構成に混ぜて 3D 測位 | 無線プロトコルは同じなので混在可。C5 の成功率が S3A と同等 |
| 5 | `firmware/tag`（C5）で測位 + ダッシュボード同時運用 | `cycle_ms`・成功率・残差が S3A タグと同等。ダッシュボード停止なし（1 コアの実時間性を見る本番項目） |
| 6 | VBAT に電池を繋いで長時間運転 | 電池のみで 3D 測位が数時間続く |
| 7 | （任意）5 GHz Wi-Fi | 2.4 GHz と成功率が変わらない |

各 Step の結果は HANDOFF.md §1（検証状況の正本）へ日付付きで追記する。

## 5. リスクと未確認事項

| 項目 | 内容 | 対処 |
|---|---|---|
| 1 コアの実時間性 | §3.3。DS-TWR の締切を Wi-Fi 処理が押し出す | Step 3・5 で定量化。優先度・負荷の調整 |
| G26（MISO）がストラップピン | G28=0 でダウンロードモードに入るとき G26 の状態が判定に使われる。MISO は Hi-Z なので浮く | 書き込みは USB-Serial/JTAG 経由（ストラップ不要）で行う。BOOT ボタン運用が要るなら 10 kΩ プルダウンを検討 |
| G11（CS）の 499 Ω 直列抵抗 | CS の立ち上がり・立ち下がりが鈍る（数 ns 程度） | 16 MHz の CS セットアップ時間に対して余裕がある見込み。L6 で確認 |
| 起動時の UART0 出力が CS に出る | ROM・ブートローダのログ | SCK 停止中なので無害の見込み。気になればブートローダログを無効化 |
| FPC の同面／異面 | 逆挿しで電源が信号線に入る | 通電前の導通確認を必須手順にする |
| ESP-IDF の C5 対応が初期段階 | 5 GHz チャネル・国コード、省電力などが整備途中 | 2.4 GHz で始める。IDF の更新に追従 |
| 3.3 V レギュレータ 600 mA | C5 の Wi-Fi 送信ピーク + UWB モジュール | Step 1 で電流実測。5 GHz 送信時も見る |
| 電池での電圧 | VBAT 3.7 V 系は LDO 経由で 3.3 V が出る（UWB の絶対最大 4.0 V に直結しない） | 回路図で確認済み。問題なし |
| LED | WS2812 前提のドライバ | §3.5 |
| ヒープ | SRAM が S3 より 128 KB 少ない | `info` の free heap を S3A と比較 |

## 6. 作業分解と見積もり

| フェーズ | 内容 | 目安 |
|---|---|---|
| A. 机上・ビルド | `boards/stampc5.h`、Kconfig・main.cpp の分岐、`sdkconfig.defaults.<target>` の分離、コア番号ヘルパ、LED の扱い、CI matrix、`verify_docs.py` | 1〜2 日 |
| B. 実機 probe / twr | §4 Step 0〜3 | 1 日 |
| C. anchor / tag 実運用、電池 | §4 Step 4〜6 | 1〜2 日 |
| D. 文書 | WIRING.md に「経路 D: Stamp-C5 FPC 直結」、GETTING_STARTED.md §1.1・§2.1・§3・§4.1（`set-target esp32c5`）、PLAN.md「対応ホストボード」、README「必要なもの」「ディレクトリ構成」、PREBUILT_BINARIES.md、HANDOFF.md §1 | 半日 |

フェーズ A は実機が無くても進められる。B 以降は Stamp-C5 の入手待ち。

## 7. 決めてもらうこと（提案付き）

| 論点 | 提案 |
|---|---|
| 先にやる役割 | **アンカー先行**。電波タスクと Wi-Fi の競合が軽く（アンカーは応答側）、電池駆動の利点が大きい。タグはフェーズ C で |
| 購入品 | Stamp-C5 無印 ×2〜（FPC 直結なら DIP 版もヘッダも不要）。保険として Stamp-C5 DIP ×1 |
| M5StampS3A の扱い | 既定ホストのまま両対応。CI も両方ビルド |
| 5 GHz | 初期スコープ外。Step 7 の任意項目 |
| LED | 初期はハートビート無効で可 |

## 8. 出典

- M5Stack Stamp-C5 製品ページ・ピン図: https://docs.m5stack.com/en/core/Stamp-C5
- Stamp-C5 回路図 V0.2（2026-02-07）: https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1258/S016_StampC5_V0.3_SCH_PDF_20260207_2026_02_07_11_34_57.pdf
- ESP32-C5 データシート（ストラップピン、SPI、GPIO）: https://documentation.espressif.com/esp32-c5_datasheet_en.html
- ESP-IDF v5.5.2 ESP32-C5 SPI master: https://docs.espressif.com/projects/esp-idf/en/v5.5.2/esp32c5/api-reference/peripherals/spi_master.html
- ESP-IDF v5.5.2 リリースノート（ESP32-C5 初期対応）: https://github.com/espressif/esp-idf/releases/tag/v5.5.2
- ESP32-C5 対応状況（Espressif）: https://github.com/espressif/esp-idf/issues/14021
- UWB モジュール側の FPC ピン配置: [`WIRING.md`](WIRING.md) §7.1（正本）
