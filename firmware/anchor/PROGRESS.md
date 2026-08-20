
---

## ハードウェア構成の確定 (2026-08-20 ユーザ指示)

| 役割 | ボード | 台数 | UWB |
|---|---|---|---|
| **タグ（移動体）** | **M5Stamp S3** | 1 | Stamp UWB-F |
| **アンカー（固定局）** | **M5 AtomS3** | 5 | Stamp UWB-F |

- `firmware/tag` の既定ボード = StampS3（変更なし）
- **`firmware/anchor` の既定ボードを StampS3 → AtomS3 に変更**（ビルド確認済み）

### この割り当てで注意すべき点
- **AtomS3 は空きGPIOが6本しかない**（`boards/atoms3.h` 参照）。
  SCK=G7 / MOSI=G6 / MISO=G5 / CS=G8 / RST=G1 / IRQ=G2 で、
  **WAKEUP と GP7 は未配線**。最小配線 + RST + IRQ は確保できている
- **IRQ が取れているのは重要**。R6（IRQ駆動化）で効くのは
  **レスポンダ側の折り返し時間**（`POLL_RX_TO_RESP_TX_DLY`）であり、
  レスポンダ＝アンカー＝AtomS3 だから。ここが IRQ 化できないと R5 の遅延短縮が頭打ちになる
- StampS3 は GPIO に余裕があり、8本フル配線できる

### 接続方法の変更（同日ユーザ要望）
FPC（0.5mm 12P）ではなく、**モジュールに出ている半田付け用パッドを使う**。
実験段階では FPC より扱いやすいため。→ `docs/SOLDER_PADS.md`（調査中）
