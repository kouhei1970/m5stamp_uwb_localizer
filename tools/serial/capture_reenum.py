#!/usr/bin/env python3
"""Capture serial log across a USB re-enumerating reset (Arduino HWCDC).

Resets the board via DTR/RTS, closes the port, waits for the USB
device to come back, reopens it, then captures for a fixed duration.
Needed because Arduino's HWCDC (Hardware CDC) USB-serial console drops
and re-creates the /dev entry on reset; capture.py's "keep the handle
open" approach does not survive that. Use capture.py for ESP-IDF
firmware (USB-Serial/JTAG console), which does not re-enumerate.

USB が再列挙されるリセット（Arduino の HWCDC）をまたいでシリアル
ログを採取する。DTR/RTS でリセットしたあと一旦ポートを閉じ、USB
デバイスが再列挙されるのを待って開き直してから、指定秒数だけ採取
する。Arduino の HWCDC（Hardware CDC）USB シリアルコンソールは
リセットで /dev エントリが消えて作り直されるため、capture.py の
「ハンドルを開いたまま待つ」方式では追随できない。ESP-IDF ファーム
（USB-Serial/JTAG コンソール、再列挙しない）には capture.py を使う。

Usage / 使い方:
    python capture_reenum.py PORT OUT.log SECONDS

    PORT     serial device, e.g. /dev/cu.usbmodem101
    OUT.log  output file path (overwritten)
    SECONDS  capture duration in seconds (measured after reopen)
"""
import sys, time, serial
port, out, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3])
s = serial.Serial(port, 115200, timeout=1)
s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False
try: s.close()
except Exception: pass
t0 = time.time(); s = None
while time.time() - t0 < 10:
    try:
        s = serial.Serial(port, 115200, timeout=1); break
    except Exception:
        time.sleep(0.3)
if s is None: print("reopen failed"); sys.exit(1)
n = 0
with open(out, 'w') as f:
    t1 = time.time()
    while time.time() - t1 < timeout:
        try: raw = s.readline()
        except Exception as e:
            time.sleep(0.5)
            try: s = serial.Serial(port, 115200, timeout=1)
            except Exception: pass
            continue
        if raw:
            f.write(raw.decode('utf-8', errors='replace')); f.flush(); n += 1
print(f"captured {n} lines -> {out}")
