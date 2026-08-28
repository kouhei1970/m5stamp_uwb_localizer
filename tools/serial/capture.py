#!/usr/bin/env python3
"""Capture serial log until the "PROBE SUMMARY" line (or timeout).

Resets the board via DTR/RTS (works for the ESP-IDF USB-Serial/JTAG
console) and keeps the same port handle open for the whole capture.
Use capture_reenum.py instead for Arduino HWCDC firmware, which
re-enumerates the USB device on reset.

シリアルログを "PROBE SUMMARY" 行が出るまで（またはタイムアウトまで）
採取する。DTR/RTS を叩いてボードをリセットする方式で、ESP-IDF の
USB-Serial/JTAG コンソールでは同じポートハンドルを開いたまま動く。
Arduino の HWCDC ファームはリセットで USB が再列挙されるため、
その場合は capture_reenum.py を使うこと。

Usage / 使い方:
    python capture.py PORT OUT.log SECONDS [noreset]

    PORT     serial device, e.g. /dev/cu.usbmodem101
    OUT.log  output file path (overwritten)
    SECONDS  capture timeout in seconds
    noreset  optional 4th arg; skip the DTR/RTS reset if given
"""
import sys, time, serial
port, out, timeout = sys.argv[1], sys.argv[2], float(sys.argv[3]) if len(sys.argv) > 3 else 40.0
noreset = (len(sys.argv) > 4 and sys.argv[4] == 'noreset')
s = serial.Serial(port, 115200, timeout=1)
# toggle DTR/RTS to reset the ESP32-S3 (USB-Serial/JTAG)
if not noreset:
    s.dtr = False; s.rts = True; time.sleep(0.1); s.rts = False; time.sleep(0.1)
t0 = time.time(); lines = []
with open(out, 'w') as f:
    while time.time() - t0 < timeout:
        raw = s.readline()
        if not raw:
            continue
        line = raw.decode('utf-8', errors='replace').rstrip('\r\n')
        f.write(line + '\n'); f.flush(); lines.append(line)
        if 'PROBE SUMMARY' in line:
            # read a couple more seconds of periodic lines then stop
            t_end = time.time() + 2
            while time.time() < t_end:
                raw = s.readline()
                if raw:
                    f.write(raw.decode('utf-8', errors='replace'))
            break
s.close()
print(f"captured {len(lines)} lines -> {out}")
