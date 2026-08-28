#!/usr/bin/env python3
"""Summarise a firmware/twr tag serial log.

Parses "SS_RANGE_STAT" lines from a tag log captured with capture.py /
capture_reenum.py and prints the final success rate, the phy: /
DIAG_CHANNEL configuration lines, rsl_dbm/fp_dbm medians for OK vs.
FAIL rows, clock_ppm stats, a breakdown of failure kinds (RXFTO /
RXFSL / RXPHE / RXFCE / ...), and the ranged distance_m stats.

firmware/twr のタグ側シリアルログを要約する。capture.py /
capture_reenum.py で採取したログから "SS_RANGE_STAT" 行を拾い、
最終成功率、phy: / DIAG_CHANNEL の設定行、OK/FAIL 別の rsl_dbm /
fp_dbm 中央値、clock_ppm（相手機との相対クロック偏差）統計、失敗
種別（RXFTO / RXFSL / RXPHE / RXFCE 等）の内訳、測距値 distance_m の
統計を表示する。

Usage / 使い方:
    python twr_stats.py LOGFILE
"""
import re, sys, statistics as st
path = sys.argv[1]
rows = [l for l in open(path, errors='replace') if 'SS_RANGE_STAT' in l]
last = rows[-1] if rows else ''
m = re.search(r'count=(\d+) ok=(\d+) fail=(\d+) rate=([\d.]+)%', last)
print(f"file={path}")
if m:
    print(f"final: count={m[1]} ok={m[2]} fail={m[3]} rate={m[4]}%")
phy = [l.strip() for l in open(path, errors='replace') if 'phy:' in l or 'DIAG_CHANNEL' in l]
for p in phy: print(p)
def nums(key, subset):
    v = []
    for l in subset:
        mm = re.search(key + r'=(-?[\d.]+)', l)
        if mm: v.append(float(mm[1]))
    return v
ok = [l for l in rows if 'last=OK' in l]; ng = [l for l in rows if 'last=FAIL' in l]
for name, sub in (('OK', ok), ('FAIL', ng)):
    r = nums('rsl_dbm', sub); f = nums('fp_dbm', sub)
    if r: print(f"{name}: rows={len(sub)} rsl_dbm median={st.median(r):.1f} min={min(r):.1f} max={max(r):.1f}"
                + (f" | rsl-fp median={st.median([a-b for a,b in zip(r,f)]):.1f} dB" if f else ''))
cp = nums('clock_ppm', ok)
if cp: print(f"clock_ppm: n={len(cp)} min={min(cp):.2f} median={st.median(cp):.2f} max={max(cp):.2f}")
kinds = {}
for l in ng:
    mm = re.search(r'\[([^\]]*)\]', l); k = mm[1] if mm else '?'
    kinds[k] = kinds.get(k, 0) + 1
print("fail kinds (logged rows):", dict(sorted(kinds.items(), key=lambda x: -x[1])))
dist = nums('distance_m', ok)
if dist: print(f"distance_m: n={len(dist)} median={st.median(dist):.3f} min={min(dist):.3f} max={max(dist):.3f}")
