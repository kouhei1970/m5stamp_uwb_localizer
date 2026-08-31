#!/usr/bin/env python3
"""文書とコードの整合チェック（CI から実行される）。

同じ表を複数の文書に置いた結果、訂正が片方だけに入って食い違う事故
（docs/HANDOFF.md の D-2）を機械的に防ぐためのもの。
依存は PyYAML のみ。リポジトリのどこから実行してもよい。
"""
import os,re,sys,subprocess
R=os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
R=os.path.dirname(R) if os.path.basename(R)=='tools' else R
os.chdir(R)
fail=[]

def md_files(include_archive=False):
    out=[]
    for dp,dn,fn in os.walk('docs'):
        if dp.startswith('docs/refs'): continue
        if not include_archive and 'archive' in dp.split(os.sep): continue
        for f in fn:
            if f.endswith('.md'): out.append(os.path.join(dp,f))
    out.append('README.md'); out.append('THIRD_PARTY_LICENSES.md')
    return out

# 1) リンク切れ
print("== 1) リンク切れ ==")
bad=0
for p in md_files(True):
    s=open(p,encoding='utf-8').read()
    for m in re.finditer(r'\]\(([^)#\s]+\.md)(#[^)]*)?\)', s):
        t=m.group(1)
        if t.startswith('http'): continue
        tp=os.path.normpath(os.path.join(os.path.dirname(p),t))
        if not os.path.exists(tp):
            print(f"  BROKEN {p}: -> {t}"); bad+=1
print(f"  切れ {bad} 件")
if bad: fail.append(f"リンク切れ {bad} 件")

# 2) 削除された文書への言及（archive 以外）
print("== 2) 削除済み文書への言及（archive 以外） ==")
gone=['SOLDER_PADS.md','BRINGUP.md','UNITS.md','PLATFORM_TUNING.md']
n=0
for p in md_files(False)+['boards/stamps3.h','boards/stampfly.h']:
    if not os.path.exists(p): continue
    s=open(p,encoding='utf-8').read()
    for g in gone:
        # 「旧 X」という経緯の記述は許容
        for m in re.finditer(re.escape(g), s):
            ctx=s[max(0,m.start()-12):m.start()]
            if '旧' in ctx or '元は' in ctx: continue
            # 統廃合そのものを説明している行（X→Y、Xを統合 等）は正当
            ls=s.rfind('\n',0,m.start())+1; le=s.find('\n',m.start())
            line_txt=s[ls:le if le>0 else len(s)]
            if any(k in line_txt for k in ('→','統合','改組','移動','廃案','削除','統廃合')): continue
            line=s[:m.start()].count('\n')+1
            print(f"  {p}:{line} {g}"); n+=1
print(f"  {n} 件")
if n: fail.append(f"削除済み文書への言及 {n} 件")

# 3) 配線表の重複（DW_CLK を含む文書）
print("== 3) パッド↔FPC 対応表の所在（D-2 の再発防止） ==")
# 「パッド番号と FPC 番号を同じ行に並べた表」= 実際に訂正漏れを起こした表。1文書のみとする
# 「パッド番号と FPC 番号を同じ行に並べた表」の行を数える。
# 信号名を含み、かつ数字だけのセルが 2 つ以上ある行が対応表の行。
# 12 行の表なので 8 行以上で「表がある」とみなす。
SIG=re.compile(r'GND|VCC_3V3|DW_[A-Za-z0-9]+')
NUMCELL=re.compile(r'^\s*\*{0,2}[\d]+(?:,\s*\d+)?\*{0,2}\s*$')
def maptable_rows(text):
    n=0
    for line in text.splitlines():
        if not line.startswith('|'): continue
        if not SIG.search(line): continue
        cells=[c for c in line.strip().strip('|').split('|')]
        if sum(1 for c in cells if NUMCELL.match(c))>=2: n+=1
    return n
holders=[q for q in md_files(False) if maptable_rows(open(q,encoding='utf-8').read())>=8]
for q in holders: print("  ",q)
if len(holders)!=1: fail.append(f"パッド↔FPC 対応表が {len(holders)} 文書にある: {holders}")
# ホスト配線表（FPC->GPIO）は手順書側にもあってよい。値の一致は 7) で機械的に検証する
hosts=[q for q in md_files(False) if 'DW_CLK' in open(q,encoding='utf-8').read()]
print("  （参考）12ピン信号名を含む文書:", hosts)

# 4) boards/*.h のピン値と文書の一致
print("== 4) boards/*.h のピン値 ==")
for b in ['boards/stamps3.h','boards/stampfly.h']:
    s=open(b,encoding='utf-8').read()
    pins=dict(re.findall(r'\.(pin_\w+)\s*=\s*(\-?\d+|UWB_PORT_PIN_UNUSED)',s))
    print(f"  {b}: {pins}")

# 5) Kconfig シンボルの実在
print("== 5) 文書が参照する Kconfig シンボルの実在 ==")
syms=set()
for p in md_files(False):
    syms|=set(re.findall(r'CONFIG_(UWB_[A-Z0-9_]+)', open(p,encoding='utf-8').read()))
defined=set()
for dp,dn,fn in [(a,b,c) for root in ('firmware','components') for a,b,c in os.walk(root)]:
    if 'build' in dp.split(os.sep): continue
    for f in fn:
        if f.startswith('Kconfig'):
            defined|=set(re.findall(r'^\s*config\s+(\w+)', open(os.path.join(dp,f),encoding='utf-8').read(), re.M))
missing=sorted(syms-defined)
for m in missing: print("  MISSING:",m)
print(f"  参照 {len(syms)} / 未定義 {len(missing)}")
if missing: fail.append(f"未定義 Kconfig 参照 {len(missing)} 件")

# 6) CI variant 名の実在
print("== 6) 文書が挙げる CI variant ==")
import yaml
ci=yaml.safe_load(open('.github/workflows/build.yml'))
names={e['name'] for e in ci['jobs']['firmware']['strategy']['matrix']['include']}
print(f"  variant {len(names)} 個")
for p in ['docs/PREBUILT_BINARIES.md']:
    s=open(p,encoding='utf-8').read()
    for m in re.findall(r'`((?:anchor|tag|twr|probe)-[a-z0-9-]+)`', s):
        if '*' in m: continue
        if m not in names:
            print(f"  {p}: 存在しない variant {m}"); fail.append(f"存在しない variant {m}")

# 7) GETTING_STARTED の FPC->GPIO 表が boards/stamps3.h と一致するか
print("== 7) 配線表 vs boards/stamps3.h ==")
import re as _re
hdr=open('boards/stamps3.h',encoding='utf-8').read()
bp=dict(_re.findall(r'\.(pin_\w+)\s*=\s*(\d+)',hdr))
want={'12':bp.get('pin_sck'),'9':bp.get('pin_mosi'),'8':bp.get('pin_miso'),
      '10':bp.get('pin_cs'),'6':bp.get('pin_rst'),'4':bp.get('pin_irq'),'5':bp.get('pin_wakeup')}
gs=open('docs/GETTING_STARTED.md',encoding='utf-8').read()
got={}
for m in _re.finditer(r'^\|\s*(\d+)(?:,\s*\d+)?\s*\|[^|]*\|\s*\*\*G(\d+)\*\*\s*\|', gs, _re.M):
    got[m.group(1)]=m.group(2)
mis=[(k,v,got.get(k)) for k,v in want.items() if v and got.get(k)!=v]
for k,v,g in mis: print(f"  不一致 FPC{k}: boards=G{v} 文書=G{g}")
print(f"  照合 {len(want)} 本 / 不一致 {len(mis)} 件")
if mis: fail.append(f"配線表と boards/stamps3.h の不一致 {len(mis)} 件")

print()
print("==== 結果 ====")
if fail:
    for f in fail: print("  NG:",f)
    sys.exit(1)
print("  すべて OK")
