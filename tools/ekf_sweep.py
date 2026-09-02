#!/usr/bin/env python3
"""tools/ekf_sweep.py — Tag 側の密結合 EKF (components/uwb_loc/src/uwb_ekf.c) を
Python でオフライン再現し、録画済み WebSocket キャプチャに対して process noise
sigma_a (Q)・測距雑音 sigma_r (R)・イノベーションゲート gate を掃引するツール。
ファームウェアは一切変更しない。

Offline replay of the tag's tightly-coupled EKF, so sigma_a (Q) / sigma_r (R) /
gate can be swept over recorded data without touching firmware.

対象は 2D モード (dim=2, z=z_fixed=0, CV モデル nx=4: [px,py,vx,vy]) のみ
(このリポジトリの検証機材はアンカー3台=dim+1台で常にこれしか使わない)。

再現の要点 (uwb_ekf.c を読んで確認・実物の C コードとの bit 精度突き合わせ済み。
下の Ekf2D クラス docstring 参照):
  * 状態遷移・プロセスノイズは transition() をそのまま (連続時間白色雑音, CV)。
  * 観測更新はスカラー逐次 (uwb_ekf_update)。実機 (onRangingSample) も測距1本
    ごとに呼ぶので、"fix" 行の "anchors"[].t を時系列順に1本ずつ流し込めば同じ。
  * イノベーションゲート: res^2 > gate*|gate|*s で棄却 (平方根なし)。
  * 立ち上げ (bootstrap): アンカーごと最新1本を pending に貯め、3経路のいずれかで
    uwb_solve_lv2 相当のスナップショットから初期化する (再構築は max_dt=2s 超過、
    または全棄却が max_rejects=5 回連続で発生)。ここでは uwb_solve_lv2 (Beck
    GTRS+Huber) の代わりに Huber-IRLS Gauss-Newton で近似する
    (_robust_trilaterate())。クリーンな幾何ではサブmmで一致するが、NLOS 混入時の
    再ブートストラップでは数十〜数百mmずれ得る (--validate 参照)。

使い方:
    python3 tools/ekf_sweep.py <raw file> [--sigma-a 0.1,0.5,1.0] [--sigma-r 0.05,0.1] [--gate 3,4]
    python3 tools/ekf_sweep.py <raw file> --validate [--sigma-a 0.5,0.4]
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

# ---------------------------------------------------------------- 読み込み

# WS フレームのバイナリ枠組みに紛れ込んだ JSON テキストを正規表現で拾う。
# 次の "{"v" の直前か、ASCII 印字可能域を外れた1バイト (次フレームのヘッダ) の
# 直前で打ち切る。
JSON_PAT = re.compile(r'\{"v":1,"type":"(?:meas|fix|anchors)".*?\}(?=\s*\{"v"|\s*$|[^\x20-\x7e])', re.S)


def load_records(path: Path) -> list[dict]:
    text = path.read_bytes().decode("utf-8", errors="ignore")
    out = []
    for m in JSON_PAT.finditer(text):
        try:
            out.append(json.loads(m.group(0)))
        except json.JSONDecodeError:
            continue
    return out


def build_cycles(records: list[dict]):
    """anchors 行からアンカー座標、fix 行から (時刻, 測距列, NLS=Lv2 参照, Lv3 参照) を作る。"""
    anchors = None
    for r in records:
        if r.get("type") == "anchors":
            anchors = {a["id"]: (np.array(a["p"][:2], dtype=float), float(a.get("antenna_delay_m", 0.0)))
                       for a in r["anchors"]}
            break
    if anchors is None:
        raise ValueError("anchors 行が見つからない")

    cycles = []
    for r in records:
        if r.get("type") != "fix":
            continue
        ranges = [(a["t"], a["a"], a["d"]) for a in r.get("anchors", []) if a.get("ok")]
        ranges.sort(key=lambda x: x[0])
        nls_ok = bool(r.get("ok"))
        nls_p = np.array(r.get("p", [0.0, 0.0, 0.0])[:2], dtype=float)
        lv3 = r.get("lv3")
        lv3_ok = bool(lv3.get("ok")) if lv3 else False
        lv3_p = np.array(lv3.get("p", [0.0, 0.0, 0.0])[:2], dtype=float) if lv3 else np.zeros(2)
        cycles.append((r["t"], ranges, nls_ok, nls_p, lv3_ok, lv3_p))
    return anchors, cycles


# ---------------------------------------------------------------- 立ち上げ用の簡易ソルバ

def _huber_weight(e: float, sigma: float, k: float = 1.345, k_pos: float = 0.6) -> float:
    """uwb_nls.c huber_weight() と同じ式 (片側損失: 残差が正=NLOS側だけ厳しく)。"""
    sigma = max(sigma, 1e-6)
    kk = k * k_pos if e > 0 else k
    au = abs(e / sigma)
    return 1.0 if au <= kk else kk / max(au, 1e-9)


def _robust_trilaterate(meas: list[tuple[np.ndarray, float]], sigma_r: float,
                         max_iter: int = 30, tol: float = 1e-4):
    """uwb_solve_lv2 (Beck GTRS + Huber) の近似: 線形初期値 + Huber-IRLS Gauss-Newton。
    戻り値は (位置, 共分散 ~ (J^T W J)^-1) か、解けなければ None。"""
    P = np.array([m[0] for m in meas])
    D = np.array([m[1] for m in meas])
    n = len(meas)
    if n < 2:
        return None
    # 線形初期値 (基準アンカーとの差分)。
    A = 2.0 * (P[1:] - P[0])
    b = np.sum(P[1:] ** 2, axis=1) - np.sum(P[0] ** 2) - D[1:] ** 2 + D[0] ** 2
    try:
        x, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        x = P.mean(axis=0)

    H = np.eye(2)
    for _ in range(max_iter):
        dv = x - P
        dist = np.maximum(np.linalg.norm(dv, axis=1), 1e-9)
        jac = dv / dist[:, None]           # dh/dx、行 i = 単位ベクトル
        resid = D - dist                    # 観測 - 予測 (uwb_evaluate と同じ符号)
        w = np.array([_huber_weight(resid[i], sigma_r) for i in range(n)]) / (sigma_r * sigma_r)
        H = (jac * w[:, None]).T @ jac
        g = (jac * (w * resid)[:, None]).sum(axis=0)
        try:
            step = np.linalg.solve(H, g)
        except np.linalg.LinAlgError:
            return None
        x = x + step
        if np.linalg.norm(step) < tol:
            break
    try:
        cov = np.linalg.inv(H)
    except np.linalg.LinAlgError:
        cov = np.full((2, 2), np.nan)
    return x, cov


# ---------------------------------------------------------------- EKF 本体 (uwb_ekf.c 再現)

class Ekf2D:
    """2D・CV モデルの密結合 EKF。状態 x=[px,py,vx,vy]。uwb_ekf.c の逐次スカラー更新を再現する。"""

    def __init__(self, anchors: dict, sigma_a: float, sigma_r: float, gate: float,
                 max_dt: float = 2.0, max_rejects: int = 5):
        self.anchors = anchors
        self.n_enabled = len(anchors)
        self.dim = 2
        self.sigma_a = sigma_a
        self.sigma_r = sigma_r
        self.gate = gate
        self.max_dt = max_dt
        self.max_rejects = max_rejects
        self.reset()

    def reset(self):
        self.x = np.zeros(4)
        self.P = np.eye(4) * 1e6
        self.t = None
        self.initialized = False
        self.rejects = 0
        self.pending: list[tuple[float, str, float]] = []
        self.boot_wait_t0 = None
        self.last_used = None  # 直近のスカラー更新が採用されたか (立ち上げ時は None)

    def position(self) -> np.ndarray:
        return self.x[0:2].copy()

    def sigma_reported(self) -> float:
        tr = self.P[0, 0] + self.P[1, 1]
        return math.sqrt(tr) if tr >= 0 else float("nan")

    def _predict(self, dt: float):
        s2 = self.sigma_a * self.sigma_a
        F = np.eye(4)
        F[0, 2] = dt
        F[1, 3] = dt
        self.x = F @ self.x
        self.P = F @ self.P @ F.T
        d2, d3 = dt * dt, dt * dt * dt
        q_pp, q_pv, q_vv = d3 / 3.0 * s2, d2 / 2.0 * s2, dt * s2
        Q = np.zeros((4, 4))
        Q[0, 0] = Q[1, 1] = q_pp
        Q[0, 2] = Q[2, 0] = Q[1, 3] = Q[3, 1] = q_pv
        Q[2, 2] = Q[3, 3] = q_vv
        self.P = self.P + Q

    def _bootstrap(self, t: float, anchor: str, d: float) -> bool:
        was_empty = len(self.pending) == 0
        self.pending.append((t, anchor, d))
        if was_empty:
            self.boot_wait_t0 = t
        cutoff = t - self.max_dt
        self.pending = [p for p in self.pending if p[0] >= cutoff]
        if not self.pending:
            self.boot_wait_t0 = None
            return False
        seed = {}
        for (_tt, a, dd) in self.pending:  # 古い順 -> 新しい方で上書き (=最新のみ残る)
            seed[a] = dd
        m = len(seed)
        want = self.dim + 2
        if m < want:
            have_all = m >= self.n_enabled
            waited_out = self.boot_wait_t0 is not None and (t - self.boot_wait_t0) >= self.max_dt
            if m < self.dim + 1 or not (have_all or waited_out):
                return False
        meas = [(self.anchors[a][0], dd - self.anchors[a][1]) for a, dd in seed.items()]
        sol = _robust_trilaterate(meas, self.sigma_r)
        if sol is None:
            return False
        p0, cov0 = sol
        self.x = np.zeros(4)
        self.x[0:2] = p0
        self.P = np.zeros((4, 4))
        if np.all(np.isfinite(cov0)):
            self.P[0:2, 0:2] = cov0 + np.eye(2) * 1e-4
        else:
            self.P[0, 0] = self.P[1, 1] = 4.0
        self.P[2, 2] = self.P[3, 3] = 10.0  # 速度: 未知なので大きめの分散から (uwb_ekf.c bootstrap())
        self.initialized = True
        self.rejects = 0
        self.pending = []
        self.boot_wait_t0 = None
        return True

    def _scalar_update(self, anchor: str, d: float) -> bool:
        pos, delay = self.anchors[anchor][0], self.anchors[anchor][1]
        dv = self.x[0:2] - pos
        dist = float(np.linalg.norm(dv))
        if dist < 1e-9:
            return False
        jac = dv / dist
        res = (d - delay) - dist
        h = np.zeros(4)
        h[0:2] = jac
        u = self.P @ h
        s = self.sigma_r * self.sigma_r + jac @ u[0:2]
        if not (s > 0) or math.isnan(s):
            return False
        gate2 = self.gate * abs(self.gate)
        if res * res > gate2 * s:
            return False
        inv_s = 1.0 / s
        self.x = self.x + u * (res * inv_s)
        self.P = self.P - np.outer(u, u) * inv_s
        return True

    def update_one(self, t: float, anchor: str, d: float):
        """測距 1 本 (uwb_ekf_update(..., n=1, ...) 相当)。"""
        if self.t is not None and t < self.t - 1e-9:
            self.last_used = None
            return
        if self.initialized and self.t is not None:
            dt = t - self.t
            if dt > self.max_dt:
                self.initialized = False
            else:
                self._predict(dt)
        if not self.initialized:
            self._bootstrap(t, anchor, d)
            self.t = t
            self.last_used = None
            return
        self.t = t
        self.last_used = self._scalar_update(anchor, d)
        if self.last_used:
            self.rejects = 0
        else:
            self.rejects += 1
            if self.rejects >= self.max_rejects:
                self.initialized = False
                self.rejects = 0
                self.pending = []
                self.boot_wait_t0 = None
                self._bootstrap(t, anchor, d)


# ---------------------------------------------------------------- リプレイ実行

def run_replay(anchors: dict, cycles: list, sigma_a: float, sigma_r: float, gate: float) -> dict:
    ekf = Ekf2D(anchors, sigma_a, sigma_r, gate)
    n = len(cycles)
    ekf_t = np.zeros(n); ekf_p = np.zeros((n, 2)); ekf_ok = np.zeros(n, dtype=bool); ekf_sigma = np.zeros(n)
    nls_t = np.zeros(n); nls_p = np.zeros((n, 2)); nls_ok = np.zeros(n, dtype=bool)
    lv3_p = np.zeros((n, 2)); lv3_ok = np.zeros(n, dtype=bool)
    n_total = 0
    n_rejected = 0
    for i, (t_cycle, ranges, nlsok, nlsp, lv3ok, lv3pp) in enumerate(cycles):
        for (t, a, d) in ranges:
            was_init = ekf.initialized
            ekf.update_one(t, a, d)
            if was_init:
                n_total += 1
                if not ekf.last_used:
                    n_rejected += 1
        ekf_t[i] = t_cycle
        ekf_p[i] = ekf.position()
        ekf_ok[i] = ekf.initialized
        ekf_sigma[i] = ekf.sigma_reported()
        nls_t[i] = t_cycle
        nls_p[i] = nlsp
        nls_ok[i] = nlsok
        lv3_p[i] = lv3pp
        lv3_ok[i] = lv3ok
    return dict(ekf_t=ekf_t, ekf_p=ekf_p, ekf_ok=ekf_ok, ekf_sigma=ekf_sigma,
                nls_t=nls_t, nls_p=nls_p, nls_ok=nls_ok, lv3_p=lv3_p, lv3_ok=lv3_ok,
                n_total=n_total, n_rejected=n_rejected)


# ---------------------------------------------------------------- 指標

def step_stats_mm(p: np.ndarray, ok: np.ndarray):
    pts = p[ok]
    if len(pts) < 2:
        return float("nan"), float("nan")
    d = np.linalg.norm(np.diff(pts, axis=0), axis=1) * 1000.0
    return float(np.median(d)), float(np.max(d))


def find_lag_ms(t: np.ndarray, ekf_p: np.ndarray, ekf_ok: np.ndarray,
                 nls_p: np.ndarray, nls_ok: np.ndarray, max_lag_ms=500, step_ms=25):
    """EKF を 0..max_lag_ms だけ後ろへずらして NLS (Lv2) に最も近づく遅れを探す。"""
    te, pe = t[ekf_ok], ekf_p[ekf_ok]
    tn, pn = t[nls_ok], nls_p[nls_ok]
    if len(te) < 5 or len(tn) < 5:
        return float("nan"), float("nan")
    best_L, best_rms = 0.0, None
    for L_ms in range(0, max_lag_ms + 1, step_ms):
        L = L_ms / 1000.0
        tq = tn - L
        valid = (tq >= te[0]) & (tq <= te[-1])
        if valid.sum() < 5:
            continue
        px = np.interp(tq[valid], te, pe[:, 0])
        py = np.interp(tq[valid], te, pe[:, 1])
        d = np.hypot(px - pn[valid, 0], py - pn[valid, 1])
        rms = math.sqrt(float(np.mean(d ** 2)))
        if best_rms is None or rms < best_rms:
            best_rms, best_L = rms, float(L_ms)
    return best_L, (best_rms if best_rms is not None else float("nan"))


def centered_moving_average(t: np.ndarray, p: np.ndarray, ok: np.ndarray, window_s: float = 0.3):
    """ゼロ位相 (時間で中心をそろえた) 移動平均。窓内に有効点がなければ NaN。"""
    order = np.argsort(t)
    ts = t[order]; ps = p[order]; oks = ok[order]
    half = window_s / 2.0
    out = np.full_like(p, np.nan)
    for i, ti in enumerate(t):
        lo = np.searchsorted(ts, ti - half, side="left")
        hi = np.searchsorted(ts, ti + half, side="right")
        sel = oks[lo:hi]
        if sel.any():
            out[i] = ps[lo:hi][sel].mean(axis=0)
    return out


def tracking_rms_mm(t, ekf_p, ekf_ok, nls_p, nls_ok, window_s=0.3):
    smoothed = centered_moving_average(t, nls_p, nls_ok, window_s)
    valid = ekf_ok & ~np.isnan(smoothed[:, 0])
    if not valid.any():
        return float("nan")
    d = np.hypot(ekf_p[valid, 0] - smoothed[valid, 0], ekf_p[valid, 1] - smoothed[valid, 1])
    return float(math.sqrt(np.mean(d ** 2)) * 1000.0)


# ---------------------------------------------------------------- CLI

def parse_list(s: str) -> list[float]:
    return [float(x) for x in s.split(",") if x.strip() != ""]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("raw_file", type=Path)
    ap.add_argument("--sigma-a", type=parse_list, default=None,
                     help="既定: 0.05,0.1,0.2,0.3,0.4,0.5,0.7,1.0,1.5,2.0")
    ap.add_argument("--sigma-r", type=parse_list, default=None, help="既定: 0.03,0.05,0.10")
    ap.add_argument("--gate", type=parse_list, default=None, help="既定: 3,4")
    ap.add_argument("--mode", choices=["static", "motion"], default=None,
                     help="既定はファイル名に 'static' を含むかで自動判定")
    ap.add_argument("--validate", action="store_true",
                     help="実機の lv3 (fix行) との RMS 差を見る。sigma_a は --sigma-a か既定 0.5,0.4 を試す")
    args = ap.parse_args()

    records = load_records(args.raw_file)
    anchors, cycles = build_cycles(records)
    n_ranges = sum(len(c[1]) for c in cycles)
    print(f"# {args.raw_file}  cycles={len(cycles)} ranges={n_ranges} anchors={list(anchors)}")

    is_static = args.mode == "static" or (args.mode is None and "static" in args.raw_file.name.lower())

    if args.validate:
        sigma_a_list = args.sigma_a or [0.5, 0.4]
        sigma_r = (args.sigma_r or [0.10])[0]
        gate = (args.gate or [3.0])[0]
        print(f"# validate: sigma_r={sigma_r} gate={gate}  (sigma_a candidates: {sigma_a_list})")
        print(f"{'sigma_a':>8} {'n_cmp':>6} {'rms_mm':>8} {'max_mm':>8}")
        for sa in sigma_a_list:
            res = run_replay(anchors, cycles, sa, sigma_r, gate)
            both = res["ekf_ok"] & res["lv3_ok"]
            if both.any():
                d = np.linalg.norm(res["ekf_p"][both] - res["lv3_p"][both], axis=1) * 1000.0
                rms, mx = math.sqrt(float(np.mean(d ** 2))), float(np.max(d))
            else:
                rms, mx = float("nan"), float("nan")
            print(f"{sa:8.2f} {int(both.sum()):6d} {rms:8.2f} {mx:8.2f}")
        print("# 検証メモ (実際に確認した内容):")
        print("#  1) predict/scalar-update/gate/再ブートストラップの遷移は、実物の uwb_ekf.c を"
              "直接リンクした C ドライバに同じイベント列を新規ブートストラップから流し込んで"
              "突き合わせ、bit 精度で一致することを確認済み (sigma_a/sigma_r/gate を振っても再現)。")
        print("#  2) 立ち上げ (bootstrap) の簡易ソルバ (_robust_trilaterate: Huber-IRLS) は、"
              "クリーンな幾何 (このスクリプトの静止/motion_clear データ) では uwb_solve_lv2 と"
              "サブmmで一致するが、NLOS 混入時の再ブートストラップでは数十〜数百mmずれ得る "
              "(gate を極端に絞って再現確認済み)。既定 gate=3/4 ではこのデータで再ブートストラップは"
              "ほぼ発生しない。")
        print("#  3) それでも static/motion_clear で数mm超のずれが残るなら、人工的に1mずらした"
              "初期状態を注入しても2〜4秒で収束することを確認済みなので「録画開始時点で実機の"
              "EKFが既に走っていた」というだけでは説明がつかない。実機が NVS 等で保持していた"
              "実際の sigma_a/sigma_r (コンソールで変更されていた可能性) がここでの既定値と"
              "違う場合に一致しない — このスクリプト単体では実機の実効パラメータを確認できない"
              "(未確認)。")
        return

    sigma_a_list = args.sigma_a or [0.05, 0.1, 0.2, 0.3, 0.4, 0.5, 0.7, 1.0, 1.5, 2.0]
    sigma_r_list = args.sigma_r or [0.03, 0.05, 0.10]
    gate_list = args.gate or [3, 4]

    if is_static:
        print(f"{'sigma_a':>8} {'sigma_r':>8} {'gate':>5} {'std_x_mm':>9} {'std_y_mm':>9} "
              f"{'med_step':>9} {'max_step':>9} {'gated%':>7} {'sigma_med':>10}")
    else:
        print(f"{'sigma_a':>8} {'sigma_r':>8} {'gate':>5} {'med_step':>9} {'max_step':>9} "
              f"{'lag_ms':>7} {'trk_rms':>9} {'gated%':>7}")

    for sa in sigma_a_list:
        for sr in sigma_r_list:
            for g in gate_list:
                res = run_replay(anchors, cycles, sa, sr, g)
                med_step, max_step = step_stats_mm(res["ekf_p"], res["ekf_ok"])
                gated_pct = 100.0 * res["n_rejected"] / res["n_total"] if res["n_total"] else float("nan")
                if is_static:
                    pts = res["ekf_p"][res["ekf_ok"]]
                    std_x = float(np.std(pts[:, 0]) * 1000.0) if len(pts) else float("nan")
                    std_y = float(np.std(pts[:, 1]) * 1000.0) if len(pts) else float("nan")
                    sig_med = float(np.median(res["ekf_sigma"][res["ekf_ok"]]) * 1000.0) if res["ekf_ok"].any() else float("nan")
                    print(f"{sa:8.2f} {sr:8.2f} {g:5.1f} {std_x:9.1f} {std_y:9.1f} "
                          f"{med_step:9.1f} {max_step:9.1f} {gated_pct:7.1f} {sig_med:10.1f}")
                else:
                    lag_ms, _ = find_lag_ms(res["ekf_t"], res["ekf_p"], res["ekf_ok"], res["nls_p"], res["nls_ok"])
                    trk_rms = tracking_rms_mm(res["nls_t"], res["ekf_p"], res["ekf_ok"], res["nls_p"], res["nls_ok"])
                    print(f"{sa:8.2f} {sr:8.2f} {g:5.1f} {med_step:9.1f} {max_step:9.1f} "
                          f"{lag_ms:7.0f} {trk_rms:9.1f} {gated_pct:7.1f}")


if __name__ == "__main__":
    main()
