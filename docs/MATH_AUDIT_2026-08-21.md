# 行列計算の残存箇所とスカラー化の設計根拠（監査 2026-08-21）

（上位モデルのエージェントによる読み取り専用監査。行番号は上流 `perf/exploit-structure` ブランチ版 = 最終同期後の `components/uwb_loc/src/` と同一）

## 結論
- 上流に残る展開不足: 11 件、うち優先度高 4 件。
  1. `uwb_anchors_coplanar`（`uwb_model.c:140-212`）が毎 fix に 3×3 Jacobi（Lv2 の 15%）→ 閉形式固有値 + 設定時キャッシュ
  2. Beck（`uwb_closed_form.c:250-308`）の 4×4 Jacobi+V（Beck の 68%）→ 3×3 Schur 補元。S は rank 3、Sc = G11 − g gᵀ/G33 = 4·Σw·Cov_w、μ_i = 1/σ_i(Sc)、λ_lo = −σ_min(Sc)。α=‖p‖² を消去: (Sc+λI)p = r0 − λ r1, α = (h4 + λ/2 − gᵀp)/G33, φ = ‖p‖² − α。Newton 各反復は 3×3 余因子解。固有ベクトルは閉形式で取らない（正方形/立方体配置で縮退し不良条件）
  3. EKF update（`uwb_ekf.c:343-388`）: P 対称なら w=u, v=rK なので Joseph 展開 ≡ P −= u uᵀ/s。上三角のみ計算して鏡映。実装済み検証: 77 件同一、CA −25%/CV −22%
  4. GDOP（`uwb_model.c:109-134`）: jac 行は既に単位ベクトル → sqrt 不要。tr(M⁻¹) = (C00+C11+C22)/det
  5. (中) `solve_nls`（`uwb_nls.c:98-149`）: Hessian 6 要素累積 + 余因子（除算 18→2）、Lv0 `:353-361` も
  6. (低) `uwb_lls_trilateration` 余因子化
  7. (中) Beck の `uwb_ata_weighted(A=[−2P,1])` を構造化（G11=4Σw ppᵀ, g=−2Σw p, G33=Σw, h=Σw b a）
  8. (低) `uwb_resolve_mirror` の R C Rᵀ → C − 2n(Cn)ᵀ − 2(Cn)nᵀ + 4(nᵀCn)nnᵀ
  9. (低) `uwb_ekf_predict`/`transition` を CV/CA × nd∈{2,3} で展開
  10. (中) `uwb_evaluate :92` の除算 3 回 → inv=1/d
  11. (低) `uwb_crlb_at`/`uwb_gdop_at` 余因子化
  → 実施後、`uwb_linalg.c` の一般 LU / コレスキー / Jacobi / ata はホットパスから消える。
- 測量: MDS をピボット付きコレスキー 3 段の逐次三辺測量に置換すると 8×8 固有分解が不要。スクラッチ実装で 281 件 0 失敗（double/float）、LM 反復数不変、≈20k flops+400 sqrt → ≈300 flops+3 sqrt、スタック −1KB。`fit_up`/`neighbor_plane` の 3×3 固有分解も閉形式化 → `uwb_survey_sym_eig` 削除可。正規方程式は 3×3 ブロック構造: build_normal 49→7 乗算/リンク、下三角パック（325 要素、スタック −2.4KB）、3×3 カーネルのブロックコレスキー。Schur 補元(δ)・CG は不採用。8 固定の完全展開は不採用（n=4 で 7 倍の無駄、I-cache 圧迫）。
- A-1（同一平面 × δ）は初期値方式では直らない（三辺測量版でも δ=0.02 で degenerate=0 を実測）。修正は LM 収束後の最終形状の 3×3 PCA（閉形式）。
- float ビルドでは A-9（LM_MIN/RIDGE_ABS が eps スケールでない）でコレスキー失敗 221 回・呼出 2.5 倍 → 線形代数より先に直す。

## ホスト実測（M3 Ultra, double）
| プリミティブ | us/回 |
|---|---:|
| uwb_anchors_coplanar (n=6) | 0.252 |
| uwb_sym_eig 4×4+V (Beck) | 0.547 |
| uwb_sym_eigvals 3×3 | 0.196 |
| 3×3 固有値 閉形式 | 0.020 |
| uwb_gdop_from_jac (n=6) | 0.095 |
| uwb_inverse 3×3 LU | 0.039 |
| uwb_solve_lin 3×3 LU | 0.022 |
| 3×3 対称 余因子 solve + tr(inv) | 0.003 |
| uwb_solve_lv2 (n=6) | 1.653 |

## uwb_math に置く部品（設計）
- vec3: dot, cross, norm, normalize(inv 1回), axpy
- sym3（6 要素パック）: eigvals 閉形式（q, p, φ）、solve 余因子（除算 1）、inverse/trace、min-eigvec（(M−λI) の 2 行の外積）、rank 判定（固有値比）
- rank-1: P −= u uᵀ/s（上三角→鏡映）、対称 rank-1 加算
- block Cholesky n≤8（3×3 カーネル: chol3, trsv3, gemm3）+ 縁取り 1 行（δ）
- 2×2 の同等物（dim=2 経路）
- 置かないもの: 一般 LU、一般 Jacobi（テスト用参照実装は tests/ 側に置く）

## 推奨実装順
1. coplanar 閉形式 + キャッシュ、GDOP の sqrt 削除 + 余因子
2. EKF update の rank-1 ダウンデート（+ nd=3/CV/CA 展開）
3. Beck 3×3 Schur 補元 + ata 構造化 + NLS 余因子化
4. 測量: 三辺測量初期値（スクラッチ `trilat_init`）、fit_up/neighbor_plane 閉形式、A-1 の最終形状 PCA
5. 測量: build_normal 7 乗算化 + 下三角パック + ブロックコレスキー、A-9 の eps スケーリング

## 実施状況 (2026-08-21、`components/uwb_loc` の書き換え)

`components/uwb_loc/` を `components/uwb_math/` (スカラー展開の 2x2 / 3x3) の上に
書き換えた。`uwb_linalg.c/.h` (一般 LU / コレスキー / Jacobi / ata) は本体から
削除し、凍結時点 (commit 4298c08) の写しを `tests/host/loc/ref/` に参照実装
として残した (回帰テスト `tests/host/loc/test_regress.c` の比較対象)。

| # | 項目 | 状態 | 実施内容 |
|--:|---|---|---|
| 1 | coplanar の Jacobi → 閉形式 + キャッシュ | 済 | `uwb_sym3_eigvals` / `uwb_sym3_min_eigvec`。結果を `uwb_config.plane` に持ち、`uwb_config_init` / 新設 `uwb_config_refresh` で作る。使用時に (enabled, 座標) の写しと比較して一致したときだけ使う (refresh 忘れでも正しい) |
| 2 | Beck 4x4 → 3x3 Schur 補元 | 済 | 重心座標で $(S_c+\lambda I)q=r_0$、$\varphi=\lVert q\rVert^2-\bar b-\lambda/2W$。$\lambda_\text{lo}=-\sigma_{\min}$ は閉形式固有値、各反復は 3x3 LDLᵀ 1 回 (除算 3、sqrt 0)。固有ベクトルは作らない。余因子逆行列 (除算 1) は float で精度が出ず不採用 (下記) |
| 3 | EKF update → rank-1 ダウンデート | 済 | `uwb_symn_rank1_downdate(P, nx, u, 1/s)`。ゲートは $\nu^2>\gamma^2 S$ で sqrt 無し。nd / CV / CA 別の完全展開は未 (ループのまま) |
| 4 | GDOP の sqrt 削除 + 余因子 | 済 | 零行だけ飛ばして `uwb_sym3_trace_inverse` (dim=2 は sym2) |
| 5 | NLS / Lv0 の Hessian 6 要素累積 | 済 | `normal_eq` で 6 要素に累積し LDLᵀ (`uwb_internal.h`、除算 3) で解く。収束判定も二乗比較で sqrt 無し。特異判定は旧 LU と同じ「厳密に 0 のときだけ失敗」(退化幾何で ok=1・巨大 cov を返す旧挙動を保つ) |
| 6 | LLS の余因子化 | 済 (LDLᵀ) | 行の sqrt(sw) 倍をやめ重み sw² で正規方程式に累積、LDLᵀ で解く。階数落ち (同一平面) は零空間方向へのリッジで最小ノルム解 (numpy lstsq と同じ。旧 1e-12 リッジは κ·1e-12 の偏りを生んでいた: 4 台配置で 5e-7 m) |
| 7 | Beck の ata 構造化 | 済 | 重心座標で $S_c=4\sum w\,dd^\top$、$r_0=-2\sum w\,b'd$ を直接累積 ($g=0$ になるので #2 に吸収) |
| 8 | resolve_mirror の $RCR^\top$ | 済 | `uwb_sym3_reflect` |
| 9 | EKF predict の CV/CA 展開 | 一部 | `transition` の除算 (CV 4 / CA 12) を定数倍に。F P Fᵀ のループ展開は未 |
| 10 | `uwb_evaluate` の除算 3 → 1 | 済 | `inv = 1/d` |
| 11 | `uwb_crlb_at` / `uwb_gdop_at` の余因子化 | 済 | sym3/sym2 累積 + `trace_inverse` |
| + | 物理ゲートの三角不等式 | 済 (追加) | アンカー間距離を二乗のまま比較 (組ごとの sqrt N(N−1)/2 回を削除) |

### float の既知の失敗の原因

`make float` が gcc / `-ffp-contract=off` で `Beck が解けない` になっていた原因は
**lo 探索の刻み**。旧実装は `lo = lam_lo + 1e-9 * span` から始めるが、
1e-9 は float の eps (1.2e-7) より小さいので `lo == lam_lo` となり、
`beck_phi` が極 ($1+\lambda\mu_{\max}=0$) を踏んで 0 を返す。次の
`lo = lam_lo + (lo - lam_lo) * 10` も差が 0 なので一歩も動けず、60 回で
失敗する。FMA 縮約があると `1 + lam*mu` が融合されて丸め前の値 ($\ne 0$、
$|den| > 10^{-12}$) になり、偶然通っていただけ。ニュートンの収束判定や
二分法の床 (`1e-14`、float では満たせないが反復上限で止まる) は直接の原因では
ない。新実装は刻みを $64\,\varepsilon\,\sigma_{\max}$ と eps 比例にし、極を踏んだ
評価は「$\varphi=+\infty$ 側」として扱うので構造的に解消する
(clang / gcc-16 / `-ffp-contract=off` / ASan+UBSan の float でいずれも通る)。

### 余因子解を採らなかった箇所 (float の精度)

監査では 3x3 を余因子 (除算 1) で解く前提だったが、余因子展開の行列式は誤差
eps·σ_max³ を持ち後退安定でない。回帰テスト (float) で
- Beck: κ ≈ 200 の配置で 2e-4 m (旧 4x4 固有分解版は 3e-7 m)。σ_min/σ_max ≈ 1e-4
  の配置では行列式が丸め誤差に埋もれ、根の近傍が「解けない領域」になった
- LLS (4 台、行 3 本、κ ≈ 1e5): 解が最大 42 m 狂った (旧 LU は正しかった)

ため、Beck / NLS / LLS の線形方程式はピボット無し LDLᵀ (`uwb_internal.h`、
sqrt 0、除算 3) にした。同じ配置で Beck 4e-6 m、LLS は旧 LU と同等。
余因子は GDOP / CRLB (`uwb_sym3_trace_inverse`: 単位ベクトルの和で条件が良く、
精度要求も低い) と coplanar の固有値だけに使う。

### 除算・sqrt 回数 (1 fix あたり、N=6、double。反復回数は乱数 2000 試行の平均)

評価回数の実測: Lv2 1 fix あたり Gauss-Newton 11.2 反復・`uwb_evaluate` 78 回
(chi2 再解を含む。新旧同じ)、Beck の $\varphi$ 評価 旧 8.0 回 / 新 11.2 回、
旧 Jacobi 回転 Beck 24 回 + coplanar 3 回。

| 関数 | 旧 除算 | 旧 sqrt | 新 除算 | 新 sqrt | 備考 |
|---|---:|---:|---:|---:|---|
| `uwb_evaluate` (1 本) | 3 | 1 | 1 | 1 | 距離の sqrt は残る |
| `uwb_gdop_from_jac` | 30 | 7 | 1 | 1 | |
| `uwb_anchors_coplanar` (非同一平面) | 13 | 8 | 0 (cached) / 2 | 0 / 2 + acos + cos | cached は比較のみ |
| `uwb_beck_gtrs` | ≈131 | ≈52 | ≈42 | 2 (+acos, cos) | 旧: コレスキー 4 sqrt、Jacobi 24 回転 × (3 除算 + 2 sqrt)、φ 8 回 × 4 除算。新: 1/σ² 6 + 1/W + 固有値 1 + φ 11.2 回 × LDLᵀ 3 |
| `uwb_lls_trilateration` | 18 | 5 | 14 | 0 | 1/σ² 6 + 調和平均 5 + LDLᵀ 3 |
| `solve_nls` 1 反復 (行列部分) | 7 | 1 | 4 | 0 | LU 6 + 減衰 1 → LDLᵀ 3 + 減衰 1。収束判定の sqrt も無し |
| `uwb_solve_lv0` | ≈98 | ≈28 | ≈31 | ≈9 | |
| `uwb_solve_lv2` | ≈660 | ≈175 | ≈320 | ≈84 | 残りはほぼ `uwb_evaluate` 78 回分 (距離 sqrt と 1/σ²) と Huber の除算 |
| `uwb_ekf_update` (N=6, CV) | ≈103 | ≈27 | ≈20 | ≈15 | 観測 1 本: 除算 nx+3 → 2、sqrt 2 → 1 |
| `uwb_ekf_predict` | CV 4 / CA 12 | 0 | 0 | 0 | |
| 物理ゲート (N=6) | 0 | 15 | 0 | 0 | |

### ホスト実測 (M3 Ultra、clang -O2、us/回。`tools/bench_loc`)

| 関数 | 条件 | double 前 | double 後 | float 前 | float 後 |
|---|---|---:|---:|---:|---:|
| uwb_solve_lv0 | N=6 | 0.489 | 0.223 | 0.637 | 0.214 |
| uwb_solve_lv1 | N=6 | 1.588 | 0.629 | 2.535 | 1.020 |
| uwb_solve_lv2 | N=6 | 1.593 | 0.636 | 2.529 | 1.028 |
| uwb_solve_lv2 | N=8 | 1.882 | 0.738 | 1.875 | 1.322 |
| uwb_beck_gtrs | N=6 | 0.850 | 0.300 | 0.732 | 0.228 |
| uwb_ekf_predict | CA(nx=9) | 0.108 | 0.097 | 0.120 | 0.111 |
| uwb_ekf_update | CV(nx=6), N=6 | 0.463 | 0.286 | 0.414 | 0.263 |
| uwb_ekf_update | CA(nx=9), N=6 | 0.560 | 0.338 | 0.559 | 0.328 |
| uwb_anchors_coplanar | N=6 (旧 / 新 nocache) | 0.066 | 0.090 | 0.226 | 0.071 |
| uwb_anchors_coplanar | N=6 cached | - | 0.007 | - | 0.007 |
| uwb_gdop_from_jac | N=6 | 0.093 | 0.032 | 0.106 | 0.035 |

(double の coplanar nocache が旧より遅いのは、ベンチの配置では散布行列が
ほぼ対角で Jacobi が 1 sweep で終わるため。N=8 では 0.233 → 0.091。
閉形式は acos/cos を必ず呼ぶので定数コストがある。)

### スタック使用量 (xtensa-esp32s3-elf-gcc 14.2 -O2 -fstack-usage、bytes)

| 関数 | double 前 | double 後 | float 前 | float 後 |
|---|---:|---:|---:|---:|
| uwb_beck_gtrs | 1776 | 720 | 896 | 368 |
| uwb_ekf_update | 592 | 256 | 320 | 160 |
| solve_nls | 880 | 720 | 512 | 400 |
| uwb_lls_trilateration | 864 | 672 | 480 | 384 |
| uwb_solve_lv0 | 1264 | 1024 | 704 | 592 |
| uwb_gdop_from_jac | 400 | 256 | 224 | 128 |
| uwb_anchors_coplanar | 432 | 48 (+plane_compute 160) | 208 | 32 (+96) |
| uwb_resolve_mirror | 352 | 176 | 176 | 80 |

### 新旧比較 (tests/host/loc/test_regress.c)

参照実装 = 凍結版 (`tests/host/loc/ref/`、`ref_` 接頭辞でリンク) と同じ乱数
シナリオ (スナップショット 2000 試行: 4〜8 台、15% 完全同一平面、10% ほぼ
同一平面、ノイズ 0〜0.3 m、NLOS 40%、欠測 30%、dim=2 20%、z_bounds 30%。
EKF 1200 軌跡 × 100 ステップ: CV/CA × nd 2/3、1 本ずつの区間、ギャップ、
瞬間移動) を流して比較。`make -C tests/host/loc test` で test_uwb の後に走る。

合否の規則 (strict 列): `ok` / `excluded` / `n_used` / `ambiguous` は完全一致、
位置は許容差以下。許容差を超えたら目的関数 (double で評価) で「新が旧より
悪くない」ことを確かめ、悪くなければ `p_3d_path` (報告のみ)、悪ければ失敗。
次の試行は報告のみ (`~` 付きカテゴリ): 同一平面判定 (使った観測のアンカー
で固有値比 < 0.05²) が立つ配置 (旧 Beck は特異な G をコレスキーの丸めで
たまたま通すことがあり、その解から始めた NLS は別の局所解・鏡像に収束しうる。
旧 LLS は κ·1e-12 の偏り)、新旧で Beck の成否が違う試行、EKF は位置が一度
許容差を超えて分岐した以降のステップ (フィルタは過去に依存)。

| ビルド | 結果 | strict 最大差 (p) | 備考 |
|---|---|---|---|
| clang double | OK 591418 件 | Lv0/LLS 0 (≤ 1e-6 判定)、Lv1/Lv2 ≤ 1e-7、Beck ≤ 1e-7、EKF ≤ 1e-7、GDOP/CRLB 2.4e-14、coplanar 2.7e-15 | Beck の成否不一致 24 件 (すべて完全同一平面。旧がまぐれで成功)。EKF の分岐 0 軌跡 |
| gcc-16 double | OK 591782 件 | 同上 | |
| clang float (FMA) | OK 591184 件 | Lv1 path 0.10 (152 件、目的関数は悪くない)、Lv2 path 9e-3 (35 件)、Beck path 4.2e-4 (28 件)、Lv0/LLS 0 超過 | GN は float のコスト比較が丸めで割れて反復回数が 610/1393 件で違う |
| clang float `-ffp-contract=off` / gcc-16 float | OK 525169 件 | Lv1 path 8.9e-3、Lv2 path 2.3e-2、Beck path 2.1e-4 | 旧 Beck は 1295 件で失敗 (既知の不具合)、新は全部解ける。EKF は 84/1200 軌跡が立ち上げの違いで分岐 (分岐後のステップで真値に近いのは新 297 / 旧 258 で互角) |
| ASan+UBSan double / float | OK | 同上 | 検出 0 |

報告のみの `~` カテゴリの最大差 (double): Lv0~ 9.8e-3 (旧リッジの偏り)、
Lv1~ 2.6、Lv2~ 3.3 (初期値が違う 24 試行のうち別の局所解 / 鏡像に収束したもの。
`excluded` の不一致 1 件)、EKF~ は同一平面の軌跡。

## 実施状況 (2026-08-21、`components/uwb_survey` の書き換え)

| 項目 | 実装 |
|---|---|
| MDS → 逐次三辺測量 | `trilat_init()`: ノード 0 基準の Gram 行列にピボット付きコレスキーを 3 段。除算 3 + sqrt 3（旧 8×8 Jacobi ≈ 除算 450 + sqrt 300）。縮退判定は 3 段目ピボット比と残差対角 |
| `fit_up` / `neighbor_plane` | `uwb_sym3_eigvals` / `uwb_sym3_min_eigvec` / `uwb_sym3_rank`。A-6 は `(M+νI)u=b` を `uwb_sym3_solve_shifted` で解きながら ‖u‖=1 の Newton（厳密解） |
| 正規方程式 | `uwb_bchol`（リンクごと `add_pair_outer` 7 乗算、縁取りに δ、LM 減衰は `damp_diag`）。`uwb_survey_dense.c/.h` 削除。1 試行の除算 434 → 53 |
| A-9 | `LM_MIN` / `RIDGE_ABS` = `UWB_MATH_RANK_TOL`（eps 比例）。float のコレスキー失敗 377 → 0 回 / 400 件 |
| A-1 | 最終形状の PCA だけではノイズ下で捕まらない（見かけの厚み ≈ sqrt(2dσ)）ので、最良近似平面へ射影して面内で LM を解き直し、残差 RMS < max(6cm, 3×rms3d) なら縮退。6 台同一平面 × δ∈{0.02,0.05,0.15} × 5cm ノイズ: 150/150 検出。本物の 3 次元配置での偽陽性 ≈1%（n=6, σ=5cm）。80m×3m×2.4m の細長い配置は縮退扱い（z の誤差が実用にならないという物理判断）。冗長度 0 では行わない |
| A-7 / A-8 | `DEGEN_RATIO` 1e-3 → 1e-5。片方向 NaN のリンクは有効な側だけ採用 |
| A-3 / A-5 | `uwb_survey_result` に `redundancy` / `n_heights` / `frame_determined` / `delay_suspect` を追加 |
| スタック（xtensa -O2） | double 9.4KB → **6.7KB**、float 5.0KB → **3.7KB**（ヘッダ注記は「約 6.7KB / 12KB 以上のタスクから」） |
| テスト | 281 → 365 件（A-1 15、A-6/7/8 61、診断フィールド 21、偽陽性上限 4。旧 dense ソルバ単体 17 件は削除）。clang / gcc-16 / `-ffp-contract=off` / ASan+UBSan の double・float すべて通過 |
| 新旧比較（乱数 400 件） | `ok`/`degenerate` 一致 400/400。冗長度>0・無雑音: 座標 1e-12（double）。ノイズ付きは座標 4cm 差（A-6 の厳密解の効果。形状と δ は 1e-9 で一致）。LM 反復 6.7 → 6.6 回/solve、コレスキー呼出 94.6 → 141.2 回/solve（平面再フィット 2 本分。1 回あたりは除算 350 → 25） |
| 未対応 | A-2（外れ値の leave-one-out）、A-4（キラリティ入力）。根本対策として「実測高さを LM の観測に入れる」を `SURVEY_SPEC.md` §2[3'] に記載 |

`uwb_math` に欲しい関数（現在は各コンポーネント内 static）: sym3 の LDLᵀ 系（loc は余因子解が float で後退安定でなかったため LDLᵀ に切替）、`sym3_null_vector`、球面拘束付き 3×3 最小化 `solve_sphere`（Beck の λ 探索と同型）、sym2 の shifted 版。次回 uwb_math を触るときに統合する。

## 統合状況 (2026-08-21、線形代数ヘルパの `uwb_math` への統合)

上の「欲しい関数」を `components/uwb_math/` に実装し、`uwb_loc` 側の static
inline (`uwb_internal.h` の LDLᵀ) を削除して置き換えた。**`uwb_survey` 側の
差し替えも済（2026-08-21）**。上の表 #5 と「余因子解を採らなかった箇所」で
`uwb_internal.h` と書いてある LDLᵀ は、いまは `uwb_math.h` にある。

### 追加した関数 (`uwb_math.h`)

| 関数 | 由来 | 置き場 | 除算 / sqrt | 備考 |
|---|---|---|---|---|
| `uwb_sym3_ldl` / `uwb_sym3_ldl_factor(s, shift, require_pd, f)` / `uwb_sym3_ldl_solve(f, b, x)` | `uwb_loc/uwb_internal.h` の `uwb_ldl3_*` をそのまま (演算順序を変えず) | **`static inline` (ヘッダ)** | 3 / 0、0 / 0 | `require_pd=1`: ピボット ≤ 0 で失敗。`=0`: 0 / NaN / ±inf だけ失敗 (LU 互換) |
| `uwb_sym3_ldl_inverse(s, shift, require_pd, inv)` | 同上 | 関数 | 3 / 0 | inv と s は同じ配列でもよい |
| `uwb_sym2_ldl` / `uwb_sym2_ldl_factor` / `uwb_sym2_ldl_solve` / `uwb_sym2_ldl_inverse` | 同上 (`uwb_ldl2_*`) | factor/solve は `static inline`、inverse は関数 | 2 / 0 | |
| `uwb_sym3_solve_tol(s, b, x, rel_tol)` / `uwb_sym2_solve_tol` | 新規 | 関数 | 1 / 0 | 余因子解の特異判定しきい値を引数で。`rel_tol = UWB_MATH_SING_TOL` は `uwb_sym3_solve` とビット一致、`rel_tol = 0` は「det が厳密に 0 のときだけ失敗」(±inf も弾く) |
| `uwb_sym3_adjugate(s, adj)` | 新規 (内部 `sym3_cofactors` の公開) | 関数 | 0 / 0 | det を返す。rank 2 なら adj = λ0λ1 n nᵀ |
| `uwb_sym2_mv` / `uwb_sym2_solve_shifted` / `uwb_sym2_inverse_shifted` | 新規 (sym3 版の 2x2) | 関数 | 0、1、1 | shift = 0 は solve / inverse とビット一致 |
| `uwb_sym3_null_vector(s, n)` | `uwb_survey.c` の `sym3_null_vector` | 関数 | 2 / 1 | 行を max\|s\| で正規化してから外積 (survey 版は生の行。1e-6 スケールだと外積 ≈ 1e-12 が `UWB_MATH_TINY` に掛かるため)。選択規則 (3 通りの最大ノルム) と符号規約は同じ。NaN を弾く |
| `uwb_sym3_principal_axis(s, v)` | `uwb_survey.c` `fit_up` の rank-1 分岐 (最大ノルム行の正規化。関数にはなっていなかった) | 関数 | 1 / 1 | 符号を「絶対値最大の成分が正」に正準化する点だけ違う (survey の使い方 `coef = e0·b/λ0`、`w = ẑ − (e0·ẑ)e0` は符号に不変) |
| `uwb_sym3_solve_sphere(M, b, lam_min, u, out_nu)` | `uwb_survey.c` の `solve_sphere` | 関数 | 反復ごと 4 / 1 | Newton の骨格・開始点 ν = 0・収束判定 16 eps・極の手前の二分はそのまま。線形部分を余因子 `uwb_sym3_solve_shifted` 2 回 (除算 2) から **LDLᵀ 1 回 + solve 2 回** (除算 3) に変えた (Beck と同じ部品。float で後退安定)。hard case で二分が極に達し LDLᵀ が失敗したら (float では ν が丸めで −λ_min ちょうどになる) 直前の u を正規化して返す (survey 版は 0 を返していた)。Beck の λ 探索と同型であることをヘッダに明記 (Beck は φ に線形項が付き rtsafe で括る)。今回 Beck は触っていない |

### `uwb_loc` 側の置き換え

- `components/uwb_loc/src/uwb_internal.h`: `uwb_ldl3` / `uwb_ldl2` 構造体と static inline 7 関数 (`uwb_ldl3_ok` 含む) を削除。`UWB_EPS` と内部 API の宣言だけ残る。
- `uwb_closed_form.c`: `sc_solve_pd` (LLS) と `beck_eval` (Beck の φ) が `uwb_sym3_ldl_factor/solve` (dim=2 は sym2) を呼ぶ。
- `uwb_nls.c`: `sym_inverse_lenient` / `sym_solve_lenient` が `uwb_sym3_ldl_inverse` / `uwb_sym3_ldl_factor/solve` を `require_pd = 0` で呼ぶ (旧 LU 互換の緩い特異判定はそのまま。`uwb_sym3_solve_tol(…, 0)` には切り替えていない: 数値を変えないため)。
- 数値の同一性: `tests/host/loc` (test_uwb 77 件 + test_regress) を clang double / clang float (FMA) / clang float `-ffp-contract=off` / gcc-16 double / gcc-16 float / clang strict の 6 構成で統合前後に流し、**出力ログが 6 構成とも完全一致** (件数 591418 / 591184 / 525169 / 591418 / 525169 / 591418、カテゴリ別の最大差まで同じ)。

### static inline にした判断 (tools/bench_loc、M3 Ultra、clang -O2、us/回、3 回の最小)

LDLᵀ の factor / solve を最初は `uwb_math.c` の関数にしてベンチしたところ、
Beck の φ 評価 (1 fix に ≈ 11 回 × factor 1 + solve 2) と NLS の反復ごとの
呼び出しコストが見えた (下表「関数化」)。ヘッダの `static inline` に置くと
統合前と同じコード生成になり (vec3 と同じ扱い)、差は ±1.5% (ノイズ) に収まった。
`*_ldl_inverse` は共分散 1 回ぶんなので関数のまま。`uwb_solve_lv0` の N=4〜6 は
ベンチの先頭行で CPU のウォームアップに掛かり単独計測では 1.2〜2.6 倍に見えたが、
統合前バイナリ (HEAD を `git archive` でビルド) と交互に 3 回ずつ走らせると
1.00〜1.01 倍だった (下表の double の lv0 / lv2 / Beck の「前」「inline」はこの
交互計測。それ以外の列は非交互の 3 回の最小)。

| 関数 | 条件 | double 前 | double 関数化 | double inline | float 前 | float 関数化 | float inline |
|---|---|---:|---:|---:|---:|---:|---:|
| uwb_solve_lv0 | N=6 | 0.177 | 0.208 (+17%) | 0.178 | 0.177 | 0.212 (+20%) | 0.217 |
| uwb_solve_lv0 | N=8 | 0.222 | 0.229 | 0.222 | 0.223 | 0.223 | 0.221 |
| uwb_solve_lv1 | N=6 | 0.620 | 0.689 (+11%) | 0.625 | 1.005 | 1.078 (+7%) | 1.011 |
| uwb_solve_lv2 | N=6 | 0.623 | 0.682 (+10%) | 0.622 | 1.039 | 1.101 (+6%) | 1.016 |
| uwb_solve_lv2 | N=8 | 0.729 | 0.785 (+8%) | 0.727 | 1.333 | 1.394 (+5%) | 1.303 |
| uwb_beck_gtrs | N=6 | 0.289 | 0.343 (+18%) | 0.288 | 0.234 | 0.275 (+18%) | 0.227 |
| uwb_ekf_update | CV(nx=6), N=6 | 0.268 | 0.269 | 0.267 | 0.249 | 0.243 | 0.242 |
| uwb_gdop_from_jac | N=6 | 0.029 | 0.032 | 0.031 | 0.033 | 0.033 | 0.036 |

(EKF / coplanar / GDOP は LDLᵀ を使わないので変化なし。float の lv0 N=6 の
0.217 は非交互計測でウォームアップの影響が残った値。)

### スタック使用量 (xtensa-esp32s3-elf-gcc 14.2 -O2 -fstack-usage、bytes、UWB_MAX_ANCHORS=8)

| 関数 | double 前 | double 後 | float 前 | float 後 |
|---|---:|---:|---:|---:|
| uwb_beck_gtrs | 720 | 720 | 368 | 368 |
| beck_eval | 128 | 128 | 64 | 64 |
| uwb_lls_trilateration | 672 | 672 | 384 | 384 |
| solve_nls | 720 | 720 | 400 | 400 |
| uwb_solve_lv0 | 1024 | 1024 | 592 | 592 |
| uwb_solve_lv2 | 32 | 32 | 32 | 32 |
| cov_from_hessian | 144 | 96 (+ uwb_sym3_ldl_inverse 96) | 96 | 80 (+ 64) |
| uwb_sym3_solve_sphere (新) | - | 176 | - | 96 |
| uwb_sym3_null_vector (新) | - | 128 | - | 80 |
| uwb_sym3_principal_axis (新) | - | 128 | - | 64 |
| uwb_sym3_solve_tol (新) | - | 144 | - | 80 |

ホットパスは inline なので前後同一。`cov_from_hessian` だけ inverse が関数呼び出しに
なり、呼び出し連鎖の合計は 144 → 192 (double) / 96 → 144 (float) に増えるが、
fix ごとに 1 回の冷たい経路で、`uwb_solve_lv0` の 1024 / 592 の内側に収まる
(lv0 のフレームは変わっていない)。float の `uwb_math.c` では呼び出し元が増えた
`sym3_maxabs` が gcc に inline されなくなり `uwb_sym3_solve` / `uwb_sym3_inverse`
が 64 → 80 になった (uwb_loc はこれらをホットパスで使わない)。

### テストとビルド

- `tests/host/math`: 3439 → **10781 件** (`test_sym3_ldl` / `test_sym2_ldl` / `test_sym3_solve_tol` / `test_sym3_adjugate` / `test_sym2_extras` / `test_sym3_null_vector` / `test_sym3_principal_axis` / `test_sym3_solve_sphere`)。参照 LU / 余因子解との比較、shift、require_pd の意味論 (不定値行列は慣性法則で `require_pd=1` が必ず失敗)、ピボット無しゆえの失敗 (s00 = 0 の非特異行列)、スケール 1e-6 / 1 / 1e6、NaN / ±inf、構造的配置 (正方形・三角形・一直線)、球面拘束は既知の解 u* と乗数 ν* から b を作って復元 + Lagrange 条件 + 乱数単位ベクトルに負けないこと + hard case。`make test / strict / float / sanitize` に加え gcc-16 (`-Werror`) の double / float、clang float `-ffp-contract=off`、float の ASan+UBSan すべて通過。LDLᵀ の誤差は eps·κ·‖x‖ で解のノルムに比例するので、比較は成分ごとの相対ではなく `|Δx|/max|x|` で見ている (float の不定値行列で小さい成分が 1e-4 に見えた)。
- `tests/host/loc`: 上記 6 構成でログ完全一致。
- `firmware/tag` / `firmware/soltest`: `idf.py build` 成功、警告 0 (uwb_math.c / uwb_closed_form.c / uwb_nls.c の再コンパイルを確認)。

### `uwb_survey` 側の差し替え (済、2026-08-21)

`components/uwb_survey/src/uwb_survey.c` の static を下表のとおり `uwb_math`
の関数に差し替えた。対応表:

| survey 側 (static / inline コード) | uwb_math | 差し替え時の注意 |
|---|---|---|
| `solve_sphere(M, b, lam_min, u)` | `uwb_sym3_solve_sphere(M, b, lam_min, u, NULL)` | 線形部分が余因子 → LDLᵀ なので結果は丸めの範囲 (double 1e-15、float 1e-6 程度) で変わる。hard case で極に達したとき survey 版は 0 (呼び出し側は u = ẑ のまま)、uwb_math 版は直前の u を返す。`fit_up` の rank 3 分岐と rank 2 の `\|u_par\| ≥ 1` 分岐の 2 箇所 |
| `sym3_null_vector(M, nvec)` | `uwb_sym3_null_vector(M, nvec)` | 行を max\|s\| で正規化する分、向きは丸めの範囲で同じ。選択規則・符号規約は同一 |
| `fit_up` rank 1 分岐の「最大ノルム行 → `uwb_v3_normalize`」 | `uwb_sym3_principal_axis(M, e0)` | 符号が正準化されるが、続く `coef = e0·b/λ0` と ẑ の射影は符号に不変なので u は変わらない |
| `fit_up` rank 2 分岐の `uwb_sym3_solve(Mr, b, upar)` | そのまま、または `uwb_sym3_ldl_factor/solve` (require_pd=1) | 任意。Mr = M + λ0 n nᵀ は正定値なので LDLᵀ の方が float で精度が出る |
| (dim=2 の shift 付き solve が要る箇所が出たら) | `uwb_sym2_solve_shifted` / `uwb_sym2_inverse_shifted` / `uwb_sym2_mv` | 今は survey に該当箇所なし |

差し替え後、`tests/host/survey` (561 件。A-2/A-4 実装で 365 → 561 に増えていた)
は clang / gcc-16 / `-ffp-contract=off` / ASan+UBSan の double・float すべて通過。
乱数 400 件（n=4..8、欠測リンク、距離ノイズ ≤5cm、δ、高さノイズ、外れ値なし）の
新旧比較は `ok`/`degenerate`/`excluded`/`redundancy` が 400/400 一致、座標差の
最大値は double 4.4e-15 m、float 1.9e-6 m（4000 件に広げても最大 2.9e-6 m、
中央値 4.8e-7 m）で、`uwb_sym3_solve_sphere` の LDLᵀ 化による丸めレベルの差
（想定どおり float ≈ 1e-6 m）に収まっている。`tests/host` 全体 (591418+10781+188+561件)
と `firmware/tag` の `idf.py build`（警告 0）も通過。
