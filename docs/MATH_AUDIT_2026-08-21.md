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
