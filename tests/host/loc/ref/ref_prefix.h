/* 参照実装 (ref 配下の .c) の外部シンボルをすべて ref_ 接頭辞に付け替える。
 *
 * ref 配下の .c は uwb_loc.h の型・関数名をそのまま使っている (上流の凍結版を
 * 無改変で写したもの)。一方 test_regress.c は新実装の components/uwb_loc/
 * を "uwb_loc.h" として include するので、同じ翻訳単位グラフの中に
 * uwb_solve_lv2 のような同名シンボルが 2 つ存在するとリンクが衝突する。
 *
 * ここではプリプロセッサの単純な置換で ref 配下の .c 側の外部シンボルだけを
 * すべて ref_ 接頭辞に付け替える。型名 (uwb_config, uwb_anchor, uwb_meas,
 * uwb_fix, uwb_ekf, uwb_real, uwb_motion, ...) は付け替えない —
 * ref 配下の .c の翻訳単位ではこれらは ref/uwb_loc.h が定義する「参照実装の型」
 * であり、新実装の同名の型と構造的には同じでも別の型として扱われる。
 * これは問題ない: ref 配下の .c を呼ぶのは ref_bridge.c だけで、ref_bridge.c も
 * ref/uwb_loc.h だけを include するので型は揃っている
 * (新旧の型を混ぜて渡すのは test_regress.c ではなく ref_bridge.c の
 * 責務であり、ref_bridge.c はプレーン C の値渡しで橋渡しする)。
 *
 * ビルド方法: ref 配下の .c は Makefile 側で `-include ref/ref_prefix.h` を
 * 付けてコンパイルする (ソース自体は無改変のまま)。
 *
 * 対象は uwb_loc.h (公開 API) + uwb_internal.h (内部 API) +
 * uwb_linalg.h (線形代数) の全関数、34 個。
 */
#ifndef UWB_REF_PREFIX_H
#define UWB_REF_PREFIX_H

/* ---- uwb_loc.h (公開 API、15 個) ---- */
#define uwb_config_init        ref_uwb_config_init
#define uwb_anchor_index       ref_uwb_anchor_index
#define uwb_solve_lv0          ref_uwb_solve_lv0
#define uwb_solve_lv1          ref_uwb_solve_lv1
#define uwb_solve_lv2          ref_uwb_solve_lv2
#define uwb_beck_gtrs          ref_uwb_beck_gtrs
#define uwb_lls_trilateration  ref_uwb_lls_trilateration
#define uwb_ekf_init           ref_uwb_ekf_init
#define uwb_ekf_reset          ref_uwb_ekf_reset
#define uwb_ekf_predict        ref_uwb_ekf_predict
#define uwb_ekf_update         ref_uwb_ekf_update
#define uwb_gdop_at            ref_uwb_gdop_at
#define uwb_crlb_at            ref_uwb_crlb_at
#define uwb_anchors_coplanar   ref_uwb_anchors_coplanar
#define uwb_version            ref_uwb_version

/* ---- uwb_internal.h (内部 API、11 個) ---- */
#define uwb_meas_usable        ref_uwb_meas_usable
#define uwb_corrected          ref_uwb_corrected
#define uwb_sigma_of           ref_uwb_sigma_of
#define uwb_evaluate           ref_uwb_evaluate
#define uwb_n_free             ref_uwb_n_free
#define uwb_project            ref_uwb_project
#define uwb_gdop_from_jac      ref_uwb_gdop_from_jac
#define uwb_resolve_mirror     ref_uwb_resolve_mirror
#define uwb_mirror_side        ref_uwb_mirror_side
#define uwb_fix_failed         ref_uwb_fix_failed
#define uwb_fix_finish         ref_uwb_fix_finish

/* ---- uwb_linalg.h (線形代数、8 個) ---- */
#define uwb_solve_lin          ref_uwb_solve_lin
#define uwb_inverse_spd        ref_uwb_inverse_spd
#define uwb_inverse            ref_uwb_inverse
#define uwb_cholesky           ref_uwb_cholesky
#define uwb_sym_eig            ref_uwb_sym_eig
#define uwb_sym_eigvals        ref_uwb_sym_eigvals
#define uwb_ata_weighted       ref_uwb_ata_weighted
#define uwb_atb_weighted       ref_uwb_atb_weighted

#endif /* UWB_REF_PREFIX_H */
