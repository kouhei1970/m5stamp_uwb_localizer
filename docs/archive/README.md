# archive/ — 経緯文書

ここにあるのは、**設計当時の調査・検討の記録**です。プロジェクトが
どう進んだか・なぜ今の実装になったかを追うための資料であって、
**現役の仕様ではありません。** 現行の仕様・手順は [`docs/`](../) 側
（[`docs/README.md`](../README.md) の索引から）を参照してください。

**現役文書とここの内容が矛盾する場合は、常に現役文書が正しい。**
ここは訂正・更新されないまま残ります（間違いも含めて、何を考えて
何を間違えたかが後から分かる記録として残す方針）。

| 文書 | 内容 |
|---|---|
| [REIMPL_PLAN.md](REIMPL_PLAN.md) | 移植元（M5Stack 版 Arduino ラッパ）の課題一覧 R1〜R12 と、それぞれの決着 |
| [CRITICAL_REVIEW.md](CRITICAL_REVIEW.md) | 移植元コードの批判的レビュー（訂正ボックス入り） |
| [SURVEY_m5stamp_uwb_module.md](SURVEY_m5stamp_uwb_module.md) | M5Stamp UWB Module のハードウェア仕様の事前調査 |
| [SURVEY_m5stamp_uwb_port.md](SURVEY_m5stamp_uwb_port.md) | Arduino → ESP-IDF 移植の対応表 |
| [SURVEY_stampfly_ecosystem.md](SURVEY_stampfly_ecosystem.md) | StampFly 側のソフト構成の事前調査 |
| [SURVEY_stampfly_grove.md](SURVEY_stampfly_grove.md) | StampFly の GROVE 端子と GPIO 空き状況の事前調査（本文中に訂正あり） |
| [SURVEY_uwb_localizer.md](SURVEY_uwb_localizer.md) | 上流測位ライブラリ（uwb_localizer）の事前調査 |
| [PROGRESS.md](PROGRESS.md) | 開発進捗ログ（著者の作業記録。何がどこまで検証済みか） |
| [MATH_AUDIT_2026-08-21.md](MATH_AUDIT_2026-08-21.md) | 行列計算の残存箇所とスカラー化の設計根拠の監査。指摘は全件対応済み（`components/uwb_math/` へ統合） |
