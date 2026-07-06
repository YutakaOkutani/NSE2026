# NSE2026 ドキュメント

セットアップと基本的な実行方法は、リポジトリ直下の [`README.md`](../README.md) を参照してください。設計、開発方針、試験・運用手順はこの `docs/` 配下に集約しています。

## はじめに読む

- [アーキテクチャ概要](architecture/overview.md)
  - リポジトリ構成、各層の責務、二機体運用、ログ保存先
- [開発・設計の基本原則](development/principles.md)
  - 安全性、観測可能性、変更管理、試験の昇格条件

## 開発・デバッグ

- [ミッション本体](development/mission.md)
  - `mission/`、`mgr/`、`phases/` の設計・デバッグ指針
- [試験コード](development/testing.md)
  - `runs/`、`diag/`、`spec/`、`orch/`、`cam/` の役割
- [解析・低レイヤコンポーネント](development/components.md)
  - `analysis/` と `lib/` の設計指針
- [フローチャート](flowchart/README.md)
  - D2 記法による制御アルゴリズム図の管理、セットアップ、生成手順

## 運用手順

- [カメラフェーズ中継試験](operations/camera_relay.md)
- [ミッション中の無線制御](operations/radio_control.md)

## テレメトリ

- [テレメトリ仕様書](telemetry/specification.md)
- [テレメトリの設計・デバッグ指針](telemetry/debugging.md)

## ドキュメント管理ルール

- リポジトリ直下と各コードフォルダには、入口として必要な `README.md` だけを置く。
- それ以外の Markdown 文書は `docs/` 配下へ置く。
- 新規文書は用途に応じて `architecture/`、`development/`、`operations/`、`telemetry/` に追加する。
- 文書を追加・移動したら、この目次と参照リンクを更新する。
