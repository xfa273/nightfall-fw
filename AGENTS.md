# AGENTS.md

このファイルは `nightfall-fw` リポジトリ全体に適用される、AIエージェント向けの共通運用ルールです。

## 1) 最優先で参照するドキュメント

作業前に以下を確認してください。

1. `docs/NIGHTFALL_FW_DEV_POLICY.md`
2. `docs/ai/STATE.md`
3. `docs/ai/WORKLOG.md`

実装・運用判断が衝突した場合は、`docs/NIGHTFALL_FW_DEV_POLICY.md` を優先します。

## 2) 役割分担（Cascade / Codex）

- **Cascade**: ユーザーとの窓口、方針決定、最終説明、委譲判断
- **Codex CLI**: 重い実装・調査・テストの下請けワーカー

重い作業は `scripts/ai/delegate_to_codex.sh` で Codex へ委譲してください。

## 3) 委譲対象の基準

以下に当てはまる場合は、Codex委譲を優先します。

- 多ファイル変更が見込まれる
- 調査→修正→検証ループが長い
- テストやビルドを複数回回す必要がある
- ユーザーとの往復よりAIの自走時間が長い

以下は Cascade で直接対応してください。

- 1〜2ファイルの軽微修正
- 説明のみ
- 既知の局所的変更

## 4) 作業隔離

- Codex作業は原則 `.ai/worktrees/` 配下の `git worktree` で実行します。
- ユーザーの作業ツリーを直接汚さないことを優先します。

## 5) Handoffと記録

- 委譲ごとに `docs/ai/HANDOFFS/` に以下を残します。
  - task packet
  - codex raw log
  - codex last message
  - result summary
  - metadata
- `docs/ai/WORKLOG.md` に実行履歴を追記してください。

## 6) 禁止事項

- ユーザー依頼と無関係な大規模変更
- 指示されていない自動コミット・自動push
- 書き込み・破壊系操作（フラッシュ書き込み含む）の勝手な実行

## 7) 推奨検証

変更時は可能な限り対象に近いビルド/テストを実行してください。

例:

- `cmake --build --preset Debug-stm32f405`
- `cmake --build --preset Debug-stm32f413`
