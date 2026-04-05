# Codex Delegation Rule

このルールは、Windsurf上でCascadeがCodex CLIへ作業委譲する際の基本基準です。

## 目的

- 重い作業をCodex CLIへ委譲し、Windsurf quota消費を抑える
- ユーザーは常にCascadeのみを操作する

## 委譲する条件

以下のいずれかに当てはまる場合、委譲を優先する。

- 多ファイル変更が見込まれる
- 調査→修正→検証のループが長い
- ビルド/テスト反復が多い
- AIの自走時間が長い

## 委譲しない条件

- 1〜2ファイルの軽微修正
- 説明のみの依頼
- 既に文脈が揃った単発修正

## 実行方法

1. タスクを整理し、成功条件を明文化する
2. `scripts/ai/delegate_to_codex.sh` で委譲する
3. `docs/ai/HANDOFFS/` の結果を確認する
4. 必要時 `scripts/ai/maybe_apply_codex_result.sh` で差分適用する

## 禁止事項

- ユーザー依頼と無関係な作業を委譲しない
- Codex側でコミット/プッシュさせない
- 危険な権限での無差別実行をしない
