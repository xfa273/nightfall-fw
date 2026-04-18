# Codex Delegation Rule

このルールは、過去のCodex委譲運用の記録です。
現在は委譲停止中で、Cascadeが全作業を直接実行します。

## 目的

- 委譲停止方針と、関連資産の扱いを明確化する
- 過去の委譲ログ/スクリプトを参照可能な状態で維持する

## 現在の運用

- 多ファイル変更、長時間調査、反復ビルド/テストを含む作業も Cascade が直接実行する
- `scripts/ai/delegate_to_codex.sh` と `scripts/ai/maybe_apply_codex_result.sh` は互換資産として残す
- `docs/ai/HANDOFFS/` はアーカイブ用途で維持し、新規成果物は原則追加しない

## 例外運用

- 将来、委譲運用を再開する場合は `AGENTS.md` と `docs/ai/STATE.md` を先に更新する
- 再開前に、このルール・workflow・関連ドキュメントの整合を再確認する

## 禁止事項

- 委譲停止方針と矛盾する自動委譲を行わない
- 関連ドキュメントを更新せずに運用方式を切り替えない
