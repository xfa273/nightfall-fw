# HANDOFFS

このディレクトリには、Codexへ委譲した各タスクの成果物を保存します。

## 生成されるファイル

- `<task-id>-packet.md`: 委譲時のタスクパケット
- `<task-id>-raw.log`: Codex CLI標準出力/標準エラー
- `<task-id>-last-message.md`: Codexの最終メッセージ
- `<task-id>-result.md`: 要約（変更/検証/リスク）
- `<task-id>-meta.env`: 追跡情報（worktree, branch, paths）
- `<task-id>.diff`: 本作業ツリーへ適用可能な差分

## 使い方

1. `scripts/ai/delegate_to_codex.sh` で委譲
2. `scripts/ai/maybe_apply_codex_result.sh --task-id <task-id>` で差分適用を検討
