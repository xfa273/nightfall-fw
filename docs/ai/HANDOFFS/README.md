# HANDOFFS

このディレクトリには、Codexへ委譲した過去タスクの成果物を保存します。

現在は委譲停止運用のため、通常は新規ファイルを追加しません。

## 生成されるファイル

- `<task-id>-packet.md`: 委譲時のタスクパケット
- `<task-id>-raw.log`: Codex CLI標準出力/標準エラー
- `<task-id>-last-message.md`: Codexの最終メッセージ
- `<task-id>-result.md`: 要約（変更/検証/リスク）
- `<task-id>-meta.env`: 追跡情報（worktree, branch, paths）
- `<task-id>.diff`: 本作業ツリーへ適用可能な差分

## 使い方

1. 過去の委譲履歴を参照する
2. 必要時のみ、互換スクリプトで旧成果物の差分適用を検討する
