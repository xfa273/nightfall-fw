---
description: 過去の委譲記録を確認してクローズする
---

1. 対象の `task-id` を決める（過去ログ確認時のみ）。
2. 以下を確認する。
   - `docs/ai/HANDOFFS/<task-id>-result.md`
   - `docs/ai/HANDOFFS/<task-id>-raw.log`
   - `docs/ai/HANDOFFS/<task-id>-last-message.md`
3. 必要時のみ、互換スクリプトで差分適用可否を検討する。
4. 必要に応じてビルド/テストを実行し、妥当性を確認する。
5. `docs/ai/WORKLOG.md` を確認し、Cascadeとしてユーザーへ最終報告する。

> 現在は委譲停止中のため、このworkflowは新規委譲作業ではなく過去記録のクローズ用途に限定します。
