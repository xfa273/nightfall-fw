---
description: Codex委譲結果を確認してクローズする
---

1. 対象の `task-id` を決める。
2. 以下を確認する。
   - `docs/ai/HANDOFFS/<task-id>-result.md`
   - `docs/ai/HANDOFFS/<task-id>-raw.log`
   - `docs/ai/HANDOFFS/<task-id>-last-message.md`
3. 差分を取り込む場合は実行する。

```bash
scripts/ai/maybe_apply_codex_result.sh --task-id <task-id>
```

4. 必要に応じてビルド/テストを実行し、妥当性を確認する。
5. `docs/ai/WORKLOG.md` を確認し、Cascadeとしてユーザーへ最終報告する。
