---
description: 重い作業をCodex CLIへ委譲する
---

1. 依頼内容から、対象ファイル・成功条件・制約を整理する。
2. `docs/ai/STATE.md` と `docs/NIGHTFALL_FW_DEV_POLICY.md` を確認する。
3. 以下のコマンドで委譲する。

```bash
scripts/ai/delegate_to_codex.sh \
  --title "<task title>" \
  --prompt-file "<task prompt file>" \
  --targets "<target paths>" \
  --success-criteria "<success criteria>"
```

4. `docs/ai/HANDOFFS/<task-id>-result.md` を確認する。
5. 差分を取り込む場合は以下を実行する。

```bash
scripts/ai/maybe_apply_codex_result.sh --task-id <task-id>
```

6. 最終的にCascadeがユーザーへ結果を報告する。
