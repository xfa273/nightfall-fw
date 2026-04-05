# Codex委譲運用ガイド（nightfall-fw）

このドキュメントは、Cascadeを窓口にしたまま重い作業をCodex CLIへ委譲するための手順です。

## 1. できること

- あなたはこれまで通り Cascade に依頼するだけでOKです。
- 重い作業は、依頼文で明示しなくても内部で `codex exec` にデフォルト委譲されます。
- 委譲結果は `docs/ai/HANDOFFS/` に保存されます。

## 2. デフォルト挙動（重要）

- 毎回のConversationで、重い作業は自動的にCodex委譲します。
- 軽微修正・説明のみ・局所修正はCascadeが直接対応します。
- 今回だけ委譲を止めたい場合は「今回は委譲せずCascadeで対応」と指定してください。

## 3. 初回確認

### 3-1. Codex CLIの確認

```bash
codex --version
```

### 3-2. ChatGPTサインイン状態の確認

```bash
codex login status
```

`Logged in using ChatGPT` と表示されれば、ChatGPT Pro側リソースを利用できます。

## 4. 委譲の実行

基本コマンド:

```bash
scripts/ai/delegate_to_codex.sh \
  --title "<task title>" \
  --prompt-file "<task prompt file>" \
  --targets "<target paths>" \
  --success-criteria "<success criteria>"
```

`--prompt` で直接テキストを渡すこともできます。

## 5. 結果確認

委譲後は `docs/ai/HANDOFFS/` に以下が出力されます。

- `<task-id>-result.md`（要約）
- `<task-id>-raw.log`（生ログ）
- `<task-id>-last-message.md`（最終回答）
- `<task-id>.diff`（適用用差分）

## 6. 差分を取り込む

```bash
scripts/ai/maybe_apply_codex_result.sh --task-id <task-id>
```

自動確認なしで適用する場合:

```bash
scripts/ai/maybe_apply_codex_result.sh --task-id <task-id> --yes
```

## 7. よくある運用

- 軽い修正はCascadeが直接対応
- 重い修正はCodex委譲
- 追加調査が必要なら同じ `task-id` 方針で再委譲

## 8. 関連ファイル

- `AGENTS.md`
- `docs/ai/STATE.md`
- `docs/ai/WORKLOG.md`
- `.windsurf/rules/10-codex-delegation.md`
- `.windsurf/workflows/delegate-heavy-task.md`
- `.windsurf/workflows/close-handoff.md`
