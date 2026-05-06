# AI運用ガイド（Cascade単独実行）

このドキュメントは、`nightfall-fw` における現在のAI運用方針を示します。

## 1. 現在の運用方針

- ユーザーはこれまで通り Cascade に依頼するだけで運用できます。
- 実装・調査・検証・最終説明は Cascade が直接実行します。
- 当面の間、Codex CLI への委譲は行いません。

## 2. 標準フロー

1. Cascade が依頼内容を整理する
2. Cascade が本作業ツリーで実装・調査・検証を行う
3. 必要に応じて `docs/ai/WORKLOG.md` に重要イベントを記録する
4. Cascade が最終報告する

## 3. 記録方針

- `docs/ai/archive/delegation-legacy/HANDOFFS/` は過去の委譲履歴として保持します。
- 新規の委譲成果物（packet/raw log/diff など）は原則追加しません。

## 4. 互換資産の扱い

以下の資産は過去運用との互換・記録のため残していますが、現行運用では参照・実行しません。
ユーザーから明示的に「委譲を復活する」指示がある場合のみ使用対象へ戻します。

- `docs/ai/archive/delegation-legacy/scripts/ai/delegate_to_codex.sh`
- `docs/ai/archive/delegation-legacy/scripts/ai/maybe_apply_codex_result.sh`
- `docs/ai/archive/delegation-legacy/scripts/ai/summarize_codex_result.sh`
- `docs/ai/archive/delegation-legacy/HANDOFFS/`
- `docs/ai/archive/delegation-legacy/windsurf/rules/10-codex-delegation.md`
- `docs/ai/archive/delegation-legacy/windsurf/workflows/close-handoff.md`

## 5. 関連ファイル

- `docs/ai/archive/AGENTS.md`
- `docs/ai/STATE.md`
- `docs/ai/WORKLOG.md`
- `docs/ai/archive/delegation-legacy/windsurf/rules/10-codex-delegation.md`
- `.windsurf/workflows/delegate-heavy-task.md`
- `docs/ai/archive/delegation-legacy/windsurf/workflows/close-handoff.md`
