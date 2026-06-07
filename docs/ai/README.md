# AI運用ガイド（Codex移行後）

このドキュメントは、`nightfall-fw` における現在のAI運用方針を示します。
2026-06-07 時点で、主担当AIを Windsurf/Cascade 前提から Codex 前提へ移行しました。

## 1. 現在の運用方針

- ユーザーは Codex に依頼するだけで運用できます。
- 実装・調査・検証・最終説明は Codex が直接実行します。
- F413 実機が ST-LINK/UART で接続されている場合、`docs/ai/HIL_SAFETY.md` の範囲で Codex が自律的にビルド、書き込み、reset、非モータ診断、ログ取得、解析を行えます。
- モータ・ファン・走行・探索・最短・NVM破壊的操作は、安全条件を満たす場合だけ実行します。

## 2. 標準フロー

1. Codex が依頼内容、対象ファイル、実機操作の有無を整理する
2. Codex が本作業ツリーで実装・調査・検証を行う
3. 必要に応じて `docs/ai/WORKLOG.md` に重要イベントを記録する
4. Codex が最終報告する

## 3. 記録方針

- `docs/ai/archive/delegation-legacy/HANDOFFS/` は過去の委譲履歴として保持します。
- 旧 Windsurf/Cascade 前提資料の変更前バックアップは `docs/ai/archive/codex-migration-backup-*/` に一時保存できますが、正本ではないため Git 管理外です。
- Git/GitHubで残すべき履歴は `docs/ai/GIT_GITHUB_POLICY.md` に従って branch、commit、PR で管理します。
- 新規の作業状態は `docs/ai/STATE.md`、重要イベントは `docs/ai/WORKLOG.md` に残します。

## 4. 互換資産の扱い

以下の資産は過去運用との互換・記録のため残していますが、現行運用では参照・実行しません。
ユーザーから明示的に「旧委譲を復活する」指示がある場合のみ使用対象へ戻します。

- `docs/ai/archive/delegation-legacy/scripts/ai/delegate_to_codex.sh`
- `docs/ai/archive/delegation-legacy/scripts/ai/maybe_apply_codex_result.sh`
- `docs/ai/archive/delegation-legacy/scripts/ai/summarize_codex_result.sh`
- `docs/ai/archive/delegation-legacy/HANDOFFS/`
- `docs/ai/archive/delegation-legacy/windsurf/rules/10-codex-delegation.md`
- `docs/ai/archive/delegation-legacy/windsurf/workflows/close-handoff.md`

## 5. 関連ファイル

- `AGENTS.md`
- `docs/ai/CODEX_ONBOARDING.md`
- `docs/ai/F413_PORTING_STATE.md`
- `docs/ai/HIL_SAFETY.md`
- `docs/ai/GIT_GITHUB_POLICY.md`
- `docs/ai/STATE.md`
- `docs/ai/WORKLOG.md`
- `docs/ai/archive/AGENTS.md`
- `docs/ai/archive/delegation-legacy/windsurf/rules/10-codex-delegation.md`
- `.windsurf/workflows/delegate-heavy-task.md`
- `docs/ai/archive/delegation-legacy/windsurf/workflows/close-handoff.md`
