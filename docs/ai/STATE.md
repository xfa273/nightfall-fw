# AI Coordination State

このファイルは、Cascade と Codex が共有する「現在の運用状態」の正本です。

## 目的

- 会話履歴ではなく、リポジトリ内の状態ファイルで運用を継続可能にする
- AI間の委譲条件と手順を固定し、再現性を高める

## 現在の構成

- オーケストレータ: Cascade（Windsurf）
- ワーカー: Codex CLI（`codex exec`）
- 実行方式: `scripts/ai/delegate_to_codex.sh` から非対話実行
- 作業隔離: `.ai/worktrees/` の `git worktree`

## Codex認証状態

- 方針: **ChatGPTサインインを使用**（API key運用ではなくPro枠活用）
- 確認コマンド: `codex login status`

## 委譲判定基準（デフォルト自動委譲）

前提:

- ユーザーが会話内で明示しなくても、重い作業はデフォルトでCodexへ委譲する
- ユーザーが「委譲せずCascadeで対応」と明示した場合のみ、委譲を抑制する

委譲する:

- 多ファイル変更
- 調査→修正→検証ループが長い
- ビルド/テスト反復が多い

委譲しない:

- 軽微な単発修正
- 説明のみ
- 局所修正

## 標準フロー

1. Cascadeがタスクを整理
2. `delegate_to_codex.sh` で task packet を作成してCodex実行
3. Codex結果を `docs/ai/HANDOFFS/` と `docs/ai/WORKLOG.md` に記録
4. 必要なら `maybe_apply_codex_result.sh` で差分を本作業ツリーへ適用
5. Cascadeが最終報告

## 今後の拡張

- Phase Bとして、`codex exec` 呼び出しをローカルMCPサーバでラップする
- `delegate_to_codex` 専用ツール化で、Cascade側呼び出しをさらに簡素化する

## 更新ルール

- 運用方式が変わったときに更新する
- 些細な一時メモは `WORKLOG.md` に残し、このファイルは方針のみ管理する
