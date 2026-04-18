# AI Coordination State

このファイルは、Cascade の現在の運用状態を記録する正本です。

## 目的

- 会話履歴ではなく、リポジトリ内の状態ファイルで運用を継続可能にする
- 実装・調査・検証の実行方針を固定し、再現性を高める

## 現在の構成

- 実行者: Cascade（Windsurf）
- 実行方式: Cascade が本作業ツリーで直接実行
- 委譲状態: Codex CLI への委譲は当面停止

## 委譲停止ポリシー

- 多ファイル変更、長時間調査、反復ビルド/テストを含む作業も Cascade が直接対応する
- `scripts/ai/delegate_to_codex.sh` と `scripts/ai/maybe_apply_codex_result.sh` は過去運用との互換資産として保持する
- `docs/ai/HANDOFFS/` は過去ログ保管用途として維持し、新規委譲成果物は原則追加しない

## 標準フロー

1. Cascadeがタスクを整理
2. Cascadeが実装・調査・検証を直接実行
3. 必要に応じて `docs/ai/WORKLOG.md` に重要イベントを記録
4. Cascadeが最終報告

## 更新ルール

- 運用方式が変わったときに更新する
- 些細な一時メモは `WORKLOG.md` に残し、このファイルは方針のみ管理する
