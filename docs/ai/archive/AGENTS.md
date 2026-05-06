# AGENTS.md

このファイルは `nightfall-fw` リポジトリ全体に適用される、AIエージェント向けの共通運用ルールです。

## 1) 最優先で参照するドキュメント

作業前に以下を確認してください。

1. `docs/NIGHTFALL_FW_DEV_POLICY.md`
2. `docs/ai/STATE.md`
3. `docs/ai/WORKLOG.md`

実装・運用判断が衝突した場合は、`docs/NIGHTFALL_FW_DEV_POLICY.md` を優先します。

## 2) 役割分担（当面はCascade単独実行）

- **Cascade**: ユーザーとの窓口、方針決定、実装・調査・検証・最終説明まで一貫して担当

当面の運用では、Codex CLI への委譲は停止します。重い作業を含め、すべて Cascade が直接実行してください。
ユーザーから明示的に「委譲を復活する」指示があるまで、委譲関連の互換資産（`docs/ai/archive/delegation-legacy/`）は参照・実行対象にしないでください。

## 3) 作業方針（直接実行）

- 多ファイル変更、長い調査、反復ビルド/テストを含む作業も Cascade が直接対応する
- 依頼範囲外の変更は行わない
- 検証可能な変更では、対象に近いビルド/テストを優先して実行する

## 4) 記録

- 運用上の重要イベントは `docs/ai/WORKLOG.md` に追記する
- 委譲関連の過去資産は `docs/ai/archive/delegation-legacy/` に保持する（新規作成は原則行わない）

## 5) 禁止事項

- ユーザー依頼と無関係な大規模変更
- 指示されていない自動コミット・自動push
- 書き込み・破壊系操作（フラッシュ書き込み含む）の勝手な実行

## 6) 推奨検証

変更時は可能な限り対象に近いビルド/テストを実行してください。

例:

- `cmake --build --preset Debug-stm32f405`
- `cmake --build --preset Debug-stm32f413`
