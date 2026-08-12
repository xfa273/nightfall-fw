# AI Coordination State

このファイルは、Codex の現在の運用状態を記録する正本です。

## 2026-08-12 mini_r3_0 PCB pause checkpoint

ユーザ指示により、`mini_r3_0` の4層PCB設計は利用枠リセットまで中断中です。
再開時は次を正本として読み、`/private/tmp` の旧試行板から再開しないでください。

- 引継ぎ: `hardware/mini_r3_0/checkpoints/2026-08-12-codex-pause/README.md`
- 共通基板: `hardware/mini_r3_0/checkpoints/2026-08-12-codex-pause/common/HM_Nightfall-mini-3a_v0.control4-led3.kicad_pcb`
- 共通基板 SHA-256: `a1a231fa70815dee5e19d0daeda741c14aa30f56400d8977a5e4750b6d5aa1ac`
- 状態: 4層、物理銅箔DRCエラー0、GND/GND2各1group、R0単一点、In1 plane-only、未配線20
- 独立してDRC-cleanな候補: PUSH、BOOT、BUZZER+MOSI局所移設、+3V3/VOL、EC_R_B、via-in-pad cleanup
- 注意: 上記候補はまだ相互統合されておらず、製造可能な最終基板ではない
- 残作業: 候補統合、IMU_CS、全未配線0、via-in-pad 0、In2最終plane、strict audit、canonical反映、ERC/DRC/parity、Gerber/ドリル/発注資料

F413電池保護は commit `5c77482` としてpush済みです。今回のPCB作業ではHIL、モータ、ファン、走行、Flash/NVM破壊操作を実行していません。

## 目的

- 会話履歴ではなく、リポジトリ内の状態ファイルで運用を継続可能にする
- 実装・調査・検証の実行方針を固定し、再現性を高める

## 現在の構成

- 実行者: Codex
- 実行方式: Codex が本作業ツリーで直接実行
- 主作業: STM32F405既存機体からSTM32F413 `mini_r2_0` 新機体への移植、実機HIL確認、ログ/ツール整備
- 旧運用: Windsurf/Cascade前提の資料はバックアップ済み。互換資産は `docs/ai/archive/` と `.windsurf/` に保持

## Codex 実行ポリシー

- 多ファイル変更、長時間調査、反復ビルド/テストを含む作業も Codex が直接対応する
- ビルド・ホストテストは必要に応じて自律実行する
- ST-LINK/UART実機操作は `docs/ai/HIL_SAFETY.md` に従う
- モータ・ファン・走行・探索・最短・NVM破壊的操作は、安全条件と作業意図が明確な場合だけ実行する
- 旧委譲関連の互換資産は `docs/ai/archive/delegation-legacy/` にアーカイブとして保持する

## 標準フロー

1. Codex がタスクを整理
2. Codex が関連ドキュメントとコード境界を確認
3. Codex が実装・調査・検証を直接実行
3. 必要に応じて `docs/ai/WORKLOG.md` に重要イベントを記録
4. Codex が最終報告

## 更新ルール

- 運用方式が変わったときに更新する
- 些細な一時メモは `WORKLOG.md` に残し、このファイルは方針のみ管理する
