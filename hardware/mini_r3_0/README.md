# hardware/mini_r3_0

STM32F413 系 mini の 2S–3S / 吸引ファン対応候補機体 (`mini_r3_0`) のハードウェア情報です。

## 状態

- 回路図 v0: 作成済み
- PCB: 外形・機構シルク導入および部品配置済み（未配線、レビュー中）
- BOM / 実装部品確定: 未着手
- 実機電源評価: 未実施

機体名 `mini_r3_0` と KiCad プロジェクト名 `HM_Nightfall-mini-3a_v0` は、`mini_r2_0` からメカと電源構成が変わるため付けた仮称です。正式名称が決まった時点で変更します。

## KiCad 編集原本

- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_sch`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_pcb`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_pro`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0-power.kicad_sym`

回路の移植範囲、電源ドメイン、確認事項は `notes/POWER_ARCHITECTURE.md` を参照してください。
