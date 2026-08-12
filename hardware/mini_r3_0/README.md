# hardware/mini_r3_0

STM32F413 系 mini の 2S–3S / 吸引ファン対応候補機体 (`mini_r3_0`) のハードウェア情報です。

## 状態

- 回路図 v0: ERC 0 まで整理済み
- PCB: 4層化し、電源・モータ・ファン・センサ・SPI・主要I/Oを配線中
- 最終統合・製造リリース: 未完了
- BOM / 実装部品確定: 未着手
- 実機電源評価: 未実施

2026-08-12 の作業中断点、物理DRC 0の共通基板、独立候補、再開手順は
[`checkpoints/2026-08-12-codex-pause/README.md`](checkpoints/2026-08-12-codex-pause/README.md)
に保存しています。チェックポイント内の候補はまだ相互統合されておらず、製造データではありません。

機体名 `mini_r3_0` と KiCad プロジェクト名 `HM_Nightfall-mini-3a_v0` は、`mini_r2_0` からメカと電源構成が変わるため付けた仮称です。正式名称が決まった時点で変更します。

## KiCad 編集原本

- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_sch`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_pcb`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_pro`
- `cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0-power.kicad_sym`

回路の移植範囲、電源ドメイン、確認事項は `notes/POWER_ARCHITECTURE.md` を参照してください。

## PCB配置ルール

- DXFから取り込んだ左右4か所のセンサ用シルク領域は機構占有領域とし、対応するIR LED／フォトトランジスタのスルーホール以外の部品を置かない。
- 左右のモータマウント用シルク輪郭内には部品を置かない。ただし、輪郭に接するエンコーダ用パッド `EC_L1..4`／`EC_R1..4` は機構基準位置を優先して動かさない。
- 吸引ファンの外周シルク円内には部品を置かない。ファン直付けパッド `J2` は円の左上外側へ接線方向に配置する。
- IMU `U4` は機体中心線 `x=148.501098 mm` 上に配置する。
- 電源スイッチ `POWER0` は本体と固定パッドを基板内に置き、操作レバーだけを後部の凹みから突出させる。
