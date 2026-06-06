# KiCad migration notes for mini_r2_0

`HM_Nightfall-mini-2e_v1` を KiCad 10.0.3 で Eagle から初回インポートした結果を保存します。

## 生成ファイル

- `cad/kicad/HM_Nightfall-mini-2e/HM_Nightfall-mini-2e_v1.kicad_pro`
- `cad/kicad/HM_Nightfall-mini-2e/HM_Nightfall-mini-2e_v1.kicad_sch`
- `cad/kicad/HM_Nightfall-mini-2e/HM_Nightfall-mini-2e_v1.kicad_pcb`
- `cad/kicad/HM_Nightfall-mini-2e/HM_Nightfall-mini-2e_v1-eagle-import.kicad_sym`
- `cad/kicad/HM_Nightfall-mini-2e/sym-lib-table`

## 初回チェック結果

- ERC: 46 errors / 88 warnings
- DRC: 544 errors / 306 warnings

## レビュー方針

- 初回インポート直後の ERC/DRC 違反は変換レビュー対象として残します。
- 回路図では未接続ピン、未解決テキスト変数、電源駆動、フットプリントライブラリ参照を優先確認します。
- PCBではクリアランス、外形、GNDポリゴン、モータドライバ周辺、SWD/UART、IMU/FRAM SPI2 周辺を優先確認します。
- レビュー完了までは Eagle 原本を併存させます。
