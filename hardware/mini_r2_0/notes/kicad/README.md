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

## サブ基板

以下の Eagle 原本は本体基板とは独立した小基板として、KiCad でも 1 基板 1 プロジェクトで管理します。

| 用途 | Eagle 原本 | KiCad 出力先 |
| --- | --- | --- |
| 左エンコーダ基板 | `cad/eagle/HM_Nightfall-mini-2e_Encoder-PCB-L_v1.sch` / `.brd` | `cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-L/` |
| 右エンコーダ基板 | `cad/eagle/HM_Nightfall-mini-2e_Encoder-PCB-R_v1.sch` / `.brd` | `cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-R/` |
| ST-LINK 変換基板 | `cad/eagle/STLink-Adapter.sch` / `.brd` | `cad/kicad/STLink-Adapter/` |
| UART 変換基板 | `cad/eagle/UART-Adapter.sch` / `.brd` | `cad/kicad/UART-Adapter/` |

### 左エンコーダ基板 初回チェック結果

- ERC: 7 errors / 7 warnings
- DRC: 12 errors / 13 warnings

### 右エンコーダ基板 初回チェック結果

- ERC: 6 errors / 7 warnings
- DRC: 5 errors / 12 warnings

### ST-LINK 変換基板 初回チェック結果

- ERC: 3 errors / 5 warnings
- DRC: 40 errors / 6 warnings

### UART 変換基板 初回チェック結果

- ERC: 4 errors / 8 warnings
- DRC: 39 errors / 35 warnings
