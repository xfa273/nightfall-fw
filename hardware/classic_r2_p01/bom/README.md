# BOM

初期回路は `mini_r3_0` を基にしています。下表は電源・ファン用FETの変更です。LDO・壁センサ・IR投光LEDを含む部品差分は[回路の由来](../cad/kicad/CM_Nightfall-Air-2a/README.md#回路の由来)にまとめています。

| Reference | Part | Function |
|---|---|---|
| Q2 | SI7135DP-T1-GE3 | 機体電源 P-channel MOSFET |
| Q3 | PMPB13XNE,115 | 吸引ファン N-channel MOSFET |

U6のFRAMは在庫品の `CY15B108QI-20LPXC`（GQFN-8）を維持しています。
