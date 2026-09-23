# Nightfall Air 2 (`classic_r2_0`)

`mini_r3_0` の回路をベースにした、STM32F413クラシックサイズ機の主基板です。

- KiCadプロジェクト: `cad/kicad/CM_Nightfall-Air-2a/CM_Nightfall-Air-2a_v0.kicad_pro`
- 基板外形: 48 mm × 80 mm（`Main-PCB.DXF`）
- 印刷シルク: `Main-PCB_Silk.DXF`
- 配置・配線: 作業中

回路はmini-3aをベースに、吸引ファンFET、機体電源FET、3.3 V LDO、壁センサ、IR投光LEDを旧 `CM_Nightfall-Air_v1` の部品へ変更しています。

- 3.3 V LDO: `NJM2884U1-33`（500 mA、SOT-89-5）
- 壁センサ: `ST-1KL3A`（2端子NPNフォトトランジスタ、TO-18）
- IR投光LED: `SFH4550`（850 nm、T1 3/4）
