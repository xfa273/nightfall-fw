# HM_Nightfall-mini-3a_v0 manual artwork project

このディレクトリは、最新回路図と配置済み・未配線PCBをまとめたKiCad 10プロジェクトです。

## 主なファイル

- `HM_Nightfall-mini-3a_v0.kicad_pro`: ここから開く
- `HM_Nightfall-mini-3a_v0.kicad_sch`: ERC 0の回路図
- `HM_Nightfall-mini-3a_v0.kicad_pcb`: 外形・機構シルク・102部品配置済みPCB
- `HM_Nightfall-mini-3a_v0.kicad_dru`: 4層／2 oz外層を想定した制約
- `HM_Nightfall-mini-3a_v0-power.kicad_sym`: 電源回路用ローカルシンボル
- `Nightfall-Power.pretty`: 電源部品用ローカルフットプリント
- `Nightfall-Local.pretty`: U5手はんだ向けローカルフットプリント
- `HM_Nightfall-mini-2e_v1.pretty`: mini-2e由来ローカルフットプリント
- `fp-lib-table`, `sym-lib-table`: プロジェクト相対パスのライブラリ設定

PCBは4層設定ですが、トラック、ビア、ゾーンはいずれも0です。まずIn1.CuにGND/GND2の
分割面を作り、R0を唯一のドメイン接続点にしてください。In2.Cuは低電流信号と+3V3用です。

`U2 = left`, `U3 = right` とし、U2はTP3/TP4、U3はTP1/TP2へ接続します。

回路図からPCBを更新する際、U5のpin 49は回路図上GNDですが、手はんだ向け
フットプリントでは下面pad 49を意図的に省略しています。この1件は仕様です。

