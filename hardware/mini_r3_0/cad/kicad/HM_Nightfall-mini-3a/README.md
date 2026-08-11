# HM_Nightfall-mini-3a_v0 KiCad project

`HM_Nightfall-mini-2e_v1` の STM32F413 / センサ / FRAM / UI 回路をベースに、`CM_Nightfall-Air_v1` の 3S 電源構成と吸引ファン駆動回路を移植した回路図です。

## ファイル

- `HM_Nightfall-mini-3a_v0.kicad_sch`: 回路図編集原本
- `HM_Nightfall-mini-3a_v0.kicad_pro`: KiCad 10 プロジェクト設定
- `HM_Nightfall-mini-3a_v0-power.kicad_sym`: MPM3610 / SI7135DP / IRLML6344TRPBF と、縦配置用受動部品・バッテリ電源のローカルシンボル
- `sym-lib-table`: `mini_r2_0` のインポート済みシンボルとローカル電源シンボルの参照

## 現在の制約

これは回路図段階の v0 です。MPM3610 の PCB フットプリントは、Classic Eagle 原本とデータシートの推奨ランドを照合してから作成します。IRLML6344TRPBF には標準 `Package_TO_SOT_SMD:SOT-23` を割り当てていますが、pin mapping と放熱銅箔を PCB 作成時に再確認します。既存 `mini_r2_0` 由来フットプリントも再割り当て・再確認が必要です。

KiCad 10.0.3 の CLI で回路図ロード、ネットリスト出力、SVG 描画を確認済みです。Q3 以外の実部品 257 pin は変更前後でネット分割が完全一致し、Q3 も旧 drain 5 pin / source 2 pin / gate 1 pin から新 D3 / S2 / G1 への役割正規化後に全ネット分割が一致することを確認しています。ERC は既存 Eagle インポート由来のものを中心に 132 件（error 36 / warning 96）が残るため、PCB 着手前に電源ブロックだけでなく全回路のベースライン整理を行います。
