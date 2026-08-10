# HM_Nightfall-mini-3a_v0 KiCad project

`HM_Nightfall-mini-2e_v1` の STM32F413 / センサ / FRAM / UI 回路をベースに、`CM_Nightfall-Air_v1` の 3S 電源構成と吸引ファン駆動回路を移植した回路図です。

## ファイル

- `HM_Nightfall-mini-3a_v0.kicad_sch`: 回路図編集原本
- `HM_Nightfall-mini-3a_v0.kicad_pro`: KiCad 10 プロジェクト設定
- `HM_Nightfall-mini-3a_v0-power.kicad_sym`: MPM3610 / SI7135DP / PMPB13XNE と、縦配置用受動部品・バッテリ電源のローカルシンボル
- `sym-lib-table`: `mini_r2_0` のインポート済みシンボルとローカル電源シンボルの参照

## 現在の制約

これは回路図段階の v0 です。MPM3610 と PMPB13XNE の PCB フットプリントは、Classic Eagle 原本と各社データシートの推奨ランドを照合してから作成します。既存 `mini_r2_0` 由来フットプリントも KiCad PCB 作成時に再割り当て・再確認が必要です。

KiCad 10.0.3 の CLI で回路図ロード、ネットリスト出力、SVG 描画を確認済みです。レイアウト変更前後で実部品 265 pin のネット分割が一致することも確認しています。既存 Eagle インポート由来の ERC 警告は残るため、PCB 着手前に電源ブロックだけでなく全回路の ERC ベースライン整理を行います。
