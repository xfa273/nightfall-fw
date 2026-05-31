# hardware/

このディレクトリは、機体ごとのハードウェア設計情報の正式置き場です。

## ディレクトリ規約

- `hardware/<board_name>/cad/`: 回路図・基板・設計ルール
- `hardware/<board_name>/cad/eagle/`: Eagle 移行前原本
- `hardware/<board_name>/cad/kicad/`: KiCad 移行後の編集原本
- `hardware/<board_name>/cad/export/`: PDF、Gerber、実装位置、発注用出力
- `hardware/<board_name>/bom/`: 部品表・発注情報
- `hardware/<board_name>/notes/`: 設計メモ・確認結果

`board_name` は `mini_r1_0` のように `<family>_r<major>_<minor>` を使用します。

## Git管理方針

- Eagle の `.sch` / `.brd` / `.dru` / `.cam` と KiCad の `.kicad_pro` / `.kicad_sch` / `.kicad_pcb` は原本として管理します。
- Eagle の `.s#1` / `.b#1` / `.l#1` などの自動バックアップと、KiCad のローカル状態・自動バックアップは管理しません。
- 発注用の Gerber zip や PDF は、再現性が必要な発注版だけを `cad/export/` または GitHub Release で管理します。
- 旧設計や大きな archive は現役基板と分け、必要に応じて別リポジトリ化します。

## 現在の公式配置

- `hardware/mini_r2_0/`: STM32F413 新機体
- `hardware/mini_r1_0/`: 現行 mini 機体（雛形）
- `hardware/classic_r1_0/`: 現行 classic 機体（雛形）

`_sandbox/` 配下は実験・一時作業用とし、正式成果物は本ディレクトリに集約します。
