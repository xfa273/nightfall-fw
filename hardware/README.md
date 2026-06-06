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
- Eagle が使えるうちに確保した旧設計の参照アーカイブは `cad/export/reference/eagle_archive_YYYYMMDD/` に置きます。
- 旧設計や大きな archive は現役基板と分け、必要に応じて別リポジトリ化します。

## 現在の公式配置

- `hardware/mini_r2_0/`: `f413_preorder` / STM32F413 搭載ハーフサイズ / Eagle `HM_Nightfall-mini-2e`
- `hardware/mini_r1_0/`: STM32F405 搭載ハーフサイズ最終版 / Eagle `HM_Nightfall-mini_v1`
- `hardware/classic_r1_0/`: STM32F405 搭載クラシックサイズ最終版 / Eagle `CM_Nightfall-Air_v1`

上記以外の Eagle プロジェクトは旧機体の参考資料として扱い、現役基板とは分けて保管します。

`_sandbox/` 配下は実験・一時作業用とし、正式成果物は本ディレクトリに集約します。
