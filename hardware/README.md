# hardware/

このディレクトリは、機体ごとのハードウェア設計情報の正式置き場です。

## ディレクトリ規約

- `hardware/<board_name>/cad/`: 回路図・基板・設計ルール
- `hardware/<board_name>/bom/`: 部品表・発注情報
- `hardware/<board_name>/notes/`: 設計メモ・確認結果

`board_name` は `mini_r1_0` のように `<family>_r<major>_<minor>` を使用します。

## 現在の公式配置

- `hardware/mini_r2_0/`: STM32F413 新機体
- `hardware/mini_r1_0/`: 現行 mini 機体（雛形）
- `hardware/classic_r1_0/`: 現行 classic 機体（雛形）

`_sandbox/` 配下は実験・一時作業用とし、正式成果物は本ディレクトリに集約します。
