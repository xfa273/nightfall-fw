# hardware/mini_r2_0

STM32F413 系 mini 新機体 (`mini_r2_0`) のハードウェア情報です。

## 構成

- `cad/`: Eagle 回路図・基板・PDF・DRC ルール
- `cad/eagle/`: Eagle 移行前原本
- `cad/kicad/`: KiCad 移行後の編集原本
- `cad/export/`: PDF、Gerber、実装位置、発注用出力
- `bom/`: 部品表・調達情報
- `notes/`: ピン一覧、ERC/DRC スクリーンショット

## 備考

- 主要ソースは `_sandbox/f413_preorder/` から本ディレクトリへ移行済みです。
- 以後の更新は `hardware/mini_r2_0/` 側を正として管理します。
- 対応する Eagle プロジェクトは `HM_Nightfall-mini-2e` です。
- 対応するソフトウェア側プロファイルは `f413_preorder` です。
- KiCad 移行中は Eagle 原本と KiCad 原本を併存させ、変換確認後は `cad/kicad/` を編集正とします。
