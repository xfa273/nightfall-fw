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
- KiCad 10.0.3 で `cad/kicad/HM_Nightfall-mini-2e/` へ初回インポート済みです。
- 初回インポート結果は `notes/kicad/` の ERC/DRC レポートを確認しながらレビューします。
- KiCad レビュー完了までは Eagle 原本と KiCad 原本を併存させ、確認後に `cad/kicad/` を編集正とします。

## KiCad プロジェクト単位

- `cad/kicad/HM_Nightfall-mini-2e/`: 本体基板
- `cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-L/`: 左エンコーダ基板
- `cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-R/`: 右エンコーダ基板
- `cad/kicad/STLink-Adapter/`: ST-LINK から機体コネクタへの変換基板
- `cad/kicad/UART-Adapter/`: UART から機体コネクタへの変換基板

各サブ基板は本体基板と独立した小基板のため、KiCad でも 1 基板 1 プロジェクトとして管理します。
- Eagle 9.6.2 で生成した参照アーカイブは `cad/export/reference/eagle_archive_20260606/` に保存しています。
