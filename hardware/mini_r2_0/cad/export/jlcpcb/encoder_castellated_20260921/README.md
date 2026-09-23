# Encoder PCB L/R 発注用データ（2026-09-21）

基板製造用の提出ファイルを左右別にまとめた。小径半穴を含むため、
**発注先の個別加工確認が必要**。メーカー承認済み・発注済みのデータではない。

## ファイル

| 基板 | 発注用ZIP | 加工図（ZIPにも同梱） |
| --- | --- | --- |
| L | [Encoder-PCB-L ZIP](HM_Nightfall-mini-2e_Encoder-PCB-L_v1_castellated_order_20260921.zip) | [加工図 L](Encoder-PCB-L_fabrication.pdf) |
| R | [Encoder-PCB-R ZIP](HM_Nightfall-mini-2e_Encoder-PCB-R_v1_castellated_order_20260921.zip) | [加工図 R](Encoder-PCB-R_fabrication.pdf) |

ZIPは混ぜずに、LとRを別の基板設計として提出する。両方とも単板のデータであり、面付けは含まない。
通常の基板製造向けで、実装・ステンシル用のBOM、実装座標、ペーストデータは含まない。

各ZIPは11ファイル:

- 表裏の銅箔: `-F_Cu.gtl`, `-B_Cu.gbl`
- 表裏のレジスト: `-F_Mask.gts`, `-B_Mask.gbs`
- 外形: `-Edge_Cuts.gm1`
- めっき穴: `-PTH.drl`（左: 9穴、右: 10穴）
- レイヤー・半穴属性: `-job.gbrjob`
- `docs/Encoder-PCB-L_fabrication.pdf` または `docs/Encoder-PCB-R_fabrication.pdf`
- `docs/FABRICATION_NOTES.txt`: メーカー向け英語の加工指定
- `docs/SOURCE_MANIFEST.json`: 元PCB、commit、SHA-256、穴数、寸法
- `SHA256SUMS.txt`: 内容物のSHA-256

NPTHは0個のため空ファイルを除外した。両面ともシルクなし。
旧Eagle由来の大きな部品参照番号が端子に重なるため、シルク層は出力していない。

## 発注条件と加工照会

- 2層FR-4、外形3.45 × 7.00 mm。
- 板厚は元PCB設定の1.60 mm。銅厚・表面処理は加工先と取り決める（ENIGを推奨候補として指定文に記載）。
- Ø0.30 mmのめっき半穴4個。JLCPCBへ提出する場合は半穴加工ありとし、小径・小外形の可否を事前に確認する。
- 全辺めっきではなく、4穴の内壁めっきで各端子の表裏をつなぐ。端子間をめっきで短絡させない。
- 既存のØ0.20 mmビアは左5個／右6個。左の既存ビアには側辺銅箔間隔0.1808 mmの箇所がある。
- 面付け保持方法、角近傍の穴、小径半穴とアニュラ幅0.15 mmを加工先へ提示する。

寸法と加工要件の詳細はZIP内の英語指定文および
[設計記録](../../../../notes/kicad/ENCODER_CASTELLATED_PADS.md)を参照。
表面・裏面のGerberを個別に反転しないこと。全データは共通の絶対原点を使用する。

## 出力元・検証

- ソースcommit: `df315e3044e7`（PCBファイルの完全SHA-256は各ZIPのmanifestに収録）。
- KiCad 10.0.3でPCBから再生成した。今回の出力作成ではPCBファイルを変更していない。
- Gerber 4.6座標精度、Excellonはmm・絶対座標。穴座標はExcellonの0.001 mm精度で丸められる。
- 左右ともDRCは既存エラー6件、警告0件、未接続0件。詳細は設計記録に記載。
- Gerber形状が前回確認した出力と一致すること、半穴4個の座標・穴種別、表裏ランド・レジストの一致を検証した。
- 全ZIPエントリーのCRCとSHA-256、および同梱PDFと単独PDFの一致を検証した。
- 加工図はPDFからPNGへ描画して目視確認した。
- 物理的な剥離強度、メーカー承認、実機/HIL操作、発注・決済は今回実施していない。

Gerber・ドリルの再生成例（出力ディレクトリは新規に用意）:

```sh
kicad-cli pcb export gerbers --layers F.Cu,B.Cu,F.Mask,B.Mask,Edge.Cuts --precision 6 -o <output-dir> <board.kicad_pcb>
kicad-cli pcb export drill --excellon-separate-th --drill-origin absolute --excellon-units mm -o <output-dir> <board.kicad_pcb>
```

再生成後に空のNPTHファイルだけを除外し、加工図・指定文を同梱する。
PCBを更新した場合は、加工図・manifest・チェックサムを含む全ファイルを更新すること。
