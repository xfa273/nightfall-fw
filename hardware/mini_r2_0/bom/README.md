# mini_r2_0 JLCPCB PCBA 発注用 BOM（2026-10-11 作成）

`mini_r2_0`（HM_Nightfall-mini-2e、STM32F413）を JLCPCB の PCBA（部品実装込み）で
発注するための BOM / 配置ファイル一式です。
部品表の原本は各 board の `.kicad_pcb` であり、ここにある CSV は
`tools/hardware/make_jlcpcb_bom.py` による自動生成物です。

## ファイル構成と発注手順

発注先ディレクトリ: `hardware/mini_r2_0/cad/export/jlcpcb/order_20261010/`

| 基板 | Gerber zip | BOM | 配置 (CPL) | スルーホール一覧 |
|---|---|---|---|---|
| メイン基板 39.0×70.0mm | `HM_Nightfall-mini-2e_v1_gerber.zip`（`../HM_Nightfall-mini-2e_v1_kicad_jlcpcb_candidate_20260606/` の検証済み Gerber と同一物。2層・1.6mm。JLC Gerber Viewer で旧 Eagle 発注品と照合済み） | `main_board_bom.csv`（74点 / 26行） | `main_board_cpl.csv`（SMT 64点のみ） | `main_board_tht.csv`（10点） |
| エンコーダ L 3.45×7.0mm | `HM_Nightfall-mini-2e_Encoder-PCB-L_v1_gerber.zip` | `encoder_L_bom.csv`（3点） | `encoder_L_cpl.csv` | `encoder_L_tht.csv`（0点） |
| エンコーダ R 3.45×7.0mm | `HM_Nightfall-mini-2e_Encoder-PCB-R_v1_gerber.zip` | `encoder_R_bom.csv`（3点） | `encoder_R_cpl.csv` | `encoder_R_tht.csv`（0点） |

JLCPCB の PCBA 発注では基板ごとに Gerber zip + BOM + CPL の 3 点をアップロードします。
エンコーダ基板は L/R で部品が左右対称（IC1/IC2 など Ref 違い）なので別発注扱いにしてください。

## 生成コマンド（再実行可能）

```sh
python3 tools/hardware/make_jlcpcb_bom.py \
    hardware/mini_r2_0/bom/lcsc_map.csv \
    hardware/mini_r2_0/cad/kicad/HM_Nightfall-mini-2e/HM_Nightfall-mini-2e_v1.kicad_pcb \
    hardware/mini_r2_0/cad/export/jlcpcb/order_20261010/main_board
```

- マッピング（設計値 → LCSC Cコード）は `lcsc_map.csv` に集約しています。
  列は `ref_pattern,value,footprint,mpn,lcsc,status,note`、status は
  `ok`（設計品が LCSC に在库）/ `alt`（在庫のある等価品に差替）/
  `none`（LCSC 未掲載＝別途手配）/ `prop`（要確認の提案）。
- テストポイント（`TP1`〜`TP4`、`EC_*`：パッドのみの探针）は BOM/CPL から除外。
- CPL は JLC が実装する SMT 部品のみ。スルーホール部品は `*_tht.csv` に分離
  （JLC PCBA はスルーホール挿入を Rows 扱いで行わないため、手はんだ前提）。

## 見積前に確認が必要な事項（2026-10-11 時点の LCSC 在庫スナップショット）

在庫数は LCSC 商品ページから機械抽出した値です。発注時に必ず再確認してください。

1. **長納期（LCSC 在庫 0）**
   - `STM32F413CHU6TR` = C3227331（在庫0。トレー品 C1337761 が5個のみ）
   - `CY15B108QI-20LPXC` = C2948632（在庫0）
   → JLC 見積では「Long lead time parts」として扱われ、基板単体先行製造になることがあります。
2. **LCSC 未掲載（NO-CODE、顧客支給 or 手はんだ）** — 合計 11 点
   - `MP6551GQB-P`（モータドライバ U2/U3）… LCSC 検索 0 件。MPS 直販/国内代理店等で調達し JLC へ送り込むか、別途実装。
   - `AP2210N-3.3TRG1`（3.3V LDO U1、SOT-223）… LCSC 0 件。
   - `TL6330AF200Q`（タクト MODE0/RESET0、2点）、`EG1215AA`（スライド電源スイッチ）… フットプリントが部品専用形状のため代替は基板修正なしには不可。
   - `SMT-0540-S-R`（Kingstate ブザ SPK1）… LCSC 0 件。
   - `ST-1KL3A`（フォトトランジスタ PT_* 4点）… LCSC 未掲載（KODENSHI）。2024-05 の部品メモどおり国内（カシノキ等）調達。
3. **在庫ありへ差替済み（alt）**
   - 0.1µF 0402（メイン10点+エンコーダ2点）: 設計 MPN `CL05B104KO5NNNC`（C1525、ページJSON上 在庫0 を確認）→
     `CC0402KRX7R8BB104`（YAGEO、X7R 16V）**C105883**（1,247,500）
   - 1kΩ 0402: 設計 MPN `0402WGF1001TCE`（C11702、在庫0）→ `HP02WAF1001TCE` **C2759536**（14,000）
   - 2N7002 SOT-523: 設計 MPN `2N7002TQ-7-F`（C526327、在庫0）→ `2N7002T`（GOODWORK）**C22466871**（29,000、SOT-523 確認済み）
   - K1 1.27mm 2×5 オスピンヘッダ: 設計 `62201021121`（Würth、LCSC 0 件）→ `HX PZ1.27-2x5P TP` **C41376037**（41,490）。**発注前にフットプリント（2.54mm 外側ピッチ/2列/10 pads）と噛み合うか確認**。K1 はスルーホールなので実装は手はんだです。
   - `BAS16WT1G` = C157565 は LCSC 在庫 20（基板あたり1個）。問題ないが数量を増やす場合は代替（BAS16HW 等）を要検討。
4. **要決定（仮置き）**
   - 0603 LED（LED1/2/3・P-LED0 の4点）: 色情報が基板データにもドキュメントにも無いため、
     在庫潤沢な `LTST-C190KFKT`（オレンジ）**C157740**（45,600）で仮置き。緑等に変える場合は
     `lcsc_map.csv` の該当行の C コードを差し替えて再生成してください（例: 緑 0603 は KT-0603G = C12624 に在庫 499,850）。
   - IR LED: 3mm スルーホール（LED3MM、2.54mm）には OSRAM `SFH 4550`（**C2900376**、2,473）を割り当て
     （classic_r2 の `SFH4550_T1-3-4_P2.54mm` と同じ部品・950nm）。PT と向き・本数だけ実機確認推奨。
5. **エンコーダ基板（L/R）について**
   - 3.45×7.0mm の超小型。JLCPCB の最低サイズ/パネル化要件に抵触する可能性があるため、
     パネル化（接合タブ）推奨で発注するか要確認。
   - 部品は基板あたり 3 点だが**両面実装**になる（L: NJL5820R-TE4=上面 / R1・C1=下面、R: IC2=下面 / R2・C2=上面）
     ため JLCPCB では両面実装のセットアップ料金が加算されます。少量なら実装なし（PCB のみ）+ 手はんだの方が安くなることがあります。
   - `NJL5820R-TE4` = C3271782 は LCSC 在庫 **1 個**（2026-10-11）。L/R×数量分は確実に足りないため長納期扱い。
6. **BOM に行が残る NO-CODE 品**は JLC の部品マッチングで「Customer Part」として扱われます。
   実装を依頼する場合は JLC へ部品送付、実装させない場合は BOM 行を削除してください。

## 発注パラメータ（参考）

- メイン基板: 2 層 / 1.6mm / ゴールド / 緑マスク（推奨）— 旧 Eagle 発注と同一設定。
- 実装面: 基板全面が F.Cu（上面）配置。CPL の Layer はすべて Front（エンコーダ基板のみ Bottom 混在）。
- 坐标系: KiCad 原点基準 mm（kicad-cli の `pcb export pos` とも突合済み。差異はテストポイントのみ）。
