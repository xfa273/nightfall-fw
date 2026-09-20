# Encoder PCB L/R: 小径半穴パッド（2026-09-21）

## 設計変更

`cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-L/` と
`cad/kicad/HM_Nightfall-mini-2e_Encoder-PCB-R/` の PCB を編集正とする。
剥がれやすい片面の端子 `EC_L1..4` / `EC_R1..4` を、表面ランド →
端面のめっき半穴 → 裏面ランドが連続する構造へ変更した。
ユーザー指定により、基板外形を維持し、小径半穴の製造可否は個別確認とする。

| 項目 | 設計値 |
| --- | --- |
| 基板外形 | 3.45 × 7.00 mm（変更なし） |
| 板厚 | KiCad 既存設定 1.60 mm（変更なし。発注時に実物仕様と照合） |
| 端子 | 各基板4個、パッド番号 `TP`、既存ネットを維持 |
| 半穴 | Ø0.30 mm、PTH、中心は下辺 `y = 108.5036 mm` 上 |
| 銅箔ランド | 両面とも 0.60 × 1.20 mm、外形加工後は内側へ0.90 mm残る |
| ランド位置 | 穴中心から基板内側へ0.30 mmオフセット |
| 穴の左右の銅箔幅 | 0.15 mm（外形加工前の設計値） |
| レジスト | 表裏ともランドと同寸法の開口、局所拡大量0 |
| ペースト | 端子には開口なし |
| 下辺以外の端子銅箔と側辺の距離 | 最小0.205 mm（輪郭中心線基準） |
| 最小端子間銅箔間隔 | L: 0.1338 mm / R: 0.1390 mm |
| 最小半穴間隔（穴端間） | L: 0.4338 mm / R: 0.4390 mm |

両端の端子1・4は、側辺との銅箔クリアランスを確保するため、それぞれ
0.1988 mm / 0.2060 mmだけ内側へ寄せた。基板寸法、センサ・R・Cの位置、
ネット割当、既存配線・ビア、回路図は維持した。
端子2・3の横位置も維持しているため、元からあった左右基板の端子3の
0.0254 mmの位置差も残っている。

| 端子 | ネット | 穴中心X: L [mm] | 穴中心X: R [mm] |
| --- | --- | ---: | ---: |
| 1 | +3V3 | 147.2811 | 147.2811 |
| 2 | GND | 148.0221 | 148.0221 |
| 3 | EC_L_B / EC_R_B | 148.9873 | 148.9619 |
| 4 | EC_L_A / EC_R_A | 149.7211 | 149.7211 |

KiCadの各端子には `pad_prop_castellated` を指定し、Board Setupにも
`castellated_pads yes` を設定した。DRCルールの緩和や違反の除外はしていない。
基板内フットプリントを直接更新しており、Eagle由来の名称 `TP06SQ_385` は残す。
旧ライブラリ・Eagle原本から端子を置換すると片面パッドへ戻るため、置換しないこと。
Eagleデータと過去の出力アーカイブは変更前の参照用であり、今回の製造原本ではない。

## 製造条件

**この設計はJLCPCB標準加工条件を満たす発注確定データではない。**
JLCPCBの[半穴加工案内](https://jlcpcb.com/help/article/what-is-castellated-holes)は
穴径・穴間隔とも0.60 mm以上とし、[製造能力表](https://jlcpcb.com/capabilities/Capab)は
半穴基板の最小外形を10 × 10 mmとしている（2026-09-21確認）。
今回の穴径、穴間隔、外形はその範囲外であり、面付けだけで小径半穴の条件は解消しない。

製造先へ上記寸法、板厚、角近傍の穴位置、アニュラ幅0.15 mmを提示し、
小径半穴加工と面付け・保持方法の可否を確認する。
表裏の銅箔・レジスト、PTHドリル、穴中心を横切る直線の `Edge.Cuts` を渡す。
半円をNPTHや外形の切欠きへ置き換えない。穴内壁のめっきが必要である。
加工対象は4穴だけであり、端子間を含む下辺全体の連続めっきは指定しない。
剥離強度の改善量は未測定なので、試作後に手はんだ・引張りで確認する。

製造照会用の指定文:

> Four plated castellated holes per board, nominal diameter 0.30 mm, centered
> on the indicated straight board edge. Preserve the plated half-barrels and
> connect each to its own top and bottom land. Do not plate the entire edge or
> bridge adjacent terminals. Finished outline: 3.45 x 7.00 mm. Review the small
> holes, 0.15 mm annular copper, corner proximity, board thickness and panel
> support before fabrication. This is a custom-process feasibility request.

シルクはインポート時の大きな参照番号が残っているため、今回のCAM確認は
`F.Cu,B.Cu,F.Mask,B.Mask,Edge.Cuts` とPTH/NPTHドリルを対象とした。
発注時のシルクは別途整理する。

## 検証

KiCad 10.0.3で変更前後のDRCを同じプロジェクトルール・全severityで比較した。

| 検査 | L | R |
| --- | ---: | ---: |
| 変更前のDRCエラー | 10 | 10 |
| 変更後のDRCエラー | 6 | 6 |
| 新規DRC違反 | 0 | 0 |
| 未接続 | 0 | 0 |
| 警告 | 0 | 0 |

端子4個の銅箔端クリアランス違反が各基板で解消した。
残存は既存のØ0.20 mmビアの穴径違反（L:5件 / R:6件）と、
Lの既存 `N$1` ビアの側辺クリアランス0.1808 mm（1件）である。
今回のパッド変更で追加・抑制した違反ではなく、製造先への確認項目として残す。

KiCadの読み込みと3D描画、Gerber/Excellon出力を確認した。
各PTHドリルにØ0.30 mmの半穴が4個あり、NPTHには含まれない。
Gerber jobの `Castellated: true`、表裏銅箔とレジストのランド一致を確認した。
外形・配線・既存ビア・その他の部品・ネットの不変性も確認した。
この検証は設計データの整合確認であり、メーカーの加工可否や実物の剥離強度の確認ではない。

再確認コマンド（`kicad-cli` は環境に応じてフルパスを指定）:

```sh
kicad-cli pcb drc --severity-all --format json -o /tmp/encoder-drc.json <board.kicad_pcb>
kicad-cli pcb export gerbers --layers F.Cu,B.Cu,F.Mask,B.Mask,Edge.Cuts -o /tmp/encoder-cam/ <board.kicad_pcb>
kicad-cli pcb export drill --excellon-separate-th --generate-report -o /tmp/encoder-cam/ <board.kicad_pcb>
```
