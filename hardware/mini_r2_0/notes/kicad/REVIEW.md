# KiCad migration review for mini_r2_0

KiCad 10.0.3 で Eagle から初回インポートした直後の ERC/DRC レポートを分類します。

## 全体方針

- 初回インポート直後の違反は、変換由来、Eagle 時代の設計ルール差、実設計上の要確認項目に分けて扱います。
- KiCad 側の `.kicad_dru` は全基板とも `(version 1)` のみで、Eagle の設計ルールは十分には移行されていません。
- PCB は通常 JLCPCB へ発注するため、KiCad の Board Setup は JLCPCB の標準的な2層PCB向けに安全側へ揃えます。
- JLCPCB の注文説明では、1/2層基板でソルダーレジストブリッジを成立させるにはパッド/ピン間隔が通常 0.2 mm 以上必要とされています。
- この 0.2 mm は銅箔クリアランスではなくソルダーマスクブリッジの目安として扱います。
- 回路図やPCBの自動修正は行わず、KiCad GUIで対象位置を目視確認してから個別に修正します。

## JLCPCB 向け Board Setup

全KiCadプロジェクトの `.kicad_pro` に以下の基準を設定します。

- `min_clearance`: 0.127 mm
- netclass clearance: 0.127 mm
- `min_track_width`: 0.127 mm
- `min_via_diameter`: 0.5 mm
- `min_via_annular_width`: 0.1 mm
- `min_through_hole_diameter`: 0.3 mm
- `min_hole_clearance`: 0.25 mm
- `min_hole_to_hole`: 0.25 mm
- `min_copper_edge_clearance`: 0.2 mm
- `min_silk_clearance`: 0.1 mm
- `solder_mask_to_copper_clearance`: 0.0 mm
- track width presets: 0.127 / 0.15 / 0.2 / 0.25 / 0.5 mm
- via preset: 0.5 mm diameter / 0.3 mm drill
- `solder_mask_bridge`: warning
- `silk_overlap` / `silk_over_copper` / `silk_edge_clearance`: ignore

## GNDゾーン復元

Eagle 原本の本体基板には `GND` / `GND2` のポリゴンが4つありました。

- `GND`: F.Cu / B.Cu
- `GND2`: F.Cu / B.Cu

KiCad 初回インポート後のPCBには `zone` が0件だったため、これらをKiCadゾーンとして復元し、`kicad-cli pcb drc --refill-zones --save-board` で塗り直しました。

## 優先度 A: 実接続に影響する可能性がある項目

### 本体基板 `HM_Nightfall-mini-2e_v1`

- `shorting_items`: 0件
- `unconnected_items`: 0件
- `via_dangling`: 0件
- JLCPCB 向けルール適用後のDRC: 4 errors / 112 warnings
- JLCPCB Gerber Viewer で旧 Eagle 発注データと KiCad 正式候補 Gerber を比較し、外形・穴・主要配線・旧 `GTO` 相当トップシルクが概ね一致することを確認済みです。

GND/GND2ゾーン復元により大量の未接続とGNDビアのdanglingは解消しました。MP6551 の `shorting_items` は、Eagleからの変換で U2/U3 の pin 1 が小さいpadと無ネット銅ポリゴンに分離されたことが原因でした。U2/U3 の pin 1 を通常のSMD padへ統合し、無ネット銅ポリゴンを削除して解消しました。

残る `copper_edge_clearance` 2件は C14 の `+5V` / `GND2` パッド端と基板外形の距離が 0.127 mm で、JLCPCB 向け設定の 0.2 mm を下回るためです。該当箇所はコンデンサのパッド端で、パッドの一部が欠けても電気的接続への影響は小さく、回避スペースもないため許容扱いにします。

残る `starved_thermal` 2件は U5 STM32F413 の `GND` パッド 35 / 47 で、F.Cu の `GND` ゾーンへのサーマルリリーフ接続が最小スポーク数2本に対して1本しか形成できない判定です。対象はMCUのGNDパッドで大電流経路ではなく、マイクロマウス用途では長時間連続稼働や産業用途レベルの冗長性を要求しないため、接続冗長性と熱バランス上の注意点として認識した上で許容扱いにします。

### モータドライバ出力ピン

MP6551 の出力ピンは同一出力が2ピンずつ出ています。

- `OUT1_2` / `OUT1`: pin 1 / pin 10
- `OUT2_2` / `OUT2`: pin 3 / pin 8

これらは外部で接続すべきピンです。ERC の `pin_to_pin` は、KiCad のピン種別がどちらも `Output` になっているために出ている変換後チェック上の警告として扱います。回路としては接続方針を維持します。

DRC の `shorting_items` は、同じ出力ピン接続そのものではなく、変換されたMP6551フットプリント内の無ネットポリゴンとパッド/配線の関係で発生していました。KiCad保存時に `fp_poly` のnet属性は維持されなかったため、無ネットポリゴンを残さず、pin 1 pad自体を `1.3 mm x 0.2 mm` に修正する方針にしました。

### 回路図 ERC

- `pin_to_pin`: MP6551 の `OUT1/OUT1_2`, `OUT2/OUT2_2` など、同一出力端子の統合表現か実短絡か確認します。
- `pin_not_connected`: MCU、IMU、モータドライバの未使用ピンか確認します。
- `wire_dangling`: Eagle からの変換でワイヤ端点がずれた可能性を確認します。

本体基板のKiCad正式候補GerberはPCB実配線と旧Eagle発注データの比較を優先して判断します。残るERCは、Eagleからの変換後チェック上の未接続ピン、PWR_FLAG相当不足、footprint library参照名、ワイヤ端点表現が中心です。今回の発注候補には直接反映せず、今後KiCad回路図を編集正として本格運用する段階で、`PWR_FLAG` 追加やERC除外を個別に行います。

## 優先度 B: 変換・ルール差の可能性が高い項目

### フットプリントリンク

- すべての基板で `footprint_link_issues` が出ています。
- Eagle インポート時のローカルシンボル/フットプリント参照名がKiCadライブラリ表に完全対応していないことが主因と考えます。
- まず既存PCB上のフットプリント形状と参照番号が維持されているかを確認し、必要ならKiCadネイティブライブラリへ置換します。

### 電源未駆動

- `power_pin_not_driven` は、Eagleの電源記号がKiCadのPWR_FLAG相当へ変換されないことで出ることがあります。
- 本体、エンコーダ基板、ST-LINK変換基板、UART変換基板で、+5V、+3V3、GND、テストポイント系を確認します。

### 未解決テキスト変数

- エンコーダ基板の `PTR1TP06SQ` や本体基板のテストポイント系で発生しています。
- Eagleライブラリ由来の属性変数がKiCadで未解決になったものとして、表示名・製造情報に影響があるか確認します。

## 優先度 C: 製造ルール調整後に再評価する項目

- `clearance`
- `solder_mask_bridge`
- `text_thickness`
- `text_height`
- `copper_edge_clearance`

これらはJLCPCB向け Board Setup 適用後も残っています。`clearance` は銅箔間隔0.127 mm未満だけを実エラーとして扱います。1.27 mmピッチピンヘッダのpad径は Eagle 由来の `1.158 mm` では隣接pad間が約0.112 mmになっていたため、`1.1 mm` へ縮小して解消しました。`solder_mask_bridge` は細ピッチICやコネクタでソルダーマスクの島が残らない可能性を示すため、warningとして扱います。

JLCPCB向けDRCで残る `solder_mask_bridge` 99件は、主に K1 の1.27 mm SWD/UARTヘッダ、U5 の STM32 UFQFPN、U4 の ISM330DHCXTR 周辺です。これはハンダマスク開口同士が近く、パッド間のレジスト島が残らない可能性を示すもので、銅箔ショートではありません。旧 Eagle 発注データと KiCad 正式候補 Gerber の比較で主要形状が一致しており、既に `clearance` と `shorting_items` は0件であるため、今回の本体基板発注候補では製造上の注意点として許容扱いにします。

## シルク出力方針

Eagle では、印刷したいものだけを test 系レイヤへ割り当て、それ以外のシルクはGerber作成時に無視していました。KiCad では、製造で印刷したいものだけを `F.SilkS` / `B.SilkS` に置き、印刷しない参照番号・値・補助表示は `F.Fab` / `B.Fab` または非表示にします。

移行直後はEagle由来の参照番号や値が `F.SilkS` に大量に残っているため、シルク系DRCは以下を `ignore` にします。

- `silk_overlap`
- `silk_over_copper`
- `silk_edge_clearance`

製造用Gerberを出す際は、シルクを整理するまでは `F.SilkS` / `B.SilkS` を出力しません。最終的に印刷したいtest相当の表示だけをKiCadの `F.SilkS` / `B.SilkS` へ移してからシルクGerberを出力します。

旧 Eagle 発注済み Gerber は `cad/export/reference/eagle_ordered_2026-03-22/HM_Nightfall-mini-2e_v1_eagle_ordered_2026-03-22.zip` に保存します。旧発注データでは `GTO` がトップシルク、`GBO` はほぼ空でした。KiCad 変換後の `F.SilkS` をそのまま出すと不要な参照番号等が混ざるため、今回の正式候補 Gerber では旧 `GTO` / `GBO` を KiCad Gerber の外形中心へ平行移動して重ねます。旧 Eagle Gerber は基板中心原点、KiCad Gerber は外形中心 `(148.501102, -105.003601)` の絶対座標です。

KiCad 正式候補 Gerber は `cad/export/jlcpcb/HM_Nightfall-mini-2e_v1_kicad_jlcpcb_candidate_20260606/HM_Nightfall-mini-2e_v1_kicad_jlcpcb_candidate_20260606.zip` とします。

## 基板別の初回レビュー優先度

| 基板 | 優先確認 |
| --- | --- |
| 本体基板 | 基板端クリアランス, サーマル接続不足, ソルダーマスクブリッジ, IMU/FRAM SPI2, SWD/UART |
| 左エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| 右エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| ST-LINK変換基板 | +5V/+3V3/GNDの電源未駆動、NRST/BOOT0ラベル、ソルダーマスクブリッジ |
| UART変換基板 | +5V/+3V3/GNDの電源未駆動、SWDIO/SWCLK/NRST/SWO孤立ラベル、ソルダーマスクブリッジ |

## Eagle参照データの保全

メイン基板以外は改変・再発注の可能性が低いため、KiCad完全移行よりも Eagle が使えるうちの参照データ確保を優先します。2026-06-06時点で以下を保存しました。

- `hardware/mini_r2_0/cad/export/reference/eagle_archive_20260606/`
- `hardware/mini_r1_0/cad/export/reference/eagle_archive_20260606/`
- `hardware/classic_r1_0/cad/export/reference/eagle_archive_20260606/`

各アーカイブには Eagle 原本、CAM/DRU/SCR、Eagle CLIで生成したCAM出力、ドリル、ネットリスト、部品表、ボード画像を含めています。これらは将来の比較・復旧用であり、そのまま現行の発注候補とは扱いません。

## 次の作業

1. 本体基板のJLCPCB正式発注前に、JLCPCB Gerber ViewerでKiCad正式候補zipを再度開き、外形・穴・表裏銅箔・トップシルク・ドリルが表示されることを最終確認します。
2. 本体基板をKiCadで継続編集する段階で、回路図ERCの `PWR_FLAG`、未接続ピン、footprint library参照名を個別に整理します。
3. メイン基板以外を再発注する必要が出た場合だけ、保存済みEagle参照アーカイブを基準にKiCad移行またはGerber再生成を行います。
