# KiCad migration review for mini_r2_0

KiCad 10.0.3 で Eagle から初回インポートした直後の ERC/DRC レポートを分類します。

## 全体方針

- 初回インポート直後の違反は、変換由来、Eagle 時代の設計ルール差、実設計上の要確認項目に分けて扱います。
- KiCad 側の `.kicad_dru` は全基板とも `(version 1)` のみで、Eagle の設計ルールは十分には移行されていません。
- PCB は通常 JLCPCB へ発注するため、KiCad の Board Setup は JLCPCB の標準的な2層PCB向けに安全側へ揃えます。
- JLCPCB の注文説明では、1/2層基板でソルダーレジストブリッジを成立させるにはパッド/ピン間隔が通常 0.2 mm 以上必要とされています。
- 回路図やPCBの自動修正は行わず、KiCad GUIで対象位置を目視確認してから個別に修正します。

## JLCPCB 向け Board Setup

全KiCadプロジェクトの `.kicad_pro` に以下の基準を設定します。

- `min_clearance`: 0.2 mm
- `min_track_width`: 0.15 mm
- `min_via_diameter`: 0.5 mm
- `min_via_annular_width`: 0.1 mm
- `min_through_hole_diameter`: 0.3 mm
- `min_hole_clearance`: 0.25 mm
- `min_hole_to_hole`: 0.25 mm
- `min_copper_edge_clearance`: 0.2 mm
- `min_silk_clearance`: 0.1 mm
- `solder_mask_to_copper_clearance`: 0.0 mm
- track width presets: 0.15 / 0.2 / 0.25 / 0.5 mm
- via preset: 0.5 mm diameter / 0.3 mm drill

## GNDゾーン復元

Eagle 原本の本体基板には `GND` / `GND2` のポリゴンが4つありました。

- `GND`: F.Cu / B.Cu
- `GND2`: F.Cu / B.Cu

KiCad 初回インポート後のPCBには `zone` が0件だったため、これらをKiCadゾーンとして復元し、`kicad-cli pcb drc --refill-zones --save-board` で塗り直しました。

## 優先度 A: 実接続に影響する可能性がある項目

### 本体基板 `HM_Nightfall-mini-2e_v1`

- `shorting_items`: 3件
  - `MOTOR_L_OUT1` と U3 周辺の無ネットポリゴン
  - `MOTOR_R_OUT1` と U2 周辺の無ネットポリゴン
- `unconnected_items`: 0件
- `via_dangling`: 0件
- JLCPCB 向けルール適用後のDRC: 461 errors / 253 warnings

GND/GND2ゾーン復元により大量の未接続とGNDビアのdanglingは解消しました。残る `shorting_items` はモータドライバ U2/U3 周辺のポリゴン/パッド接続として、KiCad GUIで実位置を確認します。

### 回路図 ERC

- `pin_to_pin`: MP6551 の `OUT1/OUT1_2`, `OUT2/OUT2_2` など、同一出力端子の統合表現か実短絡か確認します。
- `pin_not_connected`: MCU、IMU、モータドライバの未使用ピンか確認します。
- `wire_dangling`: Eagle からの変換でワイヤ端点がずれた可能性を確認します。

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
- `silk_over_copper`
- `silk_overlap`
- `silk_edge_clearance`
- `text_thickness`
- `text_height`
- `copper_edge_clearance`

これらはJLCPCB向け Board Setup 適用後も残っています。多くは既存Eagle設計の密度、シルク配置、ソルダーマスクブリッジ条件に由来する可能性があるため、Gerber生成前に個別確認します。

## 基板別の初回レビュー優先度

| 基板 | 優先確認 |
| --- | --- |
| 本体基板 | `shorting_items`, モータドライバU2/U3, JLCPCBクリアランス, ソルダーマスクブリッジ, IMU/FRAM SPI2, SWD/UART |
| 左エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| 右エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| ST-LINK変換基板 | +5V/+3V3/GNDの電源未駆動、NRST/BOOT0ラベル、ソルダーマスクブリッジ |
| UART変換基板 | +5V/+3V3/GNDの電源未駆動、SWDIO/SWCLK/NRST/SWO孤立ラベル、ソルダーマスクブリッジ |

## 次の作業

1. 本体基板の `shorting_items` 3件をKiCad GUIで優先確認します。
2. JLCPCB向けDRCで残る `clearance` / `solder_mask_bridge` を製造上許容できるか確認します。
3. 電源未駆動は、実回路として問題ない電源ネットに `PWR_FLAG` を追加するか、ERC除外にするかを個別判断します。
4. レビュー後にERC/DRCを再生成し、レポートを更新します。
