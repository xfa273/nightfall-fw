# KiCad migration review for mini_r2_0

KiCad 10.0.3 で Eagle から初回インポートした直後の ERC/DRC レポートを分類します。

## 全体方針

- 初回インポート直後の違反は、変換由来、Eagle 時代の設計ルール差、実設計上の要確認項目に分けて扱います。
- KiCad 側の `.kicad_dru` は全基板とも `(version 1)` のみで、Eagle の設計ルールは十分には移行されていません。
- DRC の clearance / solder_mask / silk 系は、まず Eagle 側の製造条件と KiCad の Board Setup を合わせてから再評価します。
- 回路図やPCBの自動修正は行わず、KiCad GUIで対象位置を目視確認してから個別に修正します。

## 優先度 A: 実接続に影響する可能性がある項目

### 本体基板 `HM_Nightfall-mini-2e_v1`

- `shorting_items`: 3件
  - `MOTOR_L_OUT1` と U3 周辺の無ネットポリゴン
  - `MOTOR_R_OUT1` と U2 周辺の無ネットポリゴン
- `unconnected_items`: 84件
- `via_dangling`: 69件
- `track_dangling`: 2件

これらは変換後のポリゴンネット割り当て、GNDベタ、モータドライバ周辺、ビア接続状態を優先確認します。

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

これらは製造先の最小配線幅、最小クリアランス、ソルダーマスク最小幅、シルク条件を KiCad Board Setup に設定したあと再実行します。

## 基板別の初回レビュー優先度

| 基板 | 優先確認 |
| --- | --- |
| 本体基板 | `shorting_items`, `unconnected_items`, GNDポリゴン, モータドライバU2/U3, IMU/FRAM SPI2, SWD/UART |
| 左エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| 右エンコーダ基板 | 電源未駆動、未解決テキスト変数、GND/3V3、外形近傍クリアランス |
| ST-LINK変換基板 | +5V/+3V3/GNDの電源未駆動、NRST/BOOT0ラベル、ソルダーマスクブリッジ |
| UART変換基板 | +5V/+3V3/GNDの電源未駆動、SWDIO/SWCLK/NRST/SWO孤立ラベル、ソルダーマスクブリッジ |

## 次の作業

1. Eagle 時代に使用していた製造ルール、または発注先の最小ルールを確認します。
2. KiCad の Board Setup にクリアランス、トラック幅、ビア、ソルダーマスク、シルク条件を設定します。
3. 本体基板の `shorting_items` と `unconnected_items` をKiCad GUIで優先確認します。
4. 電源未駆動は、実回路として問題ない電源ネットに `PWR_FLAG` を追加するか、ERC除外にするかを個別判断します。
5. レビュー後にERC/DRCを再生成し、レポートを更新します。
