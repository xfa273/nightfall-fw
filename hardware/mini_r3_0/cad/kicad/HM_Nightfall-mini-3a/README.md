# HM_Nightfall-mini-3a_v0 KiCad project

`HM_Nightfall-mini-2e_v1` の STM32F413 / センサ / FRAM / UI 回路をベースに、`CM_Nightfall-Air_v1` の 2S–3S 電源構成と吸引ファン駆動回路を移植した KiCad 10 プロジェクトです。

## ファイル

- `HM_Nightfall-mini-3a_v0.kicad_sch`: 回路図編集原本
- `HM_Nightfall-mini-3a_v0.kicad_pcb`: 外形・機構シルク・部品配置済み、未配線の PCB 原本
- `HM_Nightfall-mini-3a_v0.kicad_pro`: KiCad 10 プロジェクト設定
- `HM_Nightfall-mini-3a_v0-power.kicad_sym`: MPM3610 / SI7135DP / IRLML6344TRPBF と電源シンボルのローカルシンボル
- `Nightfall-Power.pretty`: MPM3610 と PowerPAK SO-8 のローカルフットプリント
- `HM_Nightfall-mini-2e_v1.pretty`: mini-2e 基板から移行した既存フットプリント
- `sym-lib-table` / `fp-lib-table`: プロジェクトローカルライブラリの参照

## DXF と外形

指定された OneDrive フォルダ名そのものは Mac 側に無かったため、同じ 39 mm × 70 mm 外形を持つ次の同期済みデータを使用した。

`/Users/xfa273/Library/CloudStorage/OneDrive-個人用/MicroMouse/Half-Mouse/Nightfall-mini-2e_v1/CAD/DXF`

- `Main-PCB.DXF`: `Edge.Cuts` の外形・後部開口・取付穴が 1:1 で一致することを寸法確認
- `Main-PCB_Silk.DXF`: mm / scale 1.0 / line width 0.15 mm、DXF 原点を基板中心 `(148.501098, 105.003598)` に合わせて `F.Silkscreen` へ取込み

## 配置方針

- 全 97 フットプリントを表面へ配置し、配線・ビア・ゾーンはまだ追加していない。
- MODE tact switch は `(138.214098, 138.150598)`、RESET tact switch は `(158.788098, 138.150598)`、POWER slide switch は `(151.168098, 123.037598)` に維持した。
- SWD コネクタ K1 は後部右側 `(165.265098, 121.640598, -90°)` の従来位置・向きを維持した。
- P1 と PowerPAK Q2 を後部左側で隣接させ、MPM3610 U7 と入力／出力／FB 部品を中央左側へまとめた。3.3 V LDO U1 は機構レールを避けて中央右側へ移した。
- ファン用 J2 は後部右端、SOT-23 Q3・D4・C24・R32 はその下側へまとめ、ファン電流ループを短くできる並びにした。
- MPM3610 の SW / OUT 複合ランドは、Classic Eagle 原本の形状を複数の同一番号矩形パッドで表現し、重複カスタムパッドによる異ネット短絡を避けた。

## 現在の確認状態

KiCad 10.0.3 CLI で回路図ネットリスト、PCB ロード、基板統計、DRC を確認した。回路図と PCB は 97 reference / 97 footprint で一致し、割当フットプリント名も全て一致している。新規配置について `courtyards_overlap`、`copper_edge_clearance`、`padstack_invalid`、異ネット短絡は無い。

未配線段階なので 221 個の未接続が残る。さらに mini-2e Eagle インポート由来の IR センサフットプリント内短絡 2 件、ライブラリコピー差分、既存 no-connect pin の回路図等価性警告がベースラインとして残る。配置承認後に配線・ゾーン作成とこれらのベースライン整理へ進む。
