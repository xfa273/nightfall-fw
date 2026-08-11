# HM_Nightfall-mini-3a_v0 KiCad project

`HM_Nightfall-mini-2e_v1` の STM32F413 / センサ / FRAM / UI 回路をベースに、`CM_Nightfall-Air_v1` の 2S–3S 電源構成と吸引ファン駆動回路を移植した KiCad 10 プロジェクトです。

## ファイル

- `HM_Nightfall-mini-3a_v0.kicad_sch`: 回路図編集原本
- `HM_Nightfall-mini-3a_v0.kicad_pcb`: 外形・機構シルク・部品配置済み、未配線の PCB 原本
- `HM_Nightfall-mini-3a_v0.kicad_pro`: KiCad 10 プロジェクト設定
- `HM_Nightfall-mini-3a_v0-power.kicad_sym`: MPM3610 / IRLML6402TRPBF / IRLML6344TRPBF と電源シンボルのローカルシンボル
- `Nightfall-Power.pretty`: MPM3610 と PowerPAK SO-8 のローカルフットプリント
- `HM_Nightfall-mini-2e_v1.pretty`: mini-2e 基板から移行した既存フットプリントと、走行モータ用と同じ 1.27 mm パッド形状のファン配線用 `FAN_WIRE_PADS_2P`
- `sym-lib-table` / `fp-lib-table`: プロジェクトローカルライブラリの参照

## DXF と外形

同期済みの最終 DXF として次のデータを使用した。

`/Users/xfa273/Library/CloudStorage/OneDrive-個人用/MicroMouse/Half-Mouse/Nightfall-mini-2_v1/CAD/DXF`

- `Main-PCB.DXF`: 39 mm × 70 mm の後部開口なし外形、吸引口、4 個の取付穴を `Edge.Cuts` へ 1:1 で取込み
- `Main-PCB_Silk.DXF`: 機構部品位置を示す 42 直線＋1 円を `F.Silkscreen` へ取込み
- 両 DXF とも mm / scale 1.0、DXF 原点を基板中心 `(148.501098, 105.003598)` に合わせた。取り込んだ機構シルクの線幅は 0.25 mm とした。

## 配置方針

- 全 97 フットプリントを表面へ配置し、配線・ビア・ゾーンはまだ追加していない。
- MODE tact switch は `(138.214098, 138.150598)`、RESET tact switch は `(158.788098, 138.150598)` に維持し、POWER slide switch は後部中央 `(148.501098, 136.000000)` へ移動した。レバー部が後端の凹みに入る向きとしている。
- SWD コネクタ K1 は後部右側 `(165.265098, 121.640598, -90°)` の従来位置・向きを維持した。
- 吸引ファン範囲を示す中心 `(148.501098, 92.903598)`、半径 12.5 mm のシルク円内には部品を置かず、STM32F413 U5 も円周から後方へ退避した。吸引口と 4 個の取付穴も部品・銅箔を避ける。
- 左右の前壁センサは CM_Nightfall-Air_v1 と同様に LED / phototransistor を前後方向へそろえた縦 2 列とし、対応するセンサ駆動 FET も左右外周へ縦配置した。
- 左右モータマウントのシルク輪郭内は機構占有領域として空け、`R6/C1/C2` と `R1/R7/R2` を中央側または輪郭後方へ退避し、その間隔確保のため `R3` も中央側へ移動した。シルクに接する左右4個ずつのエンコーダ用パッド `EC_L1..4`／`EC_R1..4` は元の座標を維持した。
- P1 と mini-2e と同じ SOT-23 の `IRLML6402TRPBF` Q2 を後部左側で隣接させ、MPM3610 U7 と入力／出力／FB 部品を後部中央へまとめた。3.3 V LDO U1 とデカップリング部品も後部の増加領域へ移した。
- ブザー SPK1 と駆動部品は後部左側へ移し、操作部品や基板端とのクリアランスを確保した。
- ファン用 J2 は JST から走行モータ用と同じ直接はんだ付けパッドへ変更し、ファン円の左上外側 `(137.800000, 84.200000, 50°)` に配置した。SOT-23 Q3・D4・C24・R32 は後部右側へまとめた。
- MPM3610 の SW / OUT 複合ランドは、Classic Eagle 原本の形状を複数の同一番号矩形パッドで表現し、重複カスタムパッドによる異ネット短絡を避けた。

## 現在の確認状態

KiCad 10.0.3 CLI で回路図 ERC、PCB ロード、基板統計、DRC を確認した。回路図と PCB は 97 reference / 97 footprint で一致している。今回の配置について `courtyards_overlap`、`copper_edge_clearance`、`clearance`、`padstack_invalid`、新規の異ネット短絡は無い。ファン円と左右モータマウント輪郭についても機械チェックし、指定のエンコーダ用パッドを除いて機構領域内に部品がないことを確認した。

未配線段階なので 219 個の未接続が残る。さらに mini-2e Eagle インポート由来の IR センサフットプリント内短絡 2 件、ライブラリコピー差分、既存 no-connect pin の回路図等価性警告がベースラインとして残る。配置承認後に配線・ゾーン作成とこれらのベースライン整理へ進む。
