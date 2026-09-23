# hardware/mini_r3_0

Nightfall mini 3a (`mini_r3_0`) の手動アートワーク開始用データです。

## 開き方

KiCad 10 で次のプロジェクトファイルを開いてください。

`cad/kicad/HM_Nightfall-mini-3a/HM_Nightfall-mini-3a_v0.kicad_pro`

プロジェクトディレクトリには回路図、PCB、設計ルール、ローカルのシンボル／
フットプリントライブラリを同梱しています。別のチェックアウトに依存せず編集できます。

## 引き渡し状態

- 回路図: ERC 0
- PCB: 4層 (`F.Cu / In1.Cu / In2.Cu / B.Cu`)
- 外形、取付穴、機構シルク: 導入済み
- 部品: 102個を配置済み
- 配線、ビア、ゾーン: 0（手動配線開始前）
- U5: 下面露出パッドを使わない手はんだ向けフットプリント
- PCBパッドネット: 最新回路図から再同期済み

未配線なので、PCB DRCの未接続表示が228件あるのは正常です。配線前の物理DRCには
clearance、short、hole、track-crossing、copper-edgeの違反はありません。残る警告は
ライブラリコピー差分とシルク／テキスト由来です。

## 4層の使い方

- `F.Cu / B.Cu`: 部品接続、高電流経路、局所信号
- `In1.Cu`: `GND / GND2` の分割基準面（信号トラックは置かない）
- `In2.Cu`: 低電流信号と `+3V3`
- `GND` と `GND2` は `R0` だけで接続
- 通常信号は 0.20 mm、通常viaは 0.60/0.30 mm
- QFN直近だけ 0.16 mm、局所toe viaは 0.40/0.20 mm
- モータ／ファン約2 A枝は原則0.80 mm以上、集約VBATは1.0 mm以上

回路と電源ドメインの詳細は `notes/POWER_ARCHITECTURE.md` を参照してください。

