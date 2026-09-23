# CM_Nightfall-Air-2a_v0

## 開き方

`CM_Nightfall-Air-2a_v0.kicad_pro` をKiCad 10で開いてください。回路図とPCBは同期済みです。

## 回路の由来

`HM_Nightfall-mini-3a_v0` を複製し、次の部品をクラシックサイズの `CM_Nightfall-Air_v1` と同じものへ変更しました。

| Reference | mini-3a | Air 2 |
|---|---|---|
| Q2（機体電源） | PJS6403 / SOT-23-6 | SI7135DP-T1-GE3 / PowerPAK SO-8 |
| Q3（吸引ファン） | IRLML6344 / SOT-23 | PMPB13XNE,115 / DFN2020MD-6 |
| U1（3.3 V LDO） | AP2210N-3.3TRG1 / SOT-23（300 mA） | NJM2884U1-33 / SOT-89-5（500 mA） |
| PT_FL0、PT_FR0、PT_L0、PT_R0 | LED記号による代用 / 3 mm LED外形 | ST-1KL3A / 2端子TO-18 |
| IR_LED_FL0、IR_LED_FR0、IR_LED_L0、IR_LED_R0 | LED3MM | SFH4550 / T1 3/4 |

ST-1KL3AはLED記号の代用をやめ、Collector/Emitterを持つ2端子NPNフォトトランジスタとして扱っています。データシートに従い、ST-1KL3Aは1=Emitter・2=Collector、SFH4550は1=Cathode・2=Anodeです。いずれもリードピッチは2.54 mmです。

壁センサとIR投光LEDは機械マウントで基板から浮かせるため、両フットプリントには部品本体外形とCourtyardを設けていません。基板上には2.54 mm間隔の2パッドだけがあり、穴径0.9 mm、ランド径1.5 mm、環状幅0.3 mmです。角の丸い1番パッドで極性を判別します。

U1はAir_v1と同じ接続で、1=CONTROLと5=VINをともに+5V、2=GNDをGND、4=VOUTを+3V3へ接続し、3=NCは未接続です。入力C14=1 uF、出力C15=2.2 uFはNJM2884でも使用できるため変更していません。フットプリントはAir_v1のランド寸法を再現したプロジェクトローカル版です。

ゲート抵抗、保護ダイオード、2S–3S対応電源、MPM3610、モータドライバ、MCU、FRAMなどはmini-3aから変更していません。FRAM U6は `CY15B108QI-20LPXC` のままです。

## DXF取り込み

- 基板外形: `source/Main-PCB.DXF`
- 製造時に印刷するシルク: `source/Main-PCB_Silk.DXF`
- 外形: 48 × 80 mm
- 閉じたLINE/ARC輪郭: 6
- 中央切り欠き: 直径14 mm
- NPTH: 直径1.5 mm × 4
- F.Silkscreen図形: 51

2026-09-22のモータマウント形状更新を反映済みです。外形の閉じた輪郭は従来と同じで、4個のNPTH中心は左右それぞれ0.5 mm内側へ移動しました。シルクには新しいマウント外形の線・円弧を追加しています。更新前のPCBとDXFは `.history/before-motor-mount-20260922/` に保管しました。配置・配線後のDXF差し替えには `tools/update_dxf_geometry.py` を使用してください。

部品フットプリントの通常シルクは `User.5` にあり、`F.Silkscreen` にはDXF由来図形だけがあります。

## 現在の状態

- 回路図部品: 102、PCBフットプリント: 106（固定用NPTH 4個を含む）
- 配線: 55、ビア: 3（2026-09-22時点）
- ベタ: 0
- 部品配置: 作業中
- 回路図ERC: 0件

PCB DRCは違反96件、未配線218件で、配置・配線作業中の状態です（2026-09-22時点）。今回のDXF更新前後でDRCの種類・件数は変わっていません。

外形と製造シルクのプレビューは `verification/CM_Nightfall-Air-2a_v0-outline-silk.svg` です。灰色がEdge.Cuts、黄色がF.Silkscreenです。

`tools/prepare_classic_project.py` は初期PCBを再生成するための記録です。配置・配線開始後に実行すると作業を上書きするため、通常は実行しないでください。
