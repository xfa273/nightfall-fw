# mini_r3_0 2S–3S 電源回路 v0

## 目的

現行 F413 機体 `HM_Nightfall-mini-2e` をベースに、3S 対応 Classic 機体 `CM_Nightfall-Air_v1` の MPM3610 電源と吸引ファン駆動を移植する。

## 電源ドメイン

```text
2S–3S battery
    -> SI7135DP reverse-polarity / main-switch stage
    -> VBAT_SW
       |- MP6551 left/right motor drivers
       |- 2S–3S rated suction fan
       |- battery ADC divider
       `- MPM3610 buck -> +5V logic -> AP2210 -> +3V3
```

MPM3610 の連続出力は 1.2A なので、モータとファンは 5V 出力へ接続せず `VBAT_SW` から直接給電する。

## Classic から移植した回路

- Q2: `SI7135DP-T1-GE3` P-channel MOSFET
- R12 100k と POWER0: メインスイッチのゲート回路
- U7: `MPM3610GQV-P`
- C22: 10uF / 25V X7R 入力コンデンサ
- C23: 22uF / 10V X7R 出力コンデンサ
- R31: 100k EN pull-up
- Q3: `PMPB13XNE,115` ファン low-side MOSFET
- R32: 10k gate pull-down
- D4: `BAS16WT1GW` flyback diode
- C24: 0.1uF / 25V X7R ファンノイズ抑制
- J2: 吸引ファン 2-pin コネクタ

## v0 で修正した値と接続

- Classic の FB 分圧 `75k / 18k` は MPM3610 の 0.798V 基準では約 4.12V になるため、そのまま移植せず `100k / 19.1k` とした。typical 出力は約 4.98V。
- MPM3610 の IN / EN / OUT / FB / AGND / PGND は公式 pinout に合わせ、OUT 7–9、PGND 12–14、SW 4–6 をグループ化した。
- AAM は floating (CCM)、VCC / SW / BST / NC は外部無接続とした。
- 両 MP6551 の VIN を `+5V` から `VBAT_SW` へ変更した。
- モータ入力バルク C7 / C9 を 10uF / 25V X7R とした。
- バッテリ ADC 分圧を `22k / 10k` から `100k / 27k` へ変更した。3S 満充電 12.6V で ADC 入力は約 2.68V。
- MCU の既存 `FAN_PWM` (U5 pin 45 / PB8) を Q3 gate へ接続した。

## 一次資料で確認した範囲

- [MPM3610 product page](https://www.monolithicpower.com/en/products/power-management/power-modules/step-down/mpm3610.html): 4.5–21V input、1.2A continuous。
- [MPM3610 datasheet](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MPM3610GQV-Z/document_id/2090): pinout、0.798V FB、10uF input / 22uF output、EN pull-up、layout guidance。
- [MP6551 product page](https://www.monolithicpower.com/en/products/motor-drivers-and-motor-controllers/mp6551.html): 2.5–14V input。3S 満充電 12.6V は範囲内だが余裕は 1.4V。
- [Si7135DP product page](https://www.vishay.com/en/product/68807/): -30V P-channel、PowerPAK SO-8。
- [PMPB13XNE datasheet](https://assets.nexperia.com/documents/data-sheet/PMPB13XNE.pdf): fan low-side MOSFET の pinout / package 確認元。

## PCB 前に必須の確認

1. MPM3610 と PMPB13XNE のフットプリントを Classic Eagle 原本から移植し、メーカー推奨ランドと全 pin mapping を照合する。
2. SI7135DP の source 1–3 / drain 5–8 と PowerPAK SO-8 pad mapping、バッテリ極性を照合する。
3. MPM3610 の入力コンデンサ、PGND、OUT、FB をデータシート推奨どおり最短配置する。
4. MP6551 の 14V 上限に対し、3S 回生・配線サージ・バッテリ切離し時の電圧を実測する。
5. ファンが 2S–3S 直結に対応すること、または PWM duty 上限が必要かを部品確定時に決める。
6. JST-PH2 を含むバッテリコネクタ、Q2、配線幅、銅厚の連続・ピーク電流定格を確認する。
7. F413 firmware のバッテリ電圧換算を `VBAT = ADC_voltage * 127 / 27` に更新する。
8. ERC ベースラインを整理し、電源入力・未接続 pin・フットプリント割当を全回路で確認する。

## 回路図表記

- `VBAT_RAW` / `VBAT_SW` / `+5V` は上向きの電源シンボル、`GND2` は下向きの GND シンボルで表記した。
- 機能ブロック内の EN / FB / fan- / main-switch gate は直接配線し、長距離信号の `FAN_PWM` / `VOL_CHECK` だけネットラベルを表示した。
- MPM3610、ファン、バッテリ入力、ADC 分圧は、電源を上、GND を下にそろえた。

## 検証済み接続ノード

- `VBAT_RAW`: P1 battery+、Q2 source 1–3、R12
- main-switch gate: Q2 gate、R12、POWER0
- `VBAT_SW`: Q2 drain、U7 IN、U2/U3 VIN、J2 fan+、D4 cathode、C22/C24、R4、R31
- `+5V`: U7 OUT 7–9、C23、AP2210 VIN。U2/U3 VIN は含まない
- MPM3610 EN: U7 EN、R31
- MPM3610 FB: U7 FB、R29、R30
- `FAN_PWM`: U5 PB8、Q3 gate、R32
- fan-: J2 fan-、Q3 drain、D4 anode、C24
- `GND2`: U7 AGND/PGND、Q3 source、R30/R32、C22/C23
