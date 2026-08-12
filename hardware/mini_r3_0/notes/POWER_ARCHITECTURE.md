# mini_r3_0 2S–3S 電源回路 v0

## 目的

現行 F413 機体 `HM_Nightfall-mini-2e` をベースに、3S 対応 Classic 機体 `CM_Nightfall-Air_v1` の MPM3610 電源と吸引ファン駆動を移植する。

## 電源ドメイン

```text
2S–3S battery
    -> PJS6403 P-channel main-switch stage
    -> VBAT_SW
       |- MP6551 left/right motor drivers
       |- 2S–3S rated suction fan
       |- battery ADC divider
       `- MPM3610 buck -> +5V logic -> AP2210 -> +3V3
```

`MPM3610GQV-P` の連続出力は 1A なので、モータとファンは 5V 出力へ接続せず `VBAT_SW` から直接給電する。

## Classic を基にした電源・ファン回路

- Q2: `PJS6403_S1_00001` P-channel MOSFET、SOT-23-6、-30V / ±20V
- R12 100k: source–gate pull-up
- R33 4.7k と POWER0: メインスイッチの gate pull-down 経路
- D5: `BZT52-C15S_R1_00001` 15V gate–source clamp
- U7: `MPM3610GQV-P`
- C22: 10uF / 25V X7R 入力コンデンサ
- C23: 22uF / 10V X7R 出力コンデンサ
- R31: 100k EN pull-up
- Q3: `IRLML6344TRPBF` ファン low-side MOSFET (`SOT-23`、G=1 / S=2 / D=3)
- R32: 10k gate pull-down
- D4: `SM340A` 3A / 40V Schottky flyback diode
- C24: 0.1uF / 25V X7R ファンノイズ抑制
- J2: 吸引ファン 2-pin コネクタ
- R34 / R35: 220k、左右 MP6551 の SR–GND2 スルーレート調整

## v0 で修正した値と接続

- Classic の FB 分圧 `75k / 18k` は MPM3610 の 0.798V 基準では約 4.12V になるため、そのまま移植せず `100k / 19.1k` とした。typical 出力は約 4.98V。
- MPM3610 の IN / EN / OUT / FB / AGND / PGND は公式 pinout に合わせ、OUT 7–9、PGND 12–14、SW 4–6 をグループ化した。
- AAM は floating (CCM)、VCC / SW / BST / NC は外部無接続とした。
- 両 MP6551 の VIN を `+5V` から `VBAT_SW` へ変更した。
- モータ入力バルク C7 / C9 を 10uF / 25V X7R とした。
- バッテリ ADC 分圧を `22k / 10k` から `100k / 27k` へ変更した。3S 満充電 12.6V で ADC 入力は約 2.68V。高インピーダンス分圧の ADC charge reservoir / PWM ノイズ LPF として、C25 `100nF X7R` を `VOL_CHECK`–logic `GND` 間の MCU PB0 直近に追加した。
- MCU の既存 `FAN_PWM` (U5 pin 45 / PB8) を Q3 gate へ接続した。
- 中央部の制御配線を交差させないため、物理ドライバの論理割当を
  `U2 = left`、`U3 = right` とした。U5 側は従来どおり
  `PA4 / PA5 = MOTOR_L_DIR / MOTOR_L_PWM`、
  `PB1 / PB10 = MOTOR_R_DIR / MOTOR_R_PWM` のままなので、CubeMX と
  制御ファームの左右定義は変更しない。U2 は `TP3 / TP4`、U3 は
  `TP1 / TP2` へ接続する。
- SPI CS は実装配線と FAN_PWM を同時に検証し、従来どおり
  `PA8 = FRAM_CS`、`PB12 = IMU_CS` とした。割当交換案は FAN_PWM と
  FRAM 配線が中央で競合するため採用しない。
- 2S 用機の `IRLML6402` は `VGS=±12V` / `VDS=-20V` で、3S 満充電 12.6V の直接 gate pull-down に余裕がない。Q2 を `PJS6403` へ変更し、4.7k 直列抵抗と15V gate–source clamp を追加した。
- Classic の `PMPB13XNE,115` は今回の最大約 2A・約 10秒という条件に対してパッケージが大きいため、Q3 を `IRLML6344TRPBF` へ変更した。30V / 5A、SOT-23、`RDS(on)` max 37mΩ @ `VGS=2.5V` なので 3.3V GPIO で駆動でき、2A 時の導通損失は 25℃データを使った保守的な計算でも約 0.15W。
- MP6551 の SR 直結は約 100ns の最速 edge となるため、左右に独立した 220k を追加した。初期値はノイズと 100kHz PWM 分解能の妥協であり、実機電流波形と100k / 220k / 470k の比較で最終決定する。

## 一次資料で確認した範囲

- [MPM3610 product page](https://www.monolithicpower.com/en/products/power-management/power-modules/step-down/mpm3610.html): 4.5–21V input。本機で使う `MPM3610GQV-P` は 1A 品として扱う。
- [MPM3610 datasheet](https://www.monolithicpower.com/en/documentview/productdocument/index/version/2/document_type/Datasheet/lang/en/sku/MPM3610GQV-Z/document_id/2090): pinout、0.798V FB、10uF input / 22uF output、EN pull-up、layout guidance。
- [MP6551 product page](https://www.monolithicpower.com/en/products/motor-drivers-and-motor-controllers/mp6551.html): 2.5–14V input。3S 満充電 12.6V は範囲内だが余裕は 1.4V。
- [PJS6403 datasheet](https://www.panjit.com.tw/en/Product/downloadPDF/PJS6403): -30V、`VGS=±20V`、`RDS(on)` max 32mΩ @ -10V、SOT-23-6。
- [IRLML6344 product page](https://www.infineon.com/part/IRLML6344) / [datasheet](https://www.infineon.com/assets/row/public/documents/24/49/infineon-irlml6344-datasheet-en.pdf): 30V、5A、SOT-23、`RDS(on)` max 29mΩ @ 4.5V / 37mΩ @ 2.5V、pinout G=1 / S=2 / D=3 の確認元。
- [秋月電子 106049](https://akizukidenshi.com/catalog/g/g106049/): `IRLML6344TRPBF` の国内入手先。2026-08-11 確認時点で通販在庫あり。

## PCB 前に必須の確認

1. MPM3610 のカスタムフットプリントをメーカー推奨ランドと照合する。Q3 は標準 `SOT-23` で、G=1 / S=2 / D=3、ランド寸法、2A 配線幅、放熱銅箔を PCB 上で再確認する。
2. PJS6403 の drain 1 / 2 / 5 / 6、gate 3、source 4とバッテリ極性を照合する。全 drain pad を同一 `VBAT_SW` 銅箔へ接続し、狭い neck を作らない。
3. MPM3610 の入力コンデンサ、PGND、OUT、FB をデータシート推奨どおり最短配置する。
4. MP6551 の 14V 上限に対し、3S 回生・配線サージ・バッテリ切離し時の電圧を実測する。
5. ファンが 2S–3S 直結に対応すること、または PWM duty 上限が必要かを部品確定時に決める。D4 `SM340A` の実ファン・PWM 周波数に対するパルス電流と発熱も実機で確認する。
6. JST-PH2 を含むバッテリコネクタ、Q2、配線幅、銅厚の連続・ピーク電流定格を確認する。
7. F413 の ADC1 rank 9 / channel 8 (`VOL_CHECK`) は、C25 と組み合わせて取得時間を 15 cycles から 56 cycles へ延長済み。ファームは `VBAT = ADC_voltage * 127 / 27` で実電圧へ換算し、起動時に2S/3Sを判定する。自動判定の曖昧帯、ADC未取得/停止、3.3V/cell以下ではモータとファンを禁止し、3.5V/cell以下では警告する。閾値と固定セル数指定は `params/f413_preorder/params.h` で調整できる。
8. ERC ベースラインを整理し、電源入力・未接続 pin・フットプリント割当を全回路で確認する。

## 4層基板の配線方針

- 2層で外層の電力・モータ・FAN・センサ・SPI配線まではDRC cleanに
  できたが、MCU周辺の全I/Oと+3V3配電を、デカップリング近接・連続した
  return・通常via条件を同時に満たして完了できなかった。このため最終版は
  ユーザーが許可した4層構成とする。
- JLCPCB の4層、外層2 oz / 内層1 oz、基板厚1.6 mmを発注条件とする。
  F.Cu/B.Cuの成立済み大電流経路を維持し、In1.CuはGND/GND2分割基準面、
  In2.Cuは低電流信号と+3V3配電に限定する。In2にも同じ境界の低優先度
  GND/GND2 pourを置き、信号経路を優先しながら銅バランスと局所returnを確保する。
  実際のdielectric/core厚は発注時に
  JLCPCBの対称stackupを選び、KiCadの総厚1.6 mmと照合する。
- 通常信号は
  `0.20 mm / 0.20 mm`、QFN の短い脱出部だけを `0.16 mm` とする。
- 通常 via は `0.60 / 0.30 mm`、QFN の局所脱出だけを
  `0.40 / 0.20 mm` とする。標準2層工程で樹脂充填・銅キャップを
  前提にしないため、信号・電源の via-in-pad は使用しない。
- モータ・ファンの約2 A枝は原則 `0.80 mm` 以上、集約された
  `VBAT_SW` は `1.0 mm` 以上の配線または銅箔面と並列 via bank を
  用いる。QFN端子直後の短いneckだけは部品ピッチに合わせる。
- In1のpower領域を`GND2`、ロジック・センサ領域を`GND`とし、両者はR0のみで
  接続し、境界を横断するモータ制御信号は R0 近傍へまとめる。
- MPM3610 の SW node、モータ出力、ファン電流経路は IMU、ADC、壁
  センサ、SPI から離し、入力コンデンサからICまでのhot loopを短くする。
- 手はんだ実装ではU5の下面露出padへはんだを供給できないため、側面pad
  1～48だけを持つ
  `Nightfall-Local:UFQFPN-48_7X7X0P55MM-M_noEP_hand_solder`を用いる。
  回路図symbolのpin 49はデータシートどおりGNDのまま残し、footprint側の
  pad 49と中央F.Pasteだけを意図的に省略する。

## 回路図表記

- `VBAT_RAW` / `VBAT_SW` / `+5V` は上向きの電源シンボル、`GND2` は下向きの GND シンボルで表記した。
- 機能ブロック内の EN / FB / fan- / main-switch gate は直接配線し、長距離信号の `FAN_PWM` / `VOL_CHECK` だけネットラベルを表示した。
- MPM3610、ファン、バッテリ入力、ADC 分圧は、電源を上、GND を下にそろえた。

## 検証済み接続ノード

- `VBAT_RAW`: P1 battery+、Q2 source pad 4、R12、D5 cathode
- main-switch gate: Q2 gate pad 3、R12、R33、D5 anode
- `/PWR_SWITCH_RETURN`: R33、POWER0
- `VBAT_SW`: Q2 drain、U7 IN、U2/U3 VIN、J2 fan+、D4 cathode、C22/C24、R4、R31
- `+5V`: U7 OUT 7–9、C23、AP2210 VIN。U2/U3 VIN は含まない
- MPM3610 EN: U7 EN、R31
- MPM3610 FB: U7 FB、R29、R30
- `FAN_PWM`: U5 PB8、Q3 gate、R32
- fan-: J2 fan-、Q3 drain、D4 anode、C24
- `GND2`: U7 AGND/PGND、Q3 source、R30/R32、C22/C23
