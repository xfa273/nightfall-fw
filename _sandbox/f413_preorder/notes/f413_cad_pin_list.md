# STM32F413CHU6 CAD参照ピンリスト（Nightfall F413 Preorder）

最終更新: 2026-03-22

このファイルは、回路図/CAD作業時に参照するための固定ピン一覧です。

- 対象MCU: **STM32F413CHU6 (UFQFPN48)**
- 生成元: CubeMXプロジェクト `HM_Nightfall_f413_preorder.ioc`
- 参照ソース:
  - `cubemx_project/HM_Nightfall_f413_preorder/HM_Nightfall_f413_preorder.ioc`
  - `cubemx_project/HM_Nightfall_f413_preorder/Core/Inc/main.h`
  - `cubemx_project/HM_Nightfall_f413_preorder/Core/Src/stm32f4xx_hal_msp.c`

---

## 1. 信号ベース一覧（CAD配線用）

| 信号名 | MCUピン | 種別 | 周辺機能/AF | 備考 |
|---|---|---|---|---|
| SENSOR_FR | PA0 | ADC入力 | ADC1_IN0 | フロント右センサ |
| SENSOR_R | PA1 | ADC入力 | ADC1_IN1 | 右センサ |
| SENSOR_FL | PA2 | ADC入力 | ADC1_IN2 | フロント左センサ |
| SENSOR_L | PA3 | ADC入力 | ADC1_IN3 | 左センサ |
| VOL_CHECK | PB0 | ADC入力 | ADC1_IN8 | バッテリ電圧監視 |
| EC_L_A | PA6 | タイマ入力 | TIM3_CH1 / AF2 | 左エンコーダA |
| EC_L_B | PA7 | タイマ入力 | TIM3_CH2 / AF2 | 左エンコーダB |
| EC_R_A | PB6 | タイマ入力 | TIM4_CH1 / AF2 | 右エンコーダA |
| EC_R_B | PB7 | タイマ入力 | TIM4_CH2 / AF2 | 右エンコーダB |
| MOTOR_L_PWM | PA5 | PWM出力 | TIM2_CH1 / AF1 | 左モータPWM |
| MOTOR_R_PWM | PB10 | PWM出力 | TIM2_CH3 / AF1 | 右モータPWM（CH4ではなくCH3） |
| FAN_PWM | PB8 | PWM出力 | TIM10_CH1 / AF3 | 吸引ファンPWM |
| BUZZER_PWM | PB9 | PWM出力 | TIM11_CH1 / AF3 | ブザーPWM |
| MOTOR_L_DIR | PA4 | GPIO出力 | - | 左モータDIR |
| MOTOR_R_DIR | PB1 | GPIO出力 | - | 右モータDIR |
| MOTOR_STBY | PB2 | GPIO出力 | - | モータドライバSTBY |
| FRAM_CS | PA8 | GPIO出力 | - | SPI2共有バスのFRAM CS |
| IMU_CS | PB12 | GPIO出力 | - | SPI2共有バスのIMU CS |
| SPI2_SCK | PB13 | SPI | SPI2_SCK / AF5 | IMU/FRAM共通クロック |
| SPI2_MISO | PB14 | SPI | SPI2_MISO / AF5 | IMU/FRAM共通MISO |
| SPI2_MOSI | PB15 | SPI | SPI2_MOSI / AF5 | IMU/FRAM共通MOSI |
| USART1_TX | PA9 | UART | USART1_TX / AF7 | ログ/デバッグ用 |
| USART1_RX | PA10 | UART | USART1_RX / AF7 | ログ/デバッグ用 |
| IR_FR | PC13 | GPIO出力 | - | IR制御（FR） |
| IR_L | PC14 | GPIO出力 | - | IR制御（L）, 本来OSC32_IN兼用 |
| IR_FL | PC15 | GPIO出力 | - | IR制御（FL）, 本来OSC32_OUT兼用 |
| IR_R | PB5 | GPIO出力 | - | IR制御（R） |
| LED_1 | PA15 | GPIO出力 | - | LED |
| LED_2 | PA11 | GPIO出力 | - | LED |
| LED_3 | PH1 | GPIO出力 | - | LED, 本来OSC_OUT兼用 |
| PUSH_IN_1 | PH0 | GPIO入力 | - | スイッチ入力, 本来OSC_IN兼用 |
| SWDIO | PA13 | デバッグ | SYS_JTMS-SWDIO | SWD必須 |
| SWCLK | PA14 | デバッグ | SYS_JTCK-SWCLK | SWD必須 |
| SWO | PB3 | デバッグ | SYS_JTDO-SWO | トレース利用時 |

---

## 2. ポート順一覧（クロスチェック用）

### Port A
- PA0: SENSOR_FR (ADC1_IN0)
- PA1: SENSOR_R (ADC1_IN1)
- PA2: SENSOR_FL (ADC1_IN2)
- PA3: SENSOR_L (ADC1_IN3)
- PA4: MOTOR_L_DIR (GPIO Out)
- PA5: MOTOR_L_PWM (TIM2_CH1 / AF1)
- PA6: EC_L_A (TIM3_CH1 / AF2)
- PA7: EC_L_B (TIM3_CH2 / AF2)
- PA8: FRAM_CS (GPIO Out)
- PA9: USART1_TX (AF7)
- PA10: USART1_RX (AF7)
- PA11: LED_2 (GPIO Out)
- PA13: SWDIO
- PA14: SWCLK
- PA15: LED_1 (GPIO Out)

### Port B
- PB0: VOL_CHECK (ADC1_IN8)
- PB1: MOTOR_R_DIR (GPIO Out)
- PB2: MOTOR_STBY (GPIO Out)
- PB3: SWO
- PB5: IR_R (GPIO Out)
- PB6: EC_R_A (TIM4_CH1 / AF2)
- PB7: EC_R_B (TIM4_CH2 / AF2)
- PB8: FAN_PWM (TIM10_CH1 / AF3)
- PB9: BUZZER_PWM (TIM11_CH1 / AF3)
- PB10: MOTOR_R_PWM (TIM2_CH3 / AF1)
- PB12: IMU_CS (GPIO Out)
- PB13: SPI2_SCK (AF5)
- PB14: SPI2_MISO (AF5)
- PB15: SPI2_MOSI (AF5)

### Port C
- PC13: IR_FR (GPIO Out)
- PC14: IR_L (GPIO Out, OSC32_IN兼用)
- PC15: IR_FL (GPIO Out, OSC32_OUT兼用)

### Port H
- PH0: PUSH_IN_1 (GPIO In, OSC_IN兼用)
- PH1: LED_3 (GPIO Out, OSC_OUT兼用)

---

## 3. CADでの注意点

1. **デバッグ配線は必ず引き出す**
   - PA13(SWDIO), PA14(SWCLK), NRST, GND, 3V3
   - PB3(SWO)は任意（トレース使う場合は推奨）

2. **OSC兼用ピンをGPIO運用している**
   - PC14/PC15, PH0/PH1 をGPIOとして使用中
   - 外部32kHz/外部HSE水晶を使う設計とは両立しない

3. **SPI2はIMU/FRAM共通バス**
   - CSは `FRAM_CS(PA8)` と `IMU_CS(PB12)` を独立配線

4. **起動時のGPIO初期状態（CubeMX生成コード基準）**
   - 次の出力は初期Low設定: `IR_FL, IR_L, IR_FR, LED_3, FRAM_CS, MOTOR_L_DIR, LED_1, MOTOR_R_DIR, MOTOR_STBY, IMU_CS, LED_2, IR_R`
   - 外付け回路側で必要ならプルアップ/プルダウンを追加する

---

## 4. 変更時ルール（運用）

- ピン変更を行った場合は、CubeMX再生成後にこのファイルを更新すること。
- 更新時は、少なくとも `main.h` と `stm32f4xx_hal_msp.c` で一致確認すること。

---

## 5. 機体側デバッグ/給電コネクタ（推奨）

想定用途: 将来の「ST-LINK V3MINIE + 5V分岐」アダプタ基板と接続し、
**PC給電中に SWD書き込み + UART printf** を行う。

`BOOT0` は常用コネクタに含めず、テストパッドで運用する。

### 5.1 必要信号

- 電源: `+5V_DBG`, `GND`
- SWD: `SWDIO(PA13)`, `SWCLK(PA14)`, `NRST`, `VTREF_3V3`
- printf(VCP): `USART1_TX(PA9)`, `USART1_RX(PA10)`
- 任意: `SWO(PB3)`

### 5.2 推奨コネクタ

- **10ピン / 1.27mmピッチ / 2x5 ボックスヘッダ（キー付き）**
- 理由:
  - 2.54mmより小型化できる
  - IDCソケットで挿抜しやすく、静止状態での書き込み運用に向く
  - 極性キーで逆挿しを防ぎやすい
  - 競技用試作機向けとして、GND本数を最小限にして信号本数を確保しやすい

### 5.3 推奨ピンアサイン（機体側）

| Pin | 信号名 | 方向（機体基準） | 備考 |
|---|---|---|---|
| 1 | +5V_DBG | IN | アダプタ基板からの5V入力 |
| 2 | GND | - | 電源GND |
| 3 | SWDIO | I/O | PA13 |
| 4 | SWCLK | IN | PA14 |
| 5 | NRST | IN | MCUリセット |
| 6 | VTREF_3V3 | OUT | 機体3.3VをST-LINK基準電圧として供給 |
| 7 | BOOT0_CTRL | IN | 通常Low、High+ResetでSystem Bootloader起動 |
| 8 | UART_TX | OUT | PA9 (USART1_TX) |
| 9 | UART_RX | IN | PA10 (USART1_RX) |
| 10 | SWO | OUT | PB3, 任意（未使用時NC可） |

### 5.4 実装メモ

1. `NRST` はコネクタに必ず出す（量産時の復旧性向上）。
2. `VTREF_3V3` は「電源供給」ではなく「電圧基準共有」用途。
3. `GND` は1本運用のため、コネクタ近傍でGNDプレーンへ短く太く接続する。
4. `BOOT0` は基板上で10kΩ程度のプルダウンを常設し、外部スイッチで3.3Vへ切替できるようにする。
5. `+5V_DBG` は保護素子（ヒューズ/逆流防止）を介して基板5V系へ接続する。
6. モータ・ファンの大電流駆動は、デバッグ給電経路の許容電流を超えないよう別途設計する。

### 5.5 BOOT0切替運用

- 通常運用（SWD書き込み/printf）: `BOOT0=Low`
- USARTブートローダ運用: `BOOT0=High` にして `NRST` を再投入
