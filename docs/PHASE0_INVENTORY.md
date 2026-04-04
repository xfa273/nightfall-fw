# Phase 0 棚卸し結果

## 1. 現行ソースファイルの分類

### 1.1 HAL直接依存（platform に残すべき）

| ファイル | HAL呼び出し数 | 主な依存 | 備考 |
|---|---|---|---|
| `main.c` | 145 | GPIO,ADC,SPI,TIM,UART,Flash | CubeMX生成 + USER CODE |
| `stm32f4xx_hal_msp.c` | 64 | 全ペリフェラルMSP | CubeMX生成 |
| `stm32f4xx_it.c` | 6 | 割り込みハンドラ | CubeMX生成 |
| `system_stm32f4xx.c` | - | SystemInit | CubeMX生成 |
| `syscalls.c` | - | newlib stub | toolchain依存 |
| `sysmem.c` | - | _sbrk | toolchain依存 |

### 1.2 HAL中程度依存（board 層候補、抽象化して共通APIへ）

| ファイル | HAL呼び出し数 | 主な依存 | 共通化方針 |
|---|---|---|---|
| `sensor.c` | 35 | ADC DMA, SPI (IMU) | IMU/ADC抽象化API |
| `drive.c` | 44 | TIM PWM, GPIO (motor/buzzer/fan) | motor/buzzer/fan抽象化API |
| `auxiliary.c` | 29 | GPIO (LED/button), UART (printf) | LED/button/trace抽象化API |
| `interrupt.c` | 24 | TIM callback, ADC callback | 制御周期コールバック抽象化 |

### 1.3 NVM直接依存（nvm 層へ移行）

| ファイル | Flash sector | アドレス | 用途 |
|---|---|---|---|
| `eeprom.c` | Sector 11 | `0x080E0000` | 迷路データ保存 |
| `flash_params.c` | Sector 10 | `0x080C0000` | センサ基準値・IMUオフセット |
| `distance_params.c` | Sector 9 | `0x080A0000` | 距離補正3点ワープ |

### 1.4 共通ロジック（common / app へ移行候補）

| ファイル | 内容 | HAL依存 |
|---|---|---|
| `control.c` | PID制御 (velocity/omega/distance/angle/wall) | なし（変数経由） |
| `search.c` | 迷路探索 | なし |
| `solver.c` | 経路導出 | eeprom経由のみ |
| `solver_params.c` | ソルバプロファイル | なし |
| `path.c` | パス生成 | なし |
| `run.c` | 走行実行 | drive/sensor経由 |
| `maze_grid.c` | 迷路グリッド操作 | なし |
| `sensor_distance.c` | 距離変換（ワープテーブル） | なし |
| `logging.c` | ログ収集・CSV出力 | printf (UART) |

### 1.5 モード定義（app 層候補）

| ファイル | 内容 |
|---|---|
| `mode1.c` ~ `mode7.c` | 走行モード定義 |
| `test_mode.c` | テスト・校正モード |

---

## 2. 永続データ（NVM）の棚卸し

### 2.1 現行 Flash sector 使用状況

| Sector | アドレス | サイズ | 用途 | 保護 |
|---|---|---|---|---|
| 0-8 | `0x08000000`~ | ~512KB | アプリケーション | flash_uart erase対象 |
| 9 | `0x080A0000` | 128KB | distance_params | flash_uart 保護 |
| 10 | `0x080C0000` | 128KB | flash_params (sensor) | flash_uart 保護 |
| 11 | `0x080E0000` | 128KB | eeprom (迷路) | flash_uart 保護 |

### 2.2 各データの形式

| データ | magic | version | crc | schema管理 |
|---|---|---|---|---|
| `distance_params` | `0x44495354` ('DIST') | `0x00010000` | additive checksum | あり |
| `flash_params` | `0x50415231` ('PAR1') | `0x00010001` | additive checksum | あり |
| `eeprom` (迷路) | なし | なし | なし | **なし（要改善）** |

### 2.3 機体識別ブロック

- **現状**: 存在しない
- **今後**: 内蔵Flash予約領域に新設する
- **候補位置**: アプリ領域の末尾手前または別途予約した sector/page

---

## 3. トレース・ログ経路の棚卸し

### 3.1 トレース（printf相当）

| 項目 | 現状 |
|---|---|
| 物理経路 | UART (USART1) |
| リターゲット | `__io_putchar()` → `HAL_UART_Transmit()` |
| 使用箇所 | `auxiliary.c`, `main.c`, `test_mode.c`, `mode*.c`, `logging.c` |
| 出力内容 | mode/case表示、build info、校正結果、エラー |
| 抽象化 | **なし（直接 printf）** |

### 3.2 ログ（デバッグ・調整用）

| 項目 | 現状 |
|---|---|
| 保存先 | RAM (`LogBuffer` × 2 + `SensorLogBuffer`) |
| 容量 | `MAX_LOG_ENTRIES` = 1000 × 2バッファ + sensor 1000 |
| 配置 | `log_buffer` = 通常RAM, `log_buffer2` = CCMRAM, `sensor_log_buffer` = CCMRAM |
| 出力方式 | 走行後に `log_print_all()` で UART CSV 出力 |
| メタ情報 | `#fw_target`, `#fw_git_describe`, `#fw_branch`, `#fw_dirty`, `#mm_columns` |
| 抽象化 | **なし（logging.c に UART出力が直接埋め込み）** |

---

## 4. _sandbox/ 内の成果物と正式移動先

| 現在のパス | 内容 | 正式移動先 |
|---|---|---|
| `_sandbox/f413_preorder/notes/f413_cad_pin_list.md` | ピン一覧 | `hardware/mini_r2_0/notes/` |
| `_sandbox/f413_preorder/notes/f413_cad_pin_list.pdf` | ピン一覧PDF | `hardware/mini_r2_0/notes/` |
| `_sandbox/f413_preorder/notes/HM_Nightfall-mini-2e_v1.sch` | Eagle回路図 | `hardware/mini_r2_0/cad/` |
| `_sandbox/f413_preorder/notes/HM_Nightfall-mini-2e_v1.brd` | Eagle基板 | `hardware/mini_r2_0/cad/` |
| `_sandbox/f413_preorder/notes/Nightfall-mini-2e_v1*.pdf` | 回路図PDF | `hardware/mini_r2_0/cad/` |
| `_sandbox/f413_preorder/notes/default.dru` | DRCルール | `hardware/mini_r2_0/cad/` |
| `_sandbox/f413_preorder/notes/erc*.png` | ERCスクリーンショット | `hardware/mini_r2_0/notes/` |
| `_sandbox/f413_preorder/notes/スクリーンショット*.png` | DRCスクリーンショット | `hardware/mini_r2_0/notes/` |
| `_sandbox/f413_preorder/cubemx_project/` | F413 CubeMXプロジェクト | `platform/stm32f413/` |

---

## 5. 既存名 → 新命名の移行表

| 旧名 | 新正式名 | family | HW Rev | MCU | 備考 |
|---|---|---|---|---|---|
| `mini_v1_1` | `mini_r1_1` | `mini` | `r1.1` | STM32F405 | 初代ハーフ機体 + 基板修正1回 |
| `classic_v2` | `classic_r2_0` | `classic` | `r2.0` | STM32F405 | 2世代目クラシック機体 |
| `f413_preorder` | `mini_r2_0` | `mini` | `r2.0` | STM32F413 | 新機体（MCU+FRAM変更=major up） |

### 5.1 CMakeターゲット名の移行

| 旧ターゲット | 現ターゲット | 将来（MCU系列統合後） |
|---|---|---|
| `nightfall_mini_v1_1` | `nightfall_mini_r1_1` | `nightfall_stm32f405` |
| `nightfall_classic_v2` | `nightfall_classic_r2_0` | `nightfall_stm32f405`（同一バイナリ） |
| （なし） | （なし） | `nightfall_stm32f413` |

### 5.2 params ディレクトリの移行

| 旧パス | 新パス |
|---|---|
| `params/mini_v1_1/` | `params/mini_r1_1/` |
| `params/classic_v2/` | `params/classic_r2_0/` |
| （なし） | `params/mini_r2_0/` |

---

## 6. ビルドシステムの現状

| 項目 | 現状 |
|---|---|
| ビルドシステム | CMake + Ninja + arm-none-eabi |
| プリセット | `CMakePresets.json` (Debug/Release) |
| ターゲット数 | 2 (`nightfall_mini_v1_1`, `nightfall_classic_v2`) |
| CubeMX統合 | `cmake/stm32cubemx/` |
| ビルド識別 | `cmake/gen_build_info.cmake` → `build_info.h` |
| ソース共有 | `NIGHTFALL_USER_SOURCES` を両ターゲットで共有 |
| variant切替 | `params/<variant>/params.h` + compile define |
| 関数 | `nightfall_add_firmware(target variant variant_define)` |
| stm32f405固有 | `nightfall_apply_stm32f405()` (cmake/stm32cubemx) |

### 6.1 ビルドシステムで変更が必要な箇所

- `nightfall_add_firmware()` を MCU系列ベースに再設計
- `nightfall_apply_stm32f405()` に加えて `nightfall_apply_stm32f413()` を追加
- ターゲット名を MCU系列ベース (`nightfall_stm32f405`, `nightfall_stm32f413`) へ
- variant 切替は機体識別ブロック（実行時）に移行するため、ビルド時の variant define は MCU系列単位のみに変更

---

## 7. フラグ管理の現状

### 7.1 走行状態フラグ (`mouse_flags MF`)
- 64bit bitfield union
- `RUNNING`, `FAILED`, `SCND`, `RETURN`, `CTRL`, `OVERRIDE` など走行中の状態

### 7.2 デバッグフラグ (`DebugFlags_t g_debug`)
- `test_mode_run`
- `disable_wall_end_correction`
- `disable_front_wall_correction`
- `sensor_log_enabled`
- `angle_accum_mode`

### 7.3 params.h マクロ
- 機体寸法、壁基準値、制御ゲイン、しきい値など

---

## 8. Phase 0 完了判定

- [x] 開発ポリシー更新済み
- [x] 開発計画作成済み
- [x] 命名規則、HW Rev、安定バージョン lifecycle 確定済み
- [x] ハード依存箇所の棚卸し完了
- [x] 永続データの棚卸し完了
- [x] ログ経路の棚卸し完了
- [x] `_sandbox/` の正式移動先決定
- [x] 移行表作成
