# NVM移行計画（distance_params / flash_params / eeprom）

この文書は、Phase 3 の未完了項目である既存永続化実装の移行計画をまとめたものです。

---

## 1. 現状

- `distance_params`
  - 実装: `platform/stm32f405/Core/Src/distance_params.c`
  - 保存先: Sector 9 (`0x080A0000`)
  - 形式: `magic/version/length/crc` あり
- `flash_params`
  - 実装: `platform/stm32f405/Core/Src/flash_params.c`
  - 保存先: Sector 10 (`0x080C0000`)
  - 形式: `magic/version/length/crc` あり
- `eeprom`（maze）
  - 実装: `platform/stm32f405/Core/Src/eeprom.c`
  - 保存先: Sector 11 (`0x080E0000`)
  - 形式: `magic/version/length/crc` なし（生データ）

課題:

- アプリ層/機能層からFlashレイアウトが直接見えている
- `eeprom` はスキーマ管理がなく破損時判定が弱い
- MCU系列差を吸収する境界が明確でない

---

## 2. 目標

- アプリ層から生のFlash sector/アドレス参照を排除する
- `nvm/` API経由で `distance_params` / `flash_params` / `maze` を扱う
- `eeprom` 相当データにも `magic/version/length/crc` を導入する
- F405（内蔵Flash）とF413（将来FRAM含む）で同一APIを使える形にする

---

## 3. 移行方針

### Step A: 読み出し経路の集約（互換維持）

- `nvm` に用途別APIを追加する
  - `nvm_params_distance_load/save`
  - `nvm_params_sensor_load/save`
  - `nvm_maze_load/save`
- 既存の実体書き込みは当面既存実装を呼ぶ（ラッパ）
- 呼び出し側（`sensor.c`, `search.c` など）を直接実装から切り離す

進捗（2026-04-05）:

- 実装済み
  - `nvm/nvm_params.h`, `nvm/nvm_params.c` を追加
  - `nvm_params_distance_load_and_apply/save` を導入
  - `nvm_params_sensor_load/save/defaults` を導入
  - `nvm_maze_*` ラッパ（enable/disable/write/read）を導入
  - 呼び出し側の一部を `nvm` 経由へ移行
    - `platform/stm32f405/Core/Src/main.c`（distance load）
    - `platform/stm32f405/Core/Src/test_mode.c`（distance save）
    - `platform/stm32f405/Core/Src/search.c`（maze read/write）
    - `platform/stm32f405/Core/Src/sensor.c`（sensor params load/save）

### Step B: データ形式の統一

- `maze` 用に新blob（`magic/version/length/crc`）を定義
- 旧形式が見つかった場合の読込み互換を残し、保存は新形式に統一
- 破損時は初期化 + トレース通知でセーフに復帰

進捗（2026-04-05）:

- 実装済み
  - `nvm_maze_save_map()` / `nvm_maze_load_map()` を追加
  - `maze` を `magic/version/length/crc + payload(uint16_t cells[])` 形式で保存
  - 新形式読込時に `length` と checksum を検証
  - 旧形式（raw halfword連続）を後方互換で読込
  - 未初期化（全 `0xFFFF`）を検出した場合はゼロ初期化扱いで `false` を返却
  - `search.c` の保存/読込を一括APIへ移行

### Step C: backend分離

- F405: 内蔵Flash backend
- F413: 初期は内蔵Flash read path、Phase4以降でFRAM backend追加
- `nvm_read/write/erase` をareaごとのbackendへディスパッチ

進捗（2026-04-05）:

- 実装済み（最小）
  - `nvm.c` に `nvm_get_backend()` を導入し、areaごとのbackend分岐を追加
  - `STM32F405` で `internal flash` backend の `nvm_erase()` / `nvm_write()` を実装
  - `STM32F413` で `NVM_AREA_IDENTITY` / `NVM_AREA_DISTANCE_PARAMS` / `NVM_AREA_FLASH_PARAMS` / `NVM_AREA_MAZE_MAP` の `internal flash` backend `nvm_erase()` / `nvm_write()` を実装
  - `STM32F413` の `nvm_params` で `distance_save` / `sensor save/load/defaults` / `maze save/load` を `nvm` API経由で実装し、FRAM無し暫定運用を準備
  - `STM32F413` の `nvm_params_distance_load_and_apply` は、blob読込/整合性検証に加え `sensor_distance_set_warp_*_3pt` で補正適用まで実装
- 次段
  - 実機完成後の `STM32F413` 内蔵Flash暫定運用検証は完了（2026-04-18）
    - `docs/F413_INTERNAL_FLASH_TEMP_VERIFICATION_CHECKLIST.md` の Step2〜5 を実機で確認済み
    - 保護セクタ維持（12-15）/ identity読込継続 / distance・sensor・maze の save+load と reboot後load-only を確認
  - Phase4初動として `STM32F413` の `distance/sensor/maze/trace` を外付けFRAM backendへ切替実装（2026-04-18）
    - `nvm.c` で `NVM_AREA_IDENTITY` は内蔵Flash維持、その他対象areaを `external FRAM` へディスパッチ
    - FRAM想定デバイス: `CY15B108QI`（8Mbit=1MB）
  - FRAM backend実機検証を完了（2026-04-19）
    - UARTコマンド `a/A/t/T` で `distance/sensor/maze/trace` の save+load / load-only がすべて PASS
  - trace_log schema v1 を固定（2026-04-25）
    - `nvm_trace_log_header_t` / `nvm_trace_log_record_t` を `nvm/nvm_trace_log.h` で定義
    - `NVM_AREA_TRACE_LOG` の `schema_version` を `0x00010000` へ更新
    - F413 UARTに `q(format) / r(append sample) / R(dump latest)` を追加
  - trace_log検証コマンドを拡張（2026-04-30）
    - F413 UARTに `v(dump csv)` / `k(selftest)` を追加
    - `k` は format→append→readbackの一連チェックを実機で即実行できる
  - trace_log自動収集の初版を追加（2026-04-30）
    - F413 UARTに `u(start auto)` / `U(stop auto)` を追加
    - `u` 実行時は trace area をformatし、定周期でappendする
  - 残課題: 教育用FRAM無し構成の切替運用を文書化

---

## 4. API境界（ドラフト）

- `nvm_area_t` は既存定義を継続利用
- 用途別の公開APIは `nvm/` 配下に分離
  - `nvm_distance_params.*`
  - `nvm_sensor_params.*`
  - `nvm_maze_map.*`
- 旧APIは段階的にdeprecated化し、最終的に `platform/stm32f405/Core/Src/*.c` の永続化直接実装を縮退

---

## 5. 完了条件

- アプリ層から `FLASH_SECTOR_*` や固定アドレス参照が消える
- `distance_params` / `flash_params` / `maze` が `nvm` API越しに利用される
- `maze` データにスキーマ管理と破損判定が導入される
- F405/F413の両系列でビルドが通る
