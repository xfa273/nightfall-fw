# nvm/

このディレクトリは永続化層（NVM abstraction）を管理します。

## 役割

- 内蔵 Flash / 外付け FRAM へのアクセスを抽象化
- 機体識別ブロックの読み書き API（Phase 3 以降）
- `distance_params`, `flash_params`, `eeprom` の責務整理
- CRC / schema version / migration の共通化

## 現在の状態

既存実装は `platform/stm32f405/Core/Src/*.c` に分散しています。
Phase 3 先行として、以下の土台を追加済みです。

- `nvm.h`, `nvm.c`
  - `nvm_init`, `nvm_get_area_info`, `nvm_read`, `nvm_write`, `nvm_erase`
- `nvm_identity.h`, `nvm_identity.c`
  - 機体識別ブロック構造体
  - `nvm_identity_read`, `nvm_identity_write`, `nvm_identity_validate`
- `nvm_params.h`, `nvm_params.c`
  - `distance_params` / `flash_params` / `eeprom(maze)` の呼び出しラッパ
  - `nvm_params_distance_*`, `nvm_params_sensor_*`, `nvm_maze_*`

制約:

- 機体識別ブロック read path は F405/F413 ともに有効化済み（write/erase運用は別途整備が必要）
- `nvm_write` / `nvm_erase` は `STM32F405` の internal flash backend で最小実装済み
- `STM32F413` は `NVM_AREA_IDENTITY` / `NVM_AREA_DISTANCE_PARAMS` / `NVM_AREA_FLASH_PARAMS` / `NVM_AREA_MAZE_MAP` の internal flash backend で `nvm_write` / `nvm_erase` を最小実装済み

進捗（Step A）:

- `distance` の load/save 呼び出しを `nvm_params` 経由へ移行
- `sensor` の params load/save 呼び出しを `nvm_params_sensor_*` 経由へ移行
- `maze(eeprom)` の read/write 呼び出しを `nvm_maze_*` 経由へ移行

進捗（F413 暫定運用）:

- `nvm_params_distance_save` / `nvm_params_sensor_load/save/defaults` / `nvm_maze_save_map/load_map` を `nvm` API経由で実装
- `nvm_params_distance_load_and_apply` は blob読込/整合性検証後に `sensor_distance_set_warp_*_3pt` で補正適用まで実装
