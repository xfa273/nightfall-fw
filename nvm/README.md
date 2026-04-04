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
  - `nvm_identity_read`, `nvm_identity_validate`

制約:

- 機体識別ブロック領域アドレス（F405/F413）は未確定
- `nvm_write` / `nvm_erase` は現時点では `NVM_STATUS_UNSUPPORTED` を返す

次段で、予約領域確定と既存 `distance_params` / `flash_params` / `eeprom` の段階移行を行います。
