# nvm/

このディレクトリは永続化層（NVM abstraction）を管理します。

## 役割

- 内蔵 Flash / 外付け FRAM へのアクセスを抽象化
- 機体識別ブロックの読み書き API（Phase 3 以降）
- `distance_params`, `flash_params`, `eeprom` の責務整理
- CRC / schema version / migration の共通化

## 現在の状態

既存実装は `platform/stm32f405/Core/Src/*.c` に分散しています。
Phase 3 で API 設計、Phase 4 で FRAM 連携を実装します。
