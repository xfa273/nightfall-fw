# Phase 3 チェックリスト（機体識別ブロック + NVM基盤）

このチェックリストは、`docs/NIGHTFALL_FW_2026_DEVELOPMENT_PLAN.md` の Phase 3 を進めるための運用用メモです。

---

## 1. 現在の進捗

- [x] `nvm` API の骨格を追加（`nvm_init`, `nvm_read`, `nvm_write`, `nvm_erase`）
- [x] 機体識別ブロック構造体と検証APIを追加（`nvm_identity_*`）
- [x] 機体識別ブロック仕様草案を文書化（`docs/NVM_IDENTITY_BLOCK_SPEC.md`）
- [~] F405/F413で識別ブロックの予約領域アドレスを確定
- [x] F405で識別ブロック read path 用の予約領域を有効化（`0x08080000`, sector 8）
- [ ] F413で識別ブロックの予約領域アドレスを確定
- [x] `nvm_read()` の境界チェックをオーバーフロー安全化
- [x] `nvm_identity_validate()` で不正な `length` を checksum 計算前に棄却
- [ ] 起動初期で識別ブロックを読んでセーフモード判定する
- [ ] 既存 `distance_params` / `flash_params` / `eeprom` の移行計画を作成

---

## 2. 運用・品質の定型手順

### 2.1 ビルド確認（毎回）

- `cmake --build --preset Debug-stm32f405`
- `cmake --build --preset Debug-stm32f413`

### 2.2 実験ログ更新（作業単位）

- `docs/EXPERIMENT_LOG.md` に topic / 目的 / 結果 / 参照コミットを追記する。

### 2.3 進捗反映（フェーズ節目）

- `docs/NIGHTFALL_FW_2026_DEVELOPMENT_PLAN.md` の進捗サマリを更新する。
