# NVM機体識別ブロック仕様（Phase 3 草案）

この文書は、`nvm/nvm_identity.h` で定義した機体識別ブロックの仕様草案です。

---

## 1. 目的

- MCU系列内で共通バイナリを機体ごとに切り替える一次情報とする
- 起動時に識別ブロックを検証し、不正時は通常走行へ遷移しない
- ログメタ情報・安定版manifestと同じ識別項目を使う土台にする

---

## 2. 構造体（`nvm_identity_block_t`）

先頭4項目は共通ヘッダとして扱う。

- `magic` : `NVM_IDENTITY_MAGIC`（`0x4E464944`）
- `schema_version` : `NVM_IDENTITY_SCHEMA_VERSION`（現状 `0x00010000`）
- `length` : 構造体全体サイズ（`sizeof(nvm_identity_block_t)`）
- `crc` : payload 部分（先頭16byte以降）の 32bit additive checksum

識別情報本体。

- `family`
- `board_id`
- `hw_rev_major`
- `hw_rev_minor`
- `unit_serial`
- `default_param_profile`
- `capability_flags`
- `mcu_uid[3]`（補助情報）

---

## 3. 検証ルール

`nvm_identity_validate()` は以下を順に確認する。

1. `magic` 一致
2. `schema_version` 一致
3. `length >= 16` を満たすこと
4. `length == sizeof(nvm_identity_block_t)` を満たすこと
5. `crc` 一致

いずれか不一致なら無効として扱う。

---

## 4. 現在の実装状態

- API土台: 実装済み
  - `nvm_identity_read()`
  - `nvm_identity_validate()`
  - `nvm_identity_is_valid_for_boot()`
- 起動時判定:
  - `STM32F413` では `HAL_Init()` 直後に識別ブロックを読み、起動ログ出力後に判定する
  - `NVM_STATUS_INTEGRITY_ERROR` / `NVM_STATUS_HW_ERROR` はセーフモード遷移（LED点滅ループ）
  - `NVM_STATUS_NOT_FOUND` / `NVM_STATUS_UNSUPPORTED` は起動継続（暫定運用）
- 検証ロジック:
  - 先頭16byteより短い `length` は checksum 計算前に `NVM_STATUS_INTEGRITY_ERROR` として棄却する
  - checksum 仕様は従来どおり、先頭16byte以降に対する 32bit additive checksum を維持する
- 予約領域:
  - `STM32F405`: 読み出し先を `0x08080000`（sector 8, 128KiB）として有効化済み
    - 既存の `distance_params` `0x080A0000`（sector 9）, `flash_params` `0x080C0000`（sector 10）, maze/eeprom `0x080E0000`（sector 11）とは非衝突
    - 現状は read path のみ有効で、`nvm_write()` / `nvm_erase()` は未実装
  - `STM32F413`: 識別ブロック予約領域は未確定のため、現状は `NVM_STATUS_UNSUPPORTED`

---

## 5. 今後の確定事項

- `STM32F413` 向け識別ブロック予約領域（通常アプリ書き込みで消えない場所）
- `family` / `board_id` / `default_param_profile` の符号化ルール
- セーフモード遷移時の表示・ログ方針
