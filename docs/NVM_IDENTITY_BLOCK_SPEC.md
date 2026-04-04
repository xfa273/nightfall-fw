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
3. `length` 一致
4. `crc` 一致

いずれか不一致なら無効として扱う。

---

## 4. 現在の実装状態

- API土台: 実装済み
  - `nvm_identity_read()`
  - `nvm_identity_validate()`
  - `nvm_identity_is_valid_for_boot()`
- F405/F413ともに識別ブロック領域アドレスは未確定のため、現状は `NVM_STATUS_UNSUPPORTED` を返す

---

## 5. 今後の確定事項

- MCU系列ごとの識別ブロック予約領域（通常アプリ書き込みで消えない場所）
- `family` / `board_id` / `default_param_profile` の符号化ルール
- セーフモード遷移時の表示・ログ方針
