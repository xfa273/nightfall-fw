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
  - `nvm_identity_write()`
  - `nvm_identity_validate()`
  - `nvm_identity_is_valid_for_boot()`
- 運用補助ツール:
  - `tools/flashing/make_identity_block.py` で識別ブロックバイナリを生成可能
  - 生成したバイナリは `tools/flashing/flash_uart --base <identity_base> --allow-protected` で予約領域へ書き込み可能
  - 実運用手順は `docs/NVM_IDENTITY_BLOCK_OPERATION.md` を参照
- 起動時判定:
  - `STM32F405` / `STM32F413` ともに `HAL_Init()` 直後に識別ブロックを読み、ペリフェラル初期化後かつ board依存初期化前に判定する
  - `NVM_STATUS_INTEGRITY_ERROR` / `NVM_STATUS_HW_ERROR` はセーフモード遷移（LED点滅ループ）
  - `NVM_STATUS_NOT_FOUND` / `NVM_STATUS_UNSUPPORTED` は起動継続（暫定運用）
- 検証ロジック:
  - 先頭16byteより短い `length` は checksum 計算前に `NVM_STATUS_INTEGRITY_ERROR` として棄却する
  - checksum 仕様は従来どおり、先頭16byte以降に対する 32bit additive checksum を維持する
- 予約領域:
  - `STM32F405`: 読み出し先を `0x08080000`（sector 8, 128KiB）として有効化済み
    - 既存の `distance_params` `0x080A0000`（sector 9）, `flash_params` `0x080C0000`（sector 10）, maze/eeprom `0x080E0000`（sector 11）とは非衝突
    - `internal flash` backend の `nvm_write()` / `nvm_erase()` は最小実装済み（eraseはarea単位、writeは境界チェックと `0->1` 禁止チェック付き）
  - `STM32F413`: 読み出し先を `0x08160000`（sector 15, 128KiB）として確定
    - `STM32F413CHU6` の内蔵Flashは `0x08000000`-`0x0817FFFF` の 1536KiB で、bank1 1MiB + bank2 512KiB の末尾 sector が `0x08160000` になる
    - 通常アプリ更新で消えない方針に合わせ、アプリ本体とは別に末尾 sector を予約し、標準書き込み手順ではこの sector を erase対象から外す前提とする
    - `tools/flashing/flash_uart` の `erase=app` では `STM32F413` 判定時に sector 12-15 を保護対象として除外する（maze/distance/flash_params/identityの暫定運用領域）
    - `NVM_AREA_IDENTITY` / `NVM_AREA_DISTANCE_PARAMS` / `NVM_AREA_FLASH_PARAMS` / `NVM_AREA_MAZE_MAP` の `nvm_write()` / `nvm_erase()` を最小実装済み（eraseはarea単位、writeは境界チェックと `0->1` 禁止チェック付き）

---

## 5. 符号化ルール（確定）

- `family`
  - `0`: unknown
  - `1`: mini
  - `2`: classic
- `board_id`
  - `family + board_id` の組で一意に扱う
  - 推奨形式は `0x00MMNNVV`
    - `MM`: `hw_rev_major`
    - `NN`: `hw_rev_minor`
    - `VV`: 同一 `major.minor` 内の基板バリアント番号（標準は `0x00`）
  - 例: `mini_r2_0` 標準基板は `0x00020000`
- `default_param_profile`
  - `0`: profile未指定（起動時に `family + hw_rev` の既定テーブルで決定）
  - `1..`: 運用側で定義したprofile IDを割り当てる

補足:

- 起動ログとCSVメタ情報では、`FW_TARGET`（MCU系列バイナリ名）と
  `machine_name`（`<family>_r<major>_<minor>`）を分けて出力する
