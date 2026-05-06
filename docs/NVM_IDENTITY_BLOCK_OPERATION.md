# NVM機体識別ブロック 運用手順

この手順は、`tools/flashing/make_identity_block.py` と `tools/flashing/flash_uart` を使って、
機体識別ブロックを予約領域へ書き込むための実運用手順です。

`STM32F413` の FRAM無し暫定運用全体の実機確認は
`docs/F413_INTERNAL_FLASH_TEMP_VERIFICATION_CHECKLIST.md` を参照してください。

---

## 1. 前提

- リポジトリルート: `nightfall-fw/`
- Python 3 が使える
- ST-Link V3 と UART bootloader 書き込み経路が使える
- 書き込み対象MCU:
  - `STM32F405`: 識別ブロック予約領域は `0x08080000`（sector 8）
  - `STM32F413`: 識別ブロック予約領域は `0x08160000`（sector 15）

命名と識別子の扱い（運用決定）:

- `FW_TARGET`（MCU系列バイナリ名）: 例 `nightfall_stm32f413`
- `machine_name`（機体系識別名）: `<family>_r<major>_<minor>`（例 `mini_r2_0`）
- `machine_unit`（個体識別名）: `<machine_name>_unitNNN`（例 `mini_r2_0_unit001`）
- `board_id` は推奨 `0x00MMNNVV`（`MM=major`, `NN=minor`, `VV=variant`）
- 一意性は `family + board_id` の組で扱う

注意:

- 識別領域は保護領域です。書き込み時は `flash_uart` に `--allow-protected` を必ず付けます。
- `--erase all` は使わないでください（不要領域まで消去するリスクがあります）。
- `STM32F413` は暫定運用として sector 12-15（maze/distance/flash_params/identity）をNVM予約領域として保護しています。

---

## 2. 識別ブロックbinを生成

例（mini_r2_0_unit001）:

```bash
python3 tools/flashing/make_identity_block.py \
  --out build/identity/mini_r2_0_unit001.bin \
  --family mini \
  --board-id 0x00020000 \
  --hw-rev-major 2 \
  --hw-rev-minor 0 \
  --unit-serial 1 \
  --default-param-profile 0 \
  --capability-flags 0x00000000 \
  --uid0 0x00000000 \
  --uid1 0x00000000 \
  --uid2 0x00000000
```

補足:

- `uid0/1/2` は分からない場合 `0` で生成可能です。
- 書き込み後に診断モードで UID を再反映できます（後述）。

---

## 3. 識別領域へ書き込み

### 3.1 STM32F405

```bash
python3 tools/flashing/flash_uart \
  --bin build/identity/mini_r2_0_unit001.bin \
  --base 0x08080000 \
  --allow-protected
```

### 3.2 STM32F413

```bash
python3 tools/flashing/flash_uart \
  --bin build/identity/mini_r2_0_unit001.bin \
  --base 0x08160000 \
  --allow-protected
```

---

## 4. 書き込み後の確認

1. 電源再投入（またはリセット）
2. 起動ログを確認

期待ログ例:

- 正常時:
  - `[Boot] ID family=mini(1) board=0x00020000 rev=2.0 unit=1 cap=...`
  - `[Boot] ID machine=mini_r2_0 unit_name=mini_r2_0_unit001`
- 異常時:
  - `[SAFE] identity invalid status=... uid=...`（セーフモード遷移）

---

## 5. 診断モードでの再書き込み（CRC再構築）

`F405` の `Mode 9` に、`nvm_identity_write()` を使った再書き込み入口があります。

- sub `0`: 既存の sensor 再校正保存
- sub `1`: 現在の識別ブロックを読んで、ヘッダ/CRCを再構築して再書き込み

この操作は、ヘッダ不整合やCRC不整合の復旧用途を想定しています。

---

## 6. トラブル時の確認項目

- `flash_uart` 実行時に `--allow-protected` を付け忘れていないか
- `--base` がMCUに対応した識別予約領域か
- ビルドターゲットと実機MCU系列が一致しているか
- 起動ログが `ID status=...` で継続になっていないか
