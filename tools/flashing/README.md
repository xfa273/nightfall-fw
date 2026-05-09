# tools/flashing

書き込み・NVM初期化のホスト側ツールを配置します。

## 現在の内容

- `flash_uart`: STM32 UART bootloader 経由の書き込みツール
- `flash_stlink`: ST-LINK SWD 経由のSTM32F413書き込みツール
- `make_identity_block.py`: 機体識別ブロック（`nvm_identity_block_t`）のバイナリ生成ツール

`flash_uart --erase app` の保護セクタ:

- STM32F405: sector 8/9/10/11（identity/distance/flash_params/maze）
- STM32F413: sector 12/13/14/15（maze/distance/flash_params/identity, 暫定内蔵Flash運用）

## ST-LINK V3 MINIE 経由のF413書き込み

CubeProgrammerのCLI (`STM32_Programmer_CLI`) を使い、SWD経由で `build/Debug/nightfall_stm32f413.elf` を書き込みます。

### ST-LINK認識確認

```bash
python3 tools/flashing/flash_stlink --list
```

### ビルド済みF413ファームを書き込む

```bash
python3 tools/flashing/flash_stlink
```

### ビルドしてから書き込む

```bash
python3 tools/flashing/flash_stlink --build
```

### 複数ST-LINK接続時にシリアル番号を指定する

```bash
python3 tools/flashing/flash_stlink --sn 003B00273234511537333934
```

既定値は `mode=NORMAL`, `freq=4000`, `reset=SWrst` です。`.bin` を指定した場合は、既定で `0x08000000` に書き込みます。

### 例: 識別ブロック生成

```bash
python3 tools/flashing/make_identity_block.py \
  --out build/identity/classic_r1_0_unit42.bin \
  --family classic \
  --board-id 0x00010000 \
  --hw-rev-major 1 \
  --hw-rev-minor 0 \
  --unit-serial 42 \
  --default-param-profile 0 \
  --capability-flags 0x00000000 \
  --uid0 0x00000000 \
  --uid1 0x00000000 \
  --uid2 0x00000000
```

### 例: 生成した識別ブロックを書き込む

- STM32F405 (`0x08080000`, sector 8)

```bash
python3 tools/flashing/flash_uart --bin build/identity/classic_r1_0_unit42.bin --base 0x08080000 --allow-protected
```

- STM32F413 (`0x08160000`, sector 15)

```bash
python3 tools/flashing/flash_uart --bin build/identity/classic_r1_0_unit42.bin --base 0x08160000 --allow-protected
```

## 互換パス

既存運用向けに `tools/flash_uart` はこの実体へのラッパーとして維持しています。
ST-LINK書き込み向けに `tools/flash_stlink` もラッパーとして用意しています。
新規運用では `tools/flashing/flash_uart` を利用してください。

識別ブロックの実運用手順は `docs/NVM_IDENTITY_BLOCK_OPERATION.md` を参照してください。
`STM32F413` の FRAM無し暫定運用確認は `docs/F413_INTERNAL_FLASH_TEMP_VERIFICATION_CHECKLIST.md` を参照してください。
