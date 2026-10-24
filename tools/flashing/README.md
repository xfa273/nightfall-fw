# tools/flashing

書き込み・NVM初期化のホスト側ツールを配置します。

## 現在の内容

- `flash_uart`: STM32 UART bootloader 経由の書き込みツール
- `flash_stlink`: ST-LINK SWD 経由のSTM32F413書き込みツール
- `make_identity_block.py`: 機体識別ブロック（`nvm_identity_block_t`）のバイナリ生成ツール

`flash_uart --erase app` の保護セクタ:

- STM32F405: sector 8/9/10/11（identity/distance/flash_params/maze）
- STM32F413: sector 12/13/14/15（旧FRAM無し暫定運用のmaze/distance/flash_paramsとidentity領域を保護）

現行F413ファームでは、identityは内蔵Flash sector 15、distance/sensor/maze/traceは外付けFRAM backendで運用します。保護セクタ設定は、identity保護と旧暫定運用データの誤消去防止のため維持しています。

## USB-UART経由のF413書き込み（mini_r2 / mini_r3）

機体をUARTブートローダーモード（BOOT0をHighにしてリセット）へ入れてから実行します。
ポート一覧は `python3 tools/logging/serial_terminal.py --list` で確認できます。
2026-10-10のmini_r3接続ポートは `/dev/cu.usbserial-55670290851` です。
同じポートを開いているログキャプチャ・シリアル端末は閉じてください。

リポジトリルートでのビルドと書き込み:

```bash
cmake --build --preset Debug-stm32f413
python3 tools/flashing/flash_uart \
  --port /dev/cu.usbserial-55670290851 \
  --bin build/Debug/nightfall_stm32f413.bin \
  --erase app \
  --timeout 45
```

初回で `build/Debug` がまだ無い場合は、先に `cmake --preset Debug` を実行します。
ポート名は接続環境に合わせて変更してください。`--bin` は必ずF413の生成物を指定します
（省略時の自動選択はF405イメージを優先するため）。

ツールが書き込み用の115200bps・8E1を設定し、アプリが使うsectorだけを消去します。
F413の保護sector12〜15は通常操作では消去せず、FRAMの校正・迷路・ログにも触れません。
`--erase all` や `--allow-protected` は通常のFW更新には使用しません。
`--timeout 45` はブートローダー接続待ち時間で、全書き込みの制限時間ではありません。

通常は転送後にGOコマンドを送ります。ただし2026-10-10のmini_r3ではGOのACK後に
正常なアプリ応答を確認できなかったため、書き込み後はBOOT0を通常のLowへ戻して
リセットしてください。アプリのログ通信は921600bps・8N1で、書き込み用の設定とは異なります。
`--no-go` を付けた場合はブートローダーに留まります。

`flash_uart` の `SUCCESS` は各転送のACK確認までで、読み戻し比較は行いません。
2026-10-10の実機作業では `--no-go` で転送後、別途CubeProgrammerのUART読み取りで
アプリ全体の完全一致と機体ID領域の保持を確認しています。起動確認も含む実機結果は
`docs/ai/WORKLOG.md` に記録します。

## ST-LINK V3 MINIE 経由のF413書き込み

CubeProgrammerのCLI (`STM32_Programmer_CLI`) を使い、SWD経由で `build/Debug/nightfall_stm32f413.elf` を書き込みます。

### Apple Silicon Mac

Apple Siliconではarm64対応CLIを使用します。CubeProgrammer 2.21のように
`bin/STM32_Programmer_CLI`がx86_64専用でも、同じインストールの
`api/lib/STM32_Programmer_CLI`がarm64対応なら自動選択します。
`--cli`または`STM32_PROGRAMMER_CLI`で既存のbin CLIを指定した場合も同様です。
既にarm64対応のbin CLIや独自ラッパーを指定している場合はそのまま使用します。

SDK側CLIは実行位置からデバイスDB・FlashLoaderを探すため、その実行ファイルだけを
private一時ディレクトリにコピーし、同梱ライブラリ・DB・Loader・Driversをリンクします。
終了時に一時ディレクトリを削除します。インストール済みアプリの変更、Rosetta、
追加ダウンロード、署名やセキュリティ設定の変更は不要です。
arm64版が同梱されていなければ書き込み前にエラーにし、対応版の指定を案内します。

ホスト側の回帰テスト（実機への接続・書き込みなし）:

```bash
python3 -m unittest discover -s tools/flashing -p 'test_cube_cli_runtime.py' -v
```

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
