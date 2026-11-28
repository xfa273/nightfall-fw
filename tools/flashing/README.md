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

macOSでV3 MINIEのUSB通信が停止した場合、一覧表示・書き込み・ターゲットリセットは
エラーで終了します。USBを再列挙するには、機体電源とプローブのターゲット給電を
OFFにし、開いているVCP端末を閉じてから、次の復旧操作を単独で実行します。
mini R3では通電したままUSBを再接続した際に車輪が回り、迷路が更新された記録があります。

```sh
python3 tools/flashing/flash_stlink --recover-usb --sn <STLINK_SN>
```

復旧はUSB通信エラーがあるときに対象V3 MINIEを1回だけリセットし、同じシリアル番号が
再列挙されるか最大3回確認して終了します。複数台接続時は `--sn` で対象を指定します。

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

既定値は `mode=NORMAL`, `freq=1000` (1 MHz), `reset=SWrst` です。
接続余裕を増やすため、SWDの既定値を従来の4 MHzから下げています。
速度を戻す場合は `--freq 4000` を指定できます。
`.bin` を指定した場合は、既定で `0x08000000` に書き込みます。

### `Error: failed to erase memory` が繰り返す場合

この表示だけでは、USB通信、SWD信号、機体電源、Flash保護などのどれが
原因かは判定できません。USB抜き差しで復帰するなら通信状態を優先して切り分けます。
書き込みと `--reset-only` は毎回、CubeProgrammerの `-log` による詳細ログと
実行コマンド・終了状態を `build/flashing_logs/stlink_*.log` に保存し、パスを表示します。
エラー終了、またはログ内の `Error:` を検出した場合は失敗扱いにし、消去や書き込みを
自動反復しません。CLIが起動できない場合のログはラッパー側の記録だけになります。

1. まず通常の `python3 tools/flashing/flash_stlink --build` で1 MHzを試す。
   まだ不安定なら `--freq 400` へ下げ、ログに出る実際のSWD周波数・電圧を確認する。
2. ST-LINKのNRSTが機体のNRSTへ接続されている場合は、実行中のFWから制御を
   取り戻すため次の接続を試す（NRST未接続では使えません）。

   ```bash
   python3 tools/flashing/flash_stlink --build --freq 400 --mode UR --reset-mode HWrst
   ```

3. `DEV_USB_COMM_ERR`、USB timeout、SN読取り失敗を伴う場合は、IDE/GDB等の
   別デバッガ接続を終了し、USBハブを外した直結・別ケーブル・別ポートで比較する。
   UART capture使用中のUSB抜き差しはVCPも切断するため、再接続後にcaptureを再開する。
   配線を変える前に機体電源とプローブのターゲット給電をOFFにする。`--recover-usb` はV3 MINIE用。
4. 同じセクタで再現する場合は、失敗前のST-LINK FW、ターゲット電圧、SWD周波数、
   sector番号とエラーをログで確認する。VDD/GND/SWDIO/SWCLK/NRSTの接触・配線も確認し、
   ST-LINK firmwareは公式更新ツールで更新を検討する。保護解除や全消去を復旧手順にしない。

機体IDや校正を消す `-e all`、RDP解除、option byte変更は追加しません。
2026-09-27の実機調査では、V2-1で1 MHz/240 kHz、UR接続、USB抜き差しでも
消去失敗が継続し、別ツールのOpenOCDでも失敗しました。CPU停止中の同一RAMの
繰り返し読み出しも不一致となり、50 kHzでも改善しませんでした。
周波数変更だけで直ったとは判断せず、通信経路・プローブ・機体側を切り分けます。
別実装の`st-flash`でも内容が変わらない内蔵ROMの読み出しが不一致となりました。
この状態では消去を反復せず、ケーブルやプローブを交換して読み出しの安定性から確認します。
同日、WeAct Mini Debuggerの別個体・別ハブ・別A–Cケーブルでも再現しましたが、
STLINK-V3MINIEと専用ケーブルへの交換で消去・書き込み・照合・起動に成功しました。
アプリ全域の読み戻し一致とidentity保全も確認済みです。WeAct用の共通変換基板・
SWDケーブル、またはそのプローブ系統との組み合わせが未切り分けなので、
当面は復旧確認済みのV3MINIE経路を使用します。確認済みの接続条件:

```bash
python3 tools/flashing/flash_stlink --build --freq 1000 --mode UR --reset-mode HWrst
```

このUR/HWrst条件はNRST接続済みの構成で確認しています。
調査経過は [実機調査記録](../../docs/ai/STLINK_ERASE_FAILURE_20260927.md) を参照してください。

参考: [CubeProgrammer UM2237（接続/リセット/ログ）](https://www.st.com/resource/en/user_manual/dm00403500-stm32cubeprogrammer-stmicroelectronics.pdf)、
[ST-LINK RN0093（既知制約・FW更新）](https://www.st.com/resource/en/release_note/dm00107009-firmware-upgrade-for-stlink-stlinkv2-stlinkv21-and-stlinkv3-boards-stmicroelectronics.pdf)。

書き込みの回帰テスト（プローブ接続なし）:

```bash
python3 -m unittest discover -s tools/flashing -p 'test_*.py' -v
```

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
