# tools/logging

ログ収集・変換・可視化のホスト側ツールを配置します。

## 現在の内容

- `serial_capture_csv.py`: シリアル受信をCSVへ保存
- `analyze_trace_csv.py`: trace CSVをflags位相で要約（idle/forward/coast/reverse/smoke）
- `analyze_turn_csv.py`: F413ターン調整用に角速度積分・最終角度誤差・オーバーシュート等を要約
- `analyze_front_match_csv.py`: 前壁位置合わせテストの状態遷移・距離誤差・指令追従を要約
- `export_plotjuggler_csv.py`: FRAM trace CSVをPlotJugglerで読みやすいCSVへ変換
- `run_plotjuggler.sh`: CSV変換後、固定テンプレート付きでPlotJugglerを起動
- `render_search_dump.py`: F413 UART `@` の `[SEARCH-DUMP]` をASCII迷路へ変換
- `serial_terminal.py`: ST-LINK VCPなどで使うシンプルな対話式UART端末
- `serial_capture_csv.sh`: シェル版CSVキャプチャ
- `serial_minicom.sh`: minicomベース受信
- `serial_monitor.sh`: シンプル受信
- `visualizer/`: 可視化ツール一式（`run_visualizer.sh`, `web_visualizer.py`）

## デフォルト保存先

- `serial_capture_csv.py` / `serial_capture_csv.sh` の既定保存先は `tools/logging/logs` です。
- `web_visualizer.py` の既定ログフォルダも `tools/logging/logs` です。
- `MICROMOUSE_LOG_DIR` 環境変数を設定すると、そのパスを優先します。

## `serial_capture_csv.py` のコマンド送信

- キャプチャ中に同じポートへUARTコマンドを送信できます（別シリアルモニタ不要）。
- 起動時に自動送信する場合: `python3 tools/logging/serial_capture_csv.py --send q,y,V`
- 起動後に手入力する場合: 実行中ターミナルで `q,y,V` と入力して Enter
- コマンドは半角ASCIIで入力してください（全角文字は送信せず警告表示します）。

## ST-LINK V3 MINIE VCPでのUART通信

F413の現行 `Debug-stm32f413` は `USART1` を `921600 8N1` で使います。ST-LINK V3 MINIEの `TX/RX/GND` を機体側UARTへ接続している場合、次でシリアル端末を開けます。

`NIGHTFALL_F413_UART_BAUD_RATE` をCMake cacheで変更した場合は、その値に合わせてください。

```bash
python3 tools/logging/serial_terminal.py
```

ポート候補を確認する場合:

```bash
python3 tools/logging/serial_terminal.py --list
```

ポートを明示する場合:

```bash
python3 tools/logging/serial_terminal.py --port /dev/cu.usbmodem112202
```

終了は `Ctrl-]` または `Ctrl-C` です。`tools/logging/serial_monitor.sh` と `tools/serial_monitor.sh` からも同じ端末を呼び出せます。

## `analyze_trace_csv.py` のディレクトリ指定

- `--expect x` / `--expect y` / `--expect z` / `--expect j` を付けると、ディレクトリ指定時は期待位相を満たす最新CSVを優先して選択します。
- `--expect z` は `search-safe` セッション（`motor_forward` + `motor_coast`）を検証します。
- `--expect j` は `shortest-safe` セッション（`motor_forward` + `motor_coast` + `motor_reverse`）を検証します。
- `z/j` が solver path + closed-loop session として動作した場合は、CSVの `flags` に bit12 (`0x1000`) が含まれます。
- `1`〜`5` の closed-loop 調整用テストも同じ `V` dump とCSV解析の流れで確認できます。
- 要約出力では `abort_count[...]`（bit8〜bit11）を表示し、停止理由フラグの有無を確認できます。
- `segments` の `d_enc=(L,R)` は 16bit符号付き差分で表示されるため、エンコーダwrap時の見かけ上の大値を抑制できます。
- 例:
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect y`
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect z`
  - `python3 tools/logging/analyze_trace_csv.py tools/logging/logs --expect j`

## `analyze_turn_csv.py` のターン評価

- `omega_z_mdps` を時間積分し、旋回角度の推定値を出します。
- 通常は `motor_reverse` 位相だけを評価します。全レコードを見る場合は `--all-records` を付けます。
- UART `3` / `4` の `#last_test_id` がCSVメタにある場合は目標角を自動推定します。
- 例:
  - `python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle -90`
  - `python3 tools/logging/analyze_turn_csv.py tools/logging/logs --target-angle 90 --tolerance 3`

## `analyze_front_match_csv.py` の前壁位置合わせ評価

- `mode1 case0 sub4` の専用テレメトリを抽出します。
- 各状態の継続時間、左右前センサの換算距離、位置・姿勢誤差、速度・角速度の指令値と実測値を表示します。
- `paused-wall-lost` / `paused-too-close` により安全停止の理由も確認できます。
- 例:
  - `python3 tools/logging/analyze_front_match_csv.py tools/logging/logs`
  - `python3 tools/logging/analyze_front_match_csv.py tools/logging/logs/trace_bin_YYYYMMDD_HHMMSS.csv`

## FRAM trace CSV表示（調整時の標準運用）

調整作業ではWeb版ビューアを使います。Web UI起動後は、左サイドバーのCSVメニューでログを選ぶだけで、FRAM v2ログ用の固定グラフが即時表示されます。

```bash
tools/logging/visualizer/run_visualizer.sh
```

表示対象:

- `*.plotjuggler.csv` は一覧から除外します。
- FRAM v2ログは `distance / velocity / motor / angle / omega / flags` の固定グラフで表示します。
- 各グラフには単位付きY軸、個別の時間軸、右側の系列名を表示します。
- `tune_ref` / `tune_error` などの派生列は内部で自動生成します。
- アップロードしたCSVは `tools/logging/logs/uploaded/` に保存されます。
- 旧8列CSVは従来通り、プリセットや列名指定で表示できます。

新しいログを取得した後は、Web UI左側の `Refresh log list` を押してからCSVを選びます。`Auto-refresh log list` を有効にすると、数秒ごとに一覧が更新されます。PlotJugglerのCSV読み込みポップアップは使わないため、調整作業中は毎回同じ操作で確認できます。

## PlotJugglerでのFRAM trace CSV表示（詳細確認用）

PlotJugglerで見る場合は `run_plotjuggler.sh` を使います。Nightfall標準テンプレート `tools/logging/plotjuggler/nightfall_f413_tune.xml` を指定して起動します。

macOSで未インストールの場合:

```bash
brew install --cask plotjuggler
```

最新ログを開く:

```bash
tools/logging/run_plotjuggler.sh
```

CSVを指定して開く:

```bash
tools/logging/run_plotjuggler.sh tools/logging/logs/stm32_log_20260510_153411.csv
```

ファイル選択ダイアログでCSVを選んで開く:

```bash
tools/logging/run_plotjuggler.sh --pick
```

内部では、元CSVを相対秒 `time` 列付きの `*.plotjuggler.csv` に変換してから、次の形で起動します。

```bash
plotjuggler --nosplash --datafile path/to/log.plotjuggler.csv --layout tools/logging/plotjuggler/nightfall_f413_tune.xml
```

別テンプレートを試す場合:

```bash
tools/logging/run_plotjuggler.sh --layout path/to/custom.xml tools/logging/logs/stm32_log_20260510_153411.csv
```

手動で読み込む場合:

1. PlotJugglerを起動する。
2. `python3 tools/logging/export_plotjuggler_csv.py tools/logging/logs` で変換後の `*.plotjuggler.csv` を作る。
3. `File` → `Load data...` で変換後の `*.plotjuggler.csv` を選ぶ。
4. CSV読み込みダイアログで区切り文字は `,`、時刻列は `time` を選ぶ。
5. `File` → `Load layout...` で `tools/logging/plotjuggler/nightfall_f413_tune.xml` を読む。
6. 左側の一覧から追加で見たい系列があれば右側へドラッグする。
7. 自分用に調整したら `File` → `Save layout...` でXMLレイアウトとして保存する。
8. 次回以降は先に同じ形式の `*.plotjuggler.csv` を読み込み、`File` → `Load layout...` で保存したXMLを読む。

まず見ると便利な系列:

- `distance_mm` と `tune_ref`: distance調整の目標と実測
- `target_velocity_mm_s` と `real_velocity_mm_s`: 速度追従
- `velocity_error_mm_s`: 速度誤差
- `motor_out_l` と `motor_out_r`: 左右モータ出力
- `angle_deg` と `target_angle_deg`: 角度追従
- `target_omega_dps` と `real_omega_dps`: 角速度追従
- `flag_motor_forward` / `flag_motor_coast` / `flag_motor_reverse`: 走行位相

調整用ログの場合、`tune_ref` と `tune_error` も追加されます。`reserved_i32_0` は生値のまま残し、`tune_ref` は `reserved_i32_0 / 1000` に変換した値です。

## `render_search_dump.py` の探索map表示

- F413 UART `@` の `[SEARCH-DUMP] y=..:` 行を含むログを、壁付きのASCII迷路へ変換します。
- 下位4bitを `W/S/E/N = 1/2/4/8` として解釈します。
- 通常チェックでは、F405互換の開始セル強制東壁だけは片側表現として許容します。
- 例:
  - `python3 tools/logging/serial_capture_csv.py --show-noncsv --send @ tools/logging/logs /dev/cu.usbmodem112202 921600 > /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py --summary-only /tmp/search_dump.log`
  - `python3 tools/logging/render_search_dump.py --summary-only --strict-consistency /tmp/search_dump.log`

## 互換パス

既存運用向けに以下はラッパーとして維持しています。

- `tools/serial_capture_csv.py`
- `tools/serial_capture_csv.sh`
- `tools/serial_minicom.sh`
- `tools/serial_monitor.sh`
- `tools/log_visualizer/*`

新規運用では `tools/logging/` 配下を利用してください。
