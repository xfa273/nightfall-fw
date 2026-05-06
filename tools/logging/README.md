# tools/logging

ログ収集・変換・可視化のホスト側ツールを配置します。

## 現在の内容

- `serial_capture_csv.py`: シリアル受信をCSVへ保存
- `analyze_trace_csv.py`: trace CSVをflags位相で要約（idle/forward/coast/reverse/smoke）
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

## 互換パス

既存運用向けに以下はラッパーとして維持しています。

- `tools/serial_capture_csv.py`
- `tools/serial_capture_csv.sh`
- `tools/serial_minicom.sh`
- `tools/serial_monitor.sh`
- `tools/log_visualizer/*`

新規運用では `tools/logging/` 配下を利用してください。
