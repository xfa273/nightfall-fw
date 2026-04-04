# tools/logging

ログ収集・変換・可視化のホスト側ツールを配置します。

## 現在の内容

- `serial_capture_csv.py`: シリアル受信をCSVへ保存
- `serial_capture_csv.sh`: シェル版CSVキャプチャ
- `serial_minicom.sh`: minicomベース受信
- `serial_monitor.sh`: シンプル受信
- `visualizer/`: 可視化ツール一式（`run_visualizer.sh`, `web_visualizer.py`）

## 互換パス

既存運用向けに以下はラッパーとして維持しています。

- `tools/serial_capture_csv.py`
- `tools/serial_capture_csv.sh`
- `tools/serial_minicom.sh`
- `tools/serial_monitor.sh`
- `tools/log_visualizer/*`

新規運用では `tools/logging/` 配下を利用してください。
