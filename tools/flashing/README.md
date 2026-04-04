# tools/flashing

書き込み・NVM初期化のホスト側ツールを配置します。

## 現在の内容

- `flash_uart`: STM32 UART bootloader 経由の書き込みツール

## 互換パス

既存運用向けに `tools/flash_uart` はこの実体へのラッパーとして維持しています。
新規運用では `tools/flashing/flash_uart` を利用してください。
