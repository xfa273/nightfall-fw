#!/bin/bash

# STM32 Serial Monitor using minicom (better line ending handling)
# Usage: ./serial_minicom.sh [UART_PORT] [BAUD_RATE]

# デフォルト設定
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"

# パラメータ設定
UART_PORT=${1:-$DEFAULT_PORT}
BAUD_RATE=${2:-$DEFAULT_BAUD}

echo "=== STM32 Serial Monitor (minicom) ==="
echo "Port: $UART_PORT"
echo "Baud Rate: $BAUD_RATE"
echo "Exit: Ctrl+A then X"
echo "====================================="
echo ""

# UARTポートの存在確認
if [ ! -e "$UART_PORT" ]; then
    echo "Error: UART port not found: $UART_PORT"
    exit 1
fi

# minicomでシリアル監視開始
# -D: ポート指定
# -b: ボーレート指定
# -8: 8bit データビット
# -o: ローカルエコーOFF
minicom -D $UART_PORT -b $BAUD_RATE -8 -o
