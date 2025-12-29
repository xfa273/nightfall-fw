#!/bin/bash

# STM32 Serial Monitor Script with proper line ending handling
# Usage: ./serial_monitor.sh [UART_PORT] [BAUD_RATE]

# デフォルト設定
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD="115200"

# パラメータ設定
UART_PORT=${1:-$DEFAULT_PORT}
BAUD_RATE=${2:-$DEFAULT_BAUD}

echo "=== STM32 Serial Monitor ==="
echo "Port: $UART_PORT"
echo "Baud Rate: $BAUD_RATE"
echo "Press ~. to exit"
echo "=========================="
echo ""

# UARTポートの存在確認
if [ ! -e "$UART_PORT" ]; then
    echo "Error: UART port not found: $UART_PORT"
    exit 1
fi

# 改行処理を適切に行うcuコマンド
# -l: ライン指定
# -s: ボーレート指定
# stty設定で適切な改行処理を追加
stty -F $UART_PORT $BAUD_RATE cs8 -cstopb -parenb raw
cu -l $UART_PORT -s $BAUD_RATE --parity=none
