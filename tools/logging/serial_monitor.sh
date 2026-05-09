#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${1:-}" = "--help" ] || [ "${1:-}" = "-h" ]; then
    exec python3 "$SCRIPT_DIR/serial_terminal.py" --help
fi

if [ "$#" -ge 1 ] && [[ "${1:-}" != --* ]]; then
    PORT="$1"
    shift
    if [ "$#" -ge 1 ] && [[ "${1:-}" != --* ]]; then
        BAUD="$1"
        shift
    else
        BAUD="115200"
    fi
    exec python3 "$SCRIPT_DIR/serial_terminal.py" --port "$PORT" --baud "$BAUD" "$@"
fi

exec python3 "$SCRIPT_DIR/serial_terminal.py" "$@"
