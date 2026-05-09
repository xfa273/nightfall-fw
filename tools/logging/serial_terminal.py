#!/usr/bin/env python3
import argparse
import glob
import os
import select
import sys
import termios
from pathlib import Path


PORT_PATTERNS_DARWIN = [
    "/dev/cu.usbmodem*",
    "/dev/cu.usbserial*",
    "/dev/cu.wchusbserial*",
    "/dev/tty.usbmodem*",
    "/dev/tty.usbserial*",
    "/dev/tty.wchusbserial*",
]

PORT_PATTERNS_OTHER = [
    "/dev/ttyACM*",
    "/dev/ttyUSB*",
]


def detect_port() -> str:
    patterns = PORT_PATTERNS_DARWIN if sys.platform == "darwin" else PORT_PATTERNS_OTHER
    candidates: list[str] = []
    for pat in patterns:
        candidates.extend(glob.glob(pat))
    candidates = sorted(set(candidates))

    cu_candidates = [p for p in candidates if "/cu." in p]
    if cu_candidates:
        candidates = cu_candidates

    if not candidates:
        raise RuntimeError("シリアルポートが見つかりません。--port で指定してください")
    if len(candidates) == 1:
        return candidates[0]
    raise RuntimeError("シリアルポート候補が複数あります。--port で指定してください: " + ", ".join(candidates))


def termios_baud(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise RuntimeError(f"この環境では未対応のボーレートです: {baud}")
    return getattr(termios, name)


def configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    iflag &= ~(termios.ICRNL | termios.INLCR | termios.IGNCR)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)

    cflag |= termios.CLOCAL | termios.CREAD
    cflag &= ~termios.PARENB
    cflag &= ~termios.CSTOPB
    cflag &= ~termios.CSIZE
    cflag |= termios.CS8

    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 1

    speed = termios_baud(baud)
    try:
        termios.cfsetispeed(attrs, speed)
        termios.cfsetospeed(attrs, speed)
    except AttributeError:
        attrs[4] = speed
        attrs[5] = speed

    attrs[0] = iflag
    attrs[1] = oflag
    attrs[2] = cflag
    attrs[3] = lflag
    attrs[6] = cc
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)


def stdin_raw_enter() -> list:
    attrs = termios.tcgetattr(sys.stdin.fileno())
    raw = attrs[:]
    raw[3] &= ~(termios.ICANON | termios.ECHO)
    raw[6][termios.VMIN] = 0
    raw[6][termios.VTIME] = 0
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, raw)
    return attrs


def stdin_restore(attrs: list) -> None:
    termios.tcsetattr(sys.stdin.fileno(), termios.TCSANOW, attrs)


def write_log(log_file, data: bytes) -> None:
    if log_file is not None:
        log_file.write(data)
        log_file.flush()


def main() -> int:
    ap = argparse.ArgumentParser(description="Simple serial terminal for Nightfall STM32 UART")
    ap.add_argument("--port", default=None, help="serial port, e.g. /dev/cu.usbmodem112202")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--send", default=None, help="send ASCII text at startup")
    ap.add_argument("--log", default=None, help="save received bytes to file")
    ap.add_argument("--list", action="store_true", help="list serial port candidates and exit")
    args = ap.parse_args()

    if args.list:
        patterns = PORT_PATTERNS_DARWIN if sys.platform == "darwin" else PORT_PATTERNS_OTHER
        found: list[str] = []
        for pat in patterns:
            found.extend(glob.glob(pat))
        for p in sorted(set(found)):
            print(p)
        return 0

    port = args.port or detect_port()
    log_file = None
    if args.log:
        log_path = Path(args.log).expanduser()
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_file = log_path.open("ab")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    old_stdin = None
    try:
        configure_serial(fd, args.baud)
        if args.send:
            os.write(fd, args.send.encode("ascii", errors="ignore"))

        print(f"=== Nightfall Serial Terminal ===")
        print(f"Port: {port}")
        print(f"Baud: {args.baud} 8N1")
        print("Exit: Ctrl-]  or Ctrl-C")
        print("=================================")

        old_stdin = stdin_raw_enter()
        while True:
            readable, _, _ = select.select([fd, sys.stdin.fileno()], [], [])
            if fd in readable:
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    data = b""
                if data:
                    os.write(sys.stdout.fileno(), data)
                    write_log(log_file, data)
            if sys.stdin.fileno() in readable:
                data = os.read(sys.stdin.fileno(), 1024)
                if data:
                    if b"\x1d" in data:
                        break
                    os.write(fd, data)
    finally:
        if old_stdin is not None:
            stdin_restore(old_stdin)
        if log_file is not None:
            log_file.close()
        os.close(fd)
        print("\nclosed")

    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("\nclosed")
        raise SystemExit(130)
    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        raise SystemExit(1)
