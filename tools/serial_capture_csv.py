import argparse
import datetime
import glob
import os
import re
import select
import sys
import termios


_DEFAULT_SAVE_DIR = os.path.join(os.path.expanduser("~"), "STM32Workspace", "micromouse_log_visualizer", "logs")
_DEFAULT_UART_PORT = "/dev/ttyUSB0"
_DEFAULT_BAUD = 115200


_BAUD_MAP = {
    1200: termios.B1200,
    2400: termios.B2400,
    4800: termios.B4800,
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
    230400: getattr(termios, "B230400", None),
    460800: getattr(termios, "B460800", None),
    921600: getattr(termios, "B921600", None),
}


_RE_INT = re.compile(r"^[0-9]+$")
_RE_NUM = re.compile(r"^-?[0-9]+(\.[0-9]+)?$")


def _list_ports() -> list[str]:
    candidates = []
    candidates.extend(sorted(glob.glob("/dev/tty.*")))
    candidates.extend(sorted(glob.glob("/dev/cu.*")))
    return candidates


def _baud_to_termios(baud: int) -> int:
    b = _BAUD_MAP.get(baud)
    if b is None:
        raise ValueError(f"Unsupported baud rate: {baud}")
    return b


def _configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)

    speed = _baud_to_termios(baud)
    attrs[4] = speed
    attrs[5] = speed

    iflag = attrs[0]
    oflag = attrs[1]
    cflag = attrs[2]
    lflag = attrs[3]
    cc = attrs[6]

    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    oflag &= ~termios.OPOST
    lflag &= ~(termios.ECHO | termios.ECHONL | termios.ICANON | termios.ISIG | termios.IEXTEN)

    cflag |= termios.CLOCAL | termios.CREAD
    cflag &= ~termios.PARENB
    cflag &= ~termios.CSTOPB
    cflag &= ~termios.CSIZE
    cflag |= termios.CS8

    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 1

    attrs[0] = iflag
    attrs[1] = oflag
    attrs[2] = cflag
    attrs[3] = lflag
    attrs[6] = cc

    termios.tcsetattr(fd, termios.TCSANOW, attrs)


def _is_csv8_numeric(fields: list[str]) -> bool:
    if len(fields) != 8:
        return False
    if not _RE_INT.match(fields[0]):
        return False
    for x in fields[1:]:
        if not _RE_NUM.match(x):
            return False
    return True


def _new_capture_path(save_dir: str) -> str:
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    return os.path.join(save_dir, f"stm32_log_{ts}.csv")


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("save_dir", nargs="?", default=_DEFAULT_SAVE_DIR)
    ap.add_argument("uart_port", nargs="?", default=_DEFAULT_UART_PORT)
    ap.add_argument("baud", nargs="?", type=int, default=_DEFAULT_BAUD)
    ap.add_argument("--list", action="store_true")

    args = ap.parse_args()

    if args.list:
        for p in _list_ports():
            print(p)
        return 0

    save_dir = os.path.expanduser(args.save_dir)
    uart_port = args.uart_port
    baud = args.baud

    os.makedirs(save_dir, exist_ok=True)

    if not os.path.exists(uart_port):
        sys.stderr.write(f"[ERROR] UART port not found: {uart_port}\n")
        ports = _list_ports()
        if ports:
            sys.stderr.write("[INFO] Available ports:\n")
            for p in ports:
                sys.stderr.write(f"  {p}\n")
        return 1

    fd = os.open(uart_port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        _configure_serial(fd, baud)

        sys.stderr.write("=== Serial CSV Auto Capture ===\n")
        sys.stderr.write(f"Port       : {uart_port}\n")
        sys.stderr.write(f"Baud       : {baud}\n")
        sys.stderr.write(f"Save Dir   : {save_dir}\n")
        sys.stderr.write("\n")
        sys.stderr.write("このウィンドウを閉じるか、Ctrl+C で保存を終了します。\n")
        sys.stderr.write("CSV判定: 8列 (time_ms, param1..param7) 数値のみの行だけを保存します。\n")
        sys.stderr.write("ファイル分割: 最初のCSV行、または time_ms が前回より小さくなった時に新しいCSVファイルを自動作成します。\n")
        sys.stderr.write("================================\n")
        sys.stderr.flush()

        buf = b""
        out_fp = None
        out_path = ""
        prev_ts = None

        def ensure_new_file() -> None:
            nonlocal out_fp, out_path
            if out_fp is not None:
                out_fp.close()
                out_fp = None
            out_path = _new_capture_path(save_dir)
            sys.stderr.write(f"\n[INFO] New capture file: {out_path}\n")
            sys.stderr.flush()
            out_fp = open(out_path, "a", encoding="utf-8")

        while True:
            r, _, _ = select.select([fd], [], [], 0.5)
            if fd not in r:
                continue
            data = os.read(fd, 4096)
            if not data:
                continue
            buf += data
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                raw = buf[:nl]
                buf = buf[nl + 1 :]

                raw = raw.replace(b"\r", b"")
                try:
                    line = raw.decode("utf-8", errors="ignore").strip()
                except Exception:
                    continue
                if not line:
                    continue

                fields = [x.strip() for x in line.split(",")]
                if not _is_csv8_numeric(fields):
                    continue

                ts = int(fields[0])
                if out_fp is None or (prev_ts is not None and ts < prev_ts):
                    ensure_new_file()

                sys.stdout.write(line + "\n")
                sys.stdout.flush()

                out_fp.write(line + "\n")
                out_fp.flush()

                prev_ts = ts

    except KeyboardInterrupt:
        sys.stderr.write("\n[INFO] Capture finished.\n")
        return 0
    finally:
        try:
            os.close(fd)
        except Exception:
            pass
        try:
            if "out_fp" in locals() and out_fp is not None:
                out_fp.close()
        except Exception:
            pass

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
