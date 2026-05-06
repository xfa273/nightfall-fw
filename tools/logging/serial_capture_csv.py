#!/usr/bin/env python3
import argparse
import glob
import os
import re
import select
import sys
import termios
import time
from datetime import datetime
from pathlib import Path
from typing import Optional


_NUM_RE = re.compile(r"^-?[0-9]+(\.[0-9]+)?$")


def _repo_root_from_this_file() -> Path:
    return Path(__file__).resolve().parents[2]


def _default_save_dir() -> Path:
    env = os.environ.get("MICROMOUSE_LOG_DIR")
    if env:
        return Path(env).expanduser()

    return _repo_root_from_this_file() / "tools/logging/logs"


def _is_num(s: str) -> bool:
    return bool(_NUM_RE.match(s))


def _detect_port() -> str:
    if sys.platform == "darwin":
        patterns = [
            "/dev/cu.usbmodem*",
            "/dev/cu.usbserial*",
            "/dev/cu.wchusbserial*",
            "/dev/tty.usbmodem*",
            "/dev/tty.usbserial*",
            "/dev/tty.wchusbserial*",
        ]
    else:
        patterns = ["/dev/ttyACM*", "/dev/ttyUSB*"]

    for pat in patterns:
        for p in sorted(glob.glob(pat)):
            if os.path.exists(p):
                return p
    return ""


def _termios_baud(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"Unsupported baud rate on this platform: {baud}")
    return getattr(termios, name)


def _configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)

    iflag, oflag, cflag, lflag, ispeed, ospeed, cc = attrs

    cflag |= termios.CLOCAL | termios.CREAD
    cflag &= ~termios.PARENB
    cflag &= ~termios.CSTOPB
    cflag &= ~termios.CSIZE
    cflag |= termios.CS8

    iflag &= ~(termios.IXON | termios.IXOFF | termios.IXANY)
    iflag &= ~(termios.ICRNL | termios.INLCR | termios.IGNCR)

    oflag &= ~termios.OPOST

    lflag &= ~(termios.ICANON | termios.ECHO | termios.ECHOE | termios.ISIG)

    cc[termios.VMIN] = 1
    cc[termios.VTIME] = 0

    speed = _termios_baud(baud)
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


def _new_output_file(save_dir: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    return save_dir / f"stm32_log_{ts}.csv"


def _format_mm_columns(line: str) -> str:
    s = line.strip()
    if s.startswith("#mm_columns="):
        s = s[len("#mm_columns=") :]
    cols = [c.strip() for c in s.split(",") if c.strip()]
    if not cols:
        return ""
    return ",".join(cols)


def _print_mm_columns(line: str, out_path: Optional[Path]) -> None:
    cols = _format_mm_columns(line)
    if not cols:
        return
    if out_path is not None:
        print(f"[INFO] Columns ({out_path.name}): {cols}", file=sys.stderr)
    else:
        print(f"[INFO] Columns: {cols}", file=sys.stderr)


def _parse_command_text(text: str) -> list[str]:
    out: list[str] = []
    for token in re.split(r"[\s,]+", text.strip()):
        if not token:
            continue
        for ch in token:
            out.append(ch)
    return out


def _send_command_chars(fd: int, commands: list[str], delay_ms: float) -> None:
    if not commands:
        return
    delay_sec = max(0.0, delay_ms / 1000.0)
    sent_count = 0
    for i, cmd in enumerate(commands):
        b = cmd.encode("ascii", errors="ignore")
        if len(b) != 1:
            print(f"[WARN] Skipped non-ASCII cmd: {cmd}", file=sys.stderr)
            continue
        try:
            os.write(fd, b)
        except OSError as e:
            print(f"[ERROR] Failed to send cmd '{cmd}': {e}", file=sys.stderr)
            return
        sent_count += 1
        print(f"[INFO] Sent cmd: {cmd}", file=sys.stderr)
        if i + 1 < len(commands) and delay_sec > 0.0:
            time.sleep(delay_sec)
    if sent_count == 0 and commands:
        print("[WARN] No valid ASCII command was sent", file=sys.stderr)


def main() -> int:
    ap = argparse.ArgumentParser(add_help=True)
    ap.add_argument("save_dir", nargs="?", default=None)
    ap.add_argument("port", nargs="?", default="auto")
    ap.add_argument("baud", nargs="?", type=int, default=115200)
    ap.add_argument("--show-noncsv", action="store_true")
    ap.add_argument(
        "--send",
        default="",
        help="Send command sequence after open (e.g. 'q,y,V' or 'qyV')",
    )
    ap.add_argument(
        "--send-interval-ms",
        type=float,
        default=300.0,
        help="Interval between auto-sent command chars",
    )
    ap.add_argument(
        "--interactive-send-interval-ms",
        type=float,
        default=50.0,
        help="Interval between chars for interactive stdin command send",
    )
    args = ap.parse_args()

    save_dir = _default_save_dir() if args.save_dir is None else Path(args.save_dir).expanduser()
    save_dir.mkdir(parents=True, exist_ok=True)

    port = args.port
    if port in ("", "auto"):
        port = _detect_port()

    if not port:
        print("[ERROR] UART port not found.", file=sys.stderr)
        if sys.platform == "darwin":
            print("[HINT] Candidates: ls /dev/cu.usbmodem* /dev/cu.usbserial*", file=sys.stderr)
        else:
            print("[HINT] Candidates: ls /dev/ttyACM* /dev/ttyUSB*", file=sys.stderr)
        return 2

    if not os.path.exists(port):
        print(f"[ERROR] UART port not found: {port}", file=sys.stderr)
        return 2

    print("=== Serial CSV Auto Capture (Python) ===")
    print(f"Port       : {port}")
    print(f"Baud       : {args.baud}")
    print(f"Save Dir   : {save_dir}")
    if sys.stdin.isatty():
        print("Type UART commands and press Enter (example: q,y,V)")
    print("Press Ctrl+C to stop.")

    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        _configure_serial(fd, args.baud)

        buf = b""
        out_path: Optional[Path] = None
        prev_ts: Optional[int] = None
        pending_columns: Optional[str] = None
        wrote_columns: bool = False
        pending_fw_meta: list[str] = []
        fw_meta_written_count: int = 0
        last_printed_columns: Optional[str] = None
        last_printed_file: Optional[Path] = None
        stdin_fd: Optional[int] = None
        stdin_pending = ""
        if sys.stdin.isatty():
            try:
                stdin_fd = sys.stdin.fileno()
            except OSError:
                stdin_fd = None

        auto_commands = _parse_command_text(args.send)
        if auto_commands:
            _send_command_chars(fd, auto_commands, args.send_interval_ms)

        while True:
            read_list = [fd]
            if stdin_fd is not None:
                read_list.append(stdin_fd)
            r, _, _ = select.select(read_list, [], [], 0.25)
            if not r:
                continue

            if stdin_fd is not None and stdin_fd in r:
                try:
                    stdin_chunk = os.read(stdin_fd, 1024)
                except BlockingIOError:
                    stdin_chunk = b""

                if stdin_chunk == b"":
                    stdin_fd = None
                else:
                    stdin_pending += stdin_chunk.decode("utf-8", errors="ignore")
                    stdin_pending = stdin_pending.replace("\r", "\n")
                    while "\n" in stdin_pending:
                        line, stdin_pending = stdin_pending.split("\n", 1)
                        commands = _parse_command_text(line)
                        _send_command_chars(fd, commands, args.interactive_send_interval_ms)

            if fd not in r:
                continue

            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError as e:
                print(f"\n[INFO] Serial read stopped: {e}", file=sys.stderr)
                return 0

            if not chunk:
                continue

            buf += chunk

            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break
                line_b = buf[:nl]
                buf = buf[nl + 1 :]

                line_b = line_b.replace(b"\r", b"")
                line = line_b.decode("ascii", errors="ignore").strip()
                if not line:
                    continue

                if line.startswith("#mm_columns="):
                    pending_columns = line
                    if out_path is None:
                        if pending_columns != last_printed_columns:
                            _print_mm_columns(pending_columns, None)
                            last_printed_columns = pending_columns
                            last_printed_file = None
                    else:
                        if pending_columns != last_printed_columns or out_path != last_printed_file:
                            _print_mm_columns(pending_columns, out_path)
                            last_printed_columns = pending_columns
                            last_printed_file = out_path
                    if out_path is not None and not wrote_columns:
                        with out_path.open("a", encoding="ascii", newline="\n") as f:
                            f.write(pending_columns + "\n")
                        wrote_columns = True
                    continue

                if line.startswith("#fw_"):
                    if line not in pending_fw_meta:
                        pending_fw_meta.append(line)
                        print(f"[INFO] FW Meta: {line}", file=sys.stderr)
                    if out_path is not None and fw_meta_written_count < len(pending_fw_meta):
                        with out_path.open("a", encoding="ascii", newline="\n") as f:
                            for m in pending_fw_meta[fw_meta_written_count:]:
                                f.write(m + "\n")
                        fw_meta_written_count = len(pending_fw_meta)
                    continue

                parts = [p.strip() for p in line.split(",")]
                if len(parts) == 8 and parts[0].isdigit() and all(_is_num(p) for p in parts[1:]):
                    ts = int(parts[0])
                    if out_path is None or (prev_ts is not None and ts < prev_ts):
                        out_path = _new_output_file(save_dir)
                        print(f"\n[INFO] New capture file: {out_path}", file=sys.stderr)
                        wrote_columns = False
                        fw_meta_written_count = 0
                        if pending_fw_meta:
                            with out_path.open("a", encoding="ascii", newline="\n") as f:
                                for m in pending_fw_meta:
                                    f.write(m + "\n")
                            fw_meta_written_count = len(pending_fw_meta)
                        if pending_columns is not None:
                            _print_mm_columns(pending_columns, out_path)
                            last_printed_columns = pending_columns
                            last_printed_file = out_path
                            with out_path.open("a", encoding="ascii", newline="\n") as f:
                                f.write(pending_columns + "\n")
                            wrote_columns = True

                    print(",".join(parts))
                    with out_path.open("a", encoding="ascii", newline="\n") as f:
                        if pending_fw_meta and fw_meta_written_count < len(pending_fw_meta):
                            for m in pending_fw_meta[fw_meta_written_count:]:
                                f.write(m + "\n")
                            fw_meta_written_count = len(pending_fw_meta)
                        if pending_columns is not None and not wrote_columns:
                            f.write(pending_columns + "\n")
                            wrote_columns = True
                        f.write(",".join(parts) + "\n")

                    prev_ts = ts
                else:
                    if args.show_noncsv:
                        print(line, file=sys.stderr)

    except KeyboardInterrupt:
        print(f"\n[INFO] Capture finished. Saved files under: {save_dir}")
        return 0
    finally:
        os.close(fd)


if __name__ == "__main__":
    raise SystemExit(main())
