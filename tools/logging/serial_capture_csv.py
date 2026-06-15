#!/usr/bin/env python3
import argparse
import fcntl
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

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import trace_bin_dump


_NUM_RE = re.compile(r"^-?[0-9]+(\.[0-9]+)?$")
DEFAULT_BAUD = int(os.environ.get("NIGHTFALL_UART_BAUD", "921600"))
IOSSIOSPEED = 0x80045402


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

    try:
        speed = _termios_baud(baud)
    except ValueError:
        speed = _termios_baud(9600)
        custom_baud = True
    else:
        custom_baud = False
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
    if custom_baud:
        fcntl.ioctl(fd, IOSSIOSPEED, int(baud).to_bytes(4, "little"))


def _new_output_file(save_dir: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = save_dir / f"stm32_log_{ts}.csv"
    if not base.exists():
        return base
    for i in range(1, 1000):
        candidate = save_dir / f"stm32_log_{ts}_{i:03d}.csv"
        if not candidate.exists():
            return candidate
    return save_dir / f"stm32_log_{ts}_{time.monotonic_ns()}.csv"


def _new_binary_raw_file(save_dir: Path) -> Path:
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    base = save_dir / f"trace_bin_{ts}.raw"
    if not base.exists():
        return base
    for i in range(1, 1000):
        candidate = save_dir / f"trace_bin_{ts}_{i:03d}.raw"
        if not candidate.exists():
            return candidate
    return save_dir / f"trace_bin_{ts}_{time.monotonic_ns()}.raw"


def _binary_frame_total_len(buf: bytes) -> Optional[int]:
    if len(buf) < trace_bin_dump.FRAME_STRUCT.size:
        return None
    fields = trace_bin_dump.FRAME_STRUCT.unpack_from(buf, 0)
    version = fields[1]
    header_size = fields[3]
    record_size = fields[4]
    record_count = fields[5]
    if version != 1 or header_size != trace_bin_dump.HEADER_STRUCT.size or trace_bin_dump.record_struct_for_size(record_size) is None:
        return -1
    total_len = trace_bin_dump.FRAME_STRUCT.size + header_size + record_size * record_count
    if len(buf) < total_len:
        return None
    payload = buf[trace_bin_dump.FRAME_STRUCT.size : total_len]
    if trace_bin_dump.checksum(payload) != fields[7]:
        return -1
    return total_len


def _save_binary_frame(save_dir: Path, frame_bytes: bytes) -> None:
    raw_path = _new_binary_raw_file(save_dir)
    raw_path.write_bytes(frame_bytes)
    frame, header, rows, offset = trace_bin_dump.extract_frame(frame_bytes)
    csv_path = raw_path.with_suffix(".csv")
    trace_bin_dump.write_csv(csv_path, frame, header, rows)
    print(
        f"\n[INFO] Binary trace frame: raw={raw_path} csv={csv_path} "
        f"records={len(rows)} offset={offset}",
        file=sys.stderr,
    )


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
    ap.add_argument("baud", nargs="?", type=int, default=DEFAULT_BAUD)
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
        expected_csv_columns = 8
        current_columns: list[str] = []
        seq_column_index: Optional[int] = None
        prev_seq: Optional[int] = None
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
                magic_idx = buf.find(trace_bin_dump.MAGIC)
                if magic_idx == 0:
                    total_len = _binary_frame_total_len(buf)
                    if total_len is None:
                        break
                    if total_len < 0:
                        buf = buf[1:]
                        continue
                    frame_bytes = buf[:total_len]
                    buf = buf[total_len:]
                    try:
                        _save_binary_frame(save_dir, frame_bytes)
                    except Exception as e:
                        print(f"[WARN] Failed to decode binary trace frame: {e}", file=sys.stderr)
                    out_path = None
                    pending_columns = None
                    current_columns = []
                    expected_csv_columns = 8
                    seq_column_index = None
                    wrote_columns = False
                    prev_ts = None
                    prev_seq = None
                    continue
                if magic_idx > 0:
                    first_nl = buf.find(b"\n")
                    if first_nl < 0 or first_nl > magic_idx:
                        if args.show_noncsv:
                            prefix = buf[:magic_idx].decode("ascii", errors="ignore").strip()
                            if prefix:
                                print(prefix, file=sys.stderr)
                        buf = buf[magic_idx:]
                        continue

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
                    formatted_columns = _format_mm_columns(pending_columns)
                    if formatted_columns:
                        current_columns = [c.strip() for c in formatted_columns.split(",")]
                        expected_csv_columns = len(current_columns)
                        seq_column_index = current_columns.index("seq") if "seq" in current_columns else None
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

                if line.startswith("#log_format="):
                    pending_fw_meta = [line]
                    fw_meta_written_count = 0
                    pending_columns = None
                    current_columns = []
                    expected_csv_columns = 8
                    seq_column_index = None
                    out_path = None
                    wrote_columns = False
                    prev_ts = None
                    prev_seq = None
                    print(f"[INFO] FW Meta: {line}", file=sys.stderr)
                    continue

                if (
                    line.startswith("#fw_")
                    or line.startswith("#last_test_")
                    or line.startswith("#op_")
                    or line.startswith("#tune_")
                    or line.startswith("#wall_trace_")
                ):
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
                if len(parts) == expected_csv_columns and parts[0].isdigit() and all(_is_num(p) for p in parts[1:]):
                    ts = int(parts[0])
                    seq: Optional[int] = None
                    if seq_column_index is not None:
                        seq = int(parts[seq_column_index])
                    seq_reset = (
                        seq is not None
                        and prev_seq is not None
                        and seq == 0
                        and prev_seq != 0
                    )
                    if (
                        prev_ts is not None
                        and ts < prev_ts
                        and seq is not None
                        and prev_seq is not None
                        and seq != 0
                    ):
                        print(
                            "[WARN] Dropped suspicious CSV row: "
                            f"timestamp moved backward ({ts} < {prev_ts}) but seq={seq}",
                            file=sys.stderr,
                        )
                        continue

                    if out_path is None or (prev_ts is not None and ts < prev_ts) or seq_reset:
                        out_path = _new_output_file(save_dir)
                        print(f"\n[INFO] New capture file: {out_path}", file=sys.stderr)
                        wrote_columns = False
                        fw_meta_written_count = 0
                        prev_seq = None
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
                    prev_seq = seq
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
