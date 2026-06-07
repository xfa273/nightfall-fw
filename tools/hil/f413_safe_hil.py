#!/usr/bin/env python3
"""Safe F413 HIL helpers.

This wrapper only automates non-motor workflows. It starts existing repo tools,
adds timeouts, and avoids UART commands that can drive motors, fans, search, or
shortest-run paths.
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from pathlib import Path


DEFAULT_BAUD = 921600
DEFAULT_LOG_DIR = Path("tools/logging/logs")
DEFAULT_CAPTURE_SECONDS = 8.0
NONMOTOR_SMOKE_SEND = "i,w,V"
TRACE_DUMP_SEND = "V"


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def run(cmd: list[str], cwd: Path, check: bool = True) -> int:
    print("$ " + " ".join(cmd), flush=True)
    completed = subprocess.run(cmd, cwd=str(cwd), check=check)
    return completed.returncode


def terminate_process(proc: subprocess.Popen[bytes]) -> None:
    if proc.poll() is not None:
        return
    try:
        proc.send_signal(signal.SIGINT)
        proc.wait(timeout=1.5)
        return
    except Exception:
        pass
    try:
        proc.terminate()
        proc.wait(timeout=1.5)
        return
    except Exception:
        pass
    try:
        proc.kill()
    except Exception:
        pass


def timed_capture(cmd: list[str], cwd: Path, seconds: float) -> int:
    print("$ " + " ".join(cmd), flush=True)
    proc = subprocess.Popen(cmd, cwd=str(cwd))
    try:
        proc.wait(timeout=seconds)
    except subprocess.TimeoutExpired:
        terminate_process(proc)
    return proc.returncode if proc.returncode is not None else 0


def serial_capture_args(args: argparse.Namespace, send: str | None) -> list[str]:
    cmd = [
        sys.executable,
        "tools/logging/serial_capture_csv.py",
        "--show-noncsv",
        "--send-interval-ms",
        str(args.send_interval_ms),
    ]
    if send:
        cmd.extend(["--send", send])
    cmd.extend([str(args.log_dir), args.port, str(args.baud)])
    return cmd


def serial_terminal_args(args: argparse.Namespace, log_path: Path) -> list[str]:
    return [
        sys.executable,
        "tools/logging/serial_terminal.py",
        "--port",
        args.port,
        "--baud",
        str(args.baud),
        "--log",
        str(log_path),
    ]


def stlink_args(args: argparse.Namespace, *extra: str) -> list[str]:
    cmd = [sys.executable, "tools/flashing/flash_stlink"]
    if args.sn:
        cmd.extend(["--sn", args.sn])
    cmd.extend(extra)
    return cmd


def cmd_list(args: argparse.Namespace) -> int:
    root = repo_root()
    rc1 = run(stlink_args(args, "--list"), root, check=False)
    rc2 = run([sys.executable, "tools/logging/serial_terminal.py", "--list"], root, check=False)
    return 0 if rc1 == 0 and rc2 == 0 else 1


def cmd_reset_capture(args: argparse.Namespace) -> int:
    root = repo_root()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = args.log_dir / f"f413_boot_{stamp}.log"

    capture_cmd = serial_capture_args(args, None)
    print("$ " + " ".join(capture_cmd) + f" > {log_path}", flush=True)
    log_file = log_path.open("wb")
    cap = subprocess.Popen(capture_cmd, cwd=str(root), stdout=log_file, stderr=subprocess.STDOUT)
    try:
        time.sleep(args.pre_reset_delay)
        rc = run(stlink_args(args, "--reset-only"), root, check=False)
        time.sleep(args.duration)
        return rc
    finally:
        terminate_process(cap)
        log_file.close()
        print(f"boot log: {log_path}", flush=True)


def cmd_nonmotor_smoke(args: argparse.Namespace) -> int:
    root = repo_root()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    return timed_capture(serial_capture_args(args, NONMOTOR_SMOKE_SEND), root, args.duration)


def cmd_dump_trace(args: argparse.Namespace) -> int:
    root = repo_root()
    args.log_dir.mkdir(parents=True, exist_ok=True)
    return timed_capture(serial_capture_args(args, TRACE_DUMP_SEND), root, args.duration)


def cmd_flash_nonmotor_smoke(args: argparse.Namespace) -> int:
    root = repo_root()
    rc = run(stlink_args(args, "--build"), root, check=False)
    if rc != 0:
        return rc
    time.sleep(args.after_flash_delay)
    return cmd_nonmotor_smoke(args)


def add_common(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--port", default=os.environ.get("NIGHTFALL_UART_PORT"), help="UART port, e.g. /dev/cu.usbmodem112202")
    ap.add_argument("--baud", type=int, default=int(os.environ.get("NIGHTFALL_UART_BAUD", DEFAULT_BAUD)))
    ap.add_argument("--sn", default=os.environ.get("STLINK_SN"), help="ST-LINK serial number")
    ap.add_argument("--log-dir", type=Path, default=DEFAULT_LOG_DIR)
    ap.add_argument("--duration", type=float, default=DEFAULT_CAPTURE_SECONDS)
    ap.add_argument("--send-interval-ms", type=int, default=1500)


def require_port(args: argparse.Namespace) -> None:
    if not args.port:
        raise SystemExit("ERROR: --port is required. Use `python3 tools/hil/f413_safe_hil.py list` first.")


def main() -> int:
    parser = argparse.ArgumentParser(description="Safe non-motor F413 HIL workflows")
    sub = parser.add_subparsers(dest="command", required=True)

    p_list = sub.add_parser("list", help="list ST-LINK probes and UART ports")
    add_common(p_list)
    p_list.set_defaults(func=cmd_list, needs_port=False)

    p_reset = sub.add_parser("reset-capture", help="capture UART while issuing ST-LINK software reset")
    add_common(p_reset)
    p_reset.add_argument("--pre-reset-delay", type=float, default=0.5)
    p_reset.set_defaults(func=cmd_reset_capture, needs_port=True)

    p_smoke = sub.add_parser("nonmotor-smoke", help="send safe non-motor UART checks: i,w,V")
    add_common(p_smoke)
    p_smoke.set_defaults(func=cmd_nonmotor_smoke, needs_port=True)

    p_dump = sub.add_parser("dump-trace", help="send V and capture the latest trace CSV")
    add_common(p_dump)
    p_dump.set_defaults(func=cmd_dump_trace, needs_port=True)

    p_flash = sub.add_parser("flash-nonmotor-smoke", help="build/flash F413 then run non-motor smoke")
    add_common(p_flash)
    p_flash.add_argument("--after-flash-delay", type=float, default=1.0)
    p_flash.set_defaults(func=cmd_flash_nonmotor_smoke, needs_port=True)

    args = parser.parse_args()
    if getattr(args, "needs_port", False):
        require_port(args)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
