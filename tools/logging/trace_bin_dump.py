#!/usr/bin/env python3
import argparse
import csv
import fcntl
import glob
import os
import select
import struct
import sys
import termios
import time
from pathlib import Path
from typing import Optional

MAGIC = b"NFTB"
DEFAULT_BAUD = int(os.environ.get("NIGHTFALL_UART_BAUD", "921600"))
IOSSIOSPEED = 0x80045402
FRAME_STRUCT = struct.Struct("<IIIIIIII")
HEADER_STRUCT = struct.Struct("<IIIIIIII")
RECORD_COLUMNS_V3 = [
    "timestamp_ms",
    "seq",
    "op_mode",
    "op_case",
    "op_sub",
    "test_id",
    "target_distance_mm",
    "distance_mm",
    "angle_mdeg",
    "target_velocity_mm_s",
    "real_velocity_mm_s",
    "accel_velocity_mm_s",
    "target_omega_mdps",
    "real_omega_mdps",
    "target_angle_mdeg",
    "accel_forward_mm_s2",
    "encoder_l",
    "encoder_r",
    "motor_out_l",
    "motor_out_r",
    "adc_fr",
    "adc_r",
    "adc_fl",
    "adc_l",
    "adc_vbat",
    "flags",
    "reserved_i32_0",
    "reserved_i32_1",
    "reserved_i32_2",
    "reserved_i32_3",
    "reserved_u16_0",
    "reserved_u16_1",
]
RECORD_COLUMNS_V4 = [
    "timestamp_ms",
    "seq",
    "op_mode",
    "op_case",
    "op_sub",
    "test_id",
    "target_distance_mm",
    "distance_mm",
    "angle_mdeg",
    "target_velocity_mm_s",
    "real_velocity_mm_s",
    "accel_velocity_mm_s",
    "target_omega_mdps",
    "real_omega_mdps",
    "gyro_z_raw_mdps",
    "gyro_z_lpf_002_mdps",
    "gyro_z_lpf_005_mdps",
    "gyro_z_lpf_010_mdps",
    "gyro_z_lpf_020_mdps",
    "target_angle_mdeg",
    "accel_forward_mm_s2",
    "encoder_l",
    "encoder_r",
    "motor_out_l",
    "motor_out_r",
    "adc_fr",
    "adc_r",
    "adc_fl",
    "adc_l",
    "adc_vbat",
    "flags",
    "reserved_i32_0",
    "reserved_i32_1",
    "reserved_i32_2",
    "reserved_i32_3",
    "reserved_u16_0",
    "reserved_u16_1",
]
RECORD_COLUMNS_V5 = [
    "timestamp_ms",
    "seq",
    "op_mode",
    "op_case",
    "op_sub",
    "test_id",
    "target_distance_mm",
    "distance_mm",
    "angle_mdeg",
    "target_velocity_mm_s",
    "real_velocity_mm_s",
    "accel_velocity_mm_s",
    "target_omega_mdps",
    "real_omega_mdps",
    "gyro_z_raw_mdps",
    "target_angle_mdeg",
    "accel_forward_mm_s2",
    "encoder_l",
    "encoder_r",
    "motor_out_l",
    "motor_out_r",
    "adc_fr",
    "adc_r",
    "adc_fl",
    "adc_l",
    "adc_vbat",
    "flags",
    "reserved_i32_0",
    "reserved_i32_1",
    "reserved_i32_2",
    "reserved_i32_3",
    "reserved_u16_0",
    "reserved_u16_1",
]
RECORD_COLUMNS_V6 = [
    "timestamp_ms",
    "seq",
    "op_mode",
    "op_case",
    "op_sub",
    "test_id",
    "target_distance_mm",
    "distance_mm",
    "angle_mdeg",
    "target_velocity_mm_s",
    "real_velocity_mm_s",
    "accel_velocity_mm_s",
    "target_omega_mdps",
    "real_omega_mdps",
    "gyro_z_raw_mdps",
    "target_angle_mdeg",
    "accel_forward_mm_s2",
    "encoder_l",
    "encoder_r",
    "motor_out_l",
    "motor_out_r",
    "adc_fr",
    "adc_r",
    "adc_fl",
    "adc_l",
    "adc_vbat",
    "wall_read_fr",
    "wall_read_r",
    "wall_read_fl",
    "wall_read_l",
    "flags",
    "reserved_i32_0",
    "reserved_i32_1",
    "reserved_i32_2",
    "reserved_i32_3",
    "reserved_u16_0",
    "reserved_u16_1",
]
RECORD_STRUCT_V3 = struct.Struct("<II14i4h6H4B2H")
RECORD_STRUCT_V4 = struct.Struct("<II19i4h6H4B2H")
RECORD_STRUCT_V5 = struct.Struct("<II15i4h6H4B2H")
RECORD_STRUCT_V6 = struct.Struct("<II15i4h10H4B2H")
RECORD_STRUCT = RECORD_STRUCT_V6
RECORD_COLUMNS = RECORD_COLUMNS_V6
SEARCH_EVENT_MARKER = 0x5345
SEARCH_EVENT_SESSION_START = 0xE0
SEARCH_EVENT_PHASE = 0xE1
SEARCH_EVENT_DECISION = 0xE2
SEARCH_EVENT_MOTION_END = 0xE3
SEARCH_EVENT_SESSION_END = 0xE4
SEARCH_EVENT_ROUTE_FAIL = 0xE5
SEARCH_EVENT_WALL_END = 0xE6
SEARCH_EVENT_COLUMNS = [
    "event_marker",
    "event_type",
    "event_action",
    "event_x",
    "event_y",
    "event_dir",
    "event_next_rel",
    "event_phase",
    "event_target",
    "event_flags",
    "event_wall_info",
    "event_map_cell",
    "event_smap_step",
    "event_next_after_forward",
    "event_param_index",
    "event_motion_kind",
    "event_motion_status",
    "event_motion_duration_ms",
    "event_arg0_x1000",
    "event_arg1_x1000",
    "event_completed",
    "event_route_failed",
    "event_route_reason",
]
RECORD_LAYOUTS = {
    RECORD_STRUCT_V3.size: (RECORD_STRUCT_V3, RECORD_COLUMNS_V3),
    RECORD_STRUCT_V4.size: (RECORD_STRUCT_V4, RECORD_COLUMNS_V4),
    RECORD_STRUCT_V5.size: (RECORD_STRUCT_V5, RECORD_COLUMNS_V5),
    RECORD_STRUCT_V6.size: (RECORD_STRUCT_V6, RECORD_COLUMNS_V6),
}


def record_struct_for_size(record_size: int) -> Optional[struct.Struct]:
    layout = RECORD_LAYOUTS.get(record_size)
    return layout[0] if layout else None


def record_columns_for_size(record_size: int) -> list[str]:
    layout = RECORD_LAYOUTS.get(record_size)
    return layout[1] if layout else RECORD_COLUMNS


def _detect_port() -> Optional[str]:
    candidates: list[str] = []
    if sys.platform == "darwin":
        for pattern in ("/dev/cu.usbmodem*", "/dev/cu.usbserial*"):
            candidates.extend(glob.glob(pattern))
    else:
        for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*"):
            candidates.extend(glob.glob(pattern))
    candidates = sorted(set(candidates))
    return candidates[0] if candidates else None


def _baud_const(baud: int) -> int:
    name = f"B{baud}"
    if not hasattr(termios, name):
        raise ValueError(f"unsupported baud by termios: {baud}")
    return int(getattr(termios, name))


def _configure_serial(fd: int, baud: int) -> None:
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
    attrs[3] = 0
    try:
        speed = _baud_const(baud)
    except ValueError:
        speed = _baud_const(9600)
        custom_baud = True
    else:
        custom_baud = False
    attrs[4] = speed
    attrs[5] = speed
    attrs[6][termios.VMIN] = 0
    attrs[6][termios.VTIME] = 1
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    if custom_baud:
        fcntl.ioctl(fd, IOSSIOSPEED, struct.pack("I", baud))


def capture_raw(port: str, baud: int, command: str, timeout_s: float) -> bytes:
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        _configure_serial(fd, baud)
        time.sleep(0.1)
        if command:
            os.write(fd, command.encode("ascii"))
        deadline = time.monotonic() + timeout_s
        chunks: list[bytes] = []
        while time.monotonic() < deadline:
            r, _w, _x = select.select([fd], [], [], 0.1)
            if not r:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            if chunk:
                chunks.append(chunk)
        return b"".join(chunks)
    finally:
        os.close(fd)


def checksum(data: bytes) -> int:
    return sum(data) & 0xFFFFFFFF


def _record_to_row_v3(values: tuple[int, ...]) -> list[str]:
    seq = values[0]
    timestamp_ms = values[1]
    i32 = values[2:16]
    h = values[16:20]
    u16 = values[20:26]
    op_mode, op_case, op_sub, test_id = values[26:30]
    ru16_0, ru16_1 = values[30:32]
    target_distance = i32[0] / 1000.0
    return [
        str(timestamp_ms),
        str(seq),
        str(op_mode),
        str(op_case),
        str(op_sub),
        str(test_id),
        f"{target_distance:.3f}",
        str(i32[1]),
        str(i32[2]),
        str(i32[3]),
        str(i32[4]),
        str(i32[5]),
        str(i32[6]),
        str(i32[7]),
        str(i32[8]),
        str(i32[9]),
        str(h[0]),
        str(h[1]),
        str(h[2]),
        str(h[3]),
        str(u16[0]),
        str(u16[1]),
        str(u16[2]),
        str(u16[3]),
        str(u16[4]),
        str(u16[5]),
        str(i32[10]),
        str(i32[11]),
        str(i32[12]),
        str(i32[13]),
        str(ru16_0),
        str(ru16_1),
    ]


def _record_to_row_v4(values: tuple[int, ...]) -> list[str]:
    seq = values[0]
    timestamp_ms = values[1]
    i32 = values[2:21]
    h = values[21:25]
    u16 = values[25:31]
    op_mode, op_case, op_sub, test_id = values[31:35]
    ru16_0, ru16_1 = values[35:37]
    target_distance = i32[0] / 1000.0
    return [
        str(timestamp_ms),
        str(seq),
        str(op_mode),
        str(op_case),
        str(op_sub),
        str(test_id),
        f"{target_distance:.3f}",
        str(i32[1]),
        str(i32[2]),
        str(i32[3]),
        str(i32[4]),
        str(i32[5]),
        str(i32[6]),
        str(i32[7]),
        str(i32[8]),
        str(i32[9]),
        str(i32[10]),
        str(i32[11]),
        str(i32[12]),
        str(i32[13]),
        str(i32[14]),
        str(h[0]),
        str(h[1]),
        str(h[2]),
        str(h[3]),
        str(u16[0]),
        str(u16[1]),
        str(u16[2]),
        str(u16[3]),
        str(u16[4]),
        str(u16[5]),
        str(i32[15]),
        str(i32[16]),
        str(i32[17]),
        str(i32[18]),
        str(ru16_0),
        str(ru16_1),
    ]


def _record_to_row_v5(values: tuple[int, ...]) -> list[str]:
    seq = values[0]
    timestamp_ms = values[1]
    i32 = values[2:17]
    h = values[17:21]
    u16 = values[21:27]
    op_mode, op_case, op_sub, test_id = values[27:31]
    ru16_0, ru16_1 = values[31:33]
    target_distance = i32[0] / 1000.0
    return [
        str(timestamp_ms),
        str(seq),
        str(op_mode),
        str(op_case),
        str(op_sub),
        str(test_id),
        f"{target_distance:.3f}",
        str(i32[1]),
        str(i32[2]),
        str(i32[3]),
        str(i32[4]),
        str(i32[5]),
        str(i32[6]),
        str(i32[7]),
        str(i32[8]),
        str(i32[9]),
        str(i32[10]),
        str(h[0]),
        str(h[1]),
        str(h[2]),
        str(h[3]),
        str(u16[0]),
        str(u16[1]),
        str(u16[2]),
        str(u16[3]),
        str(u16[4]),
        str(u16[5]),
        str(i32[11]),
        str(i32[12]),
        str(i32[13]),
        str(i32[14]),
        str(ru16_0),
        str(ru16_1),
    ]


def _record_to_row_v6(values: tuple[int, ...]) -> list[str]:
    seq = values[0]
    timestamp_ms = values[1]
    i32 = values[2:17]
    h = values[17:21]
    u16 = values[21:31]
    op_mode, op_case, op_sub, test_id = values[31:35]
    ru16_0, ru16_1 = values[35:37]
    target_distance = i32[0] / 1000.0
    return [
        str(timestamp_ms),
        str(seq),
        str(op_mode),
        str(op_case),
        str(op_sub),
        str(test_id),
        f"{target_distance:.3f}",
        str(i32[1]),
        str(i32[2]),
        str(i32[3]),
        str(i32[4]),
        str(i32[5]),
        str(i32[6]),
        str(i32[7]),
        str(i32[8]),
        str(i32[9]),
        str(i32[10]),
        str(h[0]),
        str(h[1]),
        str(h[2]),
        str(h[3]),
        str(u16[0]),
        str(u16[1]),
        str(u16[2]),
        str(u16[3]),
        str(u16[4]),
        str(u16[5]),
        str(u16[6]),
        str(u16[7]),
        str(u16[8]),
        str(u16[9]),
        str(i32[11]),
        str(i32[12]),
        str(i32[13]),
        str(i32[14]),
        str(ru16_0),
        str(ru16_1),
    ]


def _record_to_row(values: tuple[int, ...], record_size: int) -> list[str]:
    if record_size == RECORD_STRUCT_V3.size:
        return _record_to_row_v3(values)
    if record_size == RECORD_STRUCT_V4.size:
        return _record_to_row_v4(values)
    if record_size == RECORD_STRUCT_V5.size:
        return _record_to_row_v5(values)
    if record_size == RECORD_STRUCT_V6.size:
        return _record_to_row_v6(values)
    raise ValueError(f"unsupported record size: {record_size}")


def _int_field(row: dict[str, str], key: str, default: int = 0) -> int:
    try:
        return int(float(row.get(key, "")))
    except (TypeError, ValueError):
        return default


def _decode_search_event(row: list[str], columns: list[str]) -> list[str]:
    data = dict(zip(columns, row))
    marker = _int_field(data, "reserved_u16_0")
    event_type = _int_field(data, "test_id")

    if marker != SEARCH_EVENT_MARKER or event_type < SEARCH_EVENT_SESSION_START:
        return ["0"] * len(SEARCH_EVENT_COLUMNS)

    action = _int_field(data, "reserved_u16_1")
    pose = _int_field(data, "reserved_i32_0") & 0xFFFFFFFF
    x = pose & 0xFF
    y = (pose >> 8) & 0xFF
    direction = (pose >> 16) & 0x03
    next_rel = (pose >> 18) & 0x03
    phase = (pose >> 20) & 0x0F
    target = (pose >> 24) & 0x0F
    event_flags = (pose >> 28) & 0x0F

    wall_info = 0
    map_cell = 0
    smap_step = 0
    next_after_forward = 0
    param_index = 0
    motion_kind = 0
    motion_status = 0
    motion_duration_ms = 0
    arg0_x1000 = 0
    arg1_x1000 = 0
    completed = 0
    route_failed = 0
    route_reason = 0

    r1 = _int_field(data, "reserved_i32_1")
    r2 = _int_field(data, "reserved_i32_2")
    r3 = _int_field(data, "reserved_i32_3")

    if event_type in (SEARCH_EVENT_PHASE, SEARCH_EVENT_DECISION, SEARCH_EVENT_ROUTE_FAIL):
        wall_pack = r1 & 0xFFFFFFFF
        wall_info = wall_pack & 0xFFFF
        map_cell = (wall_pack >> 16) & 0xFFFF
    if event_type == SEARCH_EVENT_PHASE:
        smap_step = r2
    elif event_type == SEARCH_EVENT_DECISION:
        smap_step = r2
        next_after_forward = r3 & 0xFF
        param_index = (r3 >> 16) & 0xFF
    elif event_type == SEARCH_EVENT_ROUTE_FAIL:
        smap_step = r2
        route_reason = r3
    elif event_type == SEARCH_EVENT_SESSION_START:
        param_index = r2
    elif event_type == SEARCH_EVENT_MOTION_END:
        motion_pack = r3 & 0xFFFFFFFF
        motion_kind = (motion_pack >> 24) & 0xFF
        motion_status = (motion_pack >> 16) & 0xFF
        motion_duration_ms = motion_pack & 0xFFFF
        arg0_x1000 = r1
        arg1_x1000 = r2
    elif event_type == SEARCH_EVENT_WALL_END:
        motion_duration_ms = r3 & 0xFFFF
        arg0_x1000 = r1
        arg1_x1000 = r2
    elif event_type == SEARCH_EVENT_SESSION_END:
        completed = r2
        route_failed = r3

    return [
        str(marker),
        str(event_type),
        str(action),
        str(x),
        str(y),
        str(direction),
        str(next_rel),
        str(phase),
        str(target),
        str(event_flags),
        str(wall_info),
        str(map_cell),
        str(smap_step),
        str(next_after_forward),
        str(param_index),
        str(motion_kind),
        str(motion_status),
        str(motion_duration_ms),
        str(arg0_x1000),
        str(arg1_x1000),
        str(completed),
        str(route_failed),
        str(route_reason),
    ]


def extract_frame(raw: bytes) -> tuple[dict[str, int], dict[str, int], list[list[str]], int]:
    pos = 0
    while True:
        idx = raw.find(MAGIC, pos)
        if idx < 0:
            raise ValueError("trace binary magic not found")
        if idx + FRAME_STRUCT.size > len(raw):
            raise ValueError("incomplete frame header")
        fields = FRAME_STRUCT.unpack_from(raw, idx)
        frame = {
            "magic": fields[0],
            "version": fields[1],
            "schema": fields[2],
            "header_size": fields[3],
            "record_size": fields[4],
            "record_count": fields[5],
            "available_count": fields[6],
            "payload_checksum": fields[7],
        }
        total_len = FRAME_STRUCT.size + frame["header_size"] + frame["record_size"] * frame["record_count"]
        rec_struct = record_struct_for_size(frame["record_size"])
        if frame["version"] == 1 and frame["header_size"] == HEADER_STRUCT.size and rec_struct is not None and idx + total_len <= len(raw):
            payload = raw[idx + FRAME_STRUCT.size : idx + total_len]
            if checksum(payload) == frame["payload_checksum"]:
                header_values = HEADER_STRUCT.unpack_from(payload, 0)
                header = {
                    "magic": header_values[0],
                    "version": header_values[1],
                    "length": header_values[2],
                    "crc": header_values[3],
                    "record_size": header_values[4],
                    "record_capacity": header_values[5],
                    "write_index": header_values[6],
                    "total_records": header_values[7],
                }
                rows: list[list[str]] = []
                off = frame["header_size"]
                columns = record_columns_for_size(frame["record_size"])
                for _ in range(frame["record_count"]):
                    rec = rec_struct.unpack_from(payload, off)
                    row = _record_to_row(rec, frame["record_size"])
                    row.extend(_decode_search_event(row, columns))
                    rows.append(row)
                    off += frame["record_size"]
                return frame, header, rows, idx
        pos = idx + 1


def write_csv(path: Path, frame: dict[str, int], header: dict[str, int], rows: list[list[str]]) -> None:
    with path.open("w", encoding="ascii", newline="") as f:
        f.write("#log_format=nightfall_trace_bin_v1_decoded\n")
        f.write(f"#fw_log_schema=0x{frame['schema']:08X}\n")
        f.write(f"#bin_record_count={frame['record_count']}\n")
        f.write(f"#bin_available_count={frame['available_count']}\n")
        f.write(f"#bin_header_total_records={header['total_records']}\n")
        f.write("#mm_columns=" + ",".join(record_columns_for_size(frame["record_size"]) + SEARCH_EVENT_COLUMNS) + "\n")
        writer = csv.writer(f)
        for row in rows:
            writer.writerow(row)


def main() -> int:
    ap = argparse.ArgumentParser(description="Capture or decode Nightfall trace binary dump")
    ap.add_argument("input", nargs="?", help="raw binary/SWV log file to decode; omit to capture from serial")
    ap.add_argument("--port", default="auto")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    ap.add_argument("--send", default=">", help="UART command to request dump when capturing")
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--raw-out", default=None)
    ap.add_argument("--csv-out", default=None)
    args = ap.parse_args()

    if args.input:
        raw_path = Path(args.input).expanduser()
        raw = raw_path.read_bytes()
    else:
        port = _detect_port() if args.port in ("", "auto") else args.port
        if not port:
            print("UART port not found", file=sys.stderr)
            return 1
        raw = capture_raw(port, args.baud, args.send, args.timeout)
        raw_path = Path(args.raw_out) if args.raw_out else Path("tools/logging/logs") / f"trace_bin_{time.strftime('%Y%m%d_%H%M%S')}.raw"
        raw_path.parent.mkdir(parents=True, exist_ok=True)
        raw_path.write_bytes(raw)
        print(f"raw: {raw_path} ({len(raw)} bytes)")

    frame, header, rows, offset = extract_frame(raw)
    csv_path = Path(args.csv_out) if args.csv_out else raw_path.with_suffix(".csv")
    write_csv(csv_path, frame, header, rows)
    print(f"frame_offset={offset} records={len(rows)} csv={csv_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
