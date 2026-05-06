#!/usr/bin/env python3
import argparse
import struct
from pathlib import Path

NVM_IDENTITY_MAGIC = 0x4E464944
NVM_IDENTITY_SCHEMA_VERSION = 0x00010000

FAMILY_UNKNOWN = 0
FAMILY_MINI = 1
FAMILY_CLASSIC = 2

FMT = "<4I2I2H3I3I4I"


def parse_u32(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0xFFFFFFFF:
        raise argparse.ArgumentTypeError(f"u32 out of range: {text}")
    return value


def parse_u16(text: str) -> int:
    value = int(text, 0)
    if value < 0 or value > 0xFFFF:
        raise argparse.ArgumentTypeError(f"u16 out of range: {text}")
    return value


def build_blob(
    family: int,
    board_id: int,
    hw_rev_major: int,
    hw_rev_minor: int,
    unit_serial: int,
    default_param_profile: int,
    capability_flags: int,
    uid0: int,
    uid1: int,
    uid2: int,
) -> bytes:
    reserved = (0, 0, 0, 0)

    temp = struct.pack(
        FMT,
        NVM_IDENTITY_MAGIC,
        NVM_IDENTITY_SCHEMA_VERSION,
        struct.calcsize(FMT),
        0,
        family,
        board_id,
        hw_rev_major,
        hw_rev_minor,
        unit_serial,
        default_param_profile,
        capability_flags,
        uid0,
        uid1,
        uid2,
        *reserved,
    )

    payload = temp[16:]
    crc = sum(payload) & 0xFFFFFFFF

    final = struct.pack(
        FMT,
        NVM_IDENTITY_MAGIC,
        NVM_IDENTITY_SCHEMA_VERSION,
        struct.calcsize(FMT),
        crc,
        family,
        board_id,
        hw_rev_major,
        hw_rev_minor,
        unit_serial,
        default_param_profile,
        capability_flags,
        uid0,
        uid1,
        uid2,
        *reserved,
    )
    return final


def main() -> int:
    ap = argparse.ArgumentParser(description="Generate NVM identity block binary")
    ap.add_argument("--out", required=True, help="output file path")
    ap.add_argument("--family", choices=["unknown", "mini", "classic"], required=True)
    ap.add_argument("--board-id", type=parse_u32, required=True)
    ap.add_argument("--hw-rev-major", type=parse_u16, required=True)
    ap.add_argument("--hw-rev-minor", type=parse_u16, required=True)
    ap.add_argument("--unit-serial", type=parse_u32, required=True)
    ap.add_argument("--default-param-profile", type=parse_u32, default=0)
    ap.add_argument("--capability-flags", type=parse_u32, default=0)
    ap.add_argument("--uid0", type=parse_u32, default=0)
    ap.add_argument("--uid1", type=parse_u32, default=0)
    ap.add_argument("--uid2", type=parse_u32, default=0)
    args = ap.parse_args()

    family_map = {
        "unknown": FAMILY_UNKNOWN,
        "mini": FAMILY_MINI,
        "classic": FAMILY_CLASSIC,
    }
    family = family_map[args.family]

    blob = build_blob(
        family=family,
        board_id=args.board_id,
        hw_rev_major=args.hw_rev_major,
        hw_rev_minor=args.hw_rev_minor,
        unit_serial=args.unit_serial,
        default_param_profile=args.default_param_profile,
        capability_flags=args.capability_flags,
        uid0=args.uid0,
        uid1=args.uid1,
        uid2=args.uid2,
    )

    out_path = Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_bytes(blob)

    print(f"Wrote {len(blob)} bytes: {out_path}")
    print(f"magic=0x{NVM_IDENTITY_MAGIC:08X} schema=0x{NVM_IDENTITY_SCHEMA_VERSION:08X}")
    print(f"family={args.family} board_id={args.board_id} rev={args.hw_rev_major}.{args.hw_rev_minor}")
    print(f"unit_serial={args.unit_serial} profile={args.default_param_profile} cap=0x{args.capability_flags:08X}")
    print(f"uid={args.uid0:08X}-{args.uid1:08X}-{args.uid2:08X}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
