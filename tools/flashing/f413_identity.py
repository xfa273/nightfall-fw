#!/usr/bin/env python3
"""Read identity or provision an EMPTY F413 identity sector. Never erases Flash.

Application flashing is deliberately separate. Provision only after flashing a
common binary that recognizes the intended board/unit and fails closed without ID.
"""
import argparse
import hashlib
import json
import re
import shutil
import struct
import subprocess
import tempfile
from pathlib import Path

from make_identity_block import build_blob, FMT, NVM_IDENTITY_MAGIC, NVM_IDENTITY_SCHEMA_VERSION

IDENTITY_BASE = 0x08160000
IDENTITY_SECTOR_BYTES = 128 * 1024
UID_BASE = 0x1FFF7A10
CUBE_MAC = "/Applications/STMicroelectronics/STM32Cube/STM32CubeProgrammer/STM32CubeProgrammer.app/Contents/MacOS/bin/STM32_Programmer_CLI"
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def parse_machine(name):
    match = re.fullmatch(r"(mini|classic)_r(\d+)_(\d+)", name)
    if not match:
        raise ValueError("machine must be mini_rM_N or classic_rM_N")
    family = 1 if match[1] == "mini" else 2
    major, minor = int(match[2]), int(match[3])
    if not 1 <= major <= 255 or not 0 <= minor <= 255:
        raise ValueError("machine revision outside board_id encoding")
    return family, major, minor


def decode_identity(data):
    if len(data) < struct.calcsize(FMT):
        raise ValueError("short identity read")
    values = struct.unpack_from(FMT, data)
    if data[:struct.calcsize(FMT)] == b"\xff" * struct.calcsize(FMT):
        return {"status": "empty"}
    if (values[0] != NVM_IDENTITY_MAGIC or values[1] != NVM_IDENTITY_SCHEMA_VERSION or
            values[2] != struct.calcsize(FMT) or values[3] != sum(data[16:values[2]])):
        return {"status": "invalid"}
    return dict(status="valid", family=values[4], board_id=f"0x{values[5]:08X}",
                revision=f"{values[6]}.{values[7]}", unit_serial=values[8],
                profile_id=values[9], capabilities=values[10],
                uid="-".join(f"{v:08X}" for v in values[11:14]))


class Cube:
    def __init__(self, cli, serial):
        self.cli, self.serial = cli, serial

    def run(self, *args, mode="HOTPLUG"):
        command = [self.cli, "-c", "port=SWD", f"mode={mode}", "freq=4000",
                   f"sn={self.serial}", *map(str, args)]
        result = subprocess.run(command, capture_output=True, text=True, timeout=45)
        output = ANSI.sub("", result.stdout + result.stderr)
        print(output, end="", flush=True)
        if result.returncode or re.search(r"\bError\b|failed", output, re.I):
            raise RuntimeError("CubeProgrammer failed; no subsequent operation attempted")
        if not re.search(r"Device ID\s*:\s*0x463\b", output):
            raise RuntimeError("Expected STM32F413/F423 device ID 0x463")
        return output

    def upload(self, address, size, path):
        self.run("-u", hex(address), size, path)
        data = path.read_bytes()
        if len(data) != size:
            raise RuntimeError(f"Readback size mismatch: {path}")
        return data


def provision_empty(cube, backup, machine, unit, expected_uid, variant=0):
    family, major, minor = parse_machine(machine)
    if not 1 <= unit <= 0xFFFFFFFF or not 0 <= variant <= 255:
        raise ValueError("invalid unit serial or board variant")
    if not re.fullmatch(r"[0-9a-fA-F]{8}(-[0-9a-fA-F]{8}){2}", expected_uid):
        raise ValueError("expected UID must be XXXXXXXX-XXXXXXXX-XXXXXXXX")
    backup.mkdir(parents=True, exist_ok=False)
    uid_bytes = cube.upload(UID_BASE, 12, backup / "uid.bin")
    uid = struct.unpack("<3I", uid_bytes)
    uid_text = "-".join(f"{v:08X}" for v in uid)
    if uid_text != expected_uid.upper() or not any(uid):
        raise RuntimeError("MCU UID mismatch; identity NOT written")
    before = cube.upload(IDENTITY_BASE, IDENTITY_SECTOR_BYTES, backup / "identity-sector-before.bin")
    if before != b"\xff" * IDENTITY_SECTOR_BYTES:
        raise RuntimeError("Identity sector is not completely empty; preserved, NOT written or erased")
    blob = build_blob(family, (major << 16) | (minor << 8) | variant,
                      major, minor, unit, 0, 0, *uid)
    image = backup / "identity.bin"
    image.write_bytes(blob)
    manifest = dict(machine=machine, unit_serial=unit, variant=variant,
                    uid=uid_text, stlink_sn=cube.serial, address=hex(IDENTITY_BASE),
                    before_sha256=hashlib.sha256(before).hexdigest(),
                    identity_sha256=hashlib.sha256(blob).hexdigest(),
                    recovery="sector was empty; identity.bin recreates this registration; no FRAM/app touched")
    (backup / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    # Recheck UID immediately before the single bounded write. No erase command.
    if cube.upload(UID_BASE, 12, backup / "uid-recheck.bin") != uid_bytes:
        raise RuntimeError("Target changed before write")
    cube.run("--skipErase", "-d", image, hex(IDENTITY_BASE), "-v", mode="NORMAL")
    after = cube.upload(IDENTITY_BASE, IDENTITY_SECTOR_BYTES, backup / "identity-sector-after.bin")
    if after != blob + before[len(blob):]:
        raise RuntimeError("Full-sector verification failed; keep backup and do not run")
    print(f"PASS: {machine}_unit{unit:03d}, UID={uid_text}; reset and verify boot selection")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["inspect", "provision-empty"])
    parser.add_argument("--sn", required=True, help="explicit ST-LINK serial")
    parser.add_argument("--cli", default=shutil.which("STM32_Programmer_CLI") or CUBE_MAC)
    parser.add_argument("--machine")
    parser.add_argument("--unit-serial", type=int)
    parser.add_argument("--variant", type=int, default=0)
    parser.add_argument("--expected-uid")
    parser.add_argument("--backup-dir", type=Path)
    parser.add_argument("--allow-identity-write", action="store_true")
    args = parser.parse_args()
    cube = Cube(args.cli, args.sn)
    if args.command == "inspect":
        with tempfile.TemporaryDirectory(prefix="nightfall-id-") as scratch:
            root = Path(scratch)
            uid = struct.unpack("<3I", cube.upload(UID_BASE, 12, root / "uid.bin"))
            data = cube.upload(IDENTITY_BASE, struct.calcsize(FMT), root / "identity.bin")
            print(json.dumps(dict(mcu_uid="-".join(f"{v:08X}" for v in uid),
                                  identity=decode_identity(data)), indent=2))
    else:
        if not (args.allow_identity_write and args.machine and args.unit_serial and
                args.expected_uid and args.backup_dir):
            parser.error("provision-empty requires --allow-identity-write, --machine, "
                         "--unit-serial, --expected-uid and new --backup-dir")
        provision_empty(cube, args.backup_dir, args.machine, args.unit_serial,
                        args.expected_uid, args.variant)


if __name__ == "__main__":
    main()
