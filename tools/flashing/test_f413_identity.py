import contextlib
import io
from pathlib import Path
import struct
import tempfile
import unittest

from f413_identity import (IDENTITY_BASE, IDENTITY_SECTOR_BYTES, UID_BASE,
                           decode_identity, parse_machine, provision_empty)


class FakeCube:
    serial = "test-probe"

    def __init__(self, sector=None):
        self.sector = sector if sector is not None else b"\xff" * IDENTITY_SECTOR_BYTES
        self.uid = struct.pack("<3I", 1, 2, 3)
        self.commands = []

    def upload(self, address, size, path):
        data = self.uid if address == UID_BASE else self.sector
        assert len(data) == size
        path.write_bytes(data)
        return data

    def run(self, *args, **kwargs):
        self.commands.append((args, kwargs))
        assert args[0] == "--skipErase" and args[1] == "-d"
        assert args[3] == hex(IDENTITY_BASE) and args[4] == "-v"
        image = Path(args[2]).read_bytes()
        self.sector = image + self.sector[len(image):]


class IdentityTests(unittest.TestCase):
    def provision(self, cube, directory, uid="00000001-00000002-00000003"):
        with contextlib.redirect_stdout(io.StringIO()):
            return provision_empty(cube, directory, "mini_r3_0", 1, uid)

    def test_empty_write_and_full_sector_backup(self):
        with tempfile.TemporaryDirectory() as raw:
            directory = Path(raw) / "backup"
            cube = FakeCube()
            self.provision(cube, directory)
            self.assertEqual(len(cube.commands), 1)
            self.assertEqual(decode_identity(cube.sector)["board_id"], "0x00030000")
            self.assertEqual(decode_identity(cube.sector)["uid"], "00000001-00000002-00000003")
            self.assertEqual((directory / "identity-sector-before.bin").stat().st_size, IDENTITY_SECTOR_BYTES)
            self.assertEqual(cube.sector[68:], b"\xff" * (IDENTITY_SECTOR_BYTES - 68))

    def test_never_overwrite_nonempty_sector_or_wrong_uid(self):
        for cube, uid in [(FakeCube(b"\x00" + b"\xff" * (IDENTITY_SECTOR_BYTES - 1)), "00000001-00000002-00000003"),
                          (FakeCube(), "00000004-00000002-00000003")]:
            with tempfile.TemporaryDirectory() as raw:
                with self.assertRaises(RuntimeError):
                    self.provision(cube, Path(raw) / "backup", uid)
                self.assertEqual(cube.commands, [])

    def test_corrupt_and_short_id(self):
        self.assertEqual(decode_identity(b"\xff" * 68)["status"], "empty")
        self.assertEqual(decode_identity(b"\x00" * 68)["status"], "invalid")
        with self.assertRaises(ValueError):
            decode_identity(b"\x00")

    def test_names_and_classic_namespace(self):
        self.assertEqual(parse_machine("mini_r2_0"), (1, 2, 0))
        self.assertEqual(parse_machine("classic_r2_0"), (2, 2, 0))
        for name in ["mini_r256_0", "mini_r0_0", "mini_r2_256", "classic", "mini_r2_0_bad"]:
            with self.assertRaises(ValueError):
                parse_machine(name)


if __name__ == "__main__":
    unittest.main()
