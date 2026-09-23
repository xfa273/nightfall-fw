"""Host-only tests: never connect to a probe or execute a vendor CLI."""

import argparse
import contextlib
import importlib.machinery
import importlib.util
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch

import cube_cli_runtime as runtime


loader = importlib.machinery.SourceFileLoader(
    "flash_stlink", str(Path(__file__).with_name("flash_stlink")))
spec = importlib.util.spec_from_loader(loader.name, loader)
flash = importlib.util.module_from_spec(spec)
loader.exec_module(flash)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.contexts = contextlib.ExitStack()
        self.addCleanup(self.contexts.close)
        self.install = Path(self.directory.name).resolve() / "Cube App/Contents/MacOs"
        self.cli = self.install / "bin/STM32_Programmer_CLI"
        self.api = self.install / "api/lib/STM32_Programmer_CLI"
        for executable in (self.cli, self.api):
            executable.parent.mkdir(parents=True, exist_ok=True)
            executable.write_bytes(b"test fixture, not executable code")
        for name in ("Data_Base", "bin/FlashLoader", "Drivers"):
            (self.install / name).mkdir()
        (self.api.parent / "libusb-1.0.0.dylib").write_bytes(b"library")
        (self.api.parent / "QtCore.framework").mkdir()
        self.silicon = self.contexts.enter_context(patch.object(runtime, "apple_silicon", return_value=True))
        self.arches = self.contexts.enter_context(patch.object(
            runtime, "macho_arches", side_effect=lambda p: {"x86_64"} if p == self.cli else {"arm64", "x86_64"}))
        self.contexts.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def test_native_sdk_layout_and_cleanup(self):
        original = self.cli.read_bytes()
        with runtime.cube_cli_runtime(str(self.cli)) as selected:
            executable = Path(selected)
            root = executable.parent.parent
            self.assertFalse(executable.is_symlink())
            self.assertEqual(executable.read_bytes(), self.api.read_bytes())
            self.assertTrue(executable.stat().st_mode & 0o100)
            self.assertEqual((root / "Data_Base").resolve(), self.install / "Data_Base")
            self.assertEqual((root / "Drivers").resolve(), self.install / "Drivers")
            self.assertEqual((executable.parent / "FlashLoader").resolve(), self.install / "bin/FlashLoader")
            self.assertEqual((executable.parent / "libusb-1.0.0.dylib").read_bytes(), b"library")
            self.assertTrue((executable.parent / "QtCore.framework").is_dir())
        self.assertFalse(root.exists())
        self.assertEqual(self.cli.read_bytes(), original)

    def test_explicit_sdk_path_also_gets_resources(self):
        with runtime.cube_cli_runtime(str(self.api)) as selected:
            self.assertNotEqual(selected, str(self.api))
            self.assertTrue((Path(selected).parent.parent / "Data_Base").is_dir())

    def test_cleanup_on_command_error(self):
        with self.assertRaises(subprocess.CalledProcessError):
            with runtime.cube_cli_runtime(str(self.cli)) as selected:
                root = Path(selected).parent.parent
                raise subprocess.CalledProcessError(1, [selected, "-l"])
        self.assertFalse(root.exists())

    def test_native_bin_or_custom_wrapper_unchanged(self):
        for arches in ({"arm64", "x86_64"}, set()):
            self.arches.side_effect = None
            self.arches.return_value = arches
            with runtime.cube_cli_runtime(str(self.cli)) as selected:
                self.assertEqual(selected, str(self.cli))

    def test_other_hosts_unchanged(self):
        self.silicon.return_value = False
        with runtime.cube_cli_runtime(str(self.cli)) as selected:
            self.assertEqual(selected, str(self.cli))
        self.arches.assert_not_called()

    def test_missing_arm_sdk_fails_before_execution(self):
        self.arches.side_effect = None
        self.arches.return_value = {"x86_64"}
        with self.assertRaisesRegex(RuntimeError, "arm64非対応"):
            with runtime.cube_cli_runtime(str(self.cli)):
                self.fail("must not launch incompatible executable")

    def test_missing_database_fails_closed(self):
        (self.install / "Data_Base").rmdir()
        with self.assertRaisesRegex(RuntimeError, "必要なリソース"):
            with runtime.cube_cli_runtime(str(self.cli)):
                self.fail("must not launch incomplete runtime")

    def test_missing_loader_fails_closed(self):
        (self.install / "bin/FlashLoader").rmdir()
        with self.assertRaisesRegex(RuntimeError, "必要なリソース"):
            with runtime.cube_cli_runtime(str(self.cli)):
                self.fail("must not launch incomplete runtime")


class ArchitectureTests(unittest.TestCase):
    def test_lipo_parses_universal_and_rejects_non_macho(self):
        with patch.object(runtime.subprocess, "run") as run:
            run.return_value = subprocess.CompletedProcess([], 0, "x86_64 arm64\n")
            self.assertEqual(runtime.macho_arches(Path("cli")), {"x86_64", "arm64"})
            run.return_value = subprocess.CompletedProcess([], 1, "")
            self.assertEqual(runtime.macho_arches(Path("wrapper")), set())

    def test_rosetta_python_detects_arm_host(self):
        with patch.object(runtime.sys, "platform", "darwin"), \
             patch.object(runtime.platform, "machine", return_value="x86_64"), \
             patch.object(runtime.subprocess, "run") as run:
            run.return_value = subprocess.CompletedProcess([], 0, "1\n")
            self.assertTrue(runtime.apple_silicon())
            run.return_value = subprocess.CompletedProcess([], 0, "0\n")
            self.assertFalse(runtime.apple_silicon())


class CommandTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.image = self.root / "firmware.bin"
        self.image.write_bytes(b"not a firmware")
        self.args = argparse.Namespace(
            list=False, reset_only=False, build=False, image=str(self.image),
            address="0x08000000", no_verify=False, no_reset=False,
            freq="4000", mode="NORMAL", reset_mode="SWrst", sn="test-probe", shared=False)
        self.contexts = contextlib.ExitStack()
        self.addCleanup(self.contexts.close)
        self.command = self.contexts.enter_context(patch.object(
            flash.subprocess, "run", return_value=subprocess.CompletedProcess([], 0, "")))
        self.contexts.enter_context(contextlib.redirect_stdout(io.StringIO()))

    def test_list_never_builds_writes_or_resets(self):
        self.args.list = True
        self.assertEqual(flash.execute(self.args, "native-cli", self.root), 0)
        self.assertEqual(self.command.call_count, 1)
        self.assertEqual(self.command.call_args.args[0], ["native-cli", "-l", "stlink-only"])

    def test_write_command_unchanged_with_build_verify_reset(self):
        self.args.build = True
        flash.execute(self.args, "native-cli", self.root)
        commands = [call.args[0] for call in self.command.call_args_list
                    if "-l" not in call.args[0]]
        self.assertEqual(len(commands), 2)
        self.assertEqual(commands[0],
                         ["cmake", "--build", "--preset", "Debug-stm32f413"])
        self.assertEqual(commands[1], [
            "native-cli", "-c", "port=SWD", "freq=4000", "mode=NORMAL", "reset=SWrst",
            "sn=test-probe", "-w", str(self.image), "0x08000000", "-v", "-rst"])

    def test_explicit_cli_and_path_precedence_preserved(self):
        self.assertEqual(flash.find_cube_cli(str(self.image)), str(self.image))
        with patch.object(flash.shutil, "which", return_value="/custom/cli"):
            self.assertEqual(flash.find_cube_cli(None), "/custom/cli")
        with self.assertRaises(RuntimeError):
            flash.find_cube_cli(str(self.root / "missing"))


if __name__ == "__main__":
    unittest.main()
