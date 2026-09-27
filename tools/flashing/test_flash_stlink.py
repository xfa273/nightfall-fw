"""Flash failure regression tests; no vendor executable or hardware is used."""

import contextlib
import importlib.machinery
import importlib.util
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


loader = importlib.machinery.SourceFileLoader(
    "flash_stlink_failures", str(Path(__file__).with_name("flash_stlink")))
spec = importlib.util.spec_from_loader(loader.name, loader)
flash = importlib.util.module_from_spec(spec)
loader.exec_module(flash)


class FlashTests(unittest.TestCase):
    def setUp(self):
        self.contexts = contextlib.ExitStack()
        self.addCleanup(self.contexts.close)
        directory = self.contexts.enter_context(tempfile.TemporaryDirectory())
        self.root = Path(directory)
        self.image = self.root / "firmware.bin"
        self.image.write_bytes(b"fixture, not firmware")
        self.stdout = self.contexts.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.contexts.enter_context(contextlib.redirect_stderr(io.StringIO()))
        # Guard both CLI and existing USB recovery: tests cannot reach hardware.
        self.command = self.contexts.enter_context(patch.object(flash, "run"))
        self.contexts.enter_context(patch.object(flash, "ensure_stlink_ready", create=True))
        self.contexts.enter_context(patch.object(flash, "repo_root_from_this_file", return_value=self.root))
        self.contexts.enter_context(patch.object(flash, "find_cube_cli", return_value="fake-cli"))
        self.contexts.enter_context(patch.object(
            flash, "cube_cli_runtime", side_effect=lambda cli: contextlib.nullcontext(cli)))
        self.contexts.enter_context(patch.dict(flash.os.environ, {}, clear=True))

    def invoke(self, *options):
        with patch.object(flash.sys, "argv", ["flash_stlink", "--image", str(self.image), *options]):
            return flash.main()

    @staticmethod
    def log_for(command):
        return Path(command[command.index("-log") + 1])

    def test_default_frequency_and_write_verify_reset_contract(self):
        self.assertEqual(self.invoke(), 0)
        self.command.assert_called_once()
        command = self.command.call_args.args[0]
        self.assertEqual(command[3:], [
            "-c", "port=SWD", "freq=1000", "mode=NORMAL", "reset=SWrst",
            "-w", str(self.image), "0x08000000", "-v", "-rst"])
        self.assertIn("SUCCESS", self.stdout.getvalue())

    def test_explicit_options_and_image_preserved(self):
        self.invoke("--freq", "4000", "--sn", "chosen-probe", "--no-reset", "--no-verify")
        command = self.command.call_args.args[0]
        self.assertEqual(command[3:], [
            "-c", "port=SWD", "freq=4000", "mode=NORMAL", "reset=SWrst",
            "sn=chosen-probe", "-w", str(self.image), "0x08000000"])

    def test_under_reset_parameters_preserved(self):
        self.invoke("--freq", "400", "--mode", "UR", "--reset-mode", "HWrst")
        command = self.command.call_args.args[0]
        for option in ("freq=400", "mode=UR", "reset=HWrst"):
            self.assertIn(option, command)

    def test_erase_failure_keeps_diagnostics_and_never_retries(self):
        def fail(command, cwd):
            self.log_for(command).write_text(
                "ST-LINK FW: V2J43M28\nVoltage: 3.24V\nErasing sector 6\n"
                "Error: failed to erase memory\n", encoding="utf-8")
            raise subprocess.CalledProcessError(7, command)
        self.command.side_effect = fail
        with self.assertRaises(subprocess.CalledProcessError) as caught:
            self.invoke()
        self.assertEqual(caught.exception.returncode, 7)
        self.command.assert_called_once()
        log = self.log_for(caught.exception.cmd).read_text()
        self.assertIn("Erasing sector 6", log)
        self.assertIn("failed to erase memory", log)
        self.assertIn("[flash_stlink] command:", log)
        self.assertIn("exit code: 7", log)
        self.assertNotIn("SUCCESS", self.stdout.getvalue())
        self.assertNotIn("-e", caught.exception.cmd)

    def test_error_with_zero_cli_exit_is_not_success(self):
        self.command.side_effect = lambda command, cwd: self.log_for(command).write_text(
            "12:34:56 : Error: failed to erase memory\n", encoding="utf-8")
        with self.assertRaises(subprocess.CalledProcessError) as caught:
            self.invoke()
        self.assertEqual(caught.exception.returncode, 1)
        self.command.assert_called_once()
        self.assertNotIn("SUCCESS", self.stdout.getvalue())

    def test_successful_log_and_unique_paths(self):
        self.command.side_effect = lambda command, cwd: self.log_for(command).write_text(
            "Download verified successfully\n", encoding="utf-8")
        self.invoke()
        self.invoke()
        paths = [self.log_for(call.args[0]) for call in self.command.call_args_list]
        self.assertNotEqual(*paths)
        for path in paths:
            self.assertIn("Download verified successfully", path.read_text())
            self.assertIn("exit code: 0", path.read_text())

    def test_start_failure_does_not_record_success_or_retry(self):
        self.command.side_effect = FileNotFoundError("missing CLI")
        with self.assertRaises(FileNotFoundError):
            self.invoke()
        self.command.assert_called_once()
        log = self.log_for(self.command.call_args.args[0]).read_text()
        self.assertIn("not completed", log)
        self.assertNotIn("SUCCESS", self.stdout.getvalue())

    def test_reset_only_logs_but_does_not_write(self):
        self.invoke("--reset-only")
        command = self.command.call_args.args[0]
        self.assertEqual(command[3:], [
            "-c", "port=SWD", "freq=1000", "mode=NORMAL", "reset=SWrst", "-rst"])

    def test_build_failure_prevents_programming(self):
        self.command.side_effect = subprocess.CalledProcessError(2, ["cmake"])
        with self.assertRaises(subprocess.CalledProcessError):
            self.invoke("--build")
        self.command.assert_called_once_with(
            ["cmake", "--build", "--preset", "Debug-stm32f413"], cwd=self.root)
        self.assertFalse((self.root / "build/flashing_logs").exists())


if __name__ == "__main__":
    unittest.main()
