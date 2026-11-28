"""Flash failure regression tests; no vendor executable or hardware is used."""

import contextlib
import ctypes
import importlib.machinery
import importlib.util
import io
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch


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


class ProbeRecoveryTests(unittest.TestCase):
    def setUp(self):
        ctx = contextlib.ExitStack()
        self.addCleanup(ctx.close)
        self.capture = ctx.enter_context(patch.object(flash, "run_capture"))
        self.reset = ctx.enter_context(patch.object(flash, "reset_stlink_v3_minie_usb", return_value="PROBE1"))
        self.program = ctx.enter_context(patch.object(flash, "run_programmer"))
        ctx.enter_context(patch.object(flash.time, "sleep"))
        ctx.enter_context(contextlib.redirect_stdout(io.StringIO()))
        self.stall = subprocess.CompletedProcess([], 0, "DEV_USB_COMM_ERR")
        self.ready = subprocess.CompletedProcess([], 0, "ST-LINK SN  : PROBE1\n")

    def test_list_and_normal_operations_stop_on_stall(self):
        self.capture.return_value = self.stall
        for list_only, reset_only in ((True, False), (False, True), (False, False)):
            with self.subTest(list=list_only, reset=reset_only):
                args = flash.argparse.Namespace(list=list_only, reset_only=reset_only, recover_usb=False,
                                                build=False, image=__file__, sn="PROBE1")
                with self.assertRaisesRegex(RuntimeError, "USB通信エラー"):
                    flash.execute(args, "cli", Path.cwd())
        self.reset.assert_not_called()
        self.program.assert_not_called()

    def test_explicit_recovery_only_rechecks_the_selected_probe(self):
        other = subprocess.CompletedProcess([], 0, "ST-LINK SN : PROBE10\n")
        self.capture.side_effect = [self.stall, other, self.ready]
        with patch.object(flash.sys, "platform", "darwin"):
            args = flash.argparse.Namespace(recover_usb=True, sn="PROBE1")
            self.assertEqual(flash.execute(args, "cli", Path.cwd()), 0)
        self.reset.assert_called_once_with("cli", "PROBE1", require_single=True)
        self.program.assert_not_called()
        self.assertEqual(self.capture.call_count, 3)

    def test_missing_serial_after_reset_fails_after_three_checks(self):
        self.capture.side_effect = [self.stall] + [subprocess.CompletedProcess([], 0, "no probes")] * 3
        with self.assertRaisesRegex(RuntimeError, "認識できません"):
            flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.reset.assert_called_once()

    def test_healthy_probe_needs_no_reset(self):
        self.capture.return_value = self.ready
        flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        with self.assertRaises(RuntimeError):
            flash.ensure_stlink_ready("cli", "PROBE2", Path.cwd(), recover=True)
        self.reset.assert_not_called()

    @staticmethod
    def probe(index, serial, *, stalled=False):
        # CubeProgrammer 2.21.0 writes these SGR sequences even to a pipe.
        return (f"ST-Link Probe {index} :\r\n"
                f"\x1b[39;49m\x1b[0m\x1b[90m   ST-LINK SN  : {serial}\r\n"
                "\x1b[39;49m\x1b[0m\x1b[90m   Board Name  : STLINK-V3MINIE\r\n"
                + ("Error: ST-LINK error (DEV_USB_COMM_ERR)\r\n" if stalled else ""))

    def test_colored_listing_recognizes_a_healthy_probe_without_reset(self):
        self.capture.return_value = subprocess.CompletedProcess([], 0, self.probe(0, "PROBE1"))
        flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.reset.assert_not_called()

    def test_cube_dash_serial_during_usb_failure_requires_one_physical_probe(self):
        # CubeProgrammer 2.21 prints this placeholder on the failing Mac-direct V3 path.
        listing = "ST-LINK error (DEV_USB_COMM_ERR)\n" + self.probe(0, "-")
        failed = subprocess.CompletedProcess([], 0, listing)
        self.capture.side_effect = [failed, failed, self.ready]
        flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.reset.assert_called_once_with("cli", "PROBE1", require_single=True)
        self.assertEqual(self.capture.call_count, 3)

    def test_missing_serial_is_never_a_healthy_probe(self):
        for listing in (self.probe(0, "-"), "ST-LINK SN : -\n", "ST-LINK SN : \nST-LINK SN : PROBE1\n"):
            with self.subTest(listing=listing):
                self.capture.return_value = subprocess.CompletedProcess([], 0, listing)
                with self.assertRaises(RuntimeError):
                    flash.ensure_stlink_ready("cli", None, Path.cwd(), recover=True)
        self.reset.assert_not_called()

    def test_healthy_list_with_multiple_probes_and_no_serial_needs_no_reset(self):
        self.capture.return_value = subprocess.CompletedProcess([], 0, self.probe(0, "A") + self.probe(1, "B"))
        flash.ensure_stlink_ready("cli", None, Path.cwd(), recover=True)
        self.reset.assert_not_called()

    def test_peer_stall_does_not_reset_healthy_selected_probe_in_either_order(self):
        for listing in (self.probe(0, "PROBE1") + self.probe(1, "PROBE10", stalled=True),
                        self.probe(0, "PROBE10", stalled=True) + self.probe(1, "PROBE1")):
            with self.subTest(listing=listing):
                self.capture.return_value = subprocess.CompletedProcess([], 0, listing)
                flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.reset.assert_not_called()

    def test_reenumeration_confirms_only_the_reset_serial_even_while_peer_stays_stalled(self):
        peer = self.probe(0, "PROBE10", stalled=True)
        both_stalled = subprocess.CompletedProcess([], 0, peer + self.probe(1, "PROBE1", stalled=True))
        recovered = subprocess.CompletedProcess([], 0, peer + self.probe(1, "PROBE1"))
        self.capture.side_effect = [both_stalled, both_stalled, recovered]
        flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.assertEqual(self.capture.call_count, 3)
        self.reset.assert_called_once_with("cli", "PROBE1", require_single=False)
        self.program.assert_not_called()

    def test_ambiguous_listings_never_reset_a_probe(self):
        healthy = self.probe(0, "PROBE1")
        other = self.probe(1, "PROBE10")
        error = "libusb: error: pipe is stalled\r\n"
        for listing, requested in (
                (healthy + self.probe(1, "PROBE10", stalled=True), None),
                (healthy + self.probe(1, "PROBE10", stalled=True), "ABSENT"),
                (healthy + self.probe(1, "PROBE1", stalled=True), "PROBE1"),
                (error + healthy + other, "PROBE1"),
                (error + healthy + self.probe(1, "-"), "ABSENT"),
                (healthy + error + other, "PROBE1"),
                (healthy + other + "-------------------\r\nError: ST-LINK error (DEV_USB_COMM_ERR)\r\n", "PROBE10"),
                (healthy + self.probe(1, "", stalled=True), "ABSENT")):
            with self.subTest(listing=listing, requested=requested):
                self.capture.return_value = subprocess.CompletedProcess([], 0, listing)
                with self.assertRaises(RuntimeError):
                    flash.ensure_stlink_ready("cli", requested, Path.cwd(), recover=True)
                self.reset.assert_not_called()

    def test_single_stalled_record_binds_reset_to_its_serial_without_sn_option(self):
        self.capture.side_effect = [subprocess.CompletedProcess([], 0, self.probe(0, "PROBE1", stalled=True)),
                                    self.ready]
        flash.ensure_stlink_ready("cli", None, Path.cwd(), recover=True)
        self.reset.assert_called_once_with("cli", "PROBE1", require_single=True)

    def test_reenumeration_requires_successful_cli_exit(self):
        failed = subprocess.CompletedProcess([], 1, self.probe(0, "PROBE1"))
        self.capture.side_effect = [self.stall, failed, self.ready]
        flash.ensure_stlink_ready("cli", "PROBE1", Path.cwd(), recover=True)
        self.assertEqual(self.capture.call_count, 3)
        self.reset.assert_called_once_with("cli", "PROBE1", require_single=True)

    def test_recovery_rejects_combined_operations_before_opening_the_cli(self):
        for options in (["--build"], ["--reset-only"], ["--list"], ["--image", "fw.bin"]):
            with self.subTest(options=options), patch.object(flash.sys, "argv", ["flash_stlink", "--recover-usb", *options]), \
                    patch.object(flash, "find_cube_cli") as find, contextlib.redirect_stderr(io.StringIO()), \
                    self.assertRaises(SystemExit) as caught:
                flash.main()
            self.assertEqual(caught.exception.code, 2)
            find.assert_not_called()
        self.reset.assert_not_called()

    def test_ambiguous_and_unreadable_serials_are_rejected(self):
        probes = [("A", 1), ("B", 2)]
        self.assertEqual(flash.select_recovery_probe(probes, "B"), ("B", 2))
        self.assertEqual(flash.select_recovery_probe(probes[:1], None), ("A", 1))
        for matches, serial in ((probes, None), (probes, "C"), ([("", 1)], "A"),
                                ([("", None), ("A", 2)], None), ([], None)):
            with self.subTest(matches=matches, serial=serial), self.assertRaises(RuntimeError):
                flash.select_recovery_probe(matches, serial)


class UsbInventoryTests(unittest.TestCase):
    def test_reset_selection_checks_the_physical_usb_inventory(self):
        lib = Mock()
        devices = (ctypes.c_void_p * 2)(1, 2)
        device_list = ctypes.POINTER(ctypes.c_void_p)

        def list_devices(context, out):
            ctypes.cast(out, ctypes.POINTER(device_list))[0] = ctypes.cast(devices, device_list)
            return len(devices)

        def descriptor(device, out):
            if device == 2 and peer_pid is None:
                return -1
            desc = ctypes.cast(out, ctypes.POINTER(flash.LibusbDeviceDescriptor)).contents
            desc.idVendor = flash.STLINK_USB_VID
            desc.idProduct = flash.STLINK_V3_MINIE_PID if device == 1 else peer_pid
            desc.iSerialNumber = 1
            return 0

        def open_device(device, out):
            ctypes.cast(out, ctypes.POINTER(ctypes.c_void_p))[0] = device
            return 0

        def read_serial(handle, index, buf, size):
            serial = f"PROBE{handle.value}".encode("ascii")
            buf[:len(serial)] = serial
            return len(serial)

        lib.libusb_init.return_value = 0
        lib.libusb_get_device_list.side_effect = list_devices
        lib.libusb_get_device_descriptor.side_effect = descriptor
        lib.libusb_open.side_effect = open_device
        lib.libusb_get_string_descriptor_ascii.side_effect = read_serial
        lib.libusb_reset_device.return_value = 0
        with patch.object(flash.sys, "platform", "darwin"), patch.object(flash, "find_libusb", return_value="fake"), \
                patch.object(ctypes, "CDLL", return_value=lib):
            # A V2, a second MINIE, and an unreadable descriptor can hide the
            # owner of a global error. An unrelated STM32 DFU device cannot.
            for peer_pid, single, serial, expected_device in (
                    (0x3748, True, "PROBE1", None), (0x3754, True, "PROBE1", None),
                    (None, True, "PROBE1", None), (0xDF11, True, "PROBE1", 1),
                    (0xDF11, True, "ABSENT", None), (0x3754, False, "PROBE2", 2),
                    (0x3752, True, "PROBE1", None), (0x3752, False, "PROBE2", None)):
                with self.subTest(peer=peer_pid, single=single, serial=serial):
                    lib.libusb_reset_device.reset_mock()
                    if expected_device is None:
                        with self.assertRaises(RuntimeError):
                            flash.reset_stlink_v3_minie_usb("cli", serial, require_single=single)
                        lib.libusb_reset_device.assert_not_called()
                    else:
                        self.assertEqual(flash.reset_stlink_v3_minie_usb("cli", serial, require_single=single), serial)
                        lib.libusb_reset_device.assert_called_once()
                        self.assertEqual(lib.libusb_reset_device.call_args.args[0].value, expected_device)


if __name__ == "__main__":
    unittest.main()
