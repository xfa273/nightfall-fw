#!/usr/bin/env python3
"""Tests for USB-preferred Pixel HFR transfer selection."""

from __future__ import annotations

import contextlib
import subprocess
import sys
import unittest
from pathlib import Path
from unittest import mock


TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR / "android_camera_probe"))

import collect_wifi_runs as wifi  # noqa: E402
import transfer_runs as transfer  # noqa: E402


def client(host: str = "192.0.2.10") -> wifi.ApiClient:
    endpoint = wifi.Endpoint(
        host=host,
        port=46052,
        info={
            "schema": wifi.SCHEMA,
            "device_id": "device-1",
            "model": "Pixel 8",
        },
    )
    return wifi.ApiClient(endpoint, "token")


class TransferRunsTest(unittest.TestCase):
    def test_authorized_adb_serials_ignores_unauthorized_devices(self) -> None:
        result = subprocess.CompletedProcess(
            ["adb", "devices", "-l"],
            0,
            "List of devices attached\n"
            "PIXEL device usb:2097152X model:Pixel_8\n"
            "192.0.2.3:5555 device product:shiba model:Pixel_8\n"
            "OTHER unauthorized usb:1\n",
            "",
        )
        with mock.patch.object(transfer.subprocess, "run", return_value=result):
            self.assertEqual(transfer.authorized_adb_serials(), ["PIXEL"])

    def test_auto_prefers_usb_forward_and_reports_transport(self) -> None:
        wifi_client = client()
        usb_client = client("127.0.0.1")

        @contextlib.contextmanager
        def fake_forward(_client: wifi.ApiClient, _serial: str | None):
            yield usb_client

        with (
            mock.patch.object(transfer, "adb_forward_client", fake_forward),
            mock.patch.object(transfer, "collect_all", return_value=(2, 4)) as collect,
        ):
            result = transfer.collect_preferred(
                wifi_client,
                Path("runs"),
                adb_serial="PIXEL",
            )
        self.assertEqual(result, (2, 4, transfer.TRANSFER_USB))
        collect.assert_called_once_with(usb_client, Path("runs"))

    def test_auto_falls_back_to_wifi_only_when_usb_is_unavailable(self) -> None:
        wifi_client = client()

        @contextlib.contextmanager
        def unavailable(_client: wifi.ApiClient, _serial: str | None):
            raise transfer.UsbTransportUnavailable("not connected")
            yield  # pragma: no cover

        with (
            mock.patch.object(transfer, "adb_forward_client", unavailable),
            mock.patch.object(transfer, "collect_all", return_value=(1, 0)) as collect,
        ):
            result = transfer.collect_preferred(wifi_client, Path("runs"))
        self.assertEqual(result, (1, 0, transfer.TRANSFER_WIFI))
        collect.assert_called_once_with(wifi_client, Path("runs"))

    def test_explicit_usb_does_not_silently_fall_back(self) -> None:
        @contextlib.contextmanager
        def unavailable(_client: wifi.ApiClient, _serial: str | None):
            raise transfer.UsbTransportUnavailable("not connected")
            yield  # pragma: no cover

        with mock.patch.object(transfer, "adb_forward_client", unavailable):
            with self.assertRaisesRegex(
                transfer.UsbTransportUnavailable,
                "not connected",
            ):
                transfer.collect_preferred(
                    client(),
                    Path("runs"),
                    transport=transfer.TRANSFER_USB,
                )

    def test_wifi_mode_does_not_probe_adb(self) -> None:
        wifi_client = client()
        with (
            mock.patch.object(transfer, "adb_forward_client") as forward,
            mock.patch.object(transfer, "collect_all", return_value=(0, 3)),
        ):
            result = transfer.collect_preferred(
                wifi_client,
                Path("runs"),
                transport=transfer.TRANSFER_WIFI,
            )
        self.assertEqual(result, (0, 3, transfer.TRANSFER_WIFI))
        forward.assert_not_called()


if __name__ == "__main__":
    unittest.main()
