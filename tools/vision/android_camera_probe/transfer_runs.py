#!/usr/bin/env python3
"""Collect Pixel HFR runs over USB when available, with Wi-Fi fallback."""

from __future__ import annotations

import contextlib
import subprocess
from collections.abc import Iterator
from pathlib import Path

from collect_wifi_runs import (
    ApiClient,
    Endpoint,
    WifiCollectorError,
    collect_all,
    fetch_info,
)


TRANSFER_AUTO = "auto"
TRANSFER_USB = "usb"
TRANSFER_WIFI = "wifi"
TRANSFER_CHOICES = (TRANSFER_AUTO, TRANSFER_USB, TRANSFER_WIFI)


class UsbTransportUnavailable(WifiCollectorError):
    """ADB is unavailable or cannot reach the selected Pixel."""


def authorized_adb_serials() -> list[str]:
    """Return only devices for which ADB debugging is authorized."""
    try:
        result = subprocess.run(
            ["adb", "devices", "-l"],
            capture_output=True,
            text=True,
            check=False,
            timeout=5.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise UsbTransportUnavailable(f"ADBを実行できません: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise UsbTransportUnavailable(f"adb devicesに失敗しました: {detail}")
    serials: list[str] = []
    for line in result.stdout.splitlines()[1:]:
        fields = line.split()
        if (
            len(fields) >= 3
            and fields[1] == "device"
            and any(field.startswith("usb:") for field in fields[2:])
        ):
            serials.append(fields[0])
    return serials


def select_adb_serial(requested: str | None) -> str:
    serials = authorized_adb_serials()
    if requested:
        if requested not in serials:
            raise UsbTransportUnavailable(
                f"指定したADB端末 {requested} がUSBデバッグ接続されていません"
            )
        return requested
    if not serials:
        raise UsbTransportUnavailable("USBデバッグ接続された端末がありません")
    if len(serials) > 1:
        raise UsbTransportUnavailable(
            "USBデバッグ端末が複数あります。--adb-serialでPixelを指定してください"
        )
    return serials[0]


@contextlib.contextmanager
def adb_forward_client(
    wifi_client: ApiClient,
    adb_serial: str | None = None,
) -> Iterator[ApiClient]:
    """Forward the Pixel HTTP API through ADB and verify device identity."""
    serial = select_adb_serial(adb_serial)
    try:
        result = subprocess.run(
            [
                "adb",
                "-s",
                serial,
                "forward",
                "tcp:0",
                f"tcp:{wifi_client.endpoint.port}",
            ],
            capture_output=True,
            text=True,
            check=False,
            timeout=5.0,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise UsbTransportUnavailable(f"ADB転送の開始に失敗しました: {exc}") from exc
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise UsbTransportUnavailable(f"ADB転送の開始に失敗しました: {detail}")
    try:
        local_port = int(result.stdout.strip())
    except ValueError as exc:
        raise UsbTransportUnavailable(
            f"ADBが不正な転送ポートを返しました: {result.stdout!r}"
        ) from exc

    try:
        try:
            forwarded = fetch_info("127.0.0.1", local_port, timeout=3.0)
        except WifiCollectorError as exc:
            raise UsbTransportUnavailable(
                f"USBトンネル経由でPixel APIへ接続できません: {exc}"
            ) from exc
        if forwarded.device_id != wifi_client.endpoint.device_id:
            raise UsbTransportUnavailable(
                "USB接続されたPixelとWi-Fiで選択したPixelが一致しません"
            )
        endpoint = Endpoint(
            host="127.0.0.1",
            port=local_port,
            info=forwarded.info,
        )
        yield ApiClient(endpoint, wifi_client.token, timeout=wifi_client.timeout)
    finally:
        try:
            subprocess.run(
                [
                    "adb",
                    "-s",
                    serial,
                    "forward",
                    "--remove",
                    f"tcp:{local_port}",
                ],
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
        except (OSError, subprocess.TimeoutExpired):
            pass


def collect_preferred(
    wifi_client: ApiClient,
    output_root: Path,
    *,
    transport: str = TRANSFER_AUTO,
    adb_serial: str | None = None,
) -> tuple[int, int, str]:
    """Collect via ADB-forwarded HTTP when possible, preserving ACK semantics."""
    if transport not in TRANSFER_CHOICES:
        raise ValueError(f"unsupported transfer transport: {transport}")
    if transport == TRANSFER_WIFI:
        collected, skipped = collect_all(wifi_client, output_root)
        return collected, skipped, TRANSFER_WIFI

    try:
        with adb_forward_client(wifi_client, adb_serial) as usb_client:
            collected, skipped = collect_all(usb_client, output_root)
            return collected, skipped, TRANSFER_USB
    except UsbTransportUnavailable:
        if transport == TRANSFER_USB:
            raise

    collected, skipped = collect_all(wifi_client, output_root)
    return collected, skipped, TRANSFER_WIFI
