#!/usr/bin/env python3
"""Control Pixel HFR standby and collect runs over authenticated Wi-Fi."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any

from collect_wifi_runs import (
    ApiClient,
    CaptureBusyError,
    DEFAULT_OUTPUT_ROOT,
    WifiCollectorError,
    choose_endpoint,
    collect_all,
    configured_token,
    default_config_path,
    load_config,
)


CONTROL_STATUS_PATH = "/api/v1/control/status"
CONTROL_START_PATH = "/api/v1/control/standby/start"
CONTROL_STOP_PATH = "/api/v1/control/standby/stop"


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Start/stop Pixel 240 fps continuous optical standby, inspect its "
            "state, and collect completed runs over authenticated Wi-Fi."
        ),
    )
    parser.add_argument(
        "action",
        choices=("status", "start", "stop", "collect", "stop-and-collect"),
    )
    parser.add_argument(
        "output_root",
        nargs="?",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="collect/stop-and-collectの保存先",
    )
    parser.add_argument("--host", help="自動発見できない場合のPixel IP[:port]")
    parser.add_argument("--device-id", help="複数台ある場合の保存済みdevice ID")
    parser.add_argument(
        "--config",
        type=Path,
        default=default_config_path(),
        help="認証情報の保存先",
    )
    parser.add_argument("--discover-seconds", type=float, default=2.5)
    parser.add_argument(
        "--wait-seconds",
        type=float,
        default=90.0,
        help="開始完了または安全な停止を待つ最大秒数",
    )
    return parser.parse_intermixed_args(argv)


def capture_state(response: dict[str, Any]) -> dict[str, Any]:
    value = response.get("capture_control")
    if not isinstance(value, dict) or not isinstance(value.get("state"), str):
        raise WifiCollectorError(
            "Pixel appがMac撮影制御APIに未対応です。"
            "Nightfall HFR Recorder 0.5.0以降へ更新してください"
        )
    return value


def print_status(response: dict[str, Any], host: str, port: int) -> None:
    state = capture_state(response)
    print(
        "[HFR-CONTROL] "
        f"Pixel={response.get('model', 'unknown')} {host}:{port} "
        f"app={response.get('app_version', 'unknown')}"
    )
    print(
        "[HFR-CONTROL] "
        f"state={state['state']} "
        f"standby={bool(state.get('continuous_standby'))} "
        f"recording={bool(state.get('recording'))} "
        f"completed={int(state.get('completed_runs', 0))} "
        f"saved={int(response.get('saved_runs', 0))} "
        f"acknowledged={int(response.get('acknowledged_runs', 0))} "
        f"transfers={int(response.get('active_transfers', 0))}"
    )
    message = state.get("message")
    if isinstance(message, str) and message:
        print(f"[HFR-CONTROL] {message}")


def read_status(client: ApiClient) -> dict[str, Any]:
    return client.json_request("GET", CONTROL_STATUS_PATH)


def wait_for_start(client: ApiClient, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        response = read_status(client)
        state = capture_state(response)["state"]
        if state in {"armed", "recording"}:
            return response
        if state in {"idle", "error"}:
            raise WifiCollectorError(
                "撮影スタンバイ開始に失敗しました: "
                + str(capture_state(response).get("message", state))
            )
        if time.monotonic() >= deadline:
            raise WifiCollectorError(
                f"撮影スタンバイ開始待ちが{timeout:g}秒でタイムアウトしました"
            )
        time.sleep(0.25)


def wait_for_stop(client: ApiClient, timeout: float) -> dict[str, Any]:
    deadline = time.monotonic() + max(0.0, timeout)
    while True:
        response = read_status(client)
        state = capture_state(response)
        if state["state"] == "error":
            raise WifiCollectorError(
                "撮影停止中にエラーが発生しました: "
                + str(state.get("message", "unknown error"))
            )
        if state["state"] == "idle" and not response.get("capture_busy"):
            return response
        if time.monotonic() >= deadline:
            raise WifiCollectorError(
                f"撮影スタンバイ停止待ちが{timeout:g}秒でタイムアウトしました"
            )
        time.sleep(0.25)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(args.config)
        endpoint = choose_endpoint(args, config)
        token = configured_token(endpoint, config, args.config)
        client = ApiClient(endpoint, token)

        if args.action == "status":
            response = read_status(client)
            print_status(response, endpoint.host, endpoint.port)
            return 0

        if args.action == "start":
            client.json_request("POST", CONTROL_START_PATH, {})
            response = wait_for_start(client, args.wait_seconds)
            print_status(response, endpoint.host, endpoint.port)
            print("[HFR-CONTROL] 連続撮影スタンバイを開始しました")
            return 0

        if args.action in {"stop", "stop-and-collect"}:
            client.json_request("POST", CONTROL_STOP_PATH, {})
            response = wait_for_stop(client, args.wait_seconds)
            print_status(response, endpoint.host, endpoint.port)
            print("[HFR-CONTROL] 連続撮影スタンバイを終了しました")
            if args.action == "stop":
                return 0

        if args.action in {"collect", "stop-and-collect"}:
            response = read_status(client)
            if response.get("capture_busy"):
                raise CaptureBusyError(
                    "Pixelは撮影待機中です。先にhfr_control.py stopを実行するか、"
                    "stop-and-collectを使用してください"
                )
            collected, skipped = collect_all(client, args.output_root)
            print(
                f"[HFR-CONTROL] Transfer complete: collected={collected} "
                f"skipped={skipped} output={args.output_root}"
            )
            return 0

        raise AssertionError(f"unhandled action: {args.action}")
    except CaptureBusyError as exc:
        print(f"[HFR-CONTROL][BUSY] {exc}", file=sys.stderr)
        return 3
    except WifiCollectorError as exc:
        print(f"[HFR-CONTROL][ERROR] {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("[HFR-CONTROL] Interrupted.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
