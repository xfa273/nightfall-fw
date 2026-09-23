#!/usr/bin/env python3
"""Discover, pair with, and collect retained Pixel HFR runs over Wi-Fi."""

from __future__ import annotations

import argparse
import http.client
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any
from urllib.parse import quote, urlparse


SCHEMA = "nightfall_hfr_wifi_v1"
CONFIG_SCHEMA = "nightfall_hfr_wifi_client_v1"
REPORT_SCHEMA = "nightfall_android_hfr_recording_v1"
DISCOVERY_REQUEST = b"NIGHTFALL_HFR_DISCOVER_V1\n"
DISCOVERY_PORT = 46051
DEFAULT_HTTP_PORT = 46052
DEFAULT_OUTPUT_ROOT = Path("sessions/hfr-tests/pixel8/manual-runs")
SAFE_NAME = re.compile(r"^[A-Za-z0-9._-]{1,160}$")
COMPLETE_ARTIFACTS = {
    "hfr_report.json",
    "capture_results.jsonl",
    "encoder_samples.jsonl",
    "hfr_capture.mp4",
}


class WifiCollectorError(RuntimeError):
    """A user-facing collection error."""


class CaptureBusyError(WifiCollectorError):
    """The Pixel is deliberately refusing file I/O during capture."""


@dataclass(frozen=True)
class Endpoint:
    host: str
    port: int
    info: dict[str, Any]

    @property
    def device_id(self) -> str:
        return require_text(self.info, "device_id")

    @property
    def model(self) -> str:
        return require_text(self.info, "model")


class ApiClient:
    def __init__(
        self,
        endpoint: Endpoint,
        token: str | None = None,
        timeout: float = 60.0,
    ) -> None:
        self.endpoint = endpoint
        self.token = token
        self.timeout = timeout

    def json_request(
        self,
        method: str,
        path: str,
        body: dict[str, Any] | None = None,
        *,
        authorized: bool = True,
    ) -> dict[str, Any]:
        payload = None
        headers = {"Accept": "application/json"}
        if body is not None:
            payload = json.dumps(body, separators=(",", ":")).encode("utf-8")
            headers["Content-Type"] = "application/json"
            headers["Content-Length"] = str(len(payload))
        if authorized:
            headers["Authorization"] = self.authorization()
        connection = http.client.HTTPConnection(
            self.endpoint.host,
            self.endpoint.port,
            timeout=self.timeout,
        )
        try:
            connection.request(method, path, body=payload, headers=headers)
            response = connection.getresponse()
            raw = response.read()
        except OSError as exc:
            raise WifiCollectorError(
                f"Pixel {self.endpoint.host}:{self.endpoint.port}への接続失敗: {exc}"
            ) from exc
        finally:
            connection.close()
        parsed: dict[str, Any]
        try:
            parsed = json.loads(raw.decode("utf-8")) if raw else {}
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise WifiCollectorError(
                f"Pixelから不正なJSON応答を受信しました (HTTP {response.status})"
            ) from exc
        if response.status == 503:
            raise CaptureBusyError(parsed.get("error", "capture is active"))
        if response.status < 200 or response.status >= 300:
            raise WifiCollectorError(
                f"Pixel API HTTP {response.status}: "
                f"{parsed.get('error', response.reason)}"
            )
        validate_schema(parsed)
        return parsed

    def download(
        self,
        path: str,
        destination: Path,
        expected_size: int,
        label: str,
    ) -> None:
        if destination.is_file() and destination.stat().st_size == expected_size:
            print(f"[HFR-WIFI] Already downloaded: {label}")
            return
        partial = destination.with_name(destination.name + ".part")
        offset = partial.stat().st_size if partial.is_file() else 0
        if offset > expected_size:
            partial.unlink()
            offset = 0
        if offset == expected_size:
            if not partial.is_file():
                partial.parent.mkdir(parents=True, exist_ok=True)
                with partial.open("wb") as output:
                    output.flush()
                    os.fsync(output.fileno())
            os.replace(partial, destination)
            print(f"[HFR-WIFI] Resumed complete file: {label}")
            return

        headers = {
            "Authorization": self.authorization(),
            "Accept": "application/octet-stream",
        }
        if offset:
            headers["Range"] = f"bytes={offset}-"
        connection = http.client.HTTPConnection(
            self.endpoint.host,
            self.endpoint.port,
            timeout=self.timeout,
        )
        try:
            connection.request("GET", path, headers=headers)
            response = connection.getresponse()
            if response.status == 503:
                raw = response.read()
                try:
                    detail = json.loads(raw.decode("utf-8")).get("error")
                except (UnicodeDecodeError, json.JSONDecodeError):
                    detail = None
                raise CaptureBusyError(detail or "capture is active")
            if response.status not in (200, 206):
                raw = response.read()
                try:
                    detail = json.loads(raw.decode("utf-8")).get("error")
                except (UnicodeDecodeError, json.JSONDecodeError):
                    detail = None
                raise WifiCollectorError(
                    f"{label}: HTTP {response.status} "
                    f"{detail or response.reason}"
                )
            mode = "ab" if offset and response.status == 206 else "wb"
            if response.status == 200:
                offset = 0
            partial.parent.mkdir(parents=True, exist_ok=True)
            received = offset
            next_progress = received + 64 * 1024 * 1024
            with partial.open(mode) as output:
                while True:
                    chunk = response.read(1024 * 1024)
                    if not chunk:
                        break
                    output.write(chunk)
                    received += len(chunk)
                    if received >= next_progress:
                        print(
                            f"[HFR-WIFI] {label}: "
                            f"{received / (1024 * 1024):.1f}/"
                            f"{expected_size / (1024 * 1024):.1f} MiB"
                        )
                        next_progress = received + 64 * 1024 * 1024
                output.flush()
                os.fsync(output.fileno())
        except (OSError, http.client.HTTPException) as exc:
            raise WifiCollectorError(f"{label}の転送が中断しました: {exc}") from exc
        finally:
            connection.close()
        actual_size = partial.stat().st_size
        if actual_size != expected_size:
            raise WifiCollectorError(
                f"{label}のサイズ不一致: expected={expected_size}, actual={actual_size}; "
                "次回実行時に途中から再開します"
            )
        os.replace(partial, destination)
        print(
            f"[HFR-WIFI] Downloaded: {label} "
            f"({expected_size / (1024 * 1024):.1f} MiB)"
        )

    def authorization(self) -> str:
        if not self.token:
            raise WifiCollectorError("Pixelとのペア設定が必要です")
        return f"Bearer {self.token}"


def require_text(value: dict[str, Any], key: str) -> str:
    item = value.get(key)
    if not isinstance(item, str) or not item:
        raise WifiCollectorError(f"Pixel応答に有効な{key}がありません")
    return item


def validate_schema(value: dict[str, Any]) -> None:
    if value.get("schema") != SCHEMA:
        raise WifiCollectorError(
            f"未対応のPixel Wi-Fi schema: {value.get('schema')!r}"
        )


def default_config_path() -> Path:
    override = os.environ.get("NIGHTFALL_HFR_CONFIG")
    if override:
        return Path(override).expanduser()
    return Path.home() / ".config" / "nightfall-hfr" / "wifi_devices.json"


def load_config(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"schema": CONFIG_SCHEMA, "devices": {}}
    try:
        result = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WifiCollectorError(f"設定ファイルを読めません: {path}: {exc}") from exc
    if result.get("schema") != CONFIG_SCHEMA:
        raise WifiCollectorError(f"未対応の設定ファイルです: {path}")
    if not isinstance(result.get("devices"), dict):
        raise WifiCollectorError(f"設定ファイルのdevicesが不正です: {path}")
    return result


def save_config(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    os.chmod(temporary, 0o600)
    os.replace(temporary, path)
    os.chmod(path, 0o600)


def parse_host(value: str, default_port: int = DEFAULT_HTTP_PORT) -> tuple[str, int]:
    parsed = urlparse(value if "://" in value else f"http://{value}")
    if parsed.scheme != "http" or not parsed.hostname:
        raise WifiCollectorError(f"無効なPixelアドレスです: {value}")
    try:
        port = parsed.port or default_port
    except ValueError as exc:
        raise WifiCollectorError(f"無効なPixelポートです: {value}") from exc
    return parsed.hostname, port


def fetch_info(host: str, port: int, timeout: float = 5.0) -> Endpoint:
    endpoint = Endpoint(host, port, {})
    info = ApiClient(endpoint, timeout=timeout).json_request(
        "GET",
        "/api/v1/info",
        authorized=False,
    )
    reported_port = info.get("http_port")
    if not isinstance(reported_port, int) or not (1 <= reported_port <= 65535):
        raise WifiCollectorError("Pixel応答のhttp_portが不正です")
    return Endpoint(host, reported_port, info)


def discover_devices(duration: float) -> list[Endpoint]:
    deadline = time.monotonic() + max(0.2, duration)
    found: dict[str, Endpoint] = {}
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        sock.bind(("", 0))
        sock.settimeout(0.25)
        next_send = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_send:
                try:
                    sock.sendto(
                        DISCOVERY_REQUEST,
                        ("255.255.255.255", DISCOVERY_PORT),
                    )
                except OSError:
                    pass
                next_send = now + 0.5
            try:
                payload, address = sock.recvfrom(4096)
            except socket.timeout:
                continue
            try:
                info = json.loads(payload.decode("utf-8"))
                validate_schema(info)
                device_id = require_text(info, "device_id")
                port = info.get("http_port")
                if not isinstance(port, int) or not (1 <= port <= 65535):
                    continue
            except (UnicodeDecodeError, json.JSONDecodeError, WifiCollectorError):
                continue
            found[device_id] = Endpoint(address[0], port, info)
    return sorted(found.values(), key=lambda item: (item.model, item.device_id))


def choose_endpoint(
    args: argparse.Namespace,
    config: dict[str, Any],
) -> Endpoint:
    if args.host:
        host, port = parse_host(args.host)
        return fetch_info(host, port)

    discovered = discover_devices(args.discover_seconds)
    if args.device_id:
        discovered = [item for item in discovered if item.device_id == args.device_id]
    if len(discovered) == 1:
        return discovered[0]
    if len(discovered) > 1:
        descriptions = "\n".join(
            f"  {item.device_id}  {item.model}  {item.host}:{item.port}"
            for item in discovered
        )
        raise WifiCollectorError(
            "複数のNightfall HFR Recorderが見つかりました。"
            "--device-idで選択してください:\n" + descriptions
        )

    devices = config["devices"]
    candidates = []
    for device_id, saved in devices.items():
        if args.device_id and device_id != args.device_id:
            continue
        if not isinstance(saved, dict) or not saved.get("last_host"):
            continue
        candidates.append((device_id, saved))
    if len(candidates) == 1:
        _device_id, saved = candidates[0]
        return fetch_info(
            str(saved["last_host"]),
            int(saved.get("http_port", DEFAULT_HTTP_PORT)),
        )
    raise WifiCollectorError(
        "PixelをLAN上で発見できません。PixelとMacを同じWi-Fiへ接続し、"
        "Nightfall HFR Recorderを開いてください。必要なら--host IP:PORTを指定します"
    )


def pair_device(
    endpoint: Endpoint,
    code: str,
    config: dict[str, Any],
    config_path: Path,
) -> str:
    if not re.fullmatch(r"\d{6}", code):
        raise WifiCollectorError("ペアコードはPixelに表示された6桁の数字です")
    response = ApiClient(endpoint).json_request(
        "POST",
        "/api/v1/pair",
        {"code": code},
        authorized=False,
    )
    token = require_text(response, "access_token")
    config["devices"][endpoint.device_id] = {
        "model": endpoint.model,
        "token": token,
        "last_host": endpoint.host,
        "http_port": endpoint.port,
    }
    save_config(config_path, config)
    print(
        f"[HFR-WIFI] Paired: {endpoint.model} {endpoint.device_id} "
        f"({endpoint.host}:{endpoint.port})"
    )
    print(f"[HFR-WIFI] Credential saved: {config_path}")
    return token


def configured_token(
    endpoint: Endpoint,
    config: dict[str, Any],
    config_path: Path,
) -> str:
    saved = config["devices"].get(endpoint.device_id)
    if not isinstance(saved, dict) or not isinstance(saved.get("token"), str):
        raise WifiCollectorError(
            "このPixelはまだMacとペア設定されていません。Pixel上のペアコードを確認し、"
            f"次を実行してください:\n  {Path(sys.argv[0]).name} --pair 123456"
        )
    saved["last_host"] = endpoint.host
    saved["http_port"] = endpoint.port
    saved["model"] = endpoint.model
    save_config(config_path, config)
    return str(saved["token"])


def wait_until_idle(
    endpoint: Endpoint,
    timeout_seconds: int,
) -> Endpoint:
    deadline = time.monotonic() + timeout_seconds
    current = endpoint
    while current.info.get("capture_busy"):
        if timeout_seconds <= 0 or time.monotonic() >= deadline:
            raise CaptureBusyError(
                "Pixelは撮影待機中です。Pixelで「連続待機を終了」を押してから再実行するか、"
                "--wait-idle-secondsを指定してください"
            )
        remaining = max(0, int(deadline - time.monotonic()))
        print(f"[HFR-WIFI] Pixel撮影終了待ち... 残り最大{remaining}秒")
        time.sleep(2.0)
        refreshed = fetch_info(endpoint.host, endpoint.port)
        if refreshed.device_id != endpoint.device_id:
            raise WifiCollectorError(
                "待機中に接続先のdevice IDが変わりました。認証情報は送信していません"
            )
        current = refreshed
    return current


def safe_component(value: str) -> str:
    result = re.sub(r"[^A-Za-z0-9._-]", "_", value)
    return result or "unknown"


def read_report(path: Path, run_name: str) -> dict[str, Any]:
    try:
        report = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise WifiCollectorError(f"不正なrun reportです: {path}: {exc}") from exc
    if report.get("schema") != REPORT_SCHEMA:
        raise WifiCollectorError(f"未対応のrun report schemaです: {path}")
    if report.get("record_nonce") != run_name:
        raise WifiCollectorError(f"run名とreport nonceが一致しません: {run_name}")
    return report


def find_existing_run(output_root: Path, run_name: str) -> Path | None:
    if not output_root.is_dir():
        return None
    for candidate in output_root.iterdir():
        if not candidate.is_dir() or candidate.name.startswith("."):
            continue
        report_path = candidate / "hfr_report.json"
        if not report_path.is_file():
            continue
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if report.get("schema") == REPORT_SCHEMA \
                and report.get("record_nonce") == run_name:
            return candidate
    return None


def artifact_map(run: dict[str, Any]) -> dict[str, int]:
    result: dict[str, int] = {}
    artifacts = run.get("artifacts")
    if not isinstance(artifacts, list):
        raise WifiCollectorError(f"run {run.get('name')}のartifactsが不正です")
    for artifact in artifacts:
        if not isinstance(artifact, dict):
            continue
        name = artifact.get("name")
        size = artifact.get("size")
        if name in COMPLETE_ARTIFACTS and isinstance(size, int) and size >= 0:
            result[name] = size
    return result


def run_ffprobe(session_dir: Path) -> None:
    video = session_dir / "hfr_capture.mp4"
    if not video.is_file() or shutil.which("ffprobe") is None:
        return
    command = [
        "ffprobe",
        "-v",
        "error",
        "-count_frames",
        "-select_streams",
        "v:0",
        "-show_entries",
        "stream=codec_name,width,height,r_frame_rate,avg_frame_rate,duration,nb_read_frames",
        "-of",
        "json",
        str(video),
    ]
    result = subprocess.run(command, capture_output=True, text=True, check=False)
    if result.returncode != 0:
        raise WifiCollectorError(
            f"ffprobe検証に失敗しました: {result.stderr.strip()}"
        )
    try:
        parsed = json.loads(result.stdout)
    except json.JSONDecodeError as exc:
        raise WifiCollectorError("ffprobeが不正なJSONを返しました") from exc
    streams = parsed.get("streams")
    if not isinstance(streams, list) or len(streams) != 1:
        raise WifiCollectorError("動画に有効な映像ストリームがありません")
    (session_dir / "ffprobe_stream.json").write_text(
        json.dumps(parsed, indent=2) + "\n",
        encoding="utf-8",
    )


def collect_run(
    client: ApiClient,
    output_root: Path,
    run: dict[str, Any],
) -> tuple[str, Path]:
    run_name = require_text(run, "name")
    if not SAFE_NAME.fullmatch(run_name) or run_name in {".", ".."}:
        raise WifiCollectorError(f"Pixelが安全でないrun名を返しました: {run_name!r}")
    artifacts = artifact_map(run)
    if "hfr_report.json" not in artifacts:
        raise WifiCollectorError(f"run {run_name}にhfr_report.jsonがありません")
    existing = find_existing_run(output_root, run_name)
    if existing is not None:
        report = read_report(existing / "hfr_report.json", run_name)
        status = str(report.get("status", "missing"))
        if status == "complete" and not COMPLETE_ARTIFACTS.issubset(artifacts):
            raise WifiCollectorError(
                f"Pixel上のcomplete run {run_name}のartifactが不足しています"
            )
        for artifact_name, expected_size in artifacts.items():
            local = existing / artifact_name
            if not local.is_file() or local.stat().st_size != expected_size:
                raise WifiCollectorError(
                    f"既存runとPixelのサイズが一致しません。ACKしません: "
                    f"{local} expected={expected_size}"
                )
        if status == "complete" and not (existing / "ffprobe_stream.json").is_file():
            run_ffprobe(existing)
        print(f"[HFR-WIFI] Already collected: {run_name} -> {existing}")
        return "skipped", existing

    model = safe_component(client.endpoint.model)
    device = safe_component(client.endpoint.device_id[:12])
    target = output_root / f"{model}_wifi-{device}_{run_name}"
    partial = output_root / f".{target.name}.partial"
    if target.exists():
        raise WifiCollectorError(f"出力先が既に存在します: {target}")
    partial.mkdir(parents=True, exist_ok=True)

    artifact_order = [
        "hfr_report.json",
        "capture_results.jsonl",
        "encoder_samples.jsonl",
        "hfr_capture.mp4",
    ]
    for artifact_name in artifact_order:
        if artifact_name not in artifacts:
            continue
        encoded_run = quote(run_name, safe="")
        encoded_artifact = quote(artifact_name, safe="")
        client.download(
            f"/api/v1/runs/{encoded_run}/{encoded_artifact}",
            partial / artifact_name,
            artifacts[artifact_name],
            f"{run_name}/{artifact_name}",
        )

    report = read_report(partial / "hfr_report.json", run_name)
    status = str(report.get("status", "missing"))
    if status == "complete":
        missing = COMPLETE_ARTIFACTS.difference(artifacts)
        if missing:
            raise WifiCollectorError(
                f"complete run {run_name}のartifactが不足しています: {sorted(missing)}"
            )
        for name, size in artifacts.items():
            path = partial / name
            if not path.is_file() or path.stat().st_size != size:
                raise WifiCollectorError(f"{run_name}/{name}の最終検証に失敗しました")
        run_ffprobe(partial)
    os.replace(partial, target)
    return "collected", target


def acknowledge(client: ApiClient, run_name: str) -> None:
    client.json_request(
        "POST",
        f"/api/v1/runs/{quote(run_name, safe='')}/ack",
        {},
    )
    print(f"[HFR-WIFI] Pixel acknowledged: {run_name}")


def collect_all(client: ApiClient, output_root: Path) -> tuple[int, int]:
    response = client.json_request("GET", "/api/v1/runs")
    runs = response.get("runs")
    if not isinstance(runs, list):
        raise WifiCollectorError("Pixel応答のrunsが不正です")
    output_root.mkdir(parents=True, exist_ok=True)
    collected = 0
    skipped = 0
    for run in runs:
        if not isinstance(run, dict):
            raise WifiCollectorError("Pixel応答に不正なrunがあります")
        state, session_dir = collect_run(client, output_root, run)
        report = read_report(session_dir / "hfr_report.json", require_text(run, "name"))
        if report.get("status") == "complete" and not (session_dir / "hfr_capture.mp4").is_file():
            raise WifiCollectorError(
                f"既存runに動画がありません。PixelへACKしません: {session_dir}"
            )
        acknowledge(client, require_text(run, "name"))
        if state == "collected":
            collected += 1
        else:
            skipped += 1
    return collected, skipped


def parse_args(argv: list[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Collect Pixel-started HFR runs over authenticated LAN HTTP.",
    )
    parser.add_argument(
        "output_root",
        nargs="?",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
    )
    parser.add_argument("--pair", metavar="CODE", help="Pixelの6桁ペアコード")
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
        "--wait-idle-seconds",
        type=int,
        default=0,
        help="Pixelの連続撮影終了を待つ最大秒数",
    )
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    try:
        config = load_config(args.config)
        endpoint = choose_endpoint(args, config)
        if args.pair:
            token = pair_device(endpoint, args.pair, config, args.config)
        else:
            token = configured_token(endpoint, config, args.config)
        endpoint = wait_until_idle(endpoint, args.wait_idle_seconds)
        client = ApiClient(endpoint, token)
        collected, skipped = collect_all(client, args.output_root)
        print(
            f"[HFR-WIFI] Complete: collected={collected} skipped={skipped} "
            f"output={args.output_root}"
        )
        return 0
    except CaptureBusyError as exc:
        print(f"[HFR-WIFI][BUSY] {exc}", file=sys.stderr)
        return 3
    except WifiCollectorError as exc:
        print(f"[HFR-WIFI][ERROR] {exc}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("[HFR-WIFI] Interrupted; partial downloads are resumable.", file=sys.stderr)
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
