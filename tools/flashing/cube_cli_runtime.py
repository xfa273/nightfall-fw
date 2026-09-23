"""Run CubeProgrammer's bundled native CLI without modifying its installation."""

import platform
import shutil
import stat
import subprocess
import sys
import tempfile
from contextlib import contextmanager
from pathlib import Path


def apple_silicon() -> bool:
    if sys.platform != "darwin":
        return False
    if platform.machine() == "arm64":
        return True
    # Also recognize Apple Silicon when Python itself runs under Rosetta.
    result = subprocess.run(
        ["/usr/sbin/sysctl", "-n", "hw.optional.arm64"],
        capture_output=True, text=True, check=False,
    )
    return result.returncode == 0 and result.stdout.strip() == "1"


def macho_arches(path: Path) -> set:
    """Inspect, never execute, a candidate (scripts/custom wrappers return empty)."""
    try:
        result = subprocess.run(
            ["/usr/bin/lipo", "-archs", str(path)],
            capture_output=True, text=True, check=False,
        )
    except OSError:
        return set()
    return set(result.stdout.split()) if result.returncode == 0 else set()


@contextmanager
def cube_cli_runtime(cli: str):
    """Prefer the same installation's arm64 SDK CLI over its x86-only bin CLI.

    CubeProgrammer 2.21 ships a universal api/lib CLI but an x86-only bin CLI.
    The SDK executable needs bin/../Data_Base and bin/FlashLoader at runtime,
    so simply selecting api/lib/STM32_Programmer_CLI is insufficient. Copy the
    executable into a private, short-lived bin layout and link vendor resources.
    A symlink to the executable does NOT relocate its executable-relative paths.
    No download, signing change, installation edit, or target access occurs here.
    """
    if not apple_silicon():
        yield cli
        return

    selected = Path(cli).resolve()
    arches = macho_arches(selected)
    api_dir = None
    if selected.parent.name == "lib" and selected.parent.parent.name == "api":
        if "arm64" in arches:
            api_dir = selected.parent
    elif selected.parent.name == "bin" and arches and "arm64" not in arches:
        candidate = selected.parent.parent / "api/lib/STM32_Programmer_CLI"
        if candidate.is_file() and "arm64" in macho_arches(candidate):
            api_dir = candidate.parent

    if api_dir is None:
        if arches and "arm64" not in arches:
            raise RuntimeError(
                f"CubeProgrammer CLI はarm64非対応です: {selected}。"
                "arm64対応版をインストールし --cli / STM32_PROGRAMMER_CLI で指定してください"
            )
        yield cli
        return

    install = api_dir.parent.parent
    source = api_dir / "STM32_Programmer_CLI"
    for required in (install / "Data_Base", install / "bin/FlashLoader"):
        if not required.is_dir():
            raise RuntimeError(f"CubeProgrammerの必要なリソースがありません: {required}")

    with tempfile.TemporaryDirectory(prefix="nightfall-cube-arm64-") as temporary:
        runtime = Path(temporary)
        bindir = runtime / "bin"
        bindir.mkdir()
        executable = bindir / source.name
        shutil.copy2(source, executable)
        executable.chmod(executable.stat().st_mode | stat.S_IXUSR)
        for library in api_dir.iterdir():
            if library.suffix in (".dylib", ".framework", ".conf"):
                (bindir / library.name).symlink_to(library)
        for name in ("Data_Base", "Drivers"):
            if (install / name).is_dir():
                (runtime / name).symlink_to(install / name)
        for name in ("FlashLoader", "ExternalLoader", "OBL", "HSM"):
            resource = install / "bin" / name
            if resource.is_dir():
                (bindir / name).symlink_to(resource)
        print(f"CubeProgrammer native arm64: {source}", flush=True)
        yield str(executable)
