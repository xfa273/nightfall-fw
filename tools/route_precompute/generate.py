#!/usr/bin/env python3
"""Generate the F413 mode2 case6..9 motion tables on the host."""

from __future__ import annotations

import argparse
import hashlib
import os
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[2]
TOOL_DIR = Path(__file__).resolve().parent
HEADER = (
    ROOT
    / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc"
    / "f413_route_motion_table.h"
)
SOURCE = (
    ROOT
    / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src"
    / "f413_route_motion_table.c"
)
GENERATOR = TOOL_DIR / "generate_f413_route_motion.c"
INPUTS = (
    GENERATOR,
    ROOT / "common/route/motion_time.h",
    ROOT / "common/route/motion_time.c",
    ROOT / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc/f413_path_run.h",
    ROOT / "platform/stm32f413/HM_Nightfall_f413_preorder/Core/Src/f413_path_run.c",
    ROOT / "platform/stm32f405/Core/Inc/shortest_run_params.h",
    ROOT / "params/f413_preorder/params.h",
    ROOT / "params/f413_preorder/shortest_run_params_split.c",
)


def input_digest() -> str:
    digest = hashlib.sha256()
    for path in INPUTS:
        relative = path.relative_to(ROOT).as_posix().encode("utf-8")
        digest.update(len(relative).to_bytes(4, "big"))
        digest.update(relative)
        data = path.read_bytes()
        digest.update(len(data).to_bytes(8, "big"))
        digest.update(data)
    return digest.hexdigest()


def build_and_generate(header: Path, source: Path) -> None:
    compiler = os.environ.get("CC", "cc")
    with tempfile.TemporaryDirectory(prefix="nightfall-route-precompute-") as raw:
        temp = Path(raw)
        executable = temp / "generate_f413_route_motion"
        command = [
            compiler,
            "-std=c11",
            "-O2",
            "-Wall",
            "-Wextra",
            "-Werror",
            "-Wpedantic",
            f"-I{ROOT / 'common/route'}",
            f"-I{ROOT / 'platform/stm32f405/Core/Inc'}",
            f"-I{ROOT / 'platform/stm32f413/HM_Nightfall_f413_preorder/Core/Inc'}",
            f"-I{ROOT / 'params/f413_preorder'}",
            str(GENERATOR),
            str(ROOT / "common/route/motion_time.c"),
            str(ROOT / "params/f413_preorder/shortest_run_params_split.c"),
            "-lm",
            "-o",
            str(executable),
        ]
        subprocess.run(command, cwd=ROOT, check=True)
        subprocess.run(
            [str(executable), str(header), str(source), input_digest()],
            cwd=ROOT,
            check=True,
        )


def files_equal(left: Path, right: Path) -> bool:
    return left.exists() and left.read_bytes() == right.read_bytes()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="fail if checked-in generated files do not match current inputs",
    )
    args = parser.parse_args()

    with tempfile.TemporaryDirectory(prefix="nightfall-route-output-") as raw:
        temp = Path(raw)
        generated_header = temp / HEADER.name
        generated_source = temp / SOURCE.name
        build_and_generate(generated_header, generated_source)
        if args.check:
            stale = []
            if not files_equal(HEADER, generated_header):
                stale.append(HEADER.relative_to(ROOT))
            if not files_equal(SOURCE, generated_source):
                stale.append(SOURCE.relative_to(ROOT))
            if stale:
                print("stale generated route motion table:")
                for path in stale:
                    print(f"  {path}")
                print("run: tools/route_precompute/generate.py")
                return 1
            print(f"route motion table is current ({input_digest()})")
            return 0

        HEADER.parent.mkdir(parents=True, exist_ok=True)
        SOURCE.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(generated_header, HEADER)
        shutil.copyfile(generated_source, SOURCE)
        print(f"generated {HEADER.relative_to(ROOT)}")
        print(f"generated {SOURCE.relative_to(ROOT)}")
        print(f"input sha256 {input_digest()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
