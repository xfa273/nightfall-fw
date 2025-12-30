#!/usr/bin/env python3

import argparse
import subprocess
from pathlib import Path


def _run_git(repo: Path, args: list[str]) -> str:
    try:
        out = subprocess.check_output(["git", *args], cwd=str(repo), stderr=subprocess.DEVNULL)
        return out.decode("utf-8", errors="replace").strip()
    except Exception:
        return ""


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--out", required=True)
    p.add_argument("--repo", required=True)
    p.add_argument("--variant", required=True)
    p.add_argument("--target", required=True)
    args = p.parse_args()

    out_path = Path(args.out)
    repo = Path(args.repo)

    git_describe = _run_git(repo, ["describe", "--tags", "--dirty", "--always"]) or "nogit"
    git_sha = _run_git(repo, ["rev-parse", "--short", "HEAD"]) or "nogit"

    out_path.parent.mkdir(parents=True, exist_ok=True)
    content = (
        "#pragma once\n"
        f"#define NIGHTFALL_BUILD_TARGET \\\"{args.target}\\\"\n"
        f"#define NIGHTFALL_BUILD_VARIANT \\\"{args.variant}\\\"\n"
        f"#define NIGHTFALL_GIT_DESCRIBE \\\"{git_describe}\\\"\n"
        f"#define NIGHTFALL_GIT_SHA \\\"{git_sha}\\\"\n"
    )

    if out_path.exists():
        old = out_path.read_text(encoding="utf-8", errors="replace")
        if old == content:
            return 0

    out_path.write_text(content, encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
