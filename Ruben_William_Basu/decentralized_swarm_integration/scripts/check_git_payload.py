#!/usr/bin/env python3
"""Validate that non-ignored project files are suitable for a GitHub push."""

from __future__ import annotations

from pathlib import Path
import subprocess


ROOT = Path(__file__).resolve().parents[1]
LIMIT_BYTES = 100 * 1024 * 1024


def git_candidates() -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z", "--", str(ROOT)],
        cwd=ROOT,
        check=True,
        stdout=subprocess.PIPE,
    )
    candidates = []
    for raw in result.stdout.split(b"\0"):
        if not raw:
            continue
        path = ROOT / raw.decode(errors="surrogateescape")
        if path.is_file():
            candidates.append(path)
    return candidates


def main() -> int:
    candidates = git_candidates()
    oversized = [(path, path.stat().st_size) for path in candidates if path.stat().st_size >= LIMIT_BYTES]
    total = sum(path.stat().st_size for path in candidates)
    print(f"Git candidates: {len(candidates)} files, {total / 1024 / 1024:.1f} MiB total")
    if oversized:
        for path, size in oversized:
            print(f"[BLOCKED] {size / 1024 / 1024:.1f} MiB: {path.relative_to(ROOT)}")
        return 1
    print("[OK] No Git candidate reaches GitHub's 100 MiB single-file limit")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
