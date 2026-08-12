#!/usr/bin/env python3
"""Run the isolated copy of the accepted EiraX UGV + PX4 UAV baseline."""

from __future__ import annotations

import os
from pathlib import Path
import sys


PROJECT_ROOT = Path(__file__).resolve().parents[1]
BASELINE_LAUNCHER = PROJECT_ROOT / "vehicle_stack/run_cooperative_simulation.py"


def command(arguments: list[str]) -> list[str]:
    """Build the exact local-baseline command without invoking external work."""
    return [sys.executable, str(BASELINE_LAUNCHER), *arguments]


def main() -> int:
    if not BASELINE_LAUNCHER.is_file():
        print(f"[FAILED] Isolated baseline is missing: {BASELINE_LAUNCHER}", file=sys.stderr)
        return 1
    # Replace the wrapper process so signals, terminal input, output, and exit
    # status behave exactly like the accepted cooperative launcher.
    os.execv(sys.executable, command(sys.argv[1:]))
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
