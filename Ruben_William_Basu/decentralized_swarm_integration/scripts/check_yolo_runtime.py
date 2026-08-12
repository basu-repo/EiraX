#!/usr/bin/env python3
"""Load the copied YOLO runtime and weight without starting ROS or Gazebo."""

from __future__ import annotations

from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
LOCAL_PYTHON = ROOT / "third_party/python"
WEIGHTS = ROOT / "perception_stack/lrs_halmstad/weights/baylands-leader-v4-2-best.pt"


def main() -> int:
    if not LOCAL_PYTHON.is_dir() or not WEIGHTS.is_file():
        print("[FAILED] Isolated YOLO runtime or weights are missing", file=sys.stderr)
        return 1
    sys.path.insert(0, str(LOCAL_PYTHON))
    from ultralytics import YOLO

    model = YOLO(str(WEIGHTS))
    if model.task != "obb" or "ugv" not in set(model.names.values()):
        print(f"[FAILED] Unexpected model contract: task={model.task} names={model.names}")
        return 1
    print(f"[OK] {WEIGHTS.name}: task={model.task}, classes={model.names}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
