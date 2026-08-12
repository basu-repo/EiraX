"""Safe lifecycle wrappers around the imported Halmstad perception classes."""

from __future__ import annotations

import os
from pathlib import Path
import rclpy
from rclpy.executors import ExternalShutdownException
import sys


def prepare_local_dependencies() -> Path:
    """Make the project-owned inference runtime visible to copied perception."""
    project_root = Path(__file__).resolve().parents[1]
    local_python = project_root / "third_party/python"
    if not (local_python / "ultralytics").is_dir():
        raise RuntimeError(f"isolated YOLO runtime is missing: {local_python}")
    value = str(local_python)
    if value not in sys.path:
        sys.path.insert(0, value)
    runtime = project_root / ".runtime"
    yolo_config = runtime / "ultralytics"
    matplotlib_config = runtime / "matplotlib"
    yolo_config.mkdir(parents=True, exist_ok=True)
    matplotlib_config.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("YOLO_CONFIG_DIR", str(yolo_config))
    os.environ.setdefault("MPLCONFIGDIR", str(matplotlib_config))
    return local_python


def _run(node_factory):
    rclpy.init()
    node = node_factory()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        close = getattr(node, "close", None)
        if callable(close):
            close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def leader_detector_main():
    prepare_local_dependencies()
    from lrs_halmstad.perception.leader_detector import LeaderDetector

    _run(LeaderDetector)


def leader_estimator_main():
    # The estimator shares the copied perception stack but does not itself run
    # inference. Preparing one consistent environment keeps both entry points
    # reproducible when launched together.
    prepare_local_dependencies()
    from lrs_halmstad.perception.leader_estimator import LeaderEstimator

    _run(LeaderEstimator)
