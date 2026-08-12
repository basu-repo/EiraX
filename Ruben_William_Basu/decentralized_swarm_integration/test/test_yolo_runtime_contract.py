from pathlib import Path

from decentralized_swarm_integration.halmstad_runners import prepare_local_dependencies


ROOT = Path(__file__).resolve().parents[1]


def test_copied_yolo_runtime_and_weight_are_local():
    local_python = prepare_local_dependencies()
    assert local_python == ROOT / "third_party/python"
    assert (local_python / "ultralytics").is_dir()
    assert (
        ROOT
        / "perception_stack/lrs_halmstad/weights/baylands-leader-v4-2-best.pt"
    ).is_file()
