"""Create the transferable standalone UGV run dataset."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import shutil


class RunDataset:
    def __init__(self, project_root: Path, world_file: Path) -> None:
        run_id = datetime.now().astimezone().strftime("run_%Y%m%d_%H%M%S")
        self.root = project_root / "datasets" / run_id
        self.logs = self.root / "logs"
        self.rosbag = self.root / "rosbag"
        self.world = self.root / "world"
        for path in (self.logs, self.world):
            path.mkdir(parents=True, exist_ok=False)
        shutil.copy2(world_file, self.world / world_file.name)
        self.events_path = self.root / "events.jsonl"
        self.metadata_path = self.root / "metadata.json"
        self.write_metadata({
            "run_id": run_id,
            "started_at": datetime.now().astimezone().isoformat(),
            "world_file": str(world_file),
            "record_images": False,
            "data_policy": "real_ugv_transferable_only",
        })

    def write_metadata(self, values: dict) -> None:
        self.metadata_path.write_text(json.dumps(values, indent=2) + "\n", encoding="utf-8")

    def event(self, component: str, event: str, **details) -> None:
        record = {
            "timestamp": datetime.now().astimezone().isoformat(),
            "component": component,
            "event": event,
            **details,
        }
        with self.events_path.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(record, separators=(",", ":")) + "\n")
