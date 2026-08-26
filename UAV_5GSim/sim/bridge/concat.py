#!/usr/bin/env python3
"""Concatenate complete run CSV files without changing the 44-column schema."""

import argparse
import json
from pathlib import Path

import pandas as pd


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+")
    parser.add_argument("--out", required=True)
    parser.add_argument("--schema")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    schema_path = Path(args.schema) if args.schema else root / "schema_reference.json"
    columns = json.loads(schema_path.read_text(encoding="utf-8"))["columns"]
    frames = []
    for item in args.inputs:
        path = Path(item)
        frame = pd.read_csv(path)
        if list(frame.columns) != columns:
            raise ValueError(f"schema/order mismatch: {path}")
        frames.append(frame)
    result = pd.concat(frames, ignore_index=True)
    output = Path(args.out)
    output.parent.mkdir(parents=True, exist_ok=True)
    result.to_csv(output, index=False)
    print(f"CONCATENATED: {output} ({len(result)} rows, {len(columns)} columns, "
          f"{result.mission_id.nunique()} missions)")


if __name__ == "__main__":
    main()
