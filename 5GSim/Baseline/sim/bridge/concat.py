#!/usr/bin/env python3
"""Concatenate per-run datasets into the final CSV without changing the schema."""

import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("inputs", nargs="+")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    columns = json.loads((Path(__file__).resolve().parents[2] /
                          "schema_reference.json").read_text())["columns"]
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    written = 0
    with out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=columns)
        writer.writeheader()
        for name in args.inputs:
            with open(name, newline="", encoding="utf-8") as source:
                reader = csv.DictReader(source)
                if reader.fieldnames != columns:
                    raise SystemExit(f"schema or column order differs: {name}")
                for row in reader:
                    writer.writerow(row)
                    written += 1
    print(f"CONCATENATED: {out} ({written} rows, {len(columns)} columns, "
          f"{len(args.inputs)} runs)")


if __name__ == "__main__":
    main()
