#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import sys
from collections import defaultdict
from dataclasses import dataclass, field
from pathlib import Path

from lrs_halmstad.common.paths import workspace_root


def _default_datasets_root() -> Path:
    configured_root = os.environ.get("LRS_HALMSTAD_DATASETS_ROOT", "").strip()
    if configured_root:
        return Path(configured_root).expanduser().resolve()
    return (workspace_root(__file__)).resolve()


DATASETS_ROOT = _default_datasets_root()
IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff", ".webp"}
CATEGORY_EXTS = {
    "images": IMAGE_EXTS,
    "labels": {".txt"},
    "labels_aabb": {".txt"},
    "labels_det": {".txt"},
    "metadata": {".json"},
    "overlay": IMAGE_EXTS,
    "overlay_obb": IMAGE_EXTS,
    "overlay_detection": IMAGE_EXTS,
}
DEFAULT_OPTIONAL_CATEGORIES = (
    "labels",
    "labels_aabb",
    "labels_det",
    "metadata",
    "overlay",
    "overlay_obb",
    "overlay_detection",
)


@dataclass
class CategoryReport:
    missing_from_category: list[str] = field(default_factory=list)
    orphan_paths: list[Path] = field(default_factory=list)
    duplicate_groups: dict[str, list[Path]] = field(default_factory=dict)


@dataclass
class DatasetReport:
    dataset: Path
    splits: list[str]
    categories: list[str]
    reference_by_split: dict[str, str] = field(default_factory=dict)
    per_split: dict[str, dict[str, CategoryReport]] = field(default_factory=dict)

    def has_remaining_issues(self) -> bool:
        for split_reports in self.per_split.values():
            for report in split_reports.values():
                if report.missing_from_category or report.duplicate_groups or report.orphan_paths:
                    return True
        return False


def parse_args() -> argparse.Namespace:
    datasets_root = str(DATASETS_ROOT)
    parser = argparse.ArgumentParser(
        description=(
            f"Check a dataset under {datasets_root} for missing or orphaned files "
            "across images, labels, labels_aabb, metadata, and overlay outputs. "
            "Missing files are checked against images/<split>. "
            "When overlay_obb/<split> exists, files without a matching overlay_obb "
            "can be pruned as orphans."
        )
    )
    parser.add_argument(
        "dataset",
        nargs="?",
        default=None,
        help=(
            f"Dataset path, either absolute or relative to {datasets_root}. "
            f"Examples: warehouse_v1/run1 or {datasets_root}/warehouse_auto"
        ),
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help=f"Scan every dataset directory under {datasets_root} that contains an images/ folder.",
    )
    parser.add_argument(
        "--prune-orphans",
        action="store_true",
        help=(
            "Delete orphaned files from every category. "
            "Labels/metadata/overlays without matching images are deleted. "
            "If overlay_obb/<split> exists, images and matching files without overlay_obb "
            "are also deleted."
        )
    )

    parser.add_argument(
        "--prune-incomplete",
        action="store_true",
        help=(
            "Delete images and matching files for stems that are missing required "
            "categories such as labels, labels_aabb, or metadata."
        ),
    )
    
    parser.add_argument(
        "--show",
        type=int,
        default=20,
        help="Maximum number of example paths or stems to print per issue group. Use 0 to print all.",
    )
    return parser.parse_args()


def resolve_dataset_path(value: str) -> Path:
    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return candidate.resolve()
    return (DATASETS_ROOT / candidate).resolve()


def discover_dataset_dirs(root: Path = DATASETS_ROOT) -> list[Path]:
    datasets: list[Path] = []
    seen: set[Path] = set()
    for images_dir in sorted(root.rglob("images")):
        if images_dir.is_dir():
            dataset_dir = images_dir.parent.resolve()
            if dataset_dir not in seen:
                seen.add(dataset_dir)
                datasets.append(dataset_dir)
    return datasets


def discover_splits(dataset_dir: Path, categories: list[str]) -> list[str]:
    splits: set[str] = set()
    for category in categories:
        category_dir = dataset_dir / category
        if not category_dir.is_dir():
            continue
        for child in category_dir.iterdir():
            if child.is_dir():
                splits.add(child.name)
    return sorted(splits)


def discover_categories(dataset_dir: Path) -> list[str]:
    categories = ["images"]
    for category in DEFAULT_OPTIONAL_CATEGORIES:
        if (dataset_dir / category).is_dir():
            categories.append(category)
    return categories

def prune_incomplete(report: DatasetReport) -> int:
    """
    Delete image stems that are missing required partner files.

    Example:
      images/train/foo.jpg exists
      labels/train/foo.txt missing

    Then delete:
      images/train/foo.jpg
      labels_aabb/train/foo.txt, if it exists
      metadata/train/foo.json, if it exists
      overlays, if they exist
    """
    removed = 0

    for split, split_reports in report.per_split.items():
        incomplete_stems: set[str] = set()

        # Anything missing from a non-image category means the image stem is incomplete.
        for category, category_report in split_reports.items():
            if category == "images":
                continue

            incomplete_stems.update(category_report.missing_from_category)

        if not incomplete_stems:
            continue

        for category, category_report in split_reports.items():
            category_dir = report.dataset / category / split

            if not category_dir.is_dir():
                continue

            files, _ = collect_files(category_dir, CATEGORY_EXTS[category])

            for stem in sorted(incomplete_stems):
                for path in files.get(stem, []):
                    if path.exists():
                        path.unlink()
                        removed += 1

            # Clear deleted paths from the report view.
            category_report.orphan_paths = [
                path for path in category_report.orphan_paths if path.exists()
            ]

        # Also delete the actual image files for incomplete stems.
        image_dir = report.dataset / "images" / split
        image_files, _ = collect_files(image_dir, CATEGORY_EXTS["images"])

        for stem in sorted(incomplete_stems):
            for path in image_files.get(stem, []):
                if path.exists():
                    path.unlink()
                    removed += 1

        # Clear missing reports, because those stems were removed.
        for category_report in split_reports.values():
            category_report.missing_from_category = [
                stem
                for stem in category_report.missing_from_category
                if stem not in incomplete_stems
            ]

    return removed

def collect_files(base_dir: Path, extensions: set[str]) -> tuple[dict[str, list[Path]], dict[str, list[Path]]]:
    files_by_stem: dict[str, list[Path]] = defaultdict(list)
    duplicates: dict[str, list[Path]] = {}
    if not base_dir.is_dir():
        return files_by_stem, duplicates

    for path in sorted(base_dir.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in extensions:
            continue
        stem = path.relative_to(base_dir).with_suffix("").as_posix()
        files_by_stem[stem].append(path)

    for stem, paths in files_by_stem.items():
        if len(paths) > 1:
            duplicates[stem] = paths
    return files_by_stem, duplicates


def limit_items(items: list[str], maximum: int) -> list[str]:
    if maximum == 0 or len(items) <= maximum:
        return items
    return items[:maximum]


def print_report(report: DatasetReport, show_limit: int) -> None:
    print(f"Dataset: {report.dataset}")
    print(f"Splits: {', '.join(report.splits) if report.splits else 'none found'}")
    print(f"Categories checked: {', '.join(report.categories)}")
    print("")

    any_issue = False
    for split in report.splits:
        reference = report.reference_by_split.get(split, "images")
        print(f"[{split}] kept={reference}")
        split_has_issue = False
        for category, category_report in report.per_split.get(split, {}).items():
            missing = category_report.missing_from_category
            orphans = [str(path) for path in category_report.orphan_paths]
            duplicates = category_report.duplicate_groups
            if not (missing or orphans or duplicates):
                continue

            split_has_issue = True
            any_issue = True
            if missing:
                shown = limit_items(missing, show_limit)
                print(f"  missing {category}: {len(missing)}")
                for stem in shown:
                    print(f"    {stem}")
                if show_limit and len(shown) < len(missing):
                    print(f"    ... {len(missing) - len(shown)} more")
            if orphans:
                shown = limit_items(orphans, show_limit)
                print(f"  orphan {category}: {len(orphans)}")
                for path in shown:
                    print(f"    {path}")
                if show_limit and len(shown) < len(orphans):
                    print(f"    ... {len(orphans) - len(shown)} more")
            if duplicates:
                print(f"  duplicate {category}: {len(duplicates)} stem(s)")
                stems = limit_items(sorted(duplicates.keys()), show_limit)
                for stem in stems:
                    joined = ", ".join(str(path) for path in duplicates[stem])
                    print(f"    {stem}: {joined}")
                if show_limit and len(stems) < len(duplicates):
                    print(f"    ... {len(duplicates) - len(stems)} more")

        if not split_has_issue:
            print("  OK")
        print("")

    if not any_issue:
        print("No issues found.")


def prune_orphans(report: DatasetReport) -> int:
    removed = 0
    for split_reports in report.per_split.values():
        for category, category_report in split_reports.items():
            for path in category_report.orphan_paths:
                path.unlink()
                removed += 1
            category_report.orphan_paths = []
    return removed


def scan_dataset(dataset_dir: Path) -> DatasetReport:
    if not dataset_dir.is_dir():
        raise FileNotFoundError(f"Dataset directory not found: {dataset_dir}")
    if not (dataset_dir / "images").is_dir():
        raise FileNotFoundError(f"Dataset directory is missing images/: {dataset_dir}")

    categories = discover_categories(dataset_dir)
    splits = discover_splits(dataset_dir, categories)
    report = DatasetReport(dataset=dataset_dir, splits=splits, categories=categories)

    if not splits:
        raise FileNotFoundError(f"No split directories found under {dataset_dir / 'images'}")

    for split in splits:
        split_reports: dict[str, CategoryReport] = {}
        collected: dict[str, tuple[dict[str, list[Path]], dict[str, list[Path]]]] = {}
        for category in categories:
            category_dir = dataset_dir / category / split
            collected[category] = collect_files(category_dir, CATEGORY_EXTS[category])
            _, duplicates = collected[category]
            split_reports[category] = CategoryReport(duplicate_groups=duplicates)

        image_stems = set(collected["images"][0].keys())

        overlay_obb_exists = (
            "overlay_obb" in categories
            and (dataset_dir / "overlay_obb" / split).is_dir()
        )

        if overlay_obb_exists:
            overlay_obb_stems = set(collected["overlay_obb"][0].keys())
            kept_stems = image_stems & overlay_obb_stems
            report.reference_by_split[split] = "images∩overlay_obb"
        else:
            overlay_obb_stems = set()
            kept_stems = image_stems
            report.reference_by_split[split] = "images"

        for category in categories:
            files, duplicates = collected[category]
            category_stems = set(files.keys())

            # Missing files should be checked against all images, not only kept_stems.
            # Otherwise images missing overlay_obb are removed from the reference and
            # their missing labels are hidden.
            if category == "images":
                split_reports[category].missing_from_category = []
            else:
                split_reports[category].missing_from_category = sorted(image_stems - category_stems)

            # Orphans are files that do not have a matching image.
            split_reports[category].orphan_paths = [
                path for stem in sorted(category_stems - image_stems) for path in files[stem]
            ]

        # Extra pruning behavior:
        # If overlay_obb exists, images without overlay_obb should be treated as prune-orphans.
        if overlay_obb_exists:
            image_files, _ = collected["images"]
            missing_overlay_obb_stems = image_stems - overlay_obb_stems

            split_reports["images"].orphan_paths.extend(
                path
                for stem in sorted(missing_overlay_obb_stems)
                for path in image_files[stem]
            )

            # Also prune matching files in other categories for images missing overlay_obb.
            for category in categories:
                if category == "images":
                    continue

                files, _ = collected[category]

                split_reports[category].orphan_paths.extend(
                    path
                    for stem in sorted(missing_overlay_obb_stems)
                    for path in files.get(stem, [])
                )

        report.per_split[split] = split_reports

    return report


def main() -> int:
    args = parse_args()

    if args.all and args.dataset:
        print("Use either a dataset path or --all, not both.", file=sys.stderr)
        return 2
    if not args.all and not args.dataset:
        print("Provide a dataset path or use --all.", file=sys.stderr)
        return 2

    if args.all:
        datasets = discover_dataset_dirs()
    else:
        dataset_path = resolve_dataset_path(args.dataset)
        if (dataset_path / "images").is_dir():
            datasets = [dataset_path]
        else:
            datasets = discover_dataset_dirs(dataset_path)
            if not datasets:
                print(
                    f"Dataset directory is missing images/ and contains no nested datasets: {dataset_path}",
                    file=sys.stderr,
                )
                return 1
    exit_code = 0

    for index, dataset_dir in enumerate(datasets):
        if index:
            print("=" * 80)
        try:
            report = scan_dataset(dataset_dir)
        except FileNotFoundError as exc:
            print(str(exc), file=sys.stderr)
            exit_code = 1
            continue

        if args.prune_orphans:
            removed = prune_orphans(report)
            if removed:
                print(f"Removed {removed} orphaned file(s) from {dataset_dir}.")
                print("")
                report = scan_dataset(dataset_dir)

        if args.prune_incomplete:
            removed = prune_incomplete(report)
            if removed:
                print(f"Removed {removed} incomplete stem file(s) from {dataset_dir}.")
                print("")
                report = scan_dataset(dataset_dir)

        print_report(report, args.show)
        if report.has_remaining_issues():
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
