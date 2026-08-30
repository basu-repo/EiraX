#!/usr/bin/env python3
"""Reproduce the experiment-document machine-learning comparison.

This is step 8 of the experiment document's data assembly procedure: run the
baseline pipeline over the co-simulation dataset and compare it with the
synthetic reference, reporting accuracy, macro-F1, confusion matrices and
feature importances.

The document requires the comparison to be run over several feature sets --
raw metrics only, raw plus 5G context, raw plus security indicators, and the
full set -- so the contribution of each layer can be reported separately.

Label fields are never used as inputs, and neither are identifiers that would
reveal the scenario, the run or the injection point. The train/test boundary is
drawn on complete mission identifiers so that no window of a mission can appear
on both sides, and every mission derived from a measured flight that also
appears in training is forced into the held-out partition.

Usage:

    python evaluate_models.py
    python evaluate_models.py --dataset experiments/data/drone_network_telemetry_cosim.csv
"""

import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from sklearn.compose import ColumnTransformer
from sklearn.ensemble import RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import (accuracy_score, classification_report,
                             confusion_matrix, f1_score)
from sklearn.pipeline import Pipeline
from sklearn.preprocessing import OneHotEncoder, StandardScaler

HERE = Path(__file__).resolve().parent
CLASSES = ["normal", "suspicious", "malicious"]

# Never inputs: the five label fields, plus identifiers that would name the
# scenario, the mission or the run outright.
LABEL_FIELDS = ["attack_type", "attack_stage", "severity",
                "recommended_response", "incident_label"]
IDENTIFIER_FIELDS = ["timestamp", "organization_id", "fleet_id", "drone_id",
                     "mission_id", "session_id", "source_ip", "destination_ip"]

# Feature layers required by the experiment document.
RAW_NETWORK = ["packet_count", "byte_count", "packets_per_second",
               "throughput_kbps", "latency_ms", "jitter_ms",
               "packet_loss_rate", "retransmission_rate",
               "connection_duration_sec"]
FIVE_G_CONTEXT = ["signal_quality_dbm", "handover_event", "handover_count",
                  "cell_id", "network_slice_id", "protocol",
                  "source_component", "destination_component",
                  "source_port", "destination_port"]
SECURITY_INDICATORS = ["authentication_status", "command_channel_activity",
                       "video_stream_activity", "telemetry_stream_activity",
                       "api_request_count", "failed_connection_attempts",
                       "unusual_destination_flag", "session_reuse_flag",
                       "traffic_burst_flag", "anomaly_score"]

FEATURE_GROUPS = {
    "raw_network": RAW_NETWORK,
    "raw_plus_5g_context": RAW_NETWORK + FIVE_G_CONTEXT,
    "security_indicators": RAW_NETWORK + SECURITY_INDICATORS,
    "complete": RAW_NETWORK + FIVE_G_CONTEXT + SECURITY_INDICATORS,
}


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--dataset",
                        default="experiments/data/drone_network_telemetry_cosim.csv")
    parser.add_argument("--synthetic", default="data_ref/drone_network_telemetry_dataset.csv")
    parser.add_argument("--reports", default="experiments/reports")
    parser.add_argument("--split", default="experiments/reports/model_split.json",
                        help="Recorded train/test mission partition to reuse. "
                             "When absent, the partition below is derived.")
    parser.add_argument("--name", default="model_comparison",
                        help="Base name for the two report files written "
                             "into --reports.")
    parser.add_argument("--seed", type=int, default=0)
    return parser.parse_args()


# Intensity and repetition markers. The first-listed, least severe member of a
# scenario family trains; every other variant of the same measured flight is
# held out, so a model cannot score by recognising a flight it already saw.
BASELINE_MARKERS = ("_LOW_", "_MEDIUM_", "_SLOW_", "_PILOT_", "_BASE_001", "_BASE_002")


def mission_split(frame, seed, recorded=None):
    """Split on complete missions, holding out every derived mission variant."""
    missions = sorted(frame.mission_id.unique())

    if recorded:
        train = [m for m in recorded.get("train_missions", []) if m in missions]
        test = [m for m in recorded.get("test_missions", []) if m in missions]
        unassigned = sorted(set(missions) - set(train) - set(test))
        if train and test:
            # Anything the record does not mention is held out, never trained on.
            return sorted(train), sorted(set(test) | set(unassigned))

    families = {}
    for mission in missions:
        # M_DOS_LOW_001 and M_DOS_HIGH_001 belong to the family M_DOS.
        parts = mission.split("_")
        family = "_".join(parts[:2]) if len(parts) > 2 else mission
        families.setdefault(family, []).append(mission)

    train = []
    for family in sorted(families):
        group = sorted(families[family])
        seeds = [m for m in group if any(mark in m for mark in BASELINE_MARKERS)]
        train.extend(seeds or group[:1])
    train = sorted(set(train))
    test = sorted(set(missions) - set(train))
    if not test:                                  # degenerate single-mission input
        cut = max(1, len(missions) // 2)
        train, test = missions[:cut], missions[cut:]
    return train, test


def build_pipeline(frame, features, model, seed):
    numeric = [c for c in features if pd.api.types.is_numeric_dtype(frame[c])]
    categorical = [c for c in features if c not in numeric]
    pre = ColumnTransformer([
        ("numeric", StandardScaler(), numeric),
        ("categorical", OneHotEncoder(handle_unknown="ignore"), categorical),
    ])
    return Pipeline([("features", pre), ("model", model)]), numeric, categorical


def models(seed):
    return {
        "logistic_regression": LogisticRegression(max_iter=2000, random_state=seed),
        "random_forest": RandomForestClassifier(n_estimators=300, random_state=seed,
                                                n_jobs=-1),
    }


def score(truth, predicted):
    matrix = confusion_matrix(truth, predicted, labels=CLASSES)
    report = classification_report(truth, predicted, labels=CLASSES,
                                   output_dict=True, zero_division=0)
    # A false alarm is a benign window reported as suspicious or malicious.
    benign = matrix[0].sum()
    false_alarms = matrix[0, 1] + matrix[0, 2]
    return {
        "accuracy": accuracy_score(truth, predicted),
        "macro_f1": f1_score(truth, predicted, average="macro", labels=CLASSES,
                             zero_division=0),
        "confusion_matrix": matrix.tolist(),
        "per_class": {c: {"precision": report[c]["precision"],
                          "recall": report[c]["recall"],
                          "f1": report[c]["f1-score"],
                          "support": int(report[c]["support"])} for c in CLASSES},
        "false_alarm_rate": float(false_alarms / benign) if benign else 0.0,
    }


def evaluate(frame, train_missions, test_missions, seed):
    train = frame[frame.mission_id.isin(train_missions)]
    test = frame[frame.mission_id.isin(test_missions)]
    results, importance = {}, []
    for name, features in FEATURE_GROUPS.items():
        results[name] = {}
        for model_name, estimator in models(seed).items():
            pipeline, numeric, categorical = build_pipeline(frame, features,
                                                            estimator, seed)
            pipeline.fit(train[features], train.incident_label)
            results[name][model_name] = score(test.incident_label,
                                              pipeline.predict(test[features]))
            if name == "complete" and model_name == "random_forest":
                names = pipeline.named_steps["features"].get_feature_names_out()
                weights = pipeline.named_steps["model"].feature_importances_
                importance = sorted(
                    ({"feature": str(n), "importance": float(w)}
                     for n, w in zip(names, weights)),
                    key=lambda item: -item["importance"])
    return results, importance


def synthetic_reference(path, seed):
    """Baseline point of comparison, split the same way on mission identifiers."""
    frame = pd.read_csv(path)
    train_missions, test_missions = mission_split(frame, seed)
    if not test_missions:
        raise SystemExit(f"{path}: the synthetic reference has no held-out missions")
    features = FEATURE_GROUPS["complete"]
    train = frame[frame.mission_id.isin(train_missions)]
    test = frame[frame.mission_id.isin(test_missions)]
    pipeline, _, _ = build_pipeline(frame, features,
                                    models(seed)["random_forest"], seed)
    pipeline.fit(train[features], train.incident_label)
    result = score(test.incident_label, pipeline.predict(test[features]))
    result["rows"] = len(frame)
    result["train_missions"] = len(train_missions)
    result["test_missions"] = len(test_missions)
    return result


def markdown(report):
    best = report["feature_groups"]["complete"]["random_forest"]
    reference = report["synthetic_reference_best"]
    lines = [
        "# Machine-learning comparison",
        "",
        f"Dataset: `{report['dataset']}` "
        f"({report['rows']:,} rows, {report['missions']} missions).",
        "",
        "Regenerate with `python evaluate_models.py`.",
        "",
        "| Metric | Simulation-derived dataset | Synthetic reference |",
        "|---|---:|---:|",
        f"| Accuracy | {best['accuracy']:.4f} | {reference['accuracy']:.4f} |",
        f"| Macro-F1 | {best['macro_f1']:.4f} | {reference['macro_f1']:.4f} |",
        f"| False-alarm rate | {best['false_alarm_rate']:.4f} | "
        f"{reference['false_alarm_rate']:.4f} |",
        "",
        "## Feature-group comparison",
        "",
        "The experiment document requires the contribution of each feature "
        "layer to be reported separately.",
        "",
        "| Feature group | Model | Accuracy | Macro-F1 | False-alarm rate |",
        "|---|---|---:|---:|---:|",
    ]
    for group, entries in report["feature_groups"].items():
        for model, metrics in entries.items():
            lines.append(f"| {group} | {model} | {metrics['accuracy']:.4f} | "
                         f"{metrics['macro_f1']:.4f} | "
                         f"{metrics['false_alarm_rate']:.4f} |")
    lines += [
        "",
        "## Held-out confusion matrix, complete features, random forest",
        "",
        "Rows are the true class, columns the predicted class, ordered "
        f"{', '.join(CLASSES)}.",
        "",
        "```text",
    ]
    lines += ["  " + "  ".join(f"{v:6d}" for v in row)
              for row in best["confusion_matrix"]]
    lines += ["```", "", "## Top features", "",
              "| Feature | Importance |", "|---|---:|"]
    lines += [f"| `{item['feature']}` | {item['importance']:.4f} |"
              for item in report["feature_importance"][:12]]
    lines += ["", "## Split policy", "",
              report["split_policy"], "",
              f"Training missions ({len(report['train_missions'])}): "
              + ", ".join(f"`{m}`" for m in report["train_missions"]), "",
              f"Held-out missions ({len(report['test_missions'])}): "
              + ", ".join(f"`{m}`" for m in report["test_missions"]), ""]
    return "\n".join(lines)


def main():
    args = arguments()
    dataset = Path(args.dataset)
    frame = pd.read_csv(dataset)
    leaked = [c for c in LABEL_FIELDS + IDENTIFIER_FIELDS
              if c in FEATURE_GROUPS["complete"]]
    assert not leaked, f"label or identifier fields reached the feature set: {leaked}"

    split_path = Path(args.split) if args.split else None
    recorded = (json.loads(split_path.read_text())
                if split_path and split_path.is_file() else None)
    train_missions, test_missions = mission_split(frame, args.seed, recorded)
    groups, importance = evaluate(frame, train_missions, test_missions, args.seed)

    # Record the path relative to the project so the report is identical however
    # the script is invoked.
    try:
        recorded_dataset = dataset.resolve().relative_to(HERE)
    except ValueError:
        recorded_dataset = dataset
    report = {
        "dataset": str(recorded_dataset),
        "rows": len(frame),
        "missions": int(frame.mission_id.nunique()),
        "split_policy": "complete mission IDs; every measured-source derivative "
                        "kept in held-out test",
        "split_source": str(split_path) if recorded else "derived from scenario families",
        "excluded_from_features": LABEL_FIELDS + IDENTIFIER_FIELDS,
        "train_missions": train_missions,
        "test_missions": test_missions,
        "feature_groups": groups,
        "feature_importance": importance,
        "synthetic_reference_best": synthetic_reference(Path(args.synthetic), args.seed),
    }

    reports = Path(args.reports)
    reports.mkdir(parents=True, exist_ok=True)
    (reports / args.name).with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
    (reports / args.name).with_suffix(".md").write_text(markdown(report))

    best = groups["complete"]["random_forest"]
    reference = report["synthetic_reference_best"]
    print(f"[MEASURED]  accuracy={best['accuracy']:.4f} "
          f"macro-F1={best['macro_f1']:.4f} "
          f"false-alarm={best['false_alarm_rate']:.4f}")
    print(f"[SYNTHETIC] accuracy={reference['accuracy']:.4f} "
          f"macro-F1={reference['macro_f1']:.4f} "
          f"false-alarm={reference['false_alarm_rate']:.4f}")
    for group, entries in groups.items():
        rf = entries["random_forest"]
        print(f"  {group:22s} random_forest accuracy={rf['accuracy']:.4f} "
              f"macro-F1={rf['macro_f1']:.4f}")
    print(f"[WROTE] {(reports/args.name).with_suffix('.json')}")
    print(f"[WROTE] {(reports/args.name).with_suffix('.md')}")


if __name__ == "__main__":
    main()
