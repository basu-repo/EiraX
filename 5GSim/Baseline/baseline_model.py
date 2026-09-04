#!/usr/bin/env python3
"""Report accuracy, macro-F1, confusion matrices and feature importances.

The original synthetic baseline notebook was not supplied with the source
material, so this runs the same pipeline over both datasets without changing
the 44-column schema: the co-simulation CSV and the synthetic reference.

Label fields are never inputs. Neither are the identifiers and the timestamp,
which would otherwise name the run rather than describe the traffic.

    ./baseline_model.py
"""

from pathlib import Path

import pandas as pd
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score
from sklearn.model_selection import train_test_split

HERE = Path(__file__).resolve().parent
CLASSES = ["normal", "suspicious", "malicious"]
LABELS = ["attack_type", "attack_stage", "severity",
          "recommended_response", "incident_label"]
IDENTIFIERS = ["timestamp", "organization_id", "fleet_id", "drone_id",
               "mission_id", "session_id", "source_ip", "destination_ip"]


# Train on measured telemetry only and score the held-out half.
def evaluate(frame):
    features = frame.drop(columns=LABELS + IDENTIFIERS)
    features = pd.get_dummies(features)
    train_x, test_x, train_y, test_y = train_test_split(
        features, frame.incident_label, test_size=0.3,
        random_state=0, stratify=frame.incident_label)
    model = RandomForestClassifier(n_estimators=200, random_state=0)
    model.fit(train_x, train_y)
    predicted = model.predict(test_x)
    importances = sorted(zip(features.columns, model.feature_importances_),
                         key=lambda pair: -pair[1])[:10]
    return {
        "rows": len(frame),
        "accuracy": accuracy_score(test_y, predicted),
        "macro_f1": f1_score(test_y, predicted, average="macro",
                             labels=CLASSES, zero_division=0),
        "confusion": confusion_matrix(test_y, predicted, labels=CLASSES),
        "importances": importances,
    }


def section(name, result):
    lines = [f"## {name}", "",
             f"Rows: {result['rows']}. Accuracy **{result['accuracy']:.4f}**, "
             f"macro-F1 **{result['macro_f1']:.4f}**.", "",
             "Confusion matrix, rows true and columns predicted, ordered "
             + ", ".join(CLASSES) + ":", "", "```"]
    lines += ["  " + "  ".join(f"{value:6d}" for value in row)
              for row in result["confusion"]]
    lines += ["```", "", "| Feature | Importance |", "|---|---:|"]
    lines += [f"| `{feature}` | {value:.4f} |" for feature, value in result["importances"]]
    return lines + [""]


def main():
    cosim = evaluate(pd.read_csv(HERE / "data/drone_network_telemetry_cosim.csv"))
    synthetic = evaluate(pd.read_csv(HERE / "data_ref/drone_network_telemetry_dataset.csv"))

    report = ["# Baseline machine-learning metrics", "",
              "| Dataset | Accuracy | Macro-F1 |", "|---|---:|---:|",
              f"| Co-simulation | {cosim['accuracy']:.4f} | {cosim['macro_f1']:.4f} |",
              f"| Synthetic reference | {synthetic['accuracy']:.4f} | "
              f"{synthetic['macro_f1']:.4f} |", ""]
    report += section("Co-simulation dataset", cosim)
    report += section("Synthetic reference", synthetic)
    (HERE / "reports/model_metrics.md").write_text("\n".join(report))

    for name, result in (("CO-SIMULATION", cosim), ("SYNTHETIC", synthetic)):
        print(f"[{name}] rows={result['rows']} accuracy={result['accuracy']:.4f} "
              f"macro-F1={result['macro_f1']:.4f}")
    print("[WROTE] reports/model_metrics.md")


if __name__ == "__main__":
    main()
