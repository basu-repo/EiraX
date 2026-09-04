#!/usr/bin/env python3
"""Report accuracy, macro-F1, confusion matrices and feature importances.

The same pipeline runs over the co-simulation dataset and the synthetic
reference without changing the 44-column schema. The co-simulation dataset is
also scored over the four feature sets the leakage audit asks for: raw metrics
only, raw plus 5G context, raw plus security indicators, and the full set.
The random forest is the baseline; gradient boosting and logistic regression
run over the same splits so the models can be compared.

Label fields are never inputs. Neither are the identifiers and the timestamp,
which would otherwise name the run rather than describe the traffic.

    ./baseline_model.py
"""

from pathlib import Path

import pandas as pd
from sklearn.ensemble import HistGradientBoostingClassifier, RandomForestClassifier
from sklearn.linear_model import LogisticRegression
from sklearn.metrics import accuracy_score, confusion_matrix, f1_score
from sklearn.model_selection import train_test_split
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import StandardScaler

HERE = Path(__file__).resolve().parent
CLASSES = ["normal", "suspicious", "malicious"]
LABELS = ["attack_type", "attack_stage", "severity",
          "recommended_response", "incident_label"]
IDENTIFIERS = ["timestamp", "organization_id", "fleet_id", "drone_id",
               "mission_id", "session_id", "source_ip", "destination_ip"]
RAW = ["packet_count", "byte_count", "packets_per_second", "throughput_kbps",
       "latency_ms", "jitter_ms", "packet_loss_rate", "retransmission_rate",
       "connection_duration_sec"]
CONTEXT = ["source_component", "destination_component", "source_port",
           "destination_port", "protocol", "network_slice_id", "cell_id",
           "signal_quality_dbm", "handover_event", "handover_count"]
SECURITY = ["authentication_status", "command_channel_activity",
            "video_stream_activity", "telemetry_stream_activity",
            "api_request_count", "failed_connection_attempts",
            "unusual_destination_flag", "session_reuse_flag",
            "traffic_burst_flag", "anomaly_score"]
FEATURE_SETS = {
    "raw metrics only": RAW,
    "raw + 5G context": RAW + CONTEXT,
    "raw + security indicators": RAW + SECURITY,
    "full feature set": None,  # every column except labels and identifiers
}
MODELS = {
    "random forest": lambda: RandomForestClassifier(n_estimators=200, random_state=0),
    "gradient boosting": lambda: HistGradientBoostingClassifier(random_state=0),
    "logistic regression": lambda: make_pipeline(StandardScaler(),
                                                 LogisticRegression(max_iter=2000)),
}


# Train on the chosen columns only and score the held-out part.
def evaluate(frame, columns=None, model_name="random forest"):
    features = frame[columns] if columns else frame.drop(columns=LABELS + IDENTIFIERS)
    features = pd.get_dummies(features)
    train_x, test_x, train_y, test_y = train_test_split(
        features, frame.incident_label, test_size=0.3,
        random_state=0, stratify=frame.incident_label)
    model = MODELS[model_name]()
    model.fit(train_x, train_y)
    predicted = model.predict(test_x)
    importances = sorted(zip(features.columns, getattr(model, "feature_importances_", [])),
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
    cosim_frame = pd.read_csv(HERE / "data/drone_network_telemetry_cosim.csv")
    cosim = {name: evaluate(cosim_frame, columns) for name, columns in FEATURE_SETS.items()}
    synthetic_frame = pd.read_csv(HERE / "data_ref/drone_network_telemetry_dataset.csv")
    synthetic = evaluate(synthetic_frame)
    comparison = [(model, dataset, name, evaluate(frame, columns, model))
                  for model in MODELS
                  for dataset, frame, sets in (("Co-simulation", cosim_frame, FEATURE_SETS),
                                               ("Synthetic reference", synthetic_frame,
                                                {"full feature set": None}))
                  for name, columns in sets.items()]

    report = ["# Baseline machine-learning metrics", "",
              "| Dataset | Feature set | Accuracy | Macro-F1 |", "|---|---|---:|---:|"]
    report += [f"| Co-simulation | {name} | {r['accuracy']:.4f} | {r['macro_f1']:.4f} |"
               for name, r in cosim.items()]
    report += [f"| Synthetic reference | full feature set | {synthetic['accuracy']:.4f} | "
               f"{synthetic['macro_f1']:.4f} |", ""]
    report += section("Co-simulation dataset, full feature set", cosim["full feature set"])
    report += section("Co-simulation dataset, raw metrics only", cosim["raw metrics only"])
    report += section("Synthetic reference, full feature set", synthetic)
    report += ["## Model comparison", "",
               "Same splits and feature sets, three models:", "",
               "| Model | Dataset | Feature set | Accuracy | Macro-F1 |", "|---|---|---|---:|---:|"]
    report += [f"| {model} | {dataset} | {name} | {r['accuracy']:.4f} | {r['macro_f1']:.4f} |"
               for model, dataset, name, r in comparison]
    (HERE / "reports/model_metrics.md").write_text("\n".join(report) + "\n")

    for name, result in cosim.items():
        print(f"[CO-SIMULATION] {name}: accuracy={result['accuracy']:.4f} "
              f"macro-F1={result['macro_f1']:.4f}")
    print(f"[SYNTHETIC] full feature set: accuracy={synthetic['accuracy']:.4f} "
          f"macro-F1={synthetic['macro_f1']:.4f}")
    for model, dataset, name, r in comparison:
        print(f"[{model.upper()}] {dataset}, {name}: accuracy={r['accuracy']:.4f} "
              f"macro-F1={r['macro_f1']:.4f}")
    print("[WROTE] reports/model_metrics.md")


if __name__ == "__main__":
    main()
