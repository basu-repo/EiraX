#!/usr/bin/env python3
"""Write the validation report: schema result, scenario balance, metric
provenance and the leakage audit of the final dataset.

The leakage audit recomputes the three behavioural flags of every run from
the manifest and the flow contract, independently of the bridge, and counts
rows where the dataset disagrees.

    ./audit_dataset.py
"""

import json
import subprocess
import sys
from pathlib import Path

import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
FINAL = HERE / "data/drone_network_telemetry_cosim.csv"
CLASSES = ["normal", "suspicious", "malicious"]

PROVENANCE = [
    ("timestamp", "bridge", "window start, ISO 8601 UTC from the simulation clock"),
    ("organization_id, fleet_id, drone_id, mission_id, domain, mission_type", "manifest", "copied; drone_id is the identity a flow claims"),
    ("source_component, destination_component, source_ip, destination_ip, source_port, destination_port, protocol, network_slice_id, session_id", "flow contract", "the flow's identity as configured in Simu5G"),
    ("cell_id", "Simu5G", "servingCell vector of the drone's NR PHY"),
    ("packet_count, byte_count", "Simu5G", "cbrReceivedBytes vector at the edge node, counted and summed per window"),
    ("packets_per_second", "bridge", "packet_count / window"),
    ("throughput_kbps", "bridge", "byte_count * 8 / 1000 / window"),
    ("latency_ms", "Simu5G", "mean of cbrFrameDelay in the window"),
    ("jitter_ms", "bridge", "population standard deviation of cbrFrameDelay in the window"),
    ("packet_loss_rate", "bridge", "(expected - received) / expected, expected = configured rate * window"),
    ("retransmission_rate", "Simu5G", "mean uplink HARQ error rate in the window"),
    ("connection_duration_sec", "bridge", "window end minus the flow's start"),
    ("authentication_status", "flow contract", "failed when the flow is configured as failing authentication, else success"),
    ("handover_event, handover_count", "bridge", "serving cell changed in the window; cumulative changes in the run"),
    ("signal_quality_dbm", "Simu5G", "-97 dBm plus mean uplink SINR in the window"),
    ("command_channel_activity, api_request_count", "bridge", "packets received on command and API flows"),
    ("video_stream_activity, telemetry_stream_activity", "bridge", "1 when a video or telemetry flow delivered packets in the window"),
    ("failed_connection_attempts", "bridge", "packets received on flows configured as unanswered or failing authentication"),
    ("unusual_destination_flag", "bridge", "destination not in the manifest's allowed_destinations"),
    ("session_reuse_flag", "bridge", "session_id already seen from a different source IP in the run"),
    ("traffic_burst_flag", "bridge", "packets_per_second above 3 x the highest mission flow rate"),
    ("anomaly_score", "bridge", "held at 0.0, optional"),
    ("attack_type, attack_stage, severity, recommended_response, incident_label", "manifest", "label window covering the window start, applied after every measurement"),
]


# Recompute the three flags for one run from its manifest and flow contract.
def recompute_flags(run, frame):
    manifest = yaml.safe_load((HERE / "manifests" / f"{run}.yaml").read_text())
    flows = json.loads((HERE / "logs" / run / "network/flow_contract.json").read_text())
    allowed = set(manifest["allowed_destinations"])
    baseline = 3.0 * max(f["rate"] for f in flows if "start" not in f)
    seen = {}
    reuse = []
    for session, source in zip(frame.session_id, frame.source_ip):
        reuse.append(int(session in seen and seen[session] != source))
        seen.setdefault(session, source)
    return {
        "unusual_destination_flag": (~frame.destination_ip.isin(allowed)).astype(int),
        "session_reuse_flag": pd.Series(reuse, index=frame.index),
        "traffic_burst_flag": (frame.packets_per_second > baseline).astype(int),
    }


def main():
    frame = pd.read_csv(FINAL)
    runs = frame.mission_id.drop_duplicates().tolist()
    family = {run: yaml.safe_load((HERE / "manifests" / f"{run}.yaml").read_text())["scenario_family"]
              for run in runs}

    validator = subprocess.run([sys.executable, "validate_dataset.py", str(FINAL),
                                "schema_reference.json"], cwd=HERE, capture_output=True, text=True)
    report = ["# Validation report", "", "## Schema validation", "", "```",
              validator.stdout.strip(), "```", "",
              "Per-run validator results:", ""]
    for run in runs:
        line = (HERE / "reports" / f"{run}_validation.txt").read_text().strip()
        report.append(f"- {run}: {line}")

    counts = frame.incident_label.value_counts()
    report += ["", "## Scenario balance", "", "| Label | Rows | Share |", "|---|---:|---:|"]
    report += [f"| {label} | {counts.get(label, 0)} | {counts.get(label, 0) / len(frame):.3f} |"
               for label in CLASSES]
    report += ["", "| Family | Runs | Rows | normal | suspicious | malicious |",
               "|---|---:|---:|---:|---:|---:|"]
    frame["family"] = frame.mission_id.map(family)
    for name, group in frame.groupby("family"):
        c = group.incident_label.value_counts()
        report.append(f"| {name} | {group.mission_id.nunique()} | {len(group)} | "
                      f"{c.get('normal', 0)} | {c.get('suspicious', 0)} | {c.get('malicious', 0)} |")
    report += ["", "| Run | Family | Rows | normal | suspicious | malicious |",
               "|---|---|---:|---:|---:|---:|"]
    for run in runs:
        c = frame[frame.mission_id == run].incident_label.value_counts()
        report.append(f"| {run} | {family[run]} | {c.sum()} | {c.get('normal', 0)} | "
                      f"{c.get('suspicious', 0)} | {c.get('malicious', 0)} |")

    report += ["", "## Metric provenance", "", "| Column(s) | Source | Derivation |", "|---|---|---|"]
    report += [f"| {columns} | {source} | {rule} |" for columns, source, rule in PROVENANCE]

    report += ["", "## Leakage audit", "",
               "The five label columns are read by the bridge only to stamp each window "
               "after its measurements are taken, and are excluded from the model inputs "
               "together with the timestamp and identifiers. The three behavioural flags "
               "are recomputed here from the manifest and flow contract of every run and "
               "compared with the dataset:", "",
               "| Run | unusual_destination mismatches | session_reuse mismatches | "
               "traffic_burst mismatches |", "|---|---:|---:|---:|"]
    total = 0
    for run in runs:
        rows = pd.read_csv(HERE / "data" / f"{run}_cosim.csv")
        expected = recompute_flags(run, rows)
        mismatches = {flag: int((rows[flag] != values).sum()) for flag, values in expected.items()}
        total += sum(mismatches.values())
        report.append(f"| {run} | {mismatches['unusual_destination_flag']} | "
                      f"{mismatches['session_reuse_flag']} | {mismatches['traffic_burst_flag']} |")
    report += ["", f"Total mismatches: {total}. "
               + ("Every flag is reproduced from behaviour alone." if total == 0
                  else "Some flags do not follow from the documented rules; inspect the runs above."),
               ""]

    (HERE / "reports/validation_report.md").write_text("\n".join(report))
    print(validator.stdout.strip())
    print(f"[BALANCE] " + ", ".join(f"{label}={counts.get(label, 0)}" for label in CLASSES))
    print(f"[LEAKAGE] flag mismatches={total}")
    print("[WROTE] reports/validation_report.md")
    return validator.returncode or int(total > 0)


if __name__ == "__main__":
    sys.exit(main())
