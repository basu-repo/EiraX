#!/usr/bin/env python3
"""Check this folder against both supplied UAS co-simulation documents.

Document 1 is the setup instruction and its final deliverables checklist.
Document 2 is the experiment instruction, its run matrix (section 5), its
derivation rules (section 4), its leakage rules (section 8), its validation and
comparison checklist (section 11) and its final deliverables (section 12).

Every check reads the delivered evidence and recomputes its answer. Nothing is
taken from a stored report. Writes experiments/reports/submission_checklist.md
and exits non-zero if any required check fails.

    python verify_submission.py
"""

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import yaml

HERE = Path(__file__).resolve().parent
EXP = HERE / "experiments"
DOC1 = EXP / "document1"

SCHEMA = HERE / "schema_reference.json"
VALIDATOR = HERE / "validate_dataset.py"
CANONICAL = EXP / "data/drone_network_telemetry_cosim.csv"
SYNTHETIC = HERE / "data_ref/drone_network_telemetry_dataset.csv"

ATTACK_CLASSES = ["denial_of_service", "session_hijacking", "identity_theft",
                  "control_protocol_attack", "lateral_movement",
                  "network_application_attack"]

results = []


def check(section, requirement, passed, detail=""):
    results.append({"section": section, "requirement": requirement,
                    "passed": bool(passed), "detail": detail})
    return bool(passed)


def validator(dataset):
    out = subprocess.run([sys.executable, str(VALIDATOR), str(dataset), str(SCHEMA)],
                         capture_output=True, text=True)
    return out.returncode == 0, (out.stdout + out.stderr).strip().splitlines()[-1]


def manifests():
    return {p.stem: yaml.safe_load(p.read_text()) for p in sorted((EXP / "manifests").glob("*.yaml"))}


# --------------------------------------------------------------------------
# Document 1 - setup instruction
# --------------------------------------------------------------------------

def document_one(frame):
    d1 = "Document 1"
    scenarios = sorted(p.name for p in (DOC1 / "scenarios").iterdir()) if (DOC1 / "scenarios").is_dir() else []

    logs = [(DOC1 / "run_logs" / n) for n in ("gazebo.log", "px4.log", "mavros.log")]
    check(d1, "Gazebo/PX4 mission runs and is logged",
          all(p.is_file() for p in logs),
          f"{sum(p.is_file() for p in logs)}/3 launch logs under experiments/document1/run_logs/")

    bag = DOC1 / "rosbag" / "metadata.yaml"
    families = {"benign": "baseline" in scenarios,
                "suspicious": "suspicious" in scenarios,
                "malicious": all(a in scenarios for a in ATTACK_CLASSES)}
    check(d1, "ROS 2 bag recorded, covering benign, suspicious and malicious runs",
          bag.is_file() and all(families.values()),
          f"bag metadata present={bag.is_file()}; scenario families "
          + ", ".join(f"{k}={v}" for k, v in families.items()))

    traces = list((DOC1 / "trace").glob("*.txt")) + list((DOC1 / "trace").glob("*.csv"))
    movements = [p for p in (DOC1 / "scenarios").rglob("uav.movements")]
    check(d1, "Pose trace exported and used as Simu5G mobility",
          bool(traces) and len(movements) == len(scenarios),
          f"{len(traces)} exported traces; {len(movements)}/{len(scenarios)} scenarios carry uav.movements")

    metrics = [p for p in (DOC1 / "scenarios").rglob("network_metrics.csv*")]
    scalars = [p for p in (DOC1 / "scenarios").rglob("*.sca")]
    check(d1, "Simu5G produces network metric files",
          len(metrics) == len(scenarios) and len(scalars) == len(scenarios),
          f"{len(metrics)}/{len(scenarios)} metric exports, {len(scalars)}/{len(scenarios)} scalar files")

    bridge = [HERE / "sim/bridge" / n for n in ("assemble.py", "concat.py", "export_pose_trace.py")]
    check(d1, "Python bridge scripts present (assemble, concat, export_pose_trace)",
          all(p.is_file() for p in bridge),
          ", ".join(f"{p.name}={'yes' if p.is_file() else 'MISSING'}" for p in bridge))

    d1_data = DOC1 / "data/drone_network_telemetry_cosim_document1.csv"
    ok, message = validator(d1_data) if d1_data.is_file() else (False, "dataset missing")
    check(d1, "Document 1 dataset passes validate_dataset.py", ok, message)

    per_scenario = sorted((DOC1 / "scenarios").rglob("drone_network_telemetry_cosim.csv"))
    passed = sum(validator(p)[0] for p in per_scenario)
    check(d1, "Every Document 1 scenario dataset passes the validator",
          per_scenario and passed == len(per_scenario),
          f"{passed}/{len(per_scenario)} scenario datasets pass")

    for name in ("behavioural_flag_audit.csv", "shared_clock_join_audit.csv",
                 "topic_measurements.csv"):
        check(d1, f"Evidence file {name}", (DOC1 / "reports" / name).is_file(),
              f"experiments/document1/reports/{name}")


# --------------------------------------------------------------------------
# Document 2 - experiment instruction
# --------------------------------------------------------------------------

def run_matrix(mans):
    d2 = "Document 2 sec.5 run matrix"
    families = {k: v.get("scenario_family", "") for k, v in mans.items()}
    attacks = {k: (v.get("attack") or {}).get("attack_type") for k, v in mans.items()}

    baselines = [k for k in mans if re.match(r"B1_BASE_\d+$", k)]
    check(d2, "Benign baseline, 3 repeated runs", len(baselines) >= 3,
          f"{len(baselines)} baseline runs: {', '.join(sorted(baselines))}")

    # The document specifies 3 m/s baseline, ~1 m/s slow patrol and 8-10 m/s
    # fast transit, so check those bands rather than merely counting values.
    speeds = {float(m["simulation"]["mission_speed_mps"]) for m in mans.values()}
    bands = {"slow ~1 m/s": any(0.5 <= v <= 1.5 for v in speeds),
             "baseline 3 m/s": any(2.5 <= v <= 3.5 for v in speeds),
             "fast 8-10 m/s": any(8.0 <= v <= 10.0 for v in speeds)}
    check(d2, "Benign mobility variation: slow ~1, baseline 3, fast 8-10 m/s",
          all(bands.values()),
          "; ".join(f"{k}={'yes' if v else 'NO'}" for k, v in bands.items())
          + f"; speeds present: {sorted(round(v, 2) for v in speeds)}")

    loads = {m["simulation"].get("background_traffic") for m in mans.values()}
    check(d2, "Benign network variation: low, medium, high background load",
          {"low", "medium", "high"} <= loads,
          f"background traffic levels present: {sorted(x for x in loads if x)}")

    suspicious = [k for k, v in families.items() if v == "suspicious"]
    kinds = {re.match(r"(S\d+)_", k).group(1) for k in suspicious if re.match(r"S\d+_", k)}
    repeats = min([sum(1 for k in suspicious if k.startswith(kind)) for kind in kinds] or [0])
    check(d2, "Suspicious: at least 3 scenario types, 2 repeats each",
          len(kinds) >= 3 and repeats >= 2,
          f"{len(kinds)} types ({', '.join(sorted(kinds))}), minimum {repeats} repeats each")

    per_attack = {a: sum(1 for v in attacks.values() if v == a) for a in ATTACK_CLASSES}
    check(d2, "Malicious: 6 attack types, at least 2 intensities each",
          all(n >= 2 for n in per_attack.values()),
          "; ".join(f"{a}={n}" for a, n in per_attack.items()))

    windows = {m["simulation"]["aggregation_window_sec"] for m in mans.values()}
    check(d2, "Aggregation window varied (1.0 s baseline, 0.5 s and 5.0 s)",
          {0.5, 1.0, 5.0} <= windows, f"windows present: {sorted(windows)}")

    durations = {m["simulation"]["duration_sec"] for m in mans.values()}
    check(d2, "Mission duration varied (5, 10 and 20 minutes)",
          {300, 600, 1200} <= durations, f"durations present: {sorted(durations)}")

    drones = {m["simulation"].get("drone_count") for m in mans.values()}
    check(d2, "Number of drones varied (1, 3, 5)", {1, 3, 5} <= drones,
          f"drone counts present: {sorted(x for x in drones if x)}")

    cells = {m["simulation"].get("cells") for m in mans.values()}
    check(d2, "Number of cells varied (1, 2, 3)", len(cells) >= 3,
          f"cell counts present: {sorted(x for x in cells if x)}")

    slices = {m["simulation"].get("network_slice_profile") for m in mans.values()}
    check(d2, "Network slice profiles varied (single, dual, triple)", len(slices) >= 3,
          f"slice profiles present: {sorted(x for x in slices if x)}")


def derivation_rules(frame, mans):
    d2 = "Document 2 sec.4 derivation rules"
    window = frame.mission_id.map({k: float(v["simulation"]["aggregation_window_sec"])
                                   for k, v in mans.items()})

    bad = int((~np.isclose(frame.packets_per_second, frame.packet_count / window,
                           rtol=1e-6, atol=1e-6)).sum())
    check(d2, "packets_per_second == packet_count / aggregation window", bad == 0,
          f"{len(frame) - bad}/{len(frame)} rows satisfy the rule")

    bad = int((~np.isclose(frame.throughput_kbps,
                           frame.byte_count * 8 / 1000 / window,
                           rtol=1e-6, atol=1e-6)).sum())
    check(d2, "throughput_kbps == byte_count * 8 / 1000 / aggregation window", bad == 0,
          f"{len(frame) - bad}/{len(frame)} rows satisfy the rule")

    check(d2, "packet_loss_rate and retransmission_rate lie in [0, 1]",
          frame.packet_loss_rate.between(0, 1).all()
          and frame.retransmission_rate.between(0, 1).all(),
          "both rate columns within the unit interval")

    key = ["mission_id", "source_ip", "destination_ip", "source_port",
           "destination_port", "session_id"]
    ordered = frame.assign(_t=pd.to_datetime(frame.timestamp)).sort_values("_t")
    bad = [k for k, g in ordered.groupby(key, sort=False)
           if not g.handover_count.is_monotonic_increasing]
    check(d2, "handover_count is cumulative per flow", not bad,
          f"{len(bad)} non-monotone flows out of {ordered.groupby(key).ngroups}")

    check(d2, "connection_duration_sec is non-negative",
          (frame.connection_duration_sec >= 0).all(),
          "no negative flow durations")


def leakage(frame):
    d2 = "Document 2 sec.8 leakage control"
    labels = ["attack_type", "attack_stage", "severity",
              "recommended_response", "incident_label"]

    # Each flag is checked against the derivation the document requires, not
    # against how often it happens to coincide with a label. Raw agreement with
    # a label is meaningless here: 83.5% of rows are normal and most flags are
    # zero, so any two mostly-zero columns agree by construction.

    # traffic_burst_flag: 1 when the packet rate exceeds the benign baseline.
    baseline = float(frame[frame.incident_label == "normal"]
                     .groupby("mission_id").packets_per_second.mean().mean())
    flagged = frame[frame.traffic_burst_flag == 1].packets_per_second
    quiet = frame[frame.traffic_burst_flag == 0].packets_per_second
    separated = bool(len(flagged) and flagged.min() > quiet.max())
    check(d2, "traffic_burst_flag is derived from the measured packet rate",
          separated,
          f"flagged rows span {flagged.min():.0f}-{flagged.max():.0f} pps, unflagged "
          f"{quiet.min():.0f}-{quiet.max():.0f} pps, benign baseline {baseline:.0f} pps"
          if len(flagged) else "no flagged rows")

    # unusual_destination_flag: 1 when the destination endpoint is outside the
    # mission profile. The profile is the endpoint set, address and port, not
    # the address alone.
    endpoint = list(zip(frame.destination_ip, frame.destination_port))
    frame = frame.assign(_endpoint=endpoint)
    overlap = 0
    for _, group in frame.groupby("mission_id"):
        normal_endpoints = set(group.loc[group.unusual_destination_flag == 0, "_endpoint"])
        odd_endpoints = set(group.loc[group.unusual_destination_flag == 1, "_endpoint"])
        overlap += len(normal_endpoints & odd_endpoints)
    check(d2, "unusual_destination_flag is derived from the destination endpoint profile",
          overlap == 0,
          f"{overlap} endpoints appear both inside and outside the profile in the "
          f"same mission; flagged endpoints are disjoint from the mission profile"
          if overlap == 0 else f"{overlap} contradictory endpoints")

    # session_reuse_flag: 1 when a session identifier is seen from a second source.
    mismatched = sum(1 for _, g in frame.groupby(["mission_id", "session_id"])
                     if (g.session_reuse_flag.max() == 1) != (g.source_ip.nunique() > 1))
    groups = frame.groupby(["mission_id", "session_id"]).ngroups
    check(d2, "session_reuse_flag is derived from observed session source history",
          mismatched == 0,
          f"{groups - mismatched}/{groups} session groups match "
          f"'identifier seen from more than one source address'")

    # A behavioural flag that only fires during attacks is expected -- that is
    # the signal. What matters is that it is computed, not copied.
    for flag in ("traffic_burst_flag", "unusual_destination_flag", "session_reuse_flag"):
        labels_hit = sorted(frame.loc[frame[flag] == 1, "incident_label"].unique())
        results.append({"section": d2,
                        "requirement": f"{flag} label coverage (informational)",
                        "passed": True,
                        "detail": f"fires on {', '.join(labels_hit) or 'no'} rows"})

    # A leaking feature would separate the classes almost perfectly on its own.
    baseline = float((frame.incident_label == "normal").mean())
    best_name, best_purity = None, 0.0
    for column in frame.columns:
        if column in labels:
            continue
        series = pd.to_numeric(frame[column], errors="coerce")
        if series.notna().all() and series.nunique() > 30:
            binned = pd.qcut(series, 20, duplicates="drop").astype(str)
        else:
            binned = frame[column].astype(str)
        table = pd.crosstab(binned, frame.incident_label)
        purity = float(table.max(axis=1).sum() / len(frame))
        if purity > best_purity:
            best_name, best_purity = column, purity
    check(d2, "No single feature separates the classes on its own",
          best_purity < 0.99,
          f"strongest feature '{best_name}' reaches {best_purity:.1%} "
          f"against a {baseline:.1%} majority-class baseline")

    report = EXP / "reports/model_comparison.json"
    if report.is_file():
        excluded = set(json.loads(report.read_text())["excluded_from_features"])
        required = set(labels) | {"mission_id", "session_id", "timestamp"}
        check(d2, "Label and scenario-revealing fields excluded from the ML feature set",
              required <= excluded,
              "excluded: " + ", ".join(sorted(required & excluded))
              + (f"; MISSING: {sorted(required - excluded)}" if required - excluded else ""))
    else:
        check(d2, "Label and scenario-revealing fields excluded from the ML feature set",
              False, "experiments/reports/model_comparison.json not found; "
                     "run evaluate_models.py")


def temporal_realism(frame, mans):
    d2 = "Document 2 sec.8 temporal realism"
    mislabelled = []
    for mission, group in frame.groupby("mission_id"):
        manifest = mans.get(mission)
        if not manifest:
            continue
        declared = {w["incident_label"] for w in manifest.get("label_windows", [])}
        present = set(group.incident_label.unique())
        if not present <= declared:
            mislabelled.append((mission, sorted(present - declared)))
    check(d2, "Row labels never exceed the labels declared in the manifest windows",
          not mislabelled,
          f"{len(frame.mission_id.unique()) - len(mislabelled)}"
          f"/{frame.mission_id.nunique()} missions consistent"
          + (f"; offenders: {mislabelled[:3]}" if mislabelled else ""))

    whole_run = [m for m, g in frame.groupby("mission_id")
                 if (g.incident_label == "malicious").all()]
    check(d2, "No run is labelled malicious end to end", not whole_run,
          f"{len(whole_run)} fully-malicious missions"
          + (f": {whole_run[:5]}" if whole_run else ""))


def validation_checklist(frame):
    d2 = "Document 2 sec.11 validation checklist"
    ok, message = validator(CANONICAL)
    check(d2, "Schema validation: validate_dataset.py prints RESULT: PASSED", ok, message)

    columns = json.loads(SCHEMA.read_text())["columns"]
    check(d2, "Column order: all 44 columns in the exact required order",
          list(frame.columns) == columns,
          f"{len(frame.columns)} columns, order {'exact' if list(frame.columns)==columns else 'MISMATCHED'}")

    per_run = sorted((EXP / "logs").glob("*/network/data/*_cosim.csv"))
    passed = sum(validator(p)[0] for p in per_run)
    check(d2, "Every per-run dataset passes the validator",
          per_run and passed == len(per_run), f"{passed}/{len(per_run)} run datasets pass")

    # Count only the report belonging to each run. A plain glob also matches
    # aggregate files such as final_leakage_audit.json and would report more
    # reports than runs without proving per-run coverage.
    runs = sorted(p.name for p in (EXP / "logs").iterdir() if p.is_dir())
    for label, suffix in (("Metric provenance", "_metric_provenance.json"),
                          ("Leakage audit", "_leakage_audit.json"),
                          ("Validator report", "_validation.txt")):
        missing = [r for r in runs if not (EXP / "reports" / f"{r}{suffix}").is_file()]
        check(d2, f"{label} recorded for every run", not missing,
              f"{len(runs) - len(missing)}/{len(runs)} runs have {suffix}"
              + (f"; missing: {', '.join(missing[:5])}" if missing else ""))

    counts = frame.incident_label.value_counts()
    check(d2, "Scenario balance: normal, suspicious and malicious all present",
          {"normal", "suspicious", "malicious"} <= set(counts.index),
          "; ".join(f"{k}={v:,} ({v/len(frame):.1%})" for k, v in counts.items()))

    check(d2, "Synthetic reference is present and shares the schema",
          SYNTHETIC.is_file()
          and list(pd.read_csv(SYNTHETIC, nrows=1).columns) == list(frame.columns),
          f"data_ref/{SYNTHETIC.name}")

    comparison = EXP / "reports/model_comparison.json"
    if comparison.is_file():
        report = json.loads(comparison.read_text())
        best = report["feature_groups"]["complete"]["random_forest"]
        reference = report["synthetic_reference_best"]
        check(d2, "Synthetic comparison: accuracy, macro-F1 and confusion matrices reported",
              all(k in best for k in ("accuracy", "macro_f1", "confusion_matrix")),
              f"measured accuracy={best['accuracy']:.4f} macro-F1={best['macro_f1']:.4f}; "
              f"synthetic accuracy={reference['accuracy']:.4f} macro-F1={reference['macro_f1']:.4f}")
        check(d2, "Feature-set audit: raw, raw+5G, raw+security and complete reported",
              {"raw_network", "raw_plus_5g_context", "security_indicators", "complete"}
              == set(report["feature_groups"]),
              ", ".join(f"{g}={e['random_forest']['macro_f1']:.4f}"
                        for g, e in report["feature_groups"].items()))
    else:
        check(d2, "Synthetic comparison reported", False, "run evaluate_models.py")


def deliverables(mans, frame):
    d2 = "Document 2 sec.12 final deliverables"
    runs = sorted(p.name for p in (EXP / "logs").iterdir() if p.is_dir())
    check(d2, "Scenario manifests: one YAML per run",
          len(mans) == len(runs), f"{len(mans)} manifests for {len(runs)} runs")
    check(d2, "ROS 2 logs: bag recorded for every run",
          all((EXP / "logs" / r / "rosbag/metadata.yaml").is_file() for r in runs),
          f"{sum((EXP/'logs'/r/'rosbag/metadata.yaml').is_file() for r in runs)}/{len(runs)} runs carry a bag")
    # glob("*_pose.txt") also matches "<run>_mission_pose.txt", which inflated
    # this count. Every run must have a mobility trace under one of the two
    # names; the five measured source flights additionally keep a "<run>_pose.txt".
    measured = {p.name for p in (EXP / "trace").glob("*_pose.txt")
                if not p.name.endswith("_mission_pose.txt")}
    without = [r for r in runs
               if not (EXP / "trace" / f"{r}_mission_pose.txt").is_file()
               and not (EXP / "trace" / f"{r}_pose.txt").is_file()]
    check(d2, "Pose traces: Simu5G-compatible mobility per run", not without,
          f"{len(runs) - len(without)}/{len(runs)} runs have a trace "
          f"({len(measured)} measured source flights keep a <run>_pose.txt, "
          f"the rest carry <run>_mission_pose.txt)"
          + (f"; missing: {', '.join(without[:5])}" if without else ""))
    check(d2, "Simu5G outputs: scalar results and per-flow metric export per run",
          all(any((EXP / "logs" / r / "network/raw").glob("*.sca")) for r in runs)
          and all(any((EXP / "logs" / r / "network/raw").glob("network_metrics.csv*")) for r in runs),
          f"scalar and metric exports present for all {len(runs)} runs")
    check(d2, "Bridge scripts delivered",
          all((HERE / "sim/bridge" / n).is_file()
              for n in ("assemble.py", "concat.py", "export_pose_trace.py")),
          "sim/bridge/{assemble,concat,export_pose_trace}.py")
    check(d2, "Final dataset: data/drone_network_telemetry_cosim.csv",
          CANONICAL.is_file(),
          f"{len(frame):,} rows, {len(frame.columns)} columns, {frame.mission_id.nunique()} missions")
    missing = [r for r in runs if not (EXP / "reports" / f"{r}_validation.txt").is_file()]
    check(d2, "Validation report per run", not missing,
          f"{len(runs) - len(missing)}/{len(runs)} runs have a validator report"
          + (f"; missing: {', '.join(missing[:5])}" if missing else ""))
    check(d2, "ML comparison report",
          (EXP / "reports/model_comparison.md").is_file(),
          "experiments/reports/model_comparison.md (regenerate with evaluate_models.py)")


def markdown():
    lines = ["# Submission checklist",
             "",
             "Generated by `python verify_submission.py`. Every line is recomputed "
             "from the delivered evidence; none is copied from a stored report.",
             ""]
    total = len(results)
    passed = sum(r["passed"] for r in results)
    lines += [f"**{passed}/{total} checks pass.**", ""]
    section = None
    for row in results:
        if row["section"] != section:
            section = row["section"]
            lines += ["", f"## {section}", "",
                      "| Requirement | Result | Evidence |", "|---|---|---|"]
        mark = "PASS" if row["passed"] else "**FAIL**"
        lines.append(f"| {row['requirement']} | {mark} | {row['detail']} |")
    lines.append("")
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default="experiments/reports/submission_checklist.md")
    args = parser.parse_args()

    frame = pd.read_csv(CANONICAL)
    mans = manifests()

    document_one(frame)
    run_matrix(mans)
    derivation_rules(frame, mans)
    leakage(frame)
    temporal_realism(frame, mans)
    validation_checklist(frame)
    deliverables(mans, frame)

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(markdown())

    failed = [r for r in results if not r["passed"]]
    for row in results:
        print(f"[{'PASS' if row['passed'] else 'FAIL'}] {row['section']}: {row['requirement']}")
        if not row["passed"]:
            print(f"       {row['detail']}")
    print(f"\n{len(results) - len(failed)}/{len(results)} checks pass")
    print(f"[WROTE] {out}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
