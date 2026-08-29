#!/usr/bin/env python3
"""Assemble one 44-column UAS telemetry CSV from measured Simu5G vectors.

Labels are read from the scenario manifest only after all telemetry features have
been derived. The script is standalone; the experiment notebook does not import it.
"""

import argparse
import csv
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path

import numpy as np
import pandas as pd
import yaml


def arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--ros-log", required=True)
    parser.add_argument("--net-csv", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--out", required=True)
    parser.add_argument("--schema")
    parser.add_argument("--flow-contract")
    parser.add_argument("--bridge-log")
    return parser.parse_args()


def vector_rows(path):
    csv.field_size_limit(2**31 - 1)
    with path.open(encoding="utf-8", errors="replace") as stream:
        return [row for row in csv.DictReader(stream) if row.get("type") == "vector"]


def vector_values(vectors, module, name):
    for row in vectors:
        if row.get("module") == module and row.get("name") == name:
            times = row.get("vectime", "").split()
            values = row.get("vecvalue", "").split()
            return [(float(stamp), float(value)) for stamp, value in zip(times, values)]
    return []


def values_in(samples, start, end):
    return [value for stamp, value in samples if start <= stamp < end]


def label_for(manifest, second):
    for window in manifest["label_windows"]:
        if float(window["start_sec"]) <= second < float(window["end_sec"]):
            return window
    raise ValueError(f"no label window covers second {second}")


def infer_contract(net_csv, explicit):
    if explicit:
        return Path(explicit)
    candidate = net_csv.parent.parent / "input" / "flow_contract.json"
    if not candidate.is_file():
        raise FileNotFoundError("flow contract not found; pass --flow-contract")
    return candidate


def main():
    args = arguments()
    if args.window <= 0:
        raise ValueError("--window must be positive")
    ros_log = Path(args.ros_log)
    net_csv = Path(args.net_csv)
    manifest_path = Path(args.manifest)
    output = Path(args.out)
    for required in (ros_log, net_csv, manifest_path):
        if not required.exists():
            raise FileNotFoundError(required)

    root = Path(__file__).resolve().parents[2]
    schema_path = Path(args.schema) if args.schema else root / "schema_reference.json"
    schema = json.loads(schema_path.read_text(encoding="utf-8"))
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    contract_path = infer_contract(net_csv, args.flow_contract)
    flows = json.loads(contract_path.read_text(encoding="utf-8"))
    controlled_path = contract_path.with_name("controlled_flow_contract.json")
    if controlled_path.is_file():
        flows.extend(json.loads(controlled_path.read_text(encoding="utf-8")))
    vectors = vector_rows(net_csv)
    duration = int(manifest["simulation"]["duration_sec"])
    if duration % args.window:
        raise ValueError("duration must be exactly divisible by aggregation window")

    sinr = vector_values(
        vectors, "EiraXExperiment.ue[0].cellularNic.nrChannelModel[0]",
        "measuredSinrUl:vector")
    serving = vector_values(
        vectors, "EiraXExperiment.ue[0].cellularNic.nrPhy",
        "servingCell:vector")
    harq = vector_values(
        vectors, "EiraXExperiment.ue[0].cellularNic.nrMac",
        "harqErrorRateUl:vector")
    allowed_destinations = set(manifest.get("allowed_destinations", []))
    seen_sessions = {}
    handover_total = 0
    previous_cell = int(serving[0][1]) if serving else 1
    epoch = datetime(2026, 1, 1, tzinfo=timezone.utc)
    rows = []

    for bin_index in range(int(duration / args.window)):
        start = bin_index * args.window
        end = start + args.window
        cell_samples = [int(value) for value in values_in(serving, start, end)]
        sequence = [previous_cell] + cell_samples
        handovers = sum(left != right for left, right in zip(sequence, sequence[1:]))
        handover_total += handovers
        if cell_samples:
            previous_cell = cell_samples[-1]
        label = label_for(manifest, start)

        for flow_index, flow in enumerate(flows):
            if "start" in flow and not float(flow["start"]) <= start < float(flow["end"]):
                continue
            receiver = int(flow.get("receiver_app", flow_index))
            module = flow.get(
                "receiver_module",
                f"EiraXExperiment.edgeNode.app[{receiver}]" if "start" in flow
                else f"EiraXExperiment.missionBackend.app[{receiver}]")
            sizes = vector_values(vectors, module, "cbrReceivedBytes:vector")
            delays = vector_values(vectors, module, "cbrFrameDelay:vector")
            received = values_in(sizes, start, end)
            delay_ms = [value * 1000 for value in values_in(delays, start, end)]
            sinr_window = values_in(sinr, start, end)
            harq_window = values_in(harq, start, end)
            expected = max(1, round(float(flow["rate"]) * args.window))
            packet_count = len(received)
            packet_rate = packet_count / args.window
            packet_loss = max(0.0, min(1.0, (expected - packet_count) / expected))
            retransmission = max(
                0.0, min(1.0, float(np.mean(harq_window)) if harq_window else 0.0))
            benign_rate = float(flow.get("baseline_rate", 100.0 if "start" in flow else flow["rate"]))
            burst = int(packet_rate > 3.0 * benign_rate)
            destination = flow.get("destination_ip", "10.0.0.2")
            source = flow.get("source_ip", "10.0.0.1")
            session = flow.get("session_id", f"flow-{flow_index}")
            old_source = seen_sessions.setdefault(session, source)
            activity = flow.get("activity", "network")
            anomaly = min(1.0, packet_loss + retransmission + 0.2 * burst)

            rows.append({
                "timestamp": (epoch + timedelta(seconds=start)).isoformat(),
                "organization_id": manifest["organization_id"],
                "fleet_id": manifest["fleet_id"],
                "drone_id": manifest["drone_id"],
                "mission_id": manifest["mission_id"],
                "domain": manifest["domain"],
                "mission_type": manifest["mission_type"],
                "source_component": flow.get("source_component", "drone"),
                "destination_component": flow.get("destination_component", "mission_backend"),
                "source_ip": source,
                "destination_ip": destination,
                "source_port": int(flow["source_port"]),
                "destination_port": int(flow["destination_port"]),
                "protocol": flow.get("protocol", "UDP"),
                "network_slice_id": flow.get("network_slice_id", "slice_1"),
                "cell_id": f"gnb_{previous_cell}",
                "session_id": session,
                "packet_count": packet_count,
                "byte_count": int(sum(received)),
                "packets_per_second": packet_rate,
                "throughput_kbps": sum(received) * 8 / args.window / 1000,
                "latency_ms": float(np.mean(delay_ms)) if delay_ms else 0.0,
                "jitter_ms": float(np.std(delay_ms)) if len(delay_ms) > 1 else 0.0,
                "packet_loss_rate": packet_loss,
                "retransmission_rate": retransmission,
                "connection_duration_sec": int(math.ceil(end)),
                "authentication_status": flow.get("authentication_status", "success"),
                "handover_event": int(handovers > 0),
                "handover_count": handover_total,
                "signal_quality_dbm": max(
                    -200.0, min(0.0, -97.0 + (float(np.mean(sinr_window)) if sinr_window else 0.0))),
                "command_channel_activity": packet_count if activity == "command" else 0,
                "video_stream_activity": int(activity == "video" and packet_count > 0),
                "telemetry_stream_activity": int(activity == "telemetry" and packet_count > 0),
                "api_request_count": packet_count if activity == "api" else 0,
                "failed_connection_attempts": packet_count if activity == "failed_connection" else 0,
                "unusual_destination_flag": int(destination not in allowed_destinations),
                "session_reuse_flag": int(old_source != source),
                "traffic_burst_flag": burst,
                "anomaly_score": round(anomaly, 6),
                "attack_type": label["attack_type"],
                "attack_stage": label["attack_stage"],
                "severity": label["severity"],
                "recommended_response": label["recommended_response"],
                "incident_label": label["incident_label"],
            })

    frame = pd.DataFrame(rows, columns=schema["columns"])
    output.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(output, index=False)
    bridge_log = Path(args.bridge_log) if args.bridge_log else output.with_suffix(".bridge.json")
    bridge_log.write_text(json.dumps({
        "aggregation_window_sec": args.window,
        "inputs": {"ros_log": str(ros_log), "network_csv": str(net_csv),
                   "manifest": str(manifest_path), "flow_contract": str(contract_path)},
        "output": str(output), "rows": len(frame), "columns": len(frame.columns),
        "feature_order": "features derived before manifest labels",
        "derivations": {
            "packet_metrics": "Simu5G receiver vectors",
            "jitter_ms": "population standard deviation of delay samples",
            "packet_loss_rate": "expected packets from configured rate minus received packets",
            "retransmission_rate": "Simu5G uplink HARQ error vector",
            "signal_quality_dbm": "measured uplink SINR plus documented -97 dBm reference",
            "handover": "cumulative servingCell changes",
            "flags": "flow behavior and benign thresholds only"}}, indent=2), encoding="utf-8")
    print(f"ASSEMBLED: {output} ({len(frame)} rows, {len(frame.columns)} columns)")


if __name__ == "__main__":
    main()
