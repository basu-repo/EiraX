#!/usr/bin/env python3
"""Join the manifest, the ROS run and the Simu5G metrics into the 44-column CSV.

One row per flow per aggregation window. Every telemetry value is measured from
the Simu5G vectors; the manifest supplies only the labels, and only after the
measurements have been taken.
"""

import argparse
import csv
import json
import math
import statistics
from datetime import datetime, timedelta, timezone
from pathlib import Path

import yaml

EPOCH = datetime(2026, 1, 1, tzinfo=timezone.utc)
UE_IP = "10.0.0.1"
EDGE_IP = "10.0.0.2"

ALLOWED_RESPONSES = {
    "none": "continue_monitoring",
    "unknown_anomaly": "increase_monitoring",
    "denial_of_service": "rate_limit_and_block_flow",
    "session_hijacking": "revoke_session_force_reauth",
    "identity_theft": "suspend_identity_quarantine_device",
    "control_protocol_attack": "block_command_channel_safe_mode",
    "lateral_movement": "segment_slice_isolate_component",
    "network_application_attack": "rate_limit_api_alert_security_team",
}


# The document's rule: a burst is a packet rate well above the benign baseline.
def traffic_burst_flag(packets_per_second, baseline_pps):
    return int(packets_per_second > 3.0 * baseline_pps)


# The document's rule: a destination outside the mission's allowed set.
def unusual_destination_flag(destination, known_destinations):
    return int(destination not in known_destinations)


# The document's rule: one session identifier seen from a second source.
def session_reuse_flag(session_id, source_ip, seen_sessions):
    old_source = seen_sessions.get(session_id)
    seen_sessions.setdefault(session_id, source_ip)
    return int(old_source is not None and old_source != source_ip)


# Read one named vector for one module out of the opp_scavetool export.
def vector(rows, module, name):
    for row in rows:
        if row["module"] == module and row["name"] == name:
            return [(float(t), float(v)) for t, v in
                    zip(row["vectime"].split(), row["vecvalue"].split())]
    return []


def within(samples, start, end):
    return [value for stamp, value in samples if start <= stamp < end]


# The label window covering this second. Labels are read only here.
def label_for(manifest, second):
    for window in manifest["label_windows"]:
        if float(window["start_sec"]) <= second < float(window["end_sec"]):
            return window
    raise ValueError(f"no label window covers second {second}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--window", type=float, default=1.0)
    parser.add_argument("--ros-log", required=True)
    parser.add_argument("--net-csv", required=True)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--flows")
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    manifest = yaml.safe_load(Path(args.manifest).read_text())
    # Defaults to the contract the scenario wrote beside its results, so the
    # document's own command line works without naming it.
    net_csv = Path(args.net_csv)
    flows_path = Path(args.flows) if args.flows else (
        net_csv.parent / net_csv.name.replace("_net.csv", "") / "network/flow_contract.json")
    flows = json.loads(flows_path.read_text())
    schema = json.loads((Path(__file__).resolve().parents[2] /
                         "schema_reference.json").read_text())
    allowed = set(manifest["allowed_destinations"])
    duration = int(manifest["simulation"]["duration_sec"])

    csv.field_size_limit(2 ** 31 - 1)
    with open(net_csv, encoding="utf-8", errors="replace") as stream:
        vectors = [r for r in csv.DictReader(stream) if r.get("type") == "vector"]

    # Radio measurements belong to a drone, not to a flow, so they are read
    # once per drone and shared by that drone's flows.
    drones = sorted({int(f.get("ue", 0)) for f in flows})
    radio = {}
    for drone in drones:
        nic = f"EiraXExperiment.ue[{drone}].cellularNic"
        radio[drone] = {
            "sinr": vector(vectors, f"{nic}.nrChannelModel[0]", "measuredSinrUl:vector"),
            "harq": vector(vectors, f"{nic}.nrMac", "harqErrorRateUl:vector"),
            "serving": vector(vectors, f"{nic}.nrPhy", "servingCell:vector"),
        }

    # The burst threshold comes from the benign flows, never from a label.
    baseline_pps = max(f["rate"] for f in flows if "start" not in f)

    seen_sessions = {}
    cell = {d: int(radio[d]["serving"][0][1]) if radio[d]["serving"] else 1 for d in drones}
    handovers = dict.fromkeys(drones, 0)
    rows = []

    for step in range(int(duration / args.window)):
        start = step * args.window
        end = start + args.window
        changed = {}
        for drone in drones:
            seen = [int(v) for v in within(radio[drone]["serving"], start, end)]
            changed[drone] = sum(a != b for a, b in zip([cell[drone]] + seen, seen))
            handovers[drone] += changed[drone]
            if seen:
                cell[drone] = seen[-1]
        label = label_for(manifest, start)

        for flow in flows:
            if not (float(flow.get("start", 0)) <= start < float(flow.get("end", duration))):
                continue
            drone = int(flow.get("ue", 0))
            index = int(flow.get("slot", flows.index(flow)))
            sinr, harq = radio[drone]["sinr"], radio[drone]["harq"]
            # The 5G leg terminates at the edge node, so that is where the
            # radio delay and volume are measured.
            module = f"EiraXExperiment.edgeNode.app[{index * 2}]"
            received = within(vector(vectors, module, "cbrReceivedBytes:vector"), start, end)
            delays = [d * 1000 for d in
                      within(vector(vectors, module, "cbrFrameDelay:vector"), start, end)]
            expected = max(1, round(flow["rate"] * args.window))
            packets = len(received)
            rate = packets / args.window
            activity = flow["activity"]

            rows.append({
                "timestamp": (EPOCH + timedelta(seconds=start)).isoformat(),
                "organization_id": manifest["organization_id"],
                "fleet_id": manifest["fleet_id"],
                "drone_id": flow.get("claimed_drone_id", manifest["drone_id"]),
                "mission_id": manifest["mission_id"],
                "domain": manifest["domain"],
                "mission_type": manifest["mission_type"],
                "source_component": flow.get("source_component", "drone"),
                "destination_component": flow.get("destination_component", "edge_node"),
                "source_ip": flow.get("source_ip", UE_IP),
                "destination_ip": flow.get("destination_ip", EDGE_IP),
                "source_port": flow["source_port"],
                "destination_port": flow["destination_port"],
                "protocol": flow["protocol"],
                "network_slice_id": flow.get("network_slice_id", "slice_1"),
                "cell_id": f"gnb_{cell[drone]}",
                "session_id": flow["session_id"],
                "packet_count": packets,
                "byte_count": int(sum(received)),
                "packets_per_second": rate,
                "throughput_kbps": sum(received) * 8 / args.window / 1000,
                "latency_ms": statistics.fmean(delays) if delays else 0.0,
                "jitter_ms": statistics.pstdev(delays) if len(delays) > 1 else 0.0,
                "packet_loss_rate": max(0.0, min(1.0, (expected - packets) / expected)),
                "retransmission_rate": min(1.0, statistics.fmean(
                    within(harq, start, end) or [0.0])),
                "connection_duration_sec": math.ceil(end - float(flow.get("start", 0))),
                "authentication_status": "failed" if flow.get("auth_failed") else "success",
                "handover_event": int(changed[drone] > 0),
                "handover_count": handovers[drone],
                "signal_quality_dbm": max(-200.0, min(0.0, -97.0 + statistics.fmean(
                    within(sinr, start, end) or [0.0]))),
                "command_channel_activity": packets if activity == "command" else 0,
                "video_stream_activity": int(activity == "video" and packets > 0),
                "telemetry_stream_activity": int(activity == "telemetry" and packets > 0),
                "api_request_count": packets if activity == "api_request" else 0,
                "failed_connection_attempts": packets if flow.get("auth_failed") or flow.get("unanswered") else 0,
                "unusual_destination_flag": unusual_destination_flag(
                    flow.get("destination_ip", EDGE_IP), allowed),
                "session_reuse_flag": session_reuse_flag(
                    flow["session_id"], flow.get("source_ip", UE_IP), seen_sessions),
                "traffic_burst_flag": traffic_burst_flag(rate, baseline_pps),
                "anomaly_score": 0.0,
                "attack_type": label["attack_type"],
                "attack_stage": label["attack_stage"],
                "severity": label["severity"],
                "recommended_response": ALLOWED_RESPONSES[label["attack_type"]],
                "incident_label": label["incident_label"],
            })

    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=schema["columns"])
        writer.writeheader()
        writer.writerows(rows)
    print(f"ASSEMBLED: {out} ({len(rows)} rows, {len(schema['columns'])} columns)")


if __name__ == "__main__":
    main()
