# Review prompt — 5G/UAS Co-Simulation project recheck

You are reviewing a nearly finished 5G/UAS co-simulation project (Gazebo + ROS 2 + PX4 SITL + OMNeT++/Simu5G) against its official instruction documents. Your job is to **audit, not to fix**. Do not modify, move, rename, or delete anything. Produce a written report only.

Work from the project root (default `~/uas_lab`; if the root is elsewhere, state the path you used). Inspect the actual filesystem, file contents, and notebook cells — do not assume anything is present because it "should" be.

For every item below, report one of: **PASS**, **FAIL**, **MISSING**, or **PARTIAL**, with the exact path(s) you checked and a one-line reason. Group the findings under the section headings below, and finish with a prioritized list of what must change before submission.

---

## 1. Folder structure

The instructions require this exact layout under the project root:

```
<root>/
├── schema_reference.json
├── validate_dataset.py
├── data/            # final generated co-simulation datasets
├── data_ref/        # reference synthetic Drone_cyber dataset
│   └── drone_network_telemetry_dataset.csv
├── logs/            # ROS 2 bags, Simu5G exports, bridge logs, run outputs
├── manifests/       # one YAML per run with label windows
├── results/         # (created by setup; report what it is used for)
├── sim/
│   ├── bridge/      # assemble.py, concat.py, export_pose_trace.py, helpers
│   └── simu5g/      # Simu5G scenario configs and OMNeT++ result files
├── trace/           # pose/mobility traces exported from ROS 2
└── reports/         # validation, model metrics, comparison results
```

Check:
- Every directory above exists at the top level (not nested under an extra folder, not renamed, not pluralised differently).
- `schema_reference.json` and `validate_dataset.py` are at the root, not inside a subfolder.
- `data_ref/drone_network_telemetry_dataset.csv` exists and is the reference synthetic dataset (44 columns).
- `sim/bridge/` contains at minimum `assemble.py`, `concat.py`, and `export_pose_trace.py`.
- `sim/simu5g/` contains the Simu5G scenario configuration(s) and the OMNeT++ `results/` output (`.sca` / `.vec` files).
- No required content is living somewhere else (e.g. a CSV in `logs/` that should be in `data/`, a validation report in `data/` that should be in `reports/`). List any misplaced files.
- List any extra top-level folders or stray files and say whether they are harmless or should be moved/removed.

## 2. Per-run file naming and placement

Every simulation run has a `<run_id>` (e.g. `B1_NOMINAL_001`, `DOS_HIGH_001`). For **each run present**, confirm the following files exist with the exact naming pattern and location:

| Step | Required output | Location / pattern |
|---|---|---|
| Scenario manifest | YAML manifest | `manifests/<run_id>.yaml` |
| Gazebo/PX4 mission + ROS 2 recording | ROS 2 bag directory | `logs/<run_id>/` |
| Pose export | Simu5G-compatible mobility trace | `trace/<run_id>_pose.txt` |
| Simu5G run | OMNeT++ scalar/vector result files | `sim/simu5g/results/` (`<Config>-<n>.sca`, `.vec`) |
| Metric export | Per-flow network metrics CSV | `logs/<run_id>_net.csv` |
| Bridge | Per-run dataset | `data/<run_id>_cosim.csv` |
| Validation | Validator output | `reports/<run_id>_validation.txt` |
| Bridge log | Window size, input files, output row count | in `logs/` (state the file used) |

Then produce a **run inventory table**: one row per `<run_id>`, one column per artifact above, with ✓ / ✗. Flag any run that has a dataset but no manifest, a manifest but no dataset, a trace with no bag, or inconsistent run_id spelling across folders.

Also confirm:
- `run_id` values are consistent between the manifest filename, the `scenario_id` field inside the manifest, and the output filenames.
- The concatenation command reads `data/*_cosim.csv` (per the experiment document) and writes `data/drone_network_telemetry_cosim.csv`. Note: the setup document shows `logs/runs/*.csv` instead — the experiment document is authoritative; report which convention the project actually uses and whether it is consistent.

## 3. Final dataset and schema

- `data/drone_network_telemetry_cosim.csv` exists and is the concatenation of all per-run `*_cosim.csv` files (row counts should sum; check).
- The header is **exactly** these 44 columns in **exactly** this order:

```
timestamp, organization_id, fleet_id, drone_id, mission_id, domain, mission_type,
source_component, destination_component, source_ip, destination_ip, source_port,
destination_port, protocol, network_slice_id, cell_id, session_id, packet_count,
byte_count, packets_per_second, throughput_kbps, latency_ms, jitter_ms,
packet_loss_rate, retransmission_rate, connection_duration_sec, authentication_status,
handover_event, handover_count, signal_quality_dbm, command_channel_activity,
video_stream_activity, telemetry_stream_activity, api_request_count,
failed_connection_attempts, unusual_destination_flag, session_reuse_flag,
traffic_burst_flag, anomaly_score, attack_type, attack_stage, severity,
recommended_response, incident_label
```

- Every per-run `*_cosim.csv` uses the same 44 columns in the same order.
- No extra columns (e.g. `scenario_id`, `run_id`, `intensity`, `injection_point`) are present in any dataset CSV — such metadata must be kept in manifests/audit files only.
- Run `python3 validate_dataset.py data/drone_network_telemetry_cosim.csv schema_reference.json` and report the literal output. Also run it on each per-run CSV. PASS only if it prints `RESULT: PASSED`.
- Categorical values match schema vocabulary; `recommended_response` follows the fixed mapping:
  `none→continue_monitoring`, `unknown_anomaly→increase_monitoring`, `denial_of_service→rate_limit_and_block_flow`, `session_hijacking→revoke_session_force_reauth`, `identity_theft→suspend_identity_quarantine_device`, `control_protocol_attack→block_command_channel_safe_mode`, `lateral_movement→segment_slice_isolate_component`, `network_application_attack→rate_limit_api_alert_security_team`.
- Dataset contains all three `incident_label` values (normal, suspicious, malicious) and their proportions are documented somewhere in `reports/`.

## 4. Manifest structure

For every `manifests/<run_id>.yaml`, confirm it contains: `organization_id, fleet_id, drone_id, mission_id, domain, mission_type, scenario_id, scenario_family`, a `simulation:` block (`drone_count, mission_speed_mps, duration_sec, aggregation_window_sec, cells, background_traffic, network_slice_profile`), an `attack:` block (`enabled, attack_type, injection_point, start_sec, end_sec, intensity`), and `label_windows:` entries each with `start_sec, end_sec, attack_type, attack_stage, severity, incident_label, recommended_response`. Report which manifests are missing fields. Confirm label windows cover only the attack interval, not the whole run, for malicious scenarios.

## 5. Bridge scripts (leakage control)

Read `sim/bridge/assemble.py`, `concat.py`, and helpers. Confirm:
- Flags (`traffic_burst_flag`, `unusual_destination_flag`, `session_reuse_flag`) and `anomaly_score` are computed only from telemetry/observations — never from `attack_type`, `attack_stage`, `severity`, `recommended_response`, `incident_label`, or `scenario_id`. Quote the relevant lines.
- Burst thresholds are derived from benign runs or a documented configuration.
- `packets_per_second` = packet_count / window; `throughput_kbps` = byte_count×8/1000 / window; `packet_loss_rate` = (sent−received)/sent with documented zero-sent handling.
- Default aggregation window is 1.0 s and is configurable.
- Labels are assigned by matching row timestamps to manifest `label_windows`.
- The bridge logs window size, input files, and output row count.

## 6. Run matrix coverage

Compare the run inventory against the minimum run matrix and report shortfalls:
- Benign baseline (default params): 3 repeated runs
- Benign mobility variation: 1 run each at slow / normal / fast
- Benign network variation: 1 run each at low / medium / high background load
- Suspicious: at least 3 scenario types (S1–S4) × 2 repeats
- Malicious: all 6 attack classes × at least 2 intensities
- Validator + notebook re-run after every concatenated build (check timestamps or log evidence)

## 7. Notebook structure

Locate the baseline ML notebook(s) (state the path). Confirm:
- The **baseline notebook runs unchanged** on `data/drone_network_telemetry_cosim.csv` — no edits to column names, column order, or schema handling. If a copy was made for the cosim data, diff it against the original and list every change.
- It loads `data_ref/drone_network_telemetry_dataset.csv` (synthetic) and `data/drone_network_telemetry_cosim.csv` (cosim) and treats them with the same pipeline.
- Label/target columns (`attack_type, attack_stage, severity, recommended_response, incident_label`) are excluded from the feature matrix.
- It reports, for **both** datasets: accuracy, macro-F1, confusion matrices, and feature importances.
- It runs the **feature-set ablation** required by the leakage-audit rule: (a) raw metrics only, (b) raw + 5G context, (c) raw + security indicators, (d) full feature set — and reports the differences.
- It includes a **distribution comparison** between synthetic and cosim data (per-feature stats or plots).
- It includes a written **realism-gaps discussion** / comparison section.
- All cells execute top-to-bottom without error (execute it if possible; otherwise check saved outputs and cell execution order).
- Outputs are saved to `reports/model_metrics.md` and plots to `reports/` (not left only inside the notebook).

## 8. Reports folder

Confirm `reports/` contains:
- `<run_id>_validation.txt` for every run
- A final validation report for the concatenated dataset (schema pass/fail + leakage-control notes)
- `model_metrics.md` with accuracy, macro-F1, confusion matrices, feature importances for both datasets
- Comparison plots (state filenames)
- Documented scenario-balance proportions

## 9. Deliverables checklist

Finish with the official deliverables list, each marked PASS/FAIL/MISSING with path:
1. Scenario manifests (one YAML per run)
2. ROS 2 bag files for benign, suspicious, and malicious runs
3. Pose traces in `trace/`
4. Simu5G scalar/vector results + exported per-flow CSVs
5. Bridge scripts (`assemble.py`, `concat.py`, helpers)
6. Final dataset `data/drone_network_telemetry_cosim.csv`
7. Validation report
8. ML comparison report

---

## Output format

1. Summary verdict (ready to submit / not ready) in two sentences.
2. Findings by section (1–9) as described above.
3. Run inventory table.
4. Prioritized fix list: **Blocking** (would fail validation or a required deliverable is missing), **Should fix** (naming/placement inconsistencies), **Nice to have**.
