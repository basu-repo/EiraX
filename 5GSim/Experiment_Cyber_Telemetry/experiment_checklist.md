# Experiment checklist

Everything the experiment instructions ask for, in the order the document gives
it. `[x]` is done, `[ ]` is still to do, and `[~]` is done in a way the report
states rather than the document's first choice.

State: all 43 runs are flown, simulated, bridged and validated. The final
dataset has 17,779 rows, the three reports are written, and the notebook's
section 6 holds the analysis. Only the optional anomaly_score is open.

## 1. Scope

- [x] Benign mission telemetry: ROS 2 bag, pose trace, Simu5G metrics, normal rows (21 benign manifests)
- [x] Suspicious telemetry: handover loss, failed logins, API retries, telemetry dropout (S1 to S4, two each)
- [x] Malicious telemetry: one class per run at low and high intensity, plus a 30 s and a repeated-burst DoS (14 manifests)
- [x] Schema compatibility: 44 columns, vocabularies, label-response mapping, no label-derived features
- [x] Comparison with the synthetic baseline: reports/distribution_comparison.md, reports/model_metrics.md, and the realism discussion in notebook section 6.3

## 2. Telemetry goals and the evidence for each

| Goal | Evidence to collect | Where |
|---|---|---|
| Mission-realistic movement | Gazebo/PX4 launch log, ROS 2 bag, exported x/y/z pose trace | logs/<run>/mavros.log, logs/<run>/rosbag, trace/<run>_pose.txt |
| Measured network behaviour | OMNeT++ scalar/vector results and exported CSV | logs/<run>/network/results, logs/<run>_net.csv |
| Time-aligned telemetry | Bridge log showing window size, input files, output row count | logs/<run>/bridge.log, written by assemble.py |
| Scenario traceability | Manifest, run ID, label-window records | manifests/<run>.yaml, mission_id is the run ID |
| Leakage-controlled features | Bridge derivation rules and a validator/audit report | reports/validation_report.md, written by audit_dataset.py |
| Baseline compatibility | Validator pass and notebook metrics | reports/<run>_validation.txt, reports/model_metrics.md |

## 3. Required folders

- [x] data/, data_ref/, logs/, manifests/, results/, sim/bridge/, sim/Simu5G (link), trace/, reports/
- [x] schema_reference.json and validate_dataset.py at the top level

## 4. Metric definitions

One row per flow per window; the window comes from the manifest. The
derivation of every column is written into reports/validation_report.md.

| Field | Document rule | Bridge | Status |
|---|---|---|---|
| timestamp | window start, ISO UTC or sim time | epoch plus window start, ISO | [x] |
| packet_count, byte_count | per flow per window, one direction | received at the edge node | [x] |
| packets_per_second | packet_count / window | same | [x] |
| throughput_kbps | byte_count * 8 / 1000 / window | same | [x] |
| latency_ms | mean delay per flow per window | mean cbrFrameDelay | [x] |
| jitter_ms | std dev, document the formula | population std dev of the delays in the window | [x] stated |
| packet_loss_rate | (sent - received) / sent, 0 when nothing was sent | (configured rate * window - received) / expected, 0 when the window expects less than one packet | [~] stated |
| retransmission_rate | transport retransmissions or duplicate sequence events | mean uplink HARQ error rate | [~] stated |
| connection_duration_sec | window time minus first seen | window end minus flow start | [x] |
| signal_quality_dbm | RSRP or received power | -97 dBm plus mean uplink SINR | [~] stated |
| handover_event, handover_count | serving cell change; cumulative | from the servingCell vector | [x] |
| authentication_status | observed logs or scenario configuration | from the flow configuration | [x] |
| command_channel_activity, api_request_count | messages per window | packets on command and API flows | [x] |
| video_stream_activity, telemetry_stream_activity | rate or activity | 1 when the flow delivered packets | [x] |
| failed_connection_attempts | failures, rejected logins, failed requests | packets on flows configured as unanswered or failing | [~] stated |
| unusual_destination_flag | destination not in allowed_destinations | same | [x] |
| session_reuse_flag | session_id seen from a different source | same | [x] |
| traffic_burst_flag | above a multiple of the benign baseline | above 3 x the highest mission flow rate | [x] |
| anomaly_score | optional | held at 0.0 | [ ] optional |

## 5. Parameter variations

One group per run, baseline first, every setting in the manifest.

| Group | Baseline | Variation A | Variation B | Manifests |
|---|---|---|---|---|
| Aggregation window | 1.0 s | 0.5 s | 5.0 s | B_WINDOW_0P5_001, B_WINDOW_5_001 |
| Mission speed | 3 m/s | 1 m/s patrol | 8 m/s patrol | B_SPEED_SLOW_001, B_SPEED_FAST_001 |
| Mission duration | 5 min | 10 min | 20 min | B_DURATION_5_001, B_DURATION_10_001, B_DURATION_20_001 |
| Number of drones | 1 | 3 | 5 | B_DRONES_3_001, B_DRONES_5_001 |
| Cells | 1 | 2 | 3 | B_CELLS_2_001, B_CELLS_3_001 |
| Network slice | single | mission + video | mission + telemetry + backend | B_SLICE_DUAL_001, B_SLICE_TRIPLE_001 |
| Background traffic | low | medium | high | B_LOAD_MEDIUM_001, B_LOAD_HIGH_001 |
| Wireless condition | good | moderate | edge of cell | B_WIRELESS_MODERATE_001, B_WIRELESS_EDGE_001 |
| Attack intensity | off | low | high | M_*_LOW_001, M_*_HIGH_001 |
| Attack timing | none | 30 s window | repeated bursts | M_DOS_30S_001, M_DOS_REPEATED_001 |

The matrix baseline is the 135 s route flight, the same as the first document,
so the two datasets stay comparable. The 5 minute run is a patrol of the same
route. Speed and duration variations patrol, so the flight lasts exactly the
manifest duration at any speed.

Minimum run matrix, all run:

- [x] Benign baseline, 3 repeated runs (B_BASE_001 to 003, seeds 1 to 3)
- [x] Benign mobility variation, slow, normal, fast
- [x] Benign network variation, low, medium, high load
- [x] Suspicious, 4 types with 2 repeats each
- [x] Malicious, 6 attack types at 2 intensities each
- [x] Validator and baseline model after the concatenated build

Multi-drone runs record one real flight per drone in sequence, because two
PX4 instances in one Gazebo world gave the second drone an unusable position
estimate; Simu5G flies the recorded traces together.

## 6. Scenario setups

Benign:

- [x] B1 nominal patrol: B_BASE_001 to 003
- [x] B2 hover with telemetry and video: B_HOVER_001
- [x] B3 multi-cell benign mobility: B_CELLS_2_001, B_CELLS_3_001, labelled normal
- [x] B4 benign congestion: B_LOAD_MEDIUM_001, B_LOAD_HIGH_001, labelled normal

Suspicious, all suspicious / increase_monitoring:

- [x] S1 short handover loss spike: two cells at moderate signal, window 30 to 60 s around the boundary crossing the flight plan predicts at 38 s
- [x] S2 failed authentication burst: 3 failed logins per second from the drone, 40 to 80 s
- [x] S3 API retry storm: 30 unanswered requests per second from the drone, 40 to 80 s
- [x] S4 temporary telemetry dropout: the telemetry flow stops from 60 to 68 s, no attacker

Malicious, one class per run, low and high intensity, injected in Simu5G from 40 to 80 s, followed by a 10 s recovery window labelled suspicious as in the document's example manifest:

- [x] denial_of_service 50 / 200 pps, plus the 30 s and repeated-burst variants
- [x] session_hijacking 1 / 5 pps reusing the command session
- [x] identity_theft 1 / 4 pps claiming the drone's identity from another IP and slice
- [x] control_protocol_attack 5 / 20 pps of rogue commands
- [x] lateral_movement 2 / 8 pps to an internal address outside the allowed set
- [x] network_application_attack 50 / 200 unanswered API requests per second

## 7. Manifest requirements

Every manifest carries:

- [x] organization_id, fleet_id, drone_id, mission_id, domain, mission_type
- [x] scenario_id, scenario_family
- [x] simulation: drone_count, mission_speed_mps, duration_sec, aggregation_window_sec, cells, background_traffic, network_slice_profile, wireless_condition, mission_profile, seed
- [x] attack: enabled, attack_type, injection_point, intensity, start_sec and end_sec or bursts, flow
- [x] label_windows covering the whole run with no gaps, responses matching attack_type
- [x] a recovery window after each attack
- [x] expected_effects
- [x] allowed_destinations

## 8. Bridge derivation rules against leakage

- [x] Labels are never inputs; the model drops the five label columns, the timestamp and the identifiers
- [x] Flags come from packet rates, the destination profile, session history and source identity
- [x] Burst threshold comes from the mission flows, never from labels
- [x] latency, jitter, loss, throughput, packet and byte counts come from Simu5G vectors
- [x] No perfect identifiers in the model inputs; mission_id and session_id are dropped
- [x] Feature-set audit: raw, raw + 5G context, raw + security, full, in reports/model_metrics.md
- [x] Label windows cover only the attack interval
- [x] Suspicious rows are ambiguous degradation, not weak attacks
- [x] audit_dataset.py recomputes the three flags for every run and reports mismatches

## 9. Data assembly procedure

| Step | Action | Output | How |
|---|---|---|---|
| 1 | Fly the PX4 mission and record ROS 2 topics | logs/<run>/rosbag | record_mission.py, notebook section 3 |
| 2 | Export the pose trace | trace/<run>_pose.txt | record_mission.py |
| 3 | Run Simu5G with the trace and network parameters | logs/<run>/network/results | run_scenario.py, section 4 |
| 4 | Export scalar/vector metrics | logs/<run>_net.csv | run_scenario.py |
| 5 | Bridge with manifest, ROS log, network CSV, window | data/<run>_cosim.csv, logs/<run>/bridge.log | assemble.py |
| 6 | Validate schema and vocabulary | reports/<run>_validation.txt | validate_dataset.py |
| 7 | Concatenate | data/drone_network_telemetry_cosim.csv | concat.py, section 4.1 |
| 8 | Baseline notebook and comparison report | reports/model_metrics.md, distribution_comparison.md, validation_report.md | baseline_model.py, distribution_comparison.py, audit_dataset.py |

## 10. Output schema

- [x] 44 columns in the required order, validator authoritative

## 11. Validation and comparison checklist

reports/validation_report.md covers the first seven, reports/model_metrics.md
and reports/distribution_comparison.md the last.

- [x] Schema validation prints RESULT: PASSED, for all 43 runs and the final dataset
- [x] Column order, vocabulary values, response mapping
- [x] Metric provenance written down
- [x] Leakage audit written down, 0 flag mismatches over 43 runs
- [x] Scenario balance documented: 15,247 normal, 912 suspicious, 1,620 malicious
- [x] Baseline model runs without schema changes
- [x] Synthetic comparison: distributions, importances, accuracy, macro-F1, confusion matrices
- [x] Discussion of the differences and the realism gaps, notebook section 6.3

## 12. Final deliverables

- [x] Scenario manifests with settings, label windows and expected effects
- [x] ROS 2 bags for benign, suspicious and malicious runs, 49 flights
- [x] Pose traces, one per flight, cut at the manifest duration
- [x] Simu5G results and per-flow CSVs
- [x] Bridge scripts: assemble.py, concat.py, export_pose_trace.py
- [x] Final dataset data/drone_network_telemetry_cosim.csv
- [x] Validation report with leakage-control notes
- [x] ML comparison report with the synthetic baseline

## 13. Analysis, notebook section 6

- [x] 6.1 every run's measured effect against its manifest's expected effects, 43 of 43 seen
- [x] 6.2 figures: attack and suspicious panels, parameter sweeps, intensity, feature sets
- [x] 6.3 discussion of realism gaps against the synthetic dataset
- [x] 6.4 deliverables gate, 350 checks
- [ ] anomaly_score, optional, still 0.0
