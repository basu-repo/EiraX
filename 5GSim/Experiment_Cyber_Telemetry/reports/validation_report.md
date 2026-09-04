# Validation report

## Schema validation

```
RESULT: PASSED (17779 rows, 44 columns)
```

Per-run validator results:

- B_BASE_001: RESULT: PASSED (270 rows, 44 columns)
- B_BASE_002: RESULT: PASSED (270 rows, 44 columns)
- B_BASE_003: RESULT: PASSED (270 rows, 44 columns)
- B_CELLS_2_001: RESULT: PASSED (270 rows, 44 columns)
- B_CELLS_3_001: RESULT: PASSED (270 rows, 44 columns)
- B_DRONES_3_001: RESULT: PASSED (810 rows, 44 columns)
- B_DRONES_5_001: RESULT: PASSED (1350 rows, 44 columns)
- B_DURATION_10_001: RESULT: PASSED (1200 rows, 44 columns)
- B_DURATION_20_001: RESULT: PASSED (2400 rows, 44 columns)
- B_DURATION_5_001: RESULT: PASSED (600 rows, 44 columns)
- B_HOVER_001: RESULT: PASSED (405 rows, 44 columns)
- B_LOAD_HIGH_001: RESULT: PASSED (270 rows, 44 columns)
- B_LOAD_MEDIUM_001: RESULT: PASSED (270 rows, 44 columns)
- B_SLICE_DUAL_001: RESULT: PASSED (405 rows, 44 columns)
- B_SLICE_TRIPLE_001: RESULT: PASSED (405 rows, 44 columns)
- B_SPEED_FAST_001: RESULT: PASSED (270 rows, 44 columns)
- B_SPEED_SLOW_001: RESULT: PASSED (270 rows, 44 columns)
- B_WINDOW_0P5_001: RESULT: PASSED (540 rows, 44 columns)
- B_WINDOW_5_001: RESULT: PASSED (54 rows, 44 columns)
- B_WIRELESS_EDGE_001: RESULT: PASSED (270 rows, 44 columns)
- B_WIRELESS_MODERATE_001: RESULT: PASSED (270 rows, 44 columns)
- S1_HANDOVER_001: RESULT: PASSED (270 rows, 44 columns)
- S1_HANDOVER_002: RESULT: PASSED (270 rows, 44 columns)
- S2_AUTH_001: RESULT: PASSED (310 rows, 44 columns)
- S2_AUTH_002: RESULT: PASSED (310 rows, 44 columns)
- S3_RETRY_001: RESULT: PASSED (310 rows, 44 columns)
- S3_RETRY_002: RESULT: PASSED (310 rows, 44 columns)
- S4_DROPOUT_001: RESULT: PASSED (270 rows, 44 columns)
- S4_DROPOUT_002: RESULT: PASSED (270 rows, 44 columns)
- M_API_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_API_LOW_001: RESULT: PASSED (310 rows, 44 columns)
- M_CONTROL_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_CONTROL_LOW_001: RESULT: PASSED (310 rows, 44 columns)
- M_DOS_30S_001: RESULT: PASSED (300 rows, 44 columns)
- M_DOS_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_DOS_LOW_001: RESULT: PASSED (310 rows, 44 columns)
- M_DOS_REPEATED_001: RESULT: PASSED (300 rows, 44 columns)
- M_IDENTITY_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_IDENTITY_LOW_001: RESULT: PASSED (310 rows, 44 columns)
- M_LATERAL_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_LATERAL_LOW_001: RESULT: PASSED (310 rows, 44 columns)
- M_SESSION_HIGH_001: RESULT: PASSED (310 rows, 44 columns)
- M_SESSION_LOW_001: RESULT: PASSED (310 rows, 44 columns)

## Scenario balance

| Label | Rows | Share |
|---|---:|---:|
| normal | 15247 | 0.858 |
| suspicious | 912 | 0.051 |
| malicious | 1620 | 0.091 |

| Family | Runs | Rows | normal | suspicious | malicious |
|---|---:|---:|---:|---:|---:|
| benign | 21 | 11139 | 11139 | 0 | 0 |
| malicious | 14 | 4320 | 2420 | 280 | 1620 |
| suspicious | 8 | 2320 | 1688 | 632 | 0 |

| Run | Family | Rows | normal | suspicious | malicious |
|---|---|---:|---:|---:|---:|
| B_BASE_001 | benign | 270 | 270 | 0 | 0 |
| B_BASE_002 | benign | 270 | 270 | 0 | 0 |
| B_BASE_003 | benign | 270 | 270 | 0 | 0 |
| B_CELLS_2_001 | benign | 270 | 270 | 0 | 0 |
| B_CELLS_3_001 | benign | 270 | 270 | 0 | 0 |
| B_DRONES_3_001 | benign | 810 | 810 | 0 | 0 |
| B_DRONES_5_001 | benign | 1350 | 1350 | 0 | 0 |
| B_DURATION_10_001 | benign | 1200 | 1200 | 0 | 0 |
| B_DURATION_20_001 | benign | 2400 | 2400 | 0 | 0 |
| B_DURATION_5_001 | benign | 600 | 600 | 0 | 0 |
| B_HOVER_001 | benign | 405 | 405 | 0 | 0 |
| B_LOAD_HIGH_001 | benign | 270 | 270 | 0 | 0 |
| B_LOAD_MEDIUM_001 | benign | 270 | 270 | 0 | 0 |
| B_SLICE_DUAL_001 | benign | 405 | 405 | 0 | 0 |
| B_SLICE_TRIPLE_001 | benign | 405 | 405 | 0 | 0 |
| B_SPEED_FAST_001 | benign | 270 | 270 | 0 | 0 |
| B_SPEED_SLOW_001 | benign | 270 | 270 | 0 | 0 |
| B_WINDOW_0P5_001 | benign | 540 | 540 | 0 | 0 |
| B_WINDOW_5_001 | benign | 54 | 54 | 0 | 0 |
| B_WIRELESS_EDGE_001 | benign | 270 | 270 | 0 | 0 |
| B_WIRELESS_MODERATE_001 | benign | 270 | 270 | 0 | 0 |
| S1_HANDOVER_001 | suspicious | 270 | 210 | 60 | 0 |
| S1_HANDOVER_002 | suspicious | 270 | 210 | 60 | 0 |
| S2_AUTH_001 | suspicious | 310 | 190 | 120 | 0 |
| S2_AUTH_002 | suspicious | 310 | 190 | 120 | 0 |
| S3_RETRY_001 | suspicious | 310 | 190 | 120 | 0 |
| S3_RETRY_002 | suspicious | 310 | 190 | 120 | 0 |
| S4_DROPOUT_001 | suspicious | 270 | 254 | 16 | 0 |
| S4_DROPOUT_002 | suspicious | 270 | 254 | 16 | 0 |
| M_API_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_API_LOW_001 | malicious | 310 | 170 | 20 | 120 |
| M_CONTROL_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_CONTROL_LOW_001 | malicious | 310 | 170 | 20 | 120 |
| M_DOS_30S_001 | malicious | 300 | 190 | 20 | 90 |
| M_DOS_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_DOS_LOW_001 | malicious | 310 | 170 | 20 | 120 |
| M_DOS_REPEATED_001 | malicious | 300 | 190 | 20 | 90 |
| M_IDENTITY_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_IDENTITY_LOW_001 | malicious | 310 | 170 | 20 | 120 |
| M_LATERAL_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_LATERAL_LOW_001 | malicious | 310 | 170 | 20 | 120 |
| M_SESSION_HIGH_001 | malicious | 310 | 170 | 20 | 120 |
| M_SESSION_LOW_001 | malicious | 310 | 170 | 20 | 120 |

## Metric provenance

| Column(s) | Source | Derivation |
|---|---|---|
| timestamp | bridge | window start, ISO 8601 UTC from the simulation clock |
| organization_id, fleet_id, drone_id, mission_id, domain, mission_type | manifest | copied; drone_id is the identity a flow claims |
| source_component, destination_component, source_ip, destination_ip, source_port, destination_port, protocol, network_slice_id, session_id | flow contract | the flow's identity as configured in Simu5G |
| cell_id | Simu5G | servingCell vector of the drone's NR PHY |
| packet_count, byte_count | Simu5G | cbrReceivedBytes vector at the edge node, counted and summed per window |
| packets_per_second | bridge | packet_count / window |
| throughput_kbps | bridge | byte_count * 8 / 1000 / window |
| latency_ms | Simu5G | mean of cbrFrameDelay in the window |
| jitter_ms | bridge | population standard deviation of cbrFrameDelay in the window |
| packet_loss_rate | bridge | (expected - received) / expected, expected = configured rate * window |
| retransmission_rate | Simu5G | mean uplink HARQ error rate in the window |
| connection_duration_sec | bridge | window end minus the flow's start |
| authentication_status | flow contract | failed when the flow is configured as failing authentication, else success |
| handover_event, handover_count | bridge | serving cell changed in the window; cumulative changes in the run |
| signal_quality_dbm | Simu5G | -97 dBm plus mean uplink SINR in the window |
| command_channel_activity, api_request_count | bridge | packets received on command and API flows |
| video_stream_activity, telemetry_stream_activity | bridge | 1 when a video or telemetry flow delivered packets in the window |
| failed_connection_attempts | bridge | packets received on flows configured as unanswered or failing authentication |
| unusual_destination_flag | bridge | destination not in the manifest's allowed_destinations |
| session_reuse_flag | bridge | session_id already seen from a different source IP in the run |
| traffic_burst_flag | bridge | packets_per_second above 3 x the highest mission flow rate |
| anomaly_score | bridge | held at 0.0, optional |
| attack_type, attack_stage, severity, recommended_response, incident_label | manifest | label window covering the window start, applied after every measurement |

## Leakage audit

The five label columns are read by the bridge only to stamp each window after its measurements are taken, and are excluded from the model inputs together with the timestamp and identifiers. The three behavioural flags are recomputed here from the manifest and flow contract of every run and compared with the dataset:

| Run | unusual_destination mismatches | session_reuse mismatches | traffic_burst mismatches |
|---|---:|---:|---:|
| B_BASE_001 | 0 | 0 | 0 |
| B_BASE_002 | 0 | 0 | 0 |
| B_BASE_003 | 0 | 0 | 0 |
| B_CELLS_2_001 | 0 | 0 | 0 |
| B_CELLS_3_001 | 0 | 0 | 0 |
| B_DRONES_3_001 | 0 | 0 | 0 |
| B_DRONES_5_001 | 0 | 0 | 0 |
| B_DURATION_10_001 | 0 | 0 | 0 |
| B_DURATION_20_001 | 0 | 0 | 0 |
| B_DURATION_5_001 | 0 | 0 | 0 |
| B_HOVER_001 | 0 | 0 | 0 |
| B_LOAD_HIGH_001 | 0 | 0 | 0 |
| B_LOAD_MEDIUM_001 | 0 | 0 | 0 |
| B_SLICE_DUAL_001 | 0 | 0 | 0 |
| B_SLICE_TRIPLE_001 | 0 | 0 | 0 |
| B_SPEED_FAST_001 | 0 | 0 | 0 |
| B_SPEED_SLOW_001 | 0 | 0 | 0 |
| B_WINDOW_0P5_001 | 0 | 0 | 0 |
| B_WINDOW_5_001 | 0 | 0 | 0 |
| B_WIRELESS_EDGE_001 | 0 | 0 | 0 |
| B_WIRELESS_MODERATE_001 | 0 | 0 | 0 |
| S1_HANDOVER_001 | 0 | 0 | 0 |
| S1_HANDOVER_002 | 0 | 0 | 0 |
| S2_AUTH_001 | 0 | 0 | 0 |
| S2_AUTH_002 | 0 | 0 | 0 |
| S3_RETRY_001 | 0 | 0 | 0 |
| S3_RETRY_002 | 0 | 0 | 0 |
| S4_DROPOUT_001 | 0 | 0 | 0 |
| S4_DROPOUT_002 | 0 | 0 | 0 |
| M_API_HIGH_001 | 0 | 0 | 0 |
| M_API_LOW_001 | 0 | 0 | 0 |
| M_CONTROL_HIGH_001 | 0 | 0 | 0 |
| M_CONTROL_LOW_001 | 0 | 0 | 0 |
| M_DOS_30S_001 | 0 | 0 | 0 |
| M_DOS_HIGH_001 | 0 | 0 | 0 |
| M_DOS_LOW_001 | 0 | 0 | 0 |
| M_DOS_REPEATED_001 | 0 | 0 | 0 |
| M_IDENTITY_HIGH_001 | 0 | 0 | 0 |
| M_IDENTITY_LOW_001 | 0 | 0 | 0 |
| M_LATERAL_HIGH_001 | 0 | 0 | 0 |
| M_LATERAL_LOW_001 | 0 | 0 | 0 |
| M_SESSION_HIGH_001 | 0 | 0 | 0 |
| M_SESSION_LOW_001 | 0 | 0 | 0 |

Total mismatches: 0. Every flag is reproduced from behaviour alone.
