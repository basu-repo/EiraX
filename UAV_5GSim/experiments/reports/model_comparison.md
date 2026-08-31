# Machine-learning comparison

Dataset: `experiments/data/drone_network_telemetry_cosim.csv` (25,490 rows, 37 missions).

Regenerate with `python evaluate_models.py`.

| Metric | Simulation-derived dataset | Synthetic reference |
|---|---:|---:|
| Accuracy | 0.9268 | 0.9980 |
| Macro-F1 | 0.7134 | 0.9977 |
| False-alarm rate | 0.0106 | 0.0010 |

## Feature-group comparison

The experiment document requires the contribution of each feature layer to be reported separately.

| Feature group | Model | Accuracy | Macro-F1 | False-alarm rate |
|---|---|---:|---:|---:|
| raw_network | logistic_regression | 0.8484 | 0.4762 | 0.0512 |
| raw_network | random_forest | 0.8901 | 0.5607 | 0.0151 |
| raw_plus_5g_context | logistic_regression | 0.8821 | 0.5096 | 0.0113 |
| raw_plus_5g_context | random_forest | 0.9246 | 0.7057 | 0.0135 |
| security_indicators | logistic_regression | 0.8832 | 0.5179 | 0.0100 |
| security_indicators | random_forest | 0.8904 | 0.5615 | 0.0147 |
| complete | logistic_regression | 0.8813 | 0.5085 | 0.0123 |
| complete | random_forest | 0.9268 | 0.7134 | 0.0106 |

## Held-out confusion matrix, complete features, random forest

Rows are the true class, columns the predicted class, ordered normal, suspicious, malicious.

```text
   13016      75      65
     465     179      40
     483      10    1217
```

## Top features

| Feature | Importance |
|---|---:|
| `numeric__signal_quality_dbm` | 0.3343 |
| `numeric__connection_duration_sec` | 0.3188 |
| `numeric__source_port` | 0.0632 |
| `numeric__latency_ms` | 0.0489 |
| `numeric__anomaly_score` | 0.0294 |
| `numeric__handover_count` | 0.0276 |
| `numeric__unusual_destination_flag` | 0.0273 |
| `numeric__failed_connection_attempts` | 0.0221 |
| `categorical__source_component_drone` | 0.0196 |
| `categorical__source_component_external_host` | 0.0170 |
| `categorical__cell_id_gnb_1` | 0.0150 |
| `categorical__cell_id_gnb_2` | 0.0138 |

## Split policy

complete mission IDs; every measured-source derivative kept in held-out test

Training missions (16): `B1_BASE_001`, `B1_BASE_002`, `B1_PILOT_001`, `B_LOAD_LOW_001`, `B_LOAD_MEDIUM_001`, `B_SPEED_SLOW_001`, `M_API_LOW_001`, `M_CONTROL_LOW_001`, `M_DOS_LOW_001`, `M_IDENTITY_LOW_001`, `M_LATERAL_LOW_001`, `M_SESSION_LOW_001`, `S1_HANDOVER_LOSS_001`, `S2_AUTH_001`, `S3_API_RETRY_001`, `S4_DROPOUT_001`

Held-out missions (21): `B1_BASE_003`, `B2_HOVER_VIDEO_001`, `B_DURATION_10_001`, `B_DURATION_20_001`, `B_DURATION_5_001`, `B_LOAD_HIGH_001`, `B_SLICE_DUAL_001`, `B_SLICE_TRIPLE_001`, `M_API_HIGH_001`, `M_CONTROL_HIGH_001`, `M_DOS_30S_001`, `M_DOS_90S_001`, `M_DOS_HIGH_001`, `M_DOS_REPEATED_001`, `M_IDENTITY_HIGH_001`, `M_LATERAL_HIGH_001`, `M_SESSION_HIGH_001`, `S1_HANDOVER_LOSS_002`, `S2_AUTH_002`, `S3_API_RETRY_002`, `S4_DROPOUT_002`
