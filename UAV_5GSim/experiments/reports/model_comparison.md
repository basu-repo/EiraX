# Machine-learning comparison

Dataset: `experiments/data/drone_network_telemetry_cosim.csv` (25,490 rows, 37 missions).

Regenerate with `python evaluate_models.py`.

| Metric | Simulation-derived dataset | Synthetic reference |
|---|---:|---:|
| Accuracy | 0.9295 | 0.9980 |
| Macro-F1 | 0.7499 | 0.9977 |
| False-alarm rate | 0.0111 | 0.0010 |

## Feature-group comparison

The experiment document requires the contribution of each feature layer to be reported separately.

| Feature group | Model | Accuracy | Macro-F1 | False-alarm rate |
|---|---|---:|---:|---:|
| raw_network | logistic_regression | 0.8570 | 0.4809 | 0.0410 |
| raw_network | random_forest | 0.8866 | 0.6122 | 0.0287 |
| raw_plus_5g_context | logistic_regression | 0.8886 | 0.5354 | 0.0053 |
| raw_plus_5g_context | random_forest | 0.9298 | 0.7490 | 0.0106 |
| security_indicators | logistic_regression | 0.8458 | 0.4999 | 0.0482 |
| security_indicators | random_forest | 0.8893 | 0.6248 | 0.0328 |
| complete | logistic_regression | 0.8587 | 0.5128 | 0.0366 |
| complete | random_forest | 0.9295 | 0.7499 | 0.0111 |

## Held-out confusion matrix, complete features, random forest

Rows are the true class, columns the predicted class, ordered normal, suspicious, malicious.

```text
   13010      84      62
     382     282      20
     470      78    1162
```

## Top features

| Feature | Importance |
|---|---:|
| `numeric__connection_duration_sec` | 0.3460 |
| `numeric__signal_quality_dbm` | 0.2411 |
| `numeric__handover_event` | 0.0684 |
| `numeric__source_port` | 0.0626 |
| `numeric__anomaly_score` | 0.0571 |
| `numeric__latency_ms` | 0.0376 |
| `numeric__failed_connection_attempts` | 0.0229 |
| `numeric__unusual_destination_flag` | 0.0208 |
| `categorical__source_component_drone` | 0.0191 |
| `numeric__handover_count` | 0.0169 |
| `numeric__packets_per_second` | 0.0144 |
| `categorical__source_component_external_host` | 0.0135 |

## Split policy

complete mission IDs; every measured-source derivative kept in held-out test

Training missions (16): `B1_BASE_001`, `B1_BASE_002`, `B1_PILOT_001`, `B_LOAD_LOW_001`, `B_LOAD_MEDIUM_001`, `B_SPEED_SLOW_001`, `M_API_LOW_001`, `M_CONTROL_LOW_001`, `M_DOS_LOW_001`, `M_IDENTITY_LOW_001`, `M_LATERAL_LOW_001`, `M_SESSION_LOW_001`, `S1_HANDOVER_LOSS_001`, `S2_AUTH_001`, `S3_API_RETRY_001`, `S4_DROPOUT_001`

Held-out missions (21): `B1_BASE_003`, `B2_HOVER_VIDEO_001`, `B_DURATION_10_001`, `B_DURATION_20_001`, `B_DURATION_5_001`, `B_LOAD_HIGH_001`, `B_SLICE_DUAL_001`, `B_SLICE_TRIPLE_001`, `M_API_HIGH_001`, `M_CONTROL_HIGH_001`, `M_DOS_30S_001`, `M_DOS_90S_001`, `M_DOS_HIGH_001`, `M_DOS_REPEATED_001`, `M_IDENTITY_HIGH_001`, `M_LATERAL_HIGH_001`, `M_SESSION_HIGH_001`, `S1_HANDOVER_LOSS_002`, `S2_AUTH_002`, `S3_API_RETRY_002`, `S4_DROPOUT_002`
