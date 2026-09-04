# Baseline machine-learning metrics

| Dataset | Accuracy | Macro-F1 |
|---|---:|---:|
| Co-simulation | 0.9740 | 0.9305 |
| Synthetic reference | 0.9990 | 0.9989 |

## Co-simulation dataset

Rows: 2440. Accuracy **0.9740**, macro-F1 **0.9305**.

Confusion matrix, rows true and columns predicted, ordered normal, suspicious, malicious:

```
     473       0       7
       0      27       9
       2       1     213
```

| Feature | Importance |
|---|---:|
| `connection_duration_sec` | 0.3476 |
| `signal_quality_dbm` | 0.1833 |
| `latency_ms` | 0.1072 |
| `jitter_ms` | 0.0937 |
| `source_component_drone` | 0.0514 |
| `source_component_external_host` | 0.0513 |
| `source_port` | 0.0446 |
| `retransmission_rate` | 0.0367 |
| `failed_connection_attempts` | 0.0142 |
| `destination_port` | 0.0140 |

## Synthetic reference

Rows: 10000. Accuracy **0.9990**, macro-F1 **0.9989**.

Confusion matrix, rows true and columns predicted, ordered normal, suspicious, malicious:

```
    1829       3       0
       0     579       0
       0       0     589
```

| Feature | Importance |
|---|---:|
| `anomaly_score` | 0.2081 |
| `failed_connection_attempts` | 0.1443 |
| `latency_ms` | 0.1222 |
| `jitter_ms` | 0.1113 |
| `packet_loss_rate` | 0.0969 |
| `unusual_destination_flag` | 0.0805 |
| `command_channel_activity` | 0.0481 |
| `api_request_count` | 0.0352 |
| `packets_per_second` | 0.0231 |
| `session_reuse_flag` | 0.0224 |
