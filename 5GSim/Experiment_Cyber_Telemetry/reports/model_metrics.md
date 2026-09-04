# Baseline machine-learning metrics

| Dataset | Feature set | Accuracy | Macro-F1 |
|---|---|---:|---:|
| Co-simulation | raw metrics only | 0.9173 | 0.7287 |
| Co-simulation | raw + 5G context | 0.9381 | 0.7977 |
| Co-simulation | raw + security indicators | 0.9177 | 0.7282 |
| Co-simulation | full feature set | 0.9372 | 0.7924 |
| Synthetic reference | full feature set | 0.9990 | 0.9989 |

## Co-simulation dataset, full feature set

Rows: 17779. Accuracy **0.9372**, macro-F1 **0.7924**.

Confusion matrix, rows true and columns predicted, ordered normal, suspicious, malicious:

```
    4498      28      48
     121     135      18
     117       3     366
```

| Feature | Importance |
|---|---:|
| `connection_duration_sec` | 0.2241 |
| `signal_quality_dbm` | 0.1830 |
| `latency_ms` | 0.1466 |
| `jitter_ms` | 0.0642 |
| `source_port` | 0.0620 |
| `source_component_drone` | 0.0598 |
| `retransmission_rate` | 0.0556 |
| `source_component_external_host` | 0.0462 |
| `destination_port` | 0.0212 |
| `throughput_kbps` | 0.0199 |

## Co-simulation dataset, raw metrics only

Rows: 17779. Accuracy **0.9173**, macro-F1 **0.7287**.

Confusion matrix, rows true and columns predicted, ordered normal, suspicious, malicious:

```
    4451      54      69
     140     111      23
     152       3     331
```

| Feature | Importance |
|---|---:|
| `connection_duration_sec` | 0.2914 |
| `latency_ms` | 0.2447 |
| `jitter_ms` | 0.1013 |
| `retransmission_rate` | 0.0819 |
| `throughput_kbps` | 0.0774 |
| `byte_count` | 0.0731 |
| `packets_per_second` | 0.0613 |
| `packet_count` | 0.0592 |
| `packet_loss_rate` | 0.0098 |

## Synthetic reference, full feature set

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

## Model comparison

Same splits and feature sets, three models:

| Model | Dataset | Feature set | Accuracy | Macro-F1 |
|---|---|---|---:|---:|
| random forest | Co-simulation | raw metrics only | 0.9173 | 0.7287 |
| random forest | Co-simulation | raw + 5G context | 0.9381 | 0.7977 |
| random forest | Co-simulation | raw + security indicators | 0.9177 | 0.7282 |
| random forest | Co-simulation | full feature set | 0.9372 | 0.7924 |
| random forest | Synthetic reference | full feature set | 0.9990 | 0.9989 |
| gradient boosting | Co-simulation | raw metrics only | 0.9226 | 0.7297 |
| gradient boosting | Co-simulation | raw + 5G context | 0.9441 | 0.8198 |
| gradient boosting | Co-simulation | raw + security indicators | 0.9216 | 0.7283 |
| gradient boosting | Co-simulation | full feature set | 0.9447 | 0.8239 |
| gradient boosting | Synthetic reference | full feature set | 0.9983 | 0.9979 |
| logistic regression | Co-simulation | raw metrics only | 0.8656 | 0.3714 |
| logistic regression | Co-simulation | raw + 5G context | 0.8948 | 0.5807 |
| logistic regression | Co-simulation | raw + security indicators | 0.8941 | 0.5675 |
| logistic regression | Co-simulation | full feature set | 0.8946 | 0.5814 |
| logistic regression | Synthetic reference | full feature set | 0.9907 | 0.9875 |
