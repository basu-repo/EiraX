# Distribution comparison, co-simulation against the synthetic reference

Co-simulation: 17779 rows, 43 runs. Synthetic reference: 10000 rows.

## Numeric columns

| Column | Cosim mean | Cosim std | Cosim median | Synthetic mean | Synthetic std | Synthetic median |
|---|---:|---:|---:|---:|---:|---:|
| `packet_count` | 25.39 | 28.91 | 21 | 1.755e+04 | 9.58e+04 | 2285 |
| `byte_count` | 4365 | 1.974e+04 | 1596 | 1.275e+07 | 7.75e+07 | 1.386e+06 |
| `packets_per_second` | 25.41 | 27.71 | 25 | 572.8 | 2685 | 83.78 |
| `throughput_kbps` | 34.94 | 157.9 | 20.67 | 3513 | 1.586e+04 | 668.7 |
| `latency_ms` | 11.6 | 3.306 | 11.98 | 78.2 | 81.93 | 53.93 |
| `jitter_ms` | 0.4878 | 0.8204 | 0.01732 | 18.49 | 31.14 | 9.663 |
| `packet_loss_rate` | 0.006723 | 0.07146 | 0 | 0.03218 | 0.05512 | 0.01273 |
| `retransmission_rate` | 0.01289 | 0.03293 | 0 | 0.01631 | 0.01624 | 0.01142 |
| `connection_duration_sec` | 156.5 | 232.4 | 81 | 1792 | 1035 | 1786 |
| `handover_event` | 0.0005625 | 0.02371 | 0 | 0.0817 | 0.2739 | 0 |
| `handover_count` | 0.04837 | 0.2435 | 0 | 0.1987 | 0.4434 | 0 |
| `signal_quality_dbm` | -56.57 | 13.16 | -58.29 | -75.16 | 11.49 | -75.26 |
| `command_channel_activity` | 0.5328 | 1.104 | 0 | 7.873 | 27.73 | 2 |
| `video_stream_activity` | 0.02278 | 0.1492 | 0 | 0.4565 | 0.4981 | 0 |
| `telemetry_stream_activity` | 0.468 | 0.499 | 0 | 0.8488 | 0.3583 | 1 |
| `api_request_count` | 0.6942 | 9.914 | 0 | 52.26 | 223.6 | 8 |
| `failed_connection_attempts` | 0.7414 | 9.922 | 0 | 12.34 | 35.64 | 1 |
| `unusual_destination_flag` | 0.0045 | 0.06693 | 0 | 0.1893 | 0.3918 | 0 |
| `session_reuse_flag` | 0.0045 | 0.06693 | 0 | 0.0473 | 0.2123 | 0 |
| `traffic_burst_flag` | 0.007874 | 0.08839 | 0 | 0.0799 | 0.2712 | 0 |
| `anomaly_score` | 0 | 0 | 0 | 0.1379 | 0.1942 | 0.0393 |

## Categorical columns

### incident_label

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| normal | 0.858 | 0.611 |
| malicious | 0.091 | 0.196 |
| suspicious | 0.051 | 0.193 |

### attack_type

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| none | 0.858 | 0.682 |
| unknown_anomaly | 0.051 | 0.121 |
| denial_of_service | 0.024 | 0.031 |
| network_application_attack | 0.013 | 0.033 |
| control_protocol_attack | 0.013 | 0.031 |
| identity_theft | 0.013 | 0.035 |
| lateral_movement | 0.013 | 0.030 |
| session_hijacking | 0.013 | 0.036 |

### source_component

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| drone | 0.970 | 0.163 |
| external_host | 0.030 | 0.030 |
| 5g_core | 0.000 | 0.164 |
| mission_backend | 0.000 | 0.164 |
| security_monitor | 0.000 | 0.161 |
| edge_node | 0.000 | 0.160 |
| ground_control_station | 0.000 | 0.158 |

### protocol

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| UDP | 0.500 | 0.169 |
| MAVLink-like | 0.478 | 0.169 |
| HTTPS | 0.009 | 0.164 |
| HTTP | 0.009 | 0.160 |
| TCP | 0.004 | 0.169 |
| MQTT | 0.000 | 0.169 |

### network_slice_id

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| slice_1 | 0.965 | 0.165 |
| slice_2 | 0.023 | 0.166 |
| slice_3 | 0.008 | 0.168 |
| slice_4 | 0.004 | 0.163 |
| slice_6 | 0.000 | 0.176 |
| slice_5 | 0.000 | 0.162 |

### authentication_status

| Value | Cosim share | Synthetic share |
|---|---:|---:|
| success | 0.991 | 0.747 |
| failed | 0.009 | 0.011 |
| expired | 0.000 | 0.229 |
| unknown | 0.000 | 0.012 |
