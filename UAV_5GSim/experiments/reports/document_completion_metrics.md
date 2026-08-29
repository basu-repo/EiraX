# Canonical document-complete dataset evaluation

Dataset: `experiments/data/drone_network_telemetry_cosim.csv`.

The extension contains 25,490 rows, 44 columns and 37 complete mission identifiers. The core benchmark remains preserved. All source derivatives were assigned to the held-out partition rather than training.

| Metric | Document-complete measured data | Synthetic reference |
|---|---:|---:|
| Accuracy | 0.9278 | 0.9988 |
| Macro-F1 | 0.7384 | 0.9986 |
| False-alarm rate | 0.0122 | 0.0006 |

The JSON report contains per-feature-group models, class precision/recall/F1, confusion matrices and feature importance.
