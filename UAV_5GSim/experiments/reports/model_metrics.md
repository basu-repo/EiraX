# Baseline machine-learning evaluation

Target classes are `normal`, `suspicious`, and `malicious`. All partitions use complete missions. Label fields, response fields, identifiers, addresses, ports, timestamps, and session IDs are excluded from features. Balanced class weights address unequal raw row counts without deleting or fabricating measured evidence.

| Dataset | Accuracy | Macro-F1 | False-alarm rate | Mean detection delay (s) |
|---|---:|---:|---:|---:|
| Simulation-derived | 0.9138 | 0.7843 | 0.0184 | 1.7 |
| Synthetic reference | 0.9988 | 0.9986 | 0.0006 | 0.0 |

The JSON report contains precision, recall, F1 score, confusion matrices, feature importance, per-feature-set results, mission lists, and per-mission detection delay. Distribution statistics are provided separately for interpreting realism gaps.
