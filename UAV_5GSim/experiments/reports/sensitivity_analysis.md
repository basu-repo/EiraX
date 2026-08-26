# Controlled Sensitivity Results

All nine added conditions pass the authoritative 44-column validator and their run-level leakage audits. They are stored separately from the core detector benchmark because the aggregation-window conditions reuse the same measured mission and network vectors.

## Completed controlled variables

- Aggregation windows: 0.5 seconds (1200 rows), 1 second (core reference), and 5 seconds (120 rows).
- Mobility: the measured slow mission, the measured nominal mission, and a network-only fast counterfactual that preserves the measured route geometry while scaling its timestamps to 8 m/s.
- Radio condition: one fixed seed with UAV uplink power at 30, 20, and 10 dBm. Mean reported signal quality changed monotonically from -43.72 to -58.19 to -64.69 dBm; mean retransmission rate changed from 0.000063 to 0.001659 to 0.003746.
- Fleet load: one-UAV reference, three UAVs, and five UAVs. Background UAVs use the measured route with spatial offsets; the final rows describe the target UAV only.
- Radio topology: two-cell reference and a three-cell execution. The three-cell run observed 2 cumulative handovers and all three serving-cell identifiers.

## Statistical repeatability

Three independent core benign runs are summarized in `baseline_repetition_statistics.json` using run-level means, sample standard deviations, and 95% Student-t confidence intervals. Attack-class multi-seed confidence intervals remain a journal-scale expansion, not a requirement of the supplied experiment-only minimum matrix.

## Interpretation limits

- The fast case is a transparent Simu5G mobility counterfactual, not a claim that a second physical PX4 flight occurred.
- Three/five-UAV conditions test network contention; only one physical route was measured and background routes are offset replicas.
- The signal-quality field is a documented conversion from measured uplink SINR, not a modem RSSI reading.
- Window size changes aggregation and rounding behavior, not the underlying packets.
- These sensitivity rows remain out of the core machine-learning train/test split to prevent duplicated-source leakage.
