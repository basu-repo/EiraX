# Simulation-Derived 5G/UAS Experiment Status

The required telemetry pipeline and core scenario suite are complete. The preserved core CSV contains 17,720 rows, 44 columns, and 28 complete mission groups. The document-completion extension contains 25,490 rows, 44 columns, and 37 mission groups. Both pass `validate_dataset.py`; all associated run-level leakage audits pass, and every response matches the schema's attack-response mapping.

## Completed evidence

- Three nominal five-minute benign repetitions plus the pilot run.
- Two-cell mobility and measured handover evidence.
- Low, medium, and high benign background traffic.
- Two repetitions of each suspicious non-attack scenario: handover degradation, failed authentication, API retry storm, and temporary telemetry dropout.
- Six malicious classes at low and high intensity: denial of service, session hijacking, identity theft, control-protocol attack, lateral movement, and network-application attack.
- One YAML manifest, raw Simu5G scalar/vector output, exported network CSV, bridge log, 44-column CSV, provenance report, leakage audit, and validator result per run.
- Final concatenation, mission-level machine-learning split, rule baseline, logistic regression, random forest, four feature-group comparisons, confusion matrices, feature importance, false-alarm rate, and detection-delay analysis.
- Comparison with the supplied 10,000-row synthetic reference dataset whose MD5 matches the schema contract.

## Baseline result

The complete-feature random forest evaluated on held-out complete missions achieved 0.9138 accuracy, 0.7843 macro-F1, 0.0184 false-alarm rate, and 1.7 seconds mean detection delay. The synthetic reference achieved 0.9988 accuracy and 0.9986 macro-F1, showing that the synthetic data is substantially easier to classify than the simulation-derived telemetry.

## Experimental integrity

Physical flight evidence is hard-linked from completed measured PX4/Gazebo mission repetitions so controlled network comparisons do not duplicate ROS bag storage. Conditions that change network behavior use separate Simu5G executions. Aggregation-window and logical-slice mapping checks deliberately reuse unchanged raw vectors and state that provenance explicitly. Labels are applied from predeclared manifest windows only after feature derivation. No security flag is copied from an attack label.

## Completed sensitivity extension

The core detector benchmark remains unchanged to preserve its mission-level split. A separate validated sensitivity dataset adds 5,520 rows from nine controlled conditions:

- 0.5-second and 5-second aggregation, alongside the existing 1-second reference.
- A network-only fast mobility counterfactual preserving measured route geometry.
- Fixed-seed UAV uplink-power conditions at 30, 20, and 10 dBm.
- Three-UAV and five-UAV network-load conditions.
- A three-cell topology that used all three serving cells and observed two handovers.

The aggregation cases reuse unchanged raw vectors by design; the other conditions are independent Simu5G executions. The sensitivity rows are not mixed into detector training because some reuse the same measured source mission. Three independent benign repetitions now have run-level means, sample standard deviations, and 95% Student-t confidence intervals.

## Document-completion extension

The remaining experiment-only clauses were completed with a small, provenance-preserving extension:

- Five-, ten- and twenty-minute duration conditions at 0.8 m/s, using measured cruise geometry replayed as `spawn -> goal -> spawn -> goal` through the two measured waypoints.
- A 180-second hover case with a measured Simu5G video flow; 180 rows have `video_stream_activity=1` derived from received packets.
- Single, dual and triple logical application-flow slice profiles. These are explicitly not described as native 5G-core slicing.
- Separate denial-of-service timing runs for 30 seconds, 90 seconds and three repeated 10-second bursts from three sources.
- Standalone `sim/bridge/assemble.py` and `sim/bridge/concat.py`, both tested against existing measured evidence and the authoritative validator.
- A new mission-separated model evaluation and synthetic comparison. The complete-feature random forest achieved 0.9278 accuracy, 0.7384 macro-F1 and 0.0122 false-alarm rate on the document-complete held-out set.

The document-complete dataset is the required canonical file
`experiments/data/drone_network_telemetry_cosim.csv`. The original core
benchmark is preserved as
`experiments/data/drone_network_telemetry_cosim_core.csv`.

## Work outside this simulation-only completion

The following items require capabilities beyond the supplied experiment-only co-simulation workflow and are not claimed as complete:

- Live attacks that alter the moving aircraft rather than post-flight network traces.
- Native enforced 5G-core slicing; the supplied experiment is covered using clearly identified logical application-flow profiles because this installed Simu5G tree has no native slicing module.
- Real modem/radio measurements, hardware-in-the-loop execution, and physical-flight validation.
- A new physical twenty-minute flight matching the derived route, if physical rather than trace-driven validation is later required.
- Journal-scale multi-seed confidence intervals for every attack/intensity pair.
