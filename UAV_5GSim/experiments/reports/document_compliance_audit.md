# Experiment-document compliance audit

The canonical final dataset is now
`experiments/data/drone_network_telemetry_cosim.csv` (25,490 rows, 44 columns,
37 mission identifiers). The earlier core benchmark is preserved as
`experiments/data/drone_network_telemetry_cosim_core.csv`.

## Complete

- Measured Gazebo/PX4 mission evidence, ROS 2 bags and exported pose traces.
- Simu5G scalar/vector evidence and exported network metrics.
- One row per flow and aggregation window with documented derivations.
- Scenario manifests, time-local label windows, provenance and leakage audits.
- Exact 44-column schema and response mapping.
- Three benign baseline repetitions.
- Slow, normal and fast mobility sensitivity; the fast measured-geometry trace
  and all copies of its manifest now record 8.0 m/s.
- Five-, ten- and twenty-minute duration sensitivity.
- One-, three- and five-UAV network-load sensitivity.
- One-, two- and three-cell topology sensitivity.
- Low, medium and high background traffic.
- Good, moderate and edge wireless conditions.
- Single, dual and triple logical application-flow slice profiles.
- 0.5-, 1- and 5-second aggregation windows.
- Four suspicious scenario families with two runs each.
- Six malicious classes at low and high intensity.
- 30-second, 90-second and repeated multi-source denial-of-service timing.
- Hover telemetry plus a measured video flow.
- Standalone assembly and concatenation bridge utilities.
- Final validation, mission-level machine learning, confusion matrices, feature
  importance, false-alarm analysis and synthetic-data comparison.

## Provenance qualifications

- The duration route reuses measured takeoff, cruise and landing geometry. Cruise
  geometry is reversed/replayed at 0.8 m/s; it is a trace-driven simulation condition,
  not a claim that a new twenty-minute physical flight occurred.
- Three/five-UAV conditions use offset measured route replicas for background UEs.
- Logical slice profiles separate dataset/application flows. They do not claim native
  5G-core scheduler isolation because the installed Simu5G source has no native
  network-slice module.
- Aggregation and logical-slice mapping conditions reuse unchanged raw vectors by
  design; behavior-changing conditions have separate Simu5G executions.

## Outside the experiment-only document

- Live attacks that change aircraft behavior during flight.
- Physical 5G modem/core measurements and native slice enforcement.
- Hardware-in-the-loop and physical-flight validation.
- Journal-scale repetitions and confidence intervals for every attack/intensity pair.
