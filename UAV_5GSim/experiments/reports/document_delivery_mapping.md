# Experiment-document delivery mapping

The supplied instructions use `~/uas_lab` as an example laboratory root. This
project uses `/home/basudeo/Documents/EiraX/UAV_5GSim` instead.

| Document path | Project path |
|---|---|
| `data/` | `experiments/data/` |
| `data_ref/` | `data_ref/` |
| `logs/` | `experiments/logs/` |
| `manifests/` | `experiments/manifests/` |
| `results/` | `experiments/reports/` (the documents give `results/` no defined content; every result is reported under `reports/`) |
| `trace/` | `experiments/trace/` |
| `reports/` | `experiments/reports/` |
| `sim/bridge/` | `sim/bridge/` |
| `sim/simu5g/` | not created. Simu5G is a shared install at `../UGV_UAV_5G_CoSimulation/sim/simu5g/Simu5G`, and every run keeps its own complete scenario configuration and OMNeT++ results under `experiments/logs/<run_id>/network/{input,raw}/`, so a central copy would only duplicate them |
| `schema_reference.json` | `schema_reference.json` |
| `validate_dataset.py` | `validate_dataset.py` |

## Which document each folder serves

The setup document (Document 1) and the experiment document (Document 2) are
delivered side by side:

| Content | Location |
|---|---|
| Document 1 completion evidence | `experiments/document1/` |
| Document 2 experiment evidence | `experiments/logs/`, `experiments/manifests/`, `experiments/trace/` |
| Shared reports for both | `experiments/reports/` |

Document 1 carries its own 1,901-row dataset, its eight scenario runs (baseline,
suspicious and the six attack classes), the single measured benign ROS 2 bag
that all eight reuse, the shared-clock join audit and the behavioural flag
audit. Document 2 carries the 46 network runs and the canonical 25,490-row
dataset.

Run `python verify_submission.py` to recheck every requirement of both
documents against the delivered evidence.

The required canonical final dataset is
`experiments/data/drone_network_telemetry_cosim.csv`. It contains 25,490 rows,
44 columns and 37 complete mission identifiers. The earlier 28-mission core is
preserved as `experiments/data/drone_network_telemetry_cosim_core.csv`.

## Measured-flight reuse

There are five unique measured Gazebo/PX4/ROS 2 mission recordings. Controlled
network comparisons reuse those recordings through hard links and record the
source in `physical_source.json`. This keeps vehicle motion fixed while changing
one network or security condition. Every run still has a manifest, ROS bag,
pose trace, Simu5G result, bridge log, per-run dataset and validator report. No
reused trace is described as a newly executed physical flight.

## Network-slice scope

The experiment implements the document's single, dual and triple profiles as
separate logical application flows for mission, video, telemetry and backend
traffic. It does not claim scheduler-enforced 5G-core isolation because the
installed Simu5G source does not provide a native network-slice module.

## Baseline-notebook scope

The source material did not include the original synthetic baseline notebook.
The supplied 44-column schema and synthetic reference dataset were preserved,
and the compatible evaluation uses them without schema changes. The project
therefore demonstrates baseline compatibility, but does not claim byte-for-byte
execution of an unavailable original notebook. The reconstructed compatibility
notebook was executed successfully after the canonical dataset update.
