# 5G/UAS cyber-incident telemetry co-simulation

A PX4 drone flies a waypoint mission in Gazebo, ROS 2 records the flight, the
recorded pose drives a Simu5G 5G network, and a bridge joins the two into a
44-column cyber-incident telemetry dataset. Every telemetry value is measured
from a simulation output; the scenario manifest supplies only the labels, and
only after the measurements have been taken.

The work was done in two stages, in two folders:

| Folder | What it holds |
|---|---|
| `Baseline/` | The simulator and a first eight-run dataset of 2,440 rows: one benign run, one suspicious, one per attack class. |
| `Experiment_Cyber_Telemetry/` | A 43-run matrix over nine parameter groups, four suspicious scenario types and six attack classes at two intensities, recorded over 49 flights, giving 17,779 rows. |
| `Reporting/` | A four-page report, `main.tex` and `main.pdf`, covering both stages. |

Each of the two working folders has its own README describing every script and
every folder inside it.

## Where to start

1. `Reporting/main.pdf`, the four-page report: methodology, experiments,
   results and conclusion.
2. `Experiment_Cyber_Telemetry/experiment.ipynb`, which holds the run matrix,
   the analysis and the figures with their saved outputs.
3. `Experiment_Cyber_Telemetry/reports/`, the three generated reports:
   `validation_report.md` (schema result per run, label balance, the
   provenance of every column and the leakage audit),
   `model_metrics.md` (three classifiers over four feature sets) and
   `distribution_comparison.md` (against the synthetic reference).

## What can be checked without running anything

The delivered folder is self-contained for review. Both notebooks are saved
with their outputs, so the plots and tables can be read as they were produced.

- **The datasets.** `data/drone_network_telemetry_cosim.csv` in each folder,
  plus one CSV per run beside it.
- **The schema.** `validate_dataset.py` and `schema_reference.json` run on any
  of those CSVs with nothing else installed:

      python3 validate_dataset.py data/drone_network_telemetry_cosim.csv schema_reference.json

- **The scenarios.** One YAML manifest per run in `manifests/`, giving the
  simulation settings, the injected event, the expected effects and the label
  windows.
- **The evidence per run.** `logs/<run>/` keeps the generated OMNeT++
  configuration and NED file, the flow contract, the mobility input, the
  scalar results and the run logs, so each row can be traced back to the
  flight and the network run that produced it.

## What is not included

Re-running the pipeline needs the following, none of which is part of this
folder. They are third-party installations, or bulk output that any run
regenerates.

| Not included | Why |
|---|---|
| Gazebo model assets, about 442 MB | The world file loads them by absolute path from the machine it was authored on, so it needs those paths adjusted before it will open elsewhere. |
| The PX4 SITL runtime | A prebuilt runtime kept outside this folder, referenced by `run_simulation.py` and `mission.py`. |
| OMNeT++ 6.0.1, INET 4.5 and the Simu5G 1.2.2 build | Third-party, installed separately. `sim/Simu5G` and `sim/inet4.5` are links to those installations. |
| OMNeT++ vector dumps (`.vec`), the raw per-flow metric exports (`logs/<run>_net.csv`) and the ROS 2 bag payloads (`.mcap`) | About 3.6 GB of regenerable output. The scalar results, the assembled datasets and the pose traces they were derived into are all kept. |

## Versions used

| Component | Version |
|---|---|
| Operating system | Ubuntu 24.04 |
| ROS 2 | Jazzy |
| Gazebo Sim | 8.11.0 |
| PX4 | Prebuilt SITL runtime, x500 airframe |
| OMNeT++ | 6.0.1 |
| INET | 4.5 |
| Simu5G | 1.2.2 |
| Python | scikit-learn 1.7.2, pandas 2.3.3, numpy 1.26.4 |
