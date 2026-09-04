# Baseline: 5G/UAS co-simulation setup

This folder builds the co-simulator and produces the first, small dataset.
A PX4 drone flies one waypoint route in Gazebo, ROS 2 records the flight,
the recorded pose drives a Simu5G 5G network, and a bridge joins the two
into the 44-column cyber-incident telemetry schema. Eight runs were made:
one benign, one suspicious and one per attack class, giving a 2,440-row
dataset.

The experiment that follows this stage lives in `../Experiment_Cyber_Telemetry`.

## Prerequisites

None of these ship with this folder; `../README.md` lists what is missing and
which versions were used. Checking the delivered datasets, manifests, reports
and notebook needs none of them.

- Ubuntu 24.04 with ROS 2 Jazzy, Gazebo Sim 8 and MAVROS installed.
- The PX4 SITL runtime in `../../UGV_UAV/px4_runtime` (relative to the
  repository root).
- OMNeT++ 6.0.1 and INET 4.5 at `~/inet`; the Simu5G 1.2.2 build in
  `sim/Simu5G`.
- The `eirax` conda environment for the Python scripts and the notebook.

## How to run

Open `baseline_simCheck.ipynb` and run it top to bottom. Its sections are:

| Section | What it does |
|---|---|
| 1. Path setup | Points the notebook at this folder. |
| 2. Simulator check | Starts Gazebo and PX4 and flies the route once without recording. |
| 3. Run the experiment | 3.1 flies and records one mission per manifest; 3.2 runs Simu5G, the bridge and the validator one run at a time; 3.3 concatenates, validates and writes the model metrics. |
| 4. Flight comparison | Compares the eight recorded flights. |
| 5. What each attack does to the telemetry | Latency, packet rate and flag effect of each scenario against the benign run. |

Gazebo opens paused. Press play in the Gazebo window when a run asks for it;
the ROS 2 bag starts only after physics is running, so it holds the flight
and nothing else.

Every step can also be run from a terminal, in this order, for one run:

```
./run_simulation.py --check 10                     # optional simulator check
./record_mission.py --run-id B1_BENIGN_001          # fly and record
./run_scenario.py --run-id B1_BENIGN_001            # Simu5G and metric export
python3 sim/bridge/assemble.py --manifest manifests/B1_BENIGN_001.yaml \
    --ros-log logs/B1_BENIGN_001 --net-csv logs/B1_BENIGN_001_net.csv \
    --flows logs/B1_BENIGN_001/network/flow_contract.json --window 1 \
    --out data/B1_BENIGN_001_cosim.csv
python3 validate_dataset.py data/B1_BENIGN_001_cosim.csv schema_reference.json
```

Then, once every run has a per-run CSV:

```
python3 sim/bridge/concat.py data/*_cosim.csv --out data/drone_network_telemetry_cosim.csv
python3 validate_dataset.py data/drone_network_telemetry_cosim.csv schema_reference.json
./baseline_model.py
```

## Python files

| File | Purpose |
|---|---|
| `run_simulation.py` | Starts Gazebo and PX4 SITL and flies the benign mission with nothing recorded. Use it to check the simulator. Options: `--headless`, `--no-mission`, `--check N`, `--endpoint`, `--altitude`, `--speed`, `--timeout`. |
| `mission.py` | The offboard mission used by the other scripts: spawn, take off, fly to the goal, land, disarm. Not run directly. |
| `record_mission.py` | Flies one run and records it: starts Gazebo, PX4 and MAVROS, measures the rate and bandwidth of the three recorded topics, records the ROS 2 bag during the flight and exports the pose trace. Options: `--run-id`, `--headless`, `--endpoint`, `--duration`, `--altitude`, `--speed`, `--timeout`. |
| `run_scenario.py` | Builds the Simu5G scenario for one run from its manifest and pose trace, runs it and exports the scalar and vector results to a per-flow CSV. Options: `--run-id`, `--window`. |
| `sim/bridge/export_pose_trace.py` | Reads the pose topic from a bag and writes the x/y/z mobility trace Simu5G consumes. Needs the ROS 2 Python, so it runs under a sourced ROS environment. Called by `record_mission.py`. |
| `sim/bridge/assemble.py` | The bridge. Joins the manifest, the ROS run folder and the network CSV into one 44-column CSV, one row per flow per aggregation window. Every metric is measured; labels are applied last from the manifest's label windows. |
| `sim/bridge/concat.py` | Concatenates per-run CSVs into the final dataset without changing the schema. |
| `validate_dataset.py` | The supplied validator: column order, vocabularies, the signal-quality range and the response mapping. Prints `RESULT: PASSED` or the list of failures. |
| `baseline_model.py` | Trains a random forest on the final dataset and on the synthetic reference, excluding labels and identifiers, and writes `reports/model_metrics.md` with accuracy, macro-F1, confusion matrices and feature importances. |

## Folders

| Folder | Contents |
|---|---|
| `manifests/` | One YAML per run (`B1_BENIGN_001`, `S1_ANOMALY_001`, `M1_DOS_001` to `M6_API_001`): identity fields, simulation settings, the attack or event, the label windows and the allowed destinations. |
| `worlds/` | The Gazebo world with the route markers the mission reads. |
| `logs/<run>/` | Everything recorded for one run: `rosbag/` (the ROS 2 bag), `topic_measurements.csv`, `mission_trajectory.csv`, `mavros.log`, `rosbag.log`, `export.log`, `simu5g.log`, and `network/` with the generated `omnetpp.ini`, NED file, `flow_contract.json`, the mobility file and the OMNeT++ `results/`. |
| `logs/<run>_net.csv` | The per-flow network metrics exported from Simu5G for that run. |
| `trace/` | One pose trace per run, `<run>_pose.txt`, the mobility input to Simu5G. |
| `data/` | One `<run>_cosim.csv` per run and the final `drone_network_telemetry_cosim.csv`. |
| `data_ref/` | The synthetic reference dataset the co-simulated one is compared with. |
| `reports/` | `model_metrics.md`, written by `baseline_model.py`. |
| `sim/` | `bridge/` (the three bridge scripts), `Simu5G/` (the Simu5G build, also used by the experiment folder through a link) and `inet4.5` (a link to the INET installation). |

## Other files

- `schema_reference.json`: the 44-column schema, vocabularies and response
  mapping the validator checks against.
- `baseline_simCheck.ipynb`: the notebook that runs the whole stage and
  holds the plots and discussion.
