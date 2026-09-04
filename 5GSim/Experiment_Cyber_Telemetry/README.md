# Experiment: simulation-derived 5G/UAS cyber-incident telemetry

This folder reuses the simulator built in `../Baseline` to run a 43-run
matrix of benign, suspicious and malicious scenarios and to compare the
result with the synthetic reference dataset. Every run is described by one
manifest; the run, the bridge, the validation, the audit and the model
comparison all follow from it. The final dataset has 17,779 rows from 49
recorded flights.

`experiment_checklist.md` lists every requirement of the experiment and its
status.

## Prerequisites

The same as the Baseline folder: ROS 2 Jazzy, Gazebo Sim 8, MAVROS, the PX4
SITL runtime in `../../UGV_UAV/px4_runtime`, OMNeT++ 6.0.1 with INET 4.5 at
`~/inet`, and the `eirax` conda environment. `sim/Simu5G` is a link to the
Simu5G build in `../Baseline/sim/Simu5G`.

## How to run

Open `experiment.ipynb` and run it top to bottom. Its sections are:

| Section | What it does |
|---|---|
| 1. Setup | Points the notebook at this folder and defines the `step()` helper that runs a script and stops on failure. |
| 2. The run matrix | Reads the 43 manifests and prints the matrix by group. |
| 3. Fly and record one mission per run | Runs `record_mission.py` for every manifest. Press play in the Gazebo window when asked. Runs already recorded are skipped. |
| 4. Network scenario, bridge and validation | For each run: `run_scenario.py`, then the bridge, then the validator. A run with a validation file is skipped, so the loop can be restarted. 4.1 concatenates every run, validates the final dataset and writes the three reports. |
| 5. Results | Prints the reports. |
| 6. Analysis | 6.1 compares every run's measured effect with the effect its manifest expects; 6.2 draws the figures; 6.3 discusses realism gaps against the synthetic reference; 6.4 is the deliverables gate, a list of checks over everything on disk. |

For one run from a terminal:

```
./record_mission.py --run-id B_BASE_001          # fly and record (press play)
./run_scenario.py --run-id B_BASE_001            # Simu5G and metric export
python3 sim/bridge/assemble.py --manifest manifests/B_BASE_001.yaml \
    --ros-log logs/B_BASE_001 --net-csv logs/B_BASE_001_net.csv \
    --flows logs/B_BASE_001/network/flow_contract.json --window 1 \
    --out data/B_BASE_001_cosim.csv
python3 validate_dataset.py data/B_BASE_001_cosim.csv schema_reference.json
```

The window is the manifest's `aggregation_window_sec`. After every run has
a per-run CSV:

```
python3 sim/bridge/concat.py data/*_cosim.csv --out data/drone_network_telemetry_cosim.csv
python3 validate_dataset.py data/drone_network_telemetry_cosim.csv schema_reference.json
./baseline_model.py
./distribution_comparison.py
./audit_dataset.py
```

## Python files

| File | Purpose |
|---|---|
| `run_simulation.py` | Starts Gazebo and PX4 SITL and flies the route with nothing recorded, to check the simulator. Options: `--headless`, `--no-mission`, `--check N`, `--altitude`, `--speed`, `--timeout`. |
| `mission.py` | The offboard mission: take off, fly the route, patrol it for a set duration, or hover, then land and disarm. Not run directly. |
| `record_mission.py` | Flies and records one run from its manifest: speed, duration, drone count and mission profile come from the manifest. A run with several drones records one flight per drone in sequence. Writes the bag, the topic measurements and the pose trace. Options: `--run-id`, `--headless`. |
| `run_scenario.py` | Builds and runs the Simu5G scenario for one run: one UE per recorded flight, the mission flows, any injected attack or suspicious flow, cells, load, slices and wireless condition from the manifest. Exports the per-flow metrics CSV. Option: `--run-id`. |
| `sim/bridge/export_pose_trace.py` | Bag to mobility trace, cut at the manifest duration. Called by `record_mission.py` under the ROS 2 Python. |
| `sim/bridge/assemble.py` | The bridge: one row per flow per window from the manifest, the run folder, the network CSV and the flow contract. Metrics are measured, flags are derived from behaviour, labels are applied last. Also writes `logs/<run>/bridge.log`. |
| `sim/bridge/concat.py` | Concatenates per-run CSVs into the final dataset. |
| `validate_dataset.py` | The supplied validator; prints `RESULT: PASSED` or the failures. |
| `baseline_model.py` | Scores a random forest, gradient boosting and logistic regression on the final dataset over four feature sets (traffic metrics; plus 5G context; plus security indicators; all features) and on the synthetic reference. Writes `reports/model_metrics.md`. |
| `distribution_comparison.py` | Compares every numeric and categorical column of the final dataset with the synthetic reference. Writes `reports/distribution_comparison.md`. |
| `audit_dataset.py` | Writes `reports/validation_report.md`: the validator result for every run and the final build, the label balance, the provenance of every column, and the leakage audit that recomputes the three behavioural flags independently of the bridge and counts mismatches. |

## Folders

| Folder | Contents |
|---|---|
| `manifests/` | 43 YAML files, one per run. `B_*` are benign (baselines, window, speed, duration, drones, cells, slices, load, wireless, hover), `S1_` to `S4_` the suspicious scenarios (handover loss, failed logins, API retries, telemetry dropout, two each), `M_*` the attacks (DoS, session, identity, control, lateral, API at low and high intensity, plus a 30 s and a repeated-burst DoS). Each holds the simulation settings, the attack or event, the expected effects, the label windows and the allowed destinations. |
| `worlds/` | The Gazebo world with the route markers. |
| `logs/<run>/` | Recorded evidence for one run: `rosbag/`, `topic_measurements.csv`, `mission_trajectory.csv`, `mavros.log`, `rosbag.log`, `export.log`, `simu5g.log`, `bridge.log`, and `network/` with the generated `omnetpp.ini`, NED file, `flow_contract.json`, one mobility file per drone and the OMNeT++ `results/`. Multi-drone runs add `rosbag_d1/`, `uav1/` and the matching logs per extra drone. |
| `logs/<run>_net.csv` | The per-flow network metrics exported from Simu5G for that run. |
| `trace/` | One pose trace per flight: `<run>_pose.txt`, plus `<run>_pose_d1.txt` and so on for extra drones. |
| `data/` | One `<run>_cosim.csv` per run and the final `drone_network_telemetry_cosim.csv`. |
| `data_ref/` | The synthetic reference dataset. |
| `reports/` | `<run>_validation.txt` for every run, and the three reports: `model_metrics.md`, `distribution_comparison.md`, `validation_report.md`. |
| `results/` | Reserved for exported result files; empty in this delivery. |
| `sim/` | `bridge/` (the three bridge scripts), `Simu5G` (link to the build in Baseline) and `inet4.5` (link to the INET installation). |

## Other files

- `schema_reference.json`: the 44-column schema the validator checks against.
- `experiment_checklist.md`: the requirement checklist with the status of
  every item.
- `experiment.ipynb`: the notebook that runs the matrix and holds the
  analysis, figures and the deliverables gate.
