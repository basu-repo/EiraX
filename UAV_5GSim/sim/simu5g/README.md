# `sim/simu5g/` — where the 5G network scenarios actually live

The setup document creates `sim/simu5g/` as the place a Simu5G scenario is
configured and run (Listing 3 and Listing 6). This project splits that role in
two, because the Simu5G build is shared with the rest of EiraX while every
experiment run needs its own scenario configuration.

## 1. The simulator itself

Simu5G, INET and OMNeT++ are installed once and shared. `UAV_5GSim` does not
carry a second copy:

| Component | Location |
|---|---|
| Simu5G | `../../UGV_UAV_5G_CoSimulation/sim/simu5g/Simu5G` |
| INET 4.5 | `../../UGV_UAV_5G_CoSimulation/sim/simu5g/inet4.5` and `/home/basudeo/inet` |
| OMNeT++ 6.0.1 | `/home/basudeo/omnetpp-6.0.1` |

These trees are excluded from Git and are recreated from their documented
upstream versions during setup. The repository root `README.md` describes the
restore procedure.

## 2. The scenario configurations

There is no single shared scenario file, because the experiment varies the
network deliberately: across the 46 runs there are 42 distinct `omnetpp.ini`
files, 8 flow contracts and 4 NED variants. Each run therefore keeps its own
complete, self-contained configuration as tracked evidence:

```text
experiments/logs/<run_id>/network/input/
├── omnetpp.ini            # sim-time-limit, cells, traffic, attack flows
├── EiraXExperiment.ned    # network topology for this run
├── demo.xml               # INET/Simu5G scenario wiring
├── flow_contract.json     # the flows the bridge expects to find
├── uav.movements          # measured mobility, from the ROS 2 pose trace
└── scenario_manifest.yaml # labels and label windows for this run
```

The unattended live-duration automation uses
`experiments/logs/B_DURATION_20_001/network/input/` as its starting template,
because that is the 20-minute benign run, and rewrites `sim-time-limit` and
`finishTime` for the requested duration.

## 3. Running a scenario

The document's `./run -u Cmdenv -c <ConfigName> -r 0` maps onto this project as
the command `automate_live_duration_tests.py` issues, with `<run_id>` being any
directory under `experiments/logs/`:

```bash
source /home/basudeo/omnetpp-6.0.1/setenv -q
source /home/basudeo/inet/setenv -q
cd ../../UGV_UAV_5G_CoSimulation/sim/simu5g/Simu5G && source ./setenv -f

RUN=B1_BASE_001
cd experiments/logs/$RUN/network/input
simu5g -n <ned-paths> -u Cmdenv -r 0 --result-dir=../raw
```

Exporting the scalar and vector results to the per-flow CSV the bridge consumes
is the document's `opp_scavetool` step:

```bash
opp_scavetool x ../raw/baseline.sca ../raw/baseline.vec -o ../raw/network_metrics.csv
```

Completed vector and raw-metric files are gzip-compressed. Decompress a copy
with `gzip -dk` rather than deleting the archive.

## 4. Mobility input

`uav.movements` is generated from the measured ROS 2 pose trace, never hand
written. Regenerate it with the bridge exporter:

```bash
python3 sim/bridge/export_pose_trace.py \
  experiments/logs/$RUN/rosbag \
  experiments/trace/${RUN}_pose.txt \
  --movements experiments/logs/$RUN/network/input/uav.movements
```
