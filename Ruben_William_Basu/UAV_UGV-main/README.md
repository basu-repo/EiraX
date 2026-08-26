# UAV_UGV

Standalone OMNeT++ project for the UAV/UGV communication simulation and its
post-processing scripts.

This repo is intended to track only the project-specific files. External
frameworks such as OMNeT++, INET, FLORA, and Simu5G should stay outside this
repo and be installed separately.

## Repo Contents

- `src/`: custom NED and C++ Gazebo bridge modules
- `omnetpp.ini`: simulation configurations
- `ground.xml`: physical environment asset (green floor plane)
- `path_loss_analysis.py`, `network_metrics_analysis.py`, `analyze_advanced.py`:
  post-run analysis scripts
- `run_qtenv.sh`: convenience launcher (wraps the long OMNeT++ invocation)
- `images/`: project-local assets referenced by the visualizer

## External Prerequisites

The generated `Makefile` expects this layout:

```text
workspace/
  UAV_UGV/
  inet4.5/
  flora/
  Simu5G-1.4.2/
```

You also need an OMNeT++ installation available in your shell, for example:

```bash
source /path/to/omnetpp-6.2.0/setenv
```

## Build

From the project directory:

```bash
cd /path/to/UAV_UGV
make -j"$(nproc)"
```

The build links against `../inet4.5`, `../flora`, and `../Simu5G-1.4.2`.

The visualizer also expects the INET 3D assets under `../inet4.5/images/3d/`.

## Run OMNeT

WiFi with Qtenv:

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi -n src:../inet4.5/src
```

5G with Qtenv:

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-5G -n src:../inet4.5/src
```

LoRa with Qtenv:

```bash
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-LoRa-Simplex -n src:../inet4.5/src:../flora/src
```

WiFi with Cmdenv:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi -n src:../inet4.5/src
```

5G with Cmdenv:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-5G -n src:../inet4.5/src
```

LoRa with Cmdenv:

```bash
./UAV_UGV -u Cmdenv -f omnetpp.ini -c Communication-GazeboBridge-LoRa-Simplex -n src:../inet4.5/src:../flora/src
```

To keep run outputs separate:

```bash
RUN_TAG=my-run
./UAV_UGV -u Qtenv -f omnetpp.ini -c Communication-GazeboBridge-WiFi \
  -n src:../inet4.5/src \
  --result-dir="results/$RUN_TAG"
```

## Plot Results

Replace `<run_dir>` with the directory used in `--result-dir`.

Path loss and estimated RSSI:

```bash
python3 path_loss_analysis.py --results results/<run_dir> --config WiFi
```

Network metrics:

```bash
python3 network_metrics_analysis.py --results results/<run_dir> --config WiFi
```

Useful crop options for both scripts:

- `--plot 10`
- `--plot ugv-motion`
- `--plot uav-motion`

Example:

```bash
python3 network_metrics_analysis.py \
  --results results/<run_dir> \
  --config WiFi \
  --pathloss-csv results/<run_dir>/Communication-GazeboBridge-WiFi-0_path_loss.csv
```

## Live Gazebo Bridge

All `Communication-GazeboBridge-*` configs run in real-time and connect to the
ROS2 simulation in `halmstad_ws`.

**Gazebo → OMNeT (port 5555):** The `gazebo_pose_tcp_bridge` ROS2 node serves
poses over TCP. OMNeT's `GazeboPositionScheduler` polls it and drives
`GazeboDrivenMobility` for both UAV (`trackedModel=dji0`) and UGV
(`trackedModel=robot`).

**OMNeT → ROS2 (port 5556):** `OmnetMetricsServer` broadcasts live metrics once
per 100 ms:

```text
<simtime_s> <rssi_dbm> <snir_db> <per> <radio_distance_m> <pdr> <latency_s> <jitter_s>
```

- `rssi_dbm` and `snir_db` — PHY values read from received packet tags when
  the radio stack provides them
- `per` / `pdr` — sliding-window packet error rate and cumulative delivery ratio
- `latency_s` / `jitter_s` — event-based application send to radio delivery timing
- `radio_distance_m` — optional RSSI-inverted estimate. WiFi/5G use FSPL;
  native LoRa uses FLORA's `LoRaLogNormalShadowing` inverse from received RSSI,
  so it is radio-derived and does not read the UGV pose.

The `omnet_metrics_bridge` ROS2 node connects to this port and publishes
`/omnet/rssi_dbm`, `/omnet/snir_db`, `/omnet/packet_error_rate`,
`/omnet/packet_delivery_ratio`, `/omnet/latency_s`, `/omnet/jitter_s`,
`/omnet/sim_time`, and `/omnet/radio_distance`. The pose-derived geometric
link distance remains OMNeT-side for analysis and is not sent on the ROS metrics
path as `/omnet/link_distance`.

Use `run_qtenv.sh` for the quickest start:

```bash
./run_qtenv.sh wifi     # or 5g / lora
```

## Notes

- `Qtenv` is GUI mode and `Cmdenv` is terminal-only.
- LoRa needs the extra NED path `:../flora/src`.
- The analysis scripts write output back into the results directory by default.
