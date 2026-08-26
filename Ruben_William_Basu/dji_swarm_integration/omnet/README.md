# Isolated one-UGV/one-UAV OMNeT++ overlay

This directory contains local copies of the required Gazebo-driven mobility,
position scheduler and metrics-server implementations. It has no source or NED
dependency on the original imported project, Simu5G or FLORA.

The INET Wi-Fi network tracks `ugv` and `uav` from the ROS pose bridge on TCP
5555. Link metrics for the single UGV-to-UAV flow are served on TCP 5556.

## Build

```bash
source /home/basudeo/omnetpp-6.0.1/setenv
cd /home/basudeo/Documents/EiraX/Ruben_William_Basu/decentralized_swarm_integration/omnet
bash build.sh
```

## Run

```bash
./out/gcc-release/omnet \
  -u Cmdenv \
  -n .:uav_ugv:/home/basudeo/inet/src \
  -l /home/basudeo/inet/src/INET \
  omnetpp.ini
```

Geometric distance inside OMNeT++ is for propagation modelling and validation.
The ROS role layer consumes link quality metrics, not ground-truth distance.
