#!/usr/bin/env python3
"""Run the one-UAV 5G scenario in Simu5G and export the network metrics.

This is section 5 of the setup document. It builds the scenario from the
measured pose trace and the measured ROS topic rates, runs Simu5G, and exports
the scalar and vector results to a per-flow CSV on the same timeline as the
flight.

The network is deliberately small, as the document requires: one drone, one gNB
cell, one 5G core (UPF and iUPF), one edge node and one mission backend.

    ./run_scenario.py --run-id B_BASE_001
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SIMU5G = HERE / "sim/Simu5G"
INET = HERE / "sim/inet4.5"
OMNETPP = Path("/home/basudeo/omnetpp-6.0.1")

# Section 5 varies the radio, the background load and the slice profile.
# Every default reproduces the baseline scenario unchanged.
UE_TX_POWER = {"good": 26, "moderate": 18, "edge": 10}
BACKGROUND_RATE = {"none": 0.0, "low": 0.0, "medium": 100.0, "high": 400.0}
# A slice profile is the set of application flows the mission runs, and the
# slice each is carried on. Simu5G has no scheduler-enforced slicing here, so
# these are logical slices recorded per flow.
SLICE_PROFILE = {
    "single_mission_slice": [("telemetry", "slice_1"), ("command", "slice_1")],
    "mission_and_video": [("telemetry", "slice_1"), ("command", "slice_1"),
                          ("video", "slice_2")],
    "mission_telemetry_backend": [("telemetry", "slice_1"), ("command", "slice_2"),
                                  ("video", "slice_3")],
}
EXTRA_FLOW = {
    "video": {"name": "video", "activity": "video", "size": 1024, "rate": 25.0,
              "source_port": 7703, "destination_port": 8703, "protocol": "UDP",
              "session_id": "video-session"},
}

PLAYGROUND_CENTRE_M = 500.0
MINIMUM_ALTITUDE_M = 0.05
DRONE_SPACING_M = 6.0  # the fleet spawn spacing in run_simulation.start_fleet

# One gNB reproduces the original single-cell topology exactly. More cells add
# gNBs on the same core and mesh them with X2 links so handover can happen.
def build_ned(cells):
    names = ["gnb"] if cells == 1 else [f"gnb{n + 1}" for n in range(cells)]
    submodules = "\n".join(f"        {name}: gNodeB;" for name in names)
    backhaul = "\n".join(
        f"        iUpf.pppg++ <--> Eth10G <--> {name}.ppp;" for name in names)
    x2 = "\n".join(
        f"        {a}.x2++ <--> Eth10G <--> {b}.x2++;"
        for i, a in enumerate(names) for b in names[i + 1:])
    return f"""import inet.networklayer.configurator.ipv4.Ipv4NetworkConfigurator;
import inet.networklayer.ipv4.RoutingTableRecorder;
import inet.node.ethernet.Eth10G;
import inet.node.inet.Router;
import inet.node.inet.StandardHost;
import simu5g.common.binder.Binder;
import simu5g.common.carrierAggregation.CarrierAggregation;
import simu5g.nodes.Upf;
import simu5g.nodes.NR.gNodeB;
import simu5g.nodes.NR.NRUe;
import simu5g.world.radio.LteChannelControl;
network EiraXExperiment {{
    parameters: int numUe = default(1);
    submodules:
        channelControl: LteChannelControl;
        routingRecorder: RoutingTableRecorder;
        configurator: Ipv4NetworkConfigurator;
        binder: Binder;
        carrierAggregation: CarrierAggregation;
        missionBackend: StandardHost;
        edgeNode: StandardHost;
        router: Router;
        upf: Upf;
        iUpf: Upf;
{submodules}
        ue[numUe]: NRUe;
    connections:
        missionBackend.pppg++ <--> Eth10G <--> router.pppg++;
        edgeNode.pppg++ <--> Eth10G <--> router.pppg++;
        router.pppg++ <--> Eth10G <--> upf.filterGate;
        upf.pppg++ <--> Eth10G <--> iUpf.pppg++;
{backhaul}{chr(10) + x2 if x2 else ""}
}}
"""


DEMO_XML = ('<config><interface hosts="*" address="10.x.x.x" '
            'netmask="255.255.255.0"/></config>\n')


def arguments():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--run-id", required=True)
    return parser.parse_args()


# Read the exported pose trace: one header line then time/east/north/up rows.
def read_pose(trace):
    samples = []
    for line in trace.read_text().splitlines()[1:]:
        stamp, east, north, up = (float(value) for value in line.split())
        samples.append((stamp, east, north, up))
    if not samples:
        raise ValueError(f"{trace} contains no pose samples")
    return samples


# Convert the pose trace into the BonnMotion track Simu5G reads as mobility.
def write_movements(samples, path, duration, offset_east=0.0):
    track = []
    previous = -1.0
    for stamp, east, north, up in samples:
        if stamp <= previous or stamp > duration:
            continue
        previous = stamp
        track += [f"{stamp:.6f}",
                  f"{PLAYGROUND_CENTRE_M + east + offset_east:.6f}",
                  f"{PLAYGROUND_CENTRE_M + north:.6f}",
                  f"{max(MINIMUM_ALTITUDE_M, up):.6f}"]
    path.write_text(" ".join(track) + "\n")
    return len(track) // 4


# Read the measured publication rate of each recorded ROS topic.
def read_rates(measurements):
    rates = {}
    for line in measurements.read_text().splitlines()[1:]:
        topic, frequency, _ = line.split(",")
        try:
            rates[topic] = float(frequency)
        except ValueError:
            continue
    return rates


# Build the scenario. Rates come from the measured topic rates, so the 5G load
# reflects what the flight actually published. An attack adds one more flow.
def write_ini(path, duration, flows, seed=1, tx_power=26, background=0.0,
              cells=1, span=(500.0, 500.0), drones=1):
    apps = ""
    for flow in flows:
        # Each flow has a global slot, which fixes its ports and its receiver
        # apps, and an index within its own drone's application list. A flow
        # with a pause is sent by two applications with a gap between them.
        drone, slot, app = flow["ue"], flow["slot"], flow["app"]
        active = [(flow.get("start", 0), flow.get("end", duration))]
        if "pause" in flow:
            active = [(active[0][0], flow["pause"][0]), (flow["pause"][1], active[0][1])]
        apps += f'''
*.edgeNode.app[{slot * 2}].typename = "CbrReceiver"
*.edgeNode.app[{slot * 2}].localPort = {6000 + slot}'''
        for segment, (start, end) in enumerate(active):
            apps += f'''
*.ue[{drone}].app[{app + segment}].typename = "CbrSender"
*.ue[{drone}].app[{app + segment}].destAddress = "edgeNode"
*.ue[{drone}].app[{app + segment}].destPort = {6000 + slot}
*.ue[{drone}].app[{app + segment}].localPort = {flow.get("bind_port", flow["source_port"]) + segment * 100}
*.ue[{drone}].app[{app + segment}].PacketSize = {flow["size"]}
*.ue[{drone}].app[{app + segment}].sampling_time = {1.0 / flow["rate"]:.9f}s
*.ue[{drone}].app[{app + segment}].startTime = {start + 0.1:.1f}s
*.ue[{drone}].app[{app + segment}].finishTime = {end - 0.1:.1f}s'''
        apps += f'''
*.edgeNode.app[{slot * 2 + 1}].typename = "CbrSender"
*.edgeNode.app[{slot * 2 + 1}].destAddress = "missionBackend"
*.edgeNode.app[{slot * 2 + 1}].destPort = {8000 + slot}
*.edgeNode.app[{slot * 2 + 1}].localPort = {9000 + slot}
*.edgeNode.app[{slot * 2 + 1}].PacketSize = {flow["size"]}
*.edgeNode.app[{slot * 2 + 1}].sampling_time = {1.0 / flow["rate"]:.9f}s
*.edgeNode.app[{slot * 2 + 1}].startTime = {flow.get("start", 0) + 0.105:.3f}s
*.edgeNode.app[{slot * 2 + 1}].finishTime = {flow.get("end", duration) - 0.095:.3f}s
*.missionBackend.app[{slot}].typename = "CbrReceiver"
*.missionBackend.app[{slot}].localPort = {8000 + slot}
'''
    # Cells are spread across the ground the drone actually covers in this run,
    # so a boundary falls under the route and handover can happen.
    if cells == 1:
        radio = "*.gnb.mobility.initialX = 500m\n*.gnb.mobility.initialY = 500m"
    else:
        start_x, end_x = span
        lines = []
        for n in range(cells):
            # Spread over the route but stop short of its end, so the drone
            # spends time inside the last cell rather than only arriving there.
            x = start_x + (end_x - start_x) * n / cells
            lines.append(f"*.gnb{n + 1}.mobility.initialX = {x:.0f}m")
            lines.append(f"*.gnb{n + 1}.mobility.initialY = 500m")
        lines += ["*.gnb*.numX2Apps = " + str(cells - 1),
                  "*.gnb*.x2App[*].server.localPort = 5000 + ancestorIndex(1)"]
        for n in range(cells):
            peers = [f"gnb{m + 1}" for m in range(cells) if m != n]
            for index, peer in enumerate(peers):
                lines.append(f'*.gnb{n + 1}.x2App[{index}].client.connectAddress = '
                             f'"{peer}%x2ppp{0 if index == 0 else index}"')
        lines += ["**.dynamicCellAssociation = true", "**.enableHandover = true"]
        radio = "\n".join(lines)

    slots = len(flows)
    per_drone = [sum(2 if "pause" in f else 1 for f in flows if f["ue"] == d)
                 for d in range(drones)]
    if background:
        for drone in range(drones):
            slot = slots + drone
            apps += f'''
*.edgeNode.app[{slot * 2}].typename = "CbrReceiver"
*.edgeNode.app[{slot * 2}].localPort = {6000 + slot}
*.ue[{drone}].app[{per_drone[drone]}].typename = "CbrSender"
*.ue[{drone}].app[{per_drone[drone]}].destAddress = "edgeNode"
*.ue[{drone}].app[{per_drone[drone]}].destPort = {6000 + slot}
*.ue[{drone}].app[{per_drone[drone]}].localPort = 7800
*.ue[{drone}].app[{per_drone[drone]}].PacketSize = 1000
*.ue[{drone}].app[{per_drone[drone]}].sampling_time = {1.0 / background:.9f}s
*.ue[{drone}].app[{per_drone[drone]}].startTime = 0.1s
*.ue[{drone}].app[{per_drone[drone]}].finishTime = {duration - 0.1:.1f}s
'''
        per_drone = [n + 1 for n in per_drone]
        slots += drones
    # Two edge applications per flow, one per background slot: the background
    # traffic ends at the edge node and is not relayed to the backend.
    edge_apps = len(flows) * 2 + (slots - len(flows))

    # One block per drone: its cell attachment, its own recorded flight, and
    # the applications that belong to it.
    ue_block = "\n".join(
        f'''*.ue[{drone}].macCellId = 0
*.ue[{drone}].masterId = 0
*.ue[{drone}].nrMacCellId = 1
*.ue[{drone}].nrMasterId = 1
*.ue[{drone}].mobility.typename = "BonnMotionMobility"
*.ue[{drone}].mobility.traceFile = "uav{drone}.movements"
*.ue[{drone}].mobility.nodeId = 0
*.ue[{drone}].mobility.is3D = true
*.ue[{drone}].numApps = {per_drone[drone]}''' for drone in range(drones))

    path.write_text(f'''[General]
seed-set = {seed}
network = EiraXExperiment
sim-time-limit = {duration:.0f}s
image-path = {SIMU5G}/images
output-scalar-file = ${{resultdir}}/baseline.sca
output-vector-file = ${{resultdir}}/baseline.vec
**.routingRecorder.enabled = false
**.vector-recording = true
**.mobility.constraintAreaMinX = 0m
**.mobility.constraintAreaMinY = 0m
**.mobility.constraintAreaMinZ = 0m
**.mobility.constraintAreaMaxX = 1000m
**.mobility.constraintAreaMaxY = 1000m
**.mobility.constraintAreaMaxZ = 100m
**.mobility.initFromDisplayString = false
**.numBands = 50
**.ueTxPower = {tx_power}
**.eNodeBTxPower = 40
**.targetBler = 0.01
**.blerShift = 5
*.configurator.config = xmldoc("demo.xml")
{radio}
*.numUe = {drones}
{ue_block}
*.edgeNode.numApps = {edge_apps}
*.missionBackend.numApps = {len(flows)}
{apps}''')


# Run Simu5G and export the scalar and vector results to one CSV.
def run_simu5g(scenario_dir, results_dir, net_csv, log):
    ned_path = ":".join([str(SIMU5G / "simulations"), str(SIMU5G / "emulation"),
                         str(SIMU5G / "src"), str(INET / "src"), str(scenario_dir)])
    script = (f"set -e; source {OMNETPP}/setenv -q; source {INET}/setenv -q; "
              f"cd {SIMU5G}; source ./setenv -f; cd {scenario_dir}; "
              f"simu5g -n {ned_path} -u Cmdenv -r 0 --result-dir={results_dir}; "
              f"opp_scavetool x {results_dir}/baseline.sca {results_dir}/baseline.vec "
              f"-o {net_csv}")
    with log.open("w", encoding="utf-8") as handle:
        return subprocess.run(["bash", "-lc", script],
                              stdout=handle, stderr=subprocess.STDOUT).returncode


def main():
    args = arguments()
    run_dir = HERE / "logs" / args.run_id
    trace = HERE / "trace" / f"{args.run_id}_pose.txt"
    measurements = run_dir / "topic_measurements.csv"
    for required in (trace, measurements):
        if not required.is_file():
            print(f"[FAILED] Missing {required.relative_to(HERE)}. Run record_mission.py first.")
            return 1

    import yaml
    manifest = yaml.safe_load((HERE / "manifests" / f"{args.run_id}.yaml").read_text())
    samples = read_pose(trace)
    duration = float(manifest["simulation"]["duration_sec"])
    rates = read_rates(measurements)

    flows = [
        {"name": "telemetry", "activity": "telemetry", "size": 76,
         "rate": rates.get("/mavros/local_position/pose", 50.0),
         "source_port": 7701, "destination_port": 8701, "protocol": "UDP",
         "session_id": "telemetry-session"},
        {"name": "command", "activity": "command", "size": 200,
         "rate": max(rates.get("/mavros/state", 2.0), 0.1),
         "source_port": 7702, "destination_port": 8702, "protocol": "MAVLink-like",
         "session_id": "command-session"},
    ]
    attack = manifest.get("attack") or {}
    if attack.get("enabled"):
        # An attack may run as one window or as repeated bursts. Each interval
        # becomes its own Simu5G application, all sharing the flow's identity,
        # so the bridge sees one flow that is active only while it is bursting.
        intervals = attack.get("bursts") or [[attack["start_sec"], attack["end_sec"]]]
        for burst, (start, end) in enumerate(intervals):
            # Every application binds its port at t=0, so repeated bursts need
            # separate bind ports even though they report one source port.
            flows.append(dict(attack["flow"], start=float(start), end=float(end),
                              bind_port=attack["flow"]["source_port"] + burst))

    # A dropout is not an attack: the drone's own flow simply stops for a while.
    event = manifest.get("suspicious_event") or {}
    if event.get("type") == "telemetry_dropout":
        next(f for f in flows if f["activity"] == event.get("flow", "telemetry"))["pause"] = [
            float(event["start_sec"]), float(event["end_sec"])]

    setup = manifest["simulation"]
    cells = int(setup.get("cells", 1))
    drones = int(setup.get("drone_count", 1))
    scenario_dir = run_dir / "network"
    results_dir = scenario_dir / "results"
    results_dir.mkdir(parents=True, exist_ok=True)
    (scenario_dir / "EiraXExperiment.ned").write_text(build_ned(cells))
    (scenario_dir / "demo.xml").write_text(DEMO_XML)
    # One recorded flight per drone. Drone 0 uses the run's own trace; the rest
    # use the traces recorded alongside it in the same multi vehicle session.
    waypoints = 0
    for drone in range(drones):
        path = trace if drone == 0 else HERE / "trace" / f"{args.run_id}_pose_d{drone}.txt"
        if not path.is_file():
            print(f"[FAILED] Missing {path.relative_to(HERE)} for drone {drone}.")
            return 1
        waypoints += write_movements(read_pose(path), scenario_dir / f"uav{drone}.movements",
                                     duration, DRONE_SPACING_M * drone)
    flown = [PLAYGROUND_CENTRE_M + east for stamp, east, _, _ in samples
             if stamp <= duration]
    span = (max(flown), min(flown)) if flown else (500.0, 500.0)
    profile = SLICE_PROFILE[setup.get("network_slice_profile", "single_mission_slice")]
    attack_flows = [f for f in flows if "start" in f]
    base = {f["activity"]: f for f in flows if "start" not in f}
    flows = []
    for activity, slice_id in profile:
        flow = base.get(activity) or dict(EXTRA_FLOW[activity])
        flow["network_slice_id"] = slice_id
        flows.append(flow)
    # Every drone runs the mission flows. Only drone 0 carries the attack, so
    # the injection point stays a single identifiable source.
    fleet = []
    for drone in range(drones):
        for flow in flows:
            fleet.append(dict(flow, ue=drone,
                              drone_index=drone,
                              session_id=flow["session_id"] if drone == 0
                              else f"{flow['session_id']}-{drone}"))
    fleet += [dict(f, ue=0, drone_index=0) for f in attack_flows]
    for slot, flow in enumerate(fleet):
        flow["slot"] = slot
        flow["app"] = sum(2 if "pause" in earlier else 1
                          for earlier in fleet[:slot] if earlier["ue"] == flow["ue"])
    flows = fleet
    write_ini(scenario_dir / "omnetpp.ini", duration, flows, setup.get("seed", 1),
              UE_TX_POWER[setup.get("wireless_condition", "good")],
              BACKGROUND_RATE[setup.get("background_traffic", "low")], cells, span,
              drones)
    (scenario_dir / "flow_contract.json").write_text(json.dumps(flows, indent=1))

    print(f"[SCENARIO] {drones} UAV, {cells} gNB, 5G core, edge node, mission backend")
    print(f"[MOBILITY] {waypoints} points over {drones} recorded flight(s)")
    print(f"[FLOWS] {', '.join(f['name'] for f in flows)}")
    print(f"[SIMU5G] running {duration:.0f}s, this takes a few minutes", flush=True)

    net_csv = run_dir.parent / f"{args.run_id}_net.csv"
    log = run_dir / "simu5g.log"
    if run_simu5g(scenario_dir, results_dir, net_csv, log):
        print(f"[FAILED] Simu5G; see {log.relative_to(HERE)}")
        return 1
    if "End." not in log.read_text(errors="replace"):
        print(f"[FAILED] Simu5G did not finish; see {log.relative_to(HERE)}")
        return 1

    rows = sum(1 for _ in net_csv.open()) - 1
    print(f"[EXPORTED] {net_csv.relative_to(HERE)} ({rows} rows)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
