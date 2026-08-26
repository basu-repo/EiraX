#!/usr/bin/env python3
"""
Analyze OMNeT++ .vec/.sca results and generate network metric plots (SVG) without
external plotting dependencies.
"""

import argparse
import csv
import math
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def _write_svg_line_plot(
    output_file: Path,
    xs: List[float],
    ys: List[float],
    title: str,
    x_label: str,
    y_label: str,
    line_color: str = "#1f77b4",
) -> bool:
    if not xs or not ys or len(xs) != len(ys):
        return False

    width = 1200
    height = 700
    margin_left = 90
    margin_right = 30
    margin_top = 60
    margin_bottom = 90

    plot_w = width - margin_left - margin_right
    plot_h = height - margin_top - margin_bottom

    x_min, x_max = min(xs), max(xs)
    y_min, y_max = min(ys), max(ys)

    if x_max == x_min:
        x_max = x_min + 1.0
    if y_max == y_min:
        y_max = y_min + 1.0

    def map_x(x: float) -> float:
        return margin_left + (x - x_min) / (x_max - x_min) * plot_w

    def map_y(y: float) -> float:
        return margin_top + (1.0 - (y - y_min) / (y_max - y_min)) * plot_h

    points = " ".join(f"{map_x(x):.2f},{map_y(y):.2f}" for x, y in zip(xs, ys))
    point_marks = ""
    if len(xs) <= 200:
        point_marks = "\n  ".join(
            f'<circle cx="{map_x(x):.2f}" cy="{map_y(y):.2f}" r="3.5" fill="{line_color}"/>'
            for x, y in zip(xs, ys)
        )

    grid_lines = []
    ticks = 10
    for i in range(ticks + 1):
        gx = margin_left + i * plot_w / ticks
        gy = margin_top + i * plot_h / ticks
        xv = x_min + i * (x_max - x_min) / ticks
        yv = y_max - i * (y_max - y_min) / ticks
        grid_lines.append(
            f'<line x1="{gx:.2f}" y1="{margin_top}" x2="{gx:.2f}" y2="{margin_top + plot_h}" stroke="#e6e6e6" stroke-width="1"/>'
        )
        grid_lines.append(
            f'<line x1="{margin_left}" y1="{gy:.2f}" x2="{margin_left + plot_w}" y2="{gy:.2f}" stroke="#e6e6e6" stroke-width="1"/>'
        )
        grid_lines.append(
            f'<text x="{gx:.2f}" y="{margin_top + plot_h + 24}" text-anchor="middle" font-size="12" fill="#333">{xv:.2f}</text>'
        )
        grid_lines.append(
            f'<text x="{margin_left - 12}" y="{gy + 4:.2f}" text-anchor="end" font-size="12" fill="#333">{yv:.2f}</text>'
        )

    svg = f'''<?xml version="1.0" encoding="UTF-8"?>
<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
  <rect x="0" y="0" width="{width}" height="{height}" fill="white"/>
  <text x="{width/2:.1f}" y="34" text-anchor="middle" font-size="24" fill="#111">{title}</text>
  {"".join(grid_lines)}
  <line x1="{margin_left}" y1="{margin_top + plot_h}" x2="{margin_left + plot_w}" y2="{margin_top + plot_h}" stroke="#222" stroke-width="2"/>
  <line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top + plot_h}" stroke="#222" stroke-width="2"/>
  <polyline points="{points}" fill="none" stroke="{line_color}" stroke-width="2.5"/>
  {point_marks}
  <text x="{margin_left + plot_w/2:.1f}" y="{height - 35}" text-anchor="middle" font-size="16" fill="#111">{x_label}</text>
  <text x="28" y="{margin_top + plot_h/2:.1f}" transform="rotate(-90 28,{margin_top + plot_h/2:.1f})" text-anchor="middle" font-size="16" fill="#111">{y_label}</text>
</svg>
'''

    output_file.write_text(svg, encoding="utf-8")
    return True


def _latest_matching(directory: Path, pattern: str) -> Optional[Path]:
    files = sorted(directory.glob(pattern), key=lambda p: p.stat().st_mtime, reverse=True)
    return files[0] if files else None


def _parse_vector_definitions(vec_file: Path) -> Dict[int, Tuple[str, str]]:
    definitions: Dict[int, Tuple[str, str]] = {}
    with vec_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line.startswith("vector "):
                continue
            parts = line.strip().split(maxsplit=3)
            if len(parts) < 4:
                continue
            try:
                vec_id = int(parts[1])
            except ValueError:
                continue
            module = parts[2]
            name = _clean_vector_name(parts[3])
            definitions[vec_id] = (module, name)
    return definitions


def _clean_vector_name(raw: str) -> str:
    raw = raw.strip()
    if raw.startswith('"'):
        end = raw.find('"', 1)
        if end >= 0:
            return raw[1:end]
    return raw.split()[0] if raw else ""


def _metric_key(module: str, name: str) -> Optional[str]:
    if module.endswith("ugv.wlan[0].radio") and name.startswith("minSnir:vector"):
        return "snir_ugv"
    if module.endswith("uav.wlan[0].radio") and name.startswith("minSnir:vector"):
        return "snir_uav"
    if module.endswith("ugv.LoRaGWNic.radio") and name.startswith("minSnir:vector"):
        return "snir_ugv"
    if module.endswith("uav.LoRaNic.radio") and name.startswith("minSnir:vector"):
        return "snir_uav"
    if module.endswith("ugv.wlan[0].radio") and name.startswith("packetErrorRate:vector"):
        return "per_ugv"
    if module.endswith("uav.wlan[0].radio") and name.startswith("packetErrorRate:vector"):
        return "per_uav"
    if module.endswith("ugv.LoRaGWNic.radio") and name.startswith("packetErrorRate:vector"):
        return "per_ugv"
    if module.endswith("uav.LoRaNic.radio") and name.startswith("packetErrorRate:vector"):
        return "per_uav"
    if module.endswith("ugv.app[0]") and name.startswith("endToEndDelay:vector"):
        return "e2e_delay_ugv"
    if module.endswith("ugv.app[0]") and name.startswith("throughput:vector"):
        return "throughput_ugv"
    if module.endswith("uav.app[0]") and name.startswith("throughput:vector"):
        return "throughput_uav"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("queueLength:vector"):
        return "queue_len_ugv"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("queueLength:vector"):
        return "queue_len_uav"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_ugv"
    if module.endswith("ugv.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_ugv"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_uav"
    if module.endswith("uav.wlan[0].mac.dcf.channelAccess.pendingQueue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_uav"
    if module.endswith("ugv.eth[0].queue") and name.startswith("queueLength:vector"):
        return "queue_len_ugv"
    if module.endswith("uav.LoRaNic.queue") and name.startswith("queueLength:vector"):
        return "queue_len_uav"
    if module.endswith("ugv.eth[0].queue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_ugv"
    if module.endswith("ugv.eth[0].queue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_ugv"
    if module.endswith("uav.LoRaNic.queue") and name.startswith("incomingDataRate:vector"):
        return "in_data_rate_uav"
    if module.endswith("uav.LoRaNic.queue") and name.startswith("outgoingDataRate:vector"):
        return "out_data_rate_uav"
    if module.endswith("networkServer.app[0]") and name == "Vector of RSSI per node":
        return "rssi_lora"
    if module.endswith("networkServer.app[0]") and name == "Vector of SNIR per node":
        return "snir_lora_server"
    if module.endswith("uav.app[0]") and name == "SF Vector":
        return "lora_sf"
    if module.endswith("uav.app[0]") and name == "TP Vector":
        return "lora_tx_power_dbm"
    return None


def _parse_metric_series(vec_file: Path, id_to_metric: Dict[int, str]) -> Dict[str, List[Tuple[float, float]]]:
    series: Dict[str, List[Tuple[float, float]]] = {m: [] for m in sorted(set(id_to_metric.values()))}
    with vec_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if not line:
                continue
            c0 = line[0]
            if c0 < "0" or c0 > "9":
                continue
            parts = line.strip().split()
            if len(parts) < 4:
                continue
            try:
                vec_id = int(parts[0])
            except ValueError:
                continue
            metric = id_to_metric.get(vec_id)
            if not metric:
                continue
            try:
                t = float(parts[2])
                v = float(parts[3])
            except ValueError:
                continue
            series[metric].append((t, v))

    # Transform units for readability.
    for key in ("snir_ugv", "snir_uav"):
        if key in series:
            converted = []
            for t, v in series[key]:
                if v > 0:
                    converted.append((t, 10.0 * math.log10(v)))
            series[key] = converted

    if "e2e_delay_ugv" in series:
        series["e2e_delay_ugv"] = [(t, v * 1000.0) for t, v in series["e2e_delay_ugv"]]

    return series


def _read_pathloss_rssi(pathloss_csv: Optional[Path]) -> List[Tuple[float, float]]:
    if pathloss_csv is None or not pathloss_csv.exists():
        return []
    points = []
    with pathloss_csv.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                points.append((float(row["time"]), float(row["rssi"])))
            except Exception:
                continue
    return points


def _parse_plot_crop_arg(plot_arg: Optional[str]) -> Tuple[Optional[float], str]:
    """
    Parse --plot argument.
    Returns (fixed_start_time, mode) where mode is one of: none, ugv-motion, uav-motion, any-motion.
    """
    if plot_arg is None:
        return None, "none"
    value = plot_arg.strip()
    if not value:
        return None, "none"
    try:
        return float(value), "none"
    except ValueError:
        pass

    mode = value.lower()
    if mode in ("none", "ugv-motion", "uav-motion", "any-motion"):
        return None, mode
    raise SystemExit(
        "Invalid --plot value. Use a number of seconds (e.g. 10), or one of: none, ugv-motion, uav-motion, any-motion"
    )


def _detect_motion_start_from_pathloss(
    pathloss_csv: Optional[Path],
    mode: str,
    threshold_m: float,
) -> Optional[float]:
    if pathloss_csv is None or not pathloss_csv.exists():
        return None

    initial = None
    with pathloss_csv.open("r", newline="", encoding="utf-8", errors="ignore") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row["time"])
                uav = (
                    float(row["uav_x"]),
                    float(row["uav_y"]),
                    float(row["uav_z"]),
                )
                ugv = (
                    float(row["ugv_x"]),
                    float(row["ugv_y"]),
                    float(row["ugv_z"]),
                )
            except Exception:
                continue

            if initial is None:
                initial = (uav, ugv)
                continue

            (uav0, ugv0) = initial
            duav = math.sqrt((uav[0] - uav0[0]) ** 2 + (uav[1] - uav0[1]) ** 2 + (uav[2] - uav0[2]) ** 2)
            dugv = math.sqrt((ugv[0] - ugv0[0]) ** 2 + (ugv[1] - ugv0[1]) ** 2 + (ugv[2] - ugv0[2]) ** 2)

            if mode == "ugv-motion" and dugv >= threshold_m:
                return t
            if mode == "uav-motion" and duav >= threshold_m:
                return t
            if mode == "any-motion" and (duav >= threshold_m or dugv >= threshold_m):
                return t
    return None


def _filter_series_from_time(data: List[Tuple[float, float]], start_time: Optional[float]) -> List[Tuple[float, float]]:
    if start_time is None:
        return data
    return [(t, v) for t, v in data if t >= start_time]


def _trim_series_end(data: List[Tuple[float, float]], trim_seconds: float) -> List[Tuple[float, float]]:
    if not data or trim_seconds <= 0:
        return data
    max_t = max(t for t, _ in data)
    cutoff = max_t - trim_seconds
    trimmed = [(t, v) for t, v in data if t <= cutoff]
    return trimmed if len(trimmed) >= 2 else data


def _parse_sca_summary(sca_file: Optional[Path]) -> Dict[str, float]:
    summary: Dict[str, float] = {}
    if sca_file is None or not sca_file.exists():
        return summary

    sent_re = re.compile(r"^scalar\s+\S*uav\.app\[0\]\s+packetSent:count\s+([\deE+\-.]+)")
    recv_re = re.compile(r"^scalar\s+\S*ugv\.app\[0\]\s+packetReceived:count\s+([\deE+\-.]+)")
    lora_sent_re = re.compile(r"^scalar\s+\S*uav\.app\[0\]\s+sentPackets\s+([\deE+\-.]+)")
    lora_recv_re = re.compile(r"^scalar\s+\S*networkServer\.app\[0\]\s+totalReceivedPackets\s+([\deE+\-.]+)")
    lora_ns_der_re = re.compile(r"^scalar\s+\S*networkServer\.app\[0\]\s+LoRa_NS_DER\s+([\deE+\-.]+)")
    lora_gw_der_re = re.compile(r"^scalar\s+\S*ugv\.packetForwarder\s+LoRa_GW_DER\s+([\deE+\-.]+)")
    lora_radio_der_re = re.compile(r'^scalar\s+\S*ugv\.LoRaGWNic\.radio\s+"DER - Data Extraction Rate"\s+([\deE+\-.]+)')

    with sca_file.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = sent_re.match(line)
            if m:
                summary["packets_sent_uav"] = float(m.group(1))
                continue
            m = recv_re.match(line)
            if m:
                summary["packets_received_ugv"] = float(m.group(1))
                continue
            m = lora_sent_re.match(line)
            if m:
                summary["packets_sent_uav"] = float(m.group(1))
                continue
            m = lora_recv_re.match(line)
            if m:
                summary["packets_received_network_server"] = float(m.group(1))
                continue
            m = lora_ns_der_re.match(line)
            if m:
                summary["lora_network_server_der"] = float(m.group(1))
                continue
            m = lora_gw_der_re.match(line)
            if m:
                summary["lora_gateway_der"] = float(m.group(1))
                continue
            m = lora_radio_der_re.match(line)
            if m:
                summary["lora_radio_der"] = float(m.group(1))

    sent = summary.get("packets_sent_uav", 0.0)
    recv = summary.get("packets_received_ugv", summary.get("packets_received_network_server", 0.0))
    if sent > 0:
        summary["pdr_percent"] = 100.0 * recv / sent
        summary["loss_percent"] = 100.0 - summary["pdr_percent"]
    return summary


def _write_summary_txt(output_file: Path, series: Dict[str, List[Tuple[float, float]]], summary: Dict[str, float]) -> None:
    lines = ["Network Metrics Summary", "=" * 40]

    for metric, data in sorted(series.items()):
        if not data:
            continue
        vals = [v for _, v in data]
        lines.append(f"{metric}: count={len(vals)}, min={min(vals):.6g}, max={max(vals):.6g}, avg={sum(vals)/len(vals):.6g}")

    if summary:
        lines.append("")
        lines.append("Packet Summary")
        lines.append("-" * 40)
        for k in sorted(summary.keys()):
            lines.append(f"{k}: {summary[k]:.6g}")

    output_file.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot OMNeT network metrics from .vec/.sca")
    parser.add_argument("--results", default="results", help="Results directory")
    parser.add_argument("--vec", default=None, help="Specific .vec file")
    parser.add_argument("--sca", default=None, help="Specific .sca file")
    parser.add_argument("--pathloss-csv", default=None, help="Path loss CSV (for RSSI plot)")
    parser.add_argument(
        "--config",
        default=None,
        help="OMNeT config suffix after 'Communication-GazeboBridge-' (e.g. WiFi, 5G, LoRa)",
    )
    parser.add_argument(
        "--config-prefix",
        default=None,
        help="Full file prefix to auto-pick latest run (deprecated; use --config)",
    )
    parser.add_argument(
        "--output-dir",
        default=None,
        help="Output directory for plots/summary/CSV (default: results/analysis/<run-stem>)",
    )
    parser.add_argument("--csv-dir", default=None, help="CSV output directory (default: output dir)")
    parser.add_argument("--plot-dir", default=None, help="Plot output directory (default: output dir)")
    parser.add_argument("--write-csv", action="store_true", help="Write combined metric CSV (disabled by default to save space)")
    parser.add_argument(
        "--plot",
        default=None,
        help="Crop plots from a fixed time or motion start. Use seconds (e.g. 10) or: ugv-motion, uav-motion, any-motion",
    )
    parser.add_argument(
        "--motion-threshold-m",
        type=float,
        default=0.5,
        help="Motion detection threshold in meters for *-motion plot start modes (default: 0.5)",
    )
    args = parser.parse_args()

    result_dir = Path(args.results)
    config_prefix = args.config_prefix or (
        f"Communication-GazeboBridge-{args.config}" if args.config else "Communication-GazeboBridge"
    )
    fixed_plot_start_time, plot_start_mode = _parse_plot_crop_arg(args.plot)

    vec_file = Path(args.vec) if args.vec else _latest_matching(result_dir, f"{config_prefix}-#*.vec")
    if vec_file is None or not vec_file.exists():
        raise SystemExit(f"No .vec file found for prefix '{config_prefix}' in {result_dir}")

    sca_file = Path(args.sca) if args.sca else _latest_matching(result_dir, f"{config_prefix}-#*.sca")
    base = vec_file.stem
    output_dir = Path(args.output_dir) if args.output_dir else vec_file.parent / "analysis" / base
    csv_dir = Path(args.csv_dir) if args.csv_dir else output_dir
    plot_dir = Path(args.plot_dir) if args.plot_dir else output_dir
    csv_dir.mkdir(parents=True, exist_ok=True)
    plot_dir.mkdir(parents=True, exist_ok=True)

    out_csv = csv_dir / f"{base}_network_metrics.csv"
    summary_txt = csv_dir / f"{base}_network_summary.txt"

    defs = _parse_vector_definitions(vec_file)
    id_to_metric: Dict[int, str] = {}
    for vec_id, (module, name) in defs.items():
        mk = _metric_key(module, name)
        if mk is not None:
            id_to_metric[vec_id] = mk

    series = _parse_metric_series(vec_file, id_to_metric)

    pathloss_csv = None
    if args.pathloss_csv:
        pathloss_csv = Path(args.pathloss_csv)
    else:
        guessed = _latest_matching(vec_file.parent, f"{config_prefix}-*_path_loss.csv")
        pathloss_csv = guessed
    rssi_points = _read_pathloss_rssi(pathloss_csv)
    if rssi_points:
        series["rssi_estimated"] = rssi_points

    plot_start_time: Optional[float] = fixed_plot_start_time
    plot_start_reason = None
    if plot_start_time is None and plot_start_mode != "none":
        plot_start_time = _detect_motion_start_from_pathloss(pathloss_csv, plot_start_mode, args.motion_threshold_m)
        if plot_start_time is not None:
            plot_start_reason = plot_start_mode

    # Export combined CSV only when explicitly requested (can be very large).
    if args.write_csv:
        with out_csv.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["metric", "time", "value"])
            for metric, data in sorted(series.items()):
                for t, v in data:
                    writer.writerow([metric, f"{t:.9f}", f"{v:.9f}"])

    # Plot selected metrics
    plot_specs = {
        "rssi_estimated": ("Estimated RSSI Over Time", "Time (s)", "RSSI (dBm)", "#c92a2a"),
        "rssi_lora": ("LoRa RSSI Over Time", "Time (s)", "RSSI (dBm)", "#c92a2a"),
        "snir_ugv": ("UGV Min SNIR Over Time", "Time (s)", "SNIR (dB)", "#0057b8"),
        "snir_uav": ("UAV Min SNIR Over Time", "Time (s)", "SNIR (dB)", "#7b2cbf"),
        "snir_lora_server": ("LoRa Server SNIR Over Time", "Time (s)", "SNIR (dB)", "#0057b8"),
        "per_ugv": ("UGV Packet Error Rate Over Time", "Time (s)", "Packet Error Rate", "#2f9e44"),
        "per_uav": ("UAV Packet Error Rate Over Time", "Time (s)", "Packet Error Rate", "#1c7c54"),
        "e2e_delay_ugv": ("UGV End-to-End Delay Over Time", "Time (s)", "Delay (ms)", "#d9480f"),
        "throughput_ugv": ("UGV Throughput Over Time", "Time (s)", "Throughput (bps)", "#0b7285"),
        "throughput_uav": ("UAV Throughput Over Time", "Time (s)", "Throughput (bps)", "#1864ab"),
        "queue_len_ugv": ("UGV Queue Length", "Time (s)", "Packets", "#2b8a3e"),
        "queue_len_uav": ("UAV Queue Length", "Time (s)", "Packets", "#364fc7"),
        "in_data_rate_ugv": ("UGV MAC Queue Incoming Data Rate", "Time (s)", "Data Rate (bps)", "#2b8a3e"),
        "out_data_rate_ugv": ("UGV MAC Queue Outgoing Data Rate", "Time (s)", "Data Rate (bps)", "#0ca678"),
        "in_data_rate_uav": ("UAV MAC Queue Incoming Data Rate", "Time (s)", "Data Rate (bps)", "#5c7cfa"),
        "out_data_rate_uav": ("UAV MAC Queue Outgoing Data Rate", "Time (s)", "Data Rate (bps)", "#364fc7"),
    }

    made = 0
    for key, (title, xl, yl, color) in plot_specs.items():
        data = _filter_series_from_time(series.get(key, []), plot_start_time)
        if key == "out_data_rate_uav":
            data = _trim_series_end(data, trim_seconds=2.0)
        if not data:
            continue
        xs = [t for t, _ in data]
        ys = [v for _, v in data]
        out = plot_dir / f"{base}_{key}.svg"
        if _write_svg_line_plot(out, xs, ys, title, xl, yl, color):
            print(f"✓ {out}")
            made += 1

    summary = _parse_sca_summary(sca_file)
    _write_summary_txt(summary_txt, series, summary)

    print(f"\nParsed vec: {vec_file}")
    if sca_file:
        print(f"Parsed sca: {sca_file}")
    if pathloss_csv and pathloss_csv.exists():
        print(f"RSSI source: {pathloss_csv}")
    elif series.get("rssi_lora"):
        print("RSSI source: LoRa vector")
    else:
        print("RSSI source: not found (run path_loss_analysis.py first for RSSI plot)")
    if plot_start_time is None:
        if args.plot is None:
            print("Plot start time: full run (use --plot <seconds|ugv-motion|uav-motion|any-motion> to crop startup idle period)")
        else:
            print(f"Plot start time: full run (--plot {args.plot!r} did not detect motion)")
    elif plot_start_reason:
        print(f"Plot start time: {plot_start_time:.3f}s (auto-detected via {plot_start_reason})")
    else:
        print(f"Plot start time: {plot_start_time:.3f}s (fixed)")
    print("Plot tail trim: out_data_rate_uav last 2.0s removed (to suppress shutdown spike)")
    print(f"Output dir: {output_dir}")
    if args.write_csv:
        print(f"Combined metric CSV: {out_csv}")
    else:
        print("Combined metric CSV: skipped (use --write-csv to enable)")
    print(f"Summary: {summary_txt}")
    print(f"Plots generated: {made}")


if __name__ == "__main__":
    main()
