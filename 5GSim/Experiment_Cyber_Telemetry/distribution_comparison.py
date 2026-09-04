#!/usr/bin/env python3
"""Compare the co-simulation dataset with the synthetic reference, column by column.

Numeric columns are compared by mean, standard deviation and median, and the
categorical columns by the share of each value. This is the distribution part
of the synthetic comparison; the model part is in reports/model_metrics.md.

    ./distribution_comparison.py
"""

from pathlib import Path

import pandas as pd

HERE = Path(__file__).resolve().parent
CATEGORICAL = ["incident_label", "attack_type", "source_component", "protocol",
               "network_slice_id", "authentication_status"]


def main():
    cosim = pd.read_csv(HERE / "data/drone_network_telemetry_cosim.csv")
    synthetic = pd.read_csv(HERE / "data_ref/drone_network_telemetry_dataset.csv")
    numeric = [c for c in cosim.columns if cosim[c].dtype != object
               and c not in ("source_port", "destination_port")]

    report = ["# Distribution comparison, co-simulation against the synthetic reference", "",
              f"Co-simulation: {len(cosim)} rows, {cosim.mission_id.nunique()} runs. "
              f"Synthetic reference: {len(synthetic)} rows.", "",
              "## Numeric columns", "",
              "| Column | Cosim mean | Cosim std | Cosim median | Synthetic mean | "
              "Synthetic std | Synthetic median |", "|---|---:|---:|---:|---:|---:|---:|"]
    for column in numeric:
        a, b = cosim[column], synthetic[column]
        report.append(f"| `{column}` | {a.mean():.4g} | {a.std():.4g} | {a.median():.4g} | "
                      f"{b.mean():.4g} | {b.std():.4g} | {b.median():.4g} |")

    report += ["", "## Categorical columns", ""]
    for column in CATEGORICAL:
        shares = pd.concat([cosim[column].value_counts(normalize=True).rename("cosim"),
                            synthetic[column].value_counts(normalize=True).rename("synthetic")],
                           axis=1).fillna(0.0)
        report += [f"### {column}", "", "| Value | Cosim share | Synthetic share |", "|---|---:|---:|"]
        report += [f"| {value} | {row.cosim:.3f} | {row.synthetic:.3f} |"
                   for value, row in shares.iterrows()]
        report.append("")

    (HERE / "reports/distribution_comparison.md").write_text("\n".join(report))
    print("[WROTE] reports/distribution_comparison.md")


if __name__ == "__main__":
    main()
