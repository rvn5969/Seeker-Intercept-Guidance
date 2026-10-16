from pathlib import Path
import json
import sys
import pandas as pd
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
CONFIG = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "config/experiment.json"
if not CONFIG.is_absolute():
    CONFIG = (Path.cwd() / CONFIG).resolve()

cfg = json.loads(CONFIG.read_text())
csv_path = ROOT / cfg["output"]["benchmark_csv"]
summary_path = ROOT / cfg["output"]["summary_csv"]
figure_dir = ROOT / cfg["output"]["figure_dir"]
figure_dir.mkdir(parents=True, exist_ok=True)
summary_path.parent.mkdir(parents=True, exist_ok=True)

df = pd.read_csv(csv_path)
seed = int(df["benchmark_seed"].iloc[0])
rows = []
for method in ("pn", "predictive"):
    rows.append({
        "method": method,
        "runs": len(df),
        "intercept_rate": df[f"{method}_hit"].mean(),
        "median_miss_m": df[f"{method}_miss_m"].median(),
        "p95_miss_m": df[f"{method}_miss_m"].quantile(.95),
        "median_effort": df[f"{method}_effort"].median(),
        "median_position_rmse_m": df[f"{method}_position_rmse_m"].median(),
        "median_velocity_rmse_mps": df[f"{method}_velocity_rmse_mps"].median(),
    })
summary = pd.DataFrame(rows)
summary.to_csv(summary_path, index=False)
print(f"Benchmark seed: {seed}")
print(summary.to_string(index=False))

plt.figure()
plt.hist(df["pn_miss_m"], bins=40, alpha=.55, label="PN")
plt.hist(df["predictive_miss_m"], bins=40, alpha=.55, label="Predictive")
plt.xlabel("Closest-approach distance (m)")
plt.ylabel("Count")
plt.legend()
plt.tight_layout()
plt.savefig(figure_dir / "miss_distribution.png", dpi=200)
plt.close()

lat = pd.cut(df["latency_s"], bins=6)
latency = df.assign(improvement=df["pn_miss_m"] - df["predictive_miss_m"]).groupby(lat, observed=False)["improvement"].median()
centers = [x.mid * 1000 for x in latency.index]
plt.figure()
plt.plot(centers, latency.values, marker="o")
plt.axhline(0, linewidth=1)
plt.xlabel("Measurement latency (ms)")
plt.ylabel("Median PN miss - predictive miss (m)")
plt.tight_layout()
plt.savefig(figure_dir / "latency_sensitivity.png", dpi=200)
plt.close()

plt.figure()
plt.boxplot([df["pn_position_rmse_m"], df["predictive_position_rmse_m"]], tick_labels=["PN", "Predictive"])
plt.ylabel("Relative-position EKF RMSE (m)")
plt.tight_layout()
plt.savefig(figure_dir / "estimator_rmse.png", dpi=200)
plt.close()

plt.figure()
plt.boxplot([df["pn_effort"], df["predictive_effort"]], tick_labels=["PN", "Predictive"], showfliers=False)
plt.ylabel(r"Control effort $\sum u^2\Delta t$")
plt.tight_layout()
plt.savefig(figure_dir / "control_effort.png", dpi=200)
plt.close()

print(f"Wrote {summary_path.relative_to(ROOT)} and figures to {figure_dir.relative_to(ROOT)}")
