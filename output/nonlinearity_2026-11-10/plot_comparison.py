"""Plot fitted responses, training residuals, and both PWM 130 reruns."""
from pathlib import Path
from collections import defaultdict
from statistics import mean, stdev
import hashlib
import json
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/motor-ex6-matplotlib")
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
result = json.loads(Path(__file__).with_name("results.json").read_text())
for name, digest in result["sha256"].items():
    assert hashlib.sha256((ROOT / "logs" / name).read_bytes()).hexdigest() == digest

trials = defaultdict(list)
for name in ("device-monitor-261104-234913.log", "device-monitor-261104-235134.log"):
    for line in (ROOT / "logs" / name).read_text().splitlines():
        row = line.split(",")
        if len(row) == 12 and row[1] == "L" and row[5] == "MEASURE":
            trials[(name, int(row[2]), int(row[3]))].append(float(row[10]))
levels = defaultdict(list)
for (_, pwm, _), values in trials.items():
    assert len(values) == 14
    levels[pwm].append(mean(values))
commands = np.array(sorted(levels), float)
observed = np.array([mean(levels[p]) for p in commands])
spread = np.array([stdev(levels[p]) for p in commands])

bounds = result["boundaries"]
low = bounds["speed_min"]
span = bounds["speed_max"] - low


def prediction(model, pwm):
    x = (np.asarray(pwm) - 40) / 160
    if model == "power":
        y = x ** result["gamma"]
    else:
        y = np.polynomial.polynomial.polyval(x, result[model + "_normalised"])
    return low + span * y


models = ("linear", "quadratic", "power")
labels = ("Fitted linear", "Quadratic", "Power law")
colors = ("#1764a0", "#d16a12", "#278351")
styles = ("-", "--", ":")
for model in models:
    assert np.allclose(observed - prediction(model, commands),
                       result["models"][model]["residuals"], atol=1e-9)
    assert np.isclose(prediction(model, 130), result["models"][model]["prediction130"])

plt.rcParams.update({"font.size": 10, "svg.fonttype": "none"})
fig = plt.figure(figsize=(12, 9), layout="constrained")
grid = fig.add_gridspec(2, 2, height_ratios=(1.15, 1))
ax = fig.add_subplot(grid[0, :])
res = fig.add_subplot(grid[1, 0])
check = fig.add_subplot(grid[1, 1])
smooth = np.linspace(40, 200, 801)
for model, label, color, style in zip(models, labels, colors, styles):
    ax.plot(smooth, prediction(model, smooth), style, color=color,
            linewidth=2, label=label, zorder=2)
ax.errorbar(commands, observed, yerr=spread, fmt="o", color="#20252b",
            markersize=4.5, capsize=3, linewidth=1,
            label="Original PWM means (bars: trial SD)", zorder=4)
ax.axvline(130, color="#9ba5ae", linestyle="--", linewidth=1)
ax.set(title="A. Response fits: nearly overlapping across PWM 40-200",
       xlabel="Requested PWM", ylabel="Settled wheel speed (mm/s)",
       xlim=(35, 205), ylim=(90, 745))
ax.legend(loc="upper left", frameon=True, fontsize=9)
ax.text(0.98, 0.04, "Left wheel / forward / unloaded\nFit: original data only; PWM 130 excluded",
        transform=ax.transAxes, ha="right", va="bottom", fontsize=9,
        bbox=dict(facecolor="white", edgecolor="#d2d8de", alpha=0.95))

for model, label, color, style in zip(models, labels, colors, styles):
    res.plot(commands, observed - prediction(model, commands), style,
             marker="o", markersize=4, color=color, label=label)
res.axhline(0, color="#69737c", linewidth=1)
res.set(title="B. Residuals on the same nine training means",
        xlabel="Requested PWM", ylabel="Measured - predicted speed (mm/s)",
        xlim=(35, 205), ylim=(-0.9, 0.85))
res.legend(fontsize=8, loc="lower left")

for name, color, short in (
    ("device-monitor-261110-010828.log", "#8c5d93", "Run 01:08:28"),
    ("device-monitor-261110-011236.log", "#817148", "Run 01:12:36"),
):
    row = result["evaluation"][name]["130"]
    avg, sd = row["mean"], row["trial_sd"]
    check.axhspan(avg - sd, avg + sd, color=color, alpha=0.12)
    check.axhline(avg, color=color, linewidth=1.4,
                  label=f"{short}: {avg:.3f} +/- {sd:.3f} SD")
for i, (model, label, color) in enumerate(zip(models, labels, colors)):
    value = float(prediction(model, 130))
    check.scatter(i, value, color=color, s=55, zorder=5)
    check.annotate(f"{value:.3f}", (i, value), xytext=(0, 7),
                   textcoords="offset points", ha="center", fontsize=9)
check.set(title="C. PWM 130: predictions versus both measured reruns",
          ylabel="Settled wheel speed (mm/s)",
          xticks=range(3), xticklabels=labels, xlim=(-0.55, 2.55), ylim=(437, 460))
check.text(0.5, 0.97, "Prediction range < 0.1 mm/s",
           transform=check.transAxes, ha="center", va="top", fontsize=9)
check.legend(fontsize=8, loc="lower left", frameon=True)

for axis in (ax, res, check):
    axis.grid(True, color="#e1e6eb", linewidth=0.65)
    axis.set_axisbelow(True)
    axis.spines[["top", "right"]].set_visible(False)
fig.suptitle("Motor Exercise 6: added curvature gives no useful prediction improvement",
             fontsize=14, weight="bold")
fig.supxlabel("MEASURE only. Bars/bands show trial SD, not confidence intervals. "
              "PWM 130 comparison is retrospective.", fontsize=9)
target = ROOT / "docs/images/motor-nonlinearity-comparison-2026-11-10"
fig.savefig(target.with_suffix(".png"), dpi=180)
fig.savefig(target.with_suffix(".svg"))
print(f"Saved {target}.png and .svg")
