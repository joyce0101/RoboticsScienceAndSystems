"""Fit linear, quadratic and Colab power-law models for both wheels."""

from pathlib import Path
from collections import defaultdict
from statistics import mean
import csv
import json
import os

os.environ.setdefault("MPLCONFIGDIR", "/tmp/motor-fit-261117-matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/motor-fit-261117-fontcache")
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
CSV_DIR = ROOT / "output/motor_csv_2026-11-17"
FILES = {"L": "left_wheel_samples.csv", "R": "right_wheel_samples.csv"}
COLORS = {"L": "#1764a0", "R": "#c75e19"}

results = {}
fig, ax = plt.subplots(figsize=(9, 5.5), layout="constrained")
for wheel, filename in FILES.items():
    # Average samples within each trial, then average trials at each PWM.
    trials = defaultdict(list)
    with (CSV_DIR / filename).open(newline="", encoding="utf-8") as file:
        for row in csv.DictReader(file):
            if row["phase"] == "MEASURE":
                key = (int(row["pwm"]), row["source_file"], int(row["trial"]))
                trials[key].append(float(row["speed_mm_s"]))
    levels = defaultdict(list)
    for (pwm, source, trial), samples in trials.items():
        levels[pwm].append(mean(samples))

    pwm = np.array(sorted(levels), dtype=float)
    speed = np.array([mean(levels[value]) for value in pwm])
    gain, intercept = np.polyfit(pwm, speed, 1)
    predicted = gain * pwm + intercept
    residual = speed - predicted
    r2 = 1 - np.sum(residual**2) / np.sum((speed - speed.mean()) ** 2)
    rmse = np.sqrt(np.mean(residual**2))
    mae = np.mean(np.abs(residual))
    results[wheel] = {
        "csv": filename,
        "gain": float(gain),
        "intercept": float(intercept),
        "r2": float(r2),
        "rmse_mm_s": float(rmse),
        "mae_mm_s": float(mae),
        "trial_count": len(trials),
        "pwm": pwm.tolist(),
        "mean_speed_mm_s": speed.tolist(),
    }
    equation = f"{wheel}: v = {gain:.6f} PWM {intercept:+.6f}"
    print(f"{equation} mm/s; R2={r2:.8f}; RMSE={rmse:.6f} mm/s")
    ax.scatter(
        pwm, speed, color=COLORS[wheel], label=f"{wheel} measured means", zorder=3
    )
    ax.plot(pwm, predicted, color=COLORS[wheel], label=equation)

ax.set(
    title="PWM-speed linear fits", xlabel="Requested PWM", ylabel="Wheel speed (mm/s)"
)
ax.grid(True, color="#e1e6eb")
ax.set_axisbelow(True)
ax.spines[["top", "right"]].set_visible(False)
ax.legend(loc="upper left", fontsize=9)
image = ROOT / "docs/images/motor-speed-refit-2026-11-17"
image.parent.mkdir(exist_ok=True)
for suffix in ("png", "svg"):
    fig.savefig(image.with_suffix("." + suffix), dpi=180)
plt.close(fig)

report = [
    "# 左右轮 PWM—速度直线拟合",
    "",
    "读取 left_wheel_samples.csv、right_wheel_samples.csv 的 MEASURE 数据，"
    "先求每次试验均速，再求每档 PWM 均速，分别做最小二乘直线拟合。速度单位为 mm/s。",
    "",
    "| 轮子 | 公式 | PWM 范围 | R² | RMSE（mm/s） | MAE（mm/s） |",
    "| --- | --- | --- | ---: | ---: | ---: |",
]
for wheel, model in results.items():
    report.append(
        f"| {wheel} | v={model['gain']:.6f}·PWM{model['intercept']:+.6f} "
        f"| {min(model['pwm']):g}～{max(model['pwm']):g} | {model['r2']:.8f} "
        f"| {model['rmse_mm_s']:.6f} | {model['mae_mm_s']:.6f} |"
    )
report += [
    "",
    "![左右轮直线拟合](../../docs/images/motor-speed-refit-2026-11-17.png)",
    "",
]

# Exercise 6: use the same training means for every response-shape model.
VALIDATION_FILES = {
    "L": ["154254", "154406", "154908"],
    "R": ["153945", "154510", "154808"],
}
MODELS = {
    "linear": ("Fitted linear", "#1764a0", "-"),
    "quadratic": ("Quadratic", "#d27618", "--"),
    "power": ("Power law", "#38864c", ":"),
    "reference": ("Colab fixed reference", "#777777", "-."),
}


def error_metrics(measured, predicted):
    residual = measured - predicted
    return {
        "mae_mm_s": float(np.mean(np.abs(residual))),
        "rmse_mm_s": float(np.sqrt(np.mean(residual**2))),
        "predicted_mm_s": predicted.tolist(),
        "residual_mm_s": residual.tolist(),
    }


def fit_nonlinear(pwm, speed):
    """Use training endpoints and the Colab quadratic/gamma search methods."""
    x = (pwm - pwm[0]) / (pwm[-1] - pwm[0])
    y = (speed - speed[0]) / (speed[-1] - speed[0])
    quadratic = np.polyfit(x, y, 2)  # C, B, A for y = A + B*x + C*x^2.
    gamma_candidates = np.linspace(0.5, 2.0, 301)
    gamma_errors = [np.mean(np.abs(y - x**gamma)) for gamma in gamma_candidates]
    gamma = gamma_candidates[np.argmin(gamma_errors)]
    return {
        "normalization": {
            "pwm_min": float(pwm[0]), "pwm_max": float(pwm[-1]),
            "speed_min_mm_s": float(speed[0]), "speed_max_mm_s": float(speed[-1]),
        },
        "quadratic_normalized": quadratic.tolist(),
        "quadratic_pwm_coefficients": np.polyfit(pwm, speed, 2).tolist(),
        "power_gamma": float(gamma),
        "power_search": {"min": 0.5, "max": 2.0, "count": 301, "objective": "training MAE"},
    }


def predict_models(model, pwm):
    norm = model["normalization"]
    x = (pwm - norm["pwm_min"]) / (norm["pwm_max"] - norm["pwm_min"])
    low, high = norm["speed_min_mm_s"], norm["speed_max_mm_s"]
    return {
        "linear": model["gain"] * pwm + model["intercept"],
        "quadratic": low + (high - low) * np.polyval(model["quadratic_normalized"], x),
        "power": low + (high - low) * x**model["power_gamma"],
        "reference": low + (high - low) * x,
    }


def read_validation(wheel):
    trials = defaultdict(list)
    sources = [f"device-monitor-261117-{stamp}.log" for stamp in VALIDATION_FILES[wheel]]
    for source in sources:
        with (ROOT / "logs" / source).open(newline="", encoding="utf-8") as file:
            for row in csv.reader(file):
                if len(row) == 6 and row[0] == wheel and row[3] == "MEASURE":
                    key = (int(row[1]), source, int(row[2]))
                    trials[key].append(float(row[5]))
    levels = defaultdict(list)
    for (pwm, source, trial), samples in trials.items():
        levels[pwm].append(mean(samples))
    pwm = np.array(sorted(levels), dtype=float)
    return {
        "sources": sources, "trial_count": len(trials), "pwm": pwm.tolist(),
        "mean_speed_mm_s": [mean(levels[value]) for value in pwm],
        "trial_sd_mm_s": [float(np.std(levels[value], ddof=1)) for value in pwm],
    }


for wheel, model in results.items():
    pwm, speed = np.array(model["pwm"]), np.array(model["mean_speed_mm_s"])
    model.update(fit_nonlinear(pwm, speed))
    model["training"] = {
        name: error_metrics(speed, values) for name, values in predict_models(model, pwm).items()
    }
    val = read_validation(wheel)
    val["models"] = {
        name: error_metrics(np.array(val["mean_speed_mm_s"]), values)
        for name, values in predict_models(model, np.array(val["pwm"])).items()
    }
    model["validation"] = val
    print(f"{wheel}: power gamma={model['power_gamma']:.3f}")
    for name, (label, color, linestyle) in MODELS.items():
        print(f"  {label}: training MAE={model['training'][name]['mae_mm_s']:.6f}; "
              f"validation MAE={val['models'][name]['mae_mm_s']:.6f} mm/s")

fig, axes = plt.subplots(2, 2, figsize=(12, 8), layout="constrained", sharex="col")
validation_fig, validation_axes = plt.subplots(1, 2, figsize=(12, 4.3), layout="constrained", sharey=True)
for column, (wheel, model) in enumerate(results.items()):
    pwm = np.array(model["pwm"])
    dense_pwm = np.linspace(pwm[0], pwm[-1], 400)
    axes[0, column].scatter(pwm, model["mean_speed_mm_s"], color="black", label="Training means", zorder=5)
    for name, values in predict_models(model, dense_pwm).items():
        label, color, linestyle = MODELS[name]
        axes[0, column].plot(dense_pwm, values, label=label, color=color, linestyle=linestyle)
        axes[1, column].plot(pwm, model["training"][name]["residual_mm_s"], marker="o",
                             label=label, color=color, linestyle=linestyle)
        validation_axes[column].plot(model["validation"]["pwm"],
            model["validation"]["models"][name]["residual_mm_s"], marker="o",
            label=label, color=color, linestyle=linestyle)
    axes[0, column].set(title=f"{wheel} wheel: response curves", ylabel="Wheel speed (mm/s)")
    axes[1, column].set(title=f"{wheel} wheel: training residuals", xlabel="Requested PWM",
                        ylabel="Measured - predicted (mm/s)", xticks=pwm)
    validation_axes[column].set(title=f"{wheel} wheel: validation residuals", xlabel="Requested PWM",
        ylabel="Measured - predicted (mm/s)", xticks=[50, 110, 170])
    for current_ax in (axes[0, column], axes[1, column], validation_axes[column]):
        current_ax.grid(True, color="#e1e6eb")
        current_ax.set_axisbelow(True)
        current_ax.spines[["top", "right"]].set_visible(False)
        current_ax.legend(fontsize=8)
    axes[1, column].axhline(0, color="black", linewidth=0.8)
    validation_axes[column].axhline(0, color="black", linewidth=0.8)
for figure, name in [(fig, "motor-nonlinearity-2026-11-17"),
                     (validation_fig, "motor-model-validation-2026-11-17")]:
    for suffix in ("png", "svg"):
        figure.savefig(ROOT / "docs/images" / f"{name}.{suffix}", dpi=180)
    plt.close(figure)

report[0] = "# 左右轮 PWM—速度模型比较"
report += [
    "## 非线性拟合方法", "",
    "方法参考 [实验 6 Colab](https://colab.research.google.com/github/semtm0043-42/"
    "semtm0043-42.github.io/blob/main/notebooks/motor-exercise-06-model-comparison.ipynb)。"
    "各轮用 PWM 40～180 的 8 档、24 次试验拟合，用另测的 PWM 50、110、170 共 9 次试验验证。", "",
    "各轮分别固定归一化边界：x=(PWM−40)/140，y=(v−v₄₀)/(v₁₈₀−v₄₀)，"
    "速度边界取训练集端点均速。验证数据不参与参数拟合或边界计算。", "",
    "二次曲线用最小二乘拟合；幂律在 γ=0.5～2.0 的 301 个候选值中选择训练 MAE 最小者。"
    "Colab 的直线参考 y=x 是固定端点连线，另保留当前最小二乘拟合直线作比较。", "",
    "| 轮子 | v₄₀（mm/s） | v₁₈₀（mm/s） | 归一化二次曲线 | 幂律 |",
    "| --- | ---: | ---: | --- | --- |",
]
for wheel, model in results.items():
    c, b, a = model["quadratic_normalized"]
    norm = model["normalization"]
    report.append(f"| {wheel} | {norm['speed_min_mm_s']:.6f} | {norm['speed_max_mm_s']:.6f} "
                  f"| y={a:.8f}{b:+.8f}x{c:+.8f}x² | y=x^{model['power_gamma']:.3f} |")
report += ["", "## 训练与验证误差", "",
    "速度及误差单位为 mm/s。MAE、RMSE 均按每档 PWM 的均速等权计算。"
    "残差=实测−预测；负值表示预测偏高。", "",
    "| 轮子 | 模型 | 训练 MAE | 训练 RMSE | 验证 MAE | 验证 RMSE |",
    "| --- | --- | ---: | ---: | ---: | ---: |"]
for wheel, model in results.items():
    for name, (label, color, linestyle) in MODELS.items():
        train, val = model["training"][name], model["validation"]["models"][name]
        report.append(f"| {wheel} | {label} | {train['mae_mm_s']:.6f} | {train['rmse_mm_s']:.6f} "
                      f"| {val['mae_mm_s']:.6f} | {val['rmse_mm_s']:.6f} |")
report += ["", "![曲线与训练残差](../../docs/images/motor-nonlinearity-2026-11-17.png)", "",
    "上排曲线接近重合；下排放大显示同一批训练均速的残差。", "",
    "## 各验证档位", "",
    "| 轮子 | PWM | 实测均速 | 直线残差 | 二次残差 | 幂律残差 | 固定参考残差 |",
    "| --- | ---: | ---: | ---: | ---: | ---: | ---: |"]
for wheel, model in results.items():
    val = model["validation"]
    for i, pwm in enumerate(val["pwm"]):
        errors = " | ".join(f"{val['models'][name]['residual_mm_s'][i]:+.6f}" for name in MODELS)
        report.append(f"| {wheel} | {pwm:g} | {val['mean_speed_mm_s'][i]:.6f} | {errors} |")
report += ["", "![验证残差](../../docs/images/motor-model-validation-2026-11-17.png)", "",
           "验证日志：", ""]
for wheel, model in results.items():
    links = "、".join(f"[{source}](../../logs/{source})" for source in model["validation"]["sources"])
    report.append(f"- {wheel}：{links}")
report += ["", "## 结论", ""]
for wheel, model in results.items():
    val = model["validation"]["models"]
    linear_mae = val["linear"]["mae_mm_s"]
    report.append(f"- {wheel} 轮：验证 MAE 为直线 {linear_mae:.6f}、"
        f"二次 {val['quadratic']['mae_mm_s']:.6f}、幂律 {val['power']['mae_mm_s']:.6f} mm/s。"
        f"二次相对直线的 MAE 变化为 {val['quadratic']['mae_mm_s'] - linear_mae:+.6f} mm/s，"
        f"幂律为 {val['power']['mae_mm_s'] - linear_mae:+.6f} mm/s（负值表示改善）。")
report += ["",
    "左轮二次曲线的验证 MAE、RMSE 均降低，但 MAE 的绝对改善为约 0.363 mm/s。"
    "右轮二次曲线降低了 RMSE，却提高了 MAE；幂律的右轮验证误差也高于拟合直线。"
    "两轮幂律指数均为 0.995，接近 1。左右轮没有一致的非线性预测改善，当前继续采用各自的拟合直线。", ""]
(OUT / "results.json").write_text(json.dumps(results, indent=2) + "\n", encoding="utf-8")
(OUT / "report.md").write_text("\n".join(report), encoding="utf-8")
