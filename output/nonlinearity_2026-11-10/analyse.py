"""Compare response shapes using original training commands and separate reruns."""
from pathlib import Path
from collections import defaultdict
from statistics import mean, stdev
import hashlib
import json

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
OUT = Path(__file__).resolve().parent
TRAIN = ("device-monitor-261104-234913.log", "device-monitor-261104-235134.log")
EVALUATE = ("device-monitor-261110-010828.log", "device-monitor-261110-011236.log")


def trial_means(names):
    samples = defaultdict(list)
    for name in names:
        for line in (ROOT / "logs" / name).read_text().splitlines():
            row = line.split(",")
            if len(row) == 12 and row[1] == "L" and row[5] == "MEASURE":
                samples[(name, int(row[2]), int(row[3]))].append(float(row[10]))
    assert samples and all(len(values) == 14 for values in samples.values())
    levels = defaultdict(list)
    for (_, pwm, _), values in samples.items():
        levels[pwm].append(mean(values))
    return levels


levels = trial_means(TRAIN)
assert sorted(levels) == list(range(40, 201, 20))
assert sum(map(len, levels.values())) == 30
u = np.array(sorted(levels), dtype=float)
speed = np.array([mean(levels[p]) for p in u])
low = float(speed[0])
span = float(speed[-1] - speed[0])
x = (u - 40) / 160
y = (speed - low) / span
linear = np.polynomial.polynomial.polyfit(x, y, 1)
quadratic = np.polynomial.polynomial.polyfit(x, y, 2)


def power_loss(gamma):
    return float(np.sum((y - x ** gamma) ** 2))


# Minimise squared residuals in the original response scale, not log space.
a, b = 0.05, 5.0
ratio = (5 ** 0.5 - 1) / 2
for _ in range(180):
    c, d = b - ratio * (b - a), a + ratio * (b - a)
    if power_loss(c) < power_loss(d):
        b = d
    else:
        a = c
gamma = (a + b) / 2
assert power_loss(gamma) <= min(power_loss(gamma - 0.0001), power_loss(gamma + 0.0001))


def predict(model, command):
    normal = (np.asarray(command) - 40) / 160
    if model == "linear":
        result = np.polynomial.polynomial.polyval(normal, linear)
    elif model == "quadratic":
        result = np.polynomial.polynomial.polyval(normal, quadratic)
    elif model == "power":
        result = normal ** gamma
    else:
        assert model == "fixed_y=x"
        result = normal
    return low + span * result


models = {}
for model in ("linear", "quadratic", "power", "fixed_y=x"):
    residual = speed - predict(model, u)
    models[model] = dict(
        mae=float(np.mean(abs(residual))),
        rmse=float(np.sqrt(np.mean(residual ** 2))),
        prediction130=float(predict(model, 130)),
        residuals=residual.tolist(),
    )
assert models["quadratic"]["rmse"] <= models["linear"]["rmse"] + 1e-10
assert np.max(abs(predict("linear", u) - (3.722106 * u - 28.861902))) < 0.001

# Conditional, homoscedastic residual-based standard errors; not physical CIs.
design = np.column_stack([np.ones(9), x, x * x])
residual = y - design @ quadratic
variance = float(residual @ residual / 6)
quadratic_se = np.sqrt(np.diag(variance * np.linalg.inv(design.T @ design)))
derivative = np.zeros_like(x)
positive = x > 0
derivative[positive] = x[positive] ** gamma * np.log(x[positive])
gamma_se = float(np.sqrt(power_loss(gamma) / 8 / np.sum(derivative ** 2)))

evaluation = {}
for name in EVALUATE:
    observed = trial_means((name,))
    assert sorted(observed) == [100, 130]
    evaluation[name] = {}
    for pwm, values in sorted(observed.items()):
        evaluation[name][pwm] = dict(
            mean=mean(values), trial_sd=stdev(values),
            errors={model: mean(values) - float(predict(model, pwm)) for model in models},
        )

result = dict(
    boundaries=dict(pwm_min=40, pwm_max=200, speed_min=low, speed_max=low + span),
    linear_normalised=linear.tolist(), quadratic_normalised=quadratic.tolist(),
    quadratic_original=np.polynomial.polynomial.polyfit(u, speed, 2).tolist(),
    gamma=gamma, quadratic_conditional_se=quadratic_se.tolist(),
    gamma_conditional_se=gamma_se, models=models, evaluation=evaluation,
    sha256={name: hashlib.sha256((ROOT / "logs" / name).read_bytes()).hexdigest()
            for name in TRAIN + EVALUATE},
)
(OUT / "results.json").write_text(json.dumps(result, indent=2) + "\n")

lines = [
    "# 实验 6：响应形状比较", "",
    "结论：可以拟合曲线，但当前左轮正向空载 PWM 40～200 数据没有显示出值得增加模型复杂度的曲率。优先保留直线，不能据此推广为整车、右轮、反向或其他条件下没有非线性。", "",
    "## 方法与范围", "",
    "训练仅用 2026-11-04 两份原始日志的 MEASURE：30 次重复、每次 14 条，9 个 PWM 档位。先求重复均值，再求档位均值，每档等权。新日志未进入训练。",
    "归一化边界取已测窗口端点，不宣称 40 是启动阈值或 200 是饱和点。",
    f"x=(PWM−40)/160；y=(speed−{low:.9f})/{span:.9f}。固定这些边界后比较候选；不裁剪观测。",
    "候选均按相同 9 个档位均值最小化速度域平方误差；幂律直接拟合 y=x^gamma，不采用对数变换。", "",
    "## 系数", "",
    f"- 拟合直线（归一化）：y={linear[0]:.9f}+{linear[1]:.9f}x。",
    f"- 二次（归一化）：y={quadratic[0]:.9f}+{quadratic[1]:.9f}x{quadratic[2]:+.9f}x²。",
    f"- 幂律：y=x^{gamma:.9f}。",
    "- 固定参考线 y=x 单独列出，不等同于实验 5 的拟合直线。", "",
    "## 同一训练集上的误差", "",
    "误差单位均为 mm/s，对 9 个档位均值评分；不代表单样本精度。", "",
    "| 模型 | MAE | RMSE | PWM 130 预测 |",
    "| --- | ---: | ---: | ---: |",
]
for model, metrics in models.items():
    lines.append(f"| {model} | {metrics['mae']:.6f} | {metrics['rmse']:.6f} | {metrics['prediction130']:.6f} |")
lines += ["", "## 两轮补测的事后预测比较", "",
          "PWM 130 未参与拟合，但两轮结果已经查看；本比较属于事后分析，不能重新称为所有候选在查看结果前已冻结的前瞻验证。两轮均保留，禁止只选更接近模型的一轮。误差=实测−预测。", "",
          "| 日志 | PWM | 实测均速 | linear 误差 | quadratic 误差 | power 误差 |",
          "| --- | ---: | ---: | ---: | ---: | ---: |"]
for name, conditions in evaluation.items():
    for pwm, row in conditions.items():
        errors = row["errors"]
        lines.append(f"| {name} | {pwm} | {row['mean']:.6f} | {errors['linear']:+.6f} | {errors['quadratic']:+.6f} | {errors['power']:+.6f} |")
lines += ["", "## 不确定性与限制", "",
          f"二次归一化系数 C={quadratic[2]:.9f}，条件残差标准误约 {quadratic_se[2]:.9f}；gamma={gamma:.9f}，条件标准误约 {gamma_se:.9f}。这些是固定边界、档位均值独立同方差近似下的局部参数标准误，不是物理置信保证，也未覆盖批次差异和边界估计不确定性。",
          "二次曲率项相对于其条件标准误很小，gamma 接近 1。二次 RMSE 改善约 0.000044 mm/s；幂律 MAE 略减，但 RMSE 略增。三种拟合模型在 130 的预测跨度不足 0.1 mm/s，远小于补测偏差及两轮变化。",
          "证据支持当前范围保留直线，尚不支持采用曲线获得有用预测改善。不能将批次漂移用曲率强行吸收。只测试两个补测档位也不足以识别曲率。若本次比较用于选模，后续正式预测检验应另预留未用于调整选择的条件。", "",
          "所有原始日志及固件均未修改。results.json 记录原始文件散列、完整残差与未舍入参数。复现需要 Python 和 NumPy，运行本目录 analyse.py。", ""]
(OUT / "comparison.md").write_text("\n".join(lines))
print(json.dumps({"gamma": gamma, "models": models, "report": str(OUT / "comparison.md")}, indent=2))
