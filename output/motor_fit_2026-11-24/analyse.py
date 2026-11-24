"""Convert wheel logs to CSV, fit steady speed, and generate the report and SVG."""

import csv
from collections import defaultdict
from pathlib import Path
from statistics import linear_regression, mean, stdev


OUT = Path(__file__).resolve().parent
LOG_DIR = OUT.parents[1] / "logs"
SOURCES = {
    "L": ("device-monitor-261124-170044.log", "left_wheel_samples.csv"),
    "R": ("device-monitor-261124-170540.log", "right_wheel_samples.csv"),
}
LINEAR_MAX_PWM = {"L": 160, "R": 140}
SAMPLE_COLUMNS = (
    "wheel", "pwm", "trial", "phase", "phase_ms", "speed_mm_s",
    "source_file", "source_line", "use_for_steady_fit",
)


def write_csv(filename, columns, rows):
    with (OUT / filename).open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow(columns)
        writer.writerows(rows)


def read_samples(path, wheel):
    with path.open(newline="", encoding="utf-8") as file:
        for line, row in enumerate(csv.reader(file), start=1):
            if not row:
                continue
            side, pwm, trial, phase, elapsed, speed = row
            if side != wheel or phase not in {"SETTLE", "MEASURE"}:
                raise ValueError(f"{path.name}:{line}: unexpected wheel or phase")
            yield side, int(pwm), int(trial), phase, int(elapsed), speed, path.name, line, int(phase == "MEASURE")


def fit_line(levels):
    pwm = sorted(levels)
    speed = [levels[p] for p in pwm]
    slope, intercept = linear_regression(pwm, speed)
    residuals = [y - (slope * x + intercept) for x, y in zip(pwm, speed)]
    sse = sum(e**2 for e in residuals)
    average = mean(speed)
    r2 = 1 - sse / sum((y - average)**2 for y in speed)
    rmse = (sse / len(speed))**0.5
    return slope, intercept, r2, rmse


def plot_fits(level_rows, fit_rows):
    from reportlab.graphics import renderSVG
    from reportlab.graphics.charts.lineplots import LinePlot
    from reportlab.graphics.shapes import Circle, Drawing, Group, Line, Rect, String
    from reportlab.graphics.widgets.markers import makeMarker
    from reportlab.lib.colors import HexColor, white

    ink, muted, grid = map(HexColor, ("#172635", "#627284", "#e1e7ee"))
    measured, reduced, fitted = map(HexColor, ("#182f45", "#bb631b", "#176eab"))
    drawing = Drawing(860, 408)
    drawing.add(Rect(0, 0, 860, 408, fillColor=white, strokeColor=None))
    drawing.add(String(34, 376, "PWM vs. steady wheel speed", fontName="Helvetica-Bold",
                       fontSize=21, fillColor=ink))
    drawing.add(String(34, 354, "Forward, unloaded | Each point is the mean of three trials",
                       fontSize=11, fillColor=muted))

    for wheel, x, title in (("L", 66, "Left wheel"), ("R", 494, "Right wheel")):
        wheel_levels = [row for row in level_rows if row[0] == wheel]
        panel = LinePlot()
        panel.x, panel.y, panel.width, panel.height = x, 99, 336, 218
        drawing.add(String(x, 328, title, fontName="Helvetica-Bold", fontSize=13, fillColor=ink))

        fits = {row[1]: row for row in fit_rows if row[0] == wheel}
        panel.data = [
            [(p, model[5] * p + model[6]) for p in model[2:4]]
            for model in (fits["full_range"], fits["linear_range"])
        ] + [
            [(row[1], row[3]) for row in wheel_levels if row[7] == flag]
            for flag in (1, 0)
        ]
        panel.lines[0].strokeColor = muted
        panel.lines[0].strokeDashArray = [5, 4]
        panel.lines[0].strokeWidth = 1.3
        panel.lines[1].strokeColor = fitted
        panel.lines[1].strokeWidth = 2.2
        for index, color in ((2, measured), (3, reduced)):
            dot = makeMarker("FilledCircle")
            dot.size, dot.fillColor, dot.strokeColor = 6, color, white
            panel.lines[index].symbol = dot
            panel.lines[index].strokeColor = None

        panel.xValueAxis.valueMin, panel.xValueAxis.valueMax = 34, 186
        panel.xValueAxis.valueSteps = [40, 60, 80, 100, 120, 140, 160, 180]
        panel.yValueAxis.valueMin, panel.yValueAxis.valueMax = 100, 625
        panel.yValueAxis.valueSteps = [100, 200, 300, 400, 500, 600]
        panel.yValueAxis.visibleGrid = True
        panel.yValueAxis.gridStrokeColor = grid
        for axis in (panel.xValueAxis, panel.yValueAxis):
            axis.strokeColor = muted
            axis.labels.fillColor = muted
        drawing.add(panel)
        drawing.add(String(x + 168, 68, "Requested PWM", textAnchor="middle",
                           fontSize=11, fillColor=ink))

    y_label = Group()
    y_label.add(String(0, 0, "Wheel speed (mm/s)", textAnchor="middle",
                       fontSize=11, fillColor=ink))
    y_label.translate(24, 209)
    y_label.rotate(90)
    drawing.add(y_label)
    for x, color, label, style in (
        (67, measured, "Measured in linear range", "point"),
        (261, reduced, "Measured above linear range", "point"),
        (485, fitted, "Linear-range fit", "solid"),
        (635, muted, "Full-range fit", "dashed"),
    ):
        if style == "point":
            drawing.add(Circle(x + 9, 32, 3.2, fillColor=color, strokeColor=None))
        else:
            dash = [5, 4] if style == "dashed" else None
            drawing.add(Line(x, 32, x + 19, 32, strokeColor=color, strokeWidth=2,
                             strokeDashArray=dash))
        drawing.add(String(x + 26, 28, label, fontSize=10, fillColor=ink))

    renderSVG.drawToFile(drawing, str(OUT / "pwm_speed_linear_fit.svg"))


def write_report(level_rows, fit_rows):
    lines = [
        "# 左右轮 PWM—稳态轮速线性拟合",
        "",
        "2026-11-24 架空分轮前进测试。两轮各测 PWM 40–180，每隔 20 一个档位，"
        "每档重复 3 次。只取 `MEASURE`，先按试次求稳态均速，再按 PWM 档求均值，"
        "最后对各档均值做等权最小二乘拟合。",
        "",
        "| 轮子 | 拟合区间 | 公式（mm/s） | R² | RMSE（mm/s） |",
        "| --- | --- | --- | ---: | ---: |",
    ]
    for wheel, scope, low, high, _, slope, intercept, r2, rmse in fit_rows:
        side = "左轮" if wheel == "L" else "右轮"
        label = "线性段" if scope == "linear_range" else "全范围"
        lines.append(f"| {side} | {label} {low}–{high} | "
                     f"$v_{wheel}={slope:.6f}\\,PWM_{wheel}{intercept:+.6f}$ | {r2:.6f} | {rmse:.2f} |")

    speeds = {(row[0], row[1]): row[3] for row in level_rows}
    lines += [
        "",
        "![左右轮 PWM 与稳态轮速散点及拟合直线](pwm_speed_linear_fit.svg)",
        "",
        "蓝线为线性段拟合，灰色虚线为全范围拟合，橙色点是线性段以外的实测均值。",
        "",
        f"左轮从 PWM 160 升到 180 时，轮速增加 {speeds['L', 180] - speeds['L', 160]:.2f} mm/s。"
        f"右轮在同一区间只增加 {speeds['R', 180] - speeds['R', 160]:.2f} mm/s。"
        "本批数据采用左轮 40–160、右轮 40–140 的直线描述稳态关系。",
        "",
        "[分析脚本](analyse.py) · [各档均速与残差](pwm_means.csv) · [拟合参数](linear_fits.csv)",
        "",
    ]
    (OUT / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    trial_rows, level_rows, fit_rows = [], [], []

    for wheel, (source, filename) in SOURCES.items():
        samples = list(read_samples(LOG_DIR / source, wheel))
        write_csv(filename, SAMPLE_COLUMNS, samples)

        trials = defaultdict(list)
        for _, pwm, trial, phase, _, speed, *_ in samples:
            if phase == "MEASURE":
                trials[pwm, trial].append(float(speed))

        trial_means_by_pwm = defaultdict(list)
        for (pwm, trial), speeds in sorted(trials.items()):
            speed = mean(speeds)
            trial_rows.append((wheel, "forward", pwm, trial, speed, len(speeds), source))
            trial_means_by_pwm[pwm].append(speed)

        levels = {pwm: mean(trial_means) for pwm, trial_means in trial_means_by_pwm.items()}
        linear_levels = {pwm: speed for pwm, speed in levels.items() if pwm <= LINEAR_MAX_PWM[wheel]}
        full_fit = fit_line(levels)
        linear_fit = fit_line(linear_levels)
        for scope, inputs, fit in (("full_range", levels, full_fit), ("linear_range", linear_levels, linear_fit)):
            fit_rows.append((wheel, scope, min(inputs), max(inputs), len(inputs), *fit))
            slope, intercept, r2, rmse = fit
            print(f"{wheel} {scope} ({min(inputs)}–{max(inputs)}): "
                  f"v = {slope:.6f} × PWM {intercept:+.6f}; R² = {r2:.8f}, RMSE = {rmse:.4f} mm/s")

        for pwm, speed in sorted(levels.items()):
            observed = trial_means_by_pwm[pwm]
            full_prediction = full_fit[0] * pwm + full_fit[1]
            in_range = pwm in linear_levels
            prediction = linear_fit[0] * pwm + linear_fit[1]
            level_rows.append((
                wheel, pwm, len(observed), speed, stdev(observed),
                full_prediction, speed - full_prediction, int(in_range),
                prediction if in_range else "", speed - prediction if in_range else "",
            ))

    write_csv("settled_trial_means.csv",
              ("wheel", "direction", "PWM", "trial_num", "settled_speed_mm_s", "sample_count", "source_file"),
              trial_rows)
    write_csv("pwm_means.csv",
              ("wheel", "pwm", "n_trials", "mean_speed_mm_s", "trial_sd_mm_s",
               "full_fit_mm_s", "full_residual_mm_s", "use_for_linear_range_fit",
               "linear_range_fit_mm_s", "linear_range_residual_mm_s"),
              level_rows)
    write_csv("linear_fits.csv",
              ("wheel", "fit_scope", "pwm_min", "pwm_max", "n_levels",
               "slope_mm_s_per_pwm", "intercept_mm_s", "r2", "rmse_mm_s"),
              fit_rows)
    plot_fits(level_rows, fit_rows)
    write_report(level_rows, fit_rows)


if __name__ == "__main__":
    main()
