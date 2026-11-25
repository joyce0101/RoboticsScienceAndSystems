#!/usr/bin/env python3
"""E2 原地转 180° 三组对比：simulation / real_odometry / external_pose。

三组输入（PWM 固定：左转 -41/40，右转 40/-39）：
  1 名义参数       B=90,   r=16      左 1250 ms，右 1270 ms
  2 只改 B、r      B=85.8, r=16.36   左 1160 ms，右 1180 ms
  3 2 + 阶跃补偿   B=85.8, r=16.36   左 1220 ms，右 1240 ms（+63 ms，见 calc_motion_params.py --step-comp）

数据（仓库根目录 logs/）：
  遥测 E2_turn_<left|right>_180[_<条件>]_run<N>.csv（14 列，50 ms 一行）
  坐标纸 E2_turn_180[_<条件>]_external.csv（run_id,csv_path,xf,yf,xb,yb）

坐标纸读数：车头、车尾轴线在纸面上的投影，起点车头 (0, +48.5)、车尾 (0, -48.5)，
起点朝向 +90°，转角 = atan2(yf - yb, xf - xb) - 90°（左转取正、右转取负）。
里程计：固件输出的 theta（固件 r=16、B=90，三组相同），取 signal==1 的最后一行，和 analyze_arc_run.py 一致。
仿真：fit.json 稳态 counts/s-PWM 拟合，10 ms 欧拉积分，和 DigitalTwin / calc_motion_params.py 一致；
  只有稳态模型，没有起步和停车过程。

用法（仓库根目录）：python3 output/E2_turn_180/analyse.py
"""

import csv
import json
import math
import statistics
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
LOGS = REPO_ROOT / "logs"
CPR = 358.3
DT_MS = 10
PWM = {"left": (-41, 40), "right": (40, -39)}

GROUPS = [
    {"id": 1, "name": "Nominal (B=90, r=16)", "tag": "", "B": 90.0, "r": 16.0,
     "duration": {"left": 1250, "right": 1270}},
    {"id": 2, "name": "Calibrated B, r (B=85.8, r=16.36)", "tag": "_B85.8_r16.36", "B": 85.8, "r": 16.36,
     "duration": {"left": 1160, "right": 1180}},
    {"id": 3, "name": "Calibrated B, r + step comp.", "tag": "_B85.8_r16.36_step", "B": 85.8, "r": 16.36,
     "duration": {"left": 1220, "right": 1240}},
]

# 颜色：图 1 按测量来源，图 2 按组，都用同一套前三个分类色
BLUE, ORANGE, AQUA = "#257de0", "#f47230", "#00b894"
TEXT, TEXT2, GRID = "#191919", "#666666", "#e6e6e6"


def load_fit():
    fwd = json.loads((REPO_ROOT / "data/forward_steady/fit.json").read_text())
    rev = json.loads((REPO_ROOT / "data/reverse_steady/fit.json").read_text())
    return fwd, rev


def counts_per_sec(pwm, wheel, fwd, rev):
    fit = (fwd if pwm > 0 else rev)["fits"][wheel]
    speed = max(0.0, fit["k_per_pwm"] * (abs(pwm) - fit["dead_zone_p0"]))
    return speed if pwm > 0 else -speed


def simulate(pwm_l, pwm_r, duration_ms, r, B, fwd, rev):
    # 10 ms 欧拉积分，返回每步的 (时间 ms, 朝向 deg) 和终点朝向
    mm = 2 * math.pi * r / CPR
    vl = counts_per_sec(pwm_l, "left", fwd, rev) * mm / 1000.0
    vr = counts_per_sec(pwm_r, "right", fwd, rev) * mm / 1000.0
    theta = 0.0
    trace = [(0, 0.0)]
    for k in range(math.ceil(duration_ms / DT_MS)):
        theta += (vr - vl) * DT_MS / B
        trace.append(((k + 1) * DT_MS, math.degrees(theta)))
    return trace, math.degrees(theta)


def read_telemetry(path):
    rows = []
    for parts in csv.reader(open(path, errors="ignore")):
        if len(parts) != 14:
            continue
        try:
            rows.append([float(p) for p in parts])
        except ValueError:
            continue
    started = [r for r in rows if int(r[13]) == 1]
    moving = [r for r in started if r[4] != 0 or r[5] != 0]
    t0 = moving[0][0]
    trace = [(r[0] - t0, math.degrees(r[3])) for r in started]
    return {
        "theta_end_deg": math.degrees(started[-1][3]),
        "still_ms": moving[0][0] - started[0][0],
        "moving_span_ms": moving[-1][0] - moving[0][0],
        "pwm": (moving[0][4], moving[0][5]),
        "trace": trace,
    }


def paper_angle(xf, yf, xb, yb, direction):
    rot = math.degrees(math.atan2(yf - yb, xf - xb)) - 90.0
    if direction == "left" and rot < 0:
        rot += 360.0
    if direction == "right" and rot > 0:
        rot -= 360.0
    if direction == "right" and rot < -300:
        rot += 360.0
    return rot


def mean_sd(values):
    return statistics.mean(values), statistics.stdev(values)


def welch_t(a, b):
    ma, sa = mean_sd(a)
    mb, sb = mean_sd(b)
    return (ma - mb) / math.sqrt(sa ** 2 / len(a) + sb ** 2 / len(b))


def analyse():
    fwd, rev = load_fit()
    results = []
    for g in GROUPS:
        ext = list(csv.DictReader(open(LOGS / f"E2_turn_180{g['tag']}_external.csv")))
        for direction in ("left", "right"):
            pl, pr = PWM[direction]
            duration = g["duration"][direction]
            _, sim_end = simulate(pl, pr, duration, g["r"], g["B"], fwd, rev)
            # 同一输入在里程计坐标（r=16, B=90）下的稳态轨迹，用来和固件里程计比起步过程
            sim_trace_odom, sim_end_odom = simulate(pl, pr, duration, 16.0, 90.0, fwd, rev)
            runs = []
            for row in ext:
                if f"_{direction}_" not in row["run_id"]:
                    continue
                tel = read_telemetry(REPO_ROOT / row["csv_path"])
                xf, yf, xb, yb = (float(row[k]) for k in ("xf", "yf", "xb", "yb"))
                runs.append({
                    "run_id": row["run_id"],
                    "paper_deg": paper_angle(xf, yf, xb, yb, direction),
                    "paper_x": (xf + xb) / 2,
                    "paper_y": (yf + yb) / 2,
                    "mark_spacing_mm": math.hypot(xf - xb, yf - yb),
                    "odom_deg": tel["theta_end_deg"],
                    "still_ms": tel["still_ms"],
                    "moving_span_ms": tel["moving_span_ms"],
                    "pwm": tel["pwm"],
                    "trace": tel["trace"],
                })
            results.append({
                "group": g["id"], "name": g["name"], "direction": direction,
                "B": g["B"], "r": g["r"], "pwm": [pl, pr], "duration_ms": duration,
                "sim_deg": sim_end, "sim_odom_frame_deg": sim_end_odom,
                "sim_trace_odom_frame": sim_trace_odom, "runs": runs,
            })
    return results


def summarise(results):
    table = []
    for res in results:
        paper = [abs(r["paper_deg"]) for r in res["runs"]]
        odom = [abs(r["odom_deg"]) for r in res["runs"]]
        p_m, p_sd = mean_sd(paper)
        o_m, o_sd = mean_sd(odom)
        table.append({
            "group": res["group"], "direction": res["direction"], "duration_ms": res["duration_ms"],
            "n": len(paper),
            "sim_deg": abs(res["sim_deg"]),
            "odom_mean": o_m, "odom_sd": o_sd,
            "paper_mean": p_m, "paper_sd": p_sd,
            "paper_error": p_m - 180.0,
            "paper_over_odom": p_m / o_m,
            "odom_over_sim_odom_frame": o_m / abs(res["sim_odom_frame_deg"]),
            "centre_offset_mm": statistics.mean(math.hypot(r["paper_x"], r["paper_y"]) for r in res["runs"]),
            "mark_spacing_mm": statistics.mean(r["mark_spacing_mm"] for r in res["runs"]),
        })
    tests = []
    for direction in ("left", "right"):
        by = {res["group"]: [abs(r["paper_deg"]) for r in res["runs"]]
              for res in results if res["direction"] == direction}
        for a, b in ((1, 2), (2, 3), (1, 3)):
            tests.append({"direction": direction, "groups": f"{a} vs {b}",
                          "diff_deg": statistics.mean(by[a]) - statistics.mean(by[b]),
                          "welch_t": welch_t(by[a], by[b])})
    return table, tests


def style(ax):
    for side in ("top", "right"):
        ax.spines[side].set_visible(False)
    for side in ("left", "bottom"):
        ax.spines[side].set_color(TEXT2)
    ax.tick_params(colors=TEXT2, labelsize=9)
    ax.grid(axis="y", color=GRID, linewidth=0.8)
    ax.set_axisbelow(True)


def plot_final_angles(results, path):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharey=True)
    sources = [("Simulation (steady model)", "sim", AQUA, "D"),
               ("Odometry (firmware B=90, r=16)", "odom", ORANGE, "o"),
               ("External (paper)", "paper", BLUE, "o")]
    for ax, direction in zip(axes, ("left", "right")):
        style(ax)
        ax.axhline(180, color=TEXT2, linewidth=1, linestyle="--")
        ax.text(1.5, 180.6, "target 180°", color=TEXT2, fontsize=8, ha="center")
        for res in [r for r in results if r["direction"] == direction]:
            x0 = res["group"] - 1
            for k, (label, key, color, marker) in enumerate(sources):
                x = x0 + (k - 1) * 0.22
                if key == "sim":
                    ax.plot(x, abs(res["sim_deg"]), marker=marker, color=color, markersize=8,
                            linestyle="none", label=label if x0 == 0 else None)
                    continue
                vals = [abs(r[f"{key}_deg"]) for r in res["runs"]]
                m, sd = mean_sd(vals)
                ax.scatter([x] * len(vals), vals, s=18, color=color, alpha=0.45, linewidths=0)
                ax.errorbar(x, m, yerr=sd, fmt=marker, color=color, markersize=8, capsize=4,
                            elinewidth=2, label=label if x0 == 0 else None)
                if key == "paper":
                    ax.text(x + 0.09, m, f"{m:.1f}°", color=TEXT, fontsize=8, va="center")
        ax.set_xticks([0, 1, 2])
        ax.set_xticklabels(["1 Nominal", "2 Calib. B, r", "3 + step comp."], color=TEXT, fontsize=9)
        ax.set_title(f"{'Left' if direction == 'left' else 'Right'} turn (|angle|)", color=TEXT, fontsize=11)
        ax.set_xlim(-0.5, 2.6)
    axes[0].set_ylabel("Final rotation (deg)", color=TEXT)
    axes[0].set_ylim(150, 195)
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def plot_heading_traces(results, path):
    fig, axes = plt.subplots(1, 2, figsize=(10, 4.2), sharey=True)
    colors = {1: BLUE, 2: ORANGE, 3: AQUA}
    names = {1: "1 Nominal", 2: "2 Calib. B, r", 3: "3 + step comp."}
    grid = list(range(-200, 2001, 10))
    for ax, direction in zip(axes, ("left", "right")):
        style(ax)
        ax.grid(axis="x", color=GRID, linewidth=0.8)
        for res in [r for r in results if r["direction"] == direction]:
            color = colors[res["group"]]
            curves = []
            for run in res["runs"]:
                ts = [t for t, _ in run["trace"]]
                ths = [abs(th) for _, th in run["trace"]]
                curves.append([interp(ts, ths, t) for t in grid])
            mean_curve = [statistics.mean(c[i] for c in curves) for i in range(len(grid))]
            ax.plot(grid, mean_curve, color=color, linewidth=2, label=f"{names[res['group']]} odometry")
            st = [t for t, _ in res["sim_trace_odom_frame"]] + [2000]
            sv = [abs(v) for _, v in res["sim_trace_odom_frame"]]
            sv.append(sv[-1])
            ax.plot(st, sv, color=color, linewidth=1.2, linestyle="--")
            ax.axvline(res["duration_ms"], color=color, linewidth=0.8, linestyle=":")
        ax.set_title(f"{'Left' if direction == 'left' else 'Right'} turn: odometry vs steady model",
                     color=TEXT, fontsize=11)
        ax.set_xlabel("Time from first non-zero PWM (ms)", color=TEXT)
        ax.set_xlim(-200, 2000)
    axes[0].set_ylabel("|Heading| in odometry frame (deg)", color=TEXT)
    axes[0].text(30, 175, "dashed: steady model, same input\ndotted: PWM set to 0",
                 color=TEXT2, fontsize=8, va="top")
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc="upper center", ncol=3, frameon=False, fontsize=9)
    fig.tight_layout(rect=(0, 0, 1, 0.92))
    fig.savefig(path, dpi=200)
    plt.close(fig)


def interp(ts, vs, t):
    if t <= ts[0]:
        return 0.0
    if t >= ts[-1]:
        return vs[-1]
    for i in range(1, len(ts)):
        if ts[i] >= t:
            a = (t - ts[i - 1]) / (ts[i] - ts[i - 1])
            return vs[i - 1] + a * (vs[i] - vs[i - 1])
    return vs[-1]


def write_report(table, tests, results):
    lines = [
        "# E2 原地转 180° 三组对比（simulation / real_odometry / external_pose）",
        "",
        "PWM 固定（左转 −41/40，右转 40/−39），只改时长。每组每方向 5 次，先左 1–5 再右 1–5。"
        "原始数据见 `logs/E2_turn_*_180*_run*.csv` 与 `logs/E2_turn_180*_external.csv`，"
        "分析脚本 [analyse.py](analyse.py)，逐次数值 [results.json](results.json)。",
        "",
        "| 组 | 参数 | 左转时长 | 右转时长 |",
        "|---|---|---:|---:|",
        "| 1 | B=90, r=16（名义） | 1250 ms | 1270 ms |",
        "| 2 | B=85.8, r=16.36（只改 B、r） | 1160 ms | 1180 ms |",
        "| 3 | 2 + 阶跃补偿 +63 ms（`calc_motion_params.py --step-comp`） | 1220 ms | 1240 ms |",
        "",
        "## 1. 终点转角（绝对值，均值 ± 样本标准差，n = 5）",
        "",
        "| 组 | 方向 | 仿真 (°) | 里程计 (°) | 坐标纸 (°) | 坐标纸 − 180 (°) | 坐标纸 / 里程计 | 里程计 / 稳态模型 | 中心偏移 (mm) |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in table:
        lines.append(
            f"| {row['group']} | {'左' if row['direction'] == 'left' else '右'} | {row['sim_deg']:.1f} | "
            f"{row['odom_mean']:.1f} ± {row['odom_sd']:.1f} | {row['paper_mean']:.1f} ± {row['paper_sd']:.1f} | "
            f"{row['paper_error']:+.1f} | {row['paper_over_odom']:.3f} | {row['odom_over_sim_odom_frame']:.3f} | "
            f"{row['centre_offset_mm']:.1f} |")
    lines += [
        "",
        "- 仿真：fit.json 稳态模型，10 ms 欧拉积分，用本组的 B、r；没有起步和停车过程。",
        "- 里程计：固件输出（r=16、B=90，三组相同）。",
        "- 里程计 / 稳态模型：同一输入在里程计坐标（r=16、B=90）下，实测与稳态模型之比，反映起停过程少转的比例。",
        "- 第 3 组仿真约 189°：时长补了 63 ms，而仿真没有起停过程，稳态模型按全速多算了这 63 ms，这部分正是实车起停时少转的量。",
        "",
        "## 2. 组间差异（坐标纸）",
        "",
        "| 方向 | 比较 | 差 (°) | Welch t |",
        "|---|---|---:|---:|",
    ]
    for t in tests:
        lines.append(f"| {'左' if t['direction'] == 'left' else '右'} | {t['groups']} | "
                     f"{t['diff_deg']:+.1f} | {t['welch_t']:+.1f} |")
    lines += [
        "",
        "## 3. 图",
        "",
        "![终点转角](fig_final_angles.png)",
        "",
        "![里程计朝向随时间](fig_heading_traces.png)",
        "",
        "## 4. 跑前预测与结果",
        "",
        "| 组 | 跑前预测（坐标纸） | 实测 左 / 右 |",
        "|---|---|---|",
    ]
    by = {(r["group"], r["direction"]): r for r in table}
    lines.append(f"| 2 | 约 170° | {by[(2, 'left')]['paper_mean']:.1f}° / {by[(2, 'right')]['paper_mean']:.1f}° |")
    lines.append(f"| 3 | 178–180° | {by[(3, 'left')]['paper_mean']:.1f}° / {by[(3, 'right')]['paper_mean']:.1f}° |")
    lines += [
        "",
        "## 5. 局限",
        "",
        "- 阶跃响应在 PWM 51–58 直线上测得，原地转 PWM 39–41 属外推。",
        "- 试验顺序为每组先左后右，没有左右交替；每次实验前电量 90%–100%。",
        "- 坐标纸读数为车头、车尾轴线投影，起点间距 97 mm；终点间距见 results.json。",
        "",
    ]
    (OUT_DIR / "report.md").write_text("\n".join(lines), encoding="utf-8")


def main():
    results = analyse()
    table, tests = summarise(results)
    plot_final_angles(results, OUT_DIR / "fig_final_angles.png")
    plot_heading_traces(results, OUT_DIR / "fig_heading_traces.png")
    slim = []
    for res in results:
        item = {k: v for k, v in res.items() if k != "sim_trace_odom_frame"}
        item["runs"] = [{k: v for k, v in run.items() if k != "trace"} for run in res["runs"]]
        slim.append(item)
    (OUT_DIR / "results.json").write_text(
        json.dumps({"summary": table, "tests": tests, "groups": slim}, indent=2, ensure_ascii=False),
        encoding="utf-8")
    write_report(table, tests, results)
    for row in table:
        print(row["group"], row["direction"], f"sim {row['sim_deg']:.1f}",
              f"odom {row['odom_mean']:.1f}±{row['odom_sd']:.1f}",
              f"paper {row['paper_mean']:.1f}±{row['paper_sd']:.1f}",
              f"p/o {row['paper_over_odom']:.3f}", f"o/s {row['odom_over_sim_odom_frame']:.3f}")
    for t in tests:
        print(t)


if __name__ == "__main__":
    main()
