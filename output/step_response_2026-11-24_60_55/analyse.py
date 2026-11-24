"""Apply the previous step-response analysis to the new 60/55 log."""

import json
import sys
from pathlib import Path
from statistics import mean, stdev


OUT = Path(__file__).resolve().parent
OLD = OUT.parent / "step_response_2026-11-24"
LOG = OUT.parents[1] / "logs" / "step_response_60_55.csv"
sys.path.insert(0, str(OLD))
from analyse_step_response import summarise_trials


def stats(values):
    return {"n": len(values), "mean": mean(values), "sd": stdev(values), "values": values}


def add_distances(summary, trials):
    for trial in trials.values():
        for side in ("L", "R"):
            wheel = trial[side]
            wheel["total_dist_mm"] = wheel["up_dist_mm"] + wheel["coast_dist_mm"]
        trial["center_total_mm"] = trial["center_up_mm"] + trial["center_coast_mm"]
        trial["right_minus_left_total_mm"] = trial["R"]["total_dist_mm"] - trial["L"]["total_dist_mm"]
    for side in ("L", "R"):
        summary[side]["total_dist_mm"] = stats([t[side]["total_dist_mm"] for t in trials.values()])
    for key in ("center_total_mm", "right_minus_left_total_mm"):
        summary["vehicle"][key] = stats([t[key] for t in trials.values()])


def show(stat, digits=2, signed=False):
    sign = "+" if signed else ""
    return f"{stat['mean']:{sign}.{digits}f} ± {stat['sd']:.{digits}f}"


def write_report(summary, previous, trials):
    rows = [
        "# 地面阶跃响应 · PWM (60, 55)",
        "",
        "左右轮同时从零升至 PWM (60, 55)，驱动约 1 秒后归零，再记录约 1 秒滑行。"
        "汇总值为逐次指标的均值 ± 样本标准差。",
        "",
        "[原始日志](../../logs/step_response_60_55.csv) · [逐次数据与统计](results.json) · "
        "[分析脚本](analyse.py) · [上一轮 PWM (62,58)](../step_response_2026-11-24/results.json)",
        "",
        "沿用上一轮的换算参数及窗口。轮半径 16 mm、每圈 358.3 计数、轮距 90 mm。"
        "稳态轮速取 550–980 ms 窗口的计数斜率。阈值时间和时间常数采用约 40 ms 的中心差分轮速。",
        "",
        "| 试次 | 启动窗口最大采样间隔（ms） | 停车窗口最大采样间隔（ms） |",
        "| ---: | ---: | ---: |",
    ]
    for trial, values in trials.items():
        rows.append(f"| {trial} | {values['up_rise_max_dt_ms']:.0f} | {values['coast_fall_max_dt_ms']:.0f} |")
    rows += [
        "",
        "最大采样间隔超过 25 ms 的试次从相应瞬态指标中剔除。"
        f"启动用第 {', '.join(map(str, summary['clean_up_trials']))} 次，"
        f"下降时间和减速度用第 {', '.join(map(str, summary['clean_coast_trials']))} 次。"
        "稳态、距离和最后计数时间用全部 5 次。",
        "",
        "| 指标 | 左轮 PWM 60 | 右轮 PWM 55 | n |",
        "| --- | ---: | ---: | ---: |",
    ]
    for label, key, digits in [
        ("首次计数延迟（ms）", "t_lag_first_ms", 1),
        ("稳态轮速（mm/s）", "v_ss_mm_s", 2),
        ("达到 10% 稳态速度（ms）", "t10_ms", 1),
        ("达到 63.2% 稳态速度（ms）", "t63_ms", 1),
        ("达到 90% 稳态速度（ms）", "t90_ms", 1),
        ("10–90% 上升时间（ms）", "rise_10_90_ms", 1),
        ("10–90% 平均加速度（mm/s²）", "slew_up_mm_s2", 1),
        ("一阶拟合时间常数（ms）", "tau_up_ms", 1),
        ("驱动行程（mm）", "up_dist_mm", 2),
        ("滑行行程（mm）", "coast_dist_mm", 2),
        ("停稳后的总行程（mm）", "total_dist_mm", 2),
        ("零 PWM 后最后一次正计数（ms）", "t_stop_ms", 1),
        ("90–10% 下降时间（ms）", "fall_90_10_ms", 1),
        ("90–10% 平均减速度大小（mm/s²）", "slew_down_mm_s2", 1),
    ]:
        left, right = summary["L"][key], summary["R"][key]
        rows.append(f"| {label} | {show(left, digits)} | {show(right, digits)} | {left['n']} |")

    rows += [
        "",
        "## 与 PWM (62, 58) 比较",
        "",
        "行程和朝向来自编码器估算。偏航正值表示向左。中心行程为左右轮行程的平均值。",
        "",
        "| 指标 | 上一轮 (62,58) | 本轮 (60,55) |",
        "| --- | ---: | ---: |",
    ]
    for label, key, signed in [
        ("驱动阶段中心行程（mm）", "center_up_mm", False),
        ("滑行阶段中心行程（mm）", "center_coast_mm", False),
        ("总中心行程（mm）", "center_total_mm", False),
        ("总行程差，右轮减左轮（mm）", "right_minus_left_total_mm", True),
        ("驱动阶段偏航（°）", "yaw_up_deg", True),
        ("滑行阶段偏航增量（°）", "yaw_coast_deg", True),
        ("停稳后净偏航（°）", "yaw_total_deg", True),
    ]:
        rows.append(f"| {label} | {show(previous['vehicle'][key], signed=signed)} | "
                    f"{show(summary['vehicle'][key], signed=signed)} |")
    rows += [
        "",
        "本轮左轮稳态更快，但驱动结束时，右轮仍累计走得更远。"
        "左轮滑行更远，使停稳后的净偏航变为小幅右偏。"
        "终点横向偏移和实际朝向可用坐标纸外部位姿核对。",
        "",
    ]
    (OUT / "report.md").write_text("\n".join(rows), encoding="utf-8")


def main():
    summary, trials, _ = summarise_trials(LOG)
    add_distances(summary, trials)
    previous = json.loads((OLD / "results.json").read_text(encoding="utf-8"))
    add_distances(previous["summary"], previous["trials"])
    result = {"source": LOG.name, "pwm": [60, 55], "summary": summary, "trials": trials}
    (OUT / "results.json").write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    write_report(summary, previous["summary"], trials)


if __name__ == "__main__":
    main()
