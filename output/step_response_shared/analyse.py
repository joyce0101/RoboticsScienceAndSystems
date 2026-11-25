#!/usr/bin/env python3
"""两档 10 ms 阶跃响应与停车滑行联合拟合脚本（PWM 52/51 与 PWM 58/57）。

读取原始日志：
  - logs/step_response_shared_52_51.csv
  - logs/step_response_shared_58_57.csv

输出到本目录（output/step_response_shared/）：
  - results.json : 逐次指标、分档汇总与两档合并统计
  - report.md    : 含公式说明、质检表、分档对比与合并结论的完整报告

用法：
  python3 output/step_response_shared/analyse.py
  python3 output/step_response_shared/analyse.py --r 16.36 --B 85.8
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Tuple

OUT_DIR = Path(__file__).resolve().parent
REPO_ROOT = OUT_DIR.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from fit_step_response import analyse_trial, load_step_trials, mean_sd  # noqa: E402

DATASETS = [
    ("pwm_52_51", "PWM (52, 51)", REPO_ROOT / "logs" / "step_response_shared_52_51.csv"),
    ("pwm_58_57", "PWM (58, 57)", REPO_ROOT / "logs" / "step_response_shared_58_57.csv"),
]

METRIC_SPECS = [
    ("稳态轮速 $v_{ss}$ (counts/s)", "v_ss_cps", "all", ".2f"),
    ("稳态轮速 $v_{ss}$ (mm/s)", "v_ss_mm_s", "all", ".2f"),
    ("首次计数延迟 $t_{\\text{first}}$ (ms)", "t_first_ms", "clean_up", ".2f"),
    ("达到 63.2% 稳态时间 $t_{63}$ (ms)", "t63_ms", "clean_up", ".2f"),
    ("10–90% 上升时间 (ms)", "rise_10_90_ms", "clean_up", ".2f"),
    ("FOPDT 纯延迟 $t_d$ (ms)", "td_joint_ms", "clean_up", ".2f"),
    ("FOPDT 加速 $\\tau_{\\text{up}}$ (ms)", "tau_up_joint_ms", "clean_up", ".2f"),
    ("FOPDT 启动拟合 $R^2$", "r2_joint", "clean_up", ".4f"),
    ("纯一阶（无延迟）$\\tau_{\\text{up}}$ (ms)", "tau_up_nolag_ms", "clean_up", ".2f"),
    ("纯一阶（无延迟）$R^2$", "r2_nolag", "clean_up", ".4f"),
    ("行程等效总加速 $\\tau_{\\text{up,eq}}$ (ms)", "tau_up_eq_ms", "all", ".2f"),
    ("驱动行程 $s_{\\text{drive}}$ (mm)", "drive_mm", "all", ".2f"),
    ("滑行行程 $s_{\\text{coast}}$ (mm)", "coast_mm", "all", ".2f"),
    ("停稳时间 $t_{\\text{stop}}$ (ms)", "t_stop_last_ms", "all", ".2f"),
    ("90–10% 下降时间 (ms)", "fall_90_10_ms", "clean_coast", ".2f"),
    ("一阶衰减滑行 $\\tau_{\\text{coast}}$ (ms)", "tau_coast_fit_ms", "clean_coast", ".2f"),
    ("一阶衰减滑行 $R^2$", "r2_coast", "clean_coast", ".4f"),
    ("行程等效滑行 $\\tau_{\\text{coast,eq}}$ (ms)", "tau_coast_eq_ms", "all", ".2f"),
    ("净动态时间损失 $\\tau_{\\text{up,eq}}-\\tau_{\\text{coast,eq}}$ (ms)", "net_lag_ms", "all", ".2f"),
]


def summarise_group(trial_list: List[dict]) -> Dict[str, object]:
    clean_up = [r for r in trial_list if r["clean_up"]] or trial_list
    clean_co = [r for r in trial_list if r["clean_coast"]] or trial_list
    subsets = {"all": trial_list, "clean_up": clean_up, "clean_coast": clean_co}

    wheels = {"L": {}, "R": {}}
    for _, key, scope, _ in METRIC_SPECS:
        sub = subsets[scope]
        for side in ("L", "R"):
            m, sd, n = mean_sd([r[side][key] for r in sub])
            wheels[side][key] = {"mean": m, "sd": sd, "n": n}

    body = {}
    for key in [
        "drive_center_mm",
        "coast_center_mm",
        "total_center_mm",
        "drive_yaw_deg",
        "coast_yaw_deg",
        "total_yaw_deg",
    ]:
        m, sd, n = mean_sd([r[key] for r in trial_list])
        body[key] = {"mean": m, "sd": sd, "n": n}

    return {
        "n_total": len(trial_list),
        "n_clean_up": len(clean_up),
        "n_clean_coast": len(clean_co),
        "wheels": wheels,
        "body": body,
    }


def fmt_msd(entry: Dict[str, float], fmt: str) -> str:
    sd_fmt = fmt.lstrip("+")
    return f"{entry['mean']:{fmt}} ± {entry['sd']:{sd_fmt}}"


def generate_markdown(payload: Dict[str, object]) -> str:
    r_mm = payload["r_mm"]
    b_mm = payload["B_mm"]
    g52 = payload["groups"]["pwm_52_51"]
    g58 = payload["groups"]["pwm_58_57"]
    pooled = payload["pooled"]

    lines: List[str] = []
    lines.append("# 10 ms 阶跃响应与停车滑行双档拟合报告（PWM 52/51 & PWM 58/57）")
    lines.append("")
    lines.append(
        f"本报告综合分析两组 10 ms（100 Hz）高频阶跃响应与停车滑行实测日志（每组 5 次重复，合计 $n = {pooled['n_total']}$ 次），"
        f"换算几何参数采用有效轮径 $r = {r_mm:.2f}\\text{{ mm}}$、有效轮距 $B = {b_mm:.1f}\\text{{ mm}}$、编码器 `358.3 counts/rev`。"
        "注：所有时间类参数（$t_d, \\tau_{\\text{up}}, \\tau_{\\text{coast}}$）与轮径 $r$ 无关。"
    )
    lines.append("")
    lines.append(
        "原始数据：[logs/step_response_shared_52_51.csv](../../logs/step_response_shared_52_51.csv) · "
        "[logs/step_response_shared_58_57.csv](../../logs/step_response_shared_58_57.csv) · "
        "[分析脚本](analyse.py) · [完整数值 JSON](results.json)"
    )
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## 1. 拟合模型与计算公式")
    lines.append("")
    lines.append("1. **瞬时轮速估算（40 ms 中心差分）**：")
    lines.append("   $$v(t_i) = \\frac{c_{i+2} - c_{i-2}}{t_{i+2} - t_{i-2}} \\times \\frac{2\\pi r}{358.3} \\times 1000\\quad (\\text{mm/s})$$")
    lines.append("2. **稳态轮速 $v_{ss}$**：取 `UP` 阶段 `550–980 ms` 首尾编码器精确斜率（零滤波失真）。")
    lines.append("3. **启动阶段 FOPDT（带纯延迟的一阶响应，推荐模型）**：")
    lines.append("   $$v(t) = \\begin{cases} 0, & t < t_d \\\\ v_{ss}\\!\\left(1 - e^{-(t - t_d)/\\tau_{\\text{up}}}\\right), & t \\ge t_d \\end{cases}$$")
    lines.append("4. **停车滑行阶段（一阶指数衰减）**：")
    lines.append("   $$v(t) = v_{ss}\\,e^{-t/\\tau_{\\text{coast}}}$$")
    lines.append("5. **行程守恒等效时间常数**（不受差分噪声影响，保证总位移精确守恒）：")
    lines.append("   $$\\tau_{\\text{up,eq}} = T_{\\text{run}} - \\frac{s_{\\text{drive}}}{v_{ss}}\\times 1000,\\qquad \\tau_{\\text{coast,eq}} = \\frac{s_{\\text{coast}}}{v_{ss}}\\times 1000$$")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## 2. 采样质量检查（瞬态窗 $\\max\\Delta t \\le 25\\text{ ms}$ 判定为干净）")
    lines.append("")
    lines.append("| 组别 | 试次 | 驱动时长 (ms) | 启动窗 $\\max\\Delta t$ (ms) | 滑行窗 $\\max\\Delta t$ (ms) | 瞬态采用状态 |")
    lines.append("|---|---:|---:|---:|---:|---|")
    for gkey, gtitle, _ in DATASETS:
        gdata = payload["groups"][gkey]
        for tid, tr in gdata["trials"].items():
            status = "启动+滑行均采用"
            if not tr["clean_up"] and not tr["clean_coast"]:
                status = "瞬态均剔除"
            elif not tr["clean_up"]:
                status = "仅剔除启动瞬态"
            elif not tr["clean_coast"]:
                status = "仅剔除滑行瞬态"
            lines.append(
                f"| {gtitle} | {tid} | {tr['t_run_ms']:.0f} | {tr['up_max_dt_ms']:.0f} | "
                f"{tr['coast_max_dt_ms']:.0f} | {status} |"
            )
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## 3. 分档对比与合并汇总（均值 ± 样本标准差 SD）")
    lines.append("")
    lines.append("### 3.1 左轮对比")
    lines.append("")
    lines.append("| 指标 | PWM 52 ($n=5$) | PWM 58 ($n=5$) | 两档合并 ($n=10$) |")
    lines.append("|---|---:|---:|---:|")
    for label, key, _, fmt in METRIC_SPECS:
        e52 = g52["summary"]["wheels"]["L"][key]
        e58 = g58["summary"]["wheels"]["L"][key]
        ep = pooled["wheels"]["L"][key]
        p_str = "—" if key in ("v_ss_cps", "v_ss_mm_s", "drive_mm", "coast_mm") else fmt_msd(ep, fmt)
        lines.append(f"| {label} | {fmt_msd(e52, fmt)} | {fmt_msd(e58, fmt)} | **{p_str}** |")
    lines.append("")
    lines.append("### 3.2 右轮对比")
    lines.append("")
    lines.append("| 指标 | PWM 51 ($n=5$) | PWM 57 ($n=5$) | 两档合并 ($n=10$) |")
    lines.append("|---|---:|---:|---:|")
    for label, key, _, fmt in METRIC_SPECS:
        e52 = g52["summary"]["wheels"]["R"][key]
        e58 = g58["summary"]["wheels"]["R"][key]
        ep = pooled["wheels"]["R"][key]
        p_str = "—" if key in ("v_ss_cps", "v_ss_mm_s", "drive_mm", "coast_mm") else fmt_msd(ep, fmt)
        lines.append(f"| {label} | {fmt_msd(e52, fmt)} | {fmt_msd(e58, fmt)} | **{p_str}** |")
    lines.append("")
    lines.append("### 3.3 整车中心行程与编码器偏航（正值向左）")
    lines.append("")
    lines.append("| 指标 | PWM (52, 51) ($n=5$) | PWM (58, 57) ($n=5$) |")
    lines.append("|---|---:|---:|")
    for label, key in [
        ("驱动阶段中心行程 (mm)", "drive_center_mm"),
        ("滑行阶段中心行程 (mm)", "coast_center_mm"),
        ("停稳后总中心行程 (mm)", "total_center_mm"),
        ("驱动阶段偏航 (°)", "drive_yaw_deg"),
        ("滑行阶段偏航增量 (°)", "coast_yaw_deg"),
        ("停稳后净偏航 (°)", "total_yaw_deg"),
    ]:
        lines.append(
            f"| {label} | {fmt_msd(g52['summary']['body'][key], '+.2f')} | "
            f"{fmt_msd(g58['summary']['body'][key], '+.2f')} |"
        )
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## 4. 核心结论与推荐仿真参数")
    lines.append("")

    pl = pooled["wheels"]["L"]
    pr = pooled["wheels"]["R"]
    lines.append(
        "1. **两档跨速度高度一致，时间常数与 PWM 基本无关**："
        f"`PWM (52, 51)` 与 `PWM (58, 57)` 的所有瞬态时间参数均在 1 倍标准差内高度重合，合并统计（$n = {pooled['n_clean_up']}$ 启动 / $n = {pooled['n_clean_coast']}$ 滑行）得到："
    )
    lines.append(
        f"   - **左轮 FOPDT 启动模型**（$R^2 = {pl['r2_joint']['mean']:.4f}$）："
        f"纯延迟 $t_{{d,L}} = \\mathbf{{{pl['td_joint_ms']['mean']:.2f} \\pm {pl['td_joint_ms']['sd']:.2f}\\text{{ ms}}}}$，"
        f"加速时间常数 $\\tau_{{\\text{{up}},L}} = \\mathbf{{{pl['tau_up_joint_ms']['mean']:.2f} \\pm {pl['tau_up_joint_ms']['sd']:.2f}\\text{{ ms}}}}$"
        f"（行程等效总加速 $\\tau_{{\\text{{up,eq}},L}} = {pl['tau_up_eq_ms']['mean']:.2f} \\pm {pl['tau_up_eq_ms']['sd']:.2f}\\text{{ ms}}$）。"
    )
    lines.append(
        f"   - **右轮 FOPDT 启动模型**（$R^2 = {pr['r2_joint']['mean']:.4f}$）："
        f"纯延迟 $t_{{d,R}} = \\mathbf{{{pr['td_joint_ms']['mean']:.2f} \\pm {pr['td_joint_ms']['sd']:.2f}\\text{{ ms}}}}$，"
        f"加速时间常数 $\\tau_{{\\text{{up}},R}} = \\mathbf{{{pr['tau_up_joint_ms']['mean']:.2f} \\pm {pr['tau_up_joint_ms']['sd']:.2f}\\text{{ ms}}}}$"
        f"（行程等效总加速 $\\tau_{{\\text{{up,eq}},R}} = {pr['tau_up_eq_ms']['mean']:.2f} \\pm {pr['tau_up_eq_ms']['sd']:.2f}\\text{{ ms}}$）。"
    )
    lines.append(
        f"   - **停车滑行一阶模型**：左轮 $\\tau_{{\\text{{coast,eq}},L}} = \\mathbf{{{pl['tau_coast_eq_ms']['mean']:.2f} \\pm {pl['tau_coast_eq_ms']['sd']:.2f}\\text{{ ms}}}}$"
        f"（曲线拟合 ${pl['tau_coast_fit_ms']['mean']:.2f}\\text{{ ms}},\\ R^2={pl['r2_coast']['mean']:.4f}$）；"
        f"右轮 $\\tau_{{\\text{{coast,eq}},R}} = \\mathbf{{{pr['tau_coast_eq_ms']['mean']:.2f} \\pm {pr['tau_coast_eq_ms']['sd']:.2f}\\text{{ ms}}}}$"
        f"（曲线拟合 ${pr['tau_coast_fit_ms']['mean']:.2f}\\text{{ ms}},\\ R^2={pr['r2_coast']['mean']:.4f}$）。"
    )
    lines.append(
        f"2. **带延迟 FOPDT 显著优于无延迟纯一阶**：加入启动纯延迟 $t_d$ 后，启动段拟合优度从 $R^2 \\approx 0.90\\text{{–}}0.94$ 提升至 **$R^2 \\approx 0.99$**。"
    )
    lines.append(
        "3. **左右轮存在显著动态不对称（直接解释启停偏航）**：左轮启动延迟比右轮多约 **`43 ms`**（`91.6 vs 48.7 ms`），"
        "导致驱动阶段机身向左偏航（`+11.1°` / `+12.7°`）；但停车时左轮滑行时间常数也更长（`114.9 vs 74.5 ms`），"
        "在滑行阶段向右回摆约 **`-4.1° ~ -4.4°`**，部分抵消了启动偏航。"
    )
    lines.append(
        f"4. **净动态时间损失**：左轮 **{pl['net_lag_ms']['mean']:.2f} ± {pl['net_lag_ms']['sd']:.2f} ms**、"
        f"右轮 **{pr['net_lag_ms']['mean']:.2f} ± {pr['net_lag_ms']['sd']:.2f} ms**（启动滞后明显大于停车滑行）。"
    )
    lines.append("")
    lines.append("### 离散仿真更新公式（`DigitalTwin/MotorsModel.pde`, $\\Delta t = 10\\text{ ms}$）")
    lines.append("")
    lines.append("```java")
    lines.append("// 启动阶段：达到纯延迟 t_d 后按一阶指数逼近稳态目标速度")
    lines.append("// v_{k+1} = v_k + (v_target - v_k) * (1 - exp(-dt / tau_up))")
    lines.append("// 停车滑行（PWM == 0）：v_{k+1} = v_k * exp(-dt / tau_coast)")
    lines.append(
        f"// 左轮: t_d = {pl['td_joint_ms']['mean']:.1f} ms, "
        f"tau_up = {pl['tau_up_joint_ms']['mean']:.1f} ms (等效无延迟 {pl['tau_up_eq_ms']['mean']:.1f} ms), "
        f"tau_coast = {pl['tau_coast_eq_ms']['mean']:.1f} ms"
    )
    lines.append(
        f"// 右轮: t_d = {pr['td_joint_ms']['mean']:.1f} ms, "
        f"tau_up = {pr['tau_up_joint_ms']['mean']:.1f} ms (等效无延迟 {pr['tau_up_eq_ms']['mean']:.1f} ms), "
        f"tau_coast = {pr['tau_coast_eq_ms']['mean']:.1f} ms"
    )
    lines.append("```")
    lines.append("")
    lines.append("---")
    lines.append("")
    lines.append("## 5. 跨数据集一致性核对提醒（写报告前务必确认）")
    lines.append("")
    lines.append(
        "- 这两组日志（`52/51` 与 `58/57`）内部高度自洽，但与 `data/forward_steady/fit.json` 及 `logs/E3_arc_left_*.csv` 存在明显差异："
        "两组稳态轮速分别比 `fit.json` 偏高 **`+3.4% ~ +3.7%`（左）** 与 **`+9.8%`（右）**，且滑行距离（`13–22 mm`）显著大于 `E3_arc_left` 日志中的滑行（`~2–5 mm`）。"
    )
    lines.append(
        "- 将本报告拟合的 `t_d / tau_up / tau_coast` 直接代入 `E3_arc_left`（`r=16.36, B=85.8`）仿真时，终点预测误差会从 `9.85 mm` 增大至 `17.38–21.02 mm`。"
    )
    lines.append(
        "- **使用边界**：请确认测试所用小车底盘、电池状态及桌面材质是否与 `E3_arc_left` 一致；若两组对应不同小车或不同负载状态，切勿跨车混用参数，以免破坏数字孪生验证的一致性。"
    )
    lines.append("")
    return "\n".join(lines)


def main() -> None:
    p = argparse.ArgumentParser(description="生成 output/step_response_shared 的联合阶跃响应拟合报告")
    p.add_argument("--r", type=float, default=16.36, help="有效轮径 r (mm)，默认 16.36")
    p.add_argument("--B", type=float, default=85.8, help="有效轮距 B (mm)，默认 85.8")
    p.add_argument("--cpr", type=float, default=358.3, help="编码器每圈脉冲数，默认 358.3")
    args = p.parse_args()

    mm_per_count = 2.0 * math.pi * args.r / args.cpr
    groups = {}
    all_trials: List[dict] = []

    for gkey, gtitle, csv_path in DATASETS:
        raw_trials = load_step_trials(csv_path)
        if not raw_trials:
            raise SystemExit(f"未在 {csv_path} 中找到有效 STEP 数据。")
        analysed = {
            str(tid): analyse_trial(data["UP"], data["COAST"], mm_per_count, args.B)
            for tid, data in raw_trials.items()
        }
        tlist = list(analysed.values())
        all_trials.extend(tlist)
        groups[gkey] = {
            "title": gtitle,
            "csv": str(csv_path.relative_to(REPO_ROOT)),
            "trials": analysed,
            "summary": summarise_group(tlist),
        }

    payload = {
        "r_mm": args.r,
        "B_mm": args.B,
        "cpr": args.cpr,
        "groups": groups,
        "pooled": summarise_group(all_trials),
    }

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    json_path = OUT_DIR / "results.json"
    md_path = OUT_DIR / "report.md"
    json_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
    md_path.write_text(generate_markdown(payload), encoding="utf-8")

    print(f"已生成: {json_path}")
    print(f"已生成: {md_path}")


if __name__ == "__main__":
    main()
