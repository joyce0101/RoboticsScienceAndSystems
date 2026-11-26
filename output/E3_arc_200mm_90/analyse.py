#!/usr/bin/env python3
"""E3 圆弧实验（R = 200 mm, ±90°）全量批量分析与可视化脚本（纯 Python 标准库，无第三方依赖）。

汇总并复现 7 个实验批次（共 35 次独立真机试次）的三方对比（simulation / real_odometry / external_pose）：
  左弧（4 批 × 5 次）：
    1. left_nominal    : B=90.0, r=16.00, PWM=(48, 68), T=1730 ms（第 1 代名义模型）
    2. left_B85.8      : B=85.8, r=16.00, PWM=(48, 68), T=1730 ms（仅更新机载 B）
    3. left_calibrated : B=85.8, r=16.36, PWM=(50, 70), T=1610 ms（第 2 代 E0 几何校准）
    4. left_step_comp  : B=85.8, r=16.36, PWM=(50, 70), T=1660 ms（第 3 代几何+阶跃转角补偿）
  右弧（3 批 × 5 次）：
    5. right_nominal    : B=90.0, r=16.00, PWM=(69, 47), T=1730 ms（第 1 代名义模型）
    6. right_calibrated : B=85.8, r=16.36, PWM=(71, 49), T=1610 ms（第 2 代 E0 几何校准）
    7. right_step_comp  : B=85.8, r=16.36, PWM=(71, 49), T=1690 ms（第 3 代几何+阶跃转角补偿）

输出到本目录（output/E3_arc_200mm_90/）：
  - results.json            : 全部 7 批逐试次指标、均值±SD 及阶跃净滞后预测对照
  - fig_endpoints.svg       : 左右弧三代终点二维散点与理想目标对比图
  - fig_error_evolution.svg : 左右弧三代位置误差与朝向误差演进柱状图

用法（仓库根目录）：
  python3 output/E3_arc_200mm_90/analyse.py
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List

OUT_DIR = Path(__file__).resolve().parent
REPO_ROOT = OUT_DIR.parents[1]
SRC_DIR = REPO_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from analyze_arc_run import (  # noqa: E402
    DEFAULT_FWD_FIT,
    DEFAULT_REV_FIT,
    analyze_single_run,
    compute_pose_error,
    load_model,
)

STEP_RESULTS_PATH = REPO_ROOT / "output" / "step_response_shared" / "results.json"
CPR = 358.3
DT_MS = 10
RADIUS_MM = 200.0

# 配色沿用 output/E2_turn_180 风格
BLUE, ORANGE, AQUA, RED = "#257de0", "#f47230", "#00b894", "#d62728"
TEXT, TEXT2, GRID, BG = "#191919", "#666666", "#e6e6e6", "#ffffff"

BATCHES = [
    {
        "key": "left_nominal",
        "direction": "left",
        "gen": 1,
        "label": "Gen 1: Nominal (B=90, r=16, 1730 ms)",
        "batch_csv": "logs/E3_arc_external.csv",
        "angle_deg": 90.0,
        "pwm_left": 48,
        "pwm_right": 68,
        "duration_ms": 1730,
        "start_pose": (0.0, 0.0, 0.0),
        "r_mm": 16.0,
        "B_mm": 90.0,
    },
    {
        "key": "left_B85.8",
        "direction": "left",
        "gen": 1.5,
        "label": "Gen 1b: B=85.8 only (r=16, 1730 ms)",
        "batch_csv": "logs/E3_arc_left_B85.8_external.csv",
        "angle_deg": 90.0,
        "pwm_left": 48,
        "pwm_right": 68,
        "duration_ms": 1730,
        "start_pose": (0.0, 0.0, 0.0),
        "r_mm": 16.0,
        "B_mm": 85.8,
    },
    {
        "key": "left_calibrated",
        "direction": "left",
        "gen": 2,
        "label": "Gen 2: Calibrated B,r (1610 ms)",
        "batch_csv": "logs/E3_arc_left_B85.8_r16.36_external.csv",
        "angle_deg": 90.0,
        "pwm_left": 50,
        "pwm_right": 70,
        "duration_ms": 1610,
        "start_pose": (0.0, 0.0, 0.0),
        "r_mm": 16.36,
        "B_mm": 85.8,
    },
    {
        "key": "left_step_comp",
        "direction": "left",
        "gen": 3,
        "label": "Gen 3: Calibrated + step comp (1660 ms)",
        "batch_csv": "logs/E3_arc_left_B85.8_r16.36_step_external.csv",
        "angle_deg": 90.0,
        "pwm_left": 50,
        "pwm_right": 70,
        "duration_ms": 1660,
        "start_pose": (0.0, 0.0, 0.0),
        "r_mm": 16.36,
        "B_mm": 85.8,
    },
    {
        "key": "right_nominal",
        "direction": "right",
        "gen": 1,
        "label": "Gen 1: Nominal (B=90, r=16, 1730 ms)",
        "batch_csv": "logs/E3_arc_right_external.csv",
        "angle_deg": -90.0,
        "pwm_left": 69,
        "pwm_right": 47,
        "duration_ms": 1730,
        "start_pose": (0.0, 200.0, 0.0),
        "r_mm": 16.0,
        "B_mm": 90.0,
    },
    {
        "key": "right_calibrated",
        "direction": "right",
        "gen": 2,
        "label": "Gen 2: Calibrated B,r (1610 ms)",
        "batch_csv": "logs/E3_arc_right_B85.8_r16.36_external.csv",
        "angle_deg": -90.0,
        "pwm_left": 71,
        "pwm_right": 49,
        "duration_ms": 1610,
        "start_pose": (0.0, 200.0, 0.0),
        "r_mm": 16.36,
        "B_mm": 85.8,
    },
    {
        "key": "right_step_comp",
        "direction": "right",
        "gen": 3,
        "label": "Gen 3: Calibrated + step comp (1690 ms)",
        "batch_csv": "logs/E3_arc_right_B85.8_r16.36_step_external.csv",
        "angle_deg": -90.0,
        "pwm_left": 71,
        "pwm_right": 49,
        "duration_ms": 1690,
        "start_pose": (0.0, 200.0, 0.0),
        "r_mm": 16.36,
        "B_mm": 85.8,
    },
]


def mean_sd(values: List[float]) -> Dict[str, float]:
    return {
        "mean": statistics.mean(values),
        "sd": statistics.stdev(values) if len(values) > 1 else 0.0,
        "n": len(values),
    }


def compute_lag_compensated_prediction(
    model, pwm_left: int, pwm_right: int, duration_ms: int, angle_deg: float, dL_ms: float, dR_ms: float
) -> Dict[str, float]:
    """扣除实测左右轮净动态时间损失后的等效终点预测。"""
    vL = model.pwm_to_mm_per_sec("left", pwm_left)
    vR = model.pwm_to_mm_per_sec("right", pwm_right)
    sL = vL * (duration_ms - dL_ms) / 1000.0
    sR = vR * (duration_ms - dR_ms) / 1000.0
    s_eff = 0.5 * (sL + sR)
    th_rad = (sR - sL) / model.wheel_separation_mm
    R_eff = abs(s_eff / th_rad) if abs(th_rad) > 1e-9 else float("inf")
    sign = 1.0 if angle_deg >= 0 else -1.0
    x_eff = R_eff * math.sin(abs(th_rad))
    y_eff = sign * R_eff * (1.0 - math.cos(th_rad))
    return {
        "x": x_eff,
        "y": y_eff,
        "theta_deg": math.degrees(th_rad),
        "theta_rad": th_rad,
        "arc_length": s_eff,
        "radius": R_eff,
    }


def analyse_batch(cfg: dict, dL_ms: float, dR_ms: float) -> dict:
    model = load_model(
        DEFAULT_FWD_FIT,
        DEFAULT_REV_FIT,
        r_left_mm=cfg["r_mm"],
        r_right_mm=cfg["r_mm"],
        wheel_separation_mm=cfg["B_mm"],
        counts_per_rev=CPR,
        dt_ms=DT_MS,
    )
    lag_pred = compute_lag_compensated_prediction(
        model, cfg["pwm_left"], cfg["pwm_right"], cfg["duration_ms"], cfg["angle_deg"], dL_ms, dR_ms
    )

    batch_path = REPO_ROOT / cfg["batch_csv"]
    runs = []
    ideal = sim = None
    with batch_path.open("r", encoding="utf-8") as f:
        for row in csv.DictReader(f):
            csv_path = REPO_ROOT / row["csv_path"].strip()
            pf = (float(row["xf"]), float(row["yf"]))
            pb = (float(row["xb"]), float(row["yb"]))
            res = analyze_single_run(
                model=model,
                csv_path=csv_path,
                pf=pf,
                pb=pb,
                radius_mm=RADIUS_MM,
                angle_deg=cfg["angle_deg"],
                left_pwm=cfg["pwm_left"],
                right_pwm=cfg["pwm_right"],
                duration_ms=cfg["duration_ms"],
                start_pose=cfg["start_pose"],
            )
            ideal = res["ideal"]
            sim = res["sim"]
            ext = res["ext"]
            odom = res["odom"]
            err_ext_lag = compute_pose_error(ext, lag_pred)
            runs.append(
                {
                    "run_id": row["run_id"].strip(),
                    "csv_path": row["csv_path"].strip(),
                    "pf": list(pf),
                    "pb": list(pb),
                    "marker_dist_mm": math.hypot(pf[0] - pb[0], pf[1] - pb[1]),
                    "external": ext,
                    "odometry": odom,
                    "ext_minus_sim": res["err_ext_sim"],
                    "ext_minus_odom": res["err_ext_odom"],
                    "odom_minus_sim": res["err_odom_sim"],
                    "ext_minus_ideal": res["err_ext_ideal"],
                    "ext_minus_lag_pred": err_ext_lag,
                    "B_eff_est_mm": res["B_eff_est"],
                    "r_eff_est_mm": res["r_eff_est"],
                }
            )

    summary = {
        "external": {
            "x_mm": mean_sd([r["external"]["x"] for r in runs]),
            "y_mm": mean_sd([r["external"]["y"] for r in runs]),
            "theta_deg": mean_sd([r["external"]["theta_deg"] for r in runs]),
            "arc_length_mm": mean_sd([r["external"]["arc_length"] for r in runs]),
            "radius_mm": mean_sd([r["external"]["radius"] for r in runs]),
            "marker_dist_mm": mean_sd([r["marker_dist_mm"] for r in runs]),
        },
        "odometry": {
            "x_mm": mean_sd([r["odometry"]["x"] for r in runs]),
            "y_mm": mean_sd([r["odometry"]["y"] for r in runs]),
            "theta_deg": mean_sd([r["odometry"]["theta_deg"] for r in runs]),
            "arc_length_mm": mean_sd([r["odometry"]["arc_length"] for r in runs]),
            "radius_mm": mean_sd([r["odometry"]["radius"] for r in runs]),
        },
        "ext_minus_sim": {
            "e_p_mm": mean_sd([r["ext_minus_sim"]["ep"] for r in runs]),
            "e_theta_deg": mean_sd([r["ext_minus_sim"]["e_theta_deg"] for r in runs]),
        },
        "ext_minus_odom": {
            "e_p_mm": mean_sd([r["ext_minus_odom"]["ep"] for r in runs]),
            "e_theta_deg": mean_sd([r["ext_minus_odom"]["e_theta_deg"] for r in runs]),
        },
        "odom_minus_sim": {
            "e_p_mm": mean_sd([r["odom_minus_sim"]["ep"] for r in runs]),
            "e_theta_deg": mean_sd([r["odom_minus_sim"]["e_theta_deg"] for r in runs]),
        },
        "ext_minus_ideal": {
            "e_p_mm": mean_sd([r["ext_minus_ideal"]["ep"] for r in runs]),
            "e_theta_deg": mean_sd([r["ext_minus_ideal"]["e_theta_deg"] for r in runs]),
        },
        "ext_minus_lag_pred": {
            "e_p_mm": mean_sd([r["ext_minus_lag_pred"]["ep"] for r in runs]),
            "e_theta_deg": mean_sd([r["ext_minus_lag_pred"]["e_theta_deg"] for r in runs]),
        },
        "B_eff_est_mm": mean_sd([r["B_eff_est_mm"] for r in runs]),
        "r_eff_est_mm": mean_sd([r["r_eff_est_mm"] for r in runs]),
    }

    return {
        "config": cfg,
        "ideal": ideal,
        "simulation": sim,
        "lag_prediction": lag_pred,
        "runs": runs,
        "summary": summary,
    }


def _svg_marker(shape: str, cx: float, cy: float, r: float, fill: str, stroke: str, sw: float = 1.4, opacity: float = 1.0) -> str:
    if shape == "circle":
        return f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="{r:.1f}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" fill-opacity="{opacity}"/>'
    if shape == "square":
        return (
            f'<rect x="{cx - r:.1f}" y="{cy - r:.1f}" width="{2 * r:.1f}" height="{2 * r:.1f}" '
            f'fill="{fill}" stroke="{stroke}" stroke-width="{sw}" fill-opacity="{opacity}"/>'
        )
    if shape == "triangle":
        pts = f"{cx:.1f},{cy - r * 1.15:.1f} {cx - r * 1.05:.1f},{cy + r * 0.95:.1f} {cx + r * 1.05:.1f},{cy + r * 0.95:.1f}"
        return f'<polygon points="{pts}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}" fill-opacity="{opacity}"/>'
    if shape == "star":
        pts = []
        for k in range(10):
            ang = math.radians(-90 + k * 36)
            rad = r * 1.3 if k % 2 == 0 else r * 0.55
            pts.append(f"{cx + rad * math.cos(ang):.1f},{cy + rad * math.sin(ang):.1f}")
        return f'<polygon points="{" ".join(pts)}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"/>'
    return ""


def write_endpoints_svg(results: Dict[str, dict]) -> None:
    """生成纯 SVG 矢量图：左右弧三代终点二维收敛对比。"""
    W, H = 960, 480
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
        f'<rect width="{W}" height="{H}" fill="{BG}"/>',
        '<style>text{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;}</style>',
        f'<text x="{W / 2:.1f}" y="28" text-anchor="middle" font-size="16" font-weight="700" fill="{TEXT}">'
        'E3 Arc Endpoint Convergence (Solid = External Truth, Hollow = Onboard Odometry)</text>',
    ]
    gen_specs = [
        ("nominal", "Gen 1: Nominal (1730 ms)", BLUE, "circle"),
        ("calibrated", "Gen 2: Calib. B,r (1610 ms)", ORANGE, "square"),
        ("step_comp", "Gen 3: +Step comp (1660/1690 ms)", AQUA, "triangle"),
    ]
    panels = [
        ("left", "Left Arc (+90°, R = 200 mm)", 80, 68, 340, 310, (176.0, 212.0), (180.0, 216.0)),
        ("right", "Right Arc (-90°, R = 200 mm)", 550, 68, 340, 310, (176.0, 212.0), (-216.0, -180.0)),
    ]

    for direction, title, px, py, pw, ph, (xmin, xmax), (ymin, ymax) in panels:
        def mx(x: float) -> float:
            return px + (x - xmin) / (xmax - xmin) * pw

        def my(y: float) -> float:
            return py + ph - (y - ymin) / (ymax - ymin) * ph

        lines.append(f'<text x="{px + pw / 2:.1f}" y="{py - 12}" text-anchor="middle" font-size="13" font-weight="700" fill="{TEXT}">{title}</text>')
        lines.append(f'<rect x="{px}" y="{py}" width="{pw}" height="{ph}" fill="#fcfcfc" stroke="{GRID}" stroke-width="1"/>')

        # 网格与刻度
        for xv in range(180, 211, 5):
            gx = mx(xv)
            lines.append(f'<line x1="{gx:.1f}" y1="{py}" x2="{gx:.1f}" y2="{py + ph}" stroke="{GRID}" stroke-width="0.8"/>')
            lines.append(f'<text x="{gx:.1f}" y="{py + ph + 16}" text-anchor="middle" font-size="10.5" fill="{TEXT2}">{xv}</text>')
        y_ticks = range(180, 216, 5) if direction == "left" else range(-215, -179, 5)
        for yv in y_ticks:
            gy = my(yv)
            lines.append(f'<line x1="{px}" y1="{gy:.1f}" x2="{px + pw}" y2="{gy:.1f}" stroke="{GRID}" stroke-width="0.8"/>')
            lines.append(f'<text x="{px - 8}" y="{gy + 3.5:.1f}" text-anchor="end" font-size="10.5" fill="{TEXT2}">{yv}</text>')

        lines.append(f'<text x="{px + pw / 2:.1f}" y="{py + ph + 36}" text-anchor="middle" font-size="11.5" fill="{TEXT}">Local x (mm)</text>')
        lines.append(
            f'<text x="{px - 48}" y="{py + ph / 2:.1f}" text-anchor="middle" font-size="11.5" fill="{TEXT}" '
            f'transform="rotate(-90 {px - 48} {py + ph / 2:.1f})">Local y (mm)</text>'
        )

        # 理想圆弧参考曲线
        sgn = 1.0 if direction == "left" else -1.0
        arc_pts = []
        for a in range(60, 111):
            ax_v = RADIUS_MM * math.sin(math.radians(a))
            ay_v = sgn * RADIUS_MM * (1.0 - math.cos(math.radians(a)))
            if xmin <= ax_v <= xmax and ymin <= ay_v <= ymax:
                arc_pts.append(f"{mx(ax_v):.1f},{my(ay_v):.1f}")
        lines.append(f'<polyline points="{" ".join(arc_pts)}" fill="none" stroke="#bbbbbb" stroke-width="1.5" stroke-dasharray="5,4"/>')

        # 三代散点、均值与里程计均值
        for suffix, _, color, shape in gen_specs:
            b = results[f"{direction}_{suffix}"]
            for r in b["runs"]:
                lines.append(_svg_marker(shape, mx(r["external"]["x"]), my(r["external"]["y"]), 4.2, color, "none", 0.0, 0.42))
            ox = b["summary"]["odometry"]["x_mm"]["mean"]
            oy = b["summary"]["odometry"]["y_mm"]["mean"]
            lines.append(_svg_marker(shape, mx(ox), my(oy), 6.5, "none", color, 1.9, 1.0))
            ex = b["summary"]["external"]["x_mm"]["mean"]
            ey = b["summary"]["external"]["y_mm"]["mean"]
            lines.append(_svg_marker(shape, mx(ex), my(ey), 6.8, color, "#ffffff", 1.2, 1.0))

        # 理想目标星标
        lines.append(_svg_marker("star", mx(200.0), my(sgn * 200.0), 7.5, RED, "#ffffff", 0.8))

    # 底部图例
    ly = 445
    legend_items = [
        ("star", RED, "#ffffff", "Ideal target (200, ±200)"),
        ("circle", BLUE, "#ffffff", "Gen 1: Nominal (1730 ms)"),
        ("square", ORANGE, "#ffffff", "Gen 2: Calib. B,r (1610 ms)"),
        ("triangle", AQUA, "#ffffff", "Gen 3: +Step comp (1660/1690 ms)"),
    ]
    lx_starts = [70, 270, 480, 700]
    for lx, (shape, fill, stroke, text) in zip(lx_starts, legend_items):
        lines.append(_svg_marker(shape, lx, ly - 3, 5.8, fill, stroke, 1.0))
        lines.append(f'<text x="{lx + 12}" y="{ly + 1}" font-size="11" fill="{TEXT}">{text}</text>')
    lines.append(
        f'<text x="{W / 2:.1f}" y="468" text-anchor="middle" font-size="10.5" fill="{TEXT2}">'
        'Faded markers = individual trials (n=5); Solid markers = external mean; Hollow markers = onboard odometry mean</text>'
    )
    lines.append("</svg>\n")
    (OUT_DIR / "fig_endpoints.svg").write_text("\n".join(lines), encoding="utf-8")


def write_error_evolution_svg(results: Dict[str, dict]) -> None:
    """生成纯 SVG 矢量图：左右弧三代位置误差与转角误差演进柱状图。"""
    W, H = 960, 430
    gens = ["nominal", "calibrated", "step_comp"]
    g_labels = [("Gen 1", "Nominal (1730)"), ("Gen 2", "Calib. B,r (1610)"), ("Gen 3", "+Step (1660/1690)")]
    lines = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" viewBox="0 0 {W} {H}">',
        f'<rect width="{W}" height="{H}" fill="{BG}"/>',
        '<style>text{font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Helvetica,Arial,sans-serif;}</style>',
        f'<text x="{W / 2:.1f}" y="28" text-anchor="middle" font-size="16" font-weight="700" fill="{TEXT}">'
        'E3 Arc Calibration Progression Across 3 Generations (n = 5 per batch, mean ± SD)</text>',
    ]

    # Panel 1: Position error e_p vs Ideal
    px, py, pw, ph = 78, 68, 365, 275
    ymin, ymax = 0.0, 24.0
    lines.append(f'<text x="{px + pw / 2:.1f}" y="{py - 12}" text-anchor="middle" font-size="13" font-weight="700" fill="{TEXT}">Position Error vs Ideal (200, ±200 mm)</text>')
    for yv in range(0, 25, 4):
        gy = py + ph - (yv - ymin) / (ymax - ymin) * ph
        lines.append(f'<line x1="{px}" y1="{gy:.1f}" x2="{px + pw}" y2="{gy:.1f}" stroke="{GRID}" stroke-width="0.9"/>')
        lines.append(f'<text x="{px - 8}" y="{gy + 3.5:.1f}" text-anchor="end" font-size="10.5" fill="{TEXT2}">{yv}</text>')
    lines.append(f'<line x1="{px}" y1="{py + ph}" x2="{px + pw}" y2="{py + ph}" stroke="{TEXT2}" stroke-width="1.1"/>')
    lines.append(
        f'<text x="{px - 44}" y="{py + ph / 2:.1f}" text-anchor="middle" font-size="11.5" fill="{TEXT}" '
        f'transform="rotate(-90 {px - 44} {py + ph / 2:.1f})">Position error e_p (mm)</text>'
    )
    bar_w = 34
    for idx, g in enumerate(gens):
        cx = px + (idx + 0.5) * (pw / 3.0)
        for offset, direction, color in [(-bar_w * 0.56, "left", BLUE), (bar_w * 0.56, "right", ORANGE)]:
            val = results[f"{direction}_{g}"]["summary"]["ext_minus_ideal"]["e_p_mm"]["mean"]
            sd = results[f"{direction}_{g}"]["summary"]["ext_minus_ideal"]["e_p_mm"]["sd"]
            bx = cx + offset - bar_w / 2
            by = py + ph - val / ymax * ph
            bh = val / ymax * ph
            lines.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w}" height="{bh:.1f}" fill="{color}" rx="2"/>')
            # Error bar
            y_hi = py + ph - min(ymax, val + sd) / ymax * ph
            y_lo = py + ph - max(0.0, val - sd) / ymax * ph
            ex = cx + offset
            lines.append(f'<line x1="{ex:.1f}" y1="{y_hi:.1f}" x2="{ex:.1f}" y2="{y_lo:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lines.append(f'<line x1="{ex - 4:.1f}" y1="{y_hi:.1f}" x2="{ex + 4:.1f}" y2="{y_hi:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lines.append(f'<line x1="{ex - 4:.1f}" y1="{y_lo:.1f}" x2="{ex + 4:.1f}" y2="{y_lo:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lines.append(f'<text x="{ex:.1f}" y="{y_hi - 5:.1f}" text-anchor="middle" font-size="10" font-weight="600" fill="{TEXT}">{val:.1f}</text>')
        lines.append(f'<text x="{cx:.1f}" y="{py + ph + 18}" text-anchor="middle" font-size="11" font-weight="600" fill="{TEXT}">{g_labels[idx][0]}</text>')
        lines.append(f'<text x="{cx:.1f}" y="{py + ph + 33}" text-anchor="middle" font-size="10" fill="{TEXT2}">{g_labels[idx][1]}</text>')

    # Panel 2: Signed turn-angle error (+ = Overturn, - = Underturn)
    px2, py2, pw2, ph2 = 540, 68, 365, 275
    ymin2, ymax2 = -6.0, 10.0

    def map_y2(v: float) -> float:
        return py2 + ph2 - (v - ymin2) / (ymax2 - ymin2) * ph2

    lines.append(f'<text x="{px2 + pw2 / 2:.1f}" y="{py2 - 12}" text-anchor="middle" font-size="13" font-weight="700" fill="{TEXT}">Signed Turn-Angle Error (+ Overturn, - Underturn)</text>')
    for yv in range(-6, 11, 2):
        gy = map_y2(yv)
        stroke = TEXT if yv == 0 else GRID
        sw = 1.2 if yv == 0 else 0.9
        lines.append(f'<line x1="{px2}" y1="{gy:.1f}" x2="{px2 + pw2}" y2="{gy:.1f}" stroke="{stroke}" stroke-width="{sw}"/>')
        lines.append(f'<text x="{px2 - 8}" y="{gy + 3.5:.1f}" text-anchor="end" font-size="10.5" fill="{TEXT2}">{yv:+d}°</text>')
    lines.append(
        f'<text x="{px2 - 48}" y="{py2 + ph2 / 2:.1f}" text-anchor="middle" font-size="11.5" fill="{TEXT}" '
        f'transform="rotate(-90 {px2 - 48} {py2 + ph2 / 2:.1f})">Turn-angle error vs 90° (deg)</text>'
    )
    y0 = map_y2(0.0)
    for idx, g in enumerate(gens):
        cx = px2 + (idx + 0.5) * (pw2 / 3.0)
        for offset, direction, color in [(-bar_w * 0.56, "left", BLUE), (bar_w * 0.56, "right", ORANGE)]:
            raw = results[f"{direction}_{g}"]["summary"]["ext_minus_ideal"]["e_theta_deg"]["mean"]
            sd = results[f"{direction}_{g}"]["summary"]["ext_minus_ideal"]["e_theta_deg"]["sd"]
            val = raw if direction == "left" else -raw  # 正值统一表示过转
            bx = cx + offset - bar_w / 2
            yv_px = map_y2(val)
            by = min(y0, yv_px)
            bh = abs(yv_px - y0)
            lines.append(f'<rect x="{bx:.1f}" y="{by:.1f}" width="{bar_w}" height="{bh:.1f}" fill="{color}" rx="2"/>')
            y_hi = map_y2(min(ymax2, val + sd))
            y_lo = map_y2(max(ymin2, val - sd))
            ex = cx + offset
            lines.append(f'<line x1="{ex:.1f}" y1="{y_hi:.1f}" x2="{ex:.1f}" y2="{y_lo:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lines.append(f'<line x1="{ex - 4:.1f}" y1="{y_hi:.1f}" x2="{ex + 4:.1f}" y2="{y_hi:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lines.append(f'<line x1="{ex - 4:.1f}" y1="{y_lo:.1f}" x2="{ex + 4:.1f}" y2="{y_lo:.1f}" stroke="{TEXT}" stroke-width="1.3"/>')
            lbl_y = y_hi - 5 if val >= 0 else y_lo + 12
            lines.append(f'<text x="{ex:.1f}" y="{lbl_y:.1f}" text-anchor="middle" font-size="10" font-weight="600" fill="{TEXT}">{val:+.1f}°</text>')
        lines.append(f'<text x="{cx:.1f}" y="{py2 + ph2 + 18}" text-anchor="middle" font-size="11" font-weight="600" fill="{TEXT}">{g_labels[idx][0]}</text>')
        lines.append(f'<text x="{cx:.1f}" y="{py2 + ph2 + 33}" text-anchor="middle" font-size="10" fill="{TEXT2}">{g_labels[idx][1]}</text>')

    # 共享底部图例
    ly = 406
    lines.append(f'<rect x="355" y="{ly - 10}" width="14" height="12" fill="{BLUE}" rx="2"/>')
    lines.append(f'<text x="375" y="{ly}" font-size="11.5" fill="{TEXT}">Left Arc (+90°)</text>')
    lines.append(f'<rect x="500" y="{ly - 10}" width="14" height="12" fill="{ORANGE}" rx="2"/>')
    lines.append(f'<text x="520" y="{ly}" font-size="11.5" fill="{TEXT}">Right Arc (-90°)</text>')
    lines.append("</svg>\n")
    (OUT_DIR / "fig_error_evolution.svg").write_text("\n".join(lines), encoding="utf-8")


def main() -> None:
    step = json.loads(STEP_RESULTS_PATH.read_text(encoding="utf-8"))
    dL_ms = step["pooled"]["wheels"]["L"]["net_lag_ms"]["mean"]
    dR_ms = step["pooled"]["wheels"]["R"]["net_lag_ms"]["mean"]

    results: Dict[str, dict] = {}
    for cfg in BATCHES:
        res = analyse_batch(cfg, dL_ms, dR_ms)
        results[cfg["key"]] = res
        s = res["summary"]
        print(
            f"[{cfg['key']:16s}] ext=({s['external']['x_mm']['mean']:6.2f}, {s['external']['y_mm']['mean']:+7.2f}, "
            f"{s['external']['theta_deg']['mean']:+6.2f}°) | "
            f"vs ideal: e_p={s['ext_minus_ideal']['e_p_mm']['mean']:5.2f}±{s['ext_minus_ideal']['e_p_mm']['sd']:4.2f} mm, "
            f"e_θ={s['ext_minus_ideal']['e_theta_deg']['mean']:+5.2f}° | "
            f"ext-odom: e_p={s['ext_minus_odom']['e_p_mm']['mean']:4.2f} mm, "
            f"e_θ={s['ext_minus_odom']['e_theta_deg']['mean']:+5.2f}° | "
            f"B_eff={s['B_eff_est_mm']['mean']:.2f} mm"
        )

    payload = {
        "radius_mm": RADIUS_MM,
        "cpr": CPR,
        "dt_ms": DT_MS,
        "pooled_step_net_lag_ms": {"L": dL_ms, "R": dR_ms},
        "batches": results,
    }
    (OUT_DIR / "results.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    write_endpoints_svg(results)
    write_error_evolution_svg(results)
    print(f"\nSaved: {OUT_DIR / 'results.json'}, fig_endpoints.svg, fig_error_evolution.svg")


if __name__ == "__main__":
    main()
