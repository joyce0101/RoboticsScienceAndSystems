#!/usr/bin/env python3
"""圆弧单动作实验三方对比分析脚本（simulation vs real_odometry vs external_pose）。

功能：
  1. 读取 data/forward_steady/fit.json 与 data/reverse_steady/fit.json，按 10 ms 离散
     欧拉积分计算当前模型下的仿真结果（simulation）。
  2. 读取真机遥测 CSV（14 列：timestamp_ms, x, y, theta, pwmL, pwmR, encL, encR,
     DN1..DN5, signal），提取机载编码器里程计终点、弧长、等效半径与稳态轮速（real_odometry）。
  3. 根据坐标纸前后标记点 P_f(x_f, y_f)、P_b(x_b, y_b) 计算地面真实位姿、等效圆弧半径
     与等效弧长（external_pose）。
  4. 输出三方对比总表、三组误差拆解（odom-sim, ext-odom, ext-sim）、沿程/横向误差分解，
     以及单次反推的有效轮距 B_eff / 有效轮径 r_eff 参考值。
  5. 支持单次运行分析，也支持 --batch CSV 批量统计 5 次运行的均值 ± 样本标准差（SD）。

用法示例：
  # 单次分析（左弧 R=200mm, +90°, 默认起点 (0,0,0)）
  python3 src/analyze_arc_run.py \\
      --csv logs/E3_arc_left_run1.csv \\
      --pf 180,252 --pb 190,157

  # 单次分析（右弧 R=200mm, -90°, 起点 (0,200,0)）
  python3 src/analyze_arc_run.py \\
      --csv logs/E3_arc_right_run1.csv \\
      --pf 195,-45 --pb 205,50 \\
      --angle -90 --pwm-left 69 --pwm-right 47 --duration-ms 1730 --start 0,200,0

  # 批量分析（CSV 表头：run_id,csv_path,xf,yf,xb,yb）
  python3 src/analyze_arc_run.py --batch logs/E3_arc_left_external.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from calc_motion_params import (  # noqa: E402
    DEFAULT_FWD_FIT,
    DEFAULT_REV_FIT,
    RobotModel,
    load_model,
)


def wrap_rad(angle_rad: float) -> float:
    """将角度折回 (-pi, pi]：wrap(a) = atan2(sin(a), cos(a))。"""
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def wrap_deg(angle_deg: float) -> float:
    """将角度折回 (-180°, 180°]。"""
    return math.degrees(wrap_rad(math.radians(angle_deg)))


def parse_xy(text: str) -> Tuple[float, float]:
    parts = [p.strip() for p in text.replace("，", ",").split(",")]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError(f"坐标格式应为 x,y，收到: {text}")
    return float(parts[0]), float(parts[1])


def parse_pose(text: str) -> Tuple[float, float, float]:
    parts = [p.strip() for p in text.replace("，", ",").split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"位姿格式应为 x,y,theta_deg，收到: {text}")
    return float(parts[0]), float(parts[1]), float(parts[2])


def to_local_pose(
    x: float, y: float, theta_rad: float, start_pose: Tuple[float, float, float]
) -> Tuple[float, float, float]:
    """将世界系位姿转换到起点局部坐标系（起点置为 (0, 0, 0)）。

    公式：
      dx = x - x0, dy = y - y0
      x_loc =  cos(th0) * dx + sin(th0) * dy
      y_loc = -sin(th0) * dx + cos(th0) * dy
      th_loc = wrap(theta - th0)
    """
    x0, y0, th0_deg = start_pose
    th0 = math.radians(th0_deg)
    dx, dy = x - x0, y - y0
    c, s = math.cos(th0), math.sin(th0)
    return c * dx + s * dy, -s * dx + c * dy, wrap_rad(theta_rad - th0)


def compute_ideal_arc(radius_mm: float, angle_deg: float) -> Dict[str, float]:
    """计算理想圆弧终点、弧长与半径（起点局部系 (0, 0, 0)）。

    公式：
      sgn = +1 (左弧, angle > 0) 或 -1 (右弧, angle < 0)
      x_ideal = R * sin(|angle|)
      y_ideal = sgn * R * (1 - cos(|angle|))
      s_ideal = R * |angle_rad|
    """
    angle_rad = math.radians(angle_deg)
    sign = 1.0 if angle_deg >= 0 else -1.0
    x = radius_mm * math.sin(abs(angle_rad))
    y = sign * radius_mm * (1.0 - math.cos(abs(angle_rad)))
    s = radius_mm * abs(angle_rad)
    return {
        "x": x,
        "y": y,
        "theta_deg": angle_deg,
        "theta_rad": angle_rad,
        "arc_length": s,
        "radius": radius_mm,
    }


def compute_simulation_arc(
    model: RobotModel, left_pwm: int, right_pwm: int, duration_ms: int
) -> Dict[str, float]:
    """按 DigitalTwin 的 10 ms 离散欧拉积分计算仿真终点、弧长与等效半径。

    公式（每步 dt = 10 ms）：
      |counts/s| = max(0, k * (|PWM| - p0))
      v_L = (counts/s)_L * (2*pi*r_L / CPR),  v_R = (counts/s)_R * (2*pi*r_R / CPR)
      dL = v_L * dt,  dR = v_R * dt
      d  = (dL + dR) / 2,   dtheta = (dR - dL) / B
      x_{k+1} = x_k + d * cos(theta_k)
      y_{k+1} = y_k + d * sin(theta_k)
      theta_{k+1} = theta_k + dtheta
      s_sim = sum(d) = (dL_total + dR_total) / 2
      R_sim = s_sim / |theta_sim|
    """
    x = 0.0
    y = 0.0
    theta = 0.0
    vl_mm_ms = model.pwm_to_mm_per_sec("left", left_pwm) / 1000.0
    vr_mm_ms = model.pwm_to_mm_per_sec("right", right_pwm) / 1000.0
    steps = math.ceil(duration_ms / model.dt_ms)
    total_dL = 0.0
    total_dR = 0.0
    for _ in range(steps):
        dl = vl_mm_ms * model.dt_ms
        dr = vr_mm_ms * model.dt_ms
        total_dL += dl
        total_dR += dr
        d = 0.5 * (dl + dr)
        dtheta = (dr - dl) / model.wheel_separation_mm
        x += d * math.cos(theta)
        y += d * math.sin(theta)
        theta += dtheta

    arc_length = 0.5 * (total_dL + total_dR)
    radius = abs(arc_length / theta) if abs(theta) > 1e-9 else float("inf")
    return {
        "x": x,
        "y": y,
        "theta_deg": math.degrees(theta),
        "theta_rad": theta,
        "arc_length": arc_length,
        "radius": radius,
        "dL": total_dL,
        "dR": total_dR,
        "active_ms": steps * model.dt_ms,
    }


def compute_odometry_from_csv(
    csv_path: Path, model: RobotModel
) -> Dict[str, float]:
    """解析真机 14 列遥测 CSV，计算 real_odometry 终点、弧长、等效半径与稳态轮速。

    14 列顺序（Controller.cpp publishTelemetry）：
      0:timestamp_ms, 1:x, 2:y, 3:theta(rad), 4:pwmL, 5:pwmR,
      6:encL, 7:encR, 8..12:DN1..DN5, 13:signal

    公式：
      dL = (encL_end - encL_0) * (2*pi*r_L / CPR)
      dR = (encR_end - encR_0) * (2*pi*r_R / CPR)
      s_odom = (dL + dR) / 2
      R_odom = s_odom / |theta_odom|
      稳态编码器速率：v_cps = (enc(t2) - enc(t1)) / ((t2 - t1) / 1000)
    """
    rows: List[List[float]] = []
    with csv_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split(",")]
            if len(parts) != 14:
                continue
            try:
                rows.append([float(p) for p in parts])
            except ValueError:
                continue

    sig1 = [r for r in rows if int(r[13]) == 1]
    if not sig1:
        raise ValueError(f"{csv_path} 中未找到 signal == 1 的有效遥测行。")

    t0 = sig1[0][0]
    encL0, encR0 = sig1[0][6], sig1[0][7]
    final = sig1[-1]

    moving = [r for r in sig1 if r[4] != 0 or r[5] != 0]
    if not moving:
        raise ValueError(f"{csv_path} 中未找到非零 PWM 运动帧。")

    pwmL_cmd = int(round(moving[0][4]))
    pwmR_cmd = int(round(moving[0][5]))
    t_move_start = moving[0][0]
    t_move_end = moving[-1][0]

    dL_final = (final[6] - encL0) * model.mm_per_count_left
    dR_final = (final[7] - encR0) * model.mm_per_count_right
    s_odom = 0.5 * (dL_final + dR_final)
    theta_odom = final[3]
    r_odom = abs(s_odom / theta_odom) if abs(theta_odom) > 1e-9 else float("inf")

    # 稳态段轮速：跳过前 400 ms 启动爬坡与末 200 ms，计算平均编码器转速（counts/s）
    move_span = t_move_end - t_move_start
    steady_lo = 400.0
    steady_hi = max(steady_lo + 200.0, move_span - 200.0)
    mid = [r for r in moving if steady_lo <= (r[0] - t_move_start) <= steady_hi]
    if len(mid) >= 2 and (mid[-1][0] - mid[0][0]) > 0:
        dt_s = (mid[-1][0] - mid[0][0]) / 1000.0
        vL_steady_cps = (mid[-1][6] - mid[0][6]) / dt_s
        vR_steady_cps = (mid[-1][7] - mid[0][7]) / dt_s
    else:
        vL_steady_cps = float("nan")
        vR_steady_cps = float("nan")

    # 记录最后一条非零 PWM 帧到最终静止的增量（含末帧未满周期与滑行）
    end_move = moving[-1]
    dL_move = (end_move[6] - encL0) * model.mm_per_count_left
    dR_move = (end_move[7] - encR0) * model.mm_per_count_right

    return {
        "x": final[1],
        "y": final[2],
        "theta_deg": math.degrees(theta_odom),
        "theta_rad": theta_odom,
        "arc_length": s_odom,
        "radius": r_odom,
        "dL": dL_final,
        "dR": dR_final,
        "dL_after_last_pwm": dL_final - dL_move,
        "dR_after_last_pwm": dR_final - dR_move,
        "pwm_left_cmd": pwmL_cmd,
        "pwm_right_cmd": pwmR_cmd,
        "t_move_start_rel_ms": t_move_start - t0,
        "t_move_end_rel_ms": t_move_end - t0,
        "vL_steady_cps": vL_steady_cps,
        "vR_steady_cps": vR_steady_cps,
        "vL_fit_cps": model.pwm_to_counts_per_sec("left", pwmL_cmd),
        "vR_fit_cps": model.pwm_to_counts_per_sec("right", pwmR_cmd),
        "n_rows": len(sig1),
    }


def compute_external_pose(
    pf: Tuple[float, float],
    pb: Tuple[float, float],
    start_pose: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> Dict[str, float]:
    """根据坐标纸车头 P_f(x_f, y_f) 与车尾 P_b(x_b, y_b) 计算外部真实位姿与等效圆弧。

    公式：
      1) 轮轴中心位姿（世界系）：
           x_ext = (x_f + x_b) / 2
           y_ext = (y_f + y_b) / 2
           theta_ext = atan2(y_f - y_b, x_f - x_b)
      2) 前后标记间距（用于核对读数自洽性）：
           L_fb = sqrt((x_f - x_b)^2 + (y_f - y_b)^2)
      3) 转换到起点局部系 (x_loc, y_loc, theta_loc) 后，拟合从 (0, 0, 0) 出发经 (x_loc, y_loc)
         的等曲率圆弧（x = R*sin(phi), |y| = R*(1 - cos(phi)) => x^2 + y^2 = 2*R*|y|）：
           R_ext   = (x_loc^2 + y_loc^2) / (2 * |y_loc|)
           phi_ext = atan2(x_loc, R_ext - |y_loc|)
           s_ext   = R_ext * phi_ext
    """
    xf, yf = pf
    xb, yb = pb
    x_world = 0.5 * (xf + xb)
    y_world = 0.5 * (yf + yb)
    theta_world = math.atan2(yf - yb, xf - xb)
    marker_dist = math.hypot(xf - xb, yf - yb)

    x_loc, y_loc, theta_loc = to_local_pose(x_world, y_world, theta_world, start_pose)
    if abs(y_loc) > 1e-6:
        r_pos = (x_loc**2 + y_loc**2) / (2.0 * abs(y_loc))
        phi_pos = math.atan2(x_loc, r_pos - abs(y_loc))
        s_pos = r_pos * abs(phi_pos)
    else:
        r_pos = float("inf")
        phi_pos = 0.0
        s_pos = abs(x_loc)

    return {
        "x_world": x_world,
        "y_world": y_world,
        "theta_world_deg": math.degrees(theta_world),
        "x": x_loc,
        "y": y_loc,
        "theta_deg": math.degrees(theta_loc),
        "theta_rad": theta_loc,
        "arc_length": s_pos,
        "radius": r_pos,
        "phi_pos_deg": math.degrees(phi_pos),
        "marker_dist": marker_dist,
    }


def compute_pose_error(
    target: Dict[str, float], ref: Dict[str, float]
) -> Dict[str, float]:
    """计算 target - ref 的位置误差、朝向误差及沿程/横向正交分解（设计手册 7.2 节）。

    公式：
      dx = x_target - x_ref,   dy = y_target - y_ref
      e_p     = sqrt(dx^2 + dy^2)
      e_theta = atan2(sin(theta_target - theta_ref), cos(theta_target - theta_ref))
      e_along =  cos(theta_ref) * dx + sin(theta_ref) * dy   （正值表示沿参考前进方向超前）
      e_cross = -sin(theta_ref) * dx + cos(theta_ref) * dy   （正值表示偏向参考前进方向左侧）
    """
    dx = target["x"] - ref["x"]
    dy = target["y"] - ref["y"]
    ep = math.hypot(dx, dy)
    eth_rad = wrap_rad(target["theta_rad"] - ref["theta_rad"])
    th_ref = ref["theta_rad"]
    e_along = math.cos(th_ref) * dx + math.sin(th_ref) * dy
    e_cross = -math.sin(th_ref) * dx + math.cos(th_ref) * dy
    return {
        "dx": dx,
        "dy": dy,
        "ep": ep,
        "e_theta_deg": math.degrees(eth_rad),
        "e_along": e_along,
        "e_cross": e_cross,
    }


def analyze_single_run(
    model: RobotModel,
    csv_path: Optional[Path],
    pf: Optional[Tuple[float, float]],
    pb: Optional[Tuple[float, float]],
    radius_mm: float,
    angle_deg: float,
    left_pwm: int,
    right_pwm: int,
    duration_ms: int,
    start_pose: Tuple[float, float, float],
) -> Dict[str, object]:
    ideal = compute_ideal_arc(radius_mm, angle_deg)
    sim = compute_simulation_arc(model, left_pwm, right_pwm, duration_ms)
    odom = compute_odometry_from_csv(csv_path, model) if csv_path else None
    ext = compute_external_pose(pf, pb, start_pose) if (pf and pb) else None

    result: Dict[str, object] = {
        "ideal": ideal,
        "sim": sim,
        "odom": odom,
        "ext": ext,
        "err_odom_sim": compute_pose_error(odom, sim) if odom else None,
        "err_ext_sim": compute_pose_error(ext, sim) if ext else None,
        "err_ext_odom": compute_pose_error(ext, odom) if (ext and odom) else None,
        "err_ext_ideal": compute_pose_error(ext, ideal) if ext else None,
    }

    # 单次反推等效几何参数（假设无打滑时的参考值）：
    #   B_eff_est = B * (theta_odom / theta_ext)
    #   r_eff_est = r * (s_ext / s_odom)
    if odom and ext and abs(ext["theta_rad"]) > 1e-6 and abs(odom["arc_length"]) > 1e-6:
        r_nom = 0.5 * (model.r_left_mm + model.r_right_mm)
        result["B_eff_est"] = model.wheel_separation_mm * (odom["theta_rad"] / ext["theta_rad"])
        result["r_eff_est"] = r_nom * (ext["arc_length"] / odom["arc_length"])
    else:
        result["B_eff_est"] = None
        result["r_eff_est"] = None
    return result


def print_single_report(
    res: Dict[str, object],
    left_pwm: int,
    right_pwm: int,
    duration_ms: int,
    title: str = "单次圆弧实验三方对比分析",
) -> None:
    ideal = res["ideal"]
    sim = res["sim"]
    odom = res["odom"]
    ext = res["ext"]

    print(f"=== {title} (PWM=({left_pwm}, {right_pwm}), T={duration_ms} ms) ===")
    header = (
        f"{'数据来源':<26} | {'x (mm)':>8} | {'y (mm)':>8} | "
        f"{'朝向 θ':>9} | {'等效弧长 s (mm)':>15} | {'等效圆弧半径 R (mm)':>19}"
    )
    print(header)
    print("-" * len(header))

    def fmt_row(label: str, d: Dict[str, float]) -> str:
        return (
            f"{label:<26} | {d['x']:8.2f} | {d['y']:8.2f} | "
            f"{d['theta_deg']:+8.2f}° | {d['arc_length']:15.2f} | {d['radius']:19.2f}"
        )

    print(fmt_row("理想目标", ideal))
    print(fmt_row("simulation（当前仿真）", sim))
    if odom:
        print(fmt_row("real_odometry（机载编码器）", odom))
    if ext:
        print(fmt_row("external_pose（坐标纸实测）", ext))
    print()

    def print_err(label: str, e: Optional[Dict[str, float]]) -> None:
        if not e:
            return
        print(
            f"  {label:<24}: dx={e['dx']:+6.2f} mm, dy={e['dy']:+6.2f} mm, "
            f"e_p={e['ep']:5.2f} mm, e_along={e['e_along']:+6.2f} mm, "
            f"e_cross={e['e_cross']:+6.2f} mm, e_theta={e['e_theta_deg']:+6.2f}°"
        )

    print("三组误差拆解：")
    print_err("real_odometry - sim", res["err_odom_sim"])
    print_err("external_pose - odom", res["err_ext_odom"])
    print_err("external_pose - sim", res["err_ext_sim"])
    print_err("external_pose - ideal", res["err_ext_ideal"])

    if odom:
        print("\n编码器与稳态轮速诊断：")
        print(
            f"  总轮行程: dL={odom['dL']:.2f} mm (sim {sim['dL']:.2f}, "
            f"{odom['dL'] - sim['dL']:+.2f} mm / {(odom['dL'] / sim['dL'] - 1) * 100:+.2f}%), "
            f"dR={odom['dR']:.2f} mm (sim {sim['dR']:.2f}, "
            f"{odom['dR'] - sim['dR']:+.2f} mm / {(odom['dR'] / sim['dR'] - 1) * 100:+.2f}%)"
        )
        if not math.isnan(odom["vL_steady_cps"]):
            vL_err = (odom["vL_steady_cps"] / odom["vL_fit_cps"] - 1.0) * 100.0
            vR_err = (odom["vR_steady_cps"] / odom["vR_fit_cps"] - 1.0) * 100.0
            print(
                f"  稳态轮速: L={odom['vL_steady_cps']:.1f} vs fit {odom['vL_fit_cps']:.1f} cnt/s ({vL_err:+.2f}%), "
                f"R={odom['vR_steady_cps']:.1f} vs fit {odom['vR_fit_cps']:.1f} cnt/s ({vR_err:+.2f}%)"
            )
    if ext:
        print(f"\n坐标纸自检：|P_f - P_b| = {ext['marker_dist']:.2f} mm（用于核对前后标记间距）")
    if res["B_eff_est"] is not None:
        print(
            f"单次反推等效几何参考（假设无打滑）: "
            f"B_eff ≈ {res['B_eff_est']:.2f} mm, r_eff ≈ {res['r_eff_est']:.2f} mm"
        )
    print()


def mean_sd(values: List[float]) -> Tuple[float, float]:
    m = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) >= 2 else 0.0
    return m, sd


def run_batch(
    batch_csv: Path,
    model: RobotModel,
    radius_mm: float,
    angle_deg: float,
    left_pwm: int,
    right_pwm: int,
    duration_ms: int,
    start_pose: Tuple[float, float, float],
) -> None:
    """批量分析多次重复试次并输出均值 ± 样本标准差（SD）。

    批量 CSV 格式（带表头）：
      run_id,csv_path,xf,yf,xb,yb
    其中 csv_path 若为相对路径，默认相对于项目根目录解析；可留空仅分析 external_pose。
    """
    repo_root = SCRIPT_DIR.parent
    runs = []
    with batch_csv.open("r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            run_id = row["run_id"].strip()
            csv_raw = (row.get("csv_path") or "").strip()
            csv_path = None
            if csv_raw:
                p = Path(csv_raw)
                csv_path = p if p.is_absolute() else (repo_root / p)
            pf = (float(row["xf"]), float(row["yf"]))
            pb = (float(row["xb"]), float(row["yb"]))
            res = analyze_single_run(
                model, csv_path, pf, pb, radius_mm, angle_deg,
                left_pwm, right_pwm, duration_ms, start_pose,
            )
            runs.append((run_id, res))
            print_single_report(res, left_pwm, right_pwm, duration_ms, title=f"试次 {run_id}")

    if not runs:
        raise ValueError(f"批量文件 {batch_csv} 中没有数据行。")

    print(f"=== 批量汇总统计（n = {len(runs)}，均值 ± 样本标准差 SD）===")
    ext_x, ext_x_sd = mean_sd([r["ext"]["x"] for _, r in runs])
    ext_y, ext_y_sd = mean_sd([r["ext"]["y"] for _, r in runs])
    ext_th, ext_th_sd = mean_sd([r["ext"]["theta_deg"] for _, r in runs])
    ext_s, ext_s_sd = mean_sd([r["ext"]["arc_length"] for _, r in runs])
    ext_r, ext_r_sd = mean_sd([r["ext"]["radius"] for _, r in runs])
    ep_s, ep_s_sd = mean_sd([r["err_ext_sim"]["ep"] for _, r in runs])
    eth_s, eth_s_sd = mean_sd([r["err_ext_sim"]["e_theta_deg"] for _, r in runs])
    print(
        f"  external_pose : x = {ext_x:.2f} ± {ext_x_sd:.2f} mm, "
        f"y = {ext_y:.2f} ± {ext_y_sd:.2f} mm, "
        f"θ = {ext_th:+.2f} ± {ext_th_sd:.2f}°, "
        f"s = {ext_s:.2f} ± {ext_s_sd:.2f} mm, "
        f"R = {ext_r:.2f} ± {ext_r_sd:.2f} mm"
    )
    print(
        f"  ext - sim     : e_p = {ep_s:.2f} ± {ep_s_sd:.2f} mm, "
        f"e_θ = {eth_s:+.2f} ± {eth_s_sd:.2f}°"
    )

    odom_runs = [r for _, r in runs if r["odom"] is not None]
    if odom_runs:
        ox, ox_sd = mean_sd([r["odom"]["x"] for r in odom_runs])
        oy, oy_sd = mean_sd([r["odom"]["y"] for r in odom_runs])
        oth, oth_sd = mean_sd([r["odom"]["theta_deg"] for r in odom_runs])
        os_, os_sd = mean_sd([r["odom"]["arc_length"] for r in odom_runs])
        or_, or_sd = mean_sd([r["odom"]["radius"] for r in odom_runs])
        eo_ep, eo_ep_sd = mean_sd([r["err_ext_odom"]["ep"] for r in odom_runs])
        eo_th, eo_th_sd = mean_sd([r["err_ext_odom"]["e_theta_deg"] for r in odom_runs])
        b_est, b_sd = mean_sd([r["B_eff_est"] for r in odom_runs if r["B_eff_est"] is not None])
        r_est, r_sd = mean_sd([r["r_eff_est"] for r in odom_runs if r["r_eff_est"] is not None])
        print(
            f"  real_odometry : x = {ox:.2f} ± {ox_sd:.2f} mm, "
            f"y = {oy:.2f} ± {oy_sd:.2f} mm, "
            f"θ = {oth:+.2f} ± {oth_sd:.2f}°, "
            f"s = {os_:.2f} ± {os_sd:.2f} mm, "
            f"R = {or_:.2f} ± {or_sd:.2f} mm"
        )
        print(
            f"  ext - odom    : e_p = {eo_ep:.2f} ± {eo_ep_sd:.2f} mm, "
            f"e_θ = {eo_th:+.2f} ± {eo_th_sd:.2f}°"
        )
        print(
            f"  等效几何参考  : B_eff = {b_est:.2f} ± {b_sd:.2f} mm, "
            f"r_eff = {r_est:.2f} ± {r_sd:.2f} mm"
        )


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="圆弧实验三方对比分析脚本（simulation / real_odometry / external_pose）"
    )
    p.add_argument("--csv", type=Path, default=None, help="单次运行的 14 列遥测 CSV 路径")
    p.add_argument("--pf", type=parse_xy, default=None, help="车头坐标 P_f，格式 x,y（mm）")
    p.add_argument("--pb", type=parse_xy, default=None, help="车尾坐标 P_b，格式 x,y（mm）")
    p.add_argument("--batch", type=Path, default=None, help="批量汇总 CSV（run_id,csv_path,xf,yf,xb,yb）")

    p.add_argument("--radius", type=float, default=200.0, help="目标圆弧半径 R (mm)，默认 200")
    p.add_argument("--angle", type=float, default=90.0, help="目标转角 (度)，正左弧负右弧，默认 90")
    p.add_argument("--pwm-left", type=int, default=48, help="左轮 PWM，默认 48")
    p.add_argument("--pwm-right", type=int, default=68, help="右轮 PWM，默认 68")
    p.add_argument("--duration-ms", type=int, default=1730, help="运动时长 (ms)，默认 1730")
    p.add_argument("--start", type=parse_pose, default=(0.0, 0.0, 0.0), help="起点世界位姿 x,y,theta_deg，默认 0,0,0")

    p.add_argument("--r", type=float, default=16.0, help="仿真/里程计轮径 r (mm)，默认 16.0")
    p.add_argument("--r-left", type=float, default=None, help="可选：左轮独立半径 (mm)")
    p.add_argument("--r-right", type=float, default=None, help="可选：右轮独立半径 (mm)")
    p.add_argument("--B", type=float, default=90.0, help="仿真/里程计轮距 B (mm)，默认 90.0")
    p.add_argument("--cpr", type=float, default=358.3, help="编码器每圈脉冲数，默认 358.3")
    p.add_argument("--dt", type=int, default=10, help="仿真离散步长 (ms)，默认 10")
    p.add_argument("--forward-fit", type=Path, default=DEFAULT_FWD_FIT, help="正转 fit.json 路径")
    p.add_argument("--reverse-fit", type=Path, default=DEFAULT_REV_FIT, help="反转 fit.json 路径")
    return p


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    r_left = args.r if args.r_left is None else args.r_left
    r_right = args.r if args.r_right is None else args.r_right
    model = load_model(
        args.forward_fit,
        args.reverse_fit,
        r_left_mm=r_left,
        r_right_mm=r_right,
        wheel_separation_mm=args.B,
        counts_per_rev=args.cpr,
        dt_ms=args.dt,
    )

    if args.batch:
        run_batch(
            args.batch, model, args.radius, args.angle,
            args.pwm_left, args.pwm_right, args.duration_ms, args.start,
        )
        return

    if not args.csv and not (args.pf and args.pb):
        parser.error("请至少提供 --csv 或 --pf/--pb，或者提供 --batch 批量文件。")

    res = analyze_single_run(
        model, args.csv, args.pf, args.pb,
        args.radius, args.angle, args.pwm_left, args.pwm_right, args.duration_ms, args.start,
    )
    print_single_report(res, args.pwm_left, args.pwm_right, args.duration_ms)


if __name__ == "__main__":
    main()
