#!/usr/bin/env python3
"""公用小车 10 ms 阶跃响应与停车滑行拟合脚本。

读取 8 列 STEP 日志：
  STEP, trial, UP/COAST, t_ms, pwmL, pwmR, countL, countR

计算并拟合（左右轮分别输出，支持 R² 与 RMSE 评估）：
  1. 采样质检：检查上升窗 [0, 400 ms] 与滑行窗 [0, 250 ms] 最大采样间隔（> 25 ms 剔除瞬态）。
  2. 稳态轮速 v_ss（UP 阶段 550–980 ms 首尾脉冲斜率，零滤波失真）。
  3. 瞬时轮速 v(t)（40 ms 中心差分）及 10% / 63.2% / 90% 阈值时间、10–90% 上升/下降时间。
  4. 启动延迟 t_d（首次计数跳变时刻 t_first 及前一帧 t_prev）。
  5. 启动一阶拟合：
       - 带延迟 FOPDT：v(t) = v_ss * (1 - exp(-(t - t_d) / tau_up))      (t >= t_d)
       - 纯一阶（无延迟）：v(t) = v_ss * (1 - exp(-t / tau_up_nolag))     (t >= 0)
       - 行程等效时间常数：tau_up_eq = T_run - (s_drive / v_ss) * 1000
  6. 停车滑行拟合：
       - 滑行距离 s_coast、完全停稳时间 t_stop（最后一次计数跳变时刻）
       - 曲线一阶拟合：v(t) = v_ss * exp(-t / tau_coast)
       - 行程等效时间常数：tau_coast_eq = (s_coast / v_ss) * 1000
  7. 导出 JSON 结果以便复现与接入 DigitalTwin/MotorsModel.pde。

用法：
  python3 src/fit_step_response.py --csv logs/step_response_shared_58_57.csv
  python3 src/fit_step_response.py --csv logs/step_response_shared_58_57.csv --r 16.36 --B 85.8
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple


def load_step_trials(path: Path) -> Dict[int, Dict[str, List[Dict[str, float]]]]:
    trials: Dict[int, Dict[str, List[Dict[str, float]]]] = defaultdict(
        lambda: {"UP": [], "COAST": []}
    )
    with path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            parts = [p.strip() for p in line.strip().split(",")]
            if len(parts) != 8 or parts[0] != "STEP":
                continue
            phase = parts[2]
            if phase not in ("UP", "COAST"):
                continue
            trial = int(parts[1])
            trials[trial][phase].append(
                {
                    "t_ms": float(parts[3]),
                    "pwm_l": float(parts[4]),
                    "pwm_r": float(parts[5]),
                    "cl": float(parts[6]),
                    "cr": float(parts[7]),
                }
            )
    return dict(sorted(trials.items()))


def central_diff_speed(
    t_ms: List[float], counts: List[float], mm_per_count: float, half_win: int = 2
) -> List[float]:
    """用 2*half_win 帧中心差分估算瞬时轮速（mm/s），half_win=2 约对应 40 ms 窗口。"""
    n = len(t_ms)
    v = [0.0] * n
    for i in range(n):
        i0 = max(0, i - half_win)
        i1 = min(n - 1, i + half_win)
        dt = t_ms[i1] - t_ms[i0]
        if dt > 0:
            v[i] = (counts[i1] - counts[i0]) * mm_per_count * 1000.0 / dt
    return v


def interp_crossing(
    t_ms: List[float], v: List[float], target: float, rising: bool = True
) -> float:
    for i in range(1, len(t_ms)):
        if rising and v[i - 1] < target <= v[i]:
            frac = (target - v[i - 1]) / (v[i] - v[i - 1])
            return t_ms[i - 1] + frac * (t_ms[i] - t_ms[i - 1])
        if not rising and v[i - 1] > target >= v[i]:
            frac = (v[i - 1] - target) / (v[i - 1] - v[i])
            return t_ms[i - 1] + frac * (t_ms[i] - t_ms[i - 1])
    return float("nan")


def compute_r2_rmse(y_true: List[float], y_pred: List[float]) -> Tuple[float, float]:
    n = len(y_true)
    if n < 2:
        return float("nan"), float("nan")
    mean_y = sum(y_true) / n
    ss_res = sum((a - b) ** 2 for a, b in zip(y_true, y_pred))
    ss_tot = sum((a - mean_y) ** 2 for a in y_true)
    rmse = math.sqrt(ss_res / n)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 1e-12 else float("nan")
    return r2, rmse


def fit_up_first_order(
    t_ms: List[float], v: List[float], v_ss: float, t_lag: float
) -> Tuple[float, float, float]:
    """网格搜索拟合启动一阶模型：v(t) = v_ss * (1 - exp(-(t - t_lag) / tau))，t >= t_lag。"""
    pts = [(t - t_lag, val) for t, val in zip(t_ms, v) if t >= t_lag]
    if len(pts) < 5:
        return float("nan"), float("nan"), float("nan")
    best_tau = float("nan")
    best_sse = float("inf")
    # 0.5 ms 分辨率扫描 5..400 ms
    for k in range(10, 801):
        tau = k * 0.5
        sse = sum((val - v_ss * (1.0 - math.exp(-dt / tau))) ** 2 for dt, val in pts)
        if sse < best_sse:
            best_sse = sse
            best_tau = tau
    y_true = [val for _, val in pts]
    y_pred = [v_ss * (1.0 - math.exp(-dt / best_tau)) for dt, _ in pts]
    r2, rmse = compute_r2_rmse(y_true, y_pred)
    return best_tau, r2, rmse


def fit_up_fopdt_joint(
    t_ms: List[float], v: List[float], v_ss: float
) -> Tuple[float, float, float, float]:
    """联合网格搜索 (t_d, tau_up) 拟合 FOPDT：v(t) = 0 (t < t_d), v_ss*(1-exp(-(t-t_d)/tau)) (t >= t_d)。"""
    pts = [(t, val) for t, val in zip(t_ms, v) if t <= 600.0]
    if len(pts) < 5:
        return float("nan"), float("nan"), float("nan"), float("nan")
    best_td = 0.0
    best_tau = float("nan")
    best_sse = float("inf")
    for td_i in range(0, 151, 2):  # 0..150 ms, step 2 ms
        td = float(td_i)
        for tau_i in range(10, 301, 2):  # 10..300 ms, step 2 ms
            tau = float(tau_i)
            sse = 0.0
            for t, val in pts:
                pred = 0.0 if t <= td else v_ss * (1.0 - math.exp(-(t - td) / tau))
                d = val - pred
                sse += d * d
            if sse < best_sse:
                best_sse = sse
                best_td = td
                best_tau = tau
    # 局部 0.5 ms 细搜
    for td_k in range(max(0, int((best_td - 4) * 2)), int((best_td + 4) * 2) + 1):
        td = td_k * 0.5
        for tau_k in range(max(10, int((best_tau - 4) * 2)), int((best_tau + 4) * 2) + 1):
            tau = tau_k * 0.5
            sse = 0.0
            for t, val in pts:
                pred = 0.0 if t <= td else v_ss * (1.0 - math.exp(-(t - td) / tau))
                d = val - pred
                sse += d * d
            if sse < best_sse:
                best_sse = sse
                best_td = td
                best_tau = tau
    y_true = [val for _, val in pts]
    y_pred = [0.0 if t <= best_td else v_ss * (1.0 - math.exp(-(t - best_td) / best_tau)) for t, _ in pts]
    r2, rmse = compute_r2_rmse(y_true, y_pred)
    return best_td, best_tau, r2, rmse


def fit_coast_first_order(
    t_ms: List[float], v: List[float], v_ss: float, fit_window_ms: float = 350.0
) -> Tuple[float, float, float]:
    """拟合停车滑行一阶指数衰减：v(t) = v_ss * exp(-t / tau_coast)。"""
    pts = [(t, val) for t, val in zip(t_ms, v) if 0.0 <= t <= fit_window_ms]
    if len(pts) < 5:
        return float("nan"), float("nan"), float("nan")
    best_tau = float("nan")
    best_sse = float("inf")
    for k in range(10, 601):  # 5..300 ms, step 0.5 ms
        tau = k * 0.5
        sse = sum((val - v_ss * math.exp(-t / tau)) ** 2 for t, val in pts)
        if sse < best_sse:
            best_sse = sse
            best_tau = tau
    y_true = [val for _, val in pts]
    y_pred = [v_ss * math.exp(-t / best_tau) for t, _ in pts]
    r2, rmse = compute_r2_rmse(y_true, y_pred)
    return best_tau, r2, rmse


def analyse_trial(
    up_rows: List[Dict[str, float]],
    coast_rows: List[Dict[str, float]],
    mm_per_count: float,
    wheelbase_mm: float,
) -> Dict[str, object]:
    t_up = [r["t_ms"] for r in up_rows]
    cl_up = [r["cl"] for r in up_rows]
    cr_up = [r["cr"] for r in up_rows]

    t_co = [r["t_ms"] for r in coast_rows]
    cl_co = [r["cl"] for r in coast_rows]
    cr_co = [r["cr"] for r in coast_rows]

    t_run_ms = t_up[-1]
    # 拼接连续时间轴计算 40 ms 中心差分轮速（去掉 COAST t=0 的重复边界点）
    t_all = t_up + [t_run_ms + t for t in t_co[1:]]
    cl_all = cl_up + cl_co[1:]
    cr_all = cr_up + cr_co[1:]
    vl_all = central_diff_speed(t_all, cl_all, mm_per_count, half_win=2)
    vr_all = central_diff_speed(t_all, cr_all, mm_per_count, half_win=2)

    up_rise_t = [t for t in t_up if t <= 400.0]
    co_fall_t = [t for t in t_co if t <= 250.0]
    up_max_dt = max((up_rise_t[i] - up_rise_t[i - 1] for i in range(1, len(up_rise_t))), default=0.0)
    co_max_dt = max((co_fall_t[i] - co_fall_t[i - 1] for i in range(1, len(co_fall_t))), default=0.0)

    n_up = len(t_up)
    res: Dict[str, object] = {
        "pwm_l": int(round(up_rows[0]["pwm_l"])),
        "pwm_r": int(round(up_rows[0]["pwm_r"])),
        "t_run_ms": t_run_ms,
        "up_max_dt_ms": up_max_dt,
        "coast_max_dt_ms": co_max_dt,
        "clean_up": up_max_dt <= 25.0,
        "clean_coast": co_max_dt <= 25.0,
    }

    for side, c_up, c_co, v_all in [("L", cl_up, cl_co, vl_all), ("R", cr_up, cr_co, vr_all)]:
        sign = 1.0 if (c_up[-1] - c_up[0]) >= 0 else -1.0
        # 1) 启动首次计数延迟
        moved = [i for i, c in enumerate(c_up) if sign * (c - c_up[0]) > 0]
        t_first = t_up[moved[0]] if moved else float("nan")
        t_prev = t_up[moved[0] - 1] if (moved and moved[0] > 0) else 0.0

        # 2) 稳态轮速（UP 阶段 550..980 ms 首尾精确斜率）
        ss_lo = min(550.0, 0.55 * t_run_ms)
        ss_hi = max(ss_lo + 100.0, t_run_ms - 20.0)
        ss_idx = [i for i, t in enumerate(t_up) if ss_lo <= t <= ss_hi]
        v_ss = (
            sign
            * (c_up[ss_idx[-1]] - c_up[ss_idx[0]])
            * mm_per_count
            * 1000.0
            / (t_up[ss_idx[-1]] - t_up[ss_idx[0]])
        )
        v_ss_cps = v_ss / mm_per_count

        # 3) 上升瞬态与一阶拟合
        v_up = [sign * x for x in v_all[:n_up]]
        t10 = interp_crossing(t_up, v_up, 0.10 * v_ss, rising=True)
        t63 = interp_crossing(t_up, v_up, 0.632 * v_ss, rising=True)
        t90 = interp_crossing(t_up, v_up, 0.90 * v_ss, rising=True)
        rise_10_90 = t90 - t10

        tau_up_prev, r2_up_prev, rmse_up_prev = fit_up_first_order(t_up, v_up, v_ss, t_prev)
        td_joint, tau_up_joint, r2_joint, rmse_joint = fit_up_fopdt_joint(t_up, v_up, v_ss)
        tau_up_nolag, r2_nolag, rmse_nolag = fit_up_first_order(t_up, v_up, v_ss, 0.0)

        drive_mm = sign * (c_up[-1] - c_up[0]) * mm_per_count
        coast_mm = sign * (c_co[-1] - c_co[0]) * mm_per_count
        total_mm = sign * (c_co[-1] - c_up[0]) * mm_per_count

        # 行程等效一阶时间常数（保证总行程精确守恒）
        # s_drive = integral_0^{T} v_ss*(1 - exp(-t/tau)) dt ≈ v_ss * (T - tau_up_eq)
        tau_up_eq = t_run_ms - (drive_mm / v_ss) * 1000.0
        # s_coast = integral_0^{inf} v_ss*exp(-t/tau) dt = v_ss * tau_coast_eq
        tau_coast_eq = (coast_mm / v_ss) * 1000.0
        net_lag_ms = tau_up_eq - tau_coast_eq

        # 4) 滑行瞬态与一阶指数衰减拟合
        v_co = [sign * v_all[n_up - 1]] + [sign * x for x in v_all[n_up:]]
        t_fall_90 = interp_crossing(t_co, v_co, 0.90 * v_ss, rising=False)
        t_fall_10 = interp_crossing(t_co, v_co, 0.10 * v_ss, rising=False)
        fall_90_10 = t_fall_10 - t_fall_90
        tau_coast_fit, r2_co, rmse_co = fit_coast_first_order(t_co, v_co, v_ss)

        last_inc = [i for i in range(1, len(c_co)) if sign * (c_co[i] - c_co[i - 1]) > 0]
        t_stop_last = t_co[last_inc[-1]] if last_inc else 0.0

        res[side] = {
            "t_first_ms": t_first,
            "t_prev_ms": t_prev,
            "v_ss_mm_s": v_ss,
            "v_ss_cps": v_ss_cps,
            "t10_ms": t10,
            "t63_ms": t63,
            "t90_ms": t90,
            "rise_10_90_ms": rise_10_90,
            "tau_up_fixed_td_ms": tau_up_prev,
            "r2_up_fixed_td": r2_up_prev,
            "rmse_up_fixed_td_mm_s": rmse_up_prev,
            "td_joint_ms": td_joint,
            "tau_up_joint_ms": tau_up_joint,
            "r2_joint": r2_joint,
            "rmse_joint_mm_s": rmse_joint,
            "tau_up_nolag_ms": tau_up_nolag,
            "r2_nolag": r2_nolag,
            "rmse_nolag_mm_s": rmse_nolag,
            "tau_up_eq_ms": tau_up_eq,
            "drive_mm": drive_mm,
            "coast_mm": coast_mm,
            "total_mm": total_mm,
            "t_stop_last_ms": t_stop_last,
            "fall_90_10_ms": fall_90_10,
            "tau_coast_fit_ms": tau_coast_fit,
            "r2_coast": r2_co,
            "rmse_coast_mm_s": rmse_co,
            "tau_coast_eq_ms": tau_coast_eq,
            "net_lag_ms": net_lag_ms,
        }

    # 车身中心行程与偏航角（基于编码器）
    dl_drive, dr_drive = res["L"]["drive_mm"], res["R"]["drive_mm"]
    dl_coast, dr_coast = res["L"]["coast_mm"], res["R"]["coast_mm"]
    res["drive_center_mm"] = 0.5 * (dl_drive + dr_drive)
    res["coast_center_mm"] = 0.5 * (dl_coast + dr_coast)
    res["total_center_mm"] = res["drive_center_mm"] + res["coast_center_mm"]
    res["drive_yaw_deg"] = math.degrees((dr_drive - dl_drive) / wheelbase_mm)
    res["coast_yaw_deg"] = math.degrees((dr_coast - dl_coast) / wheelbase_mm)
    res["total_yaw_deg"] = res["drive_yaw_deg"] + res["coast_yaw_deg"]
    return res


def mean_sd(vals: List[float]) -> Tuple[float, float, int]:
    clean = [v for v in vals if not math.isnan(v)]
    n = len(clean)
    if n == 0:
        return float("nan"), float("nan"), 0
    if n == 1:
        return clean[0], 0.0, 1
    return statistics.mean(clean), statistics.stdev(clean), n


def main() -> None:
    p = argparse.ArgumentParser(description="公用小车 10 ms 阶跃响应与滑行拟合脚本")
    p.add_argument("--csv", type=Path, required=True, help="STEP 日志文件路径")
    p.add_argument("--r", type=float, default=16.36, help="有效轮径 r (mm)，默认 16.36")
    p.add_argument("--B", type=float, default=85.8, help="有效轮距 B (mm)，默认 85.8")
    p.add_argument("--cpr", type=float, default=358.3, help="编码器每圈脉冲数，默认 358.3")
    p.add_argument("--out-json", type=Path, default=None, help="可选：保存完整结果 JSON 路径")
    args = p.parse_args()

    mm_per_count = 2.0 * math.pi * args.r / args.cpr
    trials = load_step_trials(args.csv)
    if not trials:
        raise SystemExit(f"未在 {args.csv} 中找到有效 STEP 记录。")

    results = {}
    print(f"=== 阶跃响应拟合报告：{args.csv.name} (r={args.r} mm, B={args.B} mm) ===")
    print("采样质量检查（瞬态窗最大间隔 <= 25 ms 判定为干净）：")
    for tid, data in trials.items():
        res = analyse_trial(data["UP"], data["COAST"], mm_per_count, args.B)
        results[tid] = res
        print(
            f"  Trial {tid}: PWM=({res['pwm_l']},{res['pwm_r']}), T={res['t_run_ms']:.0f} ms | "
            f"UP max_dt={res['up_max_dt_ms']:.0f} ms ({'OK' if res['clean_up'] else 'STALL'}) | "
            f"COAST max_dt={res['coast_max_dt_ms']:.0f} ms ({'OK' if res['clean_coast'] else 'STALL'})"
        )

    all_t = list(results.values())
    clean_up = [r for r in all_t if r["clean_up"]] or all_t
    clean_co = [r for r in all_t if r["clean_coast"]] or all_t

    print(
        f"\n汇总统计（均值 ± 样本标准差 SD；全试次 n={len(all_t)}，干净启动 n={len(clean_up)}，干净滑行 n={len(clean_co)}）："
    )
    metrics = [
        ("稳态轮速 v_ss (counts/s)", "v_ss_cps", all_t),
        ("稳态轮速 v_ss (mm/s)", "v_ss_mm_s", all_t),
        ("首次计数延迟 t_first (ms)", "t_first_ms", clean_up),
        ("达到 63.2% 稳态时间 t63 (ms)", "t63_ms", clean_up),
        ("10–90% 上升时间 (ms)", "rise_10_90_ms", clean_up),
        ("【FOPDT 联合拟合】纯延迟 t_d (ms)", "td_joint_ms", clean_up),
        ("【FOPDT 联合拟合】加速 τ_up (ms)", "tau_up_joint_ms", clean_up),
        ("【FOPDT 联合拟合】R²", "r2_joint", clean_up),
        ("【纯一阶无延迟】加速 τ_up (ms)", "tau_up_nolag_ms", clean_up),
        ("【纯一阶无延迟】R²", "r2_nolag", clean_up),
        ("【行程等效】总加速时间常数 τ_up_eq (ms)", "tau_up_eq_ms", all_t),
        ("驱动行程 s_drive (mm)", "drive_mm", all_t),
        ("滑行行程 s_coast (mm)", "coast_mm", all_t),
        ("停稳时间 t_stop (ms)", "t_stop_last_ms", all_t),
        ("90–10% 下降时间 (ms)", "fall_90_10_ms", clean_co),
        ("【一阶衰减拟合】滑行 τ_coast (ms)", "tau_coast_fit_ms", clean_co),
        ("【一阶衰减拟合】R²", "r2_coast", clean_co),
        ("【行程等效】滑行 τ_coast_eq (ms)", "tau_coast_eq_ms", all_t),
        ("净动态时间损失 τ_up_eq - τ_coast_eq (ms)", "net_lag_ms", all_t),
    ]

    print(f"{'指标':<36} | {'左轮 (PWM ' + str(all_t[0]['pwm_l']) + ')':>22} | {'右轮 (PWM ' + str(all_t[0]['pwm_r']) + ')':>22} | {'n':>3}")
    print("-" * 92)
    summary = {"L": {}, "R": {}}
    for label, key, subset in metrics:
        ml, sdl, nl = mean_sd([r["L"][key] for r in subset])
        mr, sdr, nr = mean_sd([r["R"][key] for r in subset])
        summary["L"][key] = {"mean": ml, "sd": sdl, "n": nl}
        summary["R"][key] = {"mean": mr, "sd": sdr, "n": nr}
        fmt = ".4f" if "R²" in label else ".2f"
        print(f"{label:<36} | {ml:9{fmt}} ± {sdl:8{fmt}} | {mr:9{fmt}} ± {sdr:8{fmt}} | {nl:3d}")

    print("\n整车行程与编码器偏航（正值向左）：")
    for label, key in [
        ("驱动阶段中心行程 (mm)", "drive_center_mm"),
        ("滑行阶段中心行程 (mm)", "coast_center_mm"),
        ("停稳后总中心行程 (mm)", "total_center_mm"),
        ("驱动阶段偏航 (°)", "drive_yaw_deg"),
        ("滑行阶段偏航增量 (°)", "coast_yaw_deg"),
        ("停稳后净偏航 (°)", "total_yaw_deg"),
    ]:
        m, sd, n = mean_sd([r[key] for r in all_t])
        print(f"  {label:<24}: {m:+.2f} ± {sd:.2f} (n={n})")

    if args.out_json:
        args.out_json.parent.mkdir(parents=True, exist_ok=True)
        payload = {"csv": str(args.csv), "r_mm": args.r, "B_mm": args.B, "summary": summary, "trials": results}
        args.out_json.write_text(json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"\n已保存 JSON 结果至: {args.out_json}")


if __name__ == "__main__":
    main()
