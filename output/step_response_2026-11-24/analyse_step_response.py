"""Analyse 10 ms step-response and coasting logs (Motor Exercise 7: Slew-rate & lag).

Reads:  logs/step_response_62_58.csv
Writes: output/step_response_2026-11-24/{results.json, step_response_summary.png, step_response_summary.pdf}
"""

from __future__ import annotations

import json
import math
import os
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev

os.environ.setdefault("MPLCONFIGDIR", "/tmp/report-figs-matplotlib")
os.environ.setdefault("XDG_CACHE_HOME", "/tmp/report-figs-fontcache")

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
LOG_PATH = ROOT / "logs" / "step_response_62_58.csv"
OUT_DIR = Path(__file__).resolve().parent

WHEEL_RADIUS_MM = 16.0
COUNTS_PER_REV = 358.3
WHEELBASE_MM = 90.0
MM_PER_COUNT = (2.0 * math.pi * WHEEL_RADIUS_MM) / COUNTS_PER_REV


def load_trials(path: Path) -> dict[int, dict[str, list[dict[str, float]]]]:
    trials: dict[int, dict[str, list[dict[str, float]]]] = defaultdict(lambda: {"UP": [], "COAST": []})
    with path.open(encoding="utf-8") as f:
        for line in f:
            parts = [p.strip() for p in line.split(",")]
            if len(parts) != 8 or parts[0] != "STEP":
                continue
            trial = int(parts[1])
            phase = parts[2]
            if phase not in ("UP", "COAST"):
                continue
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


def compute_velocity_window(t_ms: np.ndarray, counts: np.ndarray, half_win: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Compute wheel rim speed (mm/s) via central difference across 2*half_win samples (~20-40 ms)."""
    n = len(t_ms)
    v = np.zeros(n, dtype=float)
    for i in range(n):
        i0 = max(0, i - half_win)
        i1 = min(n - 1, i + half_win)
        dt = t_ms[i1] - t_ms[i0]
        if dt > 0:
            v[i] = (counts[i1] - counts[i0]) * MM_PER_COUNT * 1000.0 / dt
    return t_ms, v


def interp_crossing(t: np.ndarray, y: np.ndarray, target: float, rising: bool = True) -> float:
    for i in range(1, len(t)):
        if rising and y[i - 1] < target <= y[i]:
            frac = (target - y[i - 1]) / (y[i] - y[i - 1])
            return float(t[i - 1] + frac * (t[i] - t[i - 1]))
        if not rising and y[i - 1] > target >= y[i]:
            frac = (y[i - 1] - target) / (y[i - 1] - y[i])
            return float(t[i - 1] + frac * (t[i] - t[i - 1]))
    return float("nan")


def fit_first_order_tau(t: np.ndarray, v: np.ndarray, v_ss: float, t_lag: float) -> float:
    """Grid search first-order time constant tau (ms) for v(t) = v_ss * (1 - exp(-(t-t_lag)/tau))."""
    mask = t >= t_lag
    tm = t[mask] - t_lag
    vm = v[mask]
    taus = np.linspace(10.0, 400.0, 781)
    best_tau = float("nan")
    best_err = float("inf")
    for tau in taus:
        pred = v_ss * (1.0 - np.exp(-tm / tau))
        err = float(np.mean((vm - pred) ** 2))
        if err < best_err:
            best_err = err
            best_tau = float(tau)
    return best_tau


def analyse_trial(up_rows: list[dict], coast_rows: list[dict]) -> dict:
    # Build continuous timeline: UP is [0, T_up], COAST is [T_up, T_up + T_coast]
    t_up = np.array([r["t_ms"] for r in up_rows], dtype=float)
    cl_up = np.array([r["cl"] for r in up_rows], dtype=float)
    cr_up = np.array([r["cr"] for r in up_rows], dtype=float)

    t_co = np.array([r["t_ms"] for r in coast_rows], dtype=float)
    cl_co = np.array([r["cl"] for r in coast_rows], dtype=float)
    cr_co = np.array([r["cr"] for r in coast_rows], dtype=float)

    t_stop_cmd = float(t_up[-1])
    # Drop duplicate transition point at COAST t=0 when concatenating for continuous velocity
    t_all = np.concatenate([t_up, t_stop_cmd + t_co[1:]])
    cl_all = np.concatenate([cl_up, cl_co[1:]])
    cr_all = np.concatenate([cr_up, cr_co[1:]])

    # 30 ms-ish central window (half_win=2 -> ~40 ms span, smooth & low quantization ~7 mm/s)
    # and half_win=1 (~20 ms span) for crisp rise-time extraction
    _, vl_20 = compute_velocity_window(t_all, cl_all, half_win=1)
    _, vr_20 = compute_velocity_window(t_all, cr_all, half_win=1)
    _, vl_40 = compute_velocity_window(t_all, cl_all, half_win=2)
    _, vr_40 = compute_velocity_window(t_all, cr_all, half_win=2)

    # Detect any Wi-Fi/TCP timestamp stalls (>25 ms) during rise [0, 400] ms or coast [0, 250] ms
    up_rise_mask = t_up <= 400.0
    up_max_dt = float(np.max(np.diff(t_up[up_rise_mask]))) if np.sum(up_rise_mask) > 1 else 0.0
    co_fall_mask = t_co <= 250.0
    co_max_dt = float(np.max(np.diff(t_co[co_fall_mask]))) if np.sum(co_fall_mask) > 1 else 0.0

    n_up = len(t_up)
    res = {
        "t_stop_cmd_ms": t_stop_cmd,
        "up_rise_max_dt_ms": up_max_dt,
        "coast_fall_max_dt_ms": co_max_dt,
        "clean_up_transient": up_max_dt <= 25.0,
        "clean_coast_transient": co_max_dt <= 25.0,
    }

    for side, c_up, c_co, v20, v40 in [
        ("L", cl_up, cl_co, vl_20, vl_40),
        ("R", cr_up, cr_co, vr_20, vr_40),
    ]:
        # 1) Pure dead-time lag: first UP sample where cumulative count increases from c_up[0]
        moved_idx = np.where(c_up > c_up[0])[0]
        t_lag_first = float(t_up[moved_idx[0]]) if len(moved_idx) else float("nan")
        t_lag_prev = float(t_up[moved_idx[0] - 1]) if len(moved_idx) and moved_idx[0] > 0 else 0.0

        # 2) Steady-state speed over UP [550, 980] ms directly from count slope (exact, zero window error)
        ss_mask = (t_up >= 550.0) & (t_up <= 980.0)
        ss_idx = np.where(ss_mask)[0]
        v_ss = float((c_up[ss_idx[-1]] - c_up[ss_idx[0]]) * MM_PER_COUNT * 1000.0 / (t_up[ss_idx[-1]] - t_up[ss_idx[0]]))

        # 3) Rise times & slew rate using 40 ms central diff during UP
        v_up = v40[:n_up]
        t10 = interp_crossing(t_up, v_up, 0.10 * v_ss, rising=True)
        t63 = interp_crossing(t_up, v_up, 0.632 * v_ss, rising=True)
        t90 = interp_crossing(t_up, v_up, 0.90 * v_ss, rising=True)
        rise_10_90 = t90 - t10
        slew_up = (0.80 * v_ss) / (rise_10_90 / 1000.0) if rise_10_90 > 0 else float("nan")
        tau_up = fit_first_order_tau(t_up, v_up, v_ss, t_lag_prev)

        # 4) Coasting phase metrics
        c_stop = float(c_co[0])
        c_final = float(c_co[-1])
        coast_counts = c_final - c_stop
        coast_mm = coast_counts * MM_PER_COUNT

        # Stop time: timestamp of the LAST count increment during COAST
        dc_co = np.diff(c_co)
        last_move_idx = np.where(dc_co > 0)[0]
        t_stop_last = float(t_co[last_move_idx[-1] + 1]) if len(last_move_idx) else 0.0

        # Fall time 90% -> 10% during COAST
        v_co = np.concatenate([[v40[n_up - 1]], v40[n_up:]])
        t_fall_90 = interp_crossing(t_co, v_co, 0.90 * v_ss, rising=False)
        t_fall_10 = interp_crossing(t_co, v_co, 0.10 * v_ss, rising=False)
        fall_90_10 = t_fall_10 - t_fall_90
        slew_down = (0.80 * v_ss) / (fall_90_10 / 1000.0) if fall_90_10 > 0 else float("nan")

        # Total UP travel
        up_mm = float((c_up[-1] - c_up[0]) * MM_PER_COUNT)

        res[side] = {
            "t_lag_prev_ms": t_lag_prev,
            "t_lag_first_ms": t_lag_first,
            "v_ss_mm_s": v_ss,
            "t10_ms": t10,
            "t63_ms": t63,
            "t90_ms": t90,
            "rise_10_90_ms": rise_10_90,
            "slew_up_mm_s2": slew_up,
            "tau_up_ms": tau_up,
            "up_dist_mm": up_mm,
            "coast_counts": coast_counts,
            "coast_dist_mm": coast_mm,
            "t_stop_ms": t_stop_last,
            "t_fall_90_ms": t_fall_90,
            "t_fall_10_ms": t_fall_10,
            "fall_90_10_ms": fall_90_10,
            "slew_down_mm_s2": slew_down,
        }

    # Differential yaw estimates (positive = left/CCW turn, R > L)
    d_up_l = res["L"]["up_dist_mm"]
    d_up_r = res["R"]["up_dist_mm"]
    d_co_l = res["L"]["coast_dist_mm"]
    d_co_r = res["R"]["coast_dist_mm"]
    res["yaw_up_deg"] = math.degrees((d_up_r - d_up_l) / WHEELBASE_MM)
    res["yaw_coast_deg"] = math.degrees((d_co_r - d_co_l) / WHEELBASE_MM)
    res["yaw_total_deg"] = res["yaw_up_deg"] + res["yaw_coast_deg"]
    res["center_up_mm"] = 0.5 * (d_up_l + d_up_r)
    res["center_coast_mm"] = 0.5 * (d_co_l + d_co_r)

    # Interpolated series on a clean 10 ms grid [0, 1950] ms for plotting mean +/- SD
    t_grid = np.arange(0.0, 1960.0, 10.0)
    res["_grid"] = {
        "t": t_grid,
        "vl": np.interp(t_grid, t_all, vl_40),
        "vr": np.interp(t_grid, t_all, vr_40),
        "theta_deg": np.degrees(((np.interp(t_grid, t_all, cr_all - cr_all[0]) - np.interp(t_grid, t_all, cl_all - cl_all[0])) * MM_PER_COUNT) / WHEELBASE_MM),
    }
    return res


def summarise_trials(log_path: Path):
    trials = load_trials(log_path)
    per_trial = {}
    grids = []

    for tid, phases in trials.items():
        r = analyse_trial(phases["UP"], phases["COAST"])
        grids.append(r.pop("_grid"))
        per_trial[tid] = r

    # Aggregate mean +/- SD across trials (use gap-free trials for UP/COAST transient timing/slew)
    clean_up_ids = [tid for tid in per_trial if per_trial[tid]["clean_up_transient"]]
    clean_co_ids = [tid for tid in per_trial if per_trial[tid]["clean_coast_transient"]]
    all_ids = list(per_trial.keys())

    summary: dict[str, dict] = {
        "n_trials_total": len(all_ids),
        "clean_up_trials": clean_up_ids,
        "clean_coast_trials": clean_co_ids,
        "L": {},
        "R": {},
        "vehicle": {},
    }
    up_transient_metrics = {"t_lag_first_ms", "t10_ms", "t63_ms", "t90_ms", "rise_10_90_ms", "slew_up_mm_s2", "tau_up_ms"}
    co_transient_metrics = {"fall_90_10_ms", "slew_down_mm_s2"}
    metrics = [
        "t_lag_first_ms",
        "v_ss_mm_s",
        "t10_ms",
        "t63_ms",
        "t90_ms",
        "rise_10_90_ms",
        "slew_up_mm_s2",
        "tau_up_ms",
        "up_dist_mm",
        "coast_dist_mm",
        "t_stop_ms",
        "fall_90_10_ms",
        "slew_down_mm_s2",
    ]
    for side in ("L", "R"):
        for m in metrics:
            use_ids = clean_up_ids if m in up_transient_metrics else (clean_co_ids if m in co_transient_metrics else all_ids)
            vals = [per_trial[tid][side][m] for tid in use_ids]
            summary[side][m] = {"n": len(vals), "mean": mean(vals), "sd": stdev(vals) if len(vals) > 1 else 0.0, "values": vals}

    for vm in ("yaw_up_deg", "yaw_coast_deg", "yaw_total_deg", "center_up_mm", "center_coast_mm"):
        vals = [per_trial[tid][vm] for tid in all_ids]
        summary["vehicle"][vm] = {"n": len(vals), "mean": mean(vals), "sd": stdev(vals) if len(vals) > 1 else 0.0, "values": vals}

    return summary, per_trial, grids


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    summary, per_trial, grids = summarise_trials(LOG_PATH)
    all_ids = list(per_trial)
    clean_up_ids = summary["clean_up_trials"]
    clean_co_ids = summary["clean_coast_trials"]

    with (OUT_DIR / "results.json").open("w", encoding="utf-8") as f:
        json.dump({"summary": summary, "trials": per_trial}, f, indent=2)

    print(
        f"=== Step Response & Coasting Summary: PWM (62, 58) ===\n"
        f"Total trials: {len(all_ids)} | Gap-free UP trials: {clean_up_ids} | Gap-free COAST trials: {clean_co_ids}"
    )
    for side in ("L", "R"):
        s = summary[side]
        print(f"\n[{side} wheel]")
        print(f"  Dead-time lag (first count, n={s['t_lag_first_ms']['n']}): {s['t_lag_first_ms']['mean']:.1f} +/- {s['t_lag_first_ms']['sd']:.1f} ms")
        print(f"  Steady speed (550-980 ms, n={s['v_ss_mm_s']['n']}):   {s['v_ss_mm_s']['mean']:.2f} +/- {s['v_ss_mm_s']['sd']:.2f} mm/s")
        print(f"  t10 / t63 / t90 (n={s['t10_ms']['n']}):             {s['t10_ms']['mean']:.1f} / {s['t63_ms']['mean']:.1f} / {s['t90_ms']['mean']:.1f} ms")
        print(f"  Rise time 10-90% (n={s['rise_10_90_ms']['n']}):          {s['rise_10_90_ms']['mean']:.1f} +/- {s['rise_10_90_ms']['sd']:.1f} ms")
        print(f"  Start-up slew-rate (n={s['slew_up_mm_s2']['n']}):        {s['slew_up_mm_s2']['mean']:.1f} +/- {s['slew_up_mm_s2']['sd']:.1f} mm/s^2")
        print(f"  First-order tau_up (n={s['tau_up_ms']['n']}):        {s['tau_up_ms']['mean']:.1f} +/- {s['tau_up_ms']['sd']:.1f} ms")
        print(f"  Coasting distance (n={s['coast_dist_mm']['n']}):         {s['coast_dist_mm']['mean']:.2f} +/- {s['coast_dist_mm']['sd']:.2f} mm")
        print(f"  Stop time last count (n={s['t_stop_ms']['n']}):      {s['t_stop_ms']['mean']:.1f} +/- {s['t_stop_ms']['sd']:.1f} ms")
        print(f"  Fall time 90-10% (n={s['fall_90_10_ms']['n']}):          {s['fall_90_10_ms']['mean']:.1f} +/- {s['fall_90_10_ms']['sd']:.1f} ms")
        print(f"  Stopping slew-rate (n={s['slew_down_mm_s2']['n']}):        {s['slew_down_mm_s2']['mean']:.1f} +/- {s['slew_down_mm_s2']['sd']:.1f} mm/s^2")

    v = summary["vehicle"]
    print("\n[Vehicle displacement & yaw]")
    print(f"  UP center travel:    {v['center_up_mm']['mean']:.2f} +/- {v['center_up_mm']['sd']:.2f} mm")
    print(f"  COAST center travel: {v['center_coast_mm']['mean']:.2f} +/- {v['center_coast_mm']['sd']:.2f} mm")
    print(f"  UP yaw:              {v['yaw_up_deg']['mean']:+.2f} +/- {v['yaw_up_deg']['sd']:.2f} deg")
    print(f"  COAST yaw:           {v['yaw_coast_deg']['mean']:+.2f} +/- {v['yaw_coast_deg']['sd']:.2f} deg")
    print(f"  TOTAL yaw:           {v['yaw_total_deg']['mean']:+.2f} +/- {v['yaw_total_deg']['sd']:.2f} deg")

    # Plot summary figure (velocity + cumulative heading across full UP + COAST)
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t_grid = grids[0]["t"]
    vl_mat = np.array([g["vl"] for g in grids])
    vr_mat = np.array([g["vr"] for g in grids])
    th_mat = np.array([g["theta_deg"] for g in grids])

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(7.2, 5.0), sharex=True, gridspec_kw={"height_ratios": [1.35, 1.0]})
    ax1.axvspan(0, 1003, color="#f0f9ff", alpha=0.55, label="PWM (62, 58) ON")
    ax1.axvspan(1003, 1600, color="#fff7ed", alpha=0.55, label="PWM (0, 0) COAST")
    ax1.axvline(1003, color="#6b7280", linestyle="--", linewidth=0.9)
    ax1.axhline(200.0, color="#6b7280", linestyle=":", linewidth=0.9, label="No-load target (200 mm/s)")

    ax1.fill_between(t_grid, vr_mat.mean(0) - vr_mat.std(0, ddof=1), vr_mat.mean(0) + vr_mat.std(0, ddof=1), color="#c75e19", alpha=0.2)
    ax1.plot(t_grid, vr_mat.mean(0), color="#c75e19", linewidth=1.6, label="Right wheel (PWM 58)")
    ax1.fill_between(t_grid, vl_mat.mean(0) - vl_mat.std(0, ddof=1), vl_mat.mean(0) + vl_mat.std(0, ddof=1), color="#1764a0", alpha=0.2)
    ax1.plot(t_grid, vl_mat.mean(0), color="#1764a0", linewidth=1.6, label="Left wheel (PWM 62)")
    ax1.set_ylabel("Wheel speed (mm/s)")
    ax1.set_title("10 ms Step-Up & Coasting Response: Ground PWM (62, 58), n = 5")
    ax1.set_ylim(-5, 225)
    ax1.grid(True, color="#e5e7eb", linewidth=0.6)
    ax1.legend(loc="upper right", fontsize=8, ncol=2)

    ax2.axvline(1003, color="#6b7280", linestyle="--", linewidth=0.9)
    ax2.fill_between(t_grid, th_mat.mean(0) - th_mat.std(0, ddof=1), th_mat.mean(0) + th_mat.std(0, ddof=1), color="#0f766e", alpha=0.2)
    ax2.plot(t_grid, th_mat.mean(0), color="#0f766e", linewidth=1.6, label="Cumulative encoder yaw (L+ / CCW)")
    ax2.set_xlabel("Elapsed time from step command (ms)")
    ax2.set_ylabel("Heading $\\theta$ (deg)")
    ax2.set_xlim(0, 1500)
    ax2.grid(True, color="#e5e7eb", linewidth=0.6)
    ax2.legend(loc="lower right", fontsize=8)

    fig.tight_layout()
    for ext in ("png", "pdf"):
        fig.savefig(OUT_DIR / f"step_response_summary.{ext}", dpi=220)
    plt.close(fig)


if __name__ == "__main__":
    main()
