#!/usr/bin/env python3
"""Offline motion-parameter calculator for StampC3 and DigitalTwin.

Directly reads the fitted encoder counts/s vs PWM linear models from:
  - data/forward_steady/fit.json
  - data/reverse_steady/fit.json

Model: |counts/s| = k * (|PWM| - p0)
Wheel rim speed: v (mm/s) = (counts/s) * (2 * pi * r / cpr)

Changing --r (wheel radius) or --B (wheel separation) automatically updates the
derived PWM <-> mm/s linear relationship without refitting.

Examples:
  python3 src/calc_motion_params.py arc --radius 200 --angle 90 --speed 180
  python3 src/calc_motion_params.py arc --radius 200 --angle -90 --speed 180 --r 16.2
  python3 src/calc_motion_params.py straight --distance 350 --speed 180
  python3 src/calc_motion_params.py turn --angle 90 --wheel-speed 112.2
  python3 src/calc_motion_params.py all --r 16.0 --B 90.0
"""

from __future__ import annotations

import argparse
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Tuple

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_FWD_FIT = REPO_ROOT / "data" / "forward_steady" / "fit.json"
DEFAULT_REV_FIT = REPO_ROOT / "data" / "reverse_steady" / "fit.json"
DEFAULT_STEP_RESULTS = REPO_ROOT / "output" / "step_response_shared" / "results.json"


@dataclass(frozen=True)
class WheelFit:
    k_per_pwm: float
    dead_zone_p0: float
    pwm_min: float
    pwm_max: float


@dataclass(frozen=True)
class RobotModel:
    fwd_left: WheelFit
    fwd_right: WheelFit
    rev_left: WheelFit
    rev_right: WheelFit
    r_left_mm: float
    r_right_mm: float
    wheel_separation_mm: float
    counts_per_rev: float
    dt_ms: int

    @property
    def mm_per_count_left(self) -> float:
        return 2.0 * math.pi * self.r_left_mm / self.counts_per_rev

    @property
    def mm_per_count_right(self) -> float:
        return 2.0 * math.pi * self.r_right_mm / self.counts_per_rev

    def mm_per_count(self, wheel: str) -> float:
        return self.mm_per_count_left if wheel == "left" else self.mm_per_count_right

    def fit_for(self, wheel: str, forward: bool) -> WheelFit:
        if wheel == "left":
            return self.fwd_left if forward else self.rev_left
        return self.fwd_right if forward else self.rev_right

    def pwm_to_counts_per_sec(self, wheel: str, pwm: float) -> float:
        if pwm > 0:
            fit = self.fit_for(wheel, True)
            return max(0.0, fit.k_per_pwm * (pwm - fit.dead_zone_p0))
        if pwm < 0:
            fit = self.fit_for(wheel, False)
            return -max(0.0, fit.k_per_pwm * (-pwm - fit.dead_zone_p0))
        return 0.0

    def pwm_to_mm_per_sec(self, wheel: str, pwm: float) -> float:
        return self.pwm_to_counts_per_sec(wheel, pwm) * self.mm_per_count(wheel)

    def mm_per_sec_to_pwm(self, wheel: str, speed_mm_s: float) -> float:
        if speed_mm_s == 0.0:
            return 0.0
        forward = speed_mm_s > 0.0
        fit = self.fit_for(wheel, forward)
        counts_per_sec = abs(speed_mm_s) / self.mm_per_count(wheel)
        pwm_mag = counts_per_sec / fit.k_per_pwm + fit.dead_zone_p0
        return pwm_mag if forward else -pwm_mag

    def check_pwm_range(self, wheel: str, pwm: int) -> str | None:
        if pwm == 0:
            return None
        fit = self.fit_for(wheel, pwm > 0)
        mag = abs(pwm)
        if mag < fit.pwm_min or mag > fit.pwm_max:
            direction = "正转" if pwm > 0 else "反转"
            return (
                f"警告: {wheel}轮{direction} |PWM|={mag} 超出拟合实测区间 "
                f"[{fit.pwm_min:.0f}, {fit.pwm_max:.0f}]"
            )
        return None


def load_wheel_fit(data: dict, wheel: str) -> WheelFit:
    entry = data["fits"][wheel]
    pwm_min, pwm_max = entry["pwm_range_measured"]
    return WheelFit(
        k_per_pwm=float(entry["k_per_pwm"]),
        dead_zone_p0=float(entry["dead_zone_p0"]),
        pwm_min=float(pwm_min),
        pwm_max=float(pwm_max),
    )


def load_model(
    fwd_path: Path,
    rev_path: Path,
    r_left_mm: float,
    r_right_mm: float,
    wheel_separation_mm: float,
    counts_per_rev: float,
    dt_ms: int,
) -> RobotModel:
    fwd = json.loads(fwd_path.read_text(encoding="utf-8"))
    rev = json.loads(rev_path.read_text(encoding="utf-8"))
    return RobotModel(
        fwd_left=load_wheel_fit(fwd, "left"),
        fwd_right=load_wheel_fit(fwd, "right"),
        rev_left=load_wheel_fit(rev, "left"),
        rev_right=load_wheel_fit(rev, "right"),
        r_left_mm=r_left_mm,
        r_right_mm=r_right_mm,
        wheel_separation_mm=wheel_separation_mm,
        counts_per_rev=counts_per_rev,
        dt_ms=dt_ms,
    )


def align_duration_ms(duration_ms: float, dt_ms: int) -> int:
    return max(dt_ms, (round(duration_ms / dt_ms)) * dt_ms)


def simulate_timed_pwm(
    model: RobotModel, left_pwm: int, right_pwm: int, duration_ms: int
) -> tuple[float, float, float, int]:
    """Matches DigitalTwin 10 ms Euler integration (Controller.pde + OdometryModel.pde)."""
    x = 0.0
    y = 0.0
    theta = 0.0
    vl_mm_ms = model.pwm_to_mm_per_sec("left", left_pwm) / 1000.0
    vr_mm_ms = model.pwm_to_mm_per_sec("right", right_pwm) / 1000.0
    steps = math.ceil(duration_ms / model.dt_ms)
    for _ in range(steps):
        dl = vl_mm_ms * model.dt_ms
        dr = vr_mm_ms * model.dt_ms
        d = 0.5 * (dl + dr)
        dtheta = (dr - dl) / model.wheel_separation_mm
        x += d * math.cos(theta)
        y += d * math.sin(theta)
        theta += dtheta
    return x, y, math.degrees(theta), steps * model.dt_ms


def print_derived_linear_model(model: RobotModel) -> None:
    print("=== 当前几何参数与派生 PWM–轮速线性关系 ===")
    print(
        f"r_L = {model.r_left_mm:.3f} mm, r_R = {model.r_right_mm:.3f} mm, "
        f"B = {model.wheel_separation_mm:.2f} mm, CPR = {model.counts_per_rev:.1f}, "
        f"dt = {model.dt_ms} ms"
    )
    for label, wheel, fwd in [
        ("左轮正转", "left", True),
        ("右轮正转", "right", True),
        ("左轮反转", "left", False),
        ("右轮反转", "right", False),
    ]:
        fit = model.fit_for(wheel, fwd)
        mm_cnt = model.mm_per_count(wheel)
        slope = fit.k_per_pwm * mm_cnt
        intercept = -slope * fit.dead_zone_p0
        print(
            f"  {label}: |v| = {slope:.4f} * (|PWM| - {fit.dead_zone_p0:.2f}) "
            f"= {slope:.4f} * |PWM| {intercept:+.4f} mm/s  "
            f"(counts/s: {fit.k_per_pwm:.4f} * (|PWM| - {fit.dead_zone_p0:.2f}), "
            f"实测 |PWM| {fit.pwm_min:.0f}–{fit.pwm_max:.0f})"
        )
    print()


def compute_arc(
    model: RobotModel, radius_mm: float, angle_deg: float, center_speed_mm_s: float
) -> dict:
    if radius_mm <= 0 or center_speed_mm_s <= 0 or angle_deg == 0:
        raise ValueError("radius 和 speed 必须为正数，angle 不能为 0。")

    angle_rad = math.radians(angle_deg)
    sign = 1.0 if angle_deg > 0 else -1.0  # +1 左弧, -1 右弧
    ratio = sign * model.wheel_separation_mm / (2.0 * radius_mm)
    v_left_target = center_speed_mm_s * (1.0 - ratio)
    v_right_target = center_speed_mm_s * (1.0 + ratio)

    pwm_left_f = model.mm_per_sec_to_pwm("left", v_left_target)
    pwm_right_f = model.mm_per_sec_to_pwm("right", v_right_target)
    pwm_left = int(round(pwm_left_f))
    pwm_right = int(round(pwm_right_f))

    v_left_act = model.pwm_to_mm_per_sec("left", pwm_left)
    v_right_act = model.pwm_to_mm_per_sec("right", pwm_right)
    v_center_act = 0.5 * (v_left_act + v_right_act)
    omega_act = (v_right_act - v_left_act) / model.wheel_separation_mm
    if abs(omega_act) < 1e-9:
        raise ValueError("取整后的左右轮速相等，无法形成圆弧，请增大速度或减小半径。")

    radius_act = abs(v_center_act / omega_act)
    duration_exact_ms = abs(angle_rad / omega_act) * 1000.0
    duration_ms = align_duration_ms(duration_exact_ms, model.dt_ms)

    # Ideal endpoint from (0, 0, 0)
    ideal_x = radius_mm * math.sin(abs(angle_rad))
    ideal_y = sign * radius_mm * (1.0 - math.cos(abs(angle_rad)))
    sim_x, sim_y, sim_deg, active_ms = simulate_timed_pwm(
        model, pwm_left, pwm_right, duration_ms
    )

    warnings = [
        w
        for w in [
            model.check_pwm_range("left", pwm_left),
            model.check_pwm_range("right", pwm_right),
        ]
        if w is not None
    ]
    return {
        "name": f"ARC_{'LEFT' if angle_deg > 0 else 'RIGHT'}",
        "radius_target": radius_mm,
        "angle_target": angle_deg,
        "speed_target": center_speed_mm_s,
        "v_left_target": v_left_target,
        "v_right_target": v_right_target,
        "pwm_left_f": pwm_left_f,
        "pwm_right_f": pwm_right_f,
        "pwm_left": pwm_left,
        "pwm_right": pwm_right,
        "v_left_act": v_left_act,
        "v_right_act": v_right_act,
        "v_center_act": v_center_act,
        "radius_act": radius_act,
        "duration_exact_ms": duration_exact_ms,
        "duration_ms": duration_ms,
        "active_ms": active_ms,
        "ideal_pose": (ideal_x, ideal_y, angle_deg),
        "sim_pose": (sim_x, sim_y, sim_deg),
        "pos_err_mm": math.hypot(sim_x - ideal_x, sim_y - ideal_y),
        "warnings": warnings,
    }


def compute_straight(model: RobotModel, distance_mm: float, speed_mm_s: float) -> dict:
    if distance_mm == 0 or speed_mm_s <= 0:
        raise ValueError("distance 不能为 0，speed 必须为正数。")

    signed_speed = speed_mm_s if distance_mm > 0 else -speed_mm_s
    pwm_left_f = model.mm_per_sec_to_pwm("left", signed_speed)
    pwm_right_f = model.mm_per_sec_to_pwm("right", signed_speed)
    pwm_left = int(round(pwm_left_f))
    pwm_right = int(round(pwm_right_f))

    v_left_act = model.pwm_to_mm_per_sec("left", pwm_left)
    v_right_act = model.pwm_to_mm_per_sec("right", pwm_right)
    v_center_act = 0.5 * (v_left_act + v_right_act)
    duration_exact_ms = abs(distance_mm / v_center_act) * 1000.0
    duration_ms = align_duration_ms(duration_exact_ms, model.dt_ms)

    sim_x, sim_y, sim_deg, active_ms = simulate_timed_pwm(
        model, pwm_left, pwm_right, duration_ms
    )
    warnings = [
        w
        for w in [
            model.check_pwm_range("left", pwm_left),
            model.check_pwm_range("right", pwm_right),
        ]
        if w is not None
    ]
    return {
        "name": "STRAIGHT_FWD" if distance_mm > 0 else "STRAIGHT_REV",
        "distance_target": distance_mm,
        "pwm_left_f": pwm_left_f,
        "pwm_right_f": pwm_right_f,
        "pwm_left": pwm_left,
        "pwm_right": pwm_right,
        "v_left_act": v_left_act,
        "v_right_act": v_right_act,
        "v_center_act": v_center_act,
        "duration_exact_ms": duration_exact_ms,
        "duration_ms": duration_ms,
        "active_ms": active_ms,
        "ideal_pose": (distance_mm, 0.0, 0.0),
        "sim_pose": (sim_x, sim_y, sim_deg),
        "pos_err_mm": math.hypot(sim_x - distance_mm, sim_y),
        "warnings": warnings,
    }


def compute_turn(model: RobotModel, angle_deg: float, wheel_speed_mm_s: float) -> dict:
    if angle_deg == 0 or wheel_speed_mm_s <= 0:
        raise ValueError("angle 不能为 0，wheel_speed 必须为正数。")

    angle_rad = math.radians(angle_deg)
    sign = 1.0 if angle_deg > 0 else -1.0  # +1 左转, -1 右转
    v_left_target = -sign * wheel_speed_mm_s
    v_right_target = sign * wheel_speed_mm_s

    pwm_left_f = model.mm_per_sec_to_pwm("left", v_left_target)
    pwm_right_f = model.mm_per_sec_to_pwm("right", v_right_target)
    pwm_left = int(round(pwm_left_f))
    pwm_right = int(round(pwm_right_f))

    v_left_act = model.pwm_to_mm_per_sec("left", pwm_left)
    v_right_act = model.pwm_to_mm_per_sec("right", pwm_right)
    omega_act = (v_right_act - v_left_act) / model.wheel_separation_mm
    duration_exact_ms = abs(angle_rad / omega_act) * 1000.0
    duration_ms = align_duration_ms(duration_exact_ms, model.dt_ms)

    sim_x, sim_y, sim_deg, active_ms = simulate_timed_pwm(
        model, pwm_left, pwm_right, duration_ms
    )
    warnings = [
        w
        for w in [
            model.check_pwm_range("left", pwm_left),
            model.check_pwm_range("right", pwm_right),
        ]
        if w is not None
    ]
    return {
        "name": "TURN_LEFT" if angle_deg > 0 else "TURN_RIGHT",
        "angle_target": angle_deg,
        "pwm_left_f": pwm_left_f,
        "pwm_right_f": pwm_right_f,
        "pwm_left": pwm_left,
        "pwm_right": pwm_right,
        "v_left_act": v_left_act,
        "v_right_act": v_right_act,
        "duration_exact_ms": duration_exact_ms,
        "duration_ms": duration_ms,
        "active_ms": active_ms,
        "ideal_pose": (0.0, 0.0, angle_deg),
        "sim_pose": (sim_x, sim_y, sim_deg),
        "pos_err_mm": math.hypot(sim_x, sim_y),
        "warnings": warnings,
    }


def print_result(res: dict) -> None:
    ix, iy, ith = res["ideal_pose"]
    sx, sy, sth = res["sim_pose"]
    print(f"[{res['name']}]")
    if "radius_target" in res:
        print(
            f"  目标: R = {res['radius_target']:.1f} mm, 转角 = {res['angle_target']:+.1f}°, "
            f"中心速度 = {res['speed_target']:.1f} mm/s"
        )
        print(
            f"  目标轮速: v_L = {res['v_left_target']:.2f} mm/s, "
            f"v_R = {res['v_right_target']:.2f} mm/s"
        )
        print(
            f"  取整后实际: R_act = {res['radius_act']:.2f} mm, "
            f"v_center = {res['v_center_act']:.2f} mm/s "
            f"(v_L = {res['v_left_act']:.2f}, v_R = {res['v_right_act']:.2f} mm/s)"
        )
    else:
        print(
            f"  取整后轮速: v_L = {res['v_left_act']:.2f} mm/s, "
            f"v_R = {res['v_right_act']:.2f} mm/s"
        )
    print(
        f"  PWM: 理论=({res['pwm_left_f']:.2f}, {res['pwm_right_f']:.2f}) -> "
        f"整数=({res['pwm_left']}, {res['pwm_right']})"
    )
    print(
        f"  时长: 理论={res['duration_exact_ms']:.1f} ms -> "
        f"10ms对齐={res['duration_ms']} ms"
    )
    print(
        f"  10ms仿真终点: ({sx:.2f}, {sy:.2f}, {sth:+.2f}°)  "
        f"| 理想终点: ({ix:.2f}, {iy:.2f}, {ith:+.2f}°)  "
        f"| 位置差={res['pos_err_mm']:.2f} mm, 角度差={sth - ith:+.2f}°"
    )
    print(
        f"  Controller调用: runTimedPwm(robot, now, "
        f"{res['pwm_left']}, {res['pwm_right']}, {res['duration_ms']});"
    )
    for warning in res["warnings"]:
        print(f"  !! {warning}")
    print()

def print_step_compensation(res: dict, model: RobotModel, step_path: Path) -> None:
    """在本动作取整后的时长上补阶跃响应的净动态损失 net_lag_ms（与 r、B 无关）。

    转角补偿 dt = (vR*dR - vL*dL) / (vR - vL)，路程补偿 dt = (vR*dR + vL*dL) / (vR + vL)，
    vL、vR 为带符号的稳态 counts/s，dL、dR 为左右轮净损失 (ms)。
    原地转只用转角补偿，直线只用路程补偿，圆弧两者不同，按要保证的量选一个。
    """
    step = json.loads(step_path.read_text(encoding="utf-8"))
    dl = step["pooled"]["wheels"]["L"]["net_lag_ms"]["mean"]
    dr = step["pooled"]["wheels"]["R"]["net_lag_ms"]["mean"]
    vl = model.pwm_to_counts_per_sec("left", res["pwm_left"])
    vr = model.pwm_to_counts_per_sec("right", res["pwm_right"])
    mean_speed = 0.5 * (abs(vl) + abs(vr))
    print(f"  阶跃补偿（净损失 左 {dl:.1f} ms, 右 {dr:.1f} ms）:")
    # 两轮速差太小（直线）不算转角补偿，两轮速和太小（原地转）不算路程补偿
    if abs(vr - vl) > 0.05 * mean_speed:
        dt = (vr * dr - vl * dl) / (vr - vl)
        total = align_duration_ms(res["duration_ms"] + dt, model.dt_ms)
        print(
            f"    转角补偿 +{dt:.1f} ms -> runTimedPwm(robot, now, "
            f"{res['pwm_left']}, {res['pwm_right']}, {total});"
        )
    if abs(vr + vl) > 0.05 * mean_speed:
        dt = (vr * dr + vl * dl) / (vr + vl)
        total = align_duration_ms(res["duration_ms"] + dt, model.dt_ms)
        print(
            f"    路程补偿 +{dt:.1f} ms -> runTimedPwm(robot, now, "
            f"{res['pwm_left']}, {res['pwm_right']}, {total});"
        )
    print()


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--r", type=float, default=16.36, help="有效轮径 r (mm)，默认 16.36"
    )
    common.add_argument(
        "--r-left", type=float, default=None, help="可选：左轮独立半径 (mm)"
    )
    common.add_argument(
        "--r-right", type=float, default=None, help="可选：右轮独立半径 (mm)"
    )
    common.add_argument(
        "--B", type=float, default=85.8, help="有效轮距 B (mm)，默认 85.8"
    )
    common.add_argument(
        "--cpr", type=float, default=358.3, help="编码器每圈脉冲数，默认 358.3"
    )
    common.add_argument("--dt", type=int, default=10, help="控制周期 (ms)，默认 10")
    common.add_argument(
        "--forward-fit", type=Path, default=DEFAULT_FWD_FIT, help="正转 fit.json 路径"
    )
    common.add_argument(
        "--reverse-fit", type=Path, default=DEFAULT_REV_FIT, help="反转 fit.json 路径"
    )
    common.add_argument(
        "--step-comp", action="store_true", help="按阶跃响应净动态损失补偿时长"
    )
    common.add_argument(
        "--step-results",
        type=Path,
        default=DEFAULT_STEP_RESULTS,
        help="阶跃响应 results.json 路径",
    )

    parser = argparse.ArgumentParser(
        description="根据 data/ 中 counts/s–PWM 拟合与几何参数计算 runTimedPwm 输入",
        parents=[common],
    )
    sub = parser.add_subparsers(dest="cmd")

    p_arc = sub.add_parser(
        "arc", parents=[common], help="计算圆弧动作的 PWM 与持续时间"
    )
    p_arc.add_argument(
        "--radius", type=float, default=200.0, help="圆弧半径 R (mm)，默认 200"
    )
    p_arc.add_argument(
        "--angle",
        type=float,
        default=90.0,
        help="转角 (度)，正为左弧、负为右弧，默认 90",
    )
    p_arc.add_argument(
        "--speed", type=float, default=180.0, help="目标中心线速度 (mm/s)，默认 180"
    )

    p_str = sub.add_parser(
        "straight", parents=[common], help="计算直行/倒车动作的 PWM 与持续时间"
    )
    p_str.add_argument(
        "--distance",
        type=float,
        default=350.0,
        help="目标距离 (mm)，正前进、负倒车，默认 350",
    )
    p_str.add_argument(
        "--speed", type=float, default=180.0, help="目标线速度幅值 (mm/s)，默认 180"
    )

    p_turn = sub.add_parser(
        "turn", parents=[common], help="计算原地旋转动作的 PWM 与持续时间"
    )
    p_turn.add_argument(
        "--angle", type=float, default=90.0, help="转角 (度)，正左转、负右转，默认 90"
    )
    p_turn.add_argument(
        "--wheel-speed",
        type=float,
        default=112.23,
        help="单轮线速度幅值 (mm/s)，默认 112.23 (~400 counts/s)",
    )

    p_all = sub.add_parser(
        "all", parents=[common], help="一次输出 5 个单动作的 PWM 与持续时间"
    )
    p_all.add_argument(
        "--straight-dist", type=float, default=350.0, help="直行距离 (mm)，默认 350"
    )
    p_all.add_argument(
        "--straight-speed", type=float, default=180.0, help="直行速度 (mm/s)，默认 180"
    )
    p_all.add_argument(
        "--turn-wheel-speed",
        type=float,
        default=112.23,
        help="原地旋转单轮速 (mm/s)，默认 112.23",
    )
    p_all.add_argument(
        "--arc-radius", type=float, default=200.0, help="圆弧半径 (mm)，默认 200"
    )
    p_all.add_argument(
        "--arc-speed", type=float, default=182.0, help="圆弧中心速度 (mm/s)，默认 182"
    )

    return parser


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

    print_derived_linear_model(model)

    def show(res: dict) -> None:
        print_result(res)
        if args.step_comp:
            print_step_compensation(res, model, args.step_results)

    cmd = args.cmd or "all"
    if cmd == "arc":
        show(compute_arc(model, args.radius, args.angle, args.speed))
    elif cmd == "straight":
        show(compute_straight(model, args.distance, args.speed))
    elif cmd == "turn":
        show(compute_turn(model, args.angle, args.wheel_speed))
    else:
        straight_dist = getattr(args, "straight_dist", 350.0)
        straight_speed = getattr(args, "straight_speed", 180.0)
        turn_ws = getattr(args, "turn_wheel_speed", 112.23)
        arc_radius = getattr(args, "arc_radius", 200.0)
        arc_speed = getattr(args, "arc_speed", 182.0)

        results = [
            compute_straight(model, straight_dist, straight_speed),
            compute_turn(model, 90.0, turn_ws),
            compute_turn(model, -90.0, turn_ws),
            compute_arc(model, arc_radius, 90.0, arc_speed),
            compute_arc(model, arc_radius, -90.0, arc_speed),
        ]
        for res in results:
            show(res)


if __name__ == "__main__":
    main()
