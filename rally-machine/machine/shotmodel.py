"""shotmodel.py: ball flight model and shot solver for the two wheel launcher.

Equations
  v0     = k_v * 0.5 * (R*w_top + R*w_bot)
  spin   = k_s * R*(w_bot - w_top) / (2*r)      positive spin = lift (backspin)
  drag   = -0.5 * rho * c_d * A * |v| * v / m
  magnus =  0.5 * rho * c_l * A * |v| * v_perp / m,   v_perp = (-vy, vx)
  c_l    = k_l * (spin * r / |v|)

Solve: bisect launch speed to hit the range, then bisect launch angle to set
net margin. Pure Python, no scipy, so it runs on the Pi as is.
"""
import math
from dataclasses import dataclass

G = 9.81
M = 0.057            # ball mass, kg
RB = 0.033           # ball radius, m
A = math.pi * RB ** 2
RHO = 1.21           # air density, kg/m3
NET_H = 0.914        # net height at centre, m
NET_Y = 11.89        # net distance from the machine along the court, m
COURT_LEN = 23.77    # singles court length, m
HALF_WIDTH = 4.115   # singles half width, m
SERVICE_Y = 18.29    # far service line distance from the machine, m


@dataclass
class Params:
    k_v: float = 0.90           # speed slip factor, fit from shots
    k_s: float = 0.50           # spin slip factor, fit from shots
    k_l: float = 0.30           # lift coefficient slope, fit from shots
    c_d: float = 0.55           # drag coefficient
    z0: float = 0.5             # launch height, m
    R_w: float = 0.0762         # wheel radius, m
    w_max: float = 650.0        # wheel rad/s at PWM 255 (no load estimate)
    dt: float = 0.002           # integration step, s
    deg_per_unit: float = 0.26  # actuator input units to launch angle, degrees
    # The wheel arm tilts between 75 and 50 degrees from horizontal, so the ball
    # leaves between 15 and 40 degrees above level.
    tilt_min: float = 15.0      # launch angle limits, degrees
    tilt_max: float = 40.0


def fly(v0, theta_deg, spin, p, net_dist=NET_Y):
    """Integrate one flight. Returns (landing range, height at net, apex height)."""
    th = math.radians(theta_deg)
    x, z = 0.0, p.z0
    vx, vz = v0 * math.cos(th), v0 * math.sin(th)
    dt, z_net, apex = p.dt, None, p.z0
    while True:
        sp = math.hypot(vx, vz)
        c_l = p.k_l * (spin * RB / sp)
        k_d = 0.5 * RHO * p.c_d * A * sp / M
        k_m = 0.5 * RHO * c_l * A * sp / M
        vx += (-k_d * vx - k_m * vz) * dt
        vz += (-k_d * vz + k_m * vx - G) * dt
        xn, zn = x + vx * dt, z + vz * dt
        if z_net is None and xn >= net_dist:
            z_net = z + (net_dist - x) / (xn - x) * (zn - z)
        apex = max(apex, zn)
        if zn <= 0.0:
            return x + z / (z - zn) * (xn - x), z_net, apex
        x, z = xn, zn
        if x > 80.0:
            return x, z_net, apex


def wheels_to_shot(top_pwm, bot_pwm, p):
    """PWM commands (0 to 255) to (launch speed m/s, spin rad/s)."""
    w_t, w_b = top_pwm / 255 * p.w_max, bot_pwm / 255 * p.w_max
    return (p.k_v * 0.5 * p.R_w * (w_t + w_b),
            p.k_s * p.R_w * (w_b - w_t) / (2 * RB))


def shot_to_wheels(v0, spin, p):
    """Invert: launch speed and spin to PWM (top, bottom). May leave 0..255."""
    mean = v0 / p.k_v
    diff = spin * 2 * RB / p.k_s
    return ((mean - diff / 2) / p.R_w / p.w_max * 255,
            (mean + diff / 2) / p.R_w / p.w_max * 255)


def speed_for_range(target, theta_deg, spin, p, net_dist=NET_Y):
    """Bisect launch speed so the ball lands at `target` metres."""
    lo, hi = 4.0, 60.0
    for _ in range(30):
        mid = 0.5 * (lo + hi)
        if fly(mid, theta_deg, spin, p, net_dist)[0] < target:
            lo = mid
        else:
            hi = mid
    return 0.5 * (lo + hi)


def solve_shot(target, margin, spin, p, net_dist=NET_Y):
    """Bisect launch angle so the ball clears the net by `margin` m and lands at
    `target` m. If even the flattest angle clears by more than asked, use the
    flattest angle. Returns a dict; ok is False when the limits cannot do it."""
    def err(th):
        v = speed_for_range(target, th, spin, p, net_dist)
        return fly(v, th, spin, p, net_dist)[1] - NET_H - margin

    lo, hi = p.tilt_min, p.tilt_max
    e_lo, e_hi = err(lo), err(hi)
    if e_hi < 0:
        return {"ok": False, "why": "margin not reachable below the tilt limit"}
    if e_lo > 0:
        th = lo                       # flattest shot already clears with room to spare
    else:
        for _ in range(14):
            mid = 0.5 * (lo + hi)
            if err(mid) < 0:
                lo = mid
            else:
                hi = mid
        th = 0.5 * (lo + hi)
    v = speed_for_range(target, th, spin, p, net_dist)
    land, zn, apex = fly(v, th, spin, p, net_dist)
    top, bot = shot_to_wheels(v, spin, p)
    return {"ok": True, "theta": th, "v0": v, "spin": spin, "land": land,
            "net_clear": zn - NET_H, "apex": apex, "top": top, "bot": bot}


def plan_shot(x, y, margin, topspin_rpm, p):
    """Court target (x lateral m, y depth m from the machine) to machine settings.
    topspin_rpm > 0 is topspin, < 0 is backspin."""
    bearing = math.atan2(x, y)
    rng = math.hypot(x, y)
    spin = -topspin_rpm * 2 * math.pi / 60      # model sign: positive spin lifts
    s = solve_shot(rng, margin, spin, p, NET_Y / math.cos(bearing))
    s.update({"x": x, "y": y, "yaw": math.degrees(bearing), "range": rng,
              "rpm": topspin_rpm})
    if s["ok"]:
        s["act_input"] = s["theta"] / p.deg_per_unit
        s["pwm_ok"] = 0 <= s["top"] <= 255 and 0 <= s["bot"] <= 255
    return s
