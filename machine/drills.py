"""drills.py: autonomous drill runner for the tennis ball machine (Raspberry Pi).

    python3 drills.py list
    python3 drills.py plan baseline_depth                 # print settings, no hardware
    python3 drills.py run  baseline_depth --shots 10
    python3 drills.py run  random_mix --shots 12 --dry    # walk through without motors

Court frame: machine at the baseline centre hash. x is lateral metres (right is
positive), y is depth metres toward the far baseline (23.77 is the far baseline).
"""
import argparse, math, random, sys, threading
from time import sleep, time
from shotmodel import Params, plan_shot, COURT_LEN, HALF_WIDTH, SERVICE_Y

P = Params()

# ---------------- hardware constants (match the wiring table) ----------------
PIN_PULSE, PIN_DIR = 17, 27
PIN_BOT_FWD, PIN_BOT_REV = 23, 24      # bottom wheel on RPWM
PIN_TOP_FWD, PIN_TOP_REV = 26, 21      # top wheel on LPWM (opposite direction)
PIN_ACT_RPWM, PIN_ACT_LPWM = 6, 5      # LPWM extends (tilt up), RPWM retracts
STEPS_PER_DEG = 200 * 16 / 360         # TB6600 at 16x microstepping, direct drive
RIGHT_DIR = 1                          # flip if right and left are swapped
STEP_DELAY = 0.003
ACT_SPEED = 0.78
TILT_TOL_DEG = 0.7
SPIN_UP_S = 2.5                        # time for the wheels to settle
FEED_WAIT_S = 6.0                      # manual feed window, see feed_ball()
V_MIN, V_MAX = 0.285, 0.610            # potentiometer volts at 0 and 45 degrees

# ---------------- drill library ----------------
# (x m, y m, net margin m, topspin rpm; negative rpm is backspin)
DRILLS = {
    "baseline_depth": {
        "about": "Deep and central, the standard rally ball",
        "shots": [(0.0, 21.5, 0.9, 1200)]},
    "crosscourt": {
        "about": "Alternate deep corners to move the player side to side",
        "shots": [(-3.4, 21.0, 0.8, 1200), (3.4, 21.0, 0.8, 1200)]},
    "angle_pass": {
        "about": "Wide, sharp angles near the singles sidelines",
        "shots": [(-3.9, 19.0, 0.6, 900), (3.9, 19.0, 0.6, 900)]},
    "short_ball": {
        "about": "Short sliced balls to pull the player to the net",
        "shots": [(-1.5, 14.5, 0.4, -700), (1.5, 14.5, 0.4, -700)]},
    "service_box": {
        "about": "Medium pace into the deuce and ad service boxes",
        "shots": [(-2.0, 16.8, 0.6, 700), (2.0, 16.8, 0.6, 700)]},
    "lob_defense": {
        "about": "High deep lobs to practice overheads",
        "shots": [(0.0, 20.5, 3.0, 400)]},
    "depth_ladder": {
        "about": "Short to deep in four steps, repeating",
        "shots": [(0.0, 14.0, 0.5, 800), (0.0, 17.0, 0.6, 1000),
                  (0.0, 20.0, 0.8, 1200), (0.0, 22.5, 1.0, 1300)]},
}
DRILLS["random_mix"] = {
    "about": "Seeded random choice across every other drill",
    "shots": [s for k, d in DRILLS.items() for s in d["shots"]]}


def plan_drill(name, count, seed=7):
    """Resolve the shot list into machine settings. Solved once per target."""
    base, cache, plan = DRILLS[name]["shots"], {}, []
    rng = random.Random(seed)
    for i in range(count):
        t = rng.choice(base) if name == "random_mix" else base[i % len(base)]
        if t not in cache:
            cache[t] = plan_shot(*t, P)
        plan.append(cache[t])
    return plan


def check(plan):
    bad = []
    for i, s in enumerate(plan):
        if not s["ok"]:
            bad.append((i, s["why"]))
        elif not s["pwm_ok"]:
            bad.append((i, "wheel command outside 0 to 255, ask for less spin or pace"))
        elif abs(s["x"]) > HALF_WIDTH or not 0 < s["y"] < COURT_LEN:
            bad.append((i, "target outside the singles court"))
    return bad


def show(plan):
    print(" #  target(x,y)    yaw   tilt  act_in  top/bot PWM  v0 m/s   rpm  net+  apex")
    for i, s in enumerate(plan, 1):
        if not s["ok"]:
            print("%2d  (%.1f, %.1f)  not solvable: %s" % (i, s["x"], s["y"], s["why"]))
            continue
        print("%2d  (%+.1f,%5.1f) %+5.1f %5.1f  %6.1f  %3.0f / %3.0f  %6.1f  %4.0f  %.2f  %.2f" % (
            i, s["x"], s["y"], s["yaw"], s["theta"], s["act_input"], s["top"], s["bot"],
            s["v0"], s["rpm"], s["net_clear"], s["apex"]))


# ---------------- hardware layer ----------------
class Machine:
    def __init__(self):
        from gpiozero import PWMOutputDevice as PWM, DigitalOutputDevice as DIG
        import board, busio
        import adafruit_ads1x15.ads1115 as ADS
        from adafruit_ads1x15.analog_in import AnalogIn
        self.pul, self.dr = DIG(PIN_PULSE), DIG(PIN_DIR)
        self.bot_f, self.bot_r = PWM(PIN_BOT_FWD), PWM(PIN_BOT_REV)
        self.top_f, self.top_r = PWM(PIN_TOP_FWD), PWM(PIN_TOP_REV)
        self.a_r, self.a_l = PWM(PIN_ACT_RPWM), PWM(PIN_ACT_LPWM)
        self.chan = AnalogIn(ADS.ADS1115(busio.I2C(board.SCL, board.SDA)), ADS.P0)
        self.yaw_steps = 0             # open loop yaw relative to the start position

    def tilt_deg(self):
        return (self.chan.voltage - V_MIN) / ((V_MAX - V_MIN) / 45.0)

    def set_tilt(self, target, timeout=15.0):
        t0 = time()
        while time() - t0 < timeout:
            err = target - self.tilt_deg()
            if abs(err) <= TILT_TOL_DEG:
                break
            up = err > 0
            self.a_r.value, self.a_l.value = (0, ACT_SPEED) if up else (ACT_SPEED, 0)
            sleep(0.05 if abs(err) < 3 else 0.15)
            self.a_r.value = self.a_l.value = 0
            sleep(0.05)
        self.a_r.value = self.a_l.value = 0
        return self.tilt_deg()

    def set_yaw(self, deg):
        want = round(deg * STEPS_PER_DEG)
        delta = want - self.yaw_steps
        self.dr.value = RIGHT_DIR if delta > 0 else 1 - RIGHT_DIR
        sleep(0.001)
        for _ in range(abs(delta)):
            self.pul.on(); sleep(STEP_DELAY); self.pul.off(); sleep(STEP_DELAY)
        self.yaw_steps = want

    def set_wheels(self, top_pwm, bot_pwm):
        self.bot_r.value = self.top_f.value = 0
        self.bot_f.value = max(0.0, min(1.0, bot_pwm / 255))
        self.top_r.value = max(0.0, min(1.0, top_pwm / 255))

    def stop(self):
        for d in (self.bot_f, self.bot_r, self.top_f, self.top_r, self.a_r, self.a_l):
            d.value = 0


def feed_ball():
    """Placeholder: this build has no powered feeder. Replace with the feeder
    hardware call; until then a person drops a ball into the wheels."""
    print("   FEED NOW (%.0f s window)" % FEED_WAIT_S)
    sleep(FEED_WAIT_S)


def run(name, count, dry):
    plan = plan_drill(name, count)
    problems = check(plan)
    show(plan)
    if problems:
        print("Refusing to run:", problems)
        return 1
    if dry:
        print("dry run complete, no motors moved")
        return 0
    m = Machine()
    try:
        for n in (3, 2, 1):
            print(n); sleep(1)
        for i, s in enumerate(plan, 1):
            print("shot %d of %d" % (i, len(plan)))
            workers = [threading.Thread(target=m.set_yaw, args=(s["yaw"],)),
                       threading.Thread(target=m.set_tilt, args=(s["theta"],))]
            m.set_wheels(s["top"], s["bot"])
            for w in workers: w.start()
            for w in workers: w.join()
            sleep(SPIN_UP_S)
            feed_ball()
            m.set_wheels(0, 0)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        m.stop()
        try:
            m.set_yaw(0)
        except Exception:
            pass
    return 0


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("cmd", choices=["list", "plan", "run"])
    ap.add_argument("drill", nargs="?")
    ap.add_argument("--shots", type=int, default=8)
    ap.add_argument("--dry", action="store_true")
    a = ap.parse_args()
    if a.cmd == "list":
        for k, d in DRILLS.items():
            print("%-16s %s" % (k, d["about"]))
    elif a.drill not in DRILLS:
        sys.exit("pick a drill: " + ", ".join(DRILLS))
    elif a.cmd == "plan":
        show(plan_drill(a.drill, a.shots))
    else:
        sys.exit(run(a.drill, a.shots, a.dry))
