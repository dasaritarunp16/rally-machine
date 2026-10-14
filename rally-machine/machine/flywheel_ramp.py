"""flywheel_ramp.py: start at full speed, step the wheels down in five stages of
three seconds each, then cut power. The run lasts about 18 seconds."""
from time import sleep
from gpiozero import PWMOutputDevice as P

s1f, s1r, s2f, s2r = P(23), P(24), P(26), P(21)

# Edit these
STEPS = [1.0, 0.8, 0.6, 0.4, 0.2]
SECONDS_PER_STEP = 3


def set_speed(v):
    s1r.value = 0; s2f.value = 0
    s1f.value = v; s2r.value = v


try:
    for n in (3, 2, 1):
        print(n); sleep(1)
    print("GO")
    for v in STEPS:
        set_speed(v)
        print("speed", int(v * 100), "percent")
        sleep(SECONDS_PER_STEP)
finally:
    s1f.value = 0; s1r.value = 0; s2f.value = 0; s2r.value = 0
    print("power off")
