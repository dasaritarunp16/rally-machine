"""demo.py: run each subsystem alone for video capture, or all three together.

    python3 demo.py stepper | actuator | shoot | all

Each run opens with a five second countdown.
"""
import sys, threading
from time import sleep
from gpiozero import PWMOutputDevice as P, DigitalOutputDevice as D

# Tune these
STEP_DELAY = 0.003
SWEEP_STEPS = 180      # about 20 degrees at 16x microstepping
SWEEPS = 2             # repeats of the left to right sweep
RIGHT_DIR = 1          # flip to 0 if left and right are swapped
ACT_TRAVEL_S = 4.0     # seconds for full bottom to top, calibrate
ACT_SPEED = 0.78
SHOOT_HOLD_S = 12      # seconds the wheels stay at full speed

LEFT_DIR = 0 if RIGHT_DIR else 1
pul, dr = D(17), D(27)
s1f, s1r, s2f, s2r = P(23), P(24), P(26), P(21)
arpwm, alpwm = P(6), P(5)


def move(steps, direction):
    dr.value = direction
    sleep(0.001)
    for _ in range(steps):
        pul.on(); sleep(STEP_DELAY); pul.off(); sleep(STEP_DELAY)


def stepper_sweeps():
    for _ in range(SWEEPS):
        move(SWEEP_STEPS, LEFT_DIR)
        move(2 * SWEEP_STEPS, RIGHT_DIR)
        move(SWEEP_STEPS, LEFT_DIR)


def actuator_reset():
    alpwm.value = 0; arpwm.value = ACT_SPEED
    sleep(ACT_TRAVEL_S + 1); arpwm.value = 0


def actuator_up():
    arpwm.value = 0; alpwm.value = ACT_SPEED
    sleep(ACT_TRAVEL_S); alpwm.value = 0


def wheels_on():
    s1r.value = 0; s2f.value = 0
    for i in range(1, 21):
        s1f.value = i / 20; s2r.value = i / 20; sleep(0.1)


def wheels_off():
    s1f.value = 0; s2r.value = 0


def countdown():
    for i in range(5, 0, -1):
        print(i); sleep(1)
    print("GO")


mode = sys.argv[1] if len(sys.argv) > 1 else "all"
try:
    if mode in ("actuator", "all"):
        actuator_reset()
    countdown()
    if mode == "stepper":
        stepper_sweeps()
    elif mode == "actuator":
        actuator_up(); sleep(3)
    elif mode == "shoot":
        wheels_on(); sleep(SHOOT_HOLD_S)
    else:
        wheels_on()
        ts = [threading.Thread(target=stepper_sweeps),
              threading.Thread(target=actuator_up)]
        for t in ts: t.start()
        sleep(SHOOT_HOLD_S)
        for t in ts: t.join()
finally:
    wheels_off(); alpwm.value = 0; arpwm.value = 0
