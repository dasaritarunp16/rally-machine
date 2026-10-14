"""stepper_test.py: one full revolution each way on the TB6600.

GPIO17 is pulse, GPIO27 is direction. At 16x microstepping, 3200 steps is one
revolution. A 0.003 s step delay ran cleanly; 0.0015 s clicked and stalled on reverse.
"""
from gpiozero import DigitalOutputDevice as D
from time import sleep

p, d = D(17), D(27)
for direction in (1, 0):
    d.value = direction
    for _ in range(3200):
        p.on(); sleep(0.003); p.off(); sleep(0.003)
