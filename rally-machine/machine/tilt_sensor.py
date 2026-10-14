"""tilt_sensor.py: read the actuator potentiometer through an ADS1115 and convert to degrees.

0.285 V at fully retracted (0 degrees) and 0.610 V at fully extended (45 degrees).
The library returns volts, not raw counts, so thresholds from the earlier ESP32
version do not carry over.
"""
import board, busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)
chan = AnalogIn(ads, ADS.P0)   # P0 shown; use the channel the potentiometer is wired to

V_MIN = 0.285   # fully retracted, 0 degrees
V_MAX = 0.610   # fully extended, 45 degrees


def tilt_degrees():
    return (chan.voltage - V_MIN) / ((V_MAX - V_MIN) / 45.0)


if __name__ == "__main__":
    from time import sleep
    while True:
        print("%.2f V  %.1f deg" % (chan.voltage, tilt_degrees()))
        sleep(0.5)
