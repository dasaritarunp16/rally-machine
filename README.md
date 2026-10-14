# Rally Machine

A tennis ball machine that aims. Built by Tarun Dasari, Ziv Respler and Johnson Xiao.

**Project site:** https://dasaritarunp16.github.io/rally-machine/

Most ball machines feed the same ball over and over. We are building one that turns a target on the court into wheel speeds, tilt and rotation, and that will eventually watch where the ball lands and where the opponent stands so it can choose its reply the way a professional would.

## What is in this repo

| Folder | What it holds |
| --- | --- |
| `index.html`, `media/` | The project website, served by GitHub Pages |
| `machine/` | Python that runs on the Raspberry Pi 4: motor control, tilt sensing, the shot solver and autonomous drills |
| `vision/` | Colab notebook testing ball and bounce detection from the machine's own camera position, with court calibration by homography |
| `match-data/` | Colab notebook that cuts broadcast matches into rallies and extracts every shot with player positions |
| `court-model/` | Colab notebook training a court keypoint model |

## Machine code (`machine/`)

| File | Purpose |
| --- | --- |
| `shotmodel.py` | Ball flight model (drag and Magnus lift) and the solver that turns a court target into launch angle, speed and wheel PWM |
| `drills.py` | Plans and runs drills from court targets; checks every shot before any motor moves |
| `demo.py` | Runs the stepper, actuator and flywheels alone or together for testing |
| `tilt_sensor.py` | Reads the actuator potentiometer through an ADS1115 |
| `flywheel_ramp.py` | Steps the wheels down from full speed in five stages |
| `stepper_test.py` | One revolution each way on the TB6600 |
| `motorcontrol.service` | systemd unit that starts the motor script at boot |

```bash
python3 machine/drills.py list
python3 machine/drills.py plan baseline_depth          # no hardware needed
python3 machine/drills.py run random_mix --shots 12 --dry
python3 machine/drills.py run crosscourt --shots 10    # on the Pi, motors run
```

### Hardware

Raspberry Pi 4, two DC flywheel motors on IBT2 (BTS7960) drivers, a 12 V linear actuator with a potentiometer read by an ADS1115 over I2C, a stepper on a TB6600 driver at 16x microstepping, and a 12 V 18 Ah sealed lead acid battery. The wheel arm tilts between 50° and 75° from horizontal, which gives a launch angle of 15° to 40°.

| Function | Physical pin | BCM GPIO |
| --- | --- | --- |
| ADS1115 SDA | 3 | GPIO2 |
| ADS1115 SCL | 5 | GPIO3 |
| Shared ground | 6 | GND |
| Stepper PUL+ | 11 | GPIO17 |
| Stepper DIR+ | 13 | GPIO27 |
| Bottom wheel RPWM / LPWM | 16 / 18 | GPIO23 / GPIO24 |
| Actuator LPWM / RPWM | 29 / 31 | GPIO5 / GPIO6 |
| Top wheel RPWM / LPWM | 37 / 40 | GPIO26 / GPIO21 |

### Pi setup

```bash
sudo apt update && sudo apt install -y python3-pip python3-gpiozero python3-rpi.gpio i2c-tools
sudo raspi-config nonint do_i2c 0
pip3 install adafruit-blinka adafruit-circuitpython-ads1x15 --break-system-packages
```

## Known limits

* **Tilt calibration.** `drills.py` passes the launch angle to `set_tilt`, which compares it against the potentiometer angle of the actuator. The two are not the same scale yet, and `deg_per_unit` assumes 0° at input 0. A measured tilt sweep will replace both.
* **Constants.** The four solver constants were fitted to our first court results and will be refit from 30 to 40 measured shots.
* **Feeding** is by hand during a 6 second window. `feed_ball()` is where a powered feeder will go.
* **Yaw** is open loop from the start position.

## Credits

Ball tracking, court keypoint and bounce detection models in the notebooks come from the open source [TennisProject](https://github.com/yastrebksv/TennisProject). The rally cutting, player assignment, shot extraction, machine view calibration, solver and machine code are our own.
