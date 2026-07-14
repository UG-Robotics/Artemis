"""Live ToF monitor that self-heals: every cycle it retries bring-up of any
missing sensor, so if you fix a wire mid-run the sensor wakes and starts reading
without a restart. Run from the repo src/ dir with the web app stopped (it owns
the I2C bus + XSHUT pins otherwise):

    sudo systemctl stop artemis-web
    cd /home/pi/artemis/src && python3 -m robot.bench.tof_test.tof_live

Ctrl+C to quit.
"""

import time

import board  # type: ignore
import busio  # type: ignore
import RPi.GPIO as GPIO  # type: ignore

from robot.hardware_config import Tof as C
from robot.drivers.tof import _make_sensor, _DEFAULT_ADDRESS, _BOOT_DELAY_S

POSITIONS = ("front", "left", "right", "rear")


def main():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in C.XSHUT_PINS.values():          # hold all in reset to clear 0x29
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
    time.sleep(_BOOT_DELAY_S)
    i2c = busio.I2C(board.SCL, board.SDA)
    sensors = {}

    def wake(pos):
        """Bring one sensor up: boots at 0x29, gets moved to its own address."""
        GPIO.output(C.XSHUT_PINS[pos], GPIO.HIGH)
        time.sleep(_BOOT_DELAY_S)
        try:
            s = _make_sensor(i2c, _DEFAULT_ADDRESS)
            s.set_address(C.I2C_ADDRESSES[pos])
            s.start_ranging()
            sensors[pos] = s
        except Exception:
            GPIO.output(C.XSHUT_PINS[pos], GPIO.LOW)   # back to reset, retry later
            time.sleep(_BOOT_DELAY_S)

    print("live ToF monitor — fix a wire and it comes alive. Ctrl+C to stop.")
    try:
        while True:
            for pos in POSITIONS:                # retry anything not yet alive
                if pos not in sensors:
                    wake(pos)
            cells = []
            for pos in POSITIONS:
                if pos in sensors:
                    try:
                        s = sensors[pos]
                        if s.data_ready:
                            d = s.distance
                            s.clear_interrupt()
                            cells.append("%-5s %s" % (
                                pos, "  --  " if d is None else "%4dmm" % (d * 10)))
                        else:
                            cells.append("%-5s  ..  " % pos)
                    except OSError:              # wire pulled mid-run -> reset + retry
                        del sensors[pos]
                        GPIO.output(C.XSHUT_PINS[pos], GPIO.LOW)
                        cells.append("%-5s DROP " % pos)
                else:
                    cells.append("%-5s DEAD " % pos)
            print(" | ".join(cells), flush=True)
            time.sleep(0.4)
    except KeyboardInterrupt:
        pass
    finally:
        for s in sensors.values():
            try:
                s.stop_ranging()
            except Exception:
                pass
        GPIO.cleanup()


if __name__ == "__main__":
    main()
