"""Background colour-line reader with a short latch.

The control loop runs at CONTROL_HZ (~30 Hz), but a line flashes under the
downward sensor in only tens of milliseconds at cruise — faster than the loop
polls. This thread reads the colour sensor as fast as the (now short)
integration allows and LATCHES a detected line for HOLD_S, so the controller
still sees 'orange'/'blue' even if the line passed between two control ticks.

Owns the one ColorSensor on i2c-3, so the run must not also construct one
(RealHardware(use_color=False)). Off-Pi (no I2C) it stays `available=False`
and `color` stays None.
"""

import threading
import time

from robot.drivers.color import ColorSensor


class ColorWorker:
    RATE_HZ = 80        # read cadence (integration-limited on the chip anyway)
    HOLD_S = 0.15       # hold a detection this long so the 30 Hz loop catches it

    def __init__(self):
        self.color = None        # latched 'orange' | 'blue' | None
        self.available = False
        self._running = False
        try:
            self._cs = ColorSensor()
            self.available = True
        except Exception:
            self._cs = None
        if self.available:
            self._last_seen = 0.0
            self._running = True
            threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        period = 1.0 / self.RATE_HZ
        while self._running:
            t0 = time.monotonic()
            try:
                d = self._cs.detect()
            except Exception:
                d = None
            now = time.monotonic()
            if d in ('orange', 'blue'):
                self.color = d
                self._last_seen = now
            elif now - self._last_seen > self.HOLD_S:
                self.color = None
            dt = time.monotonic() - t0
            if dt < period:
                time.sleep(period - dt)

    def close(self):
        self._running = False
        if self._cs is not None:
            try:
                self._cs.close()
            except Exception:
                pass
