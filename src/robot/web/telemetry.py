"""Live sensor telemetry for the web panel: the four ToF wall distances and the
IMU heading, read straight off the drivers. Degrades on a dev machine — drivers
that aren't wired up yet report null and flag 'simulated'.
"""

import threading
import time

from robot.drivers.imu import make_imu
from robot.drivers.tof import TofArray


class Telemetry:
    def __init__(self):
        # Don't let one flaky sensor kill the whole panel — show what works.
        self._tof = TofArray(require_all=False)
        # Whichever IMU is on the bus (LSM6DSOX preferred); the panel shows
        # heading=null rather than dying when neither answers.
        try:
            self._imu = make_imu()
        except Exception:
            self._imu = None
        self.simulated = False
        self._heading = None
        if self._imu is not None:
            # Integrate at a steady 50 Hz in the background: heading() uses the
            # wall-clock dt between calls, so leaving it to the dashboard's poll
            # rate would sample the gyro too coarsely during fast rotation.
            self._running = True
            threading.Thread(target=self._imu_loop, daemon=True).start()

    def _imu_loop(self):
        while self._running:
            try:
                self._heading = round(self._imu.heading(), 1)
            except Exception:
                self._heading = None
            time.sleep(0.02)

    def read(self) -> dict:
        tof = {"front": None, "rear": None, "left": None, "right": None}
        try:
            d = self._tof.read_all()
            for k in tof:
                v = d.get(k)
                tof[k] = round(v) if v is not None else None
        except Exception:
            self.simulated = True

        heading = self._heading if self._imu else None
        if heading is None:
            self.simulated = True

        return {"tof": tof, "imu_heading": heading, "simulated": self.simulated}

    def close(self) -> None:
        self._running = False
        for dev in (self._tof, self._imu):
            try:
                if dev is not None:
                    dev.close()
            except Exception:
                pass
