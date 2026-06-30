"""Live sensor telemetry for the web panel: the four ToF wall distances and the
IMU heading, read straight off the drivers. Degrades on a dev machine — drivers
that aren't wired up yet report null and flag 'simulated'.
"""

from robot.drivers.imu import ImuDriver
from robot.drivers.tof import TofArray


class Telemetry:
    def __init__(self):
        self._tof = TofArray()
        self._imu = ImuDriver()
        self.simulated = False

    def read(self) -> dict:
        tof = {"front": None, "rear": None, "left": None, "right": None}
        try:
            d = self._tof.read_all()
            for k in tof:
                v = d.get(k)
                tof[k] = round(v) if v is not None else None
        except Exception:
            self.simulated = True

        try:
            heading = round(self._imu.heading(), 1)
        except Exception:
            heading = None
            self.simulated = True

        return {"tof": tof, "imu_heading": heading, "simulated": self.simulated}

    def close(self) -> None:
        for dev in (self._tof, self._imu):
            try:
                dev.close()
            except Exception:
                pass
