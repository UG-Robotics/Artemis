"""IMU square-drive test — full-capability check of the LSM6DSOX on the mat.

Drives 4 sides of a ~1m square: drive a side straight (gyro heading-hold),
turn 90 deg on the gyro, repeat. If the turns are true 90s and the sides equal,
the robot returns to where it started — the classic dead-reckoning closure test.
Everything is logged (robot.run_logger) so you can see it after: heading, raw
accelerometer, both distance estimates, and the estimated x/y path.

Two distance sources, ALWAYS both logged:
  time  : commanded_speed * MAX_SPEED integrated over dt (predictable, safe).
  accel : forward-axis accelerometer double-integrated (the "test the IMU
          accelerometer" part) — expected to be rough (gravity tilt + low-speed
          noise); the log shows exactly how rough vs the known 1m sides.

--drive-by picks which one STOPS each side (default: time, so the first run is
safe and the square is geometrically valid). In accel mode a TIME cap
(SQUARE_DIST_CAP_MM) still hard-stops the side so a bad accel estimate can't
drive the robot into a wall — give it >1.5m of clear space per side anyway.

Hot-swaps onto the same start button as open_challenge (robot.start_button):

    python3 -m robot.square_drive                 # button start, drive-by time
    python3 -m robot.square_drive --now           # skip button, 3-2-1
    python3 -m robot.square_drive --drive-by accel --now
    python3 -m robot.square_drive --dry           # calibrate + log 3s still, no motion
    python3 -m robot.square_drive --turns left    # left turns (CCW square)

Stop artemis-web first (it owns the sensors): sudo systemctl stop artemis-web
"""

import argparse
import math
import time

from core.config import (
    MAX_SPEED, SQUARE_DIST_CAP_MM, SQUARE_DRIVE_SPEED, SQUARE_KP_HOLD,
    SQUARE_SIDE_MM, SQUARE_SIDE_TIMEOUT_S, SQUARE_TURN_SPEED, TURN_STEER,
)
from robot.hal import RealHardware
from robot.start_button import arm_start
from robot.run_logger import RunLogger

G = 9.80665  # m/s^2 per g


def _angle_diff(target: float, current: float) -> float:
    """Signed smallest target-current in degrees, in (-180, 180]."""
    d = (target - current + 180) % 360 - 180
    return d + 360 if d <= -180 else d


class AccelOdometer:
    """Forward-axis accelerometer -> velocity -> distance, best-effort.

    6-axis only, so there is no true gravity compensation during motion; we
    subtract the per-axis REST bias (which includes the static gravity
    projection) and integrate the forward axis. Velocity is zeroed at each stop
    (ZUPT) so error doesn't compound across sides. Forward axis is auto-detected
    from the acceleration burst at the start of side 1 unless given.
    """

    def __init__(self, imu, fwd_axis=None):
        self._imu = imu
        self.bias = (0.0, 0.0, 0.0)     # g, per axis at rest
        self.fwd_axis = fwd_axis        # (index, sign) or None until detected
        self.vel = 0.0                  # m/s along forward
        self.dist_mm = 0.0

    def calibrate(self, seconds=1.0):
        n, sx, sy, sz = 0, 0.0, 0.0, 0.0
        t0 = time.monotonic()
        while time.monotonic() - t0 < seconds:
            ax, ay, az = self._imu.accel()
            sx += ax; sy += ay; sz += az; n += 1
            time.sleep(0.005)
        self.bias = (sx / n, sy / n, sz / n)
        return self.bias

    def _dynamic(self):
        ax, ay, az = self._imu.accel()
        return (ax - self.bias[0], ay - self.bias[1], az - self.bias[2])

    def detect_forward(self, samples):
        """samples: list of (dyn_x,dyn_y,dyn_z) taken during a forward burst.
        Forward axis = the one with the largest |mean|; sign from that mean."""
        if not samples:
            self.fwd_axis = (0, 1.0)
            return self.fwd_axis
        means = [sum(s[i] for s in samples) / len(samples) for i in range(3)]
        i = max(range(3), key=lambda k: abs(means[k]))
        self.fwd_axis = (i, 1.0 if means[i] >= 0 else -1.0)
        return self.fwd_axis

    def zupt(self):
        self.vel = 0.0

    def update(self, dt):
        """Integrate one tick; returns (dist_mm, a_fwd_ms2, dyn_tuple)."""
        dyn = self._dynamic()
        if self.fwd_axis is None:
            return self.dist_mm, 0.0, dyn
        idx, sign = self.fwd_axis
        a = dyn[idx] * sign * G
        self.vel += a * dt
        self.dist_mm += self.vel * 1000.0 * dt
        return self.dist_mm, a, dyn


def _steer_hold(hw, heading, target):
    """Heading-hold steering: positive steer raises heading (verified sign)."""
    err = _angle_diff(target, heading)
    hw.set_steering(max(-TURN_STEER, min(TURN_STEER, SQUARE_KP_HOLD * err)))


def run(drive_by="time", turns="right", wait_button=True, log=True, dry=False):
    hw = RealHardware(use_camera=False, use_imu=True, use_color=False)
    imu = hw.imu
    odo = AccelOdometer(imu)
    turn_sign = 1 if turns == "right" else -1   # right turn raises heading

    if not arm_start(wait_button):
        hw.close()
        return
    print("GO")

    logger = RunLogger("square", drive_by,
                       {"side_mm": SQUARE_SIDE_MM, "turns": turns,
                        "drive_by": drive_by}) if log else None
    print("calibrating accel bias (hold still)...")
    imu.reset_heading(0.0)
    odo.calibrate(1.0)
    print(f"  bias(g)={tuple(round(b, 4) for b in odo.bias)}")

    # estimated path (mm), starting at origin heading 0
    x = y = 0.0
    sq = _SquareLog(logger)  # writes square.csv next to the standard streams

    if dry:
        _dry_still(hw, imu, odo, sq, logger)
        hw.stop(); hw.close()
        if logger:
            logger.close({"mode": "dry"})
        return

    dt_target = 1.0 / 100.0   # 100 Hz loop for cleaner accel integration
    fwd_samples = []
    try:
        for side in range(4):
            target_heading = (turn_sign * 90.0 * side) % 360
            # ---- drive the side ----
            odo.dist_mm = 0.0
            odo.zupt()
            time_mm = 0.0
            t_side = time.monotonic()
            last = t_side
            hw.set_speed(SQUARE_DRIVE_SPEED)
            while True:
                now = time.monotonic()
                dt = now - last
                last = now
                heading = imu.heading()
                _steer_hold(hw, heading, target_heading)
                a_dist, a_fwd, dyn = odo.update(dt)
                time_mm += SQUARE_DRIVE_SPEED * MAX_SPEED * dt
                # auto-detect forward axis from side-1's opening burst
                if side == 0 and odo.fwd_axis is None:
                    fwd_samples.append(dyn)
                    if now - t_side > 0.4:
                        print(f"  forward axis: {odo.detect_forward(fwd_samples)}")
                # position estimate uses the chosen distance source
                prim = a_dist if drive_by == "accel" else time_mm
                step = (prim - getattr(run, "_prev_prim", 0.0))
                run._prev_prim = prim
                x += step * math.cos(math.radians(heading))
                y += step * math.sin(math.radians(heading))
                sq.row(logger.t() if logger else 0.0, side, "drive", heading,
                       time_mm, a_dist, a_fwd, dyn, x, y, hw.motor_speed, hw.servo_angle)
                if logger:
                    logger.log_sensors(hw.read_sensors(), hw.servo_angle,
                                       hw.motor_speed, f"side{side}_drive", side)
                reached = (a_dist if drive_by == "accel" else time_mm) >= SQUARE_SIDE_MM
                capped = time_mm >= SQUARE_DIST_CAP_MM
                timed_out = now - t_side > SQUARE_SIDE_TIMEOUT_S
                if reached or capped or timed_out:
                    why = "reached" if reached else ("CAP" if capped else "timeout")
                    print(f"side {side}: {why} time_mm={time_mm:.0f} accel_mm={a_dist:.0f}")
                    break
            run._prev_prim = 0.0
            # ---- turn 90 ----
            _turn_90(hw, imu, turn_sign, target_heading, sq, logger, side, x, y)
    except KeyboardInterrupt:
        print("interrupted")
    finally:
        hw.stop()
        hw.close()
        close_err = math.hypot(x, y)
        print(f"\nESTIMATED closure error: {close_err:.0f} mm from start "
              f"(x={x:.0f}, y={y:.0f}). Eyeball the PHYSICAL gap too.")
        sq.close()
        if logger:
            logger.close({"closure_mm_est": round(close_err, 1),
                          "x_est": round(x, 1), "y_est": round(y, 1)})


def _turn_90(hw, imu, turn_sign, prev_target, sq, logger, side, x, y):
    target = (prev_target + turn_sign * 90.0) % 360
    hw.set_speed(SQUARE_TURN_SPEED)
    hw.set_steering(turn_sign * TURN_STEER)
    t0 = time.monotonic()
    while True:
        heading = imu.heading()
        # done when within 3 deg of the new target, or 4 s safety
        if abs(_angle_diff(target, heading)) < 3.0 or time.monotonic() - t0 > 4.0:
            break
        sq.row(logger.t() if logger else 0.0, side, "turn", heading,
               0, 0, 0, (0, 0, 0), x, y, hw.motor_speed, hw.servo_angle)
        time.sleep(0.005)
    hw.set_steering(0)


def _dry_still(hw, imu, odo, sq, logger):
    print("DRY: logging 3s stationary (heading should hold ~0, accel ~bias)...")
    t0 = time.monotonic()
    last = t0
    while time.monotonic() - t0 < 3.0:
        now = time.monotonic(); dt = now - last; last = now
        heading = imu.heading()
        a_dist, a_fwd, dyn = odo.update(dt)
        sq.row(logger.t() if logger else 0.0, -1, "dry", heading,
               0, a_dist, a_fwd, dyn, 0, 0, 0, 0)
        time.sleep(0.02)
    print(f"  after 3s still: heading={imu.heading():.2f} accel_dist={odo.dist_mm:.0f}mm")


class _SquareLog:
    """Detailed per-tick CSV alongside the standard run streams."""
    HEADER = ("t,side,phase,heading,time_mm,accel_mm,a_fwd_ms2,"
              "dyn_x,dyn_y,dyn_z,x_mm,y_mm,speed,steer\n")

    def __init__(self, logger):
        self._f = None
        if logger is not None:
            import os
            self._f = open(os.path.join(logger.dir, "square.csv"), "w", buffering=1)
            self._f.write(self.HEADER)

    def row(self, t, side, phase, heading, time_mm, accel_mm, a_fwd, dyn,
            x, y, speed, steer):
        if self._f is None:
            return
        self._f.write(f"{t:.3f},{side},{phase},{heading:.2f},{time_mm:.0f},"
                      f"{accel_mm:.0f},{a_fwd:.3f},{dyn[0]:.4f},{dyn[1]:.4f},"
                      f"{dyn[2]:.4f},{x:.0f},{y:.0f},{speed:.2f},{steer:.1f}\n")

    def close(self):
        if self._f is not None:
            self._f.close()


def _web_app_running():
    import socket
    try:
        with socket.create_connection(("127.0.0.1", 8000), timeout=0.5):
            return True
    except OSError:
        return False


def main():
    p = argparse.ArgumentParser(description="IMU square-drive test.")
    p.add_argument("--drive-by", choices=("time", "accel"), default="time",
                   help="which distance source STOPS each side (default time = safe)")
    p.add_argument("--turns", choices=("right", "left"), default="right")
    p.add_argument("--now", action="store_true", help="skip the button; 3-2-1")
    p.add_argument("--no-log", action="store_true")
    p.add_argument("--dry", action="store_true",
                   help="calibrate + log 3s stationary, no motion (plumbing check)")
    args = p.parse_args()

    if _web_app_running():
        p.error("artemis-web is running — it owns the sensors. "
                "Stop it: sudo systemctl stop artemis-web")

    run(drive_by=args.drive_by, turns=args.turns, wait_button=not args.now,
        log=not args.no_log, dry=args.dry)


if __name__ == "__main__":
    main()
