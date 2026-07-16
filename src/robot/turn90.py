"""IMU-referenced 90-degree turn test — the sim's corner turn on real hardware.

    python -m robot.turn90 --dir right --now      # full-lock right, stop at 90°
    python -m robot.turn90 --dir left --speed 0.4

Reproduces WallFollowController's TURNING state: corner speed, full-lock
steering, but closed on the REAL IMU heading instead of the dead-reckoned arc.
Also doubles as the GYRO_Z_SIGN check: a commanded RIGHT turn must make the
heading INCREASE — the script says so explicitly if the sign is backwards.

Safety: aborts on front ToF < ABORT_FRONT_MM, or after TIMEOUT_S.
"""

import argparse
import time

from core.config import CONTROL_HZ, SPEED_OPEN_CORNER, TURN_STEER
from robot.hal import RealHardware

ABORT_FRONT_MM = 150
TIMEOUT_S = 8.0
TARGET_DEG = 90.0


def _unwrap(delta: float) -> float:
    while delta > 180:
        delta -= 360
    while delta < -180:
        delta += 360
    return delta


def run(direction: str, speed: float, countdown: bool = True) -> None:
    hw = RealHardware(use_camera=False, use_imu=True, use_color=False)
    steer_sign = 1 if direction == "right" else -1
    expect_sign = steer_sign  # right turn (positive steer) should RAISE heading

    if countdown:
        for n in (3, 2, 1):
            print(n)
            time.sleep(1.0)
    print(f"TURN {direction.upper()} — full lock {steer_sign * TURN_STEER:+.0f}°, "
          f"speed {speed:.2f}, target {TARGET_DEG:.0f}°")

    hw.imu.reset_heading(0.0)
    h0 = hw.imu.heading()
    dt = 1.0 / CONTROL_HZ
    t0 = time.monotonic()
    swept = 0.0
    prev = h0
    outcome = "TIMEOUT"

    hw.set_steering(steer_sign * TURN_STEER)
    hw.set_speed(speed)
    try:
        while time.monotonic() - t0 < TIMEOUT_S:
            tick = time.monotonic()
            sensors = hw.read_sensors()
            h = sensors.imu_heading
            swept += _unwrap(h - prev)
            prev = h
            print(f"\r  t={tick - t0:4.1f}s heading={h:6.1f}° swept={swept:+7.1f}° "
                  f"front={sensors.tof_front:4.0f}mm", end="")
            if sensors.tof_front < ABORT_FRONT_MM:
                outcome = "ABORT (front wall)"
                break
            if abs(swept) >= TARGET_DEG:
                outcome = "DONE"
                break
            elapsed = time.monotonic() - tick
            if elapsed < dt:
                time.sleep(dt - elapsed)
    except KeyboardInterrupt:
        outcome = "INTERRUPTED"
    finally:
        hw.set_speed(0)
        hw.set_steering(0)
        hw.stop()
        print()

    took = time.monotonic() - t0
    print(f"{outcome}: swept {swept:+.1f}° in {took:.1f}s "
          f"(commanded {direction}, expected sign {'+' if expect_sign > 0 else '-'})")
    if outcome == "DONE" and (swept > 0) != (expect_sign > 0):
        print("!! GYRO_Z_SIGN IS BACKWARDS: a commanded right turn should raise the "
              "heading. Flip ImuLsm.GYRO_Z_SIGN in robot/hardware_config.py.")
    hw.close()


def main() -> None:
    p = argparse.ArgumentParser(description="IMU-referenced 90° turn test.")
    p.add_argument("--dir", choices=("left", "right"), default="right")
    p.add_argument("--speed", type=float, default=SPEED_OPEN_CORNER)
    p.add_argument("--now", action="store_true", help="skip the 3-2-1 countdown")
    args = p.parse_args()
    run(args.dir, args.speed, countdown=not args.now)


if __name__ == "__main__":
    main()
