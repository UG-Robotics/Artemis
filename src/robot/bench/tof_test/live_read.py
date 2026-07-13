"""Bench test: bring up all four ToFs via XSHUT readdressing and stream distances.

Run on the Pi:
    python3 live_read.py

Expected: an I2C scan afterwards shows 0x30-0x33 (not 0x29), and each column
prints a sensible mm value when you put a hand/wall in front of that sensor.
"""

import pathlib
import sys
import time

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3]))  # src/

from core.config import TOF_MAX_RANGE
from robot.drivers.tof import TofArray


def main():
    print("bringing up ToF array (XSHUT readdress)...")
    tofs = TofArray(require_all=False)
    print(f"{len(tofs._sensors)}/4 sensors ranging")
    for position, error in tofs.errors.items():
        print(f"  {position}: DEAD ({error})")
    print(f"\nmm readings ('--' = no target in range):")
    print(f"{'front':>8} {'left':>8} {'right':>8} {'rear':>8}")
    try:
        while True:
            r = tofs.read_all()
            cells = []
            for p in TofArray.POSITIONS:
                if p in tofs.errors:
                    cells.append(f"{'DEAD':>8}")
                elif r[p] >= TOF_MAX_RANGE:
                    cells.append(f"{'--':>8}")
                else:
                    cells.append(f"{r[p]:8.0f}")
            print(" ".join(cells), end="\r", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\nstopping")
    finally:
        tofs.close()


if __name__ == "__main__":
    main()
