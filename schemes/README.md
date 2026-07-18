# Schemes — wiring & electromechanical diagrams

![Artemis Wiring Diagram](circuit_image.png)

Full schematic PDF: [`../docs/schematics/`](../docs/schematics) · previous revision (2026-07-03): [`archive/`](archive)

The diagram (updated 2026-07-18 in Cirkit Designer) shows the complete electrical system: the **7.4 V 2S LiPo pack** feeding the TB6612FNG motor driver directly and the XL4015 buck producing the 5 V rail for the Raspberry Pi 3B+, SG90 steering servo, and sensor set — 4× VL53L1X ToF via individual XSHUT lines, LSM6DSOX IMU, TCS34727 color sensor, plus the DC gearmotor, start button (with 10 kΩ pulldown), and power switch. The OV5647 camera connects via the Pi's CSI ribbon and is not part of the GPIO loom.

Notes (pin-level ground truth is always [`../src/robot/hardware_config.py`](../src/robot/hardware_config.py)):
- The battery is drawn with a generic 2S LiPo illustration; the actual pack is a **7.4 V 5000 mAh (37 Wh) "Xtreme 1"** with a built-in dual-protection board — label photo: [`../docs/images/battery-2s-lipo-label.jpeg`](../docs/images/battery-2s-lipo-label.jpeg).
- The servo signal line physically passes through a level-shifter channel (LV1→HV1) that the diagram omits.
- The motor drives from TB6612 **channel A** (channel B failed at chip level — root README §4), and its encoder is **unused**: the control stack is encoder-free.
- The color sensor sits on a dedicated software I2C bus (GPIO20/21) to escape the address clash with the ToF boot sequence.

The 2026-07-03 revision in [`archive/`](archive) documents the earlier build state (MPU6050 before the counterfeit swap, 3×18650 pack, level shifter in the motor path) — kept as part of the iteration record.
