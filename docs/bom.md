# Bill of Materials & Cost Report

Prices are estimates in Ghanaian cedis (GH₵) at time of purchase; sourcing links are representative searches, not endorsements. Items priced "tbc" were acquired later and their receipts are being collated.

## Current build

| Component | Qty | Est. unit price | Line total | Role |
|---|---:|---:|---:|---|
| [Raspberry Pi 3B+](https://www.aliexpress.com/wholesale?SearchText=raspberry+pi+3b%2B) | 1 | GH₵1,200.00 | GH₵1,200.00 | Main computer — control loop, vision, logging |
| [OV5647 camera, 160° wide-angle](https://www.aliexpress.com/wholesale?SearchText=OV5647+raspberry+pi+camera+160+degree) | 1 | GH₵108.00 | GH₵108.00 | Pillar detection (obstacle challenge), heading fusion |
| [N20 DC gearmotor, 12 V](https://www.aliexpress.com/wholesale?SearchText=n20+dc+gear+motor+12v) | 1 | GH₵108.00 | GH₵108.00 | Rear drive |
| [TB6612FNG motor driver](https://www.aliexpress.com/wholesale?SearchText=TB6612FNG+motor+driver) | 1 | GH₵72.00 | GH₵72.00 | H-bridge for the drive motor |
| [VL53L1X ToF sensor](https://www.aliexpress.com/w/wholesale-vl53l1x-tof-sensor.html) | 4 | GH₵48.00 | GH₵192.00 | Wall ranging (front/left/right/rear) |
| [XL4015 step-down converter](https://www.aliexpress.com/wholesale?SearchText=XL4015+step+down) | 1 | GH₵36.00 | GH₵36.00 | 5 V rail regulation |
| [SG90 servo (180°)](https://www.aliexpress.com/wholesale?SearchText=SG90+servo+motor) | 1 | GH₵30.00 | GH₵30.00 | Steering actuation |
| [TCS34725/27 color sensor](https://www.aliexpress.com/wholesale?SearchText=TCS34725+color+sensor) | 1 | GH₵26.00 | GH₵26.00 | Mat corner-line detection |
| [Power switch](https://www.aliexpress.com/wholesale?SearchText=rocker+switch) | 1 | GH₵12.00 | GH₵12.00 | Battery master switch |
| [Push button](https://www.aliexpress.com/wholesale?SearchText=momentary+push+button) | 1 | GH₵7.00 | GH₵7.00 | Round-start trigger (rules-required single interaction) |
| [LSM6DSOX IMU](https://www.aliexpress.com/wholesale?SearchText=LSM6DSOX) | 1 | tbc | tbc | Gyro heading (replacement — see below) |
| [2S LiPo pack, 7.4 V 5000 mAh, protected](https://www.aliexpress.com/wholesale?SearchText=2s+lipo+7.4v+5000mah) ([label](images/battery-2s-lipo-label.jpeg)) | 1 | tbc | tbc | Power supply (replacement — see below) |
| Gen-2 chassis (frame, steering, ⌀55 mm wheels) | 1 | tbc | tbc | Rolling chassis, built in-house |
| PLA filament for the printed body | ~0.3 kg | tbc | tbc | Tub + lid, printed on a Bambu Lab P1S |
| **Subtotal (priced items)** | | | **GH₵1,791.00** | |

## Parts bought and retired during development

Real iteration has a bill too. These parts were purchased, used, and replaced — each replacement is documented in the [root README §4](../README.md#4-engineering-decisions-and-lessons):

| Component | Qty | Est. price | Why it was retired |
|---|---:|---:|---|
| "MPU6050" 10-DoF IMU | 1 | GH₵19.00 | Counterfeit — WHO_AM_I answered 0x98; replaced by the LSM6DSOX, and boot-time chip-ID verification added so this can't happen silently again |
| 3× 18650 pack + holder, ~12 V | 1 | GH₵120.00 | Replaced by the protected 2S LiPo brick: more capacity, built-in protection circuit, no spring contacts to bounce loose under vibration |
| LEGO wheels 55981C05 | 4 | GH₵0.00 | Generation-1 drive wheels (on hand); superseded by the gen-2 chassis's ⌀55 mm rubber wheels |
| **Retired subtotal** | | **GH₵139.00** | |

Not itemized: the gen-1 printed chassis/body filament, wiring, headers, standoffs, and M3 hardware (small consumables), and tools (3D printer access, soldering iron, multimeter) which the team did not purchase for this project.
