/*
 * TCS34725 floor-colour calibration on an Uno (bench rig before the Pi).
 *
 * Classifies the mat as ORANGE / BLUE / NONE using *normalised* channels
 * (r=R/C, g=G/C, b=B/C) so the numbers transfer to the Pi as long as the
 * mounting height, gain, integration time and lighting are kept the same.
 *
 * Wiring (Uno):  GND->GND  SDA->A4  SCL->A5  VIN->5V (Adafruit breakout)
 *   Integration 50ms, gain 4x -> matches hardware_config.Color.
 *
 * Serial (9600), single-key commands:
 *   o   capture current spot as the ORANGE reference
 *   b   capture current spot as the BLUE reference
 *   w   capture current spot as WHITE / background (= "none")
 *   p   print the three stored references (paste-ready) + thresholds
 *   l   toggle live classification on/off
 * It streams live normalised readings the whole time.
 */

#include <Wire.h>
#include "Adafruit_TCS34725.h"

Adafruit_TCS34725 tcs(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

struct Sample { float r, g, b; bool set; };
Sample ref[3];                       // 0=orange, 1=blue, 2=white
const char *NAMES[3] = {"ORANGE", "BLUE", "WHITE/none"};
bool liveClassify = true;
bool ready = false;                  // hot-detect: keep retrying begin() until found

void readNorm(float &r, float &g, float &b, uint16_t &c) {
  uint16_t rr, gg, bb, cc;
  tcs.getRawData(&rr, &gg, &bb, &cc);
  c = cc;
  if (cc == 0) { r = g = b = 0; return; }
  r = (float)rr / cc;
  g = (float)gg / cc;
  b = (float)bb / cc;
}

void capture(int idx) {
  float ar = 0, ag = 0, ab = 0;
  const int N = 16;
  for (int i = 0; i < N; i++) {
    float r, g, b; uint16_t c;
    readNorm(r, g, b, c);
    ar += r; ag += g; ab += b;
    delay(20);
  }
  ref[idx] = { ar / N, ag / N, ab / N, true };
  Serial.print("captured ");
  Serial.print(NAMES[idx]);
  Serial.print("  r="); Serial.print(ref[idx].r, 3);
  Serial.print(" g=");  Serial.print(ref[idx].g, 3);
  Serial.print(" b=");  Serial.println(ref[idx].b, 3);
}

float dist2(const Sample &s, float r, float g, float b) {
  float dr = s.r - r, dg = s.g - g, db = s.b - b;
  return dr * dr + dg * dg + db * db;   // squared euclidean in normalised space
}

void printRefs() {
  Serial.println("---- stored references (normalised r/g/b) ----");
  for (int i = 0; i < 3; i++) {
    Serial.print("  ");
    Serial.print(NAMES[i]);
    if (!ref[i].set) { Serial.println(": (not captured)"); continue; }
    Serial.print(": ");
    Serial.print(ref[i].r, 3); Serial.print(", ");
    Serial.print(ref[i].g, 3); Serial.print(", ");
    Serial.println(ref[i].b, 3);
  }
  Serial.println("  -> nearest-centroid classify; WHITE means 'none'.");
  Serial.println("----------------------------------------------");
}

void setup() {
  Serial.begin(9600);
  Wire.begin();
  Wire.setWireTimeout(25000, true);   // don't let a stuck bus hang us
  Serial.println("TCS34725 test — Uno (hot-detect; Ctrl+C-safe)");
  ready = tcs.begin();
  if (ready) Serial.println("TCS34725 ready. keys: o=orange b=blue w=white p=print l=live");
}

void loop() {
  if (!ready) {                       // wire it live; comes alive when connected
    ready = tcs.begin();
    if (ready) Serial.println("TCS34725 ready. keys: o=orange b=blue w=white p=print l=live");
    else { Serial.println("  waiting for TCS34725 (check SDA->A4 SCL->A5 VIN GND)..."); delay(600); return; }
  }

  if (Serial.available()) {
    char k = Serial.read();
    if (k == 'o') capture(0);
    else if (k == 'b') capture(1);
    else if (k == 'w') capture(2);
    else if (k == 'p') printRefs();
    else if (k == 'l') { liveClassify = !liveClassify;
                         Serial.println(liveClassify ? "live ON" : "live OFF"); }
  }

  float r, g, b; uint16_t c;
  readNorm(r, g, b, c);

  Serial.print("r="); Serial.print(r, 3);
  Serial.print(" g="); Serial.print(g, 3);
  Serial.print(" b="); Serial.print(b, 3);
  Serial.print(" | C="); Serial.print(c);

  if (liveClassify) {
    int best = -1; float bestD = 1e9;
    for (int i = 0; i < 3; i++) {
      if (!ref[i].set) continue;
      float d = dist2(ref[i], r, g, b);
      if (d < bestD) { bestD = d; best = i; }
    }
    Serial.print(" | ");
    if (best < 0)        Serial.print("(capture refs first)");
    else if (best == 2)  Serial.print("NONE");
    else                 Serial.print(NAMES[best]);
  }
  Serial.println();
  delay(250);
}
