/*
 * Bench test for the steering servo, driven over serial.
 *
 * Lets us jog the servo by angle or by raw pulse width so we can record the
 * real linkage end-stops and centre, then transfer them into
 * src/robot/hardware_config.py (Servo.PULSE_CENTER_US / MIN_US / MAX_US).
 *
 * Wiring (Uno):
 *   - Servo signal -> D9
 *   - Servo GND    -> Uno GND  (and external supply GND -- common ground!)
 *   - Servo V+     -> EXTERNAL 5-6V supply, NOT the Uno 5V pin.
 *     MG996R stall current (~2.5A) will brown out / reset the Uno.
 *     A small SG90 can limp along on 5V but external supply is still safer.
 *
 * Serial protocol (9600 baud, newline-terminated):
 *   <number>     steer to that angle in degrees, 0..180        e.g. "90"
 *   u<number>    write a raw pulse width in microseconds        e.g. "u1500"
 *   c            centre (1500 us)
 *   s            sweep min->max->min once (find buzz/end-stops)
 *   ?            print current state
 * Every command echoes the resulting angle/pulse back.
 */

#include <Servo.h>

const int SERVO_PIN = 9;
const int PULSE_MIN_US = 500;    // attach() limits; wider than a typical servo
const int PULSE_MAX_US = 2500;   // so writeMicroseconds() isn't clamped early

Servo servo;
int currentPulse = 1500;

void applyPulse(int us) {
  us = constrain(us, PULSE_MIN_US, PULSE_MAX_US);
  servo.writeMicroseconds(us);
  currentPulse = us;
}

void report() {
  Serial.print("pulse=");
  Serial.print(currentPulse);
  Serial.print("us  angle~=");
  Serial.println(map(currentPulse, 1000, 2000, 0, 180));
}

void sweep() {
  for (int us = 1000; us <= 2000; us += 10) { applyPulse(us); delay(15); }
  for (int us = 2000; us >= 1000; us -= 10) { applyPulse(us); delay(15); }
  report();
}

void setup() {
  Serial.begin(9600);
  servo.attach(SERVO_PIN, PULSE_MIN_US, PULSE_MAX_US);
  applyPulse(1500);
  Serial.println("servo_test ready. cmds: <deg> | u<us> | c | s | ?");
  report();
}

void loop() {
  if (!Serial.available()) return;
  String line = Serial.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  char cmd = line.charAt(0);
  if (cmd == 'c') {
    applyPulse(1500);
  } else if (cmd == 's') {
    sweep();
    return;
  } else if (cmd == '?') {
    report();
    return;
  } else if (cmd == 'u') {
    applyPulse(line.substring(1).toInt());
  } else {
    int angle = constrain(line.toInt(), 0, 180);
    servo.write(angle);
    currentPulse = map(angle, 0, 180, 1000, 2000);
  }
  report();
}
