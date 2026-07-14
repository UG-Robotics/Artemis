/*
 * chA_motor_test.ino  —  TB6612FNG channel A spin test, driven by an Arduino Uno
 * ---------------------------------------------------------------------------
 * Purpose: prove whether the motor controller's CHANNEL A can actually drive
 * the motor, with the Pi taken completely out of the loop (clean 5V logic).
 * Ramps forward, brakes, ramps reverse, coasts — forever — printing state.
 *
 * ===========================  WIRING  ======================================
 *   Uno pin        ->  TB6612FNG pin      Notes
 *   ---------------------------------------------------------------
 *   D10 (~PWM)     ->  PWMA               speed (analogWrite 0-255)
 *   D8             ->  AIN1               direction
 *   D9             ->  AIN2               direction
 *   D7             ->  STBY               HIGH = driver enabled
 *   5V             ->  VCC                LOGIC supply  (2.7-5.5V ONLY)
 *   GND            ->  GND                logic ground
 *
 *   TB6612 pin     ->  other
 *   ---------------------------------------------------------------
 *   VM             ->  motor battery +    MOTOR supply (2.5-13.5V) e.g. 7.4V
 *   GND            ->  motor battery -     AND Uno GND  (see COMMON GROUND)
 *   A01            ->  motor lead 1
 *   A02            ->  motor lead 2        (swap A01/A02 to flip direction)
 *
 * ===========================  CRITICAL  ====================================
 *  1. COMMON GROUND: Uno GND, TB6612 GND, and battery(-) MUST all be tied
 *     together. Without it the logic has no shared reference and nothing moves.
 *  2. VCC != VM.  VCC is LOGIC power, max 5.5V -> use the Uno 5V pin.
 *     VM is MOTOR power -> use the battery (7.4V is fine, up to 13.5V).
 *     NEVER put the battery on VCC — 7.4V will destroy the chip.
 *  3. This is the REAL motor test: A01/A02 go to the MOTOR (not Uno analog
 *     pins). If you only want to probe the outputs without a motor, use
 *     tb6612_uno_test_chA instead.
 *
 * If channel A spins here but not off the Pi -> Pi-side wiring/logic issue.
 * If channel A is dead here too (with VCC+VM confirmed) -> the chip is toast,
 * swap in another H-bridge (DRV8833 / L298N / MX1508).
 * ---------------------------------------------------------------------------
 */

const uint8_t PIN_PWMA = 10;  // ~PWM capable
const uint8_t PIN_AIN1 = 8;
const uint8_t PIN_AIN2 = 9;
const uint8_t PIN_STBY = 7;

const int   MAX_DUTY  = 255;  // 0-255; full speed
const int   STEP      = 10;   // ramp increment
const int   STEP_MS   = 60;   // time per ramp step
const int   HOLD_MS   = 1500; // dwell at top speed

void setDirection(bool forward) {
  digitalWrite(PIN_AIN1, forward ? HIGH : LOW);
  digitalWrite(PIN_AIN2, forward ? LOW  : HIGH);
}

// Ramp the PWM duty from `from` to `to` (0-255), stepping smoothly.
void ramp(int from, int to) {
  int dir = (to >= from) ? STEP : -STEP;
  for (int d = from; (dir > 0) ? (d <= to) : (d >= to); d += dir) {
    analogWrite(PIN_PWMA, d);
    Serial.print("  duty="); Serial.println(d);
    delay(STEP_MS);
  }
  analogWrite(PIN_PWMA, to);
}

void brake() {                // short brake: both inputs HIGH, PWM on
  digitalWrite(PIN_AIN1, HIGH);
  digitalWrite(PIN_AIN2, HIGH);
  analogWrite(PIN_PWMA, MAX_DUTY);
}

void coast() {                // coast: PWM off
  analogWrite(PIN_PWMA, 0);
}

void setup() {
  pinMode(PIN_PWMA, OUTPUT);
  pinMode(PIN_AIN1, OUTPUT);
  pinMode(PIN_AIN2, OUTPUT);
  pinMode(PIN_STBY, OUTPUT);
  digitalWrite(PIN_STBY, HIGH);   // enable the driver
  coast();
  Serial.begin(9600);
  delay(500);
  Serial.println("TB6612 channel A spin test — Uno");
}

void loop() {
  Serial.println("FORWARD");
  setDirection(true);
  ramp(0, MAX_DUTY);
  delay(HOLD_MS);
  ramp(MAX_DUTY, 0);

  Serial.println("BRAKE");
  brake();
  delay(600);
  coast();
  delay(600);

  Serial.println("REVERSE");
  setDirection(false);
  ramp(0, MAX_DUTY);
  delay(HOLD_MS);
  ramp(MAX_DUTY, 0);

  Serial.println("COAST / pause");
  coast();
  delay(1500);
}
