// TB6612 channel A output-stage probe, driven from an Uno.
// (Channel B confirmed dead 2026-07-04; this retests channel A.)
// GND->TB GND, 5V->VCC, D7->STBY, D8->AIN1, D9->AIN2, D10->PWMA.
// A01 -> Uno A0, A02 -> Uno A1 (VM must be 5V-safe, i.e. fed from the buck, not raw battery)
// Prints analogRead(A0)/A1 (0-1023, ~5V full scale) every 200ms while cycling states.
void setState(const char* label, bool stby, bool in1, bool in2, bool pwm) {
  digitalWrite(7, stby ? HIGH : LOW);
  digitalWrite(8, in1 ? HIGH : LOW);
  digitalWrite(9, in2 ? HIGH : LOW);
  digitalWrite(10, pwm ? HIGH : LOW);
  Serial.print("== ");
  Serial.println(label);
}

void setup() {
  pinMode(7, OUTPUT); pinMode(8, OUTPUT);
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  Serial.begin(9600);
  delay(500);
}

void printReadings(int n) {
  for (int i = 0; i < n; i++) {
    int a0 = analogRead(A0);
    int a1 = analogRead(A1);
    Serial.print("A01(A0)="); Serial.print(a0);
    Serial.print("  A02(A1)="); Serial.println(a1);
    delay(200);
  }
}

void loop() {
  setState("STBY off (both should float/low)", false, false, false, false);
  printReadings(5);

  setState("STBY on, FORWARD, PWM high (expect A01 high, A02 low)", true, true, false, true);
  printReadings(8);

  setState("STBY on, REVERSE, PWM high (expect A01 low, A02 high)", true, false, true, true);
  printReadings(8);

  setState("STBY on, FORWARD, PWM LOW (expect both low - short brake)", true, true, false, false);
  printReadings(5);

  delay(1000);
}
