// Passive voltmeter: Pi drives the TB6612, Uno just reads A01/A02.
// A01 -> Uno A0, A02 -> Uno A1, Uno GND tied into the shared ground.
void setup() { Serial.begin(9600); }
void loop() {
  Serial.print("A01(A0)="); Serial.print(analogRead(A0));
  Serial.print("  A02(A1)="); Serial.println(analogRead(A1));
  delay(150);
}
