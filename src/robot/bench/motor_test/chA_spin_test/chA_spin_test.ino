// Real spin test, channel A, forever loop.
void setup() {
  pinMode(7, OUTPUT); pinMode(8, OUTPUT);
  pinMode(9, OUTPUT); pinMode(10, OUTPUT);
  digitalWrite(7, HIGH); // STBY on
  Serial.begin(9600);
}
void loop() {
  Serial.println("forward");
  digitalWrite(8, HIGH); digitalWrite(9, LOW);
  digitalWrite(10, HIGH);
  delay(3000);
  Serial.println("reverse");
  digitalWrite(8, LOW); digitalWrite(9, HIGH);
  delay(3000);
}
