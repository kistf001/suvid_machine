void setup() {
  // put your setup code here, to run once:

  pinMode(4, digitalWrite); //pump
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(4, HIGH);
delay(125);
digitalWrite(4, LOW);
delay(800);
}
