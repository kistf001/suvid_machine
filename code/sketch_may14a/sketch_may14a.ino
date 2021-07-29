void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
  pinMode(4, digitalWrite);
  tone(6, 6); // Send 1KHz sound signal...
  delay(125);        // ...for 1 sec
  noTone(6);     // Stop sound...
  delay(64);        // ...for 1 sec
  tone(6, 6); // Send 1KHz sound signal...
  delay(125);        // ...for 1 sec
  noTone(6);     // Stop sound...

  //pinMode(4, digitalWrite); //heater
  
}

double adc_to_temp(unsigned int qq) {

  //uint16_t *a = adc_to_temp_loockup_table;
  //return ((int16_t)pgm_read_word_near(a + qq)) * 0.03125;
  float Vo;
  float R1 = 10000;
  float logR2, R2, T;
  float c1 = 1.123983726e-03, c2 = 2.355308845e-04, c3 = 0.7770613823e-07;
  Vo = qq;
  R2 = R1 * ((1024.0 / (float)Vo) - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2 * logR2 + c3 * logR2 * logR2 * logR2));
  T = T - 273.15;
  return T;

}
void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(4,HIGH);
  delay(125);
  digitalWrite(4,LOW);
  delay(875);
  Serial.print(adc_to_temp(analogRead(0)));
  Serial.print(",");
  Serial.println(adc_to_temp(analogRead(1)));
}
