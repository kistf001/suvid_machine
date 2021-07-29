#include <math.h>
#include <string.h>
#include <avr/pgmspace.h>

#include <AltSoftSerial.h>
AltSoftSerial altSerial;

int _strcmp(char *s1, char *s2) {
  int i = 0;
  while (s1[i]) {
    if (s1[i] != s2[i]) {
      return 0;
    }
    i++;
  }
  return 1;
}

int _atoi(char const *c) {
  int value = 0;
  int positive = 1;
  if (*c == '\0') return 0;
  if (*c == '-' ) positive = -1;
  while (*c) {
    if ('0' <= *c && *c <= '9')
      value = ((value * 10) + *c) - '0';
    c++;
  }
  return value * positive;
}

void pcint_setup() {
  PCICR  |= 0b100;   // pcint active 0
  PCMSK2 |= 0x80;    // pcint pin position 0, ardu 8
}
void timer2_setup() {
  TCCR2A = 0x03;   // Mode3 Fast PWM
  TCCR2B = 0b111;  // clkT2S/64 (From prescaler) 4us Period
  TIMSK2 = 0x01;   // Timer/Counter2 Overflow Interrupt Enable
  TCNT2  = 130;    // 16000000/64/(256-6)=1000Hz=1ms
}
void adc_setup() {

  //ADLAR = 0;

  DIDR0  = 0b1111;         // 디지털 IO 끄는것

  ADMUX = 0;
  ADMUX |= 0x01;         // MUX0
  //ADMUX |= 0x02;         // MUX1
  //ADMUX |= 0x04;         // MUX2
  //ADMUX |= 0x08;         // MUX3
  //ADMUX |= 0x10;
  //ADMUX |= 0x20;         // ADLAR: ADC Left Adjust Result
  ADMUX |= 0x40;         // REFS0 : Voltage Reference Selection
  //ADMUX |= 0x80;         // REFS1 : Voltage Reference Selection

  ADCSRA  = 0;
  ADCSRA |= 0x01;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x02;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x04;         // ADPS2:0: ADC Prescaler Select Bits
  ADCSRA |= 0x08;         // ADIE: ADC Interrupt Enable
  //ADCSRA |= 0x10;         // ADIF: ADC Interrupt Flag
  //ADCSRA |= 0x20;         // ADATE: ADC Auto Trigger Enable
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
  ADCSRA |= 0x80;         // ADEN: ADC Enable

  //ADMUX = 0, ADCSRA  = 0;
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
}

///////////////////////////////////////////////////
/*setup*/
#define duty_limit 128

///////////////////////////////////////////////////
uint32_t command_timer                = 0;
uint8_t  command_timer_start_signal   = 0;
uint8_t  command_temp                 = 0;
uint8_t  command_power                = 0;
///////////////////////////////////////////////////
uint16_t ADC_RAW[4]                   = {0,};
double   ADC_QUEUE[4]                 = {0,};
double   &state_temp_water            = ADC_QUEUE[0];
double   &state_temp_heater           = ADC_QUEUE[1];
double   &state_temp_heatsink         = ADC_QUEUE[2];
uint32_t state_timer                  = 0;
uint8_t  state_power                  = 0;
///////////////////////////////////////////////////
/*alert CONTROL*/
uint8_t alert                         = 0;
uint8_t alert_send_flag               = 0;
///////////////////////////////////////////////////
/*POWER CONTROL*/
uint8_t duty                          = 0;
uint8_t count                         = 0;
uint8_t state_pin                     = 0;
///////////////////////////////////////////////////
/*PID CONTROL*/
double pid_value                = 0;
///////////////////////////////////////////////////

int timers = 0;

void p_control() {
  double error = ((double)command_temp - (double)state_temp_water ) * 130;
  //double error = ((double)command_temp - (double)0 ) * 9;
  if (error < 0    ) error = 0;
  if (1023  < error) error = 1023;
  duty = (uint8_t)(error / 4);
  if (duty > duty_limit) {
    duty = duty_limit;
  }
  if (state_power == 0) {
    duty = 0;
  }
}

void alert_temp_cheack() {
  // 히터랑 물온도랑 차이가 30을 초과하면 문제가 있다고 판단
  if ((state_temp_heater - state_temp_water) > 30) {
    alert |= 0b001;
  }
  // 히트싱크 온도가 너무 높아지면 문제가 있다고 판단
  if ((state_temp_heatsink) > 65) {
    alert |= 0b010;
  }
  // 히터 온도가 너무 높아지면 문제가 있다고 판단
  if ((state_temp_heater) > 80) {
    alert |= 0b100;
  }
}

double adc_to_temp(unsigned int qq) {

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
void serial_communication() {

  static char qerer[65];

  if (Serial.available()) {
    char s = Serial.read();
    if (s == '#') qerer[-1] = 0;
    else if (s == ';') {

      qerer[qerer[-1]] = 0;

      if (_strcmp("temp=", qerer)) {
        uint8_t command_temp_buffer = _atoi(qerer + 5);
        if ((0 < command_temp_buffer) && (command_temp_buffer <= 80)) {
          command_temp = command_temp_buffer;
        }
        else command_temp = 0;
        altSerial.print(F("ok:temp=")), altSerial.println(command_temp);
      }

      else if (_strcmp("timer=", qerer)) {
        command_timer = _atoi(qerer + 5);
        altSerial.print(F("ok:timer=")), altSerial.println(command_timer);
        state_timer = command_timer;
      }

      else if (_strcmp("timer go", qerer)) {
        altSerial.print(F("ok:timer go")), altSerial.println(command_timer);
        command_timer_start_signal = 1;
      }

      else if (_strcmp("timer stop", qerer)) {
        altSerial.print(F("ok:timer stop")), altSerial.println(command_timer);
        command_timer_start_signal = 0;
      }

      else if (_strcmp("power=", qerer)) {
        command_power = _atoi(qerer + 6);
        altSerial.print(F("ok:power=")), altSerial.println(command_power);
        state_power = command_power;
      }

      else if (_strcmp("data*", qerer) | _strcmp("d*", qerer)) {
        altSerial.print(F("ok:data*"));
        altSerial.print(alert), altSerial.print(",");
        altSerial.print(state_timer), altSerial.print(",");
        altSerial.print(state_temp_water), altSerial.print(",");
        altSerial.print(state_temp_heater), altSerial.print(",");
        altSerial.print(state_temp_heatsink), altSerial.print("\n");
      }

      else if (_strcmp("reset*", qerer)) {
        command_temp  = 0;
        command_timer = 0;
        command_power = 0;
      }

      // test function
      else if (_strcmp("duty=", qerer)) {
        duty = _atoi(qerer + 5);
        altSerial.print(F("ok:")), altSerial.println(duty);
      }

      else if (_strcmp("start*", qerer)) {
        altSerial.print(F("ok:")), altSerial.println("start");
      }

    }
    else qerer[qerer[-1]] = s, qerer[-1] += 1, qerer[-1] &= 0b00111111;
  }

}

ISR(ADC_vect) {

  static uint8_t port_number;
  alert_temp_cheack();
  if (alert != 0) {
    state_pin = 0, digitalWrite( 5, LOW);
  }
  ADC_RAW[port_number] = ADCW;
  ADC_QUEUE[port_number] = adc_to_temp(ADCW);
  port_number++;
  port_number &= 0b11;
  ADMUX  &= 0xF0;
  ADMUX  |= port_number;
  ADCSRA |= 0x40;         // ADSC: ADC Start Conversion
}

ISR(PCINT0_vect) {}
ISR(PCINT1_vect) {}
ISR(PCINT2_vect) {}
ISR(TIMER2_OVF_vect) {

  timers += 1;
  timers &= 0b1111111;

  TCNT2  = 130;
  if(state_power != 0) digitalWrite( 4, HIGH);
  else digitalWrite( 4, LOW);

  if (duty < (uint8_t)24) {
    p_control();
    if ((count < duty) && (alert == 0) && (state_power == 1)) state_pin = 1, digitalWrite( 5, HIGH);
    //if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite( 5, HIGH);
    else state_pin = 0, digitalWrite( 5, LOW);
    count += 1;
  }

  else if (((uint8_t)24 <= duty) && (duty < (uint8_t)96)) {
    p_control();
    if ((count < duty) && (alert == 0) && (state_power == 0)) state_pin = 1, digitalWrite( 5, HIGH);
    //if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite( 5, HIGH);
    else state_pin = 0, digitalWrite( 5, LOW);
    count += 4;
  }

  else if ((uint8_t)96 <= duty) {
    p_control();
    if ((count < duty) && (alert == 0) && (state_power == 0)) state_pin = 1, digitalWrite( 5, HIGH);
    //if ((count < duty) && (0 == 0) && (state_power != 0)) state_pin = 1, digitalWrite( 5, HIGH);
    else state_pin = 0, digitalWrite( 5, LOW);
    count += 16;
  }

  static int a = 0;
  a++;
  if (a > 127) {
    if ((command_timer_start_signal != 0) && (command_timer >= state_timer > 0)) {
      state_timer--;
    }
    if((state_timer==0)&&(command_timer_start_signal == 1)){
      command_timer_start_signal = 0;
      state_power=0;
    }
    a = 0;
  }
}


void setup() {}
void loop() {
  
  tone(6, 6); // Send 1KHz sound signal...
  delay(125);        // ...for 1 sec
  noTone(6);     // Stop sound...
  delay(64);        // ...for 1 sec
  tone(6, 6); // Send 1KHz sound signal...
  delay(125);        // ...for 1 sec
  noTone(6);     // Stop sound...
  delay(500);        // ...for 1 sec
  
  altSerial.println(F("Hellow Sous vide"));
  
  cli();
  sei();
  
     Serial.begin(9600);
  altSerial.begin(9600);
  
  pinMode(4, digitalWrite); //pump
  pinMode(5, digitalWrite); //heater
  pinMode(10, digitalRead);
  pinMode(11, digitalRead);
  pinMode(12, digitalRead);
  
  //pcint_setup();
  timer2_setup();
  adc_setup();
  //adc_to_temp_setup(); 
  
  while (1) {

    serial_communication();

    if (alert != 0) {
      if (alert &= 0b100) ;
      if (alert &= 0b010) ;
      if (alert &= 0b001) ;
      if (alert != 0b000) ;
    }
    else {

    }
    
    if (timers >= 127) {
      Serial.print(F("set_temp: ")); Serial.print(command_temp); Serial.println(',');
      Serial.print(F("set_time: ")); Serial.print(state_timer); Serial.print(F(" / ")); Serial.print(command_timer); 
                                     Serial.print(F(" [switch state="));                Serial.print(command_timer_start_signal); Serial.print(F("]"));    
                                     Serial.println(','); 
      Serial.print(F("state_pin: ")); Serial.print(state_pin * 512); Serial.println(',');
      Serial.print(F("duty: ")); Serial.print(duty); Serial.println(',');
      Serial.print(F("alert: ")); Serial.print(alert); Serial.print(F(" <= this is bin flag ")); Serial.println(',');
      Serial.print(F("state_power: ")); Serial.print(state_power); Serial.println(',');
      Serial.print(F("state_temp_water: ")); Serial.print(state_temp_water); Serial.println(',');
      Serial.print(F("state_temp_heater: ")); Serial.print(state_temp_heater); Serial.println(',');
      Serial.print(F("state_temp_heatsink: ")); Serial.print(state_temp_heatsink); Serial.println(',');
      if (alert != 0) Serial.print(F("cheack alert state!")); Serial.println(',');
      Serial.println("\n");
    }
  }
}
