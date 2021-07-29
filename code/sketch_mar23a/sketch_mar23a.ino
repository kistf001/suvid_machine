#include <string.h>

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

  if (*c == '\0')
    return 0;

  if (*c == '-')
    positive = -1;

  while (*c) {
    if (*c > '0' && *c < '9')
      value = ((value * 10) + *c) - '0';
    c++;
  }

  return value * positive;
}

void pcint_setup(){
  PCICR  |= 0b001  ;
  PCMSK0 |= 0b10000;
}
void phase_detecter(){
  PINB|=0b00001000;
}

char command_time           = 0;
char command_temp           = 0;
char command_power          = 0;

char state_time             = 0;
char state_temp_water       = 0;
char state_temp_heater      = 0;
char state_temp_heatsink    = 0;
char state_power            = 0;

void setup()
{
  pinMode(8,digitalRead);
  Serial.begin(9600);   //시리얼모니터
  Serial1.begin(9600); //블루투스 시리얼
}
char qerer[257] = {0,};

int a = 0;
void loop()
{
  a++;
  //Serial.println(a);
  if (Serial1.available()) {

    char s = Serial1.read();

    if (s == '#') {
      qerer[-1] = 0;
    }
    else if (s == ';') {
      
      qerer[qerer[-1]] = 0;
      
      if (_strcmp("temp=", qerer)) {
        unsigned char command_temp_buffer = _atoi(qerer + 5);
        command_temp = command_temp_buffer;
        Serial.print("temp:"), Serial.println(command_temp);
      } else if (_strcmp("time=", qerer)) {
        unsigned char command_time_buffer = _atoi(qerer + 5);
        command_time = command_time_buffer;
        Serial.print("time:"), Serial.println(command_time);
      } else if (_strcmp("power=", qerer)) {
        unsigned char command_power_buffer = _atoi(qerer + 6);
        command_power = command_power_buffer;
        Serial.print("power:"), Serial.println(state_power);
      } else if (_strcmp("data*", qerer)) {
        Serial1.print("#32,12,23;"), Serial.print("data:"), Serial.println("asdfg");
      }
      
    }
    else {
      qerer[qerer[-1]] = s, qerer[-1] += 1;
    }

  }
  if (Serial.available()) {
    Serial1.write(Serial.read());  //시리얼 모니터 내용을 블루추스 측에 WRITE
  }
}
