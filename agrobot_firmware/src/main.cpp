#include <Arduino.h>

#define dir_1 5   //d5
#define pwm_1 18  //d18
#define dir_2 4
#define pwm_2 3



void setup() {
  pinMode(pwm_1,OUTPUT);
  pinMode(dir_1,OUTPUT);

  Serial.begin(115200);

}

void loop() {

  for (int i = 0; i < 255; i++)
  {
    analogWrite(pwm_1, i);
    digitalWrite(dir_1, HIGH);
    Serial.println(i);
    delay(100);
  }

}
