#include <Arduino.h>
#include <ESP32Encoder.h>

// INclude esp dsp

// #define dir_1 5   //d5
// #define pwm_1 18  //d18
// #define dir_2 4
// #define pwm_2 3


// #define Enc1Pin1 14
// #define Enc1Pin2 27 

// #define Enc1Pin1 26
// #define Enc1Pin2 25

// #define Enc1Pin1 13
// #define Enc1Pin2 12 

#define Enc1Pin1 33
#define Enc1Pin2 32

ESP32Encoder encoder1;

void setup() {

  encoder1.attachFullQuad(Enc1Pin1, Enc1Pin2);
  encoder1.setCount(0);
  Serial.begin(115200);

}

void loop() {

  long newPos = encoder1.getCount();
  Serial.println(newPos);
  // Serial.println(  encoder1.isAttached());
}



