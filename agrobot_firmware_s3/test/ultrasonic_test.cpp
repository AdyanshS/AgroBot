#include "Arduino.h"
#include "HCSR04.h"

byte trigPin = 15;
byte echoPin = 16;

void setup()
{

    Serial.begin(115200);
    HCSR04.begin(trigPin, echoPin);
}

void loop()
{
    double *distances = HCSR04.measureDistanceCm();

    Serial.print("1: ");
    Serial.print(distances[0]);
    Serial.println(" cm");

    Serial.println("---");
    delay(250);
}