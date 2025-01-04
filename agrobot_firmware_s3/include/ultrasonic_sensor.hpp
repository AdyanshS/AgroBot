#ifndef ULTRASONIC_SENSOR_HPP
#define ULTRASONIC_SENSOR_HPP

#include <Arduino.h>
#include <HCSR04.h>

#include "pin_map.hpp"

// Set the Ultrasonic Sensor pins
byte triggerPin = Trig1;
byte echoCount = 4;
byte *echoPins = new byte[echoCount]{Echo1, Echo2, Echo3, Echo4};

/**
 * @brief Initializes the ultrasonic sensors
 *
 * This function sets up the ultrasonic sensors by initializing the HCRS04 object
 * with the specified trigger pin and echo pins
 */
void ultrasonic_sensor_setup()
{
    HCSR04.begin(triggerPin, echoPins, echoCount);
}

/**
 * @brief Measures the distances using the ultrasonic sensors
 * in centimeters
 *
 * @return double* Pointer to an array of distances in centimeters.
 */
double *getUltrasonicDistancesCM()
{
    return HCSR04.measureDistanceCm();
}

#endif // ULTRASONIC_SENSOR_HPP