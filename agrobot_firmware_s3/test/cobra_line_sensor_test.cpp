// #include "Arduino.h"
// #include "cobra_line_sensor.hpp"

// void setup()
// {
//     setupCobraSensor();

//     Serial.begin(115200);
// }

// void loop()
// {
//     int32_t sensor1 = readLineSensorVoltage(CobraIR1);
//     int32_t sensor2 = readLineSensorVoltage(CobraIR2);
//     int32_t sensor3 = readLineSensorVoltage(CobraIR3);
//     int32_t sensor4 = readLineSensorVoltage(CobraIR4);

//     Serial.print("Sensor 1: ");
//     Serial.print(sensor1);
//     Serial.print(" mV, Sensor 2: ");
//     Serial.print(sensor2);
//     Serial.print(" mV, Sensor 3: ");
//     Serial.print(sensor3);
//     Serial.print(" mV, Sensor 4: ");
//     Serial.print(sensor4);

//     int32_t sensor1_raw = readLineSensorRaw(CobraIR1);
//     int32_t sensor2_raw = readLineSensorRaw(CobraIR2);
//     int32_t sensor3_raw = readLineSensorRaw(CobraIR3);
//     int32_t sensor4_raw = readLineSensorRaw(CobraIR4);

//     Serial.print("Raw Sensor 1: ");
//     Serial.print(sensor1_raw);
//     Serial.print(", Raw Sensor 2: ");
//     Serial.print(sensor2_raw);
//     Serial.print(", Raw Sensor 3: ");
//     Serial.print(sensor3_raw);
//     Serial.print(", Raw Sensor 4: ");
//     Serial.println(sensor4_raw);

//     delay(30);
// }
