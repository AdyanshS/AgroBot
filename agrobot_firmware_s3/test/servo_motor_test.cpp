// void setup()
// {
//   Serial.begin(115200);

//   setupMCPWM();
// }

// void loop()
// {
//   // for (int i = 0; i <= 180; i++)
//   // {
//   //   setMotorAngle(servo1, i);
//   //   setMotorAngle(servo2, i);
//   //   setMotorAngle(servo3, i);
//   //   setMotorAngle(servo4, i);
//   //   vTaskDelay(15 / portTICK_PERIOD_MS);
//   // }

//   for (int i = 180; i >= 0; i--)
//   {
//     setMotorAngle(servo1, i);
//     setMotorAngle(servo2, i);
//     setMotorAngle(servo3, i);
//     setMotorAngle(servo4, i);
//     vTaskDelay(15 / portTICK_PERIOD_MS);
//   }

//   // Set all angles to 0
//   setMotorAngle(servo1, 180);
//   setMotorAngle(servo2, 180);
//   setMotorAngle(servo3, 180);
//   setMotorAngle(servo4, 180);

//   // Set Motor Microseconds
//   setMotorPulseWidth(servo1, 500);
//   setMotorPulseWidth(servo2, 500);
//   setMotorPulseWidth(servo3, 500);
//   setMotorPulseWidth(servo4, 500);

//   // Get Motor PUlse Width
//   getMotorPulseWidth(servo1);
//   getMotorPulseWidth(servo2);
//   getMotorPulseWidth(servo3);
//   getMotorPulseWidth(servo4);
// }