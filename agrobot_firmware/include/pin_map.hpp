#ifndef PIN_MAP_HPP
#define PIN_MAP_HPP

// Motor Driver 1 - MDD10A
#define MotorDriver1_DIR1 19
#define MotorDriver1_PWM1 18
#define MotorDriver1_DIR2 5
#define MotorDriver1_PWM2 17

// Motor Driver 2 - MDD10A
#define MotorDriver2_DIR1 2
#define MotorDriver2_PWM1 15
#define MotorDriver2_DIR2 16
#define MotorDriver2_PWM2 4

// Motor Driver 4 - FD04A // TODO: Change the pin numbers
#define MotorDriver3_DIR1 21
#define MotorDriver3_PWM1 3

// Encoder Pins
// #define ENC3_A 13    // ! When there is a connection in 13 and 12 gpio pins the code does not get uploaded to the board
// #define ENC3_B 12
#define ENC1_A 27
#define ENC1_B 14
#define ENC2_A 25
#define ENC2_B 26
#define ENC3_A 32
#define ENC3_B 33
#define ENC4_A 34
#define ENC4_B 35
#define ENC5_A 22 // TODO: Change the pin numbers
#define ENC5_B 23

#endif // PIN_MAP_HPP