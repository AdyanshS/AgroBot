#ifndef PIN_MAP_HPP
#define PIN_MAP_HPP

// Motor Driver 1 - MDD10A
#define MotorDriver1_DIR1 19
#define MotorDriver1_PWM1 21
#define MotorDriver1_DIR2 5
#define MotorDriver1_PWM2 18

// Motor Driver 2 - MDD10A
#define MotorDriver2_DIR1 4
#define MotorDriver2_PWM1 16
#define MotorDriver2_DIR2 15
#define MotorDriver2_PWM2 2

// Motor Driver 4 - FD04A // TODO: Change the pin numbers
#define MotorDriver3_DIR1 23
#define MotorDriver3_PWM1 22

// Encoder Pins
// #define ENC3_A 13    // ! When there is a connection in 13 and 12 gpio pins the code does not get uploaded to the board
// #define ENC3_B 12
#define ENC1_A 14
#define ENC1_B 27
#define ENC2_A 26
#define ENC2_B 25
#define ENC3_A 33
#define ENC3_B 32
#define ENC4_A 35
#define ENC4_B 34
#define ENC5_A 39 // TODO: Change the pin numbers
#define ENC5_B 36

#endif // PIN_MAP_HPP