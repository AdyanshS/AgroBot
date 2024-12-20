#ifndef PIN_MAP_HPP
#define PIN_MAP_HPP

// Motor Driver 1 - MDD10A
#define MotorDriver1_DIR1 5  // D15
#define MotorDriver1_PWM1 18 // D2
#define MotorDriver1_DIR2 19 // D4
#define MotorDriver1_PWM2 21 // RX2

// Motor Driver 2 - MDD10A
#define MotorDriver2_DIR1 15 // D5
#define MotorDriver2_PWM1 2  // D18
#define MotorDriver2_DIR2 4  // D19
#define MotorDriver2_PWM2 16 // D21

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

#endif // PIN_MAP_HPP