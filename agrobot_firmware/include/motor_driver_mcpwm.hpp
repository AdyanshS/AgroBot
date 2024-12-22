#include "pin_map.hpp"
#include <Arduino.h>

#include "driver/mcpwm.h"
#include "soc/mcpwm_reg.h"
#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_periph.h"

#define MOTOR_DRIVER_PWM_FREQUENCY 20000 // 20kHz for MDD10A

// Motor Configuration Structure 
struct MotorConfig {
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_operator_t operator_id;
    u_int8_t dir_pin;
};


// Motor Mapping Array 
MotorConfig motors[] = {
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, MotorDriver1_DIR1}, // Motor 1
    {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, MotorDriver1_DIR2}, // Motor 2
    {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_A, MotorDriver2_DIR1}, // Motor 3
    {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM_OPR_B, MotorDriver2_DIR2}  // Motor 4
};

void setupMCPWM()
{

    // Initialize PWM pins
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, MotorDriver1_PWM1);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, MotorDriver1_PWM2);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1A, MotorDriver2_PWM1);
    mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM1B, MotorDriver2_PWM2);

    // Initialize direction pins
    pinMode(MotorDriver1_DIR1, OUTPUT);
    pinMode(MotorDriver1_DIR2, OUTPUT);
    pinMode(MotorDriver2_DIR1, OUTPUT);
    pinMode(MotorDriver2_DIR2, OUTPUT);

    // Configure MCPWM Parameters
    mcpwm_config_t pwm_config;
    pwm_config.frequency = MOTOR_DRIVER_PWM_FREQUENCY; // 20kHz
    pwm_config.cmpr_a = 0;                             // duty cycle of PWMxA = 0%
    pwm_config.cmpr_b = 0;                             // duty cycle of PWMxb = 0%
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Configure PWM timers
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}

void motorSetSpeed(uint8_t motor, uint8_t speed)
{

    if (motor < 1 || motor > 4) return; // ! Invalid motor number

    // Set the direction and speed of the motor
    MotorConfig &m = motors[motor - 1];

    // Set the direction and speed of the motor
    digitalWrite(m.dir_pin, speed > 0);
    mcpwm_set_duty(m.unit, m.timer, m.operator_id, abs(speed));
    mcpwm_set_duty_type(m.unit, m.timer, m.operator_id, MCPWM_DUTY_MODE_0);
}

void motorStop(uint8_t motor)
{
    if (motor < 1 || motor > 4) return; // Invalid motor index

    MotorConfig &m = motors[motor - 1];
    mcpwm_set_signal_low(m.unit, m.timer, m.operator_id);
}

void motorFullSpeed(uint8_t motor)
{
    if (motor < 1 || motor > 4) return; // Invalid motor index

    MotorConfig &m = motors[motor - 1];
    mcpwm_set_signal_high(m.unit, m.timer, m.operator_id);
}