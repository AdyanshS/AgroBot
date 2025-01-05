#ifndef MCPWM_SERVO_HPP
#define MCPWM_SERVO_HPP

#include "Arduino.h"
#include "driver/mcpwm.h"
#include "pin_map.hpp"

#define SERVO_MAX_DEGREE 300 // Maximum angle in degree upto which servo can rotate

/**
 * @brief Configuration structure for MCPWM Servo.
 */
struct ServoConfig
{
    mcpwm_unit_t unit;            // MCPWM unit number
    mcpwm_timer_t timer;          // MCPWM timer number
    mcpwm_io_signals_t signal;    // MCPWM signal
    mcpwm_operator_t operator_id; // MCPWM operator
    u_int8_t gpio;                // GPIO pin number
    uint32_t min_pulsewidth;      // Minimum pulse width in microsecond
    uint32_t max_pulsewidth;      // Maximum pulse width in microsecond
};

// Servo Configurations
ServoConfig servo1 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A, SERVO1, 500, 2500};
ServoConfig servo2 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B, SERVO2, 500, 2500};
ServoConfig servo3 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A, SERVO3, 500, 2500};
ServoConfig servo4 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B, SERVO4, 500, 2500};
ServoConfig servo5 = {MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A, SERVO5, 500, 2500};

/**
 * @brief Initializes the MCPWM Servo.
 *
 * This function sets up the MCPWM serrvo by initializing the GPIO pins and configuring
 * the MCPWM timers.
 */
void setupMCPWMServo()
{
    // Set the GPIO pins
    mcpwm_gpio_init(servo1.unit, servo1.signal, servo1.gpio);
    mcpwm_gpio_init(servo2.unit, servo2.signal, servo2.gpio);
    mcpwm_gpio_init(servo3.unit, servo3.signal, servo3.gpio);
    mcpwm_gpio_init(servo4.unit, servo4.signal, servo4.gpio);
    mcpwm_gpio_init(servo5.unit, servo5.signal, servo5.gpio);

    // Configure the PWM settings
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Configure the timers
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_2, &pwm_config);
}

/**
 * @brief Calculates the pulse width for a given degree of rotation.
 *
 * @param degree_of_rotation The angle in degrees to which the servo has to rotate.
 * @param servo The servo configuration.
 * @return uint32_t The calculated pulse width in microseconds.
 */
static uint32_t servo_per_degree_init(float degree_of_rotation, const ServoConfig &servo)
{
    return (servo.min_pulsewidth + (((servo.max_pulsewidth - servo.min_pulsewidth) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
}

/**
 * @brief Calculates the degree of rotation from a given pulse width.
 *
 * @param pulsewidth_us The pulse width in microseconds
 * @param servo The servo configuration
 * @return uint32_t The calculated degree of rotation
 */
static uint32_t degree_from_microseconds(uint32_t pulsewidth_us, const ServoConfig &servo)
{
    return (pulsewidth_us - servo.min_pulsewidth) * SERVO_MAX_DEGREE / (servo.max_pulsewidth - servo.min_pulsewidth);
}

/**
 * @brief Sets the angle of the servo motor.
 *
 * @param servo The servo configuration.
 * @param angle The angle in degrees to set the servo motor to.
 */
void setMotorAngle(ServoConfig servo, float angle)
{
    uint32_t angle_us = servo_per_degree_init(angle, servo);
    mcpwm_set_duty_in_us(servo.unit, servo.timer, servo.operator_id, angle_us);
}

/**
 * @brief Sets the angle of the servo motor.
 *
 * @param servo The servo configuration.
 */
void getMotorAngle(ServoConfig servo)
{
    int duty_us = mcpwm_get_duty_in_us(servo.unit, servo.timer, servo.operator_id);
    Serial.print("Duty cycle: ");
    Serial.print(duty_us);
    Serial.println(" us");

    int angle = degree_from_microseconds(duty_us, servo);
    Serial.print("Angle: ");
    Serial.println(angle);
}

/**
 * @brief Sets the pulse width of the servo motor.
 *
 * @param servo The servo configuration.
 * @param pulsewidth_us The pulse width in microseconds.
 */
void setMotorPulseWidth(ServoConfig servo, int pulsewidth_us)
{
    mcpwm_set_duty_in_us(servo.unit, servo.timer, servo.operator_id, pulsewidth_us);
}

/**
 * @brief Gets the pulse width of the servo motor.
 *
 * @param servo The servo configuration.
 */
void getMotorPulseWidth(ServoConfig servo)
{
    int duty_us = mcpwm_get_duty_in_us(servo.unit, servo.timer, servo.operator_id);
    Serial.print("Duty cycle: ");
    Serial.print(duty_us);
    Serial.println(" us");
}

#endif // MCPWM_SERVO_HPP