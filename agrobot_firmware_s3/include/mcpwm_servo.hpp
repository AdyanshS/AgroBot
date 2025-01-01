#include "Arduino.h"
#include "driver/mcpwm.h"
#include "pin_map.hpp"

#define SERVO_MAX_DEGREE 300 // Maximum angle in degree upto which servo can rotate

struct ServoConfig
{
    mcpwm_unit_t unit;              // MCPWM unit number
    mcpwm_timer_t timer;            // MCPWM timer number
    mcpwm_io_signals_t signal;      // MCPWM signal
    mcpwm_operator_t operator_id;   // MCPWM operator
    u_int8_t gpio;                  // GPIO pin number
    uint32_t min_pulsewidth;        // Minimum pulse width in microsecond
    uint32_t max_pulsewidth;        // Maximum pulse width in microsecond
};

ServoConfig servo1 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A, SERVO1, 500, 2500};
ServoConfig servo2 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B, SERVO2, 500, 2500};
ServoConfig servo3 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A, SERVO3, 500, 2500};
ServoConfig servo4 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B, SERVO4, 500, 2500};
ServoConfig servo5 = {MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM_OPR_A, SERVO5, 500, 2500};

void setupMCPWMServo()
{
    mcpwm_gpio_init(servo1.unit, servo1.signal, servo1.gpio);
    mcpwm_gpio_init(servo2.unit, servo2.signal, servo2.gpio);
    mcpwm_gpio_init(servo3.unit, servo3.signal, servo3.gpio);
    mcpwm_gpio_init(servo4.unit, servo4.signal, servo4.gpio);
    mcpwm_gpio_init(servo5.unit, servo5.signal, servo5.gpio);

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
   @brief Use this function to calculate pulse width for per degree rotation
   @param  degree_of_rotation the angle in degree to which servo has to rotate
   @return
       - calculated pulse width
**/
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation, const ServoConfig &servo)
{
    return (servo.min_pulsewidth + (((servo.max_pulsewidth - servo.min_pulsewidth) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
}

static uint32_t degree_from_microseconds(uint32_t pulsewidth_us, const ServoConfig &servo)
{
    return (pulsewidth_us - servo.min_pulsewidth) * SERVO_MAX_DEGREE / (servo.max_pulsewidth - servo.min_pulsewidth);
}

void setMotorAngle(ServoConfig servo, int angle)
{
    uint32_t angle_us = servo_per_degree_init(angle, servo);
    mcpwm_set_duty_in_us(servo.unit, servo.timer, servo.operator_id, angle_us);
}

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

void setMotorPulseWidth(ServoConfig servo, int pulsewidth_us)
{
    mcpwm_set_duty_in_us(servo.unit, servo.timer, servo.operator_id, pulsewidth_us);
}

void getMotorPulseWidth(ServoConfig servo)
{
    int duty_us = mcpwm_get_duty_in_us(servo.unit, servo.timer, servo.operator_id);
    Serial.print("Duty cycle: ");
    Serial.print(duty_us);
    Serial.println(" us");
}
