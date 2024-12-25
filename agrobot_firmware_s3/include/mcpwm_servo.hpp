#include "sdkconfig.h"
#include "Arduino.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"

#include <driver/adc.h>
#include "esp_sleep.h"
#include "driver/mcpwm.h"

#define SERVO1 11
#define SERVO2 12
#define SERVO3 13
#define SERVO4 14

#define SERVO_MIN_PULSEWIDTH 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH 2500 // Maximum pulse width in microsecond
#define SERVO_MAX_DEGREE 300      // Maximum angle in degree upto which servo can rotate

struct ServoConfig
{
    mcpwm_unit_t unit;
    mcpwm_timer_t timer;
    mcpwm_io_signals_t signal;
    mcpwm_operator_t operator_id;
    u_int8_t gpio;
};

ServoConfig servo1 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM_OPR_A, SERVO1};
ServoConfig servo2 = {MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0B, MCPWM_OPR_B, SERVO2};
ServoConfig servo3 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1A, MCPWM_OPR_A, SERVO3};
ServoConfig servo4 = {MCPWM_UNIT_1, MCPWM_TIMER_1, MCPWM1B, MCPWM_OPR_B, SERVO4};

void setupMCPWMServo()
{
    mcpwm_gpio_init(servo1.unit, servo1.signal, servo1.gpio);
    mcpwm_gpio_init(servo2.unit, servo2.signal, servo2.gpio);
    mcpwm_gpio_init(servo3.unit, servo3.signal, servo3.gpio);
    mcpwm_gpio_init(servo4.unit, servo4.signal, servo4.gpio);

    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50; // frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;     // initial duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;     // initial duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    // Configure the timers
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);
    mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_1, &pwm_config);
}
/**
   @brief Use this function to calculate pulse width for per degree rotation
   @param  degree_of_rotation the angle in degree to which servo has to rotate
   @return
       - calculated pulse width
**/
static uint32_t servo_per_degree_init(uint32_t degree_of_rotation)
{
    return (SERVO_MIN_PULSEWIDTH + (((SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH) * (degree_of_rotation)) / (SERVO_MAX_DEGREE)));
}

static uint32_t degree_from_microseconds(uint32_t pulsewidth_us)
{
    return (pulsewidth_us - SERVO_MIN_PULSEWIDTH) * SERVO_MAX_DEGREE / (SERVO_MAX_PULSEWIDTH - SERVO_MIN_PULSEWIDTH);
}

void setMotorAngle(ServoConfig servo, int angle)
{
    uint32_t angle_us = servo_per_degree_init(angle);
    mcpwm_set_duty_in_us(servo.unit, servo.timer, servo.operator_id, angle_us);
}

void getMotorAngle(ServoConfig servo)
{
    int duty_us = mcpwm_get_duty_in_us(servo.unit, servo.timer, servo.operator_id);
    Serial.print("Duty cycle: ");
    Serial.print(duty_us);
    Serial.println(" us");

    int angle = degree_from_microseconds(duty_us);
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

// void setup()
// {
//   Serial.begin(115200);

//   setupMCPWMServo();
// }
