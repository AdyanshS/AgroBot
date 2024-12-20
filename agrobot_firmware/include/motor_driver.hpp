#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <Arduino.h>
#include "pin_map.hpp"
#include <ESP32Encoder.h>

#define PWM_FREQUENCY 10000

class MotorDriver {
public:
    MotorDriver(int dirPin, int pwmPin, int maxPwm, int minPwm, int pwmChannel, ESP32Encoder* encoder, int pwmError);

    void setup();
    void runMotor(int pwmValue);
    void testMotor(int motorPwmValue, int duration, int cycles);

private:
    int _dirPin;
    int _pwmPin;
    int _maxPwm;
    int _minPwm;
    int _pwmChannel;
    ESP32Encoder* _encoder;
    int _pwmError;

};

MotorDriver::MotorDriver(int dirPin, int pwmPin, int maxPwm, int minPwm, int pwmChannel, ESP32Encoder* encoder, int pwmError)
    : _dirPin(dirPin), _pwmPin(pwmPin), _maxPwm(maxPwm), _minPwm(minPwm), _pwmChannel(pwmChannel), _encoder(encoder), _pwmError(pwmError) {}


void MotorDriver::setup() {
    pinMode(_dirPin, OUTPUT);

    // Configure LEDC PWM channel
    ledcSetup(_pwmChannel, PWM_FREQUENCY, 8); // 10 kHz PWM, 8-bit resolution
    ledcAttachPin(_pwmPin, _pwmChannel);
}

void MotorDriver::runMotor(int pwmValue) {

    // Serial.print("PWM Value Command: ");
    // Serial.println(pwmValue);

    int16_t pwm_constraint = constrain(abs(pwmValue) + _pwmError, _minPwm, _maxPwm);
    // Serial.print("PWM Value Constrained: ");
    // Serial.println(pwm_constraint);


    if (pwmValue == 0) {
        digitalWrite(_dirPin, LOW);
        ledcWrite(_pwmChannel, 0);
        // Serial.print("PWM Sent to motor Stop");
        // Serial.println(pwm_constraint);
    }

    else if (pwmValue > 0) {
        digitalWrite(_dirPin, HIGH);
        ledcWrite(_pwmChannel, pwm_constraint );
        // Serial.print("PWM Sent to motor Anti Clockwise");
        // Serial.println(pwm_constraint);
    } else {
        digitalWrite(_dirPin, LOW);
        ledcWrite(_pwmChannel, pwm_constraint);
        // Serial.print("PWM Sent to motor Clockwise");
        // Serial.println(pwm_constraint);
    }
    
}

void MotorDriver::testMotor(int motorPwmValue, int duration, int cycles) {
    long previousEncoderTicks = 0;
    long currentEncoderTicks = 0;
    long encoderDifference = 0;

    for (int i = 0; i < cycles; i++) {
        // Run motor clockwise
        runMotor(motorPwmValue);
        delay(duration);

        // Get current encoder ticks
        currentEncoderTicks = _encoder->getCount();
        encoderDifference = currentEncoderTicks - previousEncoderTicks;

        // Print encoder ticks and difference
        Serial.print("Cycle ");
        Serial.print(i + 1);
        Serial.print(": Encoder Ticks: ");
        Serial.print(currentEncoderTicks);
        Serial.print(", Difference: ");
        Serial.println(encoderDifference);

        // Update previous encoder ticks
        previousEncoderTicks = currentEncoderTicks;

        // Stop motor
        runMotor(0);
        delay(duration);
    }

    // Run motor counterclockwise
    for (int i = 0; i < cycles; i++) {
        // Run motor counterclockwise
        runMotor(-motorPwmValue);
        delay(duration);

        // Get current encoder ticks
        currentEncoderTicks = _encoder->getCount();
        encoderDifference = currentEncoderTicks - previousEncoderTicks;

        // Print encoder ticks and difference
        Serial.print("Cycle ");
        Serial.print(i + 1);
        Serial.print(": Encoder Ticks: ");
        Serial.print(currentEncoderTicks);
        Serial.print(", Difference: ");
        Serial.println(encoderDifference);

        // Update previous encoder ticks
        previousEncoderTicks = currentEncoderTicks;

        // Stop motor
        runMotor(0);
        delay(duration);
        
    }
}

#endif // MOTOR_DRIVER_HPP