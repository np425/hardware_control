#pragma once

#include <Arduino.h>

class Motor {
public:
    Motor(uint8_t pinEnL, uint8_t pinEnR, uint8_t pinPWML, uint8_t pinPWMR);
    void setup();

    uint8_t getSpeed_pwm();
    bool getDirection();

    void enableRotation_pwm(uint8_t speed_pwm, bool isForward);
    constexpr static bool DIRECTION_FORWARD = true;
    constexpr static bool DIRECTION_BACKWARD = false;

private:
    uint8_t _pinEnL;
    uint8_t _pinEnR;
    uint8_t _pinPWML;
    uint8_t _pinPWMR;
    uint8_t _speed_pwm = 0;
    bool _direction = DIRECTION_FORWARD;
};
