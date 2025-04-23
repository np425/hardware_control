#include "Motor.h"

Motor::Motor(uint8_t pinEnL, uint8_t pinEnR, uint8_t pinPWML, uint8_t pinPWMR) {
    _pinEnL = pinEnL;
    _pinEnR = pinEnR;
    _pinPWML = pinPWML;
    _pinPWMR = pinPWMR;
}

void Motor::setup() {
    // Motor runs at 490.20Hz frequency
    pinMode(_pinEnL, OUTPUT);
    pinMode(_pinEnR, OUTPUT);
    pinMode(_pinPWML, OUTPUT);
    pinMode(_pinPWMR, OUTPUT);

    digitalWrite(_pinEnL, 0);
    digitalWrite(_pinEnR, 0);
    analogWrite(_pinPWML, 0);
    analogWrite(_pinPWMR, 0);
}

uint8_t Motor::getSpeed_pwm() {
    return _speed_pwm;
}

bool Motor::getDirection() {
    return _direction;
}

void Motor::enableRotation_pwm(uint8_t speed_pwm, bool direction) {
    bool changeDirection = _direction != direction;

    _direction = direction;
    _speed_pwm = speed_pwm;

    if (speed_pwm == 0) {
        digitalWrite(_pinEnL, LOW);
        digitalWrite(_pinEnR, LOW);
        analogWrite(_pinPWML, 0);
        analogWrite(_pinPWMR, 0);
        return;
    }

    if (changeDirection) {
        analogWrite(_pinPWML, 0);
        analogWrite(_pinPWMR, 0);
    }

    uint8_t pin_pwm = (direction == DIRECTION_FORWARD) ? _pinPWMR : _pinPWML;
    
    analogWrite(pin_pwm, _speed_pwm);

    digitalWrite(_pinEnL, HIGH);
    digitalWrite(_pinEnR, HIGH);
}
