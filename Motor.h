#pragma once

#include <Arduino.h>

class Motor {
public:
    enum Direction {
        Clockwise = 0,
        CounterClockwise = 1,
        None = 2
    };

    Motor(byte pin_en_l, byte pin_en_r, byte pin_pwm_l, byte pin_pwm_r);
    void setup();

    int getSpeed_pwm();
    Direction getDirection();

    void rotate_pwm(int speed_pwm);
private:
    byte _pin_en_l;
    byte _pin_en_r;
    byte _pin_pwm_l;
    byte _pin_pwm_r;
    byte _speed_pwm = 0;
    Direction _direction = None;
};
