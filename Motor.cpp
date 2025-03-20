#include "Motor.h"

Motor::Motor(byte pin_en_l, byte pin_en_r, byte pin_pwm_l, byte pin_pwm_r) {
    _pin_en_l = pin_en_l;
    _pin_en_r = pin_en_r;
    _pin_pwm_l = pin_pwm_l;
    _pin_pwm_r = pin_pwm_r;
}

void Motor::setup() {
    pinMode(_pin_en_l, OUTPUT);
    pinMode(_pin_en_r, OUTPUT);
    pinMode(_pin_pwm_l, OUTPUT);
    pinMode(_pin_pwm_r, OUTPUT);
}

Motor::Direction Motor::getDirection() {
    return _direction;
}

int Motor::getSpeed_pwm() {
    return _speed_pwm;
}

void Motor::rotate_pwm(int speed_pwm) {
    Direction direction;
    if (speed_pwm > 0) {
        direction = Clockwise;
    } else if (speed_pwm < 0) {
        direction = CounterClockwise;
    } else {
        direction = None;
    }

    speed_pwm = abs(speed_pwm);
    if (speed_pwm > 255) {
        speed_pwm = 255;
    }

    bool change_direction = _direction != direction;

    _direction = direction;
    _speed_pwm = speed_pwm;

    if (_direction == None) {
        digitalWrite(_pin_en_l, LOW);
        digitalWrite(_pin_en_r, LOW);
        return;
    }

    if (change_direction) {
        analogWrite(_pin_pwm_l, 0);
        analogWrite(_pin_pwm_r, 0);
    }

    byte pin_pwm = (direction == Clockwise) ? _pin_pwm_r : _pin_pwm_l;
    
    analogWrite(pin_pwm, _speed_pwm);

    digitalWrite(_pin_en_l, HIGH);
    digitalWrite(_pin_en_r, HIGH);
}

void Motor::rotate_rad_s(float speed_rad_s) {
    int speed_pwm = rad_s_to_pwm(speed_rad_s);
    rotate_pwm(speed_pwm);
}

int Motor::rad_s_to_pwm(float speed_rad_s) {
    if (speed_rad_s > MAX_SPEED_RAD_S) {
        speed_rad_s = MAX_SPEED_RAD_S;
    } else if (speed_rad_s < -MAX_SPEED_RAD_S) {
        speed_rad_s = -MAX_SPEED_RAD_S;
    }

    int pwm_val = (speed_rad_s / MAX_SPEED_RAD_S) * 255;
    return pwm_val;
}
