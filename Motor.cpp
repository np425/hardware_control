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

byte Motor::getSpeed_pwm() {
    return _speed_pwm;
}

void Motor::rotate_pwm(Direction direction, byte speed_pwm) {
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

void Motor::rotate_rad_s(double speed_rad_s) {
    byte speed_pwm = rad_s_to_pwm(speed_rad_s);
    Direction direction = rad_s_to_direction(speed_rad_s);

    rotate_pwm(direction, speed_pwm);
}

byte Motor::rad_s_to_pwm(double speed_rad_s) {
    speed_rad_s = fabs(speed_rad_s);
    if (speed_rad_s > MAX_SPEED_RAD_S) {
        speed_rad_s = MAX_SPEED_RAD_S;
    } else if (speed_rad_s < -MAX_SPEED_RAD_S) {
        speed_rad_s = -MAX_SPEED_RAD_S;
    }

    double pwm_val = fabs(speed_rad_s) / MAX_SPEED_RAD_S * 255.0;
    return (byte)pwm_val;
}

Motor::Direction Motor::rad_s_to_direction(double speed_rad_s) {
    Direction direction;

    if (speed_rad_s > 0) {
        direction = Clockwise;
    } else if (speed_rad_s < 0) {
        direction = CounterClockwise;
    } else {
        direction = None;
    }

    return direction;
}
