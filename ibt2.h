#pragma once

#include <Arduino.h>

class IBT2 {
public:
    enum Direction {
        Clockwise = 0,
        CounterClockwise = 1,
    };

    const static int MAX_SPEED = 255;

    IBT2(byte pin_en_l, byte pin_en_r, byte pin_pwm_l, byte pin_pwm_r);
    void setup();

    void setTargetSpeed(Direction direction, byte speed);

    Direction getCurrentDirection();
    Direction getTargetDirection();

    byte getCurrentSpeed();
    byte getTargetSpeed();

    bool spin(unsigned long accelerateMs = 0);

private:
    byte _pin_en_l;
    byte _pin_en_r;
    byte _pin_pwm_l;
    byte _pin_pwm_r;
    byte _speed_current = 0;
    byte _speed_target = 255;
    byte _dir_current = Direction::Clockwise;
    byte _dir_target = Direction::Clockwise;
};

IBT2::IBT2(byte pin_en_l, byte pin_en_r, byte pin_pwm_l, byte pin_pwm_r) {
    this->_pin_en_l = pin_en_l;
    this->_pin_en_r = pin_en_r;
    this->_pin_pwm_l = pin_pwm_l;
    this->_pin_pwm_r = pin_pwm_r;
}

void IBT2::setup() {
    pinMode(this->_pin_en_l, OUTPUT);
    pinMode(this->_pin_en_r, OUTPUT);
    pinMode(this->_pin_pwm_l, OUTPUT);
    pinMode(this->_pin_pwm_r, OUTPUT);
}

void IBT2::setTargetSpeed(int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if (speed < 0) {
        speed = 0;
    }
    this->_target_speed = speed;
}

IBT2::Direction IBT2::getCurrentDirection() {
    return this->_dir_current;
}

IBT2::Direction IBT2::getTargetDirection() {
    return this->_dir_target;
}

byte IBT2::getCurrentSpeed() {
    return this->_speed_current;
}

byte IBT2::getTargetSpeed() {
    return this->_speed_target;
}

bool IBT2::spin(unsigned long accelerateMs) {

}

/*
 * This file written by Tom Dhanabhon on Jan 13, 2023
*/

#ifndef _____TomIBT2__
#define _____TomIBT2__

#include <Arduino.h>

/*
 * IBT-2
 * Usage 1:
 * VCC pick MCU 5V power supply, GND connected microcontroller GND
 * R_EN and L_EN shorted and connected to 5 level, the drive to work.
 * L_PWM, input PWM signal or high motor forward
 * R_PWM, input PWM signal or high motor reversal
 * 
 * Usage 2:
 * VCC pick MCU 5V power supply, GND connected microcontroller GND
 * R_EN and L_EN short circuit and PWM signal input connected to high-speed
 * L_PWM, pin input 5V level motor is transferred
 * R_PWM, pin input 5V level motor reversal
*/

class TomIBT2 {
    public:
        enum Direction {
            CW = 1,     // clockwise
            CCW = 2,    // counter clockwise
            UNKNOWN = 3,
        };

        TomIBT2(byte r_en_pin, byte l_en_pin, byte rpwm_pin, byte lpwm_pin);
        void begin(void);

        void rampUp(Direction direction, unsigned long timeoutMs);    // Pass in a timeout in milliseconds for how long it will take to ramp from the current speed to full speed with a linear transition.
        void rampDown(unsigned long timeoutMs);

        void rotate(int speed, Direction direction);
        void stop(void);    // Stop the motor by allowing coasting.
        void brake(void);   // Stop the motor with braking action.

        void setTargetSpeed(int speed);
        void setTargetSpeedPercent(int percent);

        int getCurrentSpeed(void);
        int getCurrentSpeedPercent(void);
        Direction getCurrentDirection(void);

    private:
        byte R_EN_PIN;  // Forward drive enable input, high enable, low close
        byte L_EN_PIN;  // Reverse drive enable input, high enable, low close
        byte RPWM_PIN;  // Forward level or PWM signal input, active high
        byte LPWM_PIN;  // Inversion level or PWM signale input, active high
        byte R_IS;      // Forward drive - side current alarm ouput (Unused)
        byte L_IS;      // Reverse drive -side current alarm output (Unused)

        byte MAX_SPEED = 255;   // 100% duty cycle

        unsigned long currentMillis = 0;
        unsigned long previousMillis = 0; 

        int currentSpeed = 0;
        int targetSpeed = MAX_SPEED;
        Direction currentDirection = UNKNOWN;
};

#endif

#include "TomIBT2.h"

// Construcstor
TomIBT2::TomIBT2(byte r_en_pin, byte l_en_pin, byte rpwm_pin, byte lpwm_pin) {
    this->R_EN_PIN = r_en_pin;
    this->L_EN_PIN = l_en_pin;
    this->RPWM_PIN = rpwm_pin;
    this->LPWM_PIN = lpwm_pin;
}

void TomIBT2::begin(void) {
    pinMode(this->R_EN_PIN, OUTPUT);
    pinMode(this->L_EN_PIN, OUTPUT);
    pinMode(this->RPWM_PIN, OUTPUT);
    pinMode(this->LPWM_PIN, OUTPUT);
}

void TomIBT2::rotate(int speed, Direction direction) {
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);

    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if (speed < 0) {
        speed = 0;
    }

    this->currentDirection = direction;
    this->currentSpeed = speed;

    int pwmPin = (this->currentDirection == CW) ? this->RPWM_PIN : this->LPWM_PIN;

    analogWrite(pwmPin, this->currentSpeed);
}

void TomIBT2::stop(void) {
    digitalWrite(R_EN_PIN, LOW);
    digitalWrite(L_EN_PIN, LOW);

    this->currentSpeed = 0;

    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);
}

void TomIBT2::brake(void) {
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);

    this->currentSpeed = 0;

    analogWrite(RPWM_PIN, 0);
    analogWrite(LPWM_PIN, 0);    
}

int TomIBT2::getCurrentSpeed(void) {
    return this->currentSpeed;
}

int TomIBT2::getCurrentSpeedPercent(void) {
    return int((this->currentSpeed / MAX_SPEED * 100.0));
}

TomIBT2::Direction TomIBT2::getCurrentDirection(void) {
    return this->currentDirection;
}

void TomIBT2::setTargetSpeed(int speed) {
    if (speed > MAX_SPEED) {
        speed = MAX_SPEED;
    } else if (speed < 0) {
        speed = 0;
    }

    this->targetSpeed = speed;
}

void TomIBT2::setTargetSpeedPercent(int percent) {
    if (percent > 100) {
        percent = 100;
    } else if (percent < 0) {
        percent = 0;
    }

    this->targetSpeed = map(percent, 0, 100, 0, 255);
}

void TomIBT2::rampUp(Direction direction,  unsigned long timeoutMs) {
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);

    this->currentDirection = direction;

    int pwmPin = (this->currentDirection == CW) ? this->RPWM_PIN : this->LPWM_PIN;

    this->currentMillis = millis();

    if (this->currentMillis - this->previousMillis >= timeoutMs / MAX_SPEED) {
        this->previousMillis = this->currentMillis;
        this->currentSpeed++;

        if (this->currentSpeed >= this->targetSpeed) {
            this->currentSpeed = this->targetSpeed;
        }

        analogWrite(pwmPin, this->currentSpeed);
    }
}

void TomIBT2::rampDown(unsigned long timeoutMs) {
    digitalWrite(R_EN_PIN, HIGH);
    digitalWrite(L_EN_PIN, HIGH);

    int pwmPin = (this->currentDirection == CW) ? this->RPWM_PIN : this->LPWM_PIN;

    this->currentMillis = millis();

    if (this->currentMillis - this->previousMillis >= timeoutMs / MAX_SPEED) {
        this->previousMillis = this->currentMillis;
        
        if (this->currentSpeed > this->targetSpeed) {
            this->currentSpeed--;

            if (this->currentSpeed <= this->targetSpeed) {
                this->currentSpeed = 0;
                stop();
            }
        }

        analogWrite(pwmPin, this->currentSpeed);
    }
}
