#pragma once

#include <Arduino.h>

class Encoder {
public:
    constexpr static bool DIRECTION_FORWARD = true;
    constexpr static bool DIRECTION_BACKWARD = false;

    constexpr static unsigned PULSES_PER_REV = 36;
    constexpr static float ANGLE_PER_PULSE = (2.0 * PI) / PULSES_PER_REV;
    constexpr static unsigned AVG_WINDOW_SIZE = 1;

    Encoder(uint8_t pinA, uint8_t pinB);
    void setup();

    long getPulseCount();
    float getPosition_rad();
    float getVelocity_rad_s();

    bool getDirection();
    void readPulse();


private:
    uint8_t _pinA;
    uint8_t _pinB;

    volatile long _pulseCount = 0;
    volatile uint8_t _lastEncoded = 0;
    volatile bool _direction = DIRECTION_FORWARD;
    volatile unsigned long _prevPulse_us = micros();
    volatile unsigned long _pulseWidth_us = 0;
};
