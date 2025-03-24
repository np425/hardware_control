#pragma once

#include <Arduino.h>

class Encoder {
public:
    constexpr static int COUNTS_PER_REV = 144;
    constexpr static double ANGLE_PER_PULSE = (2.0 * PI) / COUNTS_PER_REV;

    enum Direction {
        Clockwise = 0,
        CounterClockwise = 1,
        None = 2
    };

    Encoder(byte pin_a, byte pin_b);
    void setup();

    long getPulseCount();
    float getPosition_rad();
    float getVelocity_rad_s();

    Direction getDirection();
    void readPulse();

private:
    byte _pin_a;
    byte _pin_b;

    volatile long _pulseCount = 0;
    volatile byte _lastEncoded = 0;
    volatile Direction _direction = None;
    volatile unsigned long _lastPulseMillis = 0;
    volatile unsigned long _pulseWidthsMillis[COUNTS_PER_REV] = {0};
    volatile byte _oldestPulseIndex = 0;
    volatile unsigned long _pulseWidthMillisSum = 0;
};
