#pragma once

#include <Arduino.h>

class Encoder {
public:
    constexpr static int PULSES_PER_REV = 36;
    constexpr static double ANGLE_PER_PULSE = (2.0 * PI) / PULSES_PER_REV;
    constexpr static int AVG_WINDOW_SIZE = 4;

    enum Direction {
        Clockwise = 1,
        CounterClockwise = -1,
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
    volatile Direction _direction = Clockwise;
    volatile unsigned long _lastPulse_micros = micros();
    volatile unsigned long _pulseWidthAvg_micros = 0;
};
