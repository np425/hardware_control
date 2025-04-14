#include "Encoder.h"

Encoder::Encoder(uint8_t pinA, uint8_t pinB) {
    _pinA = pinA;
    _pinB = pinB;
}

void Encoder::setup() {
    pinMode(_pinA, INPUT_PULLUP);
    pinMode(_pinB, INPUT_PULLUP);
}

long Encoder::getPulseCount() {
    return _pulseCount;
}

float Encoder::getPosition_rad() {
    return ((float)_pulseCount / PULSES_PER_REV) * (2.0 * PI);
}
  
float Encoder::getVelocity_rad_s() {
    constexpr long US_TO_S = 1000000;

    unsigned long now_us = micros();

    if (_pulseWidth_us == 0.0 || now_us - _prevPulse_us > US_TO_S) return 0.0;

    float speed_rad_s = (2.0f * PI * US_TO_S) / ((float)PULSES_PER_REV * _pulseWidth_us);
    return speed_rad_s * _direction;
}

bool Encoder::getDirection() {
    return _direction;
}

void Encoder::readPulse() {
    noInterrupts();
    bool b = digitalRead(_pinB);
    unsigned long now_us = micros();
    interrupts();

    if (b) {
        ++_pulseCount;
        _direction = DIRECTION_FORWARD;
    } else {
        --_pulseCount;
        _direction = DIRECTION_BACKWARD;
    }

    unsigned long pulseWidth_us = now_us - _prevPulse_us;
    _pulseWidth_us = (_pulseWidth_us * (AVG_WINDOW_SIZE-1) + pulseWidth_us) / AVG_WINDOW_SIZE;
    _prevPulse_us = now_us;
}
