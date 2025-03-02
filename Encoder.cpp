#include "Encoder.h"

Encoder::Encoder(byte pin_a, byte pin_b) {
    _pin_a = pin_a;
    _pin_b = pin_b;
}

void Encoder::setup() {
    pinMode(_pin_a, INPUT_PULLUP);
    pinMode(_pin_b, INPUT_PULLUP);
}

long Encoder::getPulseCount() {
    return _pulseCount;
}

double Encoder::getPosition_rad() {
    return ((double)_pulseCount / COUNTS_PER_REV) * (2.0 * PI);
}

double Encoder::getVelocity_rad_s() {
    double pulseTimeSec = _lastPulseWidthMicros / 1'000'000.0;
    double anglePerPulse = (2.0 * PI) / COUNTS_PER_REV;

    double velocity = ANGLE_PER_PULSE / pulseTimeSec;
    
    if (_direction == Encoder::CounterClockwise) {
        velocity = -velocity;
    }
    return velocity;
}

Encoder::Direction Encoder::getDirection() {
    return _direction;
}

void Encoder::readPulse() {
    unsigned long nowMicros = micros();

    int a = digitalRead(_pin_a);
    int b = digitalRead(_pin_b);
    int encoded = (a << 1) | b;
    
    int sum = (_lastEncoded << 2) | encoded;

    if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
        _pulseCount++;
        _currentDirection = Encoder::Clockwise;

        _lastPulseWidthMicros = nowMicros - _lastPulseMicros;
        _lastPulseMicros = nowMicros;
    } else if(sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011) {
        _pulseCount--;
        _currentDirection = Encoder::CounterClockwise;

        _lastPulseWidthMicros = nowMicros - _lastPulseMicros;
        _lastPulseMicros = nowMicros;
    } else {
        _currentDirection = Encoder::None;
    }

    _lastEncoded = encoded;
}