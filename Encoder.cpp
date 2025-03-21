#include "Encoder.h"

Encoder::Encoder(byte pin_a, byte pin_b) {
    _pin_a = pin_a;
    _pin_b = pin_b;
}

void Encoder::setup() {
    pinMode(_pin_a, INPUT_PULLUP);
    pinMode(_pin_b, INPUT_PULLUP);
}

unsigned long Encoder::getPulseCount() {
    return _pulseCount;
}

float Encoder::getPosition_rad() {
    return ((float)_pulseCount / COUNTS_PER_REV) * (2.0 * PI);
}
  
float Encoder::getVelocity_rad_s() {
    unsigned long nowMillis = millis();
    if (nowMillis - _lastPulseMillis > 100) {
        return 0.0;
    }

    float pulseTimeSec = (_pulseWidthMillisSum / (float)COUNTS_PER_REV) / 1000000.0;
    float velocity = (pulseTimeSec > 0) ? (ANGLE_PER_PULSE / pulseTimeSec) : 0;
    
    if (_direction == Encoder::CounterClockwise) {
        velocity = -velocity;
    }
    return velocity;
}

Encoder::Direction Encoder::getDirection() {
    return _direction;
}

void Encoder::readPulse() {
    unsigned long nowMillis = millis();

    byte a = digitalRead(_pin_a);
    byte b = digitalRead(_pin_b);
    byte encoded = (a << 1) | b;
    
    int sum = (_lastEncoded << 2) | encoded;

    if(sum == 0b0001 || sum == 0b0111 || sum == 0b1110 || sum == 0b1000) {
        _pulseCount++;
        _direction = Clockwise;
    } else if(sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011) {
        _pulseCount--;
        _direction = CounterClockwise;
    } else {
        _direction = None;
    }

    if (_direction != None) {
        unsigned long pulseWidthMillis = nowMillis - _lastPulseMillis;
        _lastPulseMillis = nowMillis;

        _pulseWidthMillisSum = _pulseWidthMillisSum - _pulseWidthsMillis[_oldestPulseIndex];
        _pulseWidthsMillis[_oldestPulseIndex] = pulseWidthMillis;
        _pulseWidthMillisSum = _pulseWidthMillisSum + pulseWidthMillis;
        _oldestPulseIndex = (_oldestPulseIndex + 1) % sizeof(_pulseWidthsMillis);
    }

    _lastEncoded = encoded;
}
