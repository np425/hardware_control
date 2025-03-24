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

float Encoder::getPosition_rad() {
    return ((float)_pulseCount / COUNTS_PER_REV) * (2.0 * PI);
}
  
float Encoder::getVelocity_rad_s() {
    unsigned long nowMeasure_millis = millis();
    float nowPosition_rad = getPosition_rad();

    float elapsed_seconds = (nowMeasure_millis - _prevSpeedMeasure_millis) / 1000.0;
    float distance_rad = (nowPosition_rad - _prevPositionMeasure_rad);
    float velocity_rad_s = distance_rad / elapsed_seconds;

    _prevPositionMeasure_rad = nowPosition_rad;
    _prevSpeedMeasure_millis = nowMeasure_millis;

    return velocity_rad_s;
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

    _lastEncoded = encoded;
}
