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
    unsigned long nowMicros = micros();
    if (nowMicros - _lastPulseMicros > 100000) {
        return 0.0;
    }

    double pulseTimeSec = (_pulseWidthMicrosSum / (double)COUNTS_PER_REV) / 1000000.0;
    double velocity = (pulseTimeSec > 0) ? (ANGLE_PER_PULSE / pulseTimeSec) : 0;
    
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
        _direction = Clockwise;
    } else if(sum == 0b0010 || sum == 0b0100 || sum == 0b1101 || sum == 0b1011) {
        _pulseCount--;
        _direction = CounterClockwise;
    } else {
        _direction = None;
    }

    if (_direction != None) {
        double pulseWidthMicros = nowMicros - _lastPulseMicros;
        _lastPulseMicros = nowMicros;

        _pulseWidthMicrosSum = _pulseWidthMicrosSum - _pulseWidthsMicros[_oldestPulseIndex];
        _pulseWidthsMicros[_oldestPulseIndex] = pulseWidthMicros;
        _pulseWidthMicrosSum = _pulseWidthMicrosSum + pulseWidthMicros;
        _oldestPulseIndex = (_oldestPulseIndex + 1) % COUNTS_PER_REV;
    }

    _lastEncoded = encoded;
}
