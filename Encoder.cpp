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

    double velocity_rad_s = _velocity * ANGLE_PER_PULSE;
    
    if (_direction == Encoder::CounterClockwise) {
        velocity_rad_s = -velocity_rad_s;
    }
    return velocity_rad_s;
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
    
    if(_direction != None) {
        // Low-pass filter update
        _lastPulseWidthMicros = nowMicros - _lastPulseMicros;
        _lastPulseMicros = nowMicros;

        double v_inst = (_lastPulseWidthMicros > 0) ? (1e6 / _lastPulseWidthMicros) : 0.0;
        const double alpha = 0.1;
        _velocity = _velocity + alpha * (v_inst - _velocity);
    }
    
    _lastEncoded = encoded;
}
