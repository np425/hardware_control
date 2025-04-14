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
    return ((float)_pulseCount / PULSES_PER_REV) * (2.0 * PI);
}
  
float Encoder::getVelocity_rad_s() {
    unsigned long now_micros = micros();
    if (_pulseWidthAvg_micros == 0.0 || now_micros - _lastPulse_micros > 1000000.0) return 0.0;
    float speed_rad_s = (2 * PI * 1000000) / (PULSES_PER_REV * _pulseWidthAvg_micros) /* * _direction */;
    return speed_rad_s * _direction;
}

Encoder::Direction Encoder::getDirection() {
    return _direction;
}

void Encoder::readPulse() {
    noInterrupts();
    //bool a = digitalRead(_pin_a);
    bool b = digitalRead(_pin_b);
    // Serial.print(a);
    // Serial.print(" ");
    //Serial.println(b);
    unsigned long now_micros = micros();
    interrupts();

    if (b) {
        //Serial.println("b++");
        ++_pulseCount;
        _direction = Clockwise;
    }
    else {
        //Serial.println("b--");
        --_pulseCount;
        _direction = CounterClockwise;
    }

    unsigned long pulseWidth_micros = now_micros - _lastPulse_micros;
    _pulseWidthAvg_micros = (_pulseWidthAvg_micros * (AVG_WINDOW_SIZE-1) + pulseWidth_micros) / AVG_WINDOW_SIZE;
    _lastPulse_micros = now_micros;
}
