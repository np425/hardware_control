#include "PID.h"

PID::PID(float kp, float ki, float kd, float minOutput, float maxOutput) {
    _minOutput = minOutput;
    _maxOutput = maxOutput;
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

float PID::compute(float setpoint, float currentValue) {
    unsigned long now_ms = millis();
    float timeElapsed_s = (now_ms - _prevUpdate_ms) / 1000.0;
    _prevUpdate_ms = now_ms;

    if (timeElapsed_s < 0.00001) {
        timeElapsed_s = 0.0001;
    } else if (timeElapsed_s > 1) {
        // Reset after pause
        _prevIntegral = 0;
        _prevDerivative = 0;
        _prevError = 0;
        return 0;
    }
    
    float error = setpoint - currentValue;
    float proportionalTerm = _kp * error;

    _prevIntegral += error * timeElapsed_s;
    float integralTerm = _ki * _prevIntegral;
    // float integralTerm = constrain(_ki * _prevIntegral, -_maxOutput, +_maxOutput);

    float derivative = (error - _prevError) / timeElapsed_s;
    // float derivativeAvg = (DERIVATIVE_WINDOW_SIZE * _prevDerivative + derivative) / (DERIVATIVE_WINDOW_SIZE + 1);
    // _prevDerivative = derivativeAvg;

    float derivativeTerm = _kd * derivative;

    // Calculate output
    float output = proportionalTerm + integralTerm + derivativeTerm;
    output = constrain(output, _minOutput, _maxOutput);

    _prevError = error;
    return output;
}
