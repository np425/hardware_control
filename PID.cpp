#include "PID.h"

PID::PID(float kp, float ki, float kd, float minOutput, float maxOutput) {
    _minOutput = minOutput;
    _maxOutput = maxOutput;
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _integral = 0;
    _previousError = 0;
    _previousDerivativeAvg = 0;
    _lastUpdateMillis = millis();
}

float PID::compute(float setpoint, float currentValue) {
    float dt = (millis() - _lastUpdateMillis) / 1000.0;
    _lastUpdateMillis = millis();

    if (dt == 0) {
        dt = 0.0001;
    } else if (dt > 1) {
        _integral = 0;
        _previousDerivativeAvg = 0;
        _previousError = 0;
        return 0;
    }
    
    float error = setpoint - currentValue;
    float proportional = _kp * error;

    _integral += error * dt;
    float integralTerm = constrain(_ki * _integral, -_maxOutput, _maxOutput);

    float derivative = (error - _previousError) / dt;
    float derivativeAvg = (_derivativeAvgCount * _previousDerivativeAvg + derivative) / (_derivativeAvgCount + 1);

    _previousDerivativeAvg = derivativeAvg;
    float derivativeTerm = _kd * derivativeAvg;

    // Calculate output
    float output = proportional + integralTerm + derivativeTerm;
    output = constrain(output, _minOutput, _maxOutput);

    // Serial.print(" | ");
    // Serial.print(proportional);
    // Serial.print(" + ");
    // Serial.print(integralTerm);
    // Serial.print(" + ");
    // Serial.print(derivativeTerm);
    // Serial.print(" = ");
    // Serial.print(output);

    _previousError = error;
    return output;
}
