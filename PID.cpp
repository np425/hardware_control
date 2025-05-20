#include "PID.h"

PID::PID(float kp, float ki, float kd, float minOut, float maxOut)
    : kp(kp), ki(ki), kd(kd), minOutput(minOut), maxOutput(maxOut) {}

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
    float proportionalTerm = kp * error;

    // Integral clamping to prevent anti-windup
    _prevIntegral += error * timeElapsed_s;
    if (abs(ki) > 1e-6) { // Check if ki is practically non-zero
         _prevIntegral = constrain(_prevIntegral, minOutput / ki, maxOutput / ki);
    } else {
         _prevIntegral = 0; // Or handle appropriately if ki is zero
    }
    float integralTerm = ki * _prevIntegral;

    float derivative = (error - _prevError) / timeElapsed_s;
    float derivativeAvg = (DERIVATIVE_WINDOW_SIZE * _prevDerivative + derivative) / (DERIVATIVE_WINDOW_SIZE + 1);
    _prevDerivative = derivativeAvg;

    float derivativeTerm = kd * derivativeAvg;

    // Calculate output
    float output = proportionalTerm + integralTerm + derivativeTerm;
    output = constrain(output, minOutput, maxOutput);

    _prevError = error;
    return output;
}
