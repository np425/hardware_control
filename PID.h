#pragma once

#include <Arduino.h>
#include "PID.h"

class PID {
public:
    PID(float kp, float ki, float kd, float outputMin = 0, float outputMax = 255);
    float compute(float setpoint, float currentValue);

    float minOutput;
    float maxOutput;
    float kp;
    float ki;
    float kd;

private:
    constexpr static unsigned DERIVATIVE_WINDOW_SIZE = 50;

    float _prevError = 0;      
    float _prevIntegral = 0;           
    float _prevDerivative = 0;
    unsigned long _prevUpdate_ms = millis(); 
};
