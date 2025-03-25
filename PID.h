#pragma once

#include <Arduino.h>
#include "PID.h"

class PID {
  private:
    float _minOutput;
    float _maxOutput;
    float _integral;           
    float _previousError;      
    float _kp;
    float _ki;
    float _kd;
    unsigned long _lastUpdateMillis; 
    float _previousDerivativeAvg;
    const unsigned long _derivativeAvgCount = 50;
  
  public:
    PID(float kp, float ki, float kd, float minOutput = 0, float maxOutput = 255);
    float compute(float setpoint, float currentValue);
};
