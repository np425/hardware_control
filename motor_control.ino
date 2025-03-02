#include <TomIBT2.h>

const int PIN_LMOTOR_L_PWM = 2; // 490.20Hz
const int PIN_LMOTOR_R_PWM = 3; // 490.20Hz
const int PIN_LMOTOR_L_EN = 22;
const int PIN_LMOTOR_R_EN = 23;

const int PIN_RMOTOR_L_PWM = 6; // 490.20Hz
const int PIN_RMOTOR_R_PWM = 7; // 490.20Hz
const int PIN_RMOTOR_L_EN = 24;
const int PIN_RMOTOR_R_EN = 25;

TomIBT2 motor_l(PIN_LMOTOR_R_EN, PIN_LMOTOR_L_EN, PIN_LMOTOR_R_PWM, PIN_LMOTOR_L_PWM);
TomIBT2 motor_r(PIN_RMOTOR_R_EN, PIN_RMOTOR_L_EN, PIN_RMOTOR_R_PWM, PIN_RMOTOR_L_PWM);

void setup() {
    motor_l.begin();
    motor_l.setTargetSpeed(255);
    //motor_l.rotate(255, TomIBT2::CW);
    //motor_l.rampUp(TomIBT2::CW, 1000);
}

void loop() {
    if (Serial.available()) {
      
    }
  
    motor_l.rampUp(TomIBT2::CCW, 1000);
}
