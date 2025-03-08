#include "Motor.h"
#include "Encoder.h"

const byte PIN_LMOTOR_L_PWM = 6; // 490.20Hz
const byte PIN_LMOTOR_R_PWM = 7; // 490.20Hz
const byte PIN_LMOTOR_L_EN = 22;
const byte PIN_LMOTOR_R_EN = 23;

const byte PIN_RMOTOR_L_PWM = 11; // 490.20Hz
const byte PIN_RMOTOR_R_PWM = 12; // 490.20Hz
const byte PIN_RMOTOR_L_EN = 24;
const byte PIN_RMOTOR_R_EN = 25;

const byte PIN_SERVO_ACTUATOR = 30;

const byte PIN_LENCODER_A_INT = 18;
const byte PIN_LENCODER_B_INT = 19;

const byte PIN_RENCODER_A_INT = 20;
const byte PIN_RENCODER_B_INT = 21;

Motor _motor_l(PIN_LMOTOR_L_EN, PIN_LMOTOR_R_EN, PIN_LMOTOR_L_PWM, PIN_LMOTOR_R_PWM);
Motor _motor_r(PIN_RMOTOR_L_EN, PIN_RMOTOR_R_EN, PIN_RMOTOR_L_PWM, PIN_RMOTOR_R_PWM);

Encoder _encoder_l(PIN_LENCODER_A_INT, PIN_LENCODER_B_INT);
Encoder _encoder_r(PIN_RENCODER_A_INT, PIN_RENCODER_B_INT);

String _cmd = "";
bool _cmdReady = false;

void encoder_l_update() {
    _encoder_l.readPulse();
}

void encoder_r_update() {
    _encoder_r.readPulse();
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            _cmdReady = true;
        } else {
            _cmd += inChar;
        }
    }
}

bool setParameter(String name, String value) {
    bool success = false;
    if (name == "MOT_l_speed_rad_s") {
        double speed_rad_s = atof(value.c_str());
        _motor_l.rotate_rad_s(speed_rad_s);
        success = true;
    } else if (name == "MOT_r_speed_rad_s") {
        double speed_rad_s = atof(value.c_str());
        _motor_r.rotate_rad_s(speed_rad_s);
        success = true;      
    }
    return success;
}

bool executeCommand(String cmd) {
    int endPos = cmd.indexOf(' ');
    String action = cmd.substring(0, endPos);
    int startPos = endPos + 1;
    bool success = false;

    if (action == "set") {
        endPos = cmd.indexOf(' ', startPos);
        String name = cmd.substring(startPos, endPos);
        startPos = endPos + 1;
            
        endPos = cmd.indexOf(' ', startPos);
        String value = cmd.substring(startPos, endPos);
        startPos = endPos + 1;

        success = setParameter(name, value);
    }

    return success;
}

void displayEncoderData() {
    double position_l = _encoder_l.getPosition_rad();
    double position_r = _encoder_r.getPosition_rad();

    double velocity_l = _encoder_l.getVelocity_rad_s();
    double velocity_r = _encoder_r.getVelocity_rad_s();

    Serial.print("ENC_l,");
    Serial.print(position_l, 6);
    Serial.print(",");
    Serial.println(velocity_l, 6); 

    Serial.print("ENC_r,");
    Serial.print(position_r, 6);
    Serial.print(",");
    Serial.println(velocity_r, 6);
}

void setup() {
    Serial.begin(230400);
    while (!Serial) {}
  
    _motor_l.setup();
    _motor_r.setup();

    _encoder_l.setup();
    _encoder_r.setup();

    attachInterrupt(digitalPinToInterrupt(PIN_LENCODER_A_INT), encoder_l_update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_LENCODER_B_INT), encoder_l_update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RENCODER_A_INT), encoder_r_update, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_RENCODER_B_INT), encoder_r_update, CHANGE);
}

void loop() {
    if (_cmdReady) {
        executeCommand(_cmd);
        _cmd = "";
        _cmdReady = false;
    }

    displayEncoderData();
    delay(100);
}
