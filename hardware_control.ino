#include "Motor.h"
#include "Encoder.h"
#include "PID.h"

const unsigned long RATE_BAUD_BPS = 115200;

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

const float PID_KP = 30.0;
const float PID_KI = 1.0;
const float PID_KD = 5.0;
const unsigned int RATE_PID_HZ = 100;

Motor _motor_l(PIN_LMOTOR_L_EN, PIN_LMOTOR_R_EN, PIN_LMOTOR_L_PWM, PIN_LMOTOR_R_PWM);
Motor _motor_r(PIN_RMOTOR_L_EN, PIN_RMOTOR_R_EN, PIN_RMOTOR_L_PWM, PIN_RMOTOR_R_PWM);

Encoder _encoder_l(PIN_LENCODER_A_INT, PIN_LENCODER_B_INT);
Encoder _encoder_r(PIN_RENCODER_A_INT, PIN_RENCODER_B_INT);

PID _pid_motor_l(PID_KP, PID_KI, PID_KD);
PID _pid_motor_r(PID_KP, PID_KI, PID_KD);

float _set_motor_l_speed_rad_s = 0;
float _set_motor_r_speed_rad_s = 0;
bool _pwm_controlled = true;

String _cmd = "";
bool _cmdReady = false;

void encoder_l_update() {
    _encoder_l.readPulse();
}

void encoder_r_update() {
    _encoder_r.readPulse();
}

void serialEvent() {
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            _cmdReady = true;
            break;
        } else {
            _cmd += inChar;
        }
    }
}

void displayEncoderData() {
    float position_l = _encoder_l.getPosition_rad();
    float position_r = _encoder_r.getPosition_rad();

    float velocity_l = _encoder_l.getVelocity_rad_s();
    float velocity_r = _encoder_r.getVelocity_rad_s();

    Serial.print(position_l, 3);
    Serial.print(",");
    Serial.print(position_r, 3);
    Serial.print(",");
    Serial.print(velocity_l, 3); 
    Serial.print(",");
    Serial.println(velocity_r, 3);
}

bool setParameter(String name, String value) {
    bool success = false;
    if (name == "MOT_l_speed_rad_s") {
        _pwm_controlled = false;
        _set_motor_l_speed_rad_s = atof(value.c_str());
        if (_set_motor_l_speed_rad_s == 0.0) {
            _motor_l.rotate_pwm(0);
        }
        success = true;
    } else if (name == "MOT_l_speed_pwm") {
        _pwm_controlled = true;
        int pwm = atoi(value.c_str());
        _motor_l.rotate_pwm(pwm);
        success = true;
    } else if (name == "MOT_r_speed_rad_s") {
        _pwm_controlled = false;
        _set_motor_r_speed_rad_s = atof(value.c_str());
        if (_set_motor_r_speed_rad_s == 0.0) {
            _motor_r.rotate_pwm(0);
        }
        success = true;      
    } else if (name == "MOT_r_speed_pwm") {
        _pwm_controlled = true;
        int pwm = atoi(value.c_str());
        _motor_r.rotate_pwm(pwm); 
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
        if (success) {
            Serial.println("OK");
        }
    } else if (action == "get") {
        displayEncoderData();
    }

    return success;
}

void setup() {
    Serial.begin(115200, SERIAL_8N1);
    while (!Serial) {}

    _motor_l.setup();
    _motor_r.setup();

    _encoder_l.setup();
    _encoder_r.setup();

    attachInterrupt(digitalPinToInterrupt(PIN_LENCODER_A_INT), encoder_l_update, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_RENCODER_A_INT), encoder_r_update, RISING);
}

void loop() {
    static unsigned long lastPIDUpdateMillis = millis();

    if (_cmdReady) {
        executeCommand(_cmd);
        _cmd = "";
        _cmdReady = false;
    }

    unsigned long nowMillis = millis();
    if (!_pwm_controlled && nowMillis - lastPIDUpdateMillis > 1000/RATE_PID_HZ) {
        float measured_speed_rad_s;
        int calculated_speed_pwm;

        // float l_speed_rad_s = _encoder_l.getVelocity_rad_s();
        // float r_speed_rad_s = _encoder_r.getVelocity_rad_s();

        // Serial.print("[L] Measured (rad/s): ");
        // Serial.print(l_speed_rad_s);
        // Serial.print(", ");
        // Serial.print("[R] Measured (rad/s): ");
        // Serial.println(r_speed_rad_s);

        if (_set_motor_l_speed_rad_s != 0.0) {
            measured_speed_rad_s = _encoder_l.getVelocity_rad_s();
            calculated_speed_pwm = (int)round(_pid_motor_l.compute(_set_motor_l_speed_rad_s, measured_speed_rad_s));
            _motor_l.rotate_pwm(calculated_speed_pwm);

            // Serial.print("[L] Measured (rad/s): ");
            // Serial.print(measured_speed_rad_s);
            // Serial.print(", ");
            // Serial.print("Calculated (PWM): ");
            // Serial.println(calculated_speed_pwm);
        }
        if (_set_motor_r_speed_rad_s != 0.0) {            
            measured_speed_rad_s = _encoder_r.getVelocity_rad_s();
            calculated_speed_pwm = (int)round(_pid_motor_r.compute(_set_motor_r_speed_rad_s, measured_speed_rad_s));
            _motor_r.rotate_pwm(calculated_speed_pwm);

            // Serial.print("[R] Measured (rad/s): ");
            // Serial.print(measured_speed_rad_s);
            // Serial.print(", ");
            // Serial.print("Calculated (PWM): ");
            // Serial.println(calculated_speed_pwm);
        }

        lastPIDUpdateMillis = nowMillis;
    }
}
