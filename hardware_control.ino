#include "Motor.h"
#include "Encoder.h"
#include "PID.h"


const uint8_t PIN_LMOTOR_L_PWM = 6; 
const uint8_t PIN_LMOTOR_R_PWM = 7; 
const uint8_t PIN_LMOTOR_L_EN = 22;
const uint8_t PIN_LMOTOR_R_EN = 23;

const uint8_t PIN_RMOTOR_L_PWM = 11; 
const uint8_t PIN_RMOTOR_R_PWM = 12; 
const uint8_t PIN_RMOTOR_L_EN = 24;
const uint8_t PIN_RMOTOR_R_EN = 25;

const uint8_t PIN_LENCODER_A_INT = 18;
const uint8_t PIN_LENCODER_B_INT = 19;

const uint8_t PIN_RENCODER_A_INT = 20;
const uint8_t PIN_RENCODER_B_INT = 21;

const uint8_t PIN_UV_LAMP = 39;

// const float PID_KP = 3.0;
// const float PID_KI = 8.0;
// const float PID_KD = 2.5;

const unsigned int RATE_PID_HZ = 100;

const unsigned long RATE_BAUD_BPS = 115200;

const float UV_RADIANT_POWER = 5.37;


// Motors run at 490.20Hz frequency
Motor _motorL(PIN_LMOTOR_L_EN, PIN_LMOTOR_R_EN, PIN_LMOTOR_L_PWM, PIN_LMOTOR_R_PWM);
Motor _motorR(PIN_RMOTOR_L_EN, PIN_RMOTOR_R_EN, PIN_RMOTOR_L_PWM, PIN_RMOTOR_R_PWM);

Encoder _encoderL(PIN_LENCODER_A_INT, PIN_LENCODER_B_INT);
Encoder _encoderR(PIN_RENCODER_A_INT, PIN_RENCODER_B_INT);

float _pid_kp = 5.0;
float _pid_ki = 40.0;
float _pid_kd = 0.0;

PID _pidMotorL(_pid_kp, _pid_ki, _pid_kd, -255, 255);
PID _pidMotorR(_pid_kp, _pid_ki, _pid_kd, -255, 255);

bool _enablePID = false;
float _speedMotorL_radS = 0;
float _speedMotorR_radS = 0;

String _cmd;


void updateEncoderReadingL() { _encoderL.readPulse(); }
void updateEncoderReadingR() { _encoderR.readPulse(); }

void displayEncoderData() {
    float positionL_rad = _encoderL.getPosition_rad();
    float velocityL_radS = _encoderL.getVelocity_rad_s();

    float positionR_rad = _encoderR.getPosition_rad();
    float velocityR_radS = _encoderR.getVelocity_rad_s();

    Serial.print(positionL_rad, 3); Serial.print(",");
    Serial.print(positionR_rad, 3); Serial.print(",");
    Serial.print(velocityL_radS, 3); Serial.print(",");
    Serial.println(velocityR_radS, 3);
}


void extractValues(String string, char delimiter, String* values, unsigned int count) {
    int start = 0;
    int end = 0;

    for (unsigned int i = 0; i < count-1; ++i) {
        end = string.indexOf(delimiter, start);
        if (end == -1) {
            values[i] = string.substring(start);
            return;
        } 

        values[i] = string.substring(start, end);
        start = end + 1;
    }

    // Last value gets the rest
    values[count-1] = string.substring(start);
}

bool setParameter(String name, String value) {
    String parts[2];
    extractValues(value, ' ', parts, 2);

    if (name == "speed_rad_s") {
        float l = parts[0].toFloat();
        float r = parts[1].toFloat();

        _speedMotorL_radS = l;
        _speedMotorR_radS = r;
        _enablePID = true;
    } else if (name == "speed_pwm") {
        int l = parts[0].toInt();
        int r = parts[1].toInt();

        bool moveForwardL = l >= 0;
        bool moveForwardR = r >= 0;

        _enablePID = false;
        _motorL.enableRotation_pwm(abs(l), moveForwardL);
        _motorR.enableRotation_pwm(abs(r), moveForwardR);
    } else if (name == "uv_state") {
        String state = parts[0];
        if (state == "1") {
            digitalWrite(PIN_UV_LAMP, HIGH);
        } else {
            digitalWrite(PIN_UV_LAMP, LOW);
        }
    } else if (name == "pid_kp") {
        _pid_kp = parts[0].toFloat();
        _pidMotorL.kp = _pid_kp;
        _pidMotorR.kp = _pid_kp;
    } else if (name == "pid_ki") {
        _pid_ki = parts[0].toFloat();
        _pidMotorL.ki = _pid_ki;
        _pidMotorR.ki = _pid_ki;
    } else if (name == "pid_kd") {
        _pid_kd = parts[0].toFloat();
        _pidMotorL.kd = _pid_kd;
        _pidMotorR.kd = _pid_kd;
    } else return false;

    return true;
}

bool executeCommand(String cmd) {
    String parts[3];

    extractValues(cmd, ' ', parts, 3);
    String action = parts[0];
    String name = parts[1];

    bool success = true;

    if (action == "set") {
        String value = parts[2];

        success = setParameter(name, value);
        if (success) {
            Serial.println("OK");
        }
    } else if (action == "get") {
        if (name == "encoder") {
            displayEncoderData();
        } else if (name == "uv_state") {
            Serial.print(UV_RADIANT_POWER); Serial.print(",");
            Serial.println(digitalRead(PIN_UV_LAMP));
        } else if (name == "pid") {
            Serial.print(_pid_kp); Serial.print(",");
            Serial.print(_pid_ki); Serial.print(",");
            Serial.println(_pid_kd);
        } else success = false;
    } else success = false;

    return success;
}


void setup() {
    Serial.begin(RATE_BAUD_BPS, SERIAL_8N1);
    while (!Serial) {}

    _motorL.setup();
    _motorR.setup();

    _encoderL.setup();
    attachInterrupt(digitalPinToInterrupt(PIN_LENCODER_A_INT), updateEncoderReadingL, RISING);

    _encoderR.setup();
    attachInterrupt(digitalPinToInterrupt(PIN_RENCODER_A_INT), updateEncoderReadingR, RISING);

    pinMode(PIN_UV_LAMP, OUTPUT);
    digitalWrite(PIN_UV_LAMP, LOW);

    _cmd.reserve(64);
}


void loop() {
    static unsigned long lastPIDUpdate_ms = millis();
    bool readyCmd = false;

    char c;
    while (Serial.available() > 0 && !readyCmd) {
        c = Serial.read();
        readyCmd = (c == '\n');
        if (!readyCmd) _cmd += c;
    }

    if (readyCmd) {
        executeCommand(_cmd);
        _cmd.remove(0, _cmd.length());
    }

    // PID loop
    unsigned long now_ms = millis();
    if (_enablePID && now_ms - lastPIDUpdate_ms > 1000/RATE_PID_HZ) {
        float measured_radS, calculated_pwm;
        bool direction;
        
        measured_radS = _encoderL.getVelocity_rad_s();
        if (fabs(_speedMotorL_radS) <= 0.0001 && fabs(measured_radS) <= 1) {
            _motorL.enableRotation_pwm(0, true);
        } else {
            calculated_pwm = round(_pidMotorL.compute(_speedMotorL_radS, measured_radS));
            direction = calculated_pwm >= 0;
            _motorL.enableRotation_pwm((uint8_t)fabs(calculated_pwm), direction);

            Serial.print(millis());
            Serial.print(",");
            Serial.print(_speedMotorL_radS);
            Serial.print(",");
            Serial.println(measured_radS);
        }

        measured_radS = _encoderR.getVelocity_rad_s();
        if (fabs(_speedMotorR_radS) <= 0.0001 && fabs(measured_radS) <= 1) {
            _motorR.enableRotation_pwm(0, true);
        } else {
            calculated_pwm = round(_pidMotorR.compute(_speedMotorR_radS, measured_radS));
            direction = calculated_pwm >= 0;
            _motorR.enableRotation_pwm((uint8_t)fabs(calculated_pwm), direction);

            // Serial.print(calculated_pwm);
            // Serial.print(",");
            // Serial.print(_speedMotorR_radS);
            // Serial.print(",");
            // Serial.println(measured_radS);
        }

        lastPIDUpdate_ms = now_ms;
    }
}
