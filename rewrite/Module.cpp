#include "Module.h"

const float MOTOR_TO_SUN_RATIO = 0.5;
const float DIFFERENTIAL_RATIO = 1.0;

Module::Module(Motor* top, Motor* bottom, Azimuth* az, float offset) {
    topMotor = top;
    bottomMotor = bottom;
    azimuth = az;
    driveVelocity = 0.0;
    targetAngle = 0.0;
    tuningState = NOT_TUNING;
    lastUpdateTime = millis();
    this->offset = offset;
}

void Module::init() {
    topMotor->init();
    bottomMotor->init();
}

void Module::setDesiredState(float velocityRPM, float targetAngleRad) {
    velocityRPM = -velocityRPM;
    float angleTarget = targetAngleRad * RAD_TO_DEG;
    float error = angleTarget - (getCurrentAngle() * RAD_TO_DEG);
    while (error > 180.0f) error -= 360.0f;
    while (error < -180.0f) error += 360.0f;
    float pidOutput = pid(error);
    
    float steerRPM = (pidOutput * DEG_TO_RAD * 60.0 / 2*PI) / MOTOR_TO_SUN_RATIO;
    if (fabs(steerRPM) < 35.0) steerRPM = 0.0;

    float angleSpeed = steerRPM;
    // float angleFactor = fabs(error) > 0.3 ? cosf(error * DEG_TO_RAD) : 1.0;
    float angleFactor = 1.0;
    float speedSpeed = velocityRPM * angleFactor;
    float top1Rad = angleSpeed + speedSpeed;
    float bottom1Rad = angleSpeed - speedSpeed;
    float realMaxSpeed = std::max(std::fabs(top1Rad), std::fabs(bottom1Rad));
    if (realMaxSpeed > 1e-6 && std::fabs(realMaxSpeed) > 200) {
        top1Rad = top1Rad / realMaxSpeed * 200;
        bottom1Rad = bottom1Rad / realMaxSpeed * 200;
    }

    // Debugging output
    // Serial.print("Err: "); Serial.print(error, 2);
    // Serial.print("  Angle: "); Serial.print(getCurrentAngle() * RAD_TO_DEG, 2);
    // Serial.print("  SteerRPM: "); Serial.print(steerRPM, 1);
    // Serial.print("  Top: "); Serial.print(top1Rad, 1);
    // Serial.print("  Bottom: "); Serial.println(bottom1Rad, 1);

    // Apply velocities
    topMotor->setVelocity(top1Rad);
    bottomMotor->setVelocity(bottom1Rad);
}





void Module::update() {
    topMotor->update();
    bottomMotor->update();

    if (isTuning()) {
        updateAutoTune();
    }
}

float Module::getCurrentAngle() {
    float angle = azimuth->getPosition() - offset;
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    // return -angle;
    return -angle;
}

void Module::startAutoTune(bool tuneDrive) {
    Serial.println("Starting PID auto-tune...");
    tuningState = tuneDrive ? TUNING_DRIVE : TUNING_STEER;
    tuningStartTime = millis();
    tuningDuration = 30000; 
    testAmplitude = tuneDrive ? 50.0 : 1.0;
    oscillationCount = 0;
    oscillationPeriod = 0;
    lastCrossing = 0;
    lastAboveZero = false;
    maxOscillation = 0;
    criticalGain = 1.0;
    criticalPeriod = 0;

    Serial.println(tuneDrive ? "Tuning drive PID..." : "Tuning steering PID...");
}

bool Module::updateAutoTune() {
    unsigned long currentTime = millis();
    float elapsed = currentTime - tuningStartTime;

    if (elapsed > tuningDuration) {
        finalizeTuning();
        return true;
    }

    float error = 0;
    float output = 0;

    if (tuningState == TUNING_DRIVE) {
        float current = (topMotor->getVelocity() + bottomMotor->getVelocity()) / 2.0;
        error = testAmplitude - current;
        output = (error > 0) ? testAmplitude : -testAmplitude;
        driveVelocity = output;
        targetAngle = 0;
    } else if (tuningState == TUNING_STEER) {
        float current = getCurrentAngle();
        error = testAmplitude - current;
        static bool toggle = false;
        toggle = !toggle;
        float target = toggle ? testAmplitude : -testAmplitude;
        driveVelocity = 0;
        targetAngle = target;
    }

    maxOscillation = max(maxOscillation, abs(error));
    bool aboveZero = error > 0;
    if (aboveZero != lastAboveZero && elapsed > 1000) {
        if (lastCrossing > 0) {
            float period = elapsed - lastCrossing;
            oscillationPeriod = (oscillationPeriod * oscillationCount + period) / (oscillationCount + 1);
            oscillationCount++;
        }
        lastCrossing = elapsed;
    }
    lastAboveZero = aboveZero;

    if (oscillationCount >= 5 && elapsed > 10000) {
        finalizeTuning();
        return true;
    }

    return false;
}

void Module::finalizeTuning() {
    if (oscillationCount < 2) {
        Serial.println("Auto-tune failed: insufficient oscillations");
        tuningState = NOT_TUNING;
        return;
    }

    criticalPeriod = oscillationPeriod / 1000.0;
    criticalGain = (4.0 * testAmplitude) / (PI * maxOscillation);

    float kp = 0.6 * criticalGain;
    float ki = 2.0 * kp / criticalPeriod;
    float kd = kp * criticalPeriod / 8.0;

    kp *= 0.8;
    ki *= 0.5;
    kd *= 1.2;

    if (tuningState == TUNING_DRIVE) {
        topMotor->setPID(kp, ki, kd);
        bottomMotor->setPID(kp, ki, kd);
        Serial.print("Drive PID tuned - ");
    } else {
        topMotor->setPID(kp * 0.5, ki * 0.3, kd * 0.7);
        bottomMotor->setPID(kp * 0.5, ki * 0.3, kd * 0.7);
        Serial.print("Steer PID tuned - ");
    }

    Serial.print("Kp: "); Serial.print(kp);
    Serial.print(", Ki: "); Serial.print(ki);
    Serial.print(", Kd: "); Serial.println(kd);
    Serial.print("Critical Gain: "); Serial.print(criticalGain);
    Serial.print(", Critical Period: "); Serial.println(criticalPeriod);

    tuningState = TUNING_COMPLETE;
    driveVelocity = 0;
    targetAngle = 0;
}

bool Module::isTuning() {
    return tuningState == TUNING_DRIVE || tuningState == TUNING_STEER;
}

String Module::getTuningStatus() {
    switch (tuningState) {
        case NOT_TUNING: return "Not tuning";
        case TUNING_DRIVE: return "Tuning drive";
        case TUNING_STEER: return "Tuning steering";
        case TUNING_COMPLETE: return "Tuning complete";
        default: return "Unknown";
    }
}

float Module::getDriveVelocity() { return driveVelocity; }
float Module::getTopMotorVelocity() { return topMotor->getVelocity(); }
float Module::getBottomMotorVelocity() { return bottomMotor->getVelocity(); }
float Module::getTurnVelocity() { return 0.0; }
