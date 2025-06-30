#ifndef MODULE_H
#define MODULE_H

#include <Arduino.h>
#include "Motor.h"
#include "Azimuth.h"

enum TuningState {
    NOT_TUNING,
    TUNING_DRIVE,
    TUNING_STEER,
    TUNING_COMPLETE
};

class Module {
public:
    Module(Motor* top, Motor* bottom, Azimuth* az, float offset);

    void init();
    void update();

    void setDesiredState(float velocityRPM, float angleRad);

    float getDriveVelocity();
    float getTopMotorVelocity();
    float getBottomMotorVelocity();
    float getTurnVelocity(); // deprecated, but kept for compatibility
    float getCurrentAngle();

    void startAutoTune(bool tuneDrive);
    bool isTuning();
    String getTuningStatus();

private:
    Motor* topMotor;
    Motor* bottomMotor;
    Azimuth* azimuth;

    float driveVelocity;
    float targetAngle;

    float gearRatioSpin = 0.5 * 3.0;
    float wheelRadius = 2.2225f/100.0f;

    // Steering PID
    float steerKp = 2.5;
    float steerKi = 0.01;
    float steerKd = 0.5;
    const float kFF = 0.0f; 
    float steerIntegral = 0.0;
    float steerPrevError = 0.0;
    unsigned long lastUpdateTime = 0;
    float angleDeadband = 0.1; //rads

    // Tuning
    TuningState tuningState;
    unsigned long tuningStartTime;
    unsigned long tuningDuration;
    float testAmplitude;
    int oscillationCount;
    float oscillationPeriod;
    unsigned long lastCrossing;
    bool lastAboveZero;
    float maxOscillation;
    float criticalGain;
    float criticalPeriod;

    float offset = 0.0;

    PIDController pid = PIDController(3.0f, 0.0f, 0.1f, 0.0f, 200.0f);
    // PIDController pid = PIDController(2.0f, 0.0f, 0.f, 0.0f, 200.0f);

    void updateSteeringPID();
    void updateMotors();
    bool updateAutoTune();
    void finalizeTuning();
};

#endif
