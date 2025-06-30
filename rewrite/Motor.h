#ifndef MOTOR_H
#define MOTOR_H

#include <Alfredo_NoU3.h>
#include <SimpleFOC.h>

class Motor {
private:
    NoU_Motor* alfredoMotor;
    float targetVelocity;
    float currentVelocity;
    bool inverted;
    
    // PID parameters
    float kp, ki, kd;
    float integral, lastError;
    unsigned long lastTime;
    unsigned long lastEncoderUpdate;

    float reduction_ratio = 48;
    float ppr = 12;
    float cpr = ppr * reduction_ratio;
    
    float lastRPM = 0.0;

public:
    Motor(int port, int encA, int encB, bool invertMotor = false);
    ~Motor();
    
    Encoder* encoder;

    void init();
    void setVelocity(float velocity);
    float getVelocity();
    float getTargetVelocity();
    float getPosition();
    void update();
    void updateOpenLoop();
    void setPID(float p, float i, float d);
    void doA();
    void doB();
    float applySlewRateLimit(float desiredRPM, float maxDeltaRPMPerSec, float dt);
};

#endif