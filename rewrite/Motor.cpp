#include "Motor.h"

const float MAX_MOTOR_SPEED = 200.0; // RPM

// Motor electrical constants
const float R_motor = 4.8;         // Ohms
const float Kv = 93.1;             // rad/s per V
const float V_supply = 7.4;        // Nominal voltage
const float I_limit = 0.3;         // Current limit in Amps

Motor::Motor(int port, int encA, int encB, bool invertMotor) {
    alfredoMotor = new NoU_Motor(port);
    encoder = new Encoder(encA, encB, cpr); // Assuming 2048 PPR encoders
    inverted = invertMotor;
    targetVelocity = 0.0;
    currentVelocity = 0.0;

    // Default PID values
    kp = 0.1;
    ki = 0.0;
    kd = 0.0;
    integral = 0.0;
    lastError = 0.0;
    lastTime = 0;
    lastEncoderUpdate = 0;
}

Motor::~Motor() {
    delete alfredoMotor;
    delete encoder;
}

void Motor::init() {
    alfredoMotor->setDeadband(0.3);
    alfredoMotor->setMinimumOutput(0.0);
    alfredoMotor->setMaximumOutput(1.0);
    alfredoMotor->setBrakeMode(false);
    encoder->quadrature = Quadrature::OFF;
}

void Motor::doA() {
    encoder->handleA();
}

void Motor::doB() {
    encoder->handleB();
}

void Motor::setVelocity(float velocity) {
    targetVelocity = velocity;
}

float Motor::getVelocity() {    
    return encoder->getVelocity();
}

float Motor::getTargetVelocity() { 
    return targetVelocity; 
}

float Motor::getPosition() {
    return encoder->getAngle();
}

float Motor::applySlewRateLimit(float desiredRPM, float maxDeltaRPMPerSec, float dt) {
    float delta = desiredRPM - lastRPM;
    float maxDelta = maxDeltaRPMPerSec * dt;

    if (fabs(delta) > maxDelta) {
        delta = (delta > 0) ? maxDelta : -maxDelta;
    }

    lastRPM += delta;
    return lastRPM;
}


void Motor::update() {
    if (millis() - lastEncoderUpdate > 10) {
        encoder->update();
        lastEncoderUpdate = millis();
    }

    unsigned long currentTime = millis();
    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }

    float dt = (currentTime - lastTime) / 1000.0;
    if (dt <= 0) return;

    float currentVel = getVelocity();
    float error = targetVelocity - currentVel;

    // // PID control
    // integral += error * dt;
    // float derivative = (error - lastError) / dt;
    // float pidOutput = kp * error + ki * integral + kd * derivative;

    // float desiredVelocity = targetVelocity + pidOutput; // in RPM
    // float desiredRadPerSec = (desiredVelocity * 2.0 * PI) / 60.0;
    // float backEMF = desiredRadPerSec / Kv;

    // float maxV_applied = I_limit * R_motor + backEMF;
    // float maxDutyCycle = maxV_applied / V_supply;
    // maxDutyCycle = constrain(maxDutyCycle, 0.0, 1.0);

    float power = constrain(targetVelocity / MAX_MOTOR_SPEED, -1.0, 1.0);
    if (inverted) power = -power;

    // Apply current limit
    // if (abs(power) > maxDutyCycle) {
    //     power = copysign(maxDutyCycle, power);
    // }

    power = applySlewRateLimit(power, 50.0, dt);

    alfredoMotor->set(power);

    lastError = error;
    lastTime = currentTime;
}

void Motor::updateOpenLoop() {
    if (millis() - lastEncoderUpdate > 10) {
        encoder->update();
        lastEncoderUpdate = millis();
    }
}

void Motor::setPID(float p, float i, float d) {
    kp = p;
    ki = i;
    kd = d;
    integral = 0.0;
    lastError = 0.0;
}
