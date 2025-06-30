#include <Motor.h>

Motor::Motor(int enc1, int enc2, NoU_Motor* rawMotor, Encoder* sensor) :
    enc1(enc1), enc2(enc2), rawMotor(rawMotor), sensor(sensor)
{}

void Motor::begin() {
    rawMotor->setMinimumOutput(0.2);
    rawMotor->setMaximumOutput(1.0);
    rawMotor->setBrakeMode(false);
    sensor->quadrature = Quadrature::OFF;
    lastUpdate = millis();
}

void Motor::loop() {
    if (millis() - lastUpdate > 10) {  // Update encoder every 50ms (for example)
        sensor->update();
        lastUpdate = millis();
    }
}

void Motor::setVelocity(Angle velocity) {
    int dir = 1;
    if (inverted) {
        dir = -1;
    }
    float setpoint = dir * velocity.getRadians();
    
    // float error = setpoint - getVelocity().getRadians();
    // float accel = pid(error);
    // float output = getVelocity().getRadians() + accel + Kff*setpoint;
    float out = setpoint / MAX_SPEED.getRadians();
    rawMotor->set(out);
}

void Motor::setInverted(bool isInverted) {
    inverted = isInverted;
}

void Motor::stop() {
    setVelocity(Angle(0));
}

Angle Motor::getPosition() {
    // return sensor->getPreciseAngle();
    return Angle(sensor->getPreciseAngle());
}

Angle Motor::getVelocity() {
    // return sensor->getVelocity();
    return Angle(sensor->getVelocity());
}