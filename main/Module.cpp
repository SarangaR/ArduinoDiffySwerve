#include <Module.h>
#include <cmath>

Module::Module(Motor* top, Motor* bottom, moduleID id) :
    top(top),
    bottom(bottom),
    id(id)
{}

void Module::setDesiredState(moduleState state) {
    float angleSpeed = getMotorSpeedsForAngle(state.angle);
    float speedSpeed = getMotorSpeedsForSpeed(state.speed);
    if (std::fabs(angleSpeed) < 0.1) angleSpeed = 0.0;
    if (std::fabs(speedSpeed) < 0.1) speedSpeed = 0.0;
    Angle top1 = Angle(angleSpeed + speedSpeed);
    Angle bottom1 = Angle(-(angleSpeed - speedSpeed));
    float top1Rad = top1.getRadians();
    float bottom1Rad = bottom1.getRadians();
    float realMaxSpeed = std::max(std::fabs(top1Rad), std::fabs(bottom1Rad));
    if (realMaxSpeed > 1e-6 && std::fabs(realMaxSpeed) > std::fabs(top->MAX_SPEED.getRadians())) {
        top1Rad = top1Rad / realMaxSpeed * top->MAX_SPEED.getRadians();
        bottom1Rad = bottom1Rad / realMaxSpeed * top->MAX_SPEED.getRadians();
    }
    top->setVelocity(Angle(top1Rad));
    bottom->setVelocity(Angle(bottom1Rad));
}

void Module::stop() {
    top->setVelocity(Angle(0));
    bottom->setVelocity(Angle(0));
}

void Module::begin() {
    top->begin();
    bottom->begin();
    top->setInverted(false);
    pid.reset();
    bottom->setInverted(true);
}

void Module::loop() {
    top->loop();
    bottom->loop();
}

Angle Module::getModuleOrientation() {
    float topMotorAngle = top->getPosition().getRadians();
    float bottomMotorAngle = bottom->getPosition().getRadians();

    float topModuleAngle = -topMotorAngle * gearRatioTurn;
    float bottomModuleAngle = bottomMotorAngle * gearRatioTurn;

    Angle angle = Angle((topModuleAngle + bottomModuleAngle) / 2, RADIANS);

    return angle.wrapNeg180To180();
}

float Module::getModuleSpeed() {
    float topMotorSpeed = top->getVelocity().getRadians();
    float bottomMotorSpeed = bottom->getVelocity().getRadians();

    float topSpeed = topMotorSpeed * gearRatioSpin;
    float bottomSpeed = bottomMotorSpeed * gearRatioSpin;

    Angle speed = Angle((topSpeed + bottomSpeed) / 2);

    //convert from rad/s to m/s
    float speedMetersPerSecond = speed.getRadians() * wheelRadius;

    return speedMetersPerSecond;
}

float Module::getModuleSpeedRadians() {
    float topMotorSpeed = top->getVelocity().getRadians();
    float bottomMotorSpeed = bottom->getVelocity().getRadians();

    float topSpeed = topMotorSpeed * gearRatioSpin;
    float bottomSpeed = bottomMotorSpeed * gearRatioSpin;

    Angle speed = Angle((topSpeed + bottomSpeed) / 2);

    return speed.getRadians();
}

float Module::getProfileState() {
    return profilePos;
}

float Module::getMotorSpeedsForAngle(float angleDegrees) {
    angleTarget = angleDegrees;
    float error = angleDegrees - getModuleOrientation().getDegrees();
    float pidOutput = pid(error);
    
    return Angle(pidOutput / gearRatioTurn).getRadians();
}

// float Module::getMotorSpeedsForAngle(float angleDegrees) {
//     angleTarget = angleDegrees;
//     float error = angleDegrees - getModuleOrientation().getDegrees();
//     error = Angle(error, DEGREES).wrapNeg180To180().getDegrees(); // Normalize to [-180, 180]

//     float velocity = 0.0;
//     const float DEADBAND = 5.0; // Degrees of acceptable error
//     const float FIXED_SPEED = 0.5; // Radians/sec (adjust based on motor specs)

//     if (std::fabs(error) > DEADBAND) {
//         // Rotate clockwise or counterclockwise based on error
//         velocity = (error > 0) ? FIXED_SPEED : -FIXED_SPEED;
//     }

//     return velocity / gearRatioTurn; // Convert to motor radians/sec
// }

float Module::getMotorSpeedsForSpeed(float speedMetersPerSecond) {
    // float targetSpeedMeters = speedMetersPerSecond;
    // float currentSpeedMeters = getModuleSpeed();
    // float error = targetSpeedMeters - currentSpeedMeters;

    // float accel = speedPID(error);
    // float speedOutputMeters = getModuleSpeed() + accel;
    
    // float speedOutputMeters = speedMetersPerSecond;
    float speedRadPerSec = speedMetersPerSecond / wheelRadius;
    float finalSpeed = speedRadPerSec / gearRatioSpin;

    return finalSpeed;
}

moduleState Module::getState() {
    float angle = getModuleOrientation().getDegrees();
    float speed = getModuleSpeed();

    return moduleState(speed, angle);
}

float Module::rotateAngleBy(float angle, float angleToRotateBy) {
    float newAngle = angle + angleToRotateBy;
    return Angle(newAngle, DEGREES).wrap().getDegrees();
}

float Module::wrapNeg180To180(float angle) {
    if (angle > 180) {
        angle -= 360;
    }
    else if (angle < -180) {
        angle += 360;
    }
    return angle;
}

float Module::getError(float degrees) {
    return degrees - getModuleOrientation().getDegrees();
}

float Module::getErrorModifier(float expected, float actual) {
    float modifier = expected / actual;
    return modifier;
}

void Module::setMotorInvert(bool topInvert, bool bottomInvert) {
    top->setInverted(topInvert);
    bottom->setInverted(bottomInvert);
}

