#include <algorithm>
#include <Drivetrain.h>

Drivetrain::Drivetrain(Module* left, Module* right, Module* center, NoU_Motor* shooter, NoU_Motor* intake, NoU_Servo* arm) :
    left(left),
    right(right),
    center(center),
    shooter(shooter),
    intake(intake),
    arm(arm)// Initialize PID controller with appropriate gains
{
    lastOdomUpdate = millis();
}

void Drivetrain::begin() {
    left->begin();
    right->begin();
    center->begin();

    // for (int i = 0; i < numModules; i++) {
    //     inverseKinematics.row(i*2 + 0) = Eigen::Vector3d(1, 0, -modulePositions[i].y());
    //     inverseKinematics.row(i*2 + 1) = Eigen::Vector3d(0, 1, modulePositions[i].x());
    // }
    // forwardKinematics = inverseKinematics.inverse();
}

float Drivetrain::getGyroAngle() {
    //TODO: Implement this
    Angle gyroAngle = Angle(0, DEGREES);

    return gyroAngle.wrapNeg180To180().getRadians();
}

void Drivetrain::loop() {
    left->loop();
    right->loop();
    center->loop();
}

std::array<moduleState, 3> Drivetrain::toSwerveModuleStates(float vxf, float vyf, float omega, Angle angle, bool fieldOriented) {
    Angle gyroAngle = fieldOriented ? angle : Angle(0);

    std::array<float, 3U> chassisSpeeds = fromFieldRelativeSpeeds(vxf, vyf, omega, gyroAngle);
    float vx = chassisSpeeds[0];
    float vy = chassisSpeeds[1];

    std::array<moduleState, 3> moduleStates = {
        moduleState(0, 0),
        moduleState(0, 0),
        moduleState(0, 0),
    };

    // Loop over each module to calculate its speed and angle
    for (size_t i = 0; i < numModules; i++) {
        // Module position offsets
        float moduleX = modulePositions[i][0];
        float moduleY = modulePositions[i][1];

        // Compute the velocity components for each module
        float moduleVx = vx - omega * moduleX;
        float moduleVy = vy + omega * moduleY;

        // Compute speed and angle for each module
        float speed = hypot(moduleVx, moduleVy); // Calculate the module's speed
        Angle moduleAngle = (speed > 1e-6) ? Angle(moduleVx, moduleVy) : getModuleOrientations()[i];

        // Set module state with speed and angle
        moduleStates[i] = moduleState(speed, moduleAngle.wrapNeg180To180().getDegrees());
        moduleStates[i].optimize(getModuleOrientations()[i]);

    }

    return moduleStates;
}

std::array<float, 3U> Drivetrain::fromFieldRelativeSpeeds(float vx, float vy, float omega, Angle gyroAngle) {
    float robotVx = vx * cosf(gyroAngle.getRadians()) - vy * sinf(gyroAngle.getRadians());
    float robotVy = vx * sinf(gyroAngle.getRadians()) + vy * cosf(gyroAngle.getRadians());

    return {robotVx, robotVy, omega};
}
 
std::array<moduleState, 3> Drivetrain::optimize(std::array<moduleState, 3> desiredStates, std::array<moduleState, 3> currentStates) {
    std::array<moduleState, 3> newStates = {moduleState(0, 0), moduleState(0, 0), moduleState(0, 0)};
    for (int i = 0; i < desiredStates.size(); i++) {
        moduleState desiredState = desiredStates[i];
        moduleState currentState = currentStates[i];
        moduleState newState = moduleState(0, 0); 
        Angle delta = Angle(currentState.angle, DEGREES) - Angle(desiredState.angle, DEGREES);
        if (fabs(delta.wrapNeg180To180().getDegrees()) > 90.0f) {
            newState.speed = -desiredState.speed;
            newState.angle = Angle(desiredState.angle + 180, DEGREES).wrapNeg180To180().getDegrees();
        }
        else {
            newState.speed = desiredState.speed;
            newState.angle = desiredState.angle;
        }
        newStates[i] = newState;
    }
    return newStates;
} 

std::array<moduleState, 3> Drivetrain::drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented) {
    // Correct for yaw drift
    // if (omega == 0) {
    //     float headingError = correctionTargetHeading - getGyroAngle();
    //     omega = -headingStraightPID(headingError);
    // } else {
    //     correctionTargetHeading = getGyroAngle(); // Update target heading when manually rotating
    // }

    vx = constrain(vx, -0.9, 0.9);
    vy = constrain(vy, -0.9, 0.9);
    omega = constrain(omega, -0.75, 0.75);

    std::array<moduleState, 3> states = toSwerveModuleStates(vx, vy, omega, gyroAngle, fieldOriented);

    lastModuleStates = {states[0].toString(), states[1].toString(), states[2].toString()};

    left->setDesiredState(states[0]);
    right->setDesiredState(states[1]); 
    center->setDesiredState(states[2]);
    
    return states;
}

std::array<moduleState, 3> Drivetrain::normalizeSpeeds(std::array<moduleState, 3> states) {
    std::vector<float> speeds;
    for (int i = 0; i < states.size(); i++) {
        speeds.push_back(states[i].speed);
    }

    float max = *std::max_element(speeds.begin(), speeds.end());
    if (max > Module::MAX_SPEED_SPIN_MS) {
        for (int i = 0; i < speeds.size(); i++) {
            speeds[i] /= 0.9 * Module::MAX_SPEED_SPIN_MS / max;
        }
    }

    std::array<moduleState, 3> normalizedStates = {moduleState(0, 0), moduleState(0, 0), moduleState(0, 0)};

    for (int i = 0; i < states.size(); i++) {
        normalizedStates[i] = moduleState(speeds[i], states[i].angle);
    } 

    return normalizedStates;
}

std::vector<Angle> Drivetrain::getModuleOrientations() {
    return {left->getModuleOrientation(), right->getModuleOrientation(), center->getModuleOrientation()};
}

std::vector<float> Drivetrain::getModuleSpeeds() {
    return {left->getModuleSpeed(), right->getModuleSpeed(), center->getModuleSpeed()};
}

std::array<moduleState, 3> Drivetrain::getModuleStates() {
    return {left->getState(), right->getState(), center->getState()};
}

void Drivetrain::stop() {
    right->stop();
    left->stop();
    center->stop();
}

std::vector<float> Drivetrain::moveToPoint(Point point, float targetRotation, sfe_otos_pose2d_t &currentPose, float maxSpeed) {
    float xError = point.x - currentPose.x;
    float yError = point.y - currentPose.y;
    float hError = -targetRotation - currentPose.h;

    float kpx = 3.0f;
    float kpy = 3.0f;                                                                                                                                                                                                   
    float kph = 3.0f;                

    float xOutput = kpx*xError;
    float yOutput = kpy*yError;

    float vx = xOutput;
    float vy = yOutput;

    if (vx > maxSpeed) vx = maxSpeed;
    if (vy > maxSpeed) vy = maxSpeed;

    drive(-vy, vx, 0, Angle(currentPose.h, DEGREES), true);

    if (fabs(xError) < 0.01 && fabs(yError) < 0.01 && fabs(hError) < 0.01) {
        stop();
    }
    return {xError, yError, 0};
}

void Drivetrain::setDistanceStart(sfe_otos_pose2d_t &currentPose) {
    distanceStartPose = currentPose;
}

void Drivetrain::spinShooter() {
    shooter->set(-1);
}

void Drivetrain::reverseSpinShooter() {
    shooter->set(1);
}

bool Drivetrain::runIntake() {
    if (digitalRead(8) == LOW) {
        intake->set(0);
        return true;
    }
    else {
        intake->set(-1);
        return false;
    }
    return false;
}

void Drivetrain::runIntake(float speed) {
    intake->set(-speed);
}

void Drivetrain::reverseIntake() {
    intake->set(1);
}

void Drivetrain::setArmAngle(float angle) {
    arm->write(180 - angle);
}

void Drivetrain::stowArm() {
    setArmAngle(30);
}

void Drivetrain::stopEnd() {
    intake->set(0);
    shooter->set(0);
}

void Drivetrain::spinShooter(float speed) {
    shooter->set(-speed);
}

void Drivetrain::stopIntake() {
    intake->set(0);
}

void Drivetrain::stopShooter() {
    shooter->set(0);
}

sfe_otos_pose2d_t Drivetrain::getOdometryPose() {
    return odometryPose;
}

void Drivetrain::updateWheelOdometry(float robotHeading) {
    float dt = (millis() - lastOdomUpdate) / 1000.0f;
    lastOdomUpdate = millis();

    // Get the current module states
    auto moduleStates = getModuleStates();

    // Calculate the robot's velocity in the robot frame
    float vx = 0;
    float vy = 0;
    float omega = 0;

    for (size_t i = 0; i < numModules; i++) {
        float speed = moduleStates[i].speed;
        Angle angle = Angle(moduleStates[i].angle, DEGREES);

        // Module position offsets
        float moduleX = modulePositions[i][0];
        float moduleY = modulePositions[i][1];

        // Compute the velocity components for each module
        float moduleVx = speed * cosf(angle.getRadians());
        float moduleVy = speed * sinf(angle.getRadians());

        // Accumulate the robot's velocity
        vx += moduleVx;
        vy += moduleVy;
        omega += (moduleVy * moduleX - moduleVx * moduleY) / (moduleX * moduleX + moduleY * moduleY);
    }

    // Average the velocities
    vx /= numModules;
    vy /= numModules;
    omega /= numModules;

    // Convert the robot's velocity to the global frame
    float heading = radians(robotHeading);
    float globalVx = vx * cosf(heading) - vy * sinf(heading);
    float globalVy = vx * sinf(heading) + vy * cosf(heading);

    // Integrate the velocities to update the position
    odometryPose.x += globalVx * dt;
    odometryPose.y += globalVy * dt;
    odometryPose.h = robotHeading;
}

void Drivetrain::resetOdometryPose() {
    odometryPose.x = 0;
    odometryPose.y = 0;
    odometryPose.h = 0;
}