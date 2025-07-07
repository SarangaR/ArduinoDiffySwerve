#include "PathFollower.h"

PathFollower::PathFollower(QwiicOTOS* otosPtr, Drivetrain* drivetrainPtr) {
    otos = otosPtr;
    drivetrain = drivetrainPtr;

    path = nullptr;
    pathLength = 0;
    currentWaypointIndex = 0;
    pathActive = false;
    pathComplete = false;

    lookAheadDistance = 0.3;
    waypointTolerance = 0.1;
    headingTolerance = 0.1;
    maxSpeed = 1.0;
    maxAngularSpeed = 2.0;
    maxAcceleration = 0.5;
    totalPathLength = 0.0;

    positionKp = 2.0;
    positionKi = 0.0;
    positionKd = 0.1;
    headingKp = 3.0;
    headingKi = 0.0;
    headingKd = 0.2;

    positionIntegral = positionLastError = 0.0;
    headingIntegral = headingLastError = 0.0;
    lastUpdateTime = 0;

    currentPose = Pose();
    poseValid = false;
}

bool PathFollower::init() {
    Serial.println("Initializing PathFollower...");
    if (!otos->begin()) {
        Serial.println("Failed to initialize OTOS sensor!");
        return false;
    }
    otos->setLinearUnit(kSfeOtosLinearUnitMeters);
    otos->setAngularUnit(kSfeOtosAngularUnitRadians);
    calibrateOTOS();
    Serial.println("PathFollower initialized successfully!");
    return true;
}

void PathFollower::calibrateOTOS() {
    Serial.println("Calibrating OTOS sensor...");
    delay(2000);
    otos->calibrateImu();
    resetPose(0.0, 0.0, 0.0);
    Serial.println("OTOS calibration complete!");
}

void PathFollower::resetPose(float x, float y, float heading) {
    sfe_otos_pose2d_t newPose = {x, y, heading};
    otos->setPosition(newPose);
    currentPose = Pose(x, y, heading);
    Serial.print("Pose reset to: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(heading); Serial.println(")");
}

void PathFollower::setPath(Waypoint* waypoints, int length) {
    path = waypoints;
    pathLength = length;
    currentWaypointIndex = 0;
    pathComplete = false;

    totalPathLength = 0.0;
    for (int i = 1; i < pathLength; ++i) {
        totalPathLength += calculateDistance(path[i - 1].x, path[i - 1].y, path[i].x, path[i].y);
    }

    Serial.print("Path set with ");
    Serial.print(length);
    Serial.print(" waypoints. Total path length: ");
    Serial.println(totalPathLength, 3);
}

void PathFollower::startPath() {
    if (!path || pathLength == 0) {
        Serial.println("No path defined!");
        return;
    }
    pathActive = true;
    pathComplete = false;
    currentWaypointIndex = 0;
    positionIntegral = headingIntegral = 0.0;
    positionLastError = headingLastError = 0.0;
    lastUpdateTime = millis();
    Serial.println("Path following started!");
}

void PathFollower::stopPath() {
    pathActive = false;
    drivetrain->stop();
    Serial.println("Path following stopped!");
}

void PathFollower::pausePath() {
    pathActive = false;
    drivetrain->stop();
    Serial.println("Path following paused!");
}

void PathFollower::resumePath() {
    if (path && !pathComplete) {
        pathActive = true;
        lastUpdateTime = millis();
        Serial.println("Path following resumed!");
    }
}

void PathFollower::setPositionPID(float kp, float ki, float kd) {
    positionKp = kp; positionKi = ki; positionKd = kd;
}

void PathFollower::setHeadingPID(float kp, float ki, float kd) {
    headingKp = kp; headingKi = ki; headingKd = kd;
}

void PathFollower::updatePose() {
    sfe_otos_pose2d_t pose;
    if (otos->getPosition(pose) == ksfTkErrOk) {
        currentPose = Pose(pose.x, pose.y, normalizeAngle(pose.h));
        poseValid = true;
    } else {
        poseValid = false;
        Serial.println("Warning: Failed to read OTOS position!");
    }
}

float PathFollower::calculateDistance(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrt(dx * dx + dy * dy);
}

float PathFollower::normalizeAngle(float angle) {
    while (angle > PI) angle -= 2 * PI;
    while (angle < -PI) angle += 2 * PI;
    return angle;
}

float PathFollower::calculateAngleDifference(float target, float current) {
    return normalizeAngle(target - current);
}

float PathFollower::interpolateHeading(Waypoint from, Waypoint to, float t) {
    float delta = calculateAngleDifference(to.heading, from.heading);
    return normalizeAngle(from.heading + t * delta);
}

void PathFollower::update() {
    if (!pathActive || pathComplete) return;

    updatePose();
    if (!poseValid) {
        Serial.println("Invalid pose - stopping path following");
        stopPath();
        return;
    }

    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0;
    if (dt <= 0) return;

    if (currentWaypointIndex < pathLength) {
        float distanceToWaypoint = calculateDistance(
            currentPose.x, currentPose.y,
            path[currentWaypointIndex].x, path[currentWaypointIndex].y
        );
        if (distanceToWaypoint < waypointTolerance) {
            currentWaypointIndex++;
            positionIntegral = headingIntegral = 0.0;
            Serial.print("Reached waypoint ");
            Serial.println(currentWaypointIndex);
        }
    }

    if (currentWaypointIndex >= pathLength) {
        pathComplete = true;
        pathActive = false;

        float finalHeadingError = calculateAngleDifference(path[pathLength - 1].heading, currentPose.heading);
        if (abs(finalHeadingError) > headingTolerance) {
            float headingCommand = headingKp * finalHeadingError;
            headingCommand = constrain(headingCommand, -maxAngularSpeed, maxAngularSpeed);
            drivetrain->drive(0, 0, headingCommand, true);
        } else {
            drivetrain->stop();
            Serial.println("Path following complete!");
        }
        return;
    }

    Waypoint from = path[currentWaypointIndex];
    Waypoint to = path[min(currentWaypointIndex + 1, pathLength - 1)];

    float segmentLength = calculateDistance(from.x, from.y, to.x, to.y);
    float robotProgress = calculateDistance(from.x, from.y, currentPose.x, currentPose.y);
    float t = (segmentLength > 0.0f) ? constrain(robotProgress / segmentLength, 0.0f, 1.0f) : 0.0f;

    float desiredHeading = interpolateHeading(from, to, t);

    // Compute total distance traveled along path
    float distanceAlongPath = 0.0;
    for (int i = 0; i < currentWaypointIndex; ++i) {
        distanceAlongPath += calculateDistance(path[i].x, path[i].y, path[i + 1].x, path[i + 1].y);
    }
    distanceAlongPath += robotProgress;

    float accelDist = (maxSpeed * maxSpeed) / (2 * maxAcceleration);
    float decelDist = accelDist;
    float cruiseDist = max(0.0f, totalPathLength - (accelDist + decelDist));

    float targetSpeed = 0.0;
    if (distanceAlongPath < accelDist) {
        targetSpeed = sqrt(2 * maxAcceleration * distanceAlongPath);
    } else if (distanceAlongPath < (accelDist + cruiseDist)) {
        targetSpeed = maxSpeed;
    } else {
        float remaining = totalPathLength - distanceAlongPath;
        targetSpeed = sqrt(2 * maxAcceleration * remaining);
    }

    targetSpeed = constrain(targetSpeed, 0.0f, maxSpeed);

    float dx = to.x - currentPose.x;
    float dy = to.y - currentPose.y;
    float positionError = sqrt(dx * dx + dy * dy);
    float headingError = calculateAngleDifference(desiredHeading, currentPose.heading);

    positionIntegral += positionError * dt;
    float positionDerivative = (positionError - positionLastError) / dt;
    float speedCommand = positionKp * positionError + positionKi * positionIntegral + positionKd * positionDerivative;
    speedCommand = constrain(speedCommand, -targetSpeed, targetSpeed);

    headingIntegral += headingError * dt;
    float headingDerivative = (headingError - headingLastError) / dt;
    float angularCommand = headingKp * headingError + headingKi * headingIntegral + headingKd * headingDerivative;
    angularCommand = constrain(angularCommand, -maxAngularSpeed, maxAngularSpeed);

    float vx = speedCommand * cos(desiredHeading);
    float vy = speedCommand * sin(desiredHeading);

    drivetrain->drive(vx, vy, angularCommand, true);

    positionLastError = positionError;
    headingLastError = headingError;
    lastUpdateTime = currentTime;
}

float PathFollower::getDistanceToTarget() {
    if (currentWaypointIndex < pathLength) {
        return calculateDistance(currentPose.x, currentPose.y, path[currentWaypointIndex].x, path[currentWaypointIndex].y);
    }
    return 0.0;
}

float PathFollower::getHeadingError() {
    if (currentWaypointIndex < pathLength) {
        float dx = path[currentWaypointIndex].x - currentPose.x;
        float dy = path[currentWaypointIndex].y - currentPose.y;
        float desiredHeading = atan2(dy, dx);
        return calculateAngleDifference(desiredHeading, currentPose.heading);
    }
    return 0.0;
}

void PathFollower::printStatus() {
    Serial.println("=== PathFollower Status ===");
    Serial.print("Current Pose: (");
    Serial.print(currentPose.x, 3); Serial.print(", ");
    Serial.print(currentPose.y, 3); Serial.print(", ");
    Serial.print(currentPose.heading, 3); Serial.println(")");

    Serial.print("Path Active: "); Serial.println(pathActive ? "Yes" : "No");
    Serial.print("Path Complete: "); Serial.println(pathComplete ? "Yes" : "No");
    Serial.print("Current Waypoint: "); Serial.print(currentWaypointIndex);
    Serial.print(" of "); Serial.println(pathLength);

    if (currentWaypointIndex < pathLength) {
        Serial.print("Target: (");
        Serial.print(path[currentWaypointIndex].x, 3); Serial.print(", ");
        Serial.print(path[currentWaypointIndex].y, 3); Serial.println(")");
        Serial.print("Distance to target: "); Serial.println(getDistanceToTarget(), 3);
        Serial.print("Heading error: "); Serial.println(getHeadingError(), 3);
    }

    Serial.print("Pose Valid: "); Serial.println(poseValid ? "Yes" : "No");
}
