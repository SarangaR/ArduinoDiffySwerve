#include "PathFollower.h"

PathFollower::PathFollower(QwiicOTOS* otosPtr, Drivetrain* drivetrainPtr) {
    otos = otosPtr;
    drivetrain = drivetrainPtr;
    
    // Initialize path following parameters
    path = nullptr;
    pathLength = 0;
    currentWaypointIndex = 0;
    pathActive = false;
    pathComplete = false;
    
    // Default control parameters
    lookAheadDistance = 0.3;  // 30cm lookahead
    waypointTolerance = 0.1;  // 10cm tolerance
    headingTolerance = 0.1;   // ~5.7 degrees
    maxSpeed = 1.0;           // 1 m/s max
    maxAngularSpeed = 2.0;    // 2 rad/s max
    
    // Default PID parameters
    positionKp = 2.0;
    positionKi = 0.0;
    positionKd = 0.1;
    headingKp = 3.0;
    headingKi = 0.0;
    headingKd = 0.2;
    
    // Initialize PID state
    positionIntegral = 0.0;
    positionLastError = 0.0;
    headingIntegral = 0.0;
    headingLastError = 0.0;
    lastUpdateTime = 0;
    
    // Initialize pose
    currentPose = Pose(0.0, 0.0, 0.0);
    poseValid = false;
}

bool PathFollower::init() {
    Serial.println("Initializing PathFollower...");
    
    // Initialize OTOS sensor
    if (!otos->begin()) {
        Serial.println("Failed to initialize OTOS sensor!");
        return false;
    }
    
    // Configure OTOS
    otos->setLinearUnit(kSfeOtosLinearUnitMeters);
    otos->setAngularUnit(kSfeOtosAngularUnitRadians);
    
    // Calibrate the sensor
    calibrateOTOS();
    
    Serial.println("PathFollower initialized successfully!");
    return true;
}

void PathFollower::calibrateOTOS() {
    Serial.println("Calibrating OTOS sensor...");
    Serial.println("Make sure the robot is stationary and centered on the field!");
    
    delay(2000); // Give time for robot to settle
    
    // Perform calibration
    otos->calibrateImu();
    
    // Reset position to (0, 0, 0)
    resetPose(0.0, 0.0, 0.0);
    
    Serial.println("OTOS calibration complete!");
}

void PathFollower::resetPose(float x, float y, float heading) {
    sfe_otos_pose2d_t newPose;
    newPose.x = x;
    newPose.y = y;
    newPose.h = heading;
    
    otos->setPosition(newPose);
    
    currentPose.x = x;
    currentPose.y = y;
    currentPose.heading = heading;
    
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
    
    Serial.print("Path set with ");
    Serial.print(length);
    Serial.println(" waypoints");
}

void PathFollower::startPath() {
    if (path == nullptr || pathLength == 0) {
        Serial.println("No path defined!");
        return;
    }
    
    pathActive = true;
    pathComplete = false;
    currentWaypointIndex = 0;
    
    // Reset PID state
    positionIntegral = 0.0;
    positionLastError = 0.0;
    headingIntegral = 0.0;
    headingLastError = 0.0;
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
    if (path != nullptr && !pathComplete) {
        pathActive = true;
        lastUpdateTime = millis();
        Serial.println("Path following resumed!");
    }
}

void PathFollower::setPositionPID(float kp, float ki, float kd) {
    positionKp = kp;
    positionKi = ki;
    positionKd = kd;
}

void PathFollower::setHeadingPID(float kp, float ki, float kd) {
    headingKp = kp;
    headingKi = ki;
    headingKd = kd;
}

void PathFollower::updatePose() {
    sfe_otos_pose2d_t pose;
    sfTkError_t status = otos->getPosition(pose);
    
    if (status == ksfTkErrOk) {
        currentPose.x = pose.x;
        currentPose.y = pose.y;
        currentPose.heading = normalizeAngle(pose.h);
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
    float diff = target - current;
    return normalizeAngle(diff);
}

Waypoint PathFollower::findLookAheadPoint() {
    if (currentWaypointIndex >= pathLength) {
        return path[pathLength - 1]; // Return last waypoint
    }
    
    // Start with current target waypoint
    Waypoint targetPoint = path[currentWaypointIndex];
    
    // Look ahead to find optimal point
    for (int i = currentWaypointIndex; i < pathLength; i++) {
        float distanceToWaypoint = calculateDistance(
            currentPose.x, currentPose.y, 
            path[i].x, path[i].y
        );
        
        if (distanceToWaypoint >= lookAheadDistance) {
            targetPoint = path[i];
            break;
        }
    }
    
    return targetPoint;
}

void PathFollower::update() {
    if (!pathActive || pathComplete) {
        return;
    }
    
    // Update robot pose
    updatePose();
    
    if (!poseValid) {
        Serial.println("Invalid pose - stopping path following");
        stopPath();
        return;
    }
    
    unsigned long currentTime = millis();
    float dt = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
    
    if (dt <= 0) return; // Avoid division by zero
    
    // Check if we've reached the current waypoint
    if (currentWaypointIndex < pathLength) {
        Waypoint currentTarget = path[currentWaypointIndex];
        float distanceToWaypoint = calculateDistance(
            currentPose.x, currentPose.y,
            currentTarget.x, currentTarget.y
        );
        
        if (distanceToWaypoint < waypointTolerance) {
            currentWaypointIndex++;
            Serial.print("Reached waypoint ");
            Serial.print(currentWaypointIndex);
            Serial.print(" of ");
            Serial.println(pathLength);
            
            // Reset integral terms when reaching waypoint
            positionIntegral = 0.0;
            headingIntegral = 0.0;
        }
    }
    
    // Check if path is complete
    if (currentWaypointIndex >= pathLength) {
        pathComplete = true;
        pathActive = false;
        drivetrain->stop();
        Serial.println("Path following complete!");
        return;
    }
    
    // Find lookahead point
    Waypoint lookAheadPoint = findLookAheadPoint();
    
    // Calculate position error
    float dx = lookAheadPoint.x - currentPose.x;
    float dy = lookAheadPoint.y - currentPose.y;
    float positionError = sqrt(dx * dx + dy * dy);
    
    // Calculate desired heading to lookahead point
    float desiredHeading = atan2(dy, dx);
    float headingError = calculateAngleDifference(desiredHeading, currentPose.heading);
    
    // PID control for position
    positionIntegral += positionError * dt;
    float positionDerivative = (positionError - positionLastError) / dt;
    float speedCommand = positionKp * positionError + 
                        positionKi * positionIntegral + 
                        positionKd * positionDerivative;
    
    // PID control for heading
    headingIntegral += headingError * dt;
    float headingDerivative = (headingError - headingLastError) / dt;
    float angularCommand = headingKp * headingError + 
                          headingKi * headingIntegral + 
                          headingKd * headingDerivative;
    
    // Limit speeds
    speedCommand = constrain(speedCommand, -maxSpeed, maxSpeed);
    angularCommand = constrain(angularCommand, -maxAngularSpeed, maxAngularSpeed);
    
    // Convert to robot frame velocities
    float vx = speedCommand * cos(desiredHeading);
    float vy = speedCommand * sin(desiredHeading);
    
    // Send commands to drivetrain (field-oriented)
    drivetrain->drive(vx, vy, angularCommand, true);
    
    // Update PID state
    positionLastError = positionError;
    headingLastError = headingError;
    lastUpdateTime = currentTime;
}

float PathFollower::getDistanceToTarget() {
    if (currentWaypointIndex < pathLength) {
        return calculateDistance(
            currentPose.x, currentPose.y,
            path[currentWaypointIndex].x, path[currentWaypointIndex].y
        );
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