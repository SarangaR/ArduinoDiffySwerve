#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <Arduino.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include "Drivetrain.h"

// Structure to represent a waypoint
struct Waypoint {
    float x;
    float y;
    float heading;  // Desired heading at this point (in radians)

    Waypoint(float x_pos, float y_pos, float desired_heading = 0.0)
        : x(x_pos), y(y_pos), heading(desired_heading) {}
};

// Structure to represent robot pose
struct Pose {
    float x;
    float y;
    float heading;

    Pose(float x_pos = 0.0, float y_pos = 0.0, float heading_angle = 0.0)
        : x(x_pos), y(y_pos), heading(heading_angle) {}
};

class PathFollower {
private:
    QwiicOTOS* otos;
    Drivetrain* drivetrain;

    Waypoint* path;
    int pathLength;
    int currentWaypointIndex;
    bool pathActive;
    bool pathComplete;

    float lookAheadDistance;
    float waypointTolerance;
    float headingTolerance;
    float maxSpeed;
    float maxAngularSpeed;
    float maxAcceleration;
    float totalPathLength;

    float positionKp, positionKi, positionKd;
    float headingKp, headingKi, headingKd;

    float positionIntegral;
    float positionLastError;
    float headingIntegral;
    float headingLastError;
    unsigned long lastUpdateTime;

    Pose currentPose;
    bool poseValid;

    float calculateDistance(float x1, float y1, float x2, float y2);
    float normalizeAngle(float angle);
    float calculateAngleDifference(float target, float current);
    Waypoint findLookAheadPoint();
    void updatePose();
    float interpolateHeading(Waypoint from, Waypoint to, float t);

public:
    PathFollower(QwiicOTOS* otosPtr, Drivetrain* drivetrainPtr);

    bool init();
    void calibrateOTOS();
    void resetPose(float x = 0.0, float y = 0.0, float heading = 0.0);

    void setPath(Waypoint* waypoints, int length);
    void startPath();
    void stopPath();
    void pausePath();
    void resumePath();
    bool isPathActive() { return pathActive; }
    bool isPathComplete() { return pathComplete; }

    void setLookAheadDistance(float distance) { lookAheadDistance = distance; }
    void setWaypointTolerance(float tolerance) { waypointTolerance = tolerance; }
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    void setMaxAngularSpeed(float speed) { maxAngularSpeed = speed; }
    void setMaxAcceleration(float accel) { maxAcceleration = accel; }
    void setPositionPID(float kp, float ki, float kd);
    void setHeadingPID(float kp, float ki, float kd);

    void update();

    Pose getCurrentPose() { return currentPose; }
    int getCurrentWaypoint() { return currentWaypointIndex; }
    float getDistanceToTarget();
    float getHeadingError();
    void printStatus();

    float getCurrentX() { return currentPose.x; }
    float getCurrentY() { return currentPose.y; }
    float getCurrentHeading() { return currentPose.heading; }
    bool isPoseValid() { return poseValid; }
};

#endif
