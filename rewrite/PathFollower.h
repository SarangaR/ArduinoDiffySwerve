#ifndef PATHFOLLOWER_H
#define PATHFOLLOWER_H

#include <Arduino.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include "Drivetrain.h"

// Structure to represent a waypoint
struct Waypoint {
    float x;
    float y;
    float heading;  // Optional desired heading at this point (in radians)
    float speed;    // Desired speed at this waypoint (m/s)
    
    Waypoint(float x_pos, float y_pos, float desired_heading = 0.0, float desired_speed = 0.5) 
        : x(x_pos), y(y_pos), heading(desired_heading), speed(desired_speed) {}
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
    
    // Path following parameters
    Waypoint* path;
    int pathLength;
    int currentWaypointIndex;
    bool pathActive;
    bool pathComplete;
    
    // Control parameters
    float lookAheadDistance;
    float waypointTolerance;
    float headingTolerance;
    float maxSpeed;
    float maxAngularSpeed;
    
    // PID controllers for position and heading
    float positionKp, positionKi, positionKd;
    float headingKp, headingKi, headingKd;
    
    // PID state variables
    float positionIntegral;
    float positionLastError;
    float headingIntegral;
    float headingLastError;
    unsigned long lastUpdateTime;
    
    // Current robot state
    Pose currentPose;
    bool poseValid;
    
    // Helper functions
    float calculateDistance(float x1, float y1, float x2, float y2);
    float normalizeAngle(float angle);
    float calculateAngleDifference(float target, float current);
    Waypoint findLookAheadPoint();
    void updatePose();
    
public:
    PathFollower(QwiicOTOS* otosPtr, Drivetrain* drivetrainPtr);
    
    // Initialization
    bool init();
    void calibrateOTOS();
    void resetPose(float x = 0.0, float y = 0.0, float heading = 0.0);
    
    // Path management
    void setPath(Waypoint* waypoints, int length);
    void startPath();
    void stopPath();
    void pausePath();
    void resumePath();
    bool isPathActive() { return pathActive; }
    bool isPathComplete() { return pathComplete; }
    
    // Control parameters
    void setLookAheadDistance(float distance) { lookAheadDistance = distance; }
    void setWaypointTolerance(float tolerance) { waypointTolerance = tolerance; }
    void setMaxSpeed(float speed) { maxSpeed = speed; }
    void setMaxAngularSpeed(float speed) { maxAngularSpeed = speed; }
    void setPositionPID(float kp, float ki, float kd);
    void setHeadingPID(float kp, float ki, float kd);
    
    // Main update function
    void update();
    
    // Status and diagnostics
    Pose getCurrentPose() { return currentPose; }
    int getCurrentWaypoint() { return currentWaypointIndex; }
    float getDistanceToTarget();
    float getHeadingError();
    void printStatus();
    
    // Direct pose access for drivetrain
    float getCurrentX() { return currentPose.x; }
    float getCurrentY() { return currentPose.y; }
    float getCurrentHeading() { return currentPose.heading; }
    bool isPoseValid() { return poseValid; }
};

#endif