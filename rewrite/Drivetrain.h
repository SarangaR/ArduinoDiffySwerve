#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include "Module.h"
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

#define NUM_MODULES 3

struct ChassisSpeeds {
    float fwd;
    float str;
    float rot;
};

struct OdomPose {
    float x;
    float y;
    float h;
}

class PathFollower; // Forward declaration

class Drivetrain {
private:
    float placeAngleInScope(float scopeReference, float newAngle);

    Module* modules[NUM_MODULES];
    QwiicOTOS* otos;
    PathFollower* pathFollower;
    OdomPose currPose;
    unsigned long lastOdomTime = 0;
    float storedYaw = 0;

    //distance from wheel to wheel (trackwidth)
    float sideLength = 17.15008f/100.0f; //m

    //distance from vertex to center of triangle (robot is triangle)
    float halfAngle = M_PI/6.0f;
    float centroidDistance = (0.5*sideLength)/(cos(halfAngle));
    float vertexYDistance = centroidDistance*sin(halfAngle);

    float wheelRadius = 2.2225f/100.0f;

    float modulePositions[3][2] = {
        {0.0, 0.5 * sideLength},
        {0.5 * sideLength, -vertexYDistance},
        {-0.5 * sideLength, -vertexYDistance},   
    };
    
    void calculateModuleVelocities(float vx, float vy, float omega,   
                                   float* driveVels, float* steerAngularVels);
    
public:
    Drivetrain(Module* mod1, Module* mod2, Module* mod3);
    
    void init();
    void setOTOS(QwiicOTOS* otosPtr) { otos = otosPtr; }
    void setPathFollower(PathFollower* pf) { pathFollower = pf; }
    
    void drive(float vx, float vy, float omega, bool fieldOriented = true, bool headingCorrection = true);
    ChassisSpeeds forwardKinematics(float driveRPMs[3], float anglesRad[3], float modulePositions[3][2], float wheelRadius);
    float calcYawStraight(float target, float current, float kp);

    void stop();
    void update();
    void updateOdom();
    
    void startAutoTune(bool tuneDrive);
    bool isTuning();
    
    void printStatus();
    
    // Getters for sensor access
    QwiicOTOS* getOTOS() { return otos; }
    float getCurrentHeading();
    OdomPose getOdomPose();
};

#endif