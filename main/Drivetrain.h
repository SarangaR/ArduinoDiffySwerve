#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H

#include <Arduino.h>
#include <Module.h>
#include <vector>
#include <PathPlanner.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

class Drivetrain {
    public:
        Drivetrain(Module* left, Module* right, Module* center, NoU_Motor* shooter, NoU_Motor* intake, NoU_Servo* arm);
        std::array<moduleState, 3> drive(float vx, float vy, float omega, Angle gyroAngle, bool fieldOriented);
        std::array<moduleState, 3> toSwerveModuleStates(float vx, float vy, float omega, Angle angle, bool fieldOriented);
        void stop();
        void begin();
        float getGyroAngle();
        void loop();

        std::vector<Angle> getModuleOrientations();
        std::vector<float> getModuleSpeeds();
        std::array<moduleState, 3> getModuleStates();
        std::array<moduleState, 3> optimize(std::array<moduleState, 3> desiredStates, std::array<moduleState, 3> currentStates);
        std::array<moduleState, 3> normalizeSpeeds(std::array<moduleState, 3> speeds);

        std::vector<String> lastModuleStates = {"", ""};

        //distance from wheel to wheel (trackwidth)
        float sideLength = 13.5f/100.0f; //m

        //distance from vertex to center of triangle (robot is triangle)
        float halfAngle = M_PI/6.0f;
        float centroidDistance = (0.5*sideLength)/(cos(halfAngle));
        float vertexYDistance = centroidDistance*sin(halfAngle);

        std::vector<std::vector<float>> modulePositions = {
          {-0.5*sideLength, -vertexYDistance, 0},
          {0.5*sideLength, -vertexYDistance, 0},
          {0, 0.5*sideLength, 0}
        };

        std::vector<float> moveToPoint(Point point, float targetRotation, sfe_otos_pose2d_t &currentPose, float maxSpeed);
        void setDistanceStart(sfe_otos_pose2d_t &currentPose);

        void spinShooter();
        void spinShooter(float speed);
        void reverseSpinShooter();
        void stopEnd();
        void setArmAngle(float angle);
        bool runIntake();
        void runIntake(float speed);
        void reverseIntake();
        void stowArm();
        void stopIntake();
        void stopShooter();

        void updateWheelOdometry(float robotHeading);
        sfe_otos_pose2d_t getOdometryPose();
        void resetOdometryPose();

    private:
        Module* left;
        Module* right;
        Module* center;
        NoU_Motor* shooter;
        NoU_Motor* intake;
        NoU_Servo* arm;

        bool initialBreak = false;

        float armRestPos = 179;

        sfe_otos_pose2d_t distanceStartPose;
        sfe_otos_pose2d_t odometryPose;
        float lastOdomUpdate = 0;
        float correctionTargetHeading = 0;

        static const int numModules = 3;

        std::array<float, 3U> fromFieldRelativeSpeeds(float vx, float vy, float omega, Angle gyroAngle); 

        PIDController xPID = PIDController(5.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        PIDController yPID = PIDController(5.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        PIDController hPID = PIDController(5.0f, 0.0f, 0.0f, 0.0f, 0.0f);
        PIDController headingStraightPID = PIDController(1.0f, 0.0f, 0.0f, 0.0f, 0.0f);

};

#endif // DRIVETRAIN_H