#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>
#include <SimpleFOC.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>
#include <SparkFun_TMAG5273_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include "Config.h"
#include "Module.h"
#include "Drivetrain.h"
#include "PathFollower.h"
#include "Azimuth.h"

// ========== GLOBAL OBJECTS ==========
// Motors for each module
// Top and Bottom is with respect to the gears (gear on top means top, gear on bottom means bottom)
Motor* centerTop;
Motor* centerBottom;
Motor* rightTop;
Motor* rightBottom;
Motor* leftTop;
Motor* leftBottom;

// NOTE: Set these numbers to OPPOSITE of the displayed value (- is +, + is -)
float centerOffset = 0.0;
float leftOffset = 0.0;
float rightOffset = 0.0;

// NoU_Servo arm;
NoU_Motor intake(8);

// enum ScoringState {
//     STOW,
//     L3,
//     L2,
//     L1
// };

// Modules
Module* center;
Module* right;
Module* left;

QWIICMUX mux;
TMAG5273 centerSensor;
TMAG5273 rightSensor;
TMAG5273 leftSensor;

Azimuth* centerAzimuth;
Azimuth* rightAzimuth;
Azimuth* leftAzimuth;

uint8_t i2cAddress1 = 0x22;
uint8_t i2cAddress2 = 0x23;
uint8_t i2cAddress3 = 0x24;

// Drivetrain and sensors
Drivetrain* drivetrain;
QwiicOTOS otos;
PathFollower* pathFollower;

// State machine
RobotState currentState = TELEOP;
unsigned long stateStartTime = 0;
unsigned long lastTelemetryTime = 0;

// Autonomous sequence variables
int autoStep = 0;
unsigned long autoStepStartTime = 0;

// Path following variables
Waypoint* currentPath = nullptr;
int currentPathLength = 0;
bool pathFollowingActive = false;

// ========== PREDEFINED PATHS ==========
// Simple square path
Waypoint squarePath[] = {
    {1.0, 0.0},   // Move forward 1m
    {1.0, 1.0},   // Move right 1m
    {0.0, 1.0},   // Move back 1m
    {0.0, 0.0}    // Return to start
};

// Figure-8 path
Waypoint figure8Path[] = {
	{0.0, 0.0, 60.0},
	{-0.048, 0.018, 60.0},
	{-0.096, 0.036, 60.0},
	{-0.144, 0.053, 60.0},
	{-0.192, 0.07, 60.0},
	{-0.241, 0.087, 60.0},
	{-0.289, 0.103, 60.0},
	{-0.338, 0.119, 60.0},
	{-0.387, 0.134, 60.0},
	{-0.436, 0.149, 60.0},
	{-0.484, 0.163, 60.0},
	{-0.533, 0.176, 60.0},
	{-0.582, 0.189, 60.0},
	{-0.638, 0.203, 60.0},
	{-0.687, 0.215, 60.0},
	{-0.729, 0.225, 60.0},
	{-0.776, 0.236, 60.0},
	{-0.821, 0.248, 60.0},
	{-0.837, 0.253, 60.0},
	{-0.85, 0.259, 60.0},
	{-0.859, 0.265, 60.0},
	{-0.868, 0.278, 60.0},
	{-0.867, 0.295, 60.0},
	{-0.858, 0.314, 60.0}
};

// Simple forward and back path
Waypoint simplePath[] = {
    {0.5, 0.0},   // Move forward 1m
    {0.0, 0.0}    // Return to start
};


// ========== ENCODER INTERRUPT HANDLERS ==========
void encoderCenterTopA()  { centerTop->doA(); }
void encoderCenterTopB()  { centerTop->doB(); }
void encoderCenterBottomA()  { centerBottom->doA(); }
void encoderCenterBottomB()  { centerBottom->doB(); }

void encoderRightTopA()  { rightTop->doA(); }
void encoderRightTopB()  { rightTop->doB(); }
void encoderRightBottomA()  { rightBottom->doA(); }
void encoderRightBottomB()  { rightBottom->doB(); }

void encoderLeftTopA()  { leftTop->doA(); }
void encoderLeftTopB()  { leftTop->doB(); }
void encoderLeftBottomA()  { leftBottom->doA(); }
void encoderLeftBottomB()  { leftBottom->doB(); }

// ========== SETUP FUNCTION ==========
void setup() {
    Serial.begin(115200);
    NoU3.begin();
    PestoLink.begin("Bumbotics");
    
    Serial.println("Initializing BumBot...");
    Wire.begin();

    Serial.println("Init MUX");
    while (mux.begin(0x70, Wire) == false) {
        Serial.println("Mux not detected...");
    }
    Serial.println("Mux detected");
    mux.enablePort(3);

    Serial.println("Sensor 1 Setup: ");
    mux.setPort(0);
    while (rightSensor.begin(i2cAddress1, Wire) == false) {
        Serial.println("first sensor not found");
    }
    Serial.println("Sensor 1 enabled");
    Serial.print("Sensor 1 Original: "); Serial.println(rightSensor.getI2CAddress(), HEX);
    while (rightSensor.setI2CAddress(i2cAddress1) != 0) {
        Serial.println("Address not set correctly!");
    }
    Serial.print("Sensor 1 New: "); Serial.println(rightSensor.getI2CAddress(), HEX);
    delay(500);
    
    Serial.println("Sensor 2 Setup: ");
    mux.setPort(1);
    while (leftSensor.begin(i2cAddress1, Wire) == false) {
        Serial.println("second sensor not found");
    }
    Serial.println("Sensor 2 enabled");
    Serial.print("Sensor 2 Original: "); Serial.println(leftSensor.getI2CAddress(), HEX);
    while (leftSensor.setI2CAddress(i2cAddress2) != 0) {
        Serial.println("Address not set correctly!");
    }
    Serial.print("Sensor 2 New: "); Serial.println(leftSensor.getI2CAddress(), HEX);
    delay(500);

    Serial.println("Sensor 3 Setup: ");
    mux.setPort(2);
    while (centerSensor.begin(i2cAddress1, Wire) == false) {
        Serial.println("third sensor not found");
    }
    Serial.println("Sensor 3 enabled");
    Serial.print("Sensor 3 Original: "); Serial.println(centerSensor.getI2CAddress(), HEX);
    while (centerSensor.setI2CAddress(i2cAddress3) != 0) {
        Serial.println("Address not set correctly!");
    }
    Serial.print("Sensor 3 New: "); Serial.println(centerSensor.getI2CAddress(), HEX);
    delay(500);

    mux.enablePort(0);
    mux.enablePort(1);
    mux.enablePort(2);
    mux.enablePort(3);

    centerAzimuth = new Azimuth(&centerSensor, 0x22, 0);
    rightAzimuth = new Azimuth(&rightSensor, 0x23, 1);
    leftAzimuth = new Azimuth(&leftSensor, 0x24, 2);

    centerAzimuth->init();
    rightAzimuth->init();
    leftAzimuth->init();
    
    // Create motor objects
    centerTop = new Motor(CENTER_TOP_PORT, ENCODER1_TOP_A, ENCODER1_TOP_B);
    centerBottom = new Motor(CENTER_BOTTOM_PORT, ENCODER1_BOTTOM_A, ENCODER1_BOTTOM_B, true);
    
    rightTop = new Motor(RIGHT_TOP_PORT, ENCODER2_TOP_A, ENCODER2_TOP_B, true);
    rightBottom = new Motor(RIGHT_BOTTOM_PORT, ENCODER2_BOTTOM_A, ENCODER2_BOTTOM_B, false);
    
    leftTop = new Motor(LEFT_TOP_PORT, ENCODER3_TOP_A, ENCODER3_TOP_B, false);
    leftBottom = new Motor(LEFT_BOTTOM_PORT, ENCODER3_BOTTOM_A, ENCODER3_BOTTOM_B, false);
    
    // Create module objects
    //offsets are negative of actual value
    center = new Module(centerTop, centerBottom, centerAzimuth, (centerOffset) * DEG_TO_RAD);
    right = new Module(rightTop, rightBottom, rightAzimuth, (rightOffset) * DEG_TO_RAD);
    left = new Module(leftTop, leftBottom, leftAzimuth, (leftOffset) * DEG_TO_RAD);
    
    // Create drivetrain
    drivetrain = new Drivetrain(center, right, left);
    
    // Create path follower
    pathFollower = new PathFollower(&otos, drivetrain);

    drivetrain->setOTOS(&otos);
    
    // Link path follower to drivetrain
    drivetrain->setPathFollower(pathFollower);
    
    // Initialize everything
    drivetrain->init();

    centerTop->encoder->enableInterrupts(encoderCenterTopA, encoderCenterTopB);
    centerTop->encoder->init();
    centerBottom->encoder->enableInterrupts(encoderCenterBottomA, encoderCenterBottomB);
    centerBottom->encoder->init();

    rightTop->encoder->enableInterrupts(encoderRightTopA, encoderRightTopB);
    rightTop->encoder->init();
    rightBottom->encoder->enableInterrupts(encoderRightBottomA, encoderRightBottomB);
    rightBottom->encoder->init();

    leftTop->encoder->enableInterrupts(encoderLeftTopA, encoderLeftTopB);
    leftTop->encoder->init();
    leftBottom->encoder->enableInterrupts(encoderLeftBottomA, encoderLeftBottomB);
    leftBottom->encoder->init();
    
    
    while (!pathFollower->init()) {
        Serial.println("Failed to initialize PathFollower!");
    }
    
    Serial.println("Initialization complete!");
    Serial.println("Available commands:");
    Serial.println("  A - Start basic autonomous sequence");
    Serial.println("  1 - Follow simple forward/back path");
    Serial.println("  2 - Follow square path");
    Serial.println("  3 - Follow figure-8 path");
    Serial.println("  enter - Enter teleop mode");
    Serial.println("  4 - Auto-tune drive PIDs");
    Serial.println("  5 - Auto-tune steering PIDs");
    Serial.println("  6 - Reset robot pose to (0,0,0)");
    Serial.println("  7 - Print system status");
    Serial.println("  space - Disable robot");
    
    currentState = TELEOP;
    stateStartTime = millis();

    PestoLink.setTerminalPeriod(10);
}

int auto_count = 0;
float MAX_LIN_VEL = 0.46547931149358335;
float robotRadius = 0.0495;
float MAX_ANG_VEL = MAX_LIN_VEL / robotRadius;

// ========== MAIN LOOP ==========
void loop() {
    if (PestoLink.update()) {
        unsigned long currentTime = millis();
        
        // Update drivetrain and path follower
        drivetrain->update();
        if (pathFollowingActive) {
            pathFollower->update();
            
            // Check if path is complete
            if (pathFollower->isPathComplete()) {
                pathFollowingActive = false;
                Serial.println("Path following complete! Entering disabled state.");
                currentState = ROBOT_DISABLED;
            }
        }
        
        // State machine
        switch (currentState) {
            case INIT:
                // Already handled in setup
                break;
                
            case AUTONOMOUS:
                runBasicAutonomous();

                break;
                
            case TELEOP:
                auto_count = 0;
                runTeleop();
                break;
                
            case ROBOT_DISABLED:
                auto_count = 0;
                drivetrain->stop();
                if (pathFollowingActive) {
                    pathFollower->stopPath();
                    pathFollowingActive = false;
                }
                break;
        }
        
        // Check for state transitions
        checkStateTransitions();
        
        // Periodic telemetry
        // if (currentTime - lastTelemetryTime > 100) { // Every 2 seconds
        //     if (currentState != ROBOT_DISABLED || Serial.available()) {
        //         printSystemStatus();
        //     }
        //     lastTelemetryTime = currentTime;
        // }
        PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
        // ChassisSpeeds speeds = drivetrain->getCurrentSpeeds();
        // sfe_otos_pose2d_t pos;
        // drivetrain->getOTOS()->getPosition(pos);
        // String vels = String(pos.x) + String(" ") + String(pos.y) + String(" ") + String(pos.h);
        // PestoLink.printTerminal(vels.c_str());
        // OdomPose pos = drivetrain->getOdomPose();
        // PestoLink.printTerminal(position.c_str());
    }
 
    // right->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // center->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // left->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // leftTop->setVelocity(80.0);
    // leftBottom->setVelocity(80.0);
    Serial.print(center->getCurrentAngle() * RAD_TO_DEG); Serial.print(" ");
    Serial.print(right->getCurrentAngle() * RAD_TO_DEG); Serial.print(" ");
    Serial.println(left->getCurrentAngle() * RAD_TO_DEG);
    // center->update();
    // right->update();
    // left->update();

    // Serial.println("NOT CRASHED");
}

// ========== BASIC AUTONOMOUS MODE (Non-Path Following) ==========
void runBasicAutonomous() {
    Waypoint targetPt = simplePath[auto_count];
    sfe_otos_pose2d_t currPose;
    drivetrain->getOTOS()->getPosition(currPose);

    float x_err = targetPt.x - currPose.x;
    float y_err = targetPt.y - currPose.y;
    float h_err = targetPt.heading * DEG_TO_RAD - currPose.h;

    float x_kp = 50.0;
    float y_kp = 50.0;
    float h_kp = 1.0;

    float x = x_kp * x_err;
    float y = y_kp * y_err;
    float w = h_kp * h_err;

    String print = String("output: ") + String(x_err) + " " + String(y_err) + " " + String(h_err);
    PestoLink.printTerminal(print.c_str());

    drivetrain->drive(y * MAX_LIN_VEL, x * MAX_LIN_VEL, 0.0, true, false);

    if (fabs(x_err) < 0.05 && fabs(y_err) < 0.05) {
        auto_count++;
    }
}

float vxf = 0.0;
float vyf = 0.0;
float omega = 0.0;

// ========== TELEOP MODE ==========
void runTeleop() {
    vxf = applyDeadband(PestoLink.getAxis(1), 0.1) * MAX_LIN_VEL;
    vyf = -applyDeadband(PestoLink.getAxis(0), 0.1) * MAX_LIN_VEL;
    omega = -applyDeadband(PestoLink.getAxis(2), 0.1) * MAX_ANG_VEL;

    if (PestoLink.keyHeld(Key::Space)) {
        currentState = ROBOT_DISABLED;
        if (pathFollowingActive) {
            pathFollower->stopPath();
            pathFollowingActive = false;
        }
        Serial.println("Robot Disabled");
    }
    else {
        drivetrain->drive(vxf, vyf, omega, true);
    }

    if (PestoLink.buttonHeld(6)) {
        intake.set(-1);
    }
    else if (PestoLink.buttonHeld(7)) {
        intake.set(1);
    }
    else {
        intake.set(0);
    }
}

// ========== PATH FOLLOWING FUNCTIONS ==========
void startPathFollowing(Waypoint* path, int pathLength, const char* pathName) {
    Serial.print("Starting path following: ");
    Serial.println(pathName);
    
    // Stop any current path
    if (pathFollowingActive) {
        pathFollower->stopPath();
    }
    
    // Set new path
    pathFollower->setPath(path, pathLength);
    pathFollower->startPath();
    pathFollowingActive = true;
    currentState = AUTONOMOUS;
    
    Serial.print("Path set with ");
    Serial.print(pathLength);
    Serial.println(" waypoints");
}

// ========== STATE TRANSITIONS ==========
void checkStateTransitions() {
    if (PestoLink.keyHeld(Key::A)) {
        currentState = AUTONOMOUS;
        autoStep = 0;
        autoStepStartTime = millis();
        pathFollowingActive = false; // Use basic autonomous
        Serial.println("Entering Basic Autonomous Mode");
        
    } else if (PestoLink.keyHeld(Key::Digit1)) {
        startPathFollowing(simplePath, sizeof(simplePath)/sizeof(Waypoint), "Simple Path");
        
    } else if (PestoLink.keyHeld(Key::Digit2)) {
        startPathFollowing(squarePath, sizeof(squarePath)/sizeof(Waypoint), "Square Path");
        
    } else if (PestoLink.keyHeld(Key::Digit3)) {
        startPathFollowing(figure8Path, sizeof(figure8Path)/sizeof(Waypoint), "Figure-8 Path");
        
    } else if (PestoLink.keyHeld(Key::Enter)) {
        currentState = TELEOP;
        if (pathFollowingActive) {
            pathFollower->stopPath();
            pathFollowingActive = false;
        }
        Serial.println("Entering Teleop Mode");
        Serial.println("Use Joystick for movement, SPACE to stop");
        Serial.println("Use 'f' for field-oriented, 'r' for robot-oriented");
        
    } else if (PestoLink.keyHeld(Key::Digit4)) {
        Serial.println("Starting drive PID auto-tune...");
        drivetrain->startAutoTune(true);
        
    } else if (PestoLink.keyHeld(Key::Digit5)) {
        Serial.println("Starting steering PID auto-tune...");
        drivetrain->startAutoTune(false);
        
    } else if (PestoLink.keyHeld(Key::Digit6)) {
        pathFollower->resetPose(0.0, 0.0, 0.0); 
        Serial.println("Robot pose reset to origin");
        
    } else if (PestoLink.keyHeld(Key::Digit7)) {
        printSystemStatus();
        
    } else if (PestoLink.keyHeld(Key::J) && pathFollowingActive) {
        pathFollower->pausePath();
        Serial.println("Path following paused");
        
    } else if (PestoLink.keyHeld(Key::K) && pathFollowingActive) {
        pathFollower->resumePath();
        Serial.println("Path following resumed");
        
    } else if (PestoLink.keyHeld(Key::L) && pathFollowingActive) {
        pathFollower->stopPath();
        pathFollowingActive = false;
        currentState = ROBOT_DISABLED;
        Serial.println("Path following stopped");
        
    } else if (PestoLink.keyHeld(Key::Space)) {
        currentState = ROBOT_DISABLED;
        if (pathFollowingActive) {
            pathFollower->stopPath();
            pathFollowingActive = false;
        }
        Serial.println("Robot Disabled");
    }
    else if (PestoLink.buttonHeld(8)) { //TODO: replace with gyro reset button
        otos.resetTracking();
    }
}

// ========== STATUS AND DEBUGGING ==========
void printSystemStatus() {
    Serial.println("\n========== SYSTEM STATUS ==========");
    
    // Print current state
    Serial.print("Robot State: ");
    switch (currentState) {
        case INIT: Serial.println("INIT"); break;
        case AUTONOMOUS: Serial.println("AUTONOMOUS"); break;
        case TELEOP: Serial.println("TELEOP"); break;
        case ROBOT_DISABLED: Serial.println("DISABLED"); break;
    }
    
    // Print path following status
    Serial.print("Path Following: ");
    if (pathFollowingActive) {
        Serial.println("ACTIVE");
        pathFollower->printStatus();
    } else {
        Serial.println("INACTIVE");
    }
    
    Serial.print("Control: ");
    Serial.print(vxf); Serial.print(" ");
    Serial.print(vyf); Serial.print(" ");
    Serial.println(omega);

    // Print drivetrain status
    drivetrain->printStatus();
    
    Serial.println("==================================\n");
}