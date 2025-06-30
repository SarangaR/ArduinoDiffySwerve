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
    {0.5, 0.0},   // Forward
    {1.0, 0.5},   // Up and right
    {1.5, 0.0},   // Center top
    {2.0, -0.5},  // Down and right
    {1.5, -1.0},  // Bottom right
    {1.0, -0.5},  // Up toward center
    {0.5, -1.0},  // Down and left
    {0.0, -0.5},  // Up and left
    {0.5, 0.0},   // Back to center
    {0.0, 0.0}    // Return to start
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
    center = new Module(centerTop, centerBottom, centerAzimuth, (34.44) * DEG_TO_RAD);
    right = new Module(rightTop, rightBottom, rightAzimuth, (-126.69) * DEG_TO_RAD);
    left = new Module(leftTop, leftBottom, leftAzimuth, (-125.62) * DEG_TO_RAD);
    
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
}

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
                if (pathFollowingActive) {
                    // Path following is handled in pathFollower->update() above
                    // Just monitor for completion or manual stop
                } else {
                    runBasicAutonomous();
                }
                break;
                
            case TELEOP:
                runTeleop();
                break;
                
            case ROBOT_DISABLED:
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
    }

    // right->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // center->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // left->setDesiredState(200.0, 90.0 * DEG_TO_RAD);
    // leftTop->setVelocity(80.0);
    // leftBottom->setVelocity(80.0);
    // Serial.print(center->getCurrentAngle() * RAD_TO_DEG); Serial.print(" ");
    // Serial.print(right->getCurrentAngle() * RAD_TO_DEG); Serial.print(" ");
    // Serial.println(left->getCurrentAngle() * RAD_TO_DEG);
    // center->update();
    // right->update();
    // left->update();

    // Serial.println("NOT CRASHED");
}

// ========== BASIC AUTONOMOUS MODE (Non-Path Following) ==========
void runBasicAutonomous() {
    unsigned long currentTime = millis();
    unsigned long stepTime = currentTime - autoStepStartTime;
    
    // Simple autonomous sequence without path following
    switch (autoStep) {
        case 0: // Drive forward
            drivetrain->drive(0.5, 0, 0, false); // 0.5 m/s forward, robot-oriented
            if (stepTime > 2000) { // 2 seconds
                autoStep++;
                autoStepStartTime = currentTime;
            }
            break;
            
        case 1: // Turn right
            drivetrain->drive(0, 0, 1.0, false); // 1 rad/s clockwise
            if (stepTime > 1570) { // ~90 degree turn (π/2 / 1.0 * 1000)
                autoStep++;
                autoStepStartTime = currentTime;
            }
            break;
            
        case 2: // Drive forward again
            drivetrain->drive(0.5, 0, 0, false);
            if (stepTime > 2000) {
                autoStep++;
                autoStepStartTime = currentTime;
            }
            break;
            
        case 3: // Stop
            drivetrain->stop();
            if (stepTime > 1000) {
                currentState = ROBOT_DISABLED;
                Serial.println("Basic autonomous complete!");
            }
            break;
    }
}

float vxf = 0.0;
float vyf = 0.0;
float omega = 0.0;
float MAX_LIN_VEL = 0.46547931149358335;
float robotRadius = 0.0495;
float MAX_ANG_VEL = MAX_LIN_VEL / robotRadius;

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