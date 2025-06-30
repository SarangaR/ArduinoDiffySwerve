#ifndef CONFIG_H
#define CONFIG_H

// ========== HARDWARE CONFIGURATION ==========
// Pin definitions (adjust based on your wiring)
// Top and Bottom is with respect to the gears (gear on top means top, gear on bottom means bottom)
// Center Module
#define CENTER_TOP_PORT 7
#define CENTER_BOTTOM_PORT 6
#define ENCODER1_TOP_A 40
#define ENCODER1_TOP_B 39
#define ENCODER1_BOTTOM_A 11
#define ENCODER1_BOTTOM_B 10

// Right Module
#define RIGHT_TOP_PORT 3
#define RIGHT_BOTTOM_PORT 4
#define ENCODER2_TOP_A 18
#define ENCODER2_TOP_B 17
#define ENCODER2_BOTTOM_A 16
#define ENCODER2_BOTTOM_B 15

// Left Module
#define LEFT_TOP_PORT 5
#define LEFT_BOTTOM_PORT 2
#define ENCODER3_TOP_A 41
#define ENCODER3_TOP_B 42
#define ENCODER3_BOTTOM_A 38
#define ENCODER3_BOTTOM_B 37

// ========== ROBOT STATES ==========
enum RobotState {
    INIT,
    AUTONOMOUS,
    TELEOP,
    ROBOT_DISABLED
};

float applyDeadband(float value, float deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

#endif