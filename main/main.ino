#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>
#include <Alfredo_NoU3.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

#include <string>
#include <iostream>

#define DEBUG_MODE_SERIAL false
#define MAG_DEC 14.44504

enum GameState {
  AUTO_INIT,
  AUTO_LOOP,
  TELEOP
};

void updateJoy();
void runDrive();
void updateAll();
void updateGeneral();
void stowLogic();
float applyDeadband(float, float);
void updatePose();
std::vector<float> pathSpeeds();

void top1A();
void top1B();
void bottom1A();
void bottom1B();
void top2A();
void top2B();
void bottom2A();
void bottom2B();
void top3A();
void top3B();
void bottom3A();
void bottom3B();

GameState gameState = TELEOP;

NoU_Motor rawtop1(3);
NoU_Motor rawbottom1(4);
NoU_Motor rawtop2(5);
NoU_Motor rawbottom2(2);
NoU_Motor rawtop3(7);
NoU_Motor rawbottom3(6);

NoU_Motor shooterMotor(1);
NoU_Motor intakeMotor(8);
NoU_Servo armServo(1);

float reduction_ratio = 48;
float ppr = 12;
float cpr = ppr * reduction_ratio;

Encoder top1Encoder = Encoder(16, 15, cpr);
Encoder bottom1Encoder = Encoder(11, 10, cpr);
Encoder top2Encoder = Encoder(41, 42, cpr);
Encoder bottom2Encoder = Encoder(18, 17, cpr);
Encoder top3Encoder = Encoder(38, 37, cpr);
Encoder bottom3Encoder = Encoder(40, 39, cpr);

int encoderPins[] = {16, 15, 11, 10, 41, 42, 18, 17, 38, 37, 40, 39};

Motor top1 = Motor(16, 15, &rawtop1, &top1Encoder);
Motor bottom1 = Motor(11, 10, &rawbottom1, &bottom1Encoder);
Motor top2 = Motor(41, 42, &rawtop2, &top2Encoder);
Motor bottom2 = Motor(18, 17, &rawbottom2, &bottom2Encoder);
Motor top3 = Motor(38, 37, &rawtop3, &top3Encoder);
Motor bottom3 = Motor(40, 39, &rawbottom3, &bottom3Encoder);

Module right(&top1, &bottom1, RIGHT);
Module left(&top2, &bottom2, LEFT);
Module center(&top3, &bottom3, CENTER);

Drivetrain drivetrain(&left, &right, &center, &shooterMotor, &intakeMotor, &armServo);

const float MAX_SPEED = Module::MAX_SPEED_SPIN_MS;

QwiicOTOS otos;
std::vector<Pose2d> path;

sfe_otos_pose2d_t robotPose;
bool startedDistance = false;
bool intakeDone = false;
bool doingPreset = false;
bool isAuto = false;
float autoStart = 0;
float autoLoopStart = 0;
float splineCount = 0.0;

float vxf = 0;
float vyf = 0;
float omega = 0;
float mult = 1;
float MODULE_MAX = 0.8f;

bool stowUP = true; 
bool done = false;

void setup() {
  NoU3.begin();
  Serial.begin(115200);
  #if DEBUG_MODE_SERIAL
    while(!Serial);
  #endif

  PestoLink.begin("BumBotics");

  pinMode(8, INPUT);

  top1Encoder.enableInterrupts(top1A, top1B);
  bottom1Encoder.enableInterrupts(bottom1A, bottom1B);
  top2Encoder.enableInterrupts(top2A, top2B);
  bottom2Encoder.enableInterrupts(bottom2A, bottom2B);
  top3Encoder.enableInterrupts(top3A, top3B);
  bottom3Encoder.enableInterrupts(bottom3A, bottom3B);

  top1Encoder.init();
  bottom1Encoder.init();
  top2Encoder.init();
  bottom2Encoder.init();
  top3Encoder.init();
  bottom3Encoder.init();

  left.setMotorInvert(false, true);
  right.setMotorInvert(true, true);

  while (!otos.begin()) {
    Serial.println("OTOS not connected, check your wiring and I2C address!");
    delay(1000);
  }

  NoU3.updateIMUs();
  otos.calibrateImu();
  otos.setLinearUnit(kSfeOtosLinearUnitMeters);
  otos.setAngularUnit(kSfeOtosAngularUnitDegrees);
  otos.setAngularScalar(3600/(3600-10.92));
  otos.setLinearScalar(1.127);
  otos.resetTracking();
  drivetrain.setArmAngle(150);

  path = {
		{0.25, 0.25, 0.0},
		{0.298, 0.254, 0.0},
		{0.322, 0.261, 0.0},
		{0.332, 0.265, 0.0},
		{0.336, 0.267, 0.0},
		{0.338, 0.269, 0.0},
		{0.339, 0.269, 0.0},
		{0.34, 0.27, 0.0},
		{0.34, 0.27, 0.0},
		{0.34, 0.27, 0.0},
		{0.34, 0.27, 0.0},
		{0.341, 0.27, 0.0},
		{0.342, 0.271, 0.0},
		{0.343, 0.272, 0.0},
		{0.345, 0.274, 0.0},
		{0.35, 0.278, 0.0},
		{0.358, 0.287, 0.0},
		{0.368, 0.308, 0.0},
		{0.375, 0.359, 0.0},
		{0.376, 0.408, 0.0},
		{0.387, 0.455, 0.0},
		{0.429, 0.49, 0.0},
		{0.483, 0.5, 0.0},
		{0.5, 0.5, 0.0}
	};

}

void loop() {
  NoU3.updateServiceLight();
  sfe_otos_pose2d_t odomPose = drivetrain.getOdometryPose();

  if (PestoLink.update()) {
    NoU3.setServiceLight(LIGHT_ENABLED);

    //auto
    if (PestoLink.keyHeld(Key::A)) {
      gameState = AUTO_INIT;
      autoStart = millis();
    }

    if (PestoLink.buttonHeld(15)) {
      stowUP = false;
    }
    else if (PestoLink.buttonHeld(14)) {
      stowUP = true;
    }

    switch (gameState) {
      case AUTO_INIT:
        Serial.println("AUTON BEGIN");
        otos.resetTracking();
        updateAll();

        if (PestoLink.keyHeld(Key::B)) {
          gameState = TELEOP;
        }

        splineCount = 1.0;
        {
          sfe_otos_pose2d_t newStartPose = {path[0].x, path[0].y, path[0].h};
          otos.setPosition(newStartPose);
        }

        autoLoopStart = millis();

        gameState = AUTO_LOOP;

        break;
      
      case AUTO_LOOP:
        if ((millis() - autoLoopStart) < 15000 && splineCount < path.size()) {

          if (PestoLink.keyHeld(Key::B)) {
            gameState = TELEOP;
            break;
          } 

          std::vector<float> followSpeeds = pathSpeeds();

          float outX = followSpeeds[0];
          float outY = followSpeeds[1];
          float outH = followSpeeds[2];
          float limit = followSpeeds[3];

          if (limit == 1.0) {
            splineCount++;
          }
          else {
            drivetrain.drive(outY*MODULE_MAX*5, outX*MODULE_MAX*5, 0.0, Angle(robotPose.h, DEGREES), true);
          }
        }
        else {
          drivetrain.stopIntake();
          drivetrain.stop();
          drivetrain.stopShooter();
          // drivetrain.stowArm();
          gameState = TELEOP;
        }

        break;

      case TELEOP:
        // +FWD = +Y
        // +STR = +X
        // if (PestoLink.keyHeld(Key::N)) {
        //   drivetrain.drive(0, 0, 1, Angle(robotPose.h, DEGREES), true); 
        // }
        // else {
        //   drivetrain.stop();
        // }
        
        runDrive();
        // center.setDesiredState({1, 90});
        // left.setDesiredState({1, 90});
        // right.setDesiredState({1, 90});
        // String center_angle = "Center Angle: " + String(center.getModuleOrientation().getDegrees());
        // String center_angle = "Motor Vel: " + String(top3.getVelocity().getRadians());
        // String odomPose = "X: " + String(robotPose.x) + "Y: " + String(robotPose.y) + "Z: " + String(robotPose.h);
        // Serial.println(center.getModuleOrientation().getDegrees());

        // Serial.println(String(center.getModuleOrientation().getDegrees()).c_str());

        break;

    }
  }
  else {
    NoU3.setServiceLight(LIGHT_ON);
  }

  updateGeneral();
}

// =====================================================================================
// =============================== METHOD DEFINITIONS ==================================
// =====================================================================================

std::vector<float> pathSpeeds() {
  float currX = robotPose.x;
  float currY = robotPose.y;
  float heading = robotPose.h;

  Pose2d target = path[splineCount];
  float targetX = target.x;
  float targetY = target.y;
  float targetHeading = 0.0; // target.h;

  Serial.print(targetX);
  Serial.print(" ");
  Serial.print(targetY);
  Serial.print(" ");
  Serial.println(targetHeading);

  float kpx = 10.0;
  float kpy = 10.0;
  float kph = 0.5;

  float errorX = targetX - currX;
  float errorY = targetY - currY;
  float errorH = Angle(targetHeading - heading, DEGREES).getRadians();

  float outX = kpx * errorX;
  float outY = kpy * errorY;
  float outH = kph * errorH;

  bool inLimit = fabs(errorX) < 0.05 && fabs(errorY) < 0.05;
  float limit = inLimit;

  return {outX, outY, outH, limit};
}

void updateJoy() {
  vxf = applyDeadband(PestoLink.getAxis(1), 0.1)*mult*MODULE_MAX;
  vyf = -applyDeadband(PestoLink.getAxis(0), 0.1)*mult*MODULE_MAX;
  omega = -applyDeadband(PestoLink.getAxis(2), 0.1)*4*M_PI*mult;
}

unsigned long lastUpdate = 0;
float lastVxf = 0, lastVyf = 0, lastOmega = 0;

void runDrive() {
  unsigned long now = millis();
  if (now - lastUpdate < 20) return;
  lastUpdate = now;

  // // Low-pass filter (alpha = 0.5 for moderate smoothing)
  // const float alpha = 0.5;
  // vxf = alpha * vxf + (1 - alpha) * lastVxf;
  // vyf = alpha * vyf + (1 - alpha) * lastVyf;
  // omega = alpha * omega + (1 - alpha) * lastOmega;
  // lastVxf = vxf; lastVyf = vyf; lastOmega = omega;

  if (std::fabs(vxf) > 0.01 || std::fabs(vyf) > 0.01 || std::fabs(omega) > 0.01) {
      drivetrain.drive(vxf, vyf, omega, Angle(robotPose.h, DEGREES), false);
  } else {
      drivetrain.stop();
  }
}

void updateAll() {
  drivetrain.loop();
  updatePose();
  PestoLink.update();
  drivetrain.updateWheelOdometry(robotPose.h);
  updateJoy();
  runDrive();
  PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
}

void updateGeneral() {
  drivetrain.loop();
  updatePose();
  drivetrain.updateWheelOdometry(robotPose.h);
  updateJoy();
  PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());
}

void updatePose() {
  sfe_otos_pose2d_t pose;
  otos.getPosition(pose);
  robotPose.x = pose.x;
  robotPose.y = pose.y;
  robotPose.h = pose.h;
}

void stowLogic() {
  updateAll();
  if (stowUP) {
    drivetrain.setArmAngle(150);
  }
  else {
    drivetrain.stowArm();
  }
}

void top1A() {
  top1Encoder.handleA();
}
void top1B() {
  top1Encoder.handleB();
}
void bottom1A() {
  bottom1Encoder.handleA();
}
void bottom1B() {
  bottom1Encoder.handleB();
}
void top2A() {
  top2Encoder.handleA();
}
void top2B() {
  top2Encoder.handleB();
} 
void bottom2A() {
  bottom2Encoder.handleA();
}
void bottom2B() {
  bottom2Encoder.handleB();
}
void top3A() {
  top3Encoder.handleA();
}
void top3B() {
  top3Encoder.handleB();
}
void bottom3A() {
  bottom3Encoder.handleA();
}
void bottom3B() {
  bottom3Encoder.handleB();
}

float applyDeadband(float value, float deadband) {
  if (abs(value) < deadband) {
    return 0;
  }
  return value;
}

float wrapYaw(float currentYaw) {
  static bool isInitialized = false;
  static int rotations = 0;
  static float previousYaw = 0;


  if(isInitialized == false){
    previousYaw = currentYaw;
    isInitialized = true;
  }

    // Check for wrapping
    if (currentYaw - previousYaw > 180.0) {
        rotations--; // Wrapped from -180 to 180
    } else if (currentYaw - previousYaw < -180.0) {
        rotations++; // Wrapped from 180 to -180
    }

    // Wrap the yaw angle between -180 and 180
    float wrappedYaw = currentYaw + rotations * 360.0;

    previousYaw = currentYaw;

    return wrappedYaw;
}
