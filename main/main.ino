#include <Arduino.h>
#include <Motor.h>
#include <PestoLink-Receive.h>
#include <Drivetrain.h>
#include <Alfredo_NoU3.h>
#include <ESP32Encoder.h>
#include <SparkFun_Qwiic_OTOS_Arduino_Library.h>

#define DEBUG_MODE_SERIAL false
#define MAG_DEC 14.44504

NoU_Motor rawtop1(3);
NoU_Motor rawbottom1(4);
NoU_Motor rawtop2(5);
NoU_Motor rawbottom2(2);
NoU_Motor rawtop3(7);
NoU_Motor rawbottom3(6);

NoU_Motor shooterMotor(1);
NoU_Motor intakeMotor(8);
NoU_Servo armServo(1);

float reduction_ratio = 50;
float ppr = 7;
float cpr = ppr * reduction_ratio;

Encoder top1Encoder = Encoder(16, 15, cpr);
Encoder bottom1Encoder = Encoder(11, 10, cpr);
Encoder top2Encoder = Encoder(41, 42, cpr);
Encoder bottom2Encoder = Encoder(18, 17, cpr);
Encoder top3Encoder = Encoder(38, 37, cpr);
Encoder bottom3Encoder = Encoder(40, 39, cpr);

int encoderPins[] = {16, 15, 11, 10, 41, 42, 18, 17, 38, 37, 40, 39};

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

QwiicOTOS otos;
std::vector<Point> path;

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

  path = {
    {1, 0},
    {0, 0}
  };

  drivetrain.setArmAngle(150);
}

sfe_otos_pose2d_t robotPose;
bool startedDistance = false;
bool intakeDone = false;
bool doingPreset = false;
bool isAuto = false;
float autoStart = 0;

void updatePose() {
  sfe_otos_pose2d_t pose;
  otos.getPosition(pose);
  robotPose.x = pose.x*(0.5/0.34);
  robotPose.y = pose.y;
  robotPose.h = pose.h;
}

float vxf = 0;
float vyf = 0;
float omega = 0;
float mult = 1;

void updateJoy() {
  vxf = applyDeadband(PestoLink.getAxis(1), 0.1)*mult;
  vyf = -applyDeadband(PestoLink.getAxis(0), 0.1)*mult;
  omega = -applyDeadband(PestoLink.getAxis(2), 0.1)*4*M_PI*mult;
}

void runDrive() {
  if (vxf != 0 || vyf != 0 || omega != 0) {
    drivetrain.drive(vxf, vyf, omega, Angle(robotPose.h, DEGREES), true);
  }
  else {
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
}

bool stowUP = true; 

void stowLogic() {
  updateAll();
  if (stowUP) {
    drivetrain.setArmAngle(150);
  }
  else {
    drivetrain.stowArm();
  }
}

bool done = false;

void loop() {
  NoU3.updateServiceLight();
  sfe_otos_pose2d_t odomPose = drivetrain.getOdometryPose();

  if (PestoLink.update()) {
    NoU3.setServiceLight(LIGHT_ENABLED);
    vxf = applyDeadband(PestoLink.getAxis(1), 0.1)*mult;
    vyf = -applyDeadband(PestoLink.getAxis(0), 0.1)*mult;
    omega = -applyDeadband(PestoLink.getAxis(2), 0.1)*4*M_PI*mult;

    //auto
    if (PestoLink.keyHeld(Key::A)) {
      isAuto = true;
      autoStart = millis();
    }

    if (PestoLink.buttonHeld(15)) {
      stowUP = false;
    }
    else if (PestoLink.buttonHeld(14)) {
      stowUP = true;
    }
    
    while (isAuto) {
      otos.resetTracking();
      drivetrain.spinShooter(); 
      updateAll();

        
      if (PestoLink.keyHeld(Key::B)) {
        isAuto = false;
      }

      delay(1000);

      drivetrain.setArmAngle(20);
      delay(1000);
      drivetrain.runIntake(1);
      delay(1000);

      float start = millis();

      while (millis() - start < 1500) {
        drivetrain.drive(-1, 0, 0, Angle(0), false);
        updateAll();
      }

      drivetrain.stop();

      drivetrain.stopShooter();
      isAuto = false;
    }

    if (PestoLink.buttonHeld(8)) {
      otos.resetTracking();
      drivetrain.resetOdometryPose();
    }
    
    //home
    while (PestoLink.buttonHeld(0)) {
      stowLogic();
      intakeDone = false;
      updateAll();
    }

    //reverse intake
    if (PestoLink.buttonHeld(1)) {
      drivetrain.reverseIntake();
      drivetrain.reverseSpinShooter();
      intakeDone = false;
    }
    else {
      drivetrain.stopShooter();
    }

    //amp
    while (PestoLink.buttonHeld(5)) {
      drivetrain.setArmAngle(90);
      updateAll();

      if (PestoLink.buttonHeld(1)) {
        drivetrain.reverseIntake();
        drivetrain.reverseSpinShooter();
        intakeDone = false;
      }
      else {
        drivetrain.stopShooter();
      }
    }

    //speaker
    while (PestoLink.buttonHeld(4)) {
      drivetrain.setArmAngle(20);
      drivetrain.spinShooter(); 
      updateAll();

      if (intakeDone || digitalRead(8) == LOW) {
        float start = millis();
        while ((millis() - start < 500) && PestoLink.buttonHeld(7)) {
          drivetrain.runIntake(1);
          updateAll();
        }
        drivetrain.stopIntake();
        intakeDone = false;
      }
    }

    if (digitalRead(8) == HIGH) {
      intakeDone = false;
    }

    //intake and shoot
    if (PestoLink.buttonHeld(6) || PestoLink.buttonHeld(7)) {
      if (PestoLink.buttonHeld(6) && !intakeDone) {
        
        drivetrain.setArmAngle(6);
        while (!drivetrain.runIntake() && PestoLink.buttonHeld(6)) {
          updateAll();
        }
        float start = 0;
        if (PestoLink.buttonHeld(6))
          start = millis();
        while ((millis() - start < 300) && PestoLink.buttonHeld(6)) {
          intakeMotor.set(0.2);
          updateAll();
          intakeDone = true;
        }
        intakeMotor.set(0);
      }
      if (PestoLink.buttonHeld(7)) {
        // if (intakeDone || digitalRead(8) == LOW) {
        //   float start = millis();
        //   while ((millis() - start < 500) && PestoLink.buttonHeld(7)) {
        //     drivetrain.runIntake(1);
        //     updateAll();
        //   }
        //   drivetrain.stopIntake();
        //   intakeDone = false;
        // }
        drivetrain.runIntake(1);
        intakeDone = false;
      }
    }
    else {
      stowLogic();
      drivetrain.stopIntake();
    }

    //auton path test

    //driving
    runDrive();
    PestoLink.printBatteryVoltage(NoU3.getBatteryVoltage());

    //move to point test
    if (PestoLink.buttonHeld(13)) {
      drivetrain.moveToPoint({0, 1}, 0, odomPose);
    }
  }
  else {
    NoU3.setServiceLight(LIGHT_ON);
  }

  drivetrain.updateWheelOdometry(robotPose.h);

  drivetrain.stopShooter();
  drivetrain.loop();
  updatePose();
}
