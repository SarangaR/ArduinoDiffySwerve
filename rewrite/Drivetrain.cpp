#include "Drivetrain.h"
#include "PathFollower.h"
#include <algorithm>

Drivetrain::Drivetrain(Module* mod1, Module* mod2, Module* mod3) {
    modules[0] = mod1;
    modules[1] = mod2;
    modules[2] = mod3;
    otos = nullptr;
    pathFollower = nullptr;
}

void Drivetrain::init() {
    for (int i = 0; i < NUM_MODULES; i++) {
        modules[i]->init();
    }
}

float Drivetrain::getCurrentHeading() {
    if (otos != nullptr) {
        sfe_otos_pose2d_t pose;
        sfTkError_t status = otos->getPosition(pose);
        if (status == ksfTkErrOk) {
            return pose.h;
        }
    }
    if (pathFollower != nullptr) {
        return pathFollower->getCurrentHeading();
    }
    return 0.0;
}

void Drivetrain::drive(float vx, float vy, float omega, bool fieldOriented) {
    if (fieldOriented) {
        float heading = getCurrentHeading();

        float robot_vx = vx * cosf(heading) - vy * sinf(heading);
        float robot_vy = vx * sinf(heading) + vy * cosf(heading);

        vx = robot_vx;
        vy = robot_vy;
    }

    for (int i = 0; i < NUM_MODULES; i++) {
        float moduleY = modulePositions[i][0];
        float moduleX = modulePositions[i][1];

        float moduleVx = vx - omega * moduleY;
        float moduleVy = vy + omega * moduleX;

        float speed = hypot(moduleVx, moduleVy);// m/s
        float angle = (speed > 1e-6) ? atan2(moduleVy, moduleVx) : modules[i]->getCurrentAngle(); //rads

        float driveRPM = speed*60.0 / (PI * wheelRadius * 2);

        // optimize module angles
        float currentAngle = modules[i]->getCurrentAngle();
        // float targetAngle = placeAngleInScope(currentAngle * RAD_TO_DEG, angle * RAD_TO_DEG) * DEG_TO_RAD;
        float targetAngle = currentAngle;
        float delta = targetAngle - angle;
        while (delta > PI) delta -= 2*PI;
        while (delta < -PI) delta += 2*PI;

        if (fabs(delta) > (PI/2.0)) {
            driveRPM = -driveRPM;
            angle = delta > PI ? (angle - PI) : (angle + PI);
            while (angle > PI) angle -= 2*PI;
            while (angle < -PI) angle += 2*PI;
        }

        modules[i]->setDesiredState(driveRPM, angle);
    }
}

ChassisSpeeds Drivetrain::forwardKinematics(float driveRPMS[3], float anglesRad[3], float modulePositions[3][2], float wheelRadius) {
    float A[6][3]; // 2 rows per module: [vx, vy, omega]
    float b[6];

    for (int i = 0; i < 3; i++) {
        float rpm = driveRPMs[i];
        float angle = anglesRad[i];
        float x = modulePositions[i][1];
        float y = modulePositions[i][0];

        float speed = (rpm / 60.0f) * (2.0f * PI * wheelRadius);
        float vx_i = speed * cosf(angle);
        float vy_i = speed * sinf(angle);

        // vx = vx_robot - omega * y
        A[2 * i][0] = 1.0f;
        A[2 * i][1] = 0.0f;
        A[2 * i][2] = -y;
        b[2 * i] = vx_i;

        // vy = vy_robot + omega * x
        A[2 * i + 1][0] = 0.0f;
        A[2 * i + 1][1] = 1.0f;
        A[2 * i + 1][2] = x;
        b[2 * i + 1] = vy_i;
    }

    // Solve least squares using normal equations: A^T * A * x = A^T * b
    float AtA[3][3] = {0};
    float Atb[3] = {0};

    // Compute AtA and Atb
    for (int row = 0; row < 6; row++) {
        for (int col1 = 0; col1 < 3; col1++) {
            Atb[col1] += A[row][col1] * b[row];
            for (int col2 = 0; col2 < 3; col2++) {
                AtA[col1][col2] += A[row][col1] * A[row][col2];
            }
        }
    }

    // Solve 3x3 linear system AtA * x = Atb using Cramer's rule (or substitute with Gaussian if needed)
    // We'll use a basic matrix inverse (valid since it's 3x3 and well-conditioned)
    float det =
        AtA[0][0] * (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) -
        AtA[0][1] * (AtA[1][0] * AtA[2][2] - AtA[1][2] * AtA[2][0]) +
        AtA[0][2] * (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]);

    if (fabs(det) < 1e-6) {
        vx = vy = omega = 0.0f;
        return;
    }

    float invDet = 1.0f / det;

    float invAtA[3][3];

    invAtA[0][0] = (AtA[1][1] * AtA[2][2] - AtA[1][2] * AtA[2][1]) * invDet;
    invAtA[0][1] = (AtA[0][2] * AtA[2][1] - AtA[0][1] * AtA[2][2]) * invDet;
    invAtA[0][2] = (AtA[0][1] * AtA[1][2] - AtA[0][2] * AtA[1][1]) * invDet;

    invAtA[1][0] = (AtA[1][2] * AtA[2][0] - AtA[1][0] * AtA[2][2]) * invDet;
    invAtA[1][1] = (AtA[0][0] * AtA[2][2] - AtA[0][2] * AtA[2][0]) * invDet;
    invAtA[1][2] = (AtA[0][2] * AtA[1][0] - AtA[0][0] * AtA[1][2]) * invDet;

    invAtA[2][0] = (AtA[1][0] * AtA[2][1] - AtA[1][1] * AtA[2][0]) * invDet;
    invAtA[2][1] = (AtA[0][1] * AtA[2][0] - AtA[0][0] * AtA[2][1]) * invDet;
    invAtA[2][2] = (AtA[0][0] * AtA[1][1] - AtA[0][1] * AtA[1][0]) * invDet;

    vx = vy = omega = 0.0f;
    for (int i = 0; i < 3; i++) {
        vx += invAtA[0][i] * Atb[i];
        vy += invAtA[1][i] * Atb[i];
        omega += invAtA[2][i] * Atb[i];
    }
}

float Drivetrain::placeAngleInScope(float scopeReference, float newAngle) {
    float lowerBound = 0.0;
    float upperBound = 0.0;
    float lowerOffset = fmod(scopeReference, 360.0);
    if (lowerOffset >= 0.0) {
        lowerBound = scopeReference - lowerOffset;
        upperBound = scopeReference + (360.0 - lowerOffset);
    }
    else {
        upperBound = scopeReference - lowerOffset;
        lowerBound = scopeReference - (360.0 + lowerOffset);
    }
    while (newAngle < lowerBound) {
        newAngle += 360.0;
    }
    while (newAngle > upperBound) {
        newAngle -= 360.0;
    }
    if (newAngle - scopeReference > 180.0) {
        newAngle -= 360.0;
    } 
    else if (newAngle - scopeReference < -180.0) {
        newAngle += 360.0;
    }
    return newAngle;
}

void Drivetrain::stop() {
    for (int i = 0; i < NUM_MODULES; i++) {
        modules[i]->setDesiredState(0.0, modules[i]->getCurrentAngle());
    }
}

void Drivetrain::update() {
    for (int i = 0; i < NUM_MODULES; i++) {
        modules[i]->update();
    }
}

void Drivetrain::startAutoTune(bool tuneDrive) {
    Serial.println("Starting drivetrain auto-tune...");
    for (int i = 0; i < NUM_MODULES; i++) {
        Serial.print("Tuning module "); Serial.println(i);
        modules[i]->startAutoTune(tuneDrive);
        while (modules[i]->isTuning()) {
            modules[i]->update();
        }
        Serial.print("Module "); Serial.print(i);
        Serial.print(" tuning complete: ");
        Serial.println(modules[i]->getTuningStatus());
        delay(1000);
    }
    Serial.println("All modules tuned!");
}

bool Drivetrain::isTuning() {
    for (int i = 0; i < NUM_MODULES; i++) {
        if (modules[i]->isTuning()) {
            return true;
        }
    }
    return false;
}

void Drivetrain::printStatus() {
    Serial.println("=== Drivetrain Status ===");
    if (otos != nullptr) {
        Serial.print("Current Heading: ");
        Serial.print(getCurrentHeading() * 180.0 / PI);
        Serial.println("°");
    }

    for (int i = 0; i < NUM_MODULES; i++) {
        Serial.print("Module "); Serial.print(i);
        Serial.print(" - Drive: "); Serial.print(modules[i]->getDriveVelocity());
        Serial.print(" Turn: "); Serial.print(modules[i]->getTurnVelocity());
        Serial.print(" Top: "); Serial.print(modules[i]->getTopMotorVelocity());
        Serial.print(" Bottom: "); Serial.print(modules[i]->getBottomMotorVelocity());
        Serial.print(" Angle: "); Serial.print(modules[i]->getCurrentAngle());
        Serial.print(" Status: "); Serial.println(modules[i]->getTuningStatus());
    }
}
