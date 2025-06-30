#include "Azimuth.h"
#include "Wire.h"

Azimuth::Azimuth(TMAG5273* sensor, uint8_t i2cAddress, int channel) {
    this->sensor = sensor;
    this->i2cAddress = i2cAddress;
    this->channel = channel;
}

void Azimuth::init() {
    sensor->setConvAvg(conversionAverage);
    sensor->setMagneticChannel(magneticChannel);
    sensor->setAngleEn(angleCalculation);
}

void Azimuth::setI2cAddress(uint8_t address) {
  while (sensor->setI2CAddress(address) != 0) {
    Serial.print("Sensor "); Serial.print(channel); Serial.println(" address change failed!");
  }
  Serial.print("Sensor "); Serial.print(channel); Serial.print(" address changed to "); Serial.println(sensor->getI2CAddress(), HEX);
}

float Azimuth::getPosition() {
    if((sensor->getMagneticChannel() != 0) && (sensor->getAngleEn() != 0)) // Checks if mag channels are on - turns on in setup
    {
      float angleCalculation = sensor->getAngleResult();
      angleCalculation = wrapAngle(angleCalculation);
      float angleMag = sensor->getMagnitudeResult();

      if (abs(angleMag) < 0.1) angleCalculation = 0.0;

      return angleCalculation * DEG_TO_RAD;
    }
    else
    {
      Serial.print("Sensor "); Serial.print(channel); Serial.println("  mag channels disabled!");
      return 0.0;
    }
}

float Azimuth::wrapAngle(float angle) {
    while (angle <= -180) angle += 360;
    while (angle > 180) angle -= 360;
    return angle;
}