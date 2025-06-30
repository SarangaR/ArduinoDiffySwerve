#ifndef AZIMUTH_H
#define AZIMUTH_H

#include <Arduino.h>
#include <Alfredo_NoU3.h>
#include <SimpleFOC.h>
#include <SparkFun_TMAG5273_Arduino_Library.h>
#include <SparkFun_I2C_Mux_Arduino_Library.h>

class Azimuth {
private:
    TMAG5273* sensor;
    
    uint8_t i2cAddress = 0x35;
    uint8_t DEFAULT_ADDRESS = 0x35;

    uint8_t conversionAverage = TMAG5273_X32_CONVERSION;
    uint8_t magneticChannel = TMAG5273_XYX_ENABLE;
    uint8_t angleCalculation = TMAG5273_XY_ANGLE_CALCULATION;

    float wrapAngle(float angle);

    
public:
    Azimuth(TMAG5273* sensor, uint8_t i2cAddress, int channel);

    float offset = 0.0;
    int channel = 0;

    void init();
    float getPosition();
    void setI2cAddress(uint8_t address);
    void update();
};

#endif