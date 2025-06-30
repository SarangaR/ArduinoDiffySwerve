/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Alfredo_NoU3_LSM6.h"

LSM6DSOXClass::LSM6DSOXClass()
{
}

LSM6DSOXClass::~LSM6DSOXClass()
{
}

int LSM6DSOXClass::begin(TwoWire& wire)
{
	_slaveAddress = LSM6DSOX_ADDRESS;
	
	_wire = &wire;

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }

  //set the gyroscope control register to work at 104 Hz, 2000 dps and in bypass mode
  //writeRegister(LSM6DSOX_CTRL2_G, 0x4C); //0100 1100
  //set the gyroscope control register to work at 26 Hz, 2000 dps and in bypass mode
  //writeRegister(LSM6DSOX_CTRL2_G, 0x2C); //0010 1100
  //set the gyroscope control register to work at 104 Hz, 500 dps and in bypass mode
  writeRegister(LSM6DSOX_CTRL2_G, 0x44); //0100 0100

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  writeRegister(LSM6DSOX_CTRL1_A, 0x4A);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);//0000 0000

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_A, 0x09);

  return 1;
}

void LSM6DSOXClass::end()
{
    writeRegister(LSM6DSOX_CTRL2_G, 0x00);
    writeRegister(LSM6DSOX_CTRL1_A, 0x00);
    _wire->end();
}

int LSM6DSOXClass::readAcceleration(float *x, float *y, float *z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_A, (uint8_t*)data, sizeof(data))) {
    *x = NAN;
    *y = NAN;
    *z = NAN;

    return 0;
  }

  //REORDERED FOR NOU3 CONVENTION
  *x = 1 * data[1] * 4.0 / 32768.0;
  *y = -1 * data[0] * 4.0 / 32768.0;
  *z =  1 * data[2] * 4.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DSOXClass::readGyroscope(float *x, float *y, float *z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    *x = NAN;
    *y = NAN;
    *z = NAN;

    return 0;
  }

  //REORDERED FOR NOU3 CONVENTION
  *x =  1 * float(data[1]) * (PI/180.0) * 500.0 / 32768.0;
  *y = -1 * float(data[0]) * (PI/180.0) * 500.0 / 32768.0;
  *z =  1 * float(data[2]) * (PI/180.0) * 500.0 / 32768.0;

  return 1;
}

int LSM6DSOXClass::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOXClass::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOXClass::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOXClass::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOXClass::gyroscopeSampleRate()
{
  return 104.0F;
}

void LSM6DSOXClass::enableInterrupt()
{
  writeRegister(LSM6DSOX_INT1_CTRL, 0x03); //0000 0011
}

int LSM6DSOXClass::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOXClass::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);

    if (_wire->endTransmission(false) != 0) {
      return -1;
    }

    if (_wire->requestFrom(_slaveAddress, length) != length) {
      return 0;
    }

    for (size_t i = 0; i < length; i++) {
      *data++ = _wire->read();
    }
  
  return 1;
}

int LSM6DSOXClass::writeRegister(uint8_t address, uint8_t value)
{

    _wire->beginTransmission(_slaveAddress);
    _wire->write(address);
    _wire->write(value);
    if (_wire->endTransmission() != 0) {
      return 0;
    }
  
  return 1;
}