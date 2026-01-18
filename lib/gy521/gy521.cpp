#include "gy521.h"

GY521::GY521(uint8_t address) {
    _address = address;
}

void GY521::begin() {
    Wire.begin();
    Wire.beginTransmission(_address);
    Wire.write(0x6B); // PWR_MGMT_1 register
    Wire.write(0);    // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
}

void GY521::setAccelRange(AccelRange range) {
    uint8_t regValue = 0;
    switch (range) {
        case ACCEL_2G:
            regValue = 0b00000000;
            _accelDivider = 16384.0;
            break;
        case ACCEL_4G:
            regValue = 0b00001000;
            _accelDivider = 8192.0;
            break;
        case ACCEL_8G:
            regValue = 0b00010000;
            _accelDivider = 4096.0;
            break;
        case ACCEL_16G:
            regValue = 0b00011000;
            _accelDivider = 2048.0;
            break;
    }
    Wire.beginTransmission(_address);
    Wire.write(0x1C); // ACCEL_CONFIG register
    Wire.write(regValue);
    Wire.endTransmission(true);
}

void GY521::setGyroRange(GyroRange range) {
    uint8_t regValue = 0;
    switch (range) {
        case GYRO_250DPS:
            regValue = 0b00000000;
            _gyroDivider = 131.0;
            break;
        case GYRO_500DPS:
            regValue = 0b00001000;
            _gyroDivider = 65.5;
            break;
        case GYRO_1000DPS:
            regValue = 0b00010000;
            _gyroDivider = 32.8;
            break;
        case GYRO_2000DPS:
            regValue = 0b00011000;
            _gyroDivider = 16.4;
            break;
    }
    Wire.beginTransmission(_address);
    Wire.write(0x1B); // GYRO_CONFIG register
    Wire.write(regValue);
    Wire.endTransmission(true);
}

void GY521::readAccel(float &AcX, float &AcY, float &AcZ) {
    float rawX, rawY, rawZ;
    Wire.beginTransmission(_address);
    Wire.write(0x3B); // Starting register for Accel Readings
    Wire.endTransmission(false);
    Wire.requestFrom(_address, 6, true); // Request 6 registers

    rawX = (Wire.read() << 8 | Wire.read());
    rawY = (Wire.read() << 8 | Wire.read());
    rawZ = (Wire.read() << 8 | Wire.read());

    AcX = rawX / _accelDivider;
    AcY = rawY / _accelDivider;
    AcZ = rawZ / _accelDivider;
}

void GY521::readGyro(float &GyX, float &GyY, float &GyZ) {
    float rawX, rawY, rawZ;
    Wire.beginTransmission(_address);
    Wire.write(0x43); // Starting register for Gyro Readings
    Wire.endTransmission(false);
    Wire.requestFrom(_address, 6, true); // Request 6 registers

    rawX = (Wire.read() << 8 | Wire.read());
    rawY = (Wire.read() << 8 | Wire.read());
    rawZ = (Wire.read() << 8 | Wire.read());

    GyX = rawX / _gyroDivider;
    GyY = rawY / _gyroDivider;
    GyZ = rawZ / _gyroDivider;
}

float GY521::readTemp() {
    Wire.beginTransmission(_address);
    Wire.write(0x41); // Starting register for Temperature Readings
    Wire.endTransmission(false);
    Wire.requestFrom(_address, 2, true); // Request 2 registers

    int16_t temp = (Wire.read() << 8 | Wire.read());
    return temp / 340.0 + 36.53; // Convert to Celsius
}