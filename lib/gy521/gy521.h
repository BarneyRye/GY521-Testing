#ifndef GY521_H
#define GY521_H

#include <Wire.h>

enum AccelRange {
    ACCEL_2G,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G
};

enum GyroRange {
    GYRO_250DPS,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS
};

class GY521 {
    public:
        GY521(uint8_t address = 0x68); //Deafult address for MPU-6050
        void begin();
        void setAccelRange(AccelRange range);
        void setGyroRange(GyroRange range);
        void readAccel(float &AcX, float &AcY, float &AcZ);
        void readGyro(float &GyX, float &GyY, float &GyZ);
        float readTemp();
    private:
        uint8_t _address;
        float _accelDivider = 16384.0; // Default for ACCEL_2G
        float _gyroDivider = 131.0;    // Default for GYRO_250DPS
};

#endif // GY521_H