#include <Arduino.h>
#include "gy521.h"

typedef struct {
  uint32_t time;
  float AcX;
  float AcY;
  float AcZ;
  float GyX;
  float GyY;
  float GyZ;  
  float Tmp;
}dataStruct;

dataStruct data;
GY521 imu;

void  setup(){

  Serial.begin(115200); //Start serial communication at 115200 baud rate
  imu.begin(); //Initialize the GY521 sensor
  imu.setAccelRange(ACCEL_2G); //Set accelerometer range to ±2g
  imu.setGyroRange(GYRO_250DPS); //Set gyroscope range to ±250°/s

}
void  loop(){
  uint32_t currentTime = millis();
  imu.readAccel(data.AcX, data.AcY, data.AcZ);  //Read accelerometer data
  imu.readGyro(data.GyX, data.GyY, data.GyZ);   //Read gyroscope data
  data.Tmp = imu.readTemp();                    //Read temperature data
  data.time = millis();                         //Get current time in milliseconds

  Serial.print(data.time); Serial.print(","); //Print to serial in .CSV format
  Serial.print(data.AcX); Serial.print(",");
  Serial.print(data.AcY); Serial.print(",");
  Serial.print(data.AcZ); Serial.print(",");
  Serial.print(data.GyX); Serial.print(",");
  Serial.print(data.GyY); Serial.print(",");
  Serial.print(data.GyZ); Serial.print(",");
  Serial.println(data.Tmp);

  while (millis() - currentTime < 100) {
    // Wait to maintain a 100ms interval
  }

}
