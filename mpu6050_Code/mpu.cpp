#include <Wire.h>
#include <MPU6050_light.h>
#include <mpu.h>

MPU6050 mpu(Wire);


void intitialize_mpu(){
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  //while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");
}

void MPU_UPDATE(){
   mpu.update();
}

float roll(){
  return (mpu.getAngleX());
}

float pitch(){
  return (mpu.getAngleY());
}