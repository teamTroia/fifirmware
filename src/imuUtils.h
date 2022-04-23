#include "Arduino.h"
#include "Wire.h"

TwoWire Wire2 (2, I2C_FAST_MODE);
#define MPU_ADDR 0x68
void init_mpu(){
  Wire2.begin();
  Wire2.beginTransmission(MPU_ADDR);
  Wire2.write(0x6B);
  Wire2.write(0); 
  Wire2.endTransmission(true);
}

int readAngularSpeed(){
  int AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
  Wire2.beginTransmission(MPU_ADDR);
  Wire2.write(0x3B);
  Wire2.endTransmission(false);
  Wire2.requestFrom(MPU_ADDR,14,true);  
  AcX=Wire2.read()<<8|Wire2.read();  //0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire2.read()<<8|Wire2.read();  //0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire2.read()<<8|Wire2.read();  //0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire2.read()<<8|Wire2.read();  //0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire2.read()<<8|Wire2.read();  //0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire2.read()<<8|Wire2.read();  //0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire2.read()<<8|Wire2.read();  //0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  delay(333);
  return GyZ;
}   