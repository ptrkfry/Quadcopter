#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

float AcX,AcY,AcZ,GyX,GyY,GyZ; //real raw values (converted from 16bit number)
float angleAroundX,angleAroundY,angleAroundZ;

void setup(){
  setupMPU6050();
  Serial.begin(115200);
}

void loop()
{
  getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2
  calculateAnglesFromAccelerations();
  outputValues();
  delay(200);
}

void calculateAnglesFromAccelerations()
{
  angleAroundX=atan(AcX/(sqrt(square(AcY)+square(AcZ))))*180/PI;
  angleAroundY=atan(AcY/(sqrt(square(AcX)+square(AcZ))))*180/PI;
  angleAroundZ=atan(-AcX/AcZ)*180/PI;
}

void outputValues()
{
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  //Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.print(GyZ);
  Serial.print(" | angleAroundX = "); Serial.print(angleAroundX); //roll
  Serial.print(" | angleAroundY = "); Serial.print(angleAroundY); //pitch
  Serial.print(" | angleAroundZ = "); Serial.println(angleAroundZ); //yaw
}

void getAccelAndGyro()
{
  int16_t AcX_raw,AcY_raw,AcZ_raw,Tmp,GyX_raw,GyY_raw,GyZ_raw;
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true);  // request a total of 14 registers
  //read registers
  AcX_raw=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY_raw=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ_raw=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX_raw=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY_raw=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ_raw=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //convert to real Values
  AcX=AcX_raw*0.000598755;
  AcY=AcY_raw*0.000598755;
  AcZ=AcZ_raw*0.000598755;
  GyX=GyX_raw*0.00762939;
  GyY=GyY_raw*0.00762939;
  GyZ=GyZ_raw*0.00762939;
}

void setupMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

