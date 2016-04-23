#include<Wire.h>

const int MPU=0x68;  // I2C address of the MPU-6050

float GyX,GyY,GyZ; //rates measured using Gyro
float AccelAngleX,AccelAngleY,AccelAngleZ; //angles measured using accelerometer

//Kalman variables
float KalmanAngleX=0, gyroXBias=0, Xp00=90, Xp01=0, Xp10=0, Xp11=90; //angle around X-axis
float KalmanAngleY=0, gyroYBias=0, Yp00=90, Yp01=0, Yp10=0, Yp11=90; //angle around Y-axis
float KalmanAngleZ=0, gyroZBias=0, Zp00=90, Zp01=0, Zp10=0, Zp11=90; //angle around Z-axis
//float Q_gyroBias=0.001; 
//float Q_AccelAngle=0.003;
//float R=0.03;
float Q_gyroBias=0.005; 
float Q_AccelAngle=0.015;
float R=0.03;
//float deltaT=0.02; //noch anpassen an tatsächliche step Breite

long lastTime=0;
//long timeLastPrint=0;

void setup(){
  setupMPU6050();
  Serial.begin(115200);
  lastTime=micros();
//  timeLastPrint=micros();
}

void loop()
{

  getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2 and then to angles

  float delt=(micros()-lastTime)/1000000.0;
  lastTime=micros();
  //Serial.println(delt,6);
  
  kalmanUpdate(KalmanAngleX, gyroXBias, Xp00, Xp01, Xp10, Xp11, AccelAngleX, GyX, delt); //update angle around x-axis
  kalmanUpdate(KalmanAngleY, gyroYBias, Yp00, Yp01, Yp10, Yp11, AccelAngleY, GyY, delt); //update angle around y-axis
  kalmanUpdate(KalmanAngleZ, gyroZBias, Zp00, Zp01, Zp10, Zp11, AccelAngleZ, GyZ, delt); //update angle around z-axis

//  if(micros()-timeLastPrint>=20000)//limit serial output to 50Hz
//  {
    Serial.println("start");
    Serial.println(KalmanAngleX);
    Serial.println(KalmanAngleY);
    Serial.println(KalmanAngleZ);
//    timeLastPrint=micros();
//  }
}

void kalmanUpdate(float &KalmanAngle, float &gyroBias, float &p00, float &p01, float &p10, float &p11, float AngleFromAccel, float rateFromGyro, float deltaT) //pass values as references so that we only need one kalmanUpdate function for all angles (else we wouldnt know which global variables to update)
{
  //Step 1: Update state with system model (state is angle to be estimated and bias of gyro)
  float priorAngle=KalmanAngle+(rateFromGyro-gyroBias)*deltaT;
  float priorBias=gyroBias;
  //Step 2: Update covariance matrix of estimate with system model
  float p00Prior=p00+deltaT*(deltaT*p11-p01-p10+Q_AccelAngle);
  float p01Prior=p01-deltaT*p11;
  float p10Prior=p10-deltaT*p11;
  float p11Prior=p00+deltaT*Q_gyroBias;
  //Step 3: Use measurement to calculate deviation of measurement from prior
  float y=AngleFromAccel-priorAngle;
  //Step 4: Calculate Kalman gain K
  //Step 4a: Calculate S
  float S=p00Prior+R;
  //Step 4b: Calculate K
  float K0=1/S*p00Prior;
  float K1=1/S*p10Prior;
  //Step 5: Calculate posterior state
  KalmanAngle=priorAngle+K0*y;
  gyroBias=priorBias+K1*y; 
  //Step 6: Update covariance of estimate with measurement
  p00=p00Prior-p00Prior*K0;
  p01=p01Prior-p01Prior*K0;
  p10=p10Prior-p00Prior*K1;
  p11=p11Prior-p01Prior*K1;
}

void getAccelAndGyro()
{
  int16_t AcX_raw,AcY_raw,AcZ_raw,Tmp,GyX_raw,GyY_raw,GyZ_raw;
  float AcX,AcY,AcZ;
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

  //convert to angles
  AccelAngleX=atan(AcX/(sqrt(square(AcY)+square(AcZ))))*180/PI;
  AccelAngleY=atan(AcY/(sqrt(square(AcX)+square(AcZ))))*180/PI;
  AccelAngleZ=atan(-AcX/AcZ)*180/PI;
}

void setupMPU6050()
{
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

