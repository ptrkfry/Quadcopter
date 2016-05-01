#include<Wire.h> //for IMU
#include <Servo.h> //for ESC

// IMU
const int MPU=0x68;  // I2C address of the MPU-6050
  //Measurements
float GyX,GyY,GyZ; //rates measured using Gyro
float AccelAngleX,AccelAngleY,AccelAngleZ; //angles measured using accelerometer
  //Kalman variables
float KalmanAngleX=0, gyroXBias=0, Xp00=90, Xp01=0, Xp10=0, Xp11=90; //angle around X-axis
float KalmanAngleY=0, gyroYBias=0, Yp00=90, Yp01=0, Yp10=0, Yp11=90; //angle around Y-axis
float KalmanAngleZ=0, gyroZBias=0, Zp00=90, Zp01=0, Zp10=0, Zp11=90; //angle around Z-axis
float Q_gyroBias=0.005; // process noise gyro bias
float Q_AccelAngle=0.015; //process noise accelerometer
float R=0.03; //measurement noise accelerometer
long lastTime=0; //time of last kalman update


//PPM Reader
const int numberOfChannels=6;
volatile long currentTime;
volatile long delta;
volatile long lastPulse=0;
// which channel is which axis or throttle???????????????????
volatile int pulseLengths[numberOfChannels+1]; //framespace uses one space too
volatile int channel=0;
volatile bool start=false;

//ESC output
Servo ESC_1,ESC_2,ESC_3,ESC_4;  // create servo object to control ESC, any pins can be used

//PID control
float Kp=2;
float Ki=0.3;
float Kd=0;

void setup() 
{
  Serial.begin(115200);
  
  //IMU
  setupMPU6050();
  lastTime=micros();

  //PPM Reader
  attachInterrupt(digitalPinToInterrupt(2),pulseDetected,RISING);

  //ESC
  ESC_1.attach(9); //.attach(pin)
  ESC_2.attach(10);
  ESC_3.attach(11);
  ESC_4.attach(12);
  ESC_1.writeMicroseconds(1000); //write lowest value to ESC, to avoid it going into calibration or programming mode and to arm it
  ESC_2.writeMicroseconds(1000);
  ESC_3.writeMicroseconds(1000);
  ESC_4.writeMicroseconds(1000);
}

void loop() 
{
  //IMU and Kalman
  getAccelAndGyro(); //get values from registers of MPU6050 and convert Accelerations to m/s^2 and then to angles
  float delt=(micros()-lastTime)/1000000.0;
  lastTime=micros(); 
  kalmanUpdate(KalmanAngleX, gyroXBias, Xp00, Xp01, Xp10, Xp11, AccelAngleX, GyX, delt); //update angle around x-axis
  kalmanUpdate(KalmanAngleY, gyroYBias, Yp00, Yp01, Yp10, Yp11, AccelAngleY, GyY, delt); //update angle around y-axis
  //kalmanUpdate(KalmanAngleZ, gyroZBias, Zp00, Zp01, Zp10, Zp11, AccelAngleZ, GyZ, delt); //update angle around z-axis //not needed?
    
  //PPM Reader
  //debugPPMReader();
  float throttle_setpoint=0, x_setpoint=0, y_setpoint=0, z_setpoint=0;
  mapValues(throttle_setpoint, x_setpoint, y_setpoint, z_setpoint);
//  Serial.print(x_setpoint);
//  Serial.print(" / ");
//  Serial.print(y_setpoint);
//  Serial.print(" /");
//  Serial.print(throttle_setpoint);
//  Serial.print(" / ");
//  Serial.println(z_setpoint);

  //PID
  PIDcontrol(KalmanAngleX,KalmanAngleY,GyZ, delt); //calculation and output to ESC
}

void PIDcontrol(float KalmanAngleX, float KalmanAngleY, float GyZ, float dT)
{
  static float I_x=0, I_y=0;
  static float lastErr_x, lastErr_y;
  float err_x, err_y;
  float PID_x, PID_y, PID_z, throttle=0, D_x, D_y; //get throttle from array of PPM Reader + scale?
  float controlSignal_1, controlSignal_2, controlSignal_3, controlSignal_4;
  
  //PID
  err_x=0-KalmanAngleX; // "0" (= desired angle) noch anpassen nach Einbau
  err_y=0-KalmanAngleY;

  I_x+=err_x;
  D_x=(err_x-lastErr_x)/dT; //Rückwärtsdifferenzenquotient
  PID_x=Kp*err_x+Ki*I_x+Kd*D_x;

  
  //Calculation x-Axis (Motor 1 and 3)
  controlSignal_1=throttle+PID_x-PID_z;
  controlSignal_3=throttle-PID_x-PID_z;

  //Calculation y-Axis (Motor 2 and 4)
  controlSignal_1=throttle+PID_y+PID_z;
  controlSignal_3=throttle-PID_y+PID_z;

  //Calculation z-Axis


  lastErr_x=err_x;
  lastErr_y=err_y;
  
  // Output
  //scale to usable scale first
  outputToESC(controlSignal_1, controlSignal_2, controlSignal_3, controlSignal_4);
}

void outputToESC(int signalESC_1, int signalESC_2, int signalESC_3, int signalESC_4)
{
      ESC_1.writeMicroseconds(signalESC_1);
      ESC_2.writeMicroseconds(signalESC_2);
      ESC_3.writeMicroseconds(signalESC_3);
      ESC_4.writeMicroseconds(signalESC_4);
}


/*
 * 
 * IMU and Kalman Functions
 * 
 * 
 */
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

/*
 * 
 * PPM Reader Functions
 * 
 * 
 */

void mapValues(float &throttle_setpoint, float &x_setpoint, float &y_setpoint, float &z_setpoint)
{
  //!! map function does not constrain values to within the range!!
  x_setpoint=map(pulseLengths[1],1004,1996,-45,45); //channel 1, x-axis? (right stick)
  y_setpoint=map(pulseLengths[2],1000,1992,-45,45); //channel 2, y-axis? (right stick)
  throttle_setpoint=map(pulseLengths[3],1010,1992,0,100); //channel 3, throttle (left stick)
  z_setpoint=map(pulseLengths[4],1004,1992,-100,100); //channel 3, z-axis (left stick)
}
 
void debugPPMReader()
{
   //debug for ppm reader
  for(int i=1;i<numberOfChannels+1;i++)
  {
   Serial.print(pulseLengths[i]);
   if(i<numberOfChannels) //no forward slash at end
   {
    Serial.print(" / ");
   }
  }
  Serial.print("\n"); 

  delay(200);
}

//ISR for PPM Reader
void  pulseDetected()
{
  currentTime=micros();
  delta=currentTime-lastPulse;
  lastPulse=currentTime;
  if(delta>10000)
  {
    //Serial.println(" ");
    channel=0;
    start=true;
  }
  else
  {
    channel=channel+1;
  }
  
  if(start)
  {
    //Serial.println(delta);
    pulseLengths[channel]=delta;
  }
}


