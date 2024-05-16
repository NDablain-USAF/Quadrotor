#include "IMU.h"
#include <Wire.h>

IMU::IMU(){
}

void IMU::update(bool *calStatus){
  if (k>10){
    read();
    k = 0;
  }
  k++;
  int16_t x = DataAG[0]<<8;
  x += DataAG[1];
  int32_t X = (x*b2mg)-accelBias[0];
  accelGrav[0] = accelGrav[0]*c1AccelGrav + X*c2AccelGrav;
  int16_t y = DataAG[2]<<8;
  y += DataAG[3];
  int32_t Y = (y*b2mg)-accelBias[1];
  accelGrav[1] = accelGrav[1]*c1AccelGrav + Y*c2AccelGrav;
  int16_t z = DataAG[4]<<8;
  z += DataAG[5];
  int32_t Z = (z*b2mg)-accelBias[2];  
  accelGrav[2] = accelGrav[2]*c1AccelGrav + Z*c2AccelGrav;
  double q = DataAG[8]<<8;
  q += (DataAG[9]/131);
  q = (q/gyroRange)-gyroBias[1];
  double p = DataAG[10]<<8;
  p += (DataAG[11]/131);
  p = (p/gyroRange)-gyroBias[0];
  double r = DataAG[12]<<8;
  r += (DataAG[13]/131);
  r = (r/gyroRange)-gyroBias[2];
  angularRates[0] = p;
  angularRates[1] = q;
  angularRates[2] = r;
  float mres = 4912.0/32760.0;
  int16_t mx = DataM[1]<<8;
  mx += DataM[0];
  int16_t my = DataM[3]<<8;
  my += DataM[2];
  int16_t mz = DataM[5]<<8;
  mz += DataM[4];
  magField[0] = (magField[0]*c1Mag) + ((float)mx*mres*magXcoef*c2Mag);
  magField[1] = (magField[1]*c1Mag) + ((float)my*mres*magYcoef*c2Mag);
  magField[2] = (magField[2]*c1Mag) + ((float)mz*mres*magZcoef*c2Mag);
  if (calWait==5000){
    for (uint8_t i=0;i<3;i++){
      gyroBias[i] = angularRates[i];
      if (i<2){
        accelBias[i] = accelGrav[i];
      }
      else {
        accelBias[i] = accelGrav[i]-9810;
      }
    }
    ++calWait;
    *calStatus = true;
  }
  else if (calWait<5000){
    ++calWait;
    *calStatus = false;
  }
}

void IMU::read(){
  uint8_t i = 0;
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Accel x Higher Bit Address
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress,14);
  while(Wire.available()){
    DataAG[i++] = Wire.read();
  }
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x03);
  Wire.endTransmission();
  i = 0;
  Wire.requestFrom(MAGAddress, 7);
  while (Wire.available()){
    DataM[i++] = Wire.read();
  }
}

void IMU::configIMU(){
  uint8_t data[3];
  Wire.begin();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x00); // Reset
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x0F); // Set FuseROM mode
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x10);
  Wire.endTransmission();
  uint8_t i = 0;
  Wire.requestFrom(MAGAddress, 3);
  while( Wire.available() ) {
    data[i++] = Wire.read();
  }
  magXcoef = (float)(data[0] - 128) / 256.0 + 1.0;
  magYcoef = (float)(data[1] - 128) / 256.0 + 1.0;
  magZcoef = (float)(data[2] - 128) / 256.0 + 1.0;
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x00); // Reset
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0b00010010); // Set 16 bit readings and continuous mode
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C); // Accel config
  Wire.write(0b00001000); // Set range to +- 4g
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B); // Gyro config
  Wire.write(0b00000000); // Set range to 250 dps
  Wire.endTransmission();
  delay(10);
}

void IMU::Kalman_filter(double EulerAngles[3], double w[3]){
  w[0] = angularRates[0]*(PI/180); // Convert deg/s to rad/s
  w[1] = angularRates[1]*(PI/180);
  w[2] = angularRates[2]*(PI/180);
  double a_x = accelGrav[0]/1e3; // Convert mm/s^2 to m/s^2
  double a_y = accelGrav[1]/1e3;
  double a_z = accelGrav[2]/1e3;
  // Measure
  double theta_measured = atan2(a_x,sqrt(pow(a_y,2) + pow(a_z,2))); 
  double phi_measured = atan2(a_y,a_z); 
  double mag_x = cos(-EulerAngles[0])*magField[0] + sin(EulerAngles[1])*sin(-EulerAngles[0])*magField[1] - cos(EulerAngles[1])*sin(-EulerAngles[0])*magField[2];
  double mag_y = cos(EulerAngles[1])*magField[1] + sin(EulerAngles[1])*magField[2];
  if (onetime_KF==0){
    psi_offset = atan2(mag_y,mag_x);
    onetime_KF++;
  }
  double psi_measured = c1psi*EulerAngles[2] + c2psi*(atan2(mag_y,mag_x)-psi_offset);
  double dt = (micros()-timelast_KF)/1e6;
  timelast_KF = micros();
  // Predict
  double phi_hat_k1_k0 = EulerAngles[0] + (w[0] + sin(phi_measured)*tan(theta_measured)*w[1] + cos(phi_measured)*tan(theta_measured)*w[2])*dt;
  double theta_hat_k1_k0 = EulerAngles[1] + (cos(phi_measured)*w[1] - sin(phi_measured)*w[2])*dt;
  double psi_hat_k1_k0 = EulerAngles[2] + ((sin(phi_measured)/cos(theta_measured))*w[1] + (cos(phi_measured)/cos(theta_measured))*w[2])*dt;
  // Residual
  double phi_bar = phi_measured-phi_hat_k1_k0;
  double theta_bar = theta_measured-theta_hat_k1_k0;
  double psi_bar = psi_measured-psi_hat_k1_k0;
  // Update
  double phi = phi_hat_k1_k0 + L*phi_bar;
  double theta = theta_hat_k1_k0 + L*theta_bar;
  double psi = psi_hat_k1_k0 + L*psi_bar;
  EulerAngles[0] = phi;
  EulerAngles[1] = theta;
  EulerAngles[2] = psi;
}