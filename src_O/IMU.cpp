#include "IMU.h"
#include <Wire.h>

IMU::IMU(){
}

void IMU::update(bool *calStatus){
  // Read
  int8_t i = -1;
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Accel x Higher Bit Address
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress,14);
  while(Wire.available()){
    DataAG[++i] = Wire.read();
  }
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x03);
  Wire.endTransmission();
  i = -1;
  Wire.requestFrom(MAGAddress, 7);
  while (Wire.available()){
    DataM[++i] = Wire.read();
  }
  // Update
  int16_t x = DataAG[0]<<8;
  x += DataAG[1];
  int32_t X = x*b2mg;
  accel[0] = X-accelBias[0];
  accelGrav[0] = accelGrav[0]*c1AccelGrav + (X-accelGravBias[0])*c2AccelGrav;
  int16_t y = DataAG[2]<<8;
  y += DataAG[3];
  int32_t Y = y*b2mg;
  accel[1] = Y-accelBias[1];
  accelGrav[1] = accelGrav[1]*c1AccelGrav + (Y-accelGravBias[1])*c2AccelGrav;
  int16_t z = DataAG[4]<<8;
  z += DataAG[5];
  int32_t Z = z*b2mg;
  accel[2] = Z-accelBias[2];
  accelGrav[2] = accelGrav[2]*c1AccelGrav + (Z-accelGravBias[2])*c2AccelGrav;
  float q = DataAG[8]<<8;
  q += (DataAG[9]/131);
  q = (q/gyroRange)-gyroBias[1];
  float p = DataAG[10]<<8;
  p += (DataAG[11]/131);
  p = (p/gyroRange)-gyroBias[0];
  float r = DataAG[12]<<8;
  r += (DataAG[13]/131);
  r = (r/gyroRange)-gyroBias[2];
  angularRates[0] = q;
  angularRates[1] = -p;
  angularRates[2] = r;
  int16_t mx = DataM[1]<<8;
  mx += DataM[0];
  int16_t my = DataM[3]<<8;
  my += DataM[2];
  int16_t mz = DataM[5]<<8;
  mz += DataM[4];
  magField[0] = (magField[0]*c1Mag) + ((float)mx*mres*magXcoef*c2Mag);
  magField[1] = (magField[1]*c1Mag) + ((float)my*mres*magYcoef*c2Mag);
  magField[2] = (magField[2]*c1Mag) + ((float)mz*mres*magZcoef*c2Mag);

  if (calWait==1000){
    for (int8_t i=-1;i<3;++i){
      gyroBias[i] = angularRates[i];
      accelBias[i] = accel[i];
      if (i<2){
        accelGravBias[i] = accelGrav[i];
      }
      else {
        accelGravBias[i] = accelGrav[i]-9810;
      }
    }
    ++calWait;
    *calStatus = true;
  }
  else if (calWait<1000){
    ++calWait;
    *calStatus = false;
  }
}

void IMU::Initialize(){
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

void IMU::Test(uint8_t *RESET_PIN){
  // Checks to see if data is being transmitted from IMU, if not then board will be reset.
  uint8_t i = 0;
  uint8_t check = 0;
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Accel x Higher Bit Address
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress,14);
  while(Wire.available()){
    DataAG[i++] = Wire.read();
  }
  for (uint8_t j=0;j<14;j++){
    if (DataAG[j]==(j-6)){
      check++;
    }
  }
  if (check==14){
    digitalWrite(*RESET_PIN,LOW);
  }
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x03);
  Wire.endTransmission();
  i = 0;
  check = 0;
  Wire.requestFrom(MAGAddress, 7);
  while (Wire.available()){
    DataM[i++] = Wire.read();
  }
  for (uint8_t j=0;j<7;j++){
    if (DataM[j]==(j+1)){
      check++;
    }
  }
  if (check==7){
    digitalWrite(*RESET_PIN,LOW);
  }
}

void IMU::Kalman_filter_attitude(float EulerAngles[3], int16_t w[3]){
  // This function implements an infinite horizon Kalman filter to estimate the Euler Angles (roll,pitch,yaw)...
  // of the drone. Gyro measurements are treated as inputs, this moves the trig. nonlinearities to the B matrix (xdot=Ax+Bu)..
  // so we can treat the system as Linear Time Invariant (LTI). It is assumed that the Euler Angles can be measured by finding the...
  // gravity vector and comparing its components for roll and pitch and using magnetic north from the magnetometer for yaw...
  // These measurements are passed through IIRs which act as digital low pass filters with very low cutoff frequencies.
  // This introduces phase lag, the Kalman filter compensates for this. Offline simulations were performed in Matlab to find the Kalman gain.
  w_rad[0] = angularRates[0]*d2r; // Convert deg/s to rad/s
  w_rad[1] = angularRates[1]*d2r;
  w_rad[2] = angularRates[2]*d2r;
  w[0] = angularRates[0];
  w[1] = angularRates[1];
  w[2] = angularRates[2];
  // Measure
  float theta_measured = atan2(accelGrav[0],sqrt(pow(accelGrav[1],2) + pow(accelGrav[2],2))); 
  float phi_measured = atan2(accelGrav[1],accelGrav[2]); 
  // A DCM is used to convert body frame magnetometer readings to inertial frame, rather than propogate DCM a continuous update of the...
  // relevant entries is made using Euler Angles.
  float mag_x = cos(-EulerAngles[0])*magField[0] + sin(EulerAngles[1])*sin(-EulerAngles[0])*magField[1] - cos(EulerAngles[1])*sin(-EulerAngles[0])*magField[2];
  float mag_y = cos(EulerAngles[1])*magField[1] + sin(EulerAngles[1])*magField[2];
  if (onetime_KF==0){ // Zero out the yaw
    psi_offset = atan2(mag_y,mag_x);
    onetime_KF++;
  }
  // IIR for yaw
  float psi_measured = c1psi*EulerAngles[2] + c2psi*(atan2(mag_y,mag_x)-psi_offset);
  float dt = (micros()-timelast[0])/1e6;
  if (dt>0.05){ // Limit large integration steps
    dt = 0.05;
  }
  timelast[0] = micros();
  // Because the measurements happen at the same time as the inputs are found, the prediction and update steps always occur at the same time.
  // Predict
  float phi_hat_k1_k0 = EulerAngles[0] + (w_rad[0] + sin(phi_measured)*tan(theta_measured)*w_rad[1] + cos(phi_measured)*tan(theta_measured)*w_rad[2])*dt;
  float theta_hat_k1_k0 = EulerAngles[1] + (cos(phi_measured)*w_rad[1] - sin(phi_measured)*w_rad[2])*dt;
  float psi_hat_k1_k0 = EulerAngles[2] + ((sin(phi_measured)/cos(theta_measured))*w_rad[1] + (cos(phi_measured)/cos(theta_measured))*w_rad[2])*dt;
  // Update
  EulerAngles[0] = phi_hat_k1_k0 + L_Euler*(phi_measured-phi_hat_k1_k0);
  EulerAngles[1] = theta_hat_k1_k0 + L_Euler*(theta_measured-theta_hat_k1_k0);
  EulerAngles[2] = psi_hat_k1_k0 + L_Euler*(psi_measured-psi_hat_k1_k0);
}

void IMU::Kalman_filter_altitude(float h[2], float *height_Measured, float EulerAngles[3], uint8_t mode){
  // This function implements an infite horizon Kalman filter to estimate the height of the drone above the ground.
  // Similar to the attitude Kalman filter, nonlinearities have been moved to the input matrix by treating the accelerometer...
  // measurements as inputs to the system. The system becomes LTI and a constant Kalman gain is found (and luckily it's unity!).
  // The vertical velocity is also estimated as it is included in the LQR controller
  float dt = (micros()-timelast[1])/1e6;
  if (dt>0.2){
    dt = 0.2;
  }
  timelast[1] = micros();
  // Predict
  float height_dot_hat_k1_k0 = h[1] + (-sin(EulerAngles[0])*accel[0] + cos(EulerAngles[1])*sin(EulerAngles[0])*accel[1] + cos(EulerAngles[1])*cos(EulerAngles[0])*accel[2])*(dt/1e3);
  float height_hat_k1_k0 = h[0] + height_dot_hat_k1_k0*dt;
  if (mode==0){ // Use prediction if between measurement steps
    h[1] = height_hat_k1_k0;
  }
  else if (mode==1){ // Incorporate measurement, Kalman Gain was found to be unity, assume no measurement available of rate of change of altitude
    h[0] = *height_Measured;
    h[1] += *height_Measured-height_hat_k1_k0;
  }
}
