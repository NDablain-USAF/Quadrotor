#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

class IMU {
  private:
    float
      magField[3],
      gyroBias[3],
      w_rad[3],
      magXcoef,
      magYcoef,
      magZcoef,
      psi_offset,
      // Filter constants, 0-1, increase c1 to lower cutoff frequency
      c1AccelGrav = 0.99, // Use for gravity vector from accelerometers 0.998
      c2AccelGrav = 1-c1AccelGrav,
      c1Mag = 0.995,
      c2Mag = 1-c1Mag,
      c1psi = 0.95,
      c2psi = 1-c1psi,
      // Random constants
      mres = 4912/32760, // Magnetometer resolution, bits to micro tesla
      b2mg = 1.1975, // Converts bits to milli g's for acceleration, assume 8 g range and 16 bit precision
      d2r = PI/180, // Converts degrees to radians
      L_Euler = 0.0032; // Kalman Gain, Euler Angles, 0.0032
    uint32_t
      timelast[2];
    int32_t
      accel[3],
      accelGrav[3];
    uint16_t
      calWait;
    int16_t
      angularRates[3],
      accelBias[3],
      accelGravBias[3];
    uint8_t
      DataM[7] = {1,2,3,4,5,6,7}, // Initial values used in test function, if changed must change check in Test
      onetime_KF,
      gyroRange = 250;
    int8_t
      DataAG[14] = {-6,-5,-4,-3,-2,-1,0,1,2,3,4,5,6,7}, // Initial values used in test function, if changed must change check in Test
      IMUAddress = 0x68,
      MAGAddress = 0x0C;
    void
      read();
  public:
    IMU();
    void
      Initialize(),
      Test(uint8_t *RESET_PIN),
      update(bool *calStatus),
      Kalman_filter_attitude(float EulerAngles[3], int16_t w[3]),
      Kalman_filter_altitude(float h[2], float *height_Measured, float EulerAngles[3],uint8_t mode);
};

#endif
