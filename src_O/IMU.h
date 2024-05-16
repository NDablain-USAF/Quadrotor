#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

class IMU {
  private:
    double
      angularRates[3],
      magField[3],
      EulerAngles[3],
      gyroBias[3],
      accelBias[3],
      magXcoef,
      magYcoef,
      magZcoef,
      psi_offset,
      // Filter constants, 0-1, increase c1 to lower cutoff frequency
      c1AccelGrav = 0.99, // Use for gravity vector from accelerometers 0.998
      c2AccelGrav = 1-c1AccelGrav,
      c1Mag = 0.999,
      c2Mag = 1-c1Mag,
      c1psi = 0.95,
      c2psi = 1-c1psi,
      // Random constants
      mres = 4912/32760, // Magnetometer resolution, bits to micro tesla
      b2mg = 1.1975, // Converts bits to milli g's for acceleration, assume 8 g range and 16 bit precision
      L = 0.0057; // Kalman Gain
    uint32_t
      timelast_KF;
    int32_t
      accelGrav[3];
    uint16_t
      calWait;
    uint8_t
      DataM[7],
      k,
      onetime_KF,
      gyroRange = 250;
    int8_t
      DataAG[14],
      IMUAddress = 0x68,
      MAGAddress = 0x0C;
    void
      read();
  public:
    IMU();
    void
      configIMU(),
      update(bool *calStatus),
      Kalman_filter(double EulerAngles[3], double w[3]);
};

#endif