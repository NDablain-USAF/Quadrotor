#ifndef LQR_H
#define LQR_H

#include <Arduino.h>

class LQR {
  private:
  float
    U[4],
    omega[4],
    Thrust,
    e_int_altitude,
    l = 0.25,
    K_f = 1399e-8, // Force constant
    K_t = 632e-8; // Torque constant
  uint32_t
    timelast[2];
  int32_t
    K_attitude[3][3] = {{314,359901,49183},{314,359901,49183},{315,378994,70875}},
    u[3],
    e_int_attitude[3];
  int16_t
    y[3];
  int8_t
    K_altitude[3] = {10,14,10},
    e[3];
  public:
    LQR();
  void
    control_altitude(float h[2], float *r),
    control_attitude(int16_t r[3], int16_t MotorSpeeds[4], float EulerAngles[3], int16_t w[3]);
};

#endif