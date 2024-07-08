#ifndef LQR_H
#define LQR_H

#include <Arduino.h>

class LQR {
  private:
  float
    U[4],
    omega[4],
    Thrust,
    l = 0.25,
    decay_rate = 2.5e-4,
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
    K_altitude[2] = {10,14},
    e[3];
  public:
    LQR();
  void
    control_altitude(float *height, float *r),
    control_attitude(int16_t r[3], int16_t MotorSpeeds[4], float EulerAngles[3], int16_t w[3]);
};

#endif
