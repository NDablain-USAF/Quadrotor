#ifndef LQR_H
#define LQR_H

#include <Arduino.h>

class LQR {
  private:
  double
    K[3][6],
    m = 3,
    g = 9.81,
    W = m*g,
    l = 0.25,
    K_f = 1399e-8, // Force constant
    K_t = 632e-8; // Torque constant
  uint8_t 
    mode; // 1 for attitude, 2 for position
  void
    updateGains(double x[6]),
    attitudeControl(double u[3], double x[6]),
    positionControl(double u[4], double x[6]);
  public:
    LQR(uint8_t MODE);
  void
    Control(double r[6], double u[], double x[6]);
};

#endif