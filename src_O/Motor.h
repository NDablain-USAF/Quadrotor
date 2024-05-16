#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    double
      z_lead = -20, // Zero of lead compensator (s domain)
      z_lag = -2, // Zero of lag compensator
      p_lead = -45, // Pole of lead compensator (s domain)
      p_lag = -0.1, // Pole of lag compensator
      K = 5, // Gain of lead compensator
      Y, // Output of compensator
      Y_int, // Integrated compensator output
      Y_int2, // Double integrated compensator output
      X, // Compensator input
      X_int, // Integrated compensator input
      X_int2, // Double integrated compensator input
      e, // Difference between reference and measured speed values
      w_dot,
      w_measured;
    uint32_t
      time[2]; // time[0]: speed dt, time[1]: derivative control dt
    uint16_t
      interval = 1e4; // How often to measure speed (pulses)
    uint8_t
      CPR = 34,
      pwm_pin,
      input;
  public:
    MOTOR(uint8_t PWM_PIN);
    void 
      measure(uint16_t **Count),
      control(double w_desired, uint16_t *Count);
};

#endif