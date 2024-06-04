#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    float
      c1 = 0.95,
      c2 = 1-c1,
      Y_int, // Integrated compensator output
      Y_int2, // float integrated compensator output
      X, // Compensator input
      X_int, // Integrated compensator input
      X_int2, // float integrated compensator input
      p_lag = -0.1; // Pole of lag compensator, scaled by 10
    uint32_t
      time[2]; // time[0]: speed dt, time[1]: derivative control dt
    int16_t
      Y,
      z_lead = -25, // -Zero of lead compensator (s domain)
      z_lag = -2, // Zero of lag compensator
      p_lead = -45, // Pole of lead compensator (s domain)
      e; // Difference between reference and measured speed values
    uint16_t
      w_measured,
      K = 5, // Gain of lead compensator
      interval = 1e4; // How often to measure speed (pulses)
    uint8_t
      CPR = 17,
      pwm_pin,
      input;
  public:
    MOTOR(uint8_t PWM_PIN);
    void 
      measure(volatile uint16_t **Count),
      control(int16_t w_desired, volatile uint16_t *Count);
};

#endif