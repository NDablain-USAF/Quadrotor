#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    float
      w_measured,
      c1 = 0.95, // IIR constants for speed measurement
      c2 = 1-c1,
      e, // Difference between reference and measured speed values
      Y_int, // Integrated compensator output
      Y_int2, // float integrated compensator output
      X, // Compensator input
      X_int, // Integrated compensator input
      X_int2, // float integrated compensator input
      Y,
      K = 5.0, // Gain of lead compensator
      z_lead = -18.0, // -Zero of lead compensator (s domain)
      z_lag = -8.0, // Zero of lag compensator
      p_lead = -45.0, // Pole of lead compensator (s domain)
      p_lag = -0.1; // Pole of lag compensator
    uint32_t
      time[3]; // time[0]: speed dt, time[1]: derivative control dt, time[2]: test finish time
    uint16_t
      w_test, // Motor test speed
      interval = 1e4; // How often to measure speed (us)
    uint8_t
      CPR = 17, // Counts per revolution, RS555EN 24V motor has 17 pulses per revolution
      pwm_pin,
      input, // Duty cycle of pwm
      onetime_test;
  public:
    MOTOR(uint8_t PWM_PIN, uint16_t W_TEST);
    void 
      measure(volatile uint16_t **Count),
      control(int16_t w_desired, volatile uint16_t *Count),
      test(uint8_t *RESET_PIN, uint8_t *motor_cal);
};

#endif
