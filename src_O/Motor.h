#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    float
      c1 = 0.95, // IIR constants for speed measurement
      c2 = 1-c1,
      Y_int, 
      Y_int2, 
      X, 
      X_int, 
      X_int2, 
      p_lag = -0.1; // Pole of lag compensator
    uint32_t
      time[2]; // time[0]: measurement dt, time[1]: compensator dt
    int16_t
      Y,
      z_lead = -25, // -Zero of lead compensator 
      z_lag = -2, // Zero of lag compensator
      p_lead = -45, // Pole of lead compensator 
      e; // Difference between reference and measured speed values
    uint16_t
      w_measured,
      K = 5, // Gain of lead compensator
      interval = 1e4; // How often to measure speed (microseconds)
    uint8_t
      CPR = 17, // Counts per revolution, RS555EN 24V motor has 17 pulses per revolution
      pwm_pin,
      input; // Duty cycle of pwm
  public:
    MOTOR(uint8_t PWM_PIN);
    void 
      measure(volatile uint16_t **Count),
      control(int16_t w_desired, volatile uint16_t *Count);
};

#endif
