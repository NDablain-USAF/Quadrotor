#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    double
      k = 25, // Compensator gain
      z = -375, // Zero of compensator (s domain)
      p = -800, // Pole of compensator (s domain)
      y, // Output of compensator
      x, // Output of controller
      e, // Difference between reference and measured speed values
      w_measured,
      percent_overshoot;
    uint32_t
      time[3]; // time[0]: speed dt, time[1]: reference switch dt, time[2]: derivative control dt
    uint16_t
      w_desired,
      intervalRef = 1e3, // How often to change reference value (ms)
      interval = 34, // How often to measure speed (pulses)
      reference[5] = {1200,1000,1500,792,950}; // Initial setpoint from rest, target step speed (rad/s)
    static volatile uint8_t
      counter1,
      counter2,
      counter3,
      counter4,
      counter5,
      counter6,
      counter7,
      counter8;
    uint8_t
      pulse_per_rev = 17,
      encoder1_pin,
      encoder2_pin,
      pwm_pin,
      number,
      input,
      index[4];
    static void
      ISR1(void),
      ISR2(void),
      ISR3(void),
      ISR4(void),
      ISR5(void),
      ISR6(void),
      ISR7(void),
      ISR8(void);
  public:
    MOTOR(uint8_t ENCODER1_PIN, uint8_t ENCODER2_PIN, uint8_t PWM_PIN, uint8_t NUMBER);
    void 
      attach(),
      measure(),
      performance(),
      control();
};

#endif
