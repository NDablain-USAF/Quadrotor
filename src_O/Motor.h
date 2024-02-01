#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class MOTOR {
  private:
    double
      gama = 0.00003,
      gamb = 0.00003,
      bhat = 5, 
      bmin = 1,
      u = 0,
      wref,
      ahat,
      bhatdot,
      w_measured,
      percent_overshoot;
    uint16_t
      w_desired,
      largest,
      t_rise,
      t_settle,
      reference[2] = {1200,950}, // Initial setpoint from rest, target step speed (rad/s)
      window[25]; // Establish an array "window" of size n
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
      bref = 7,
      pulse_per_rev = 17,
      encoder1_pin,
      encoder2_pin,
      pwm_pin,
      number,
      index[4];
    int8_t
      aref = -7;
    //char 
    //  mode1[7] = "RISING",
    //  mode2[8] = "FALLING";
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
      DIMRAC();
    uint32_t
      time[4];
    uint16_t
      interval = 1e3;
};

#endif
