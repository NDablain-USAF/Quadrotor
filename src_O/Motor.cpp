#include "Motor.h"

MOTOR::MOTOR(uint8_t PWM_PIN){
  pwm_pin = PWM_PIN;
  pinMode(pwm_pin, OUTPUT);
};

void MOTOR::measure(volatile uint16_t **Count){
  w_measured = w_measured*c1 + (**Count*(1e6/(micros()-time[0]))*2*PI*c2)/CPR; // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
  time[0] = micros(); // Reset for future interval 
  **Count = 0;
}

void MOTOR::control(int16_t w_desired, volatile uint16_t *Count){ 
  if ((micros()-time[0])>=interval){
    measure(&Count);
  }
  e = w_desired-w_measured;
  X = K*e; 
  float dt = (micros()-time[1])/1e6;
  if (dt>0.05){ // Limit large integration step when first starting or restarting
    dt = 0.05;
    X_int = 0;
    X_int2 = 0;
    Y = 0;
    Y_int = 0;
    Y_int2 = 0;
  }
  X_int += (X*dt);
  X_int2 += (X_int*dt);
  Y = X - (z_lead+z_lag)*X_int + z_lead*z_lag*X_int2 + (p_lead+p_lag)*Y_int - p_lead*p_lag*Y_int2; // Lead/lag compensation
  Y_int += (Y*dt);
  Y_int2 += (Y_int*dt);
  time[1] = micros();
  input = constrain(Y,0,255); // Range of pwm values
  analogWrite(pwm_pin,input);
}