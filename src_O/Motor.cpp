#include "Motor.h"

MOTOR::MOTOR(uint8_t PWM_PIN){
  pwm_pin = PWM_PIN;
  pinMode(pwm_pin, OUTPUT);
};

void MOTOR::measure(uint16_t **Count){
  double counter = **Count;
  **Count = 0;
  double dt = micros()-time[0];
  double c1 = 0.95;
  double c2 = 1-c1;
  w_measured = w_measured*c1 + ((counter/CPR)*(1e6/dt)*2*PI*c2); // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
  time[0] = micros(); // Reset for future interval 
}

void MOTOR::control(double w_desired, uint16_t *Count){ 
  if ((micros()-time[0])>=interval){
    measure(&Count);
  }
  e = w_desired-w_measured;
  X = K*e; 
  Y = X - (z_lead+z_lag)*X_int + z_lead*z_lag*X_int2 + (p_lead+p_lag)*Y_int - p_lead*p_lag*Y_int2; // Lead/lag compensation
  double dt = (micros()-time[1])/1e6;
  if (dt>0.05){ // Limit large integration step when first starting
    dt = 0.05;
  }
  X_int += X*dt;
  X_int2 += X_int*dt;
  Y_int += Y*dt;
  Y_int2 += Y_int*dt;
  time[1] = micros();
  input = constrain(Y,0,255); // Range of pwm values
  analogWrite(pwm_pin,input);
}