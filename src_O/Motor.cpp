#include "Motor.h"

MOTOR::MOTOR(uint8_t PWM_PIN, uint16_t W_TEST){
  pwm_pin = PWM_PIN;
  w_test = W_TEST;
  pinMode(pwm_pin, OUTPUT);
};

void MOTOR::measure(volatile uint16_t **Count){
  // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval...
  // IIR implemented here as well, reset the count for future measurements
  w_measured = w_measured*c1 + (**Count*(1e6/(micros()-time[0]))*2*PI*c2)/CPR; // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
  time[0] = micros(); // Reset for future interval 
  **Count = 0;
}

void MOTOR::control(int16_t w_desired, volatile uint16_t *Count){ 
  // Motor speed is measured at a set time interval, this will allow speed decay when motor is not turning...
  // and generating pulses.
  if ((micros()-time[0])>=interval){
    measure(&Count);
  }
  // Motor is controlled with a lead/lag compensator, e is the output error, K is the compensator gain
  // X is the compensator input and Y is the compensator output, int and int2 are the first and second integrals
  e = w_desired-w_measured;
  X = K*e; 
  float dt = (micros()-time[1])/1e6;
  // Limit large integration step when first starting or restarting
  if (dt>0.05){ 
    dt = 0.005;
    X_int = 0;
    X_int2 = 0;
    Y = 0;
    Y_int = 0;
    Y_int2 = 0;
  }
  X_int += (X*dt);
  X_int2 += (X_int*dt);
  Y = X - ((z_lead+z_lag)*X_int) + (z_lead*z_lag*X_int2) + ((p_lead+p_lag)*Y_int) - (p_lead*p_lag*Y_int2); // Lead/lag compensation
  Y_int += (Y*dt);
  Y_int2 += (Y_int*dt);
  time[1] = micros();
  // Compensator output must be constrained to range of pwm, which is 8bits
  input = constrain(Y,0,255); // Range of pwm values
  analogWrite(pwm_pin,input);
}

void MOTOR::test(uint8_t *RESET_PIN, uint8_t *motor_cal){
  // Give the motor 2.5 seconds to reach the target speed, if the error is larger than 10 rad/s, trigger a reset
  int16_t w_cal = w_test-w_measured;
  if (onetime_test==0){
    time[2] = millis();
    ++onetime_test;
  }
  if ((millis()-time[2])>2.5e3){
    analogWrite(pwm_pin,0);
    if (abs(w_cal)>10){
      digitalWrite(*RESET_PIN,LOW);
    }
    *motor_cal = 1;
  }
}
