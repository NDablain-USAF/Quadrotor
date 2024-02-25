#include "Motor.h"

MOTOR::MOTOR(uint8_t ENCODER1_PIN, uint8_t ENCODER2_PIN, uint8_t PWM_PIN, uint8_t NUMBER){
  encoder1_pin = ENCODER1_PIN;
  encoder2_pin = ENCODER2_PIN;
  pwm_pin = PWM_PIN;
  number = NUMBER;
  pinMode(encoder1_pin, INPUT_PULLUP);
  pinMode(encoder2_pin, INPUT_PULLUP);
  pinMode(pwm_pin, OUTPUT);
  attach();
};

void MOTOR::attach(){
  if (number==1){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR2, CHANGE);
  }
  else if (number==2){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR4, CHANGE);
  }
  else if (number==3){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR5, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR6, CHANGE);
  }
  else if (number==4){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR7, CHANGE);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR8, CHANGE);
  }
}

void MOTOR::measure(){
  double count;
  if (number==1){
    count = counter1+counter2;
    counter1 = 0;
    counter2 = 0;
  }
  else if (number==2){
    count = counter3+counter4;
    counter3 = 0;
    counter4 = 0;
  }
  else if (number==3){
    count = counter5+counter6;
    counter5 = 0;
    counter6 = 0;
  }
  else if (number==4){
    count = counter7+counter8;
    counter7 = 0;
    counter8 = 0;
  }
  double dt = micros()-time[0];
  double c1 = 0.9;
  double c2 = 1-c1;
  w_measured = w_measured*c1 + ((count/(pulse_per_rev*4))*(1e6/dt)*2*PI*c2); // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
  time[0] = micros(); // Reset for future interval 
}

void MOTOR::performance(){
  w_desired = reference[index[0]]; // Desired motor speed (rad/s)

  if ((millis()-time[1])>intervalRef){
    time[1] = millis();
    if (index[0]<4){
      index[0]++;
    }
    else {
      index[0] = 0;
    }
  }
  
  if ((counter1+counter2)>=interval){
    measure();
  }

  Serial.print(w_measured);
  Serial.print(",");
  Serial.print(w_desired);
  Serial.print(",");
}

void MOTOR::control(){
  double dt = (micros()-time[2])/1e6;
  if (dt>0.25){
    dt = 0.25;
  }
  e = w_desired-w_measured;
  double xdot = ((k*e)-x)/dt;
  x = k*e; 
  double ydot = xdot + (p*y) - (z*x); // Lead compensation
  y += (ydot*dt);
  time[2] = micros();
  input = constrain(y,0,255); // Range of pwm values
  Serial.println(input);
  analogWrite(pwm_pin,input);
}
