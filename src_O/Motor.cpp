#include "Motor.h"

MOTOR::MOTOR(uint8_t ENCODER1_PIN, uint8_t ENCODER2_PIN, uint8_t PWM_PIN, uint8_t NUMBER){
  encoder1_pin = ENCODER1_PIN;
  encoder2_pin = ENCODER2_PIN;
  pwm_pin = PWM_PIN;
  //mode1 = "RISING";
  //mode2 = "FALLING";
  number = NUMBER;
  pinMode(encoder1_pin, INPUT_PULLUP);
  pinMode(encoder2_pin, INPUT_PULLUP);
  pinMode(pwm_pin, OUTPUT);
  attach();
};

void MOTOR::attach(){
  if (number==1){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR1, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR2, FALLING);
  }
  else if (number==2){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR3, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR4, FALLING);
  }
  else if (number==3){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR5, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR6, FALLING);
  }
  else if (number==4){
    attachInterrupt(digitalPinToInterrupt(encoder1_pin), ISR7, RISING);
    attachInterrupt(digitalPinToInterrupt(encoder2_pin), ISR8, FALLING);
  }
}

void MOTOR::measure(){
  uint8_t count;
  noInterrupts();
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
  interrupts();
  w_measured = ((double)count/(pulse_per_rev*2))*(1e6/(micros()-time[1]))*2*PI; // Find the rpm, 17 pulses per revolution, 60000 ms per minute, dt ms per interval. 
  time[1] = micros(); // Reset for future interval 
}

void MOTOR::performance(){
  w_desired = reference[index[0]]; // Desired motor speed (rad/s)
  uint8_t number_points = sizeof(window)/sizeof(window[0]);
  window[index[2]] = w_measured;
  index[2]++;
  if (index[2]==number_points){
    index[2] = 0;
  }
  uint32_t sum = 0;
  for (byte i = 0;i<number_points;i++){
    sum += window[i];
  }
  double moving_avg = sum/number_points; 
  double e_ss = abs(moving_avg-w_desired); // Steady state error calculation
  if (w_measured>largest){
    largest = w_measured;
  }
  if ((abs(w_measured-w_desired)<10)&&(index[1]==0)){
    t_rise = millis()-time[3];
    index[1]++;
  }
  if ((abs(moving_avg-w_desired)<1.5)&&(index[3]<2)){
    t_settle = millis()-time[2]; // Settling time calculation
    percent_overshoot = ((largest-w_desired)/(w_desired))*100; // Percent overshoot calculation
    time[2] = millis();
    index[3]++;
    if (index[0]<1){
      index[0]++;
      time[3] = millis();
      index[1] = 0;
    }
  }
  /*
  Serial.print("Steady State Error (rad/s): ");
  Serial.print(e_ss);
  Serial.print("  ");
  Serial.print("Rise Time (ms): ");
  Serial.print(t_rise);
  Serial.print("  ");
  Serial.print("Settling Time (ms): ");
  Serial.print(t_settle);
  Serial.print("  ");
  Serial.print("Percent Overshoot (%): ");
  Serial.println(percent_overshoot);
*/
}

void MOTOR::DIMRAC(){
  uint32_t dt = (micros()-time[0])/1e6; // In microseconds
  time[0] = micros();
  double wrefdot = (aref*wref)+(bref*w_desired);
  wref += (wrefdot*dt);
  double e = w_measured-wref;
  double ahatdot = gama*w_measured*e;
  if ((abs(bhat)>bmin)||((bhat==bmin)&&(u*e>0))){
    bhatdot = gamb*u*e;
  }
  else{
    bhatdot = 0;
  }
  ahat += (ahatdot*dt);
  bhat += (bhatdot*dt);
  u = (1/bhat)*(((aref-ahat)*w_measured) + bref*w_desired);
  uint8_t input = constrain(u,0,255); // u constrained to range of 0-255
  Serial.println(input);
  analogWrite(pwm_pin,input);
}
