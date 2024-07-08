#include "LQR.h"

LQR::LQR(){
}

void LQR::control_attitude(int16_t r[3], int16_t MotorSpeeds[4], float EulerAngles[3], int16_t w[3]){
  uint32_t dt = millis()-timelast[0];
  if (dt>100){
    dt = 100;
  }
  timelast[0] = millis();
  // Solve for torques: Tau_phi, Tau_theta, Tau_psi
  for (uint8_t i=0;i<3;i++){
    // Output tracking
    y[i] = EulerAngles[i]*(180/PI);
    e[i] = r[i]-y[i];
    if (abs(U[i+1])<2){
      e_int_attitude[i] += K_attitude[i][0]*e[i]*dt;
    }
    // State regulation, interestingly a stationary LQR can be nearly approximated with a PID (P: Euler Angle, I: Integrated Euler Angle, D: Angular Rate)
    // The only discrepancy being the reference frame difference of Euler angles and angular rates
    u[i] = e_int_attitude[i] - K_attitude[i][1]*y[i] - K_attitude[i][2]*w[i];
  }
  for (uint8_t i=1;i<4;i++){
    // Back to floating point numbers
    U[i] = u[i-1]*(PI/(180*1e5));
  }
  U[0] = Thrust;
  // Solve for corresponding motor speeds (squared motor speeds)
  omega[3] = U[0]/(4*K_f) - U[3]/(4*K_t) - U[2]/(2*K_f*l);
  omega[0] = U[0]/(4*K_f) + U[3]/(4*K_t) - U[1]/(2*K_f*l);
  omega[1] = U[0]/(4*K_f) - U[3]/(4*K_t) + U[2]/(2*K_f*l);
  omega[2] = U[0]/(4*K_f) + U[3]/(4*K_t) + U[1]/(2*K_f*l);
  for (uint8_t i=0;i<4;i++){
    omega[i] = constrain(omega[i],0,1e6);
    MotorSpeeds[i] = sqrt(omega[i]);
  }
}

void LQR::control_altitude(float *height, float *r){
  // Actually just a PID
  *r -= decay_rate;
  static float e_int;
  float dt = (micros()-timelast[1])/1e6;
  if (dt>0.1){
    dt = 0.1;
  }
  timelast[1] = micros();
  float e = *r-*height;
  if ((Thrust<60)&&(Thrust>0)){
    e_int += e*dt;
  }
  Thrust = K_altitude[0]*e_int + K_altitude[1]*e;
  Thrust = constrain(Thrust,0,60);
}
