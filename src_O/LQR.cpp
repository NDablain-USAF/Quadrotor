#include "LQR.h"

LQR::LQR(uint8_t MODE){
  mode = MODE;
}

void LQR::Control(double r[6], double u[], double x[6]){
  double e[6];
  for (uint8_t i=0;i<6;i++){
    e[i] = r[i]-x[i];
  }
  updateGains(x);
  if (mode==1){
    attitudeControl(u,e);
  }
  else if (mode==2){
    positionControl(u,x);
  }
}

void LQR::positionControl(double u[4], double x[6]){

}

void LQR::attitudeControl(double omega[4], double x[6]){
  double U[3];
  // Solve for torques: Tau_phi, Tau_theta, Tau_psi
  U[0] = K[0][0]*x[0] + K[0][1]*x[1] + K[0][2]*x[2] + K[0][3]*x[3] + K[0][4]*x[4] + K[0][5]*x[5];
  U[1] = K[1][0]*x[0] + K[1][1]*x[1] + K[1][2]*x[2] + K[1][3]*x[3] + K[1][4]*x[4] + K[1][5]*x[5];
  U[2] = K[2][0]*x[0] + K[2][1]*x[1] + K[2][2]*x[2] + K[2][3]*x[3] + K[2][4]*x[4] + K[2][5]*x[5];
  // Solve for corresponding motor speeds
  omega[1] = sqrt(W/(4*K_f) - U[2]/(4*K_t) - U[1]/(2*K_f*l));
  omega[2] = sqrt(W/(4*K_f) + U[2]/(4*K_t) - U[0]/(2*K_f*l));
  omega[3] = sqrt(W/(4*K_f) - U[2]/(4*K_t) + U[1]/(2*K_f*l));
  omega[0] = sqrt(W/(4*K_f) + U[2]/(4*K_t) + U[0]/(2*K_f*l));
}

void LQR::updateGains(double x[6]){
  if (mode==1){
    K[0][0] = -0.1129*pow(x[1],2) - 0.0018*x[1] + 0.985;
    K[0][1] = 0;
    K[0][2] = -0.445*x[1];
    K[0][3] = 1.02;
    K[0][4] = 0;
    K[0][5] = 0.025*x[1];
    K[1][0] = 0;
    K[1][1] = -0.343*pow(x[0],2) - 0.098*x[0] + 0.985;
    K[1][2] = 0.663*x[0];
    K[1][3] = 0;
    K[1][4] = 1.022;
    K[1][5] = 0;
    K[2][0] = 0.445*x[1];
    K[2][1] = -0.668*x[0];
    K[2][2] = (-0.345*pow(x[0],2) - 0.098*x[0] + 0.985)*(-0.118*pow(x[1],2) + 0.002*x[1] + 0.991);
    K[2][3] = 0.0135*x[1];
    K[2][4] = 0;
    K[2][5] = 0.6*pow(x[1],2) - 0.411*x[1] + 1.05;
  }
}