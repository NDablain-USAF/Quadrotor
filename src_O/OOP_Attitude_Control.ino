#include "Button.h"
#include "Motor.h"
#include "IMU.h"
#include "LQR.h"
#include <Wire.h>

// Button Setup
const uint8_t BUTTON_POWER_PIN = 13;
const uint8_t BUTTON_ON_PIN = 12;
const uint8_t BUTTON_OFF_PIN = 11;
const uint8_t CONTROL_PINS[4] = {3,5,6,9}; // Should be the same as PWM pin driving the motor
BUTTON button(BUTTON_POWER_PIN, BUTTON_ON_PIN, BUTTON_OFF_PIN, CONTROL_PINS);
// Motor Setup
const uint8_t ENCODER_PIN[4] = {A1,A2,A3,A4}; // Encoder pins must be interrupt pins
const uint8_t PWM_PIN[4] = {3,5,6,9};
volatile uint16_t count[4];
volatile uint8_t pulse_check[4];
MOTOR motor1(PWM_PIN[0]);
MOTOR motor2(PWM_PIN[1]);
MOTOR motor3(PWM_PIN[2]);
MOTOR motor4(PWM_PIN[3]);
// IMU Setup
IMU IMU;
// Attitude Controller Setup
LQR LQR(1);

void setup() {  
  IMU.configIMU();
  PCICR |= B00000010;   // Enable port C (A0-A5)
  PCMSK1 |= B00011110;  // Enable A4-A1
}

void loop() {
  button.receive();

  static double EulerAngles[3];
  static double w[3];
  static double MotorSpeeds[4];
  static bool calStatus;
  double ReferenceAngles[6] = {0,0,0,0,0,0};
  IMU.update(&calStatus);
  if (calStatus==true){
    IMU.Kalman_filter(EulerAngles,w);
    double X_attitude[6] = {EulerAngles[0],EulerAngles[1],EulerAngles[2],w[0],w[1],w[2]};
    LQR.Control(ReferenceAngles, MotorSpeeds, X_attitude);
  }

  Run_Motor(MotorSpeeds);
}

void Run_Motor(double MotorSpeeds[4]){
  motor1.control(MotorSpeeds[0], &count[0]);
  motor2.control(MotorSpeeds[1], &count[1]);
  motor3.control(MotorSpeeds[2], &count[2]);
  motor4.control(MotorSpeeds[3], &count[3]);
}

ISR(PCINT1_vect) {  // PCINT1_vect is for port C
  for (uint8_t i=0;i<4;i++){
    if ((digitalRead(ENCODER_PIN[i]) == HIGH)&&(pulse_check[i]==0)) {
      count[i]++;
      pulse_check[i] = 1;
    }
    else if ((digitalRead(ENCODER_PIN[i]) == LOW)&&(pulse_check[i]==1)){
      count[i]++;
      pulse_check[i] = 0;
    }
  }
}