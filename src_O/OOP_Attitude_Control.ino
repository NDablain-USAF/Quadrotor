#include <Wire.h>
#include "Button.h"
#include "Motor.h"
#include "IMU.h"
#include "LQR.h"
#include "Barometer.h"

const uint8_t RESET_PIN = 8;
// Button Setup
const uint8_t BUTTON_POWER_PIN = 13;
const uint8_t BUTTON_ON_PIN = 12;
const uint8_t BUTTON_OFF_PIN = 11;
const uint8_t CONTROL_PINS[4] = {3,5,6,9}; // Should be the same as PWM pin driving the motor
BUTTON button(BUTTON_POWER_PIN, BUTTON_ON_PIN, BUTTON_OFF_PIN, CONTROL_PINS);
// Motor Setup
const uint8_t PWM_PIN[4] = {3,5,6,9}; // Implements motor control
volatile uint16_t count[4]; // Stores encoder counts
volatile uint8_t pulse_check[4]; // Ensures ISR increments correct encoder
int16_t MotorSpeeds[4];
MOTOR motor1(PWM_PIN[0]);
MOTOR motor2(PWM_PIN[1]);
MOTOR motor3(PWM_PIN[2]);
MOTOR motor4(PWM_PIN[3]);
// IMU Setup
float EulerAngles[3];
int16_t w[3];
bool calStatus[2];
int16_t r_attitude[3] = {0,0,0};
IMU IMU;
// Attitude Controller Setup
LQR LQR;
// Barometer Setup
float h[2];
float height_measured;
uint32_t timelast;
uint8_t mode; // Predict (0), or update (1), for height Kalman Filter
float r_altitude = 0.5;
BAROMETER Barometer;

void setup() {
  Initialize();
}

void loop() {
  Calibrate();
  
  if ((calStatus[0]==true)&&(calStatus[1]==true)){
    Run();
  }
  Print();
}

void Initialize(){
  PORTB |= 0b00000001;
  Serial.begin(115200);
  IMU.Initialize();
  Barometer.Initialize();
  PCICR |= B00000010;   // Enable port C (A0-A5)
  PCMSK1 |= B00001111;  // Enable A3-A0
  pinMode(RESET_PIN,OUTPUT);
  IMU.Test(&RESET_PIN);
  Barometer.Test(&RESET_PIN);
}

void Calibrate(){
  //button.receive(EulerAngles);
  IMU.update(&calStatus[0]);
  if ((millis()-timelast)>80){ // Data rate of barometer is 12.5 Hz (80ms)
    Barometer.update(&height_measured,&calStatus[1]);
    timelast = millis();
    mode = 1;
  }
}

void Run(){
  IMU.Kalman_filter_attitude(EulerAngles, w);
  IMU.Kalman_filter_altitude(h,&height_measured,EulerAngles,mode);
  LQR.control_altitude(h, &r_altitude);
  LQR.control_attitude(r_attitude, MotorSpeeds, EulerAngles, w);
  Run_Motor(MotorSpeeds);
}

void Print(){
  static uint8_t time;
  if (time>25){
     for (uint8_t i=0;i<4;i++){
       Serial.print(MotorSpeeds[i]);
       Serial.print(",");
     }
    Serial.println(h[0]);
    time = 0;
  }
  time++;
}

void Run_Motor(int16_t MotorSpeeds[4]){
  motor1.control(MotorSpeeds[0], &count[0]);
  motor2.control(MotorSpeeds[1], &count[1]);
  motor3.control(MotorSpeeds[2], &count[2]);
  motor4.control(MotorSpeeds[3], &count[3]);
}

ISR(PCINT1_vect) {  // PCINT1_vect is for port C
    if ((pulse_check[0]==0)&&((PINC & B00000001)==0)) {
      count[0]++;
      pulse_check[0]++;
    }
    else if ((pulse_check[0]!=0)&&((PINC & B00000001)!=0)) {
      pulse_check[0] = 0;
    }
    if ((pulse_check[1]==0)&&((PINC & B00000010)==0)) {
      count[1]++;
      pulse_check[1]++;
    }
    else if ((pulse_check[1]!=0)&&((PINC & B00000010)!=0)) {
      pulse_check[1] = 0;
    }
    if ((pulse_check[2]==0)&&((PINC & B00000100)==0)) {
      count[2]++;
      pulse_check[2]++;
    }
    else if ((pulse_check[2]!=0)&&((PINC & B00000100)!=0)) {
      pulse_check[2] = 0;
    }
    if ((pulse_check[3]==0)&&((PINC & B00001000)==0)) {
      count[3]++;
      pulse_check[3]++;
    }
    else if ((pulse_check[3]!=0)&&((PINC & B00001000)!=0)) {
      pulse_check[3] = 0;
    }
}