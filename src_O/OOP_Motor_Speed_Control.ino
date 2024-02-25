#include "Button.h"
#include "Motor.h"
// Button Setup
const uint8_t BUTTON_POWER_PIN = 13;
const uint8_t BUTTON_ON_PIN = 12;
const uint8_t BUTTON_OFF_PIN = 11;
const uint8_t CONTROL_PIN = 5; // Should be the same as PWM pin driving the motor
BUTTON button(BUTTON_POWER_PIN, BUTTON_ON_PIN, BUTTON_OFF_PIN, CONTROL_PIN);
// Motor Setup
const uint8_t ENCODER1_PIN = 3; // Encoder pins must be interrupt pins
const uint8_t ENCODER2_PIN = 2;
const uint8_t PWM_PIN = 5;
// Initialize the counter 1-8 variables in the class
volatile uint8_t MOTOR::counter1;
volatile uint8_t MOTOR::counter2;
volatile uint8_t MOTOR::counter3;
volatile uint8_t MOTOR::counter4;
volatile uint8_t MOTOR::counter5;
volatile uint8_t MOTOR::counter6;
volatile uint8_t MOTOR::counter7;
volatile uint8_t MOTOR::counter8;
// Initialize the ISR 1-8 functions within the class
void MOTOR::ISR1(void) {++(MOTOR::counter1);}
void MOTOR::ISR2(void) {++(MOTOR::counter1);}
void MOTOR::ISR3(void) {++(MOTOR::counter3);}
void MOTOR::ISR4(void) {++(MOTOR::counter4);}
void MOTOR::ISR5(void) {++(MOTOR::counter5);}
void MOTOR::ISR6(void) {++(MOTOR::counter6);}
void MOTOR::ISR7(void) {++(MOTOR::counter7);}
void MOTOR::ISR8(void) {++(MOTOR::counter8);}

MOTOR motor(ENCODER1_PIN, ENCODER2_PIN, PWM_PIN, 1);

void setup() {  
  Serial.begin(115200);
}

void loop() {
  button.receive();

  motor.performance();
  motor.control();

}
