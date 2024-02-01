#include "Button.h"

BUTTON::BUTTON(uint8_t POWER_PIN, uint8_t ON_PIN, uint8_t OFF_PIN, uint8_t CONTROL_PIN){
    power_pin = POWER_PIN;
    on_pin = ON_PIN;
    off_pin = OFF_PIN;
    control_pin = CONTROL_PIN;
    init();
}

void BUTTON::init(){
  pinMode(power_pin,OUTPUT);
  pinMode(on_pin,INPUT_PULLUP);
  pinMode(off_pin,INPUT_PULLUP);
  pinMode(control_pin,OUTPUT);
  uint8_t i = 0;
}

void BUTTON::receive(){
  while (i < 1){
    delay(1);
    if (digitalRead(on_pin)==0){
      ++i;
    }
  }
  if (digitalRead(off_pin)==0){
    analogWrite(control_pin,0);
    i = 0;
  }
}
