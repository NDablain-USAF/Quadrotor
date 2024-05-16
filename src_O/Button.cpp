#include "Button.h"

BUTTON::BUTTON(uint8_t POWER_PIN, uint8_t ON_PIN, uint8_t OFF_PIN, uint8_t CONTROL_PINS[4]){
    power_pin = POWER_PIN;
    on_pin = ON_PIN;
    off_pin = OFF_PIN;
    for (uint8_t j=0;j<4;j++){
      control_pins[j] = CONTROL_PINS[j];
    }
    init();
}

void BUTTON::init(){
  pinMode(power_pin,OUTPUT);
  pinMode(on_pin,INPUT_PULLUP);
  pinMode(off_pin,INPUT_PULLUP);
  for (uint8_t j=0;j<4;j++){
    pinMode(control_pins[j],OUTPUT);
  }
  uint8_t i = 0;
}

void BUTTON::receive(){
  while (i < 1){
    delay(1);
    for (uint8_t j=0;j<4;j++){
      analogWrite(control_pins[j],0);
    }
    if (digitalRead(on_pin)==0){
      ++i;
    }
  }
  if (digitalRead(off_pin)==0){
    i = 0;
  }
}