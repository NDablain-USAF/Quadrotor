#include "Button.h"

BUTTON::BUTTON(const uint8_t POWER_PIN, const uint8_t ON_PIN, const uint8_t OFF_PIN, const uint8_t CONTROL_PINS[4]){
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
  i = 0; // This will be used to toggle the motors on/off, 0:off, 0>:on
}

void BUTTON::receive(float EulerAngles[3]){
  while (i < 1){ // This will keep the motors off until the on_pin is sent low via a mechanical button press
    delay(1);
    for (uint8_t j=0;j<4;j++){
      analogWrite(control_pins[j],0);
    }
    if (digitalRead(on_pin)==0){
      ++i;
    }
  }
  // The motors are disabled under the following conditions...
  //        1: The off_pin is sent low via a mechanical button press
  //        2: The roll or pitches angles exceed +-pi/12 radians (15 degrees)
  //        3: The yaw angle exceeds +-pi/2 radians (90 degrees)
  //        4: 30 seconds have passed since SYSTEM START (Not when the mechanical on_pin is pressed)
  if ((digitalRead(off_pin)==0)||(abs(EulerAngles[0])>(PI/12))||(abs(EulerAngles[1])>(PI/12))||(abs(EulerAngles[2])>PI/2)){
    i = 0;
  }
  if (micros()>30e6){
    i = 0;
  }
}
