#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class BUTTON{
  private:
    uint8_t 
      power_pin,
      on_pin,
      off_pin,
      control_pins[4],
      i;
  public:
    BUTTON(const uint8_t POWER_PIN, const uint8_t ON_PIN, const uint8_t OFF_PIN, const uint8_t CONTROL_PINS[4]);
    void receive(float EulerAngles[3]);
    void init();
};

#endif