#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

class BUTTON{
  private:
    uint8_t power_pin;
    uint8_t on_pin;
    uint8_t off_pin;
    uint8_t control_pins[4];
    uint8_t i;
  public:
    BUTTON(uint8_t POWER_PIN, uint8_t ON_PIN, uint8_t OFF_PIN, uint8_t CONTROL_PINS[4]);
    void receive();
    void init();
};

#endif