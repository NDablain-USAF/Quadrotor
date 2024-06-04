#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>

class BAROMETER{
    private:
      float
        PAR_T[3],
        PAR_P[11],
        Tb = 288.15,
        Lb = -0.0065,
        Pb = 101325,
        R = 8.31432,
        g = 9.80665,
        M = 0.0289644,
        c1 = 0.9,
        c2 = 1-c1,
        Pressure_Comp,
        HeightBias;
      int8_t
        Barometer_Address = 0x77;
      uint8_t
        Data[6] = {1,2,3,4,5,6},
        calWait;
    public:
    BAROMETER();
      void
        Initialize();
      void
        Test(uint8_t *RESET_PIN),
        update(float *height_measured, bool *calStatus);
};

#endif