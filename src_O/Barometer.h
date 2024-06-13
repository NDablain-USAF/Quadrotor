#ifndef BAROMETER_H
#define BAROMETER_H

#include <Arduino.h>
#include <Wire.h>

class BAROMETER{
    private:
      float
        PAR_T[3],
        PAR_P[11],
        Tb = 288.15, // Standard temperature at sea level (K)
        Lb = -0.0065, // Standard temperature lapse rate (K/m)
        Pb = 101325, // Standard static pressure at sea level (Pa)
        R = 8.31432, // Universal gas constant (N-m)/(mol-K)
        g = 9.80665, // Acceleration due to gravity (m/s^2)
        M = 0.0289644, // Molar mass of air (kg/mol)
        c1 = 0.9, // IIR pressure filter constant
        c2 = 1-c1,
        Pressure_Comp,
        HeightBias;
      int8_t
        Barometer_Address = 0x77; // Default BMP390 I2C address
      uint8_t
        Data[6] = {1,2,3,4,5,6}, // Use this to test BMP390 is transmitting properly
        calWait,
        calWaitTimeout = 200; // Iterations before sensor exits calibration and starts transmitting height
    public:
    BAROMETER();
      void
        Initialize();
      void
        Test(uint8_t *RESET_PIN),
        update(float *height_measured, bool *calStatus);
};

#endif
