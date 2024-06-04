#include "Barometer.h"
#include <Wire.h>

BAROMETER::BAROMETER(){

}

void BAROMETER::Initialize(){
  Wire.begin();
  uint8_t T1_T2[4];
  int8_t T3_P4[7];
  uint8_t P5_P6[4];
  int8_t P7_P11[6];
  uint8_t i = 0;
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x1B);
  Wire.write(0b00110011); // Set normal mode, enable pressure and temperature measurement
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x1C);
  Wire.write(0b00010101); // Set oversampling rate to 2x for temperature and 32x for pressure
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x1D);
  Wire.write(0x04); // Set output data rate to 12.5 Hz
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x31); // Read factory calibration data
  Wire.endTransmission();
  Wire.requestFrom(Barometer_Address,21);

  while(Wire.available()>0){
    if (i<4){
      T1_T2[i++] = Wire.read();
    }
    else if (i<11){
      T3_P4[(i++)-4] = Wire.read();
    }
    else if (i<15){
      P5_P6[(i++)-11] = Wire.read();
    }
    else if (i<21){
      P7_P11[(i++)-15] = Wire.read();
    }
  }
  uint16_t NVM_PAR_T1 = T1_T2[1]<<8;
  NVM_PAR_T1 += T1_T2[0];
  uint16_t NVM_PAR_T2 = T1_T2[3]<<8;
  NVM_PAR_T2 += T1_T2[2];
  int8_t NVM_PAR_T3 = T3_P4[0];
  int16_t NVM_PAR_P1 = T3_P4[2]<<8;
  NVM_PAR_P1 += T3_P4[1];
  int16_t NVM_PAR_P2 = T3_P4[4]<<8;
  NVM_PAR_P2 += T3_P4[3];
  int8_t NVM_PAR_P3 = T3_P4[5];
  int8_t NVM_PAR_P4 = T3_P4[6];
  uint16_t NVM_PAR_P5 = P5_P6[1]<<8;
  NVM_PAR_P5 += P5_P6[0];
  uint16_t NVM_PAR_P6 = P5_P6[3]<<8;
  NVM_PAR_P6 += P5_P6[2];
  int8_t NVM_PAR_P7 = P7_P11[0];
  int8_t NVM_PAR_P8 = P7_P11[1];
  int16_t NVM_PAR_P9 = P7_P11[3]<<8;
  NVM_PAR_P9 += P7_P11[2];
  int8_t NVM_PAR_P10 = P7_P11[4];
  int8_t NVM_PAR_P11 = P7_P11[5];
  
  PAR_T[0] = NVM_PAR_T1/pow(2,-8);
  PAR_T[1] = NVM_PAR_T2/pow(2,30);
  PAR_T[2] = NVM_PAR_T3/pow(2,48);
  PAR_P[0] = (NVM_PAR_P1-pow(2,14))/pow(2,20);
  PAR_P[1] = (NVM_PAR_P2-pow(2,14))/pow(2,29);
  PAR_P[2] = NVM_PAR_P3/pow(2,32);
  PAR_P[3] = NVM_PAR_P4/pow(2,37);
  PAR_P[4] = NVM_PAR_P5/pow(2,-3);
  PAR_P[5] = NVM_PAR_P6/pow(2,6);
  PAR_P[6] = NVM_PAR_P7/pow(2,8);
  PAR_P[7] = NVM_PAR_P8/pow(2,15);
  PAR_P[8] = NVM_PAR_P9/pow(2,48);
  PAR_P[9] = NVM_PAR_P10/pow(2,48);
  PAR_P[10] = NVM_PAR_P11/pow(2,65);
}

void BAROMETER::update(float *height_measured, bool *calStatus){
  uint8_t i = 0;
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(Barometer_Address,6);
  while(Wire.available()){
    Data[i++] = Wire.read();
  }
  uint32_t Pressure = Data[2];
  Pressure = Pressure<<16;
  uint16_t MRFudge = Data[1];
  MRFudge = MRFudge<<8;
  MRFudge += Data[0];
  Pressure += MRFudge;
  uint32_t Temperature = Data[5];
  Temperature = Temperature<<16;
  uint16_t MRSFudge = Data[4];
  MRSFudge = MRSFudge<<8;
  MRSFudge += Data[3];
  Temperature += MRSFudge;
  // Compensation equations come from pages 56-57 of BST-BMP390L-DS001-02 Datasheet
  float tpartial_data[2];
  float ppartial_data[4];
  float ppartial_out[2];
  tpartial_data[0] = (float)(Temperature-PAR_T[0]);
  tpartial_data[1] = (float)(tpartial_data[0]*PAR_T[1]);
  float Temperature_Comp = tpartial_data[1]+pow(tpartial_data[0],2)*PAR_T[2];
  float temp1 = pow(Temperature_Comp,2);
  float temp2 = pow(Temperature_Comp,3);
  ppartial_data[0] = PAR_P[5]*Temperature_Comp;
  ppartial_data[1] = PAR_P[6]*temp1;
  ppartial_data[2] = PAR_P[7]*temp2;
  ppartial_out[0] = PAR_P[4]+ppartial_data[0]+ppartial_data[1]+ppartial_data[2];
  ppartial_data[0] = PAR_P[1]*Temperature_Comp;
  ppartial_data[1] = PAR_P[2]*temp1;
  ppartial_data[2] = PAR_P[3]*temp2;
  ppartial_out[1] = (float)Pressure*(PAR_P[0]+ppartial_data[0]+ppartial_data[1]+ppartial_data[2]);
  ppartial_data[0] = pow((float)Pressure,2);
  ppartial_data[1] = PAR_P[8] + PAR_P[9]*Temperature_Comp;
  ppartial_data[2] = ppartial_data[0]*ppartial_data[1];
  ppartial_data[3] = ppartial_data[2] + pow((float)Pressure,3)*PAR_P[10];
  Pressure_Comp = Pressure_Comp*c1 + (ppartial_out[0]+ppartial_out[1]+ppartial_data[3])*c2;
  if (calWait<200){
    HeightBias = (Tb/Lb)*(pow(Pressure_Comp/Pb,(-R*Lb)/(g*M))-1);
    ++calWait;
    *calStatus = false;
  }
  else if (calWait==200){
    *calStatus = true;
    *height_measured = (Tb/Lb)*(pow(Pressure_Comp/Pb,(-R*Lb)/(g*M))-1) - HeightBias;
  }
}

void BAROMETER::Test(uint8_t *RESET_PIN){
  uint8_t i = 0;
  uint8_t check = 0;
  Wire.beginTransmission(Barometer_Address);
  Wire.write(0x04);
  Wire.endTransmission();
  Wire.requestFrom(Barometer_Address,6);
  while(Wire.available()){
    Data[i++] = Wire.read();
  }
  for (uint8_t j=0;j<6;j++){
    if (Data[j]==j+1){
      check++;
    }
  }
  if (check==6){
    digitalWrite(*RESET_PIN,LOW);
  }
}