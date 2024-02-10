# include "GPS.h"
GPS gps1;

void setup() {
  Serial.begin(250000);
  gps1.init();

}

void loop() {
  double Xn;
  double Xe;
  double h;
  gps1.updategps(&Xn,&Xe,&h);
  
}
