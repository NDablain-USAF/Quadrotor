#include <Wire.h>
 // Accel & Gyro are at address 0x68, Magnetometer is at address 0x0C
#define IMUAddress 0x68 // I2C address of 9250
#define MAGAddress 0x0C // AK8963 I2C slave address
float magXcoef, magYcoef, magZcoef, psiOffset;

void setup() {
  Serial.begin(115200);
  configIMU();
}

void loop() {
  static int32_t accelGrav[3]; // in mm/s^2
  static double angularRates[3];
  static double magField[3];
  static double EulerAngles[3];
  static uint8_t count;
  int8_t DataAG[14];
  uint8_t DataM[7];

  static uint8_t k;
  if (k>10){
    read(DataAG,DataM);
    k = 0;
  }
  k++;
  bool calStatus = update(accelGrav,angularRates,magField,DataAG,DataM);

  if (calStatus==true){
    Euler(accelGrav,angularRates,magField,EulerAngles);
    count++;
    if (count>25){
      for (uint8_t i=0;i<3;i++){
        Serial.print(EulerAngles[i]);
        Serial.print(",");
      }
      Serial.println();
      count = 0;
    }
  }
  
}

void read(int8_t DataAG[14], uint8_t DataM[7]){
  uint8_t i = 0;
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x3B); // Accel x Higher Bit Address
  Wire.endTransmission();
  Wire.requestFrom(IMUAddress,14);
  while(Wire.available()){
    DataAG[i++] = Wire.read();
  }
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x03);
  Wire.endTransmission();
  i = 0;
  Wire.requestFrom(MAGAddress, 7);
  while (Wire.available()){
    DataM[i++] = Wire.read();
  }
}

bool update(int32_t accelGrav[3], double angularRates[3], double magField[3], int8_t DataAG[14], uint8_t DataM[7]){
  static double gyroBias[3];
  static double accelBias[3];
  static uint16_t calWait;
  static bool calStatus;
  // Filter constants, 0-1, increase c1 to lower cutoff frequency
  double c1AccelGrav = 0.998; // Use for gravity vector from accelerometers 0.998
  double c2AccelGrav = 1-c1AccelGrav;
  double c1Gyro = 0.9; // Use for rate gyro
  double c2Gyro = 1-c1Gyro;
  double c1Mag = 0.99;
  double c2Mag = 1-c1Mag;
  // Random constants
  double b2mg = 1.1975; // Converts bits to milli g's for acceleration, assume 8 g range and 16 bit precision
  uint8_t gyroRange = 250;
  //
  int16_t x = DataAG[0]<<8;
  x += DataAG[1];
  int32_t X = (x*b2mg)-accelBias[0];
  accelGrav[0] = accelGrav[0]*c1AccelGrav + X*c2AccelGrav;
  int16_t y = DataAG[2]<<8;
  y += DataAG[3];
  int32_t Y = (y*b2mg)-accelBias[1];
  accelGrav[1] = accelGrav[1]*c1AccelGrav + Y*c2AccelGrav;
  int16_t z = DataAG[4]<<8;
  z += DataAG[5];
  int32_t Z = (z*b2mg)-accelBias[2];  
  accelGrav[2] = accelGrav[2]*c1AccelGrav + Z*c2AccelGrav;
  double q = DataAG[8]<<8;
  q += (DataAG[9]/131);
  q = (q/gyroRange)-gyroBias[1];
  double p = DataAG[10]<<8;
  p += (DataAG[11]/131);
  p = (p/gyroRange)-gyroBias[0];
  double r = DataAG[12]<<8;
  r += (DataAG[13]/131);
  r = (r/gyroRange)-gyroBias[2];
  angularRates[0] = (angularRates[0]*c1Gyro) + (p*c2Gyro);
  angularRates[1] = (angularRates[1]*c1Gyro) + (q*c2Gyro);
  angularRates[2] = (angularRates[2]*c1Gyro) + (r*c2Gyro);
  float mres = 4912.0/32760.0;
  int16_t mx = DataM[1]<<8;
  mx += DataM[0];
  int16_t my = DataM[3]<<8;
  my += DataM[2];
  int16_t mz = DataM[5]<<8;
  mz += DataM[4];
  magField[0] = (magField[0]*c1Mag) + ((float)mx*mres*magXcoef*c2Mag);
  magField[1] = (magField[1]*c1Mag) + ((float)my*mres*magYcoef*c2Mag);
  magField[2] = (magField[2]*c1Mag) + ((float)mz*mres*magZcoef*c2Mag);
  if (calWait==5000){
    for (uint8_t i=0;i<3;i++){
      gyroBias[i] = angularRates[i];
      if (i<2){
        accelBias[i] = accelGrav[i];
      }
      else {
        accelBias[i] = accelGrav[i]-9810;
      }
    }
    ++calWait;
    calStatus = true;
  }
  else if (calWait<5000){
    ++calWait;
    calStatus = false;
  }
  return calStatus;
}

void Euler(int32_t accelGrav[3], double angularrates[3], double magField[3], double EulerAngles[3]){
  static uint32_t time;
  static double phi,theta,psi;
  double c1 = 0.95;
  double c2 = 1-c1;

  double p = angularrates[0]*(PI/180); // Convert deg/s to rad/s
  double q = angularrates[1]*(PI/180);
  double r = angularrates[2]*(PI/180);
  double ax = accelGrav[0]/1e3; // Convert mm/s^2 to m/s^2
  double ay = accelGrav[1]/1e3;
  double az = accelGrav[2]/1e3;
  // DCM from body to fixed frame for mag readings
  double magFixedX = (cos(theta)*cos(psi)*magField[0]) + (((sin(phi)*sin(theta)*cos(psi))-(cos(phi)*sin(psi)))*magField[1]) + (((cos(phi)*sin(theta)*cos(psi))+(sin(phi)*sin(psi)))*magField[2]);
  double magFixedY = (cos(theta)*sin(psi)*magField[0]) + (((sin(phi)*sin(theta)*sin(psi))+(cos(phi)*cos(psi)))*magField[1]) + (((cos(phi)*sin(theta)*sin(psi))-(sin(phi)*cos(psi)))*magField[2]);
  //
  // Skew Symmetric matrix for gyro readings
  double phiGyroDot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
  double thetaGyroDot = cos(phi)*q -sin(phi)*r;
  double psiGyroDot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;
  //
  double thetaAccel = atan2(ax,sqrt(pow(ay,2) + pow(az,2))); 
  double phiAccel = atan2(ay,az); 
  double psiMag = atan2(-magField[1],magField[0]);// - psiOffset;
  uint32_t dt = micros()-time;
  if (dt>250000){
    dt = 250000;
  }
  double phiGyro = phi + (phiGyroDot*dt)/1e6;
  double thetaGyro = theta + (thetaGyroDot*dt)/1e6;
  double psiGyro = psi + (psiGyroDot*dt)/1e6;
  time = micros();

  phi = phiGyro*c1 + phiAccel*c2;
  theta = thetaGyro*c1 + thetaAccel*c2;
  psi = psiMag;//psiGyro*c1 + psiMag*c2;

  EulerAngles[0] = phi*(180/PI);
  EulerAngles[1] = theta*(180/PI);
  EulerAngles[2] = psi*(180/PI);
}

void configIMU() {
  uint8_t data[3];
  Wire.begin();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x37);
  Wire.write(0x02);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x00); // Reset
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x0F); // Set FuseROM mode
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x10);
  Wire.endTransmission();
  uint8_t i = 0;
  Wire.requestFrom(MAGAddress, 3);
  while( Wire.available() ) {
    data[i++] = Wire.read();
  }
  magXcoef = (float)(data[0] - 128) / 256.0 + 1.0;
  magYcoef = (float)(data[1] - 128) / 256.0 + 1.0;
  magZcoef = (float)(data[2] - 128) / 256.0 + 1.0;
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0x00); // Reset
  Wire.endTransmission();
  delay(1);
  Wire.beginTransmission(MAGAddress);
  Wire.write(0x0A);
  Wire.write(0b00010010); // Set 16 bit readings and continuous mode
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1C); // Accel config
  Wire.write(0b00001000); // Set range to +- 4g
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(IMUAddress);
  Wire.write(0x1B); // Gyro config
  Wire.write(0b00000000); // Set range to 250 dps
  Wire.endTransmission();
  delay(10);
}