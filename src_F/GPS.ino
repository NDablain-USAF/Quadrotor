const uint8_t FIX_PIN = 3;
const uint8_t SYNC_PIN = 7;

byte setBaud9600[15] = {36,80,77,84,75,50,53,49,44,48,42,50,56,13,10}; // 9600 baud rate, default on cold start
////////////////////////$  P  M  T  K  2  5  1  ,  0  *  2  8  CR LF
byte setBaud38400[19] = {36,80,77,84,75,50,53,49,44,51,56,52,48,48,42,50,55,13,10}; //38400 baud rate
/////////////////////////$  P  M  T  K  2  5  1  ,  3  8  4  0  0  *  2  7  CR LF
byte setBaud115200[20] = {36,80,77,84,75,50,53,49,44,49,49,53,50,48,48,42,49,70,13,10}; //115200 baud rate
//////////////////////////$  P  M  T  K  2  5  1  ,  1  1  5  2  0  0  *  1  F  CR LF
byte setUpdate10[25] = {36,80,77,84,75,51,48,48,44,49,48,48,44,48,44,48,44,48,44,48,42,50,67,13,10}; // 10Hz data rate
////////////////////////$  P  M  T  K  5  0  0  ,  1  0  0  ,  0  ,  0  ,  0  ,  0  *  2  C  CR LF
byte queryUpdate[13] = {36,80,77,84,75,52,48,48,42,51,54,13,10}; // Ask for update rate
////////////////////////$  P  M  T  K  5  0  0  *  3  6  CR LF
byte setOutput[51] = {36,80,77,84,75,51,49,52,44,48,44,48,44,48,44,49,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,42,50,57
,13,10}; // Only enable GGA


void setup() {
  Serial.begin(9600);
  /* Run this section on cold start
  Serial1.begin(9600); 
  Serial1.write(setBaud115200,20);
  delay(1000);
  Serial1.end();
  delay(1000);
  */
  Serial1.begin(115200);
  delay(1000);
  Serial1.write(setUpdate10,25);
  delay(1000);
  Serial1.write(setOutput,51);
  delay(1000);

  pinMode(FIX_PIN, INPUT);
  pinMode(SYNC_PIN, INPUT);
}

void loop() {
  static bool Integrity;
  static uint8_t latitudeArray[9],longitudeArray[10],altitudeArray[5],EWIndicator,NSIndicator;

  bool FIX = fix();
  if (FIX==true){
    Integrity = readGPS(latitudeArray,longitudeArray,altitudeArray,&NSIndicator,&EWIndicator);
    if (Integrity>0){
      uint8_t latitudeSize = sizeof(latitudeArray)/sizeof(latitudeArray[0]); // Convert arrays to doubles for calculations
      double latitude = convertGeo(latitudeArray,latitudeSize,NSIndicator);
      uint8_t longitudeSize = sizeof(longitudeArray)/sizeof(longitudeArray[0]);
      double longitude = convertGeo(longitudeArray,longitudeSize,EWIndicator);
      uint8_t altitudeSize = sizeof(altitudeArray)/sizeof(altitudeArray[0]);
      double altitude = convertAlt(altitudeArray,altitudeSize);
      displacements(latitude,longitude,altitude);
    }
  }
  else {
    Serial.println("Fixing");
  }
}

void displacements(double latitude, double longitude, double altitude){
  static uint32_t biasControl;
  static uint32_t timelast;
  static double xE, xN, h, Latitudelast, Longitudelast, Altitudelast, xNlast, xElast, hlast;
  double c[4] = {0.9,0.995,0.10,0.005}; // Filter constants
  double r1 = 6378137; // in meters
  double r2 = 6356752;
  double cutoff = (0.01)*(PI/180);
  double Latitude = latitude*c[2] + Latitudelast*c[0];
  double Longitude = longitude*c[3] + Longitudelast*c[1];
  double Altitude = altitude*c[2] + Altitudelast*c[0];

  if (biasControl==50000){
    ++biasControl;
  }
  else if (biasControl<50000){
    ++biasControl;
    if ((biasControl==20000)||(biasControl==35000)){
      Serial.println("Calibrating");
    }
  }
  else {
    if ((abs(Latitude-Latitudelast)>cutoff)||(Latitude==0)||(Latitude==(PI/2))){
      Latitude = Latitudelast*c[1];
    }
    if (abs(Longitude-Longitudelast)>cutoff){
      Longitude = Longitudelast*c[1];
    }
    double radiusEarth = sqrt((pow(pow(r1,2)*cos(Latitude),2)+pow(pow(r2,2)*sin(Latitude),2))/(pow(r1*cos(Latitude),2)+pow(r2*sin(Latitude),2)));
    double R = radiusEarth+Altitude;
    xE += (R*(Longitude-Longitudelast)*sin(Latitude));
    xN += (R*(Latitude-Latitudelast));
    h += (Altitude-Altitudelast);

    if (isnan(xE)==1){
      xE = xElast;
    }
    if (isnan(xN)==1){
      xN = xNlast;
    }
    if (isnan(h)==1){
      h = hlast;
    }
    
    Serial.print("Displacment North: ");
    Serial.print(xN);
    Serial.print(" , ");
    Serial.print("Displacement East: ");
    Serial.print(xE);
    Serial.print(" , ");
    Serial.print("Height: ");
    Serial.println(h);
  }
  xNlast = xN;
  xElast = xE;
  hlast = h;
  Latitudelast = Latitude;
  Longitudelast = Longitude;
  Altitudelast = Altitude;
}

double convertGeo(uint8_t coordinate[],uint8_t coordinateSize,uint8_t indicator){ // Converts array in degrees and months to single value in radians
  double minutes,degrees;
  uint8_t digitCount;
  for (uint8_t i=0;i<coordinateSize;i++){ // Find the number of digits to the right of the decimal
    if (coordinate[i]==46){
      digitCount = i;
    }
  }
  for (uint8_t i=0;i<coordinateSize;i++){
    if (i<digitCount){ // For values to the right of the decimal, input follows ddmm.mmmm or dddmm.mmmm
      minutes+=((coordinate[i]-48)*pow(10,i-digitCount));
    }
    else if ((i>digitCount)&&(i<digitCount+3)){ 
      minutes+=((coordinate[i]-48)*pow(10,i-(digitCount+1)));
    }
    else if ((i>digitCount)&&(i<coordinateSize)){ 
      degrees+=((coordinate[i]-48)*pow(10,i-(digitCount+3)));
    }
  }
  int8_t sign = -1;
  if ((indicator==78)||(indicator==69)){ // Positive latitude & longitude measure N & E
    sign = 1;
  }
  double coordinateConverted = (degrees+(minutes/60))*(PI/180)*sign;
  return(coordinateConverted);
}

double convertAlt(uint8_t coordinate[],uint8_t coordinateSize){
  double coordinateConverted;
  uint8_t digitCount;
  for (uint8_t i=0;i<coordinateSize;i++){ // Find the number of digits to the right of the decimal
    if (coordinate[i]==46){
      digitCount = i;
    }
  }
  for (uint8_t i=0;i<coordinateSize;i++){
    if (i<digitCount){
      coordinateConverted+=((coordinate[i]-48)*pow(10,i-digitCount));
    }
    else if (i>digitCount){
      coordinateConverted+=((coordinate[i]-48)*pow(10,i-(digitCount+1)));
    }
  }
  return(coordinateConverted);
}

bool readGPS(uint8_t latitude[9],uint8_t longitude[10],uint8_t altitude[5],uint8_t* NSIndicator1,uint8_t* EWIndicator1){
  static uint8_t data[1000],messageID[5],UTCTime1[10],latitude1[9],longitude1[10],horizontalDilutionOfPrecision[4];
  static uint8_t result,checksum1,fixIndicator,satellitesUsed[2],unitsMSL,geoSeparation[4],unitsGeo;
  
  static uint16_t index[4];
  static bool messageIntegrity;
  
  if (Serial1.available()>0){
    uint8_t inbound = Serial1.read();
    if (inbound==36){ // Reset on the $
      for(uint8_t i=0;i<4;i++){
        index[i]=0;
      }
    }
    else if ((inbound==42)||((index[2]>0)&&(index[2]<3))){ // Take * and next two values
      ++index[2];
      index[1] = 1;
      if (index[2]==2){
        checksum1 = inbound;
      }
      if (index[2]==3){
        uint8_t checksum2 = inbound;
        uint8_t mask = 0b00001111;
        uint8_t result1 = result>>4; // First value in binary
        if (result1<10){ // If the value in HEX is a numeral (less than 10 in dec)
          result1+=48; // Add 48 to get its HEX representation
        }
        else{ // If the value in HEX is a character
          result1+=55; // Add 55 to gets its HEX representation
        }
        uint8_t result2 = result&mask; // Second value in binary
        if (result2<10){ // If the value in binary is a numeral (less than 10)
          result2+=48; // Add 48 to get its Dec representation
        }
        else{ // If the value in binary is a character
          result2+=55; // Corresponding value in Dec
        }
        if ((result1==checksum1)&&(result2==checksum2)){
          messageIntegrity = true;
        }
        else{
          messageIntegrity = false;
        }        
      }
    }
    else if (index[1]!=1){
      data[index[0]] = inbound;
      uint16_t sum;
      for (uint8_t i=0;i<5;i++){ // Sum Dec equivalents of binary letters
        sum+=messageID[i];
      }
      if (data[index[0]]==44){ // If reading a comma, move to next entry
        index[3]++;
      }

      else if (index[3]==0){ // If on message ID
        messageID[abs(index[0]-4)] = data[index[0]];
      }

      else if (index[3]==1){ 
        if (sum==358){ // GPGGA & GPRMC
          UTCTime1[abs(index[0]-15)] = data[index[0]];
        }
      }

      else if (index[3]==2){
        if (sum==358){
          latitude[abs(index[0]-25)] = data[index[0]];
        }
      }

      else if (index[3]==3){
        if (sum==358){
          *NSIndicator1 = data[index[0]];
        }
      }

      else if (index[3]==4){
        if (sum==358){
          longitude[abs(index[0]-38)] = data[index[0]];
        }
      }

      else if (index[3]==5){
        if (sum==358){
          *EWIndicator1 = data[index[0]];
        }
      }

      else if (index[3]==6){
        if (sum==358){
          fixIndicator = data[index[0]];
        }
      }

      else if (index[3]==7){
        if (sum==358){
          satellitesUsed[abs(index[0]-45)] = data[index[0]];
        }
      }      

      else if (index[3]==8){
        if (sum==358){
          horizontalDilutionOfPrecision[abs(index[0]-50)] = data[index[0]];
        }
      }    

      else if (index[3]==9){
        if (sum==358){
          altitude[abs(index[0]-56)] = data[index[0]];
        }
      }    

      else if (index[3]==10){
        if (sum==358){
          unitsMSL = data[index[0]];
        }
      }   

      else if (index[3]==11){
        if (sum==358){
          geoSeparation[abs(index[0]-61)] = data[index[0]];
        }
      }  

      else if (index[3]==12){
        if (sum==358){
          unitsGeo = data[index[0]];
        }
      } 

      if (index[0]==1){ // For first two values, establish result variable
        result = data[index[0]]^data[index[0]-1]; // Bitwise xor, order of operation doesn't matter
      }
      else if (index[0]>1){ // Once result variable is established, use it
        result = result^data[index[0]];
      }
      ++index[0];
    }  
  }
  return(messageIntegrity);
}

bool fix(){
  static uint8_t index;
  static uint32_t period,time;
  bool fixFound;
  if ((digitalRead(FIX_PIN)==1)&&(index==0)){ // Rising edge
    index = 1;
  }
  else if ((digitalRead(FIX_PIN)==0)&&(index==1)){ // Falling edge
    index = 0;
    period = (millis()-time)/2; // Record the period
    time = millis();
  }

  if (period<1500){ // A period of 1s means a fix hasn't been found
    fixFound = false;
  }
  else { // A longer duration period means a fix has been found, may be around 15s for 2D and 30s for 3D but need to confirm
    fixFound = true;
  }
  return(fixFound);
}
