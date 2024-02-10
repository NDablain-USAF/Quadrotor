struct GPSinfo {
  String MESSAGE_ID;
  double UTC_TIME;
  double LATITUDE;
  char NS_INDICATOR;
  double LONGITUDE;
  char EW_INDICATOR;
  uint8_t FIX_INDICATOR;
  uint8_t SATELLITES_USED;
  double HORIZONTAL_DILUTION_OF_PRECISION;
  double ALTITUDE;
  char UNITS_MSL;
  double GEO_SEPARATION;
  char UNITS_GEO;
};

struct Inertialinfo {
  double DISPLACEMENT_NORTH;
  double DISPLACEMENT_EAST;
  double HEIGHT;
};

uint8_t setBaud9600[15] = {36,80,77,84,75,50,53,49,44,48,42,50,56,13,10}; // 9600 baud rate, default on cold start
////////////////////////$  P  M  T  K  2  5  1  ,  0  *  2  8  CR LF
uint8_t setBaud38400[19] = {36,80,77,84,75,50,53,49,44,51,56,52,48,48,42,50,55,13,10}; //38400 baud rate
/////////////////////////$  P  M  T  K  2  5  1  ,  3  8  4  0  0  *  2  7  CR LF
uint8_t setBaud115200[20] = {36,80,77,84,75,50,53,49,44,49,49,53,50,48,48,42,49,70,13,10}; //115200 baud rate
//////////////////////////$  P  M  T  K  2  5  1  ,  1  1  5  2  0  0  *  1  F  CR LF
uint8_t setUpdate10[25] = {36,80,77,84,75,51,48,48,44,49,48,48,44,48,44,48,44,48,44,48,42,50,67,13,10}; // 10Hz data rate
////////////////////////$  P  M  T  K  5  0  0  ,  1  0  0  ,  0  ,  0  ,  0  ,  0  *  2  C  CR LF
uint8_t queryUpdate[13] = {36,80,77,84,75,52,48,48,42,51,54,13,10}; // Ask for update rate
////////////////////////$  P  M  T  K  5  0  0  *  3  6  CR LF
uint8_t setOutput[51] = {36,80,77,84,75,51,49,52,44,48,44,48,44,48,44,49,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,44,48,42,50,57
,13,10}; // Only enable GGA


void setup() {
  Serial.begin(250000);
  ///Run this section on cold start
  Serial1.begin(9600); 
  Serial1.write(setBaud115200,20);
  delay(1000);
  Serial1.end();
  delay(1000);
  ////////////////////////////////
  Serial1.begin(115200);
  delay(1000);
  Serial1.write(setUpdate10,25);
  delay(1000);
  Serial1.write(setOutput,51);
  delay(1000);
}

void loop() {
  static GPSinfo GPSreading;
  static Inertialinfo InertialPositions;

  bool Integrity = readGPS(&GPSreading);

  uint8_t FIX = GPSreading.FIX_INDICATOR;
  
  if (FIX==2){
    if (Integrity==true){
      static uint32_t printTime;
      double latitude = GPSreading.LATITUDE;
      double longitude = GPSreading.LONGITUDE;
      double altitude = GPSreading.ALTITUDE;
      uint8_t positionStatus = displacements(latitude,longitude,altitude,&InertialPositions);
      
      if (positionStatus==1){
        Serial.println("Calibrating");
      }
      
      if (printTime==50000){ // Prevent sending too much info to Serial monitor w/o using delay()
        Serial.print("Displacement North: ");
        Serial.print(InertialPositions.DISPLACEMENT_NORTH);
        Serial.print(" , ");
        Serial.print("Displacement East: ");
        Serial.print(InertialPositions.DISPLACEMENT_EAST);
        Serial.print(" , ");
        Serial.print("Height: ");
        Serial.println(InertialPositions.HEIGHT);
        printTime=0;
      }
      printTime++;
    }
  }
  else {
    Serial.println("Fixing");
  }
  
}

double displacements(double latitude, double longitude, double altitude, Inertialinfo *Inertial){
  static uint16_t biasControl;
  static double xE, xN, h, latitudeLast, longitudeLast, altitudeLast;
  uint8_t status;
  double r1 = 6378137; // in meters
  double r2 = 6356752;
  if (biasControl<=1000){
    biasControl++;
    status = 1; // Calibrating
  }
  else {
    status = 2; // Running
    double radiusEarth = sqrt((pow(pow(r1,2)*cos(latitude),2)+pow(pow(r2,2)*sin(latitude),2))/(pow(r1*cos(latitude),2)+pow(r2*sin(latitude),2)));
    double R = (radiusEarth+altitude);
    double deltaphi = longitude-longitudeLast;
    double deltatheta = latitude-latitudeLast;
    double deltar = altitude-altitudeLast;
    xE += (R*deltaphi*sin(latitude));
    xN += (R*deltatheta);
    h += (deltar);
    Inertial->DISPLACEMENT_NORTH = xN;
    Inertial->DISPLACEMENT_EAST = xE;
    Inertial->HEIGHT = h;
  }
  latitudeLast = latitude;
  longitudeLast = longitude;
  altitudeLast = altitude;
  return(status);
}

double convertGeo(uint8_t coordinate[],uint8_t coordinateSize){ // Converts array in degrees and months to single value in radians
  double minutes,degrees;
  uint8_t digitCount;
  for (uint8_t i=0;i<coordinateSize;i++){ // Find the number of digits to the left of the decimal
    if (coordinate[i]==46){
      digitCount = i;
      break;
    }
  }
  for (uint8_t i=0;i<coordinateSize;i++){
    if ((i<digitCount)&&(digitCount==4)){ // For values to the right of the decimal, input follows ddmm.mmmm or dddmm.mmmm
      if (i<2){
        degrees+=((coordinate[i]-48)*pow(10,digitCount-3-i)); // 4-3-(0,1)
      }
      else{
        minutes+=((coordinate[i]-48)*pow(10,digitCount-1-i)); // 4-1-(2,3)
      }
    }
    else if ((i<digitCount)&&(digitCount==5)){
      if (i<3){
        degrees+=((coordinate[i]-48)*pow(10,digitCount-3-i)); // 5-3-(0,1,2)
      }
      else{
        minutes+=((coordinate[i]-48)*pow(10,digitCount-1-i)); // 5-1-(3,4)
      }
    }
    else if (i>digitCount){ // for i 5-7
      minutes+=((coordinate[i]-48)*pow(10,digitCount-i)); // 4-(5,6,7)
    }
  }
  double coordinateConverted = (degrees+(minutes/60))*(PI/180);
  return(coordinateConverted);
}

double convertNum(uint8_t coordinate[],uint8_t coordinateSize){
  double coordinateConverted;
  uint8_t digitCount;
  for (uint8_t i=0;i<coordinateSize;i++){ // Find the number of digits to the left of the decimal
    if (coordinate[i]==46){
      digitCount = i;
      break;
    }
    else{
      digitCount = coordinateSize;
    }
  }
  for (uint8_t i=0;i<coordinateSize;i++){
    if (i<digitCount){
      coordinateConverted+=((coordinate[i]-48)*pow(10,digitCount-i-1));
    }
    else if (i>digitCount){
      coordinateConverted+=((coordinate[i]-48)*pow(10,digitCount-i));
    }
  }
  return(coordinateConverted);
}

bool readGPS(GPSinfo *info){
  static uint8_t data[1000],lengthOfEntry[15],indexOfComma[15],result,checksum1;  
  static uint16_t index[3]; // index[0] controls the current data byte to look at, index[1] controls the checksum calculation, index[2] controls which variable to write to
  static bool messageIntegrity;

  if (Serial1.available()>0){
    uint8_t inbound = Serial1.read();
    if (inbound==36){ // Reset on the $
      for(uint8_t i=0;i<3;i++){
        index[i]=0;
      }
    }
    else if ((inbound==42)||((index[1]>0)&&(index[1]<3))){ // Take * and next two values
      for (uint8_t i=0;i<=index[2];i++){

        if (i==0){ // If on message ID
          char messageIDArray[lengthOfEntry[i]+1];
          for (uint8_t j=0;j<=lengthOfEntry[i];j++){
            messageIDArray[j] = (char)data[j];
            if (j==lengthOfEntry[i]){
              messageIDArray[j] = '\0';
            }
          }
          String messageID = messageIDArray;
          info->MESSAGE_ID = messageID;
        }

        else if (i==1){ 
          uint8_t UTCTimeArray[lengthOfEntry[i]]; // UTC is 5 hours ahead of EST 
          uint8_t UTCTimeArraySize = sizeof(UTCTimeArray)/sizeof(UTCTimeArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            UTCTimeArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double UTCTime = convertNum(UTCTimeArray,UTCTimeArraySize);
          info->UTC_TIME = UTCTime;
        }

        else if (i==2){ 
          uint8_t latitudeArray[lengthOfEntry[i]];
          uint8_t latitudeArraySize = sizeof(latitudeArray)/sizeof(latitudeArray[0]);          
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            latitudeArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double latitude = convertGeo(latitudeArray,latitudeArraySize);
          info->LATITUDE = latitude;
        }

        else if (i==3){ 
          char NSIndicator;
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            NSIndicator = data[j+indexOfComma[i-1]+1];
          }
          if ((NSIndicator=='S')&&(info->LATITUDE>0)){
            info->LATITUDE *= -1;
          }
          info->NS_INDICATOR = NSIndicator;
        }

        else if (i==4){ 
          uint8_t longitudeArray[lengthOfEntry[i]];
          uint8_t longitudeArraySize = sizeof(longitudeArray)/sizeof(longitudeArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            longitudeArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double longitude = convertGeo(longitudeArray,longitudeArraySize);
          info->LONGITUDE = longitude;
        }

        else if (i==5){ 
          char EWIndicator;
          EWIndicator = (data[indexOfComma[i-1]+1]);
          if (info->EW_INDICATOR=='W'){
            info->LONGITUDE *= -1;
          }
          info->EW_INDICATOR = EWIndicator;
        }

        else if (i==6){ 
          uint8_t fixIndicator;
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            fixIndicator = data[j+indexOfComma[i-1]+1];
          }
          info->FIX_INDICATOR = fixIndicator-48;
        }

        else if (i==7){ 
          uint8_t satellitesUsedArray[lengthOfEntry[i]];
          uint8_t satellitesUsedArraySize = sizeof(satellitesUsedArray)/sizeof(satellitesUsedArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            satellitesUsedArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double satellitesUsed = convertNum(satellitesUsedArray,satellitesUsedArraySize);
          info->SATELLITES_USED = satellitesUsed;
        }    

        else if (i==8){ 
          uint8_t horizontalDilutionOfPrecisionArray[lengthOfEntry[i]];
          uint8_t horizontalDilutionOfPrecisionArraySize = sizeof(horizontalDilutionOfPrecisionArray)/sizeof(horizontalDilutionOfPrecisionArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            horizontalDilutionOfPrecisionArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double horizontalDilutionOfPrecision = convertNum(horizontalDilutionOfPrecisionArray,horizontalDilutionOfPrecisionArraySize);
          info->HORIZONTAL_DILUTION_OF_PRECISION = horizontalDilutionOfPrecision;
        }  

        else if (i==9){ 
          uint8_t altitudeArray[lengthOfEntry[i]];
          uint8_t altitudeArraySize = sizeof(altitudeArray)/sizeof(altitudeArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            altitudeArray[j] = (data[j+indexOfComma[i-1]+1]);
          }
          double altitude = convertNum(altitudeArray,altitudeArraySize);
          info->ALTITUDE = altitude;
        }    

        else if (i==10){ 
          char unitsMSL;
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            unitsMSL = data[j+indexOfComma[i-1]+1];
          }
          info->UNITS_MSL = unitsMSL;
        }  

        else if (i==11){ 
          uint8_t geoSeparationArray[lengthOfEntry[i]];
          uint8_t geoSeparationArraySize = sizeof(geoSeparationArray)/sizeof(geoSeparationArray[0]);
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            geoSeparationArray[j] = data[j+indexOfComma[i-1]+1];
          }
          double geoSeparation = convertNum(geoSeparationArray,geoSeparationArraySize);
          info->GEO_SEPARATION = geoSeparation;
        } 

        else if (i==12){ 
          char unitsGeo;
          for (uint8_t j=0;j<lengthOfEntry[i];j++){
            unitsGeo = data[j+indexOfComma[i-1]+1];
          }
          info->UNITS_GEO = unitsGeo;
        }
      } 
      ++index[1];
      if (index[1]==2){
        checksum1 = inbound;
      }
      if (index[1]==3){
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
    else  {
      data[index[0]] = inbound;

      if (data[index[0]]==44){ // If reading a comma, record size of entry and move to next entry
        indexOfComma[index[2]] = index[0];
        if (index[2]<1){
          lengthOfEntry[index[2]] = index[0];
        }
        else{
          lengthOfEntry[index[2]] = index[0]-indexOfComma[index[2]-1]-1;
        }
        index[2]++;
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
