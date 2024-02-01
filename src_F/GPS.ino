const uint8_t FIX_PIN = 3;
const uint8_t SYNC_PIN = 7;

void setup() {
  Serial.begin(9600);  
  Serial1.begin(9600);
  pinMode(FIX_PIN, INPUT);
  pinMode(SYNC_PIN, INPUT);
}

void loop() {
  static uint8_t latitudeArray[9],longitudeArray[10],altitudeArray[3];
  static double xN,xE,h;
  
  bool FIX = fix();
  if (FIX==true){
    bool Integrity = readGPS(latitudeArray,longitudeArray,altitudeArray);
    if (Integrity>0){
      uint8_t latitudeSize = sizeof(latitudeArray)/sizeof(latitudeArray[0]); // Convert arrays to doubles for calculations
      double latitude = convertGeo(latitudeArray,latitudeSize);
      uint8_t longitudeSize = sizeof(longitudeArray)/sizeof(longitudeArray[0]);
      double longitude = convertGeo(longitudeArray,longitudeSize);
      uint8_t altitudeSize = sizeof(altitudeArray)/sizeof(altitudeArray[0]);
      double altitude = convertAlt(altitudeArray,altitudeSize);
      displacements(&xN,&xE,latitude,longitude);
      
      Serial.print("Displacment North: ");
      Serial.print(xN);
      Serial.print(" , ");
      Serial.print("Displacment East: ");
      Serial.print(xE);
      Serial.print(" , ");
      Serial.print("Height: ");
      Serial.println(altitude);
      
    }
  }
}

void displacements(double *Xn, double *Xe, double latitude, double longitude){
  static uint32_t biasControl;
  static double biasLat,biasLong,latitudeLast,longitudeLast;
  uint32_t r1 = 6378137; // in meters
  uint32_t r2 = 6356752;
  float c[2] = {0.6,0.4};
  latitude = latitudeLast*c[1]+latitude*c[0];
  longitude = longitudeLast*c[1]+longitude*c[0];
  latitudeLast = latitude;
  longitudeLast = longitude;
  if (biasControl==5000){
    biasLat = latitude;
    biasLong = longitude;
    ++biasControl;
  }
  else if (biasControl<5000){
    ++biasControl;
  }
  else {
  uint32_t radiusEarth = sqrt((pow(pow(r1,2)*cos(latitude),2)+pow(pow(r2,2)*sin(latitude),2))/(pow(r1*cos(latitude),2)+pow(r2*sin(latitude),2)));
  *Xn = radiusEarth*tan(latitude-biasLat);
  *Xe = -radiusEarth*cos(latitude-biasLat)*tan(longitude-biasLong);
  }  
}

double convertGeo(uint8_t coordinate[],uint8_t coordinateSize){ // Converts array in degrees and months to single value in radians
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
  double coordinateConverted = (degrees+(minutes/60))*(PI/180);
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

bool readGPS(uint8_t latitude[9],uint8_t longitude[10],uint8_t altitudeMSL[3]){
  static uint8_t data[1000],messageID[5],UTCTime1[10],UTCTime2[10],latitude1[9],latitude2[9],longitude1[10],longitude2[10],horizontalDilutionOfPrecision[4];
  static uint8_t SV1[2],SV2[2],SV3[2],SV4[2],SV5[2],SV6[2],SV7[2],SV8[2],SV9[2],SV10[2],SV11[2],SV12[2],speedOverGround[4],courseOverGround[6],date[6];
  static uint8_t index[4],result,checksum1,mode1,mode2,status,NSIndicator1,NSIndicator2,EWIndicator1,EWIndicator2,fixIndicator,satellitesUsed[2],unitsMSL;
  static uint8_t geoSeparation[4],unitsGeo;

  static bool messageIntegrity;
  
  if (Serial1.available()>0){
    uint8_t inbound = Serial1.read();
    if (inbound==36){ // Reset on the $
      uint32_t start = millis();
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
        else if (sum==370){ // GPGSA
          mode1 = data[index[0]];
        }
        else if (sum==377){
          UTCTime2[abs(index[0]-15)] = data[index[0]];
        }
      }

      else if (index[3]==2){
        if (sum==358){
          latitude[abs(index[0]-25)] = data[index[0]];
        }
        else if (sum==370){
          mode2 = data[index[0]];
        }
        else if (sum==377){
          status = data[index[0]];
        }
      }

      else if (index[3]==3){
        if (sum==358){
          NSIndicator1 = data[index[0]];
        }
        else if (sum==370){
          SV1[abs(index[0]-11)] = data[index[0]];
        }
        else if (sum==377){
          latitude2[abs(index[0]-27)] = data[index[0]];
        }
      }

      else if (index[3]==4){
        if (sum==358){
          longitude[abs(index[0]-38)] = data[index[0]];
        }
        else if (sum==370){
          SV2[abs(index[0]-14)] = data[index[0]];
        }
        else if (sum==377){
          NSIndicator2 = data[index[0]];
        }
      }

      else if (index[3]==5){
        if (sum==358){
          EWIndicator1 = data[index[0]];
        }
        else if (sum==370){
          SV3[abs(index[0]-17)] = data[index[0]];
        }
        else if (sum==377){
          longitude2[abs(index[0]-40)] = data[index[0]];
        }
      }

      else if (index[3]==6){
        if (sum==358){
          fixIndicator = data[index[0]];
        }
        else if (sum==370){
          SV4[abs(index[0]-20)] = data[index[0]];
        }
        else if (sum==377){
          EWIndicator2 = data[index[0]];
        }
      }

      else if (index[3]==7){
        if (sum==358){
          satellitesUsed[abs(index[0]-44)] = data[index[0]];
        }
        else if (sum==370){
          SV5[abs(index[0]-23)] = data[index[0]];
        }
        else if (sum==377){
          speedOverGround[abs(index[0]-47)] = data[index[0]];
        }
      }      

      else if (index[3]==8){
        if (sum==358){
          horizontalDilutionOfPrecision[abs(index[0]-49)] = data[index[0]];
        }
        else if (sum==370){
          SV6[abs(index[0]-26)] = data[index[0]];
        }
        else if (sum==377){
          courseOverGround[abs(index[0]-54)] = data[index[0]];
        }
      }    

      else if (index[3]==9){
        if (sum==358){
          altitudeMSL[abs(index[0]-53)] = data[index[0]];
        }
        else if (sum==370){
          SV7[abs(index[0]-29)] = data[index[0]];
        }
        else if (sum==377){
          date[abs(index[0]-61)] = data[index[0]];
        }
      }    

      else if (index[3]==10){
        if (sum==358){
          unitsMSL = data[index[0]];
        }
        else if (sum==370){
          SV8[abs(index[0]-32)] = data[index[0]];
        }
        else if (sum==377){
          mode2 = data[index[0]];
        }
      }   

      else if (index[3]==11){
        if (sum==358){
          geoSeparation[abs(index[0]-60)] = data[index[0]];
        }
        else if (sum==370){
          SV9[abs(index[0]-35)] = data[index[0]];
        }
      }  

      else if (index[3]==12){
        if (sum==358){
          unitsGeo = data[index[0]];
        }
        else if (sum==370){
          SV10[abs(index[0]-38)] = data[index[0]];
        }
      } 


      if (index[0]>0){
        if (index[0]==1){ // For first two values, establish result variable
          result = data[index[0]]^data[index[0]-1]; // Bitwise xor, order of operation doesn't matter
        }
        else if (index[0]>1){ // Once result variable is established, use it
          result = result^data[index[0]];
        }
      }
    /*
    for (int i=0;i<4;i++){
      Serial.print((char)horizontalDilutionOfPrecision[i]);
    }
    Serial.println();
    */  
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
