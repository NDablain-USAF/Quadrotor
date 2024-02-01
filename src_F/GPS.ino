const int FIX_PIN = 3;
const int SYNC_PIN = 7;
int i;
void setup() {
  Serial.begin(9600);  
  Serial1.begin(9600);
  pinMode(FIX_PIN, INPUT);
  pinMode(SYNC_PIN, INPUT);
}

void loop() {
  static byte latitude[9],longitude[10],altitudeMSL[3];
  static double Xn,Xe,h;

  bool FIX = fix();
  if (FIX==true){
    bool Integrity = readGPS(latitude,longitude,altitudeMSL);
    if (Integrity>0){
      byte latitudesize = sizeof(latitude)/sizeof(latitude[0]); // Convert arrays to doubles for calculations
      double Latitude = convertGeo(latitude,latitudesize);
      byte longitudesize = sizeof(longitude)/sizeof(longitude[0]);
      double Longitude = convertGeo(longitude,longitudesize);
      byte altitudesize = sizeof(altitudeMSL)/sizeof(altitudeMSL[0]);
      double Altitude = convertAlt(altitudeMSL,altitudesize);
      Displacements(&Xn,&Xe,Latitude,Longitude);
      
      Serial.print("Displacment North: ");
      Serial.print(Xn);
      Serial.print(" , ");
      Serial.print("Displacment East: ");
      Serial.print(Xe);
      Serial.print(" , ");
      Serial.print("Height: ");
      Serial.println(Altitude);
    }
  }
}

void Displacements(double *Xn, double *Xe, double Latitude, double Longitude){
  static long biasControl;
  static double biasLat,biasLong;
  static double Latitudelast,Longitudelast;
  double r1 = 6378.137e3; // in meters
  double r2 = 6356.752e3;
  float c[2] = {0.5,0.5};
  Latitude = Latitudelast*c[1]+Latitude*c[0];
  Longitude = Longitudelast*c[1]+Longitude*c[0];
  Latitudelast = Latitude;
  Longitudelast = Longitude;
  if (biasControl==10000){
    biasLat = Latitude;
    biasLong = Longitude;
    ++biasControl;
  }
  else if (biasControl<10000){
    ++biasControl;
  }
  else {
  double radiusEarth = sqrt((pow(pow(r1,2)*cos(Latitude),2)+pow(pow(r2,2)*sin(Latitude),2))/(pow(r1*cos(Latitude),2)+pow(r2*sin(Latitude),2)));
  *Xn = radiusEarth*tan(Latitude-biasLat);
  *Xe = -radiusEarth*cos(Latitude-biasLat)*tan(Longitude-biasLong);
  }

  
}

double convertGeo(byte coordinate[],byte coordinatesize){ // Converts from Degrees & Months to radians
  double Minutes,Degrees;
  byte n;
  for (int i=0;i<coordinatesize;i++){ // Find the number of digits to the right of the decimal
    if (coordinate[i]==46){
      n = i;
    }
  }
  for (int i=0;i<coordinatesize;i++){
    if (i<n){
      Minutes+=((coordinate[i]-48)*pow(10,i-n));
    }
    else if ((i>n)&&(i<n+3)){
      Minutes+=((coordinate[i]-48)*pow(10,i-(n+1)));
    }
    else if ((i>n)&&(i<coordinatesize)){
      Degrees+=((coordinate[i]-48)*pow(10,i-(n+3)));
    }
  }
  double Coordinate = (Degrees+(Minutes/60))*(PI/180);
  return(Coordinate);
}

double convertAlt(byte coordinate[],byte coordinatesize){
  double Coordinate;
  byte n;
  for (int i=0;i<coordinatesize;i++){ // Find the number of digits to the right of the decimal
    if (coordinate[i]==46){
      n = i;
    }
  }
  for (int i=0;i<coordinatesize;i++){
    if (i<n){
      Coordinate+=((coordinate[i]-48)*pow(10,i-n));
    }
    else if (i>n){
      Coordinate+=((coordinate[i]-48)*pow(10,i-(n+1)));
    }
  }
  return(Coordinate);
}

bool readGPS(byte latitude[9],byte longitude[10],byte altitudeMSL[3]){
  static byte data[1000],messageID[5],UTCTime1[10],UTCTime2[10],latitude1[9],latitude2[9],longitude1[10],longitude2[10],horizontalDilutionOfPrecision[4];
  static byte SV1[2],SV2[2],SV3[2],SV4[2],SV5[2],SV6[2],SV7[2],SV8[2],SV9[2],SV10[2],SV11[2],SV12[2],speedOverGround[4],courseOverGround[6],date[6];
  static byte i,j,k,l,result,checksum1,checksum2,mode1,mode2,status,NSIndicator1,NSIndicator2,EWIndicator1,EWIndicator2,fixIndicator,satellitesUsed[2],unitsMSL;
  static byte geoSeparation[4],unitsGeo;

  static uint32_t start;
  
  if (Serial1.available()>0){
    byte inbound = Serial1.read();
    if (inbound==36){ // Reset on the $
      start = millis();
      j = 0;
      i = 0;
      k = 0; 
      l = 0;
    }
    else if ((inbound==42)||((k>0)&&(k<3))){ // Take * and next two values
      if (k==1){
        checksum1 = inbound;
      }
      if (k==2){
        checksum2 = inbound;
      }
      if (k==2){
        byte mask = 0b00001111;
        byte result1 = result>>4; // First value in binary
          if (result1<10){ // If the value in HEX is a numeral (less than 10 in dec)
            result1+=48; // Add 48 to get its HEX representation
          }
          else{ // If the value in HEX is a character
            result1+=55; // Add 55 to gets its HEX representation
          }
        byte result2 = result&mask; // Second value in binary
          if (result2<10){ // If the value in binary is a numeral (less than 10)
            result2+=48; // Add 48 to get its Dec representation
          }
          else{ // If the value in binary is a character
            result2+=55; // Corresponding value in Dec
          }
        bool message_Integrity = false;
        if ((result1==checksum1)&&(result2==checksum2)){
          message_Integrity = true;
        }
        return(message_Integrity);
      }
      ++k;
      j = 1;
    }
    else if (j!=1){
      data[i] = inbound;
      int sum;
      for (int ii=0;ii<5;ii++){ // Sum Dec equivalents of binary letters
        sum+=messageID[ii];
      }


      if (data[i]==44){ // If reading a comma, move to next entry
        l++;
      }

      else if (l==0){ // If on message ID
        messageID[abs(i-4)] = data[i];
      }

      else if (l==1){ 
        if (sum==358){ // GPGGA & GPRMC
          UTCTime1[abs(i-15)] = data[i];
        }
        else if (sum==370){ // GPGSA
          mode1 = data[i];
        }
        else if (sum==377){
          UTCTime2[abs(i-15)] = data[i];
        }
      }

      else if (l==2){
        if (sum==358){
          latitude[abs(i-25)] = data[i];
        }
        else if (sum==370){
          mode2 = data[i];
        }
        else if (sum==377){
          status = data[i];
        }
      }

      else if (l==3){
        if (sum==358){
          NSIndicator1 = data[i];
        }
        else if (sum==370){
          SV1[abs(i-11)] = data[i];
        }
        else if (sum==377){
          latitude2[abs(i-27)] = data[i];
        }
      }

      else if (l==4){
        if (sum==358){
          longitude[abs(i-38)] = data[i];
        }
        else if (sum==370){
          SV2[abs(i-14)] = data[i];
        }
        else if (sum==377){
          NSIndicator2 = data[i];
        }
      }

      else if (l==5){
        if (sum==358){
          EWIndicator1 = data[i];
        }
        else if (sum==370){
          SV3[abs(i-17)] = data[i];
        }
        else if (sum==377){
          longitude2[abs(i-40)] = data[i];
        }
      }

      else if (l==6){
        if (sum==358){
          fixIndicator = data[i];
        }
        else if (sum==370){
          SV4[abs(i-20)] = data[i];
        }
        else if (sum==377){
          EWIndicator2 = data[i];
        }
      }

      else if (l==7){
        if (sum==358){
          satellitesUsed[abs(i-44)] = data[i];
        }
        else if (sum==370){
          SV5[abs(i-23)] = data[i];
        }
        else if (sum==377){
          speedOverGround[abs(i-47)] = data[i];
        }
      }      

      else if (l==8){
        if (sum==358){
          horizontalDilutionOfPrecision[abs(i-49)] = data[i];
        }
        else if (sum==370){
          SV6[abs(i-26)] = data[i];
        }
        else if (sum==377){
          courseOverGround[abs(i-54)] = data[i];
        }
      }    

      else if (l==9){
        if (sum==358){
          altitudeMSL[abs(i-53)] = data[i];
        }
        else if (sum==370){
          SV7[abs(i-29)] = data[i];
        }
        else if (sum==377){
          date[abs(i-61)] = data[i];
        }
      }    

      else if (l==10){
        if (sum==358){
          unitsMSL = data[i];
        }
        else if (sum==370){
          SV8[abs(i-32)] = data[i];
        }
        else if (sum==377){
          mode2 = data[i];
        }
      }   

      else if (l==11){
        if (sum==358){
          geoSeparation[abs(i-60)] = data[i];
        }
        else if (sum==370){
          SV9[abs(i-35)] = data[i];
        }
      }  

      else if (l==12){
        if (sum==358){
          unitsGeo = data[i];
        }
        else if (sum==370){
          SV10[abs(i-38)] = data[i];
        }
      } 


      if (i>0){
        if (i==1){ // For first two values, establish result variable
          result = data[i]^data[i-1]; // Bitwise xor, order of operation doesn't matter
        }
        else if (i>1){ // Once result variable is established, use it
          result = result^data[i];
        }
      }
      ++i;
    }
  }
}

bool fix(){
  static int a;
  static unsigned long period;
  static unsigned long time;
  bool FIX;
  if ((digitalRead(FIX_PIN)==1)&&(a==0)){ // Rising edge
    a = 1;
  }
  else if ((digitalRead(FIX_PIN)==0)&&(a==1)){ // Falling edge
    a = 0;
    period = (millis()-time)/2; // Record the period
    time = millis();
  }

  if (period<1500){ // A period of 1s means a fix hasn't been found
    FIX = false;
  }
  else { // A longer duration period means a fix has been found, may be around 15s for 2D and 30s for 3D but need to confirm
    FIX = true;
  }
  return(FIX);
}
