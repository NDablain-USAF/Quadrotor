#ifndef GPS_H
#define GPS_H

#include <Arduino.h>

class GPS {
  private:
    String
      messageID;
    double
      displacementNorth,
      displacementEast,
      height,
      UTCTime,
      latitude,
      latitudeLast,
      longitude,
      longitudeLast,
      horizontalDilutionOfPrecision,
      altitude,
      altitudeLast,
      geoSeparation;
    uint32_t
      printTime;
    uint16_t
      index[3],
      biasControl;
    uint8_t
      positionStatus,
      fixIndicator,
      satellitesUsed,
      result,
      checksum1,
      data[1000],
      lengthOfEntry[15],
      indexOfComma[15];
    char
      NSIndicator,
      EWIndicator,
      unitsMSL,
      unitsGeo;
    bool
      messageIntegrity;
  public:
    GPS();
    double
      convertGeo(uint8_t coordinate[], uint8_t coordinateSize),
      convertNum(uint8_t number[], uint8_t numberSize);
    void
      displacements(),
      readGPS(),
      updategps(double *DISPLACEMENTNORTH, double *DISPLACEMENTEAST, double *HEIGHT),
      init();
};

#endif