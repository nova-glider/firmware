#include "main.h"
#include "stm32f1xx_hal.h"

#ifndef SENSORS_H
#define SENSORS_H

typedef struct SENSORDATA {
  float ax, ay, az, gx, gy, gz, r, p, y;
  float Temperature;
  float Pressure;
  float Humidity;
  float AltitudeP;
  float AltitudeTP;
  double latitude;      // latitude in degrees with decimal places
  char latSide;         // N or S
  double longitude;     // longitude in degrees with decimal places
  char lonSide;         // E or W
  float altitude;       // altitude in meters
  float hdop;           // horizontal dilution of precision
  int satelliteCount;   // number of satellites used in measurement
  int fix;              // 1 = fix, 0 = no fix
  char lastMeasure[10]; // hhmmss.ss UTC of last successful measurement; time
} SENSORDATA;

void initall(void);
void readall(SENSORDATA *sensordata);

#endif