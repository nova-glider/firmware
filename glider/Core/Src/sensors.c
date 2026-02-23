#include "sensors.h"
#include "MPU9250.h"
#include "RF98.h"
#include "bme280.h"
#include "fatfs.h"
#include "nmea_parse.h"
#include "sd.h"

#define RxBuffer_SIZE 64   // configure uart receive buffer size
#define GPSBuffer_SIZE 512 // gather a few rxBuffer frames before parsing

uint16_t oldPos = 0;
uint16_t newPos = 0;
uint8_t RxBuffer[RxBuffer_SIZE];
uint8_t GPSBuffer[GPSBuffer_SIZE];

GPS GPSData;

BME280_Data_t BME280;
#define SD_SPI_HANDLE hspi2

FATFS FatFs;  // Fatfs handle
FIL fil;      // File handle
FRESULT fres; // Result after operations

void initall() {
  HAL_Delay(1000); // a short delay is important to let the SD card settle

  RF98_Init();

  MPU_begin(&hi2c1, AD0_LOW, AFSR_16G, GFSR_2000DPS, 0.98, 0.004);
  MPU_calibrateGyro(&hi2c1, 1500);
  MPU_calcAttitude(&hi2c1);

  Reset_BME280();
  BME280Init();
}

void readall(SENSORDATA *sensordata) {
  BME280Calculation(&BME280);
  MPU_readProcessedData(&hi2c1);
  MPU_calcAttitude(&hi2c1);
  nmea_parse(&GPSData, GPSBuffer);

  sensordata->ax = MPUData.ax;
  sensordata->ay = MPUData.ay;
  sensordata->az = MPUData.az;

  sensordata->gx = MPUData.gx;
  sensordata->gy = MPUData.gy;
  sensordata->gz = MPUData.gz;

  sensordata->p = MPUattitude.p;
  sensordata->r = MPUattitude.r;
  sensordata->y = MPUattitude.y;

  sensordata->Temperature = BME280.Temperature;
  sensordata->Pressure = BME280.Pressure;
  sensordata->Humidity = BME280.Humidity;
  sensordata->AltitudeTP = BME280.AltitudeTP;
  sensordata->AltitudeP = BME280.AltitudeP;

  sensordata->latitude = GPSData.latitude;
  sensordata->latSide = GPSData.latSide;
  sensordata->longitude = GPSData.longitude;
  sensordata->lonSide = GPSData.lonSide;
  sensordata->altitude = GPSData.altitude;
  sensordata->hdop = GPSData.hdop;
  sensordata->satelliteCount = GPSData.satelliteCount;
  sensordata->fix = GPSData.fix;
  strcpy(sensordata->lastMeasure, GPSData.lastMeasure);
}