#include "sd.h"
#include "RF98.h"
#include "fatfs.h"
#include "main.h"
#include "sensors.h"
#include <stdio.h>
#include <string.h>

FATFS FatFs;  // Fatfs handle
FIL fil;      // File handle
FRESULT fres; // Result after operations

void initsd(void) {
  HAL_Delay(1000);
  fres = f_mount(&FatFs, "", 1); // 1=mount now
  if (fres != FR_OK) {
    while (1)
      ;
  }

  DWORD free_clusters, free_sectors, total_sectors;

  FATFS *getFreeFs;

  fres = f_getfree("", &free_clusters, &getFreeFs);
  if (fres != FR_OK) {
    while (1)
      ;
  }

  // Formula comes from ChaN's documentation
  total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
  free_sectors = free_clusters * getFreeFs->csize;

  fres =
      f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

  BYTE readBuf[256];
  const char *header = "ax,ay,az,gx,gy,gz,r,p,y,Temperature,Pressure,Humidity,"
                       "AltitudeP,AltitudeTP,latitude,latSide,longitude,"
                       "lonSide,altitude,hdop,satelliteCount,fix,lastMeasure\n";
  size_t headerLen = strlen(header);
  strncpy((char *)readBuf, header, sizeof(readBuf) - 1);
  readBuf[sizeof(readBuf) - 1] = '\0';
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, headerLen, &bytesWrote);
}

void writeandsend(SENSORDATA *sensordata) {
  BYTE readBuf[30];

  // Now let's try and write a file "write.txt"
  fres =
      f_open(&fil, "write.txt", FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);

  // Copy in a string
  strncpy((char *)readBuf, "a new file is made!", 19);
  UINT bytesWrote;
  fres = f_write(&fil, readBuf, 19, &bytesWrote);

  // Format sensor data as CSV string
  char csvBuf[512];
  snprintf(
      csvBuf, sizeof(csvBuf),
      "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%c,%f,%c,%f,%f,%d,%d,%s\n",
      sensordata->ax, sensordata->ay, sensordata->az, sensordata->gx,
      sensordata->gy, sensordata->gz, sensordata->r, sensordata->p,
      sensordata->y, sensordata->Temperature, sensordata->Pressure,
      sensordata->Humidity, sensordata->AltitudeP, sensordata->AltitudeTP,
      sensordata->latitude, sensordata->latSide, sensordata->longitude,
      sensordata->lonSide, sensordata->altitude, sensordata->hdop,
      sensordata->satelliteCount, sensordata->fix, sensordata->lastMeasure);
  fres = f_write(&fil, csvBuf, strlen(csvBuf), &bytesWrote);

  // Be a tidy kiwi - don't forget to close your file!
  f_close(&fil);

  // todo: send
  RF98_send((uint8_t *)csvBuf); // easy hopelijk 🙏
}