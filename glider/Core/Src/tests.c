#include "MPU9250.h"
#include "RF98.h"
#include "bme280.h"
#include "nmea_parse.h"
#include "sd.h"
#include "sensors.h"
#include <stdint.h>

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

bool err = false;

void test_mpu9250(void) {
    //  init
    MPU_begin(&hi2c1, AD0_LOW, AFSR_16G, GFSR_2000DPS, 0.98, 0.004);
    MPU_calibrateGyro(&hi2c1, 1500);
    MPU_calcAttitude(&hi2c1);

    // read shit
    MPU_readRawData(&hi2c1);
    MPU_readProcessedData(&hi2c1);
    
}

void test_bme280(void) {
    // init
    Reset_BME280();
    BME280Init();

    // read
    BME280Calculation(&BME280);
}

void test_nmea(void) {
    // parse the NMEA sentence
    nmea_parse(&GPSData, GPSBuffer);
}

void test_sd(void) {
    // short wait
    HAL_Delay(1000);

    // init
    initsd();

    // write a simple file
    int success = write("test.txt", "Hello, SD card!");
    if (!success) {
        while (1)
            ;
    }
}

void test_rfm98(void) {
    RF98_Init();
    RF98_send((uint8_t *)"Hello, RFM96!");
}