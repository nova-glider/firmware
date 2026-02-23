/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "RF98.h"
#include "bme280.h"
#include "nmea_parse.h"
#include "sd.h"
#include <stdint.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    RF98_send((uint8_t *)"blub");
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, TX_LED_Pin|RX_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SERVO1_Pin|SERVO2_Pin|SERVO3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RFM96_DIO0_Pin|RES_RFM69_Pin|CS_SD_Pin|CS_RFM96_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : TX_LED_Pin RX_LED_Pin */
  GPIO_InitStruct.Pin = TX_LED_Pin|RX_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : SERVO1_Pin SERVO2_Pin SERVO3_Pin */
  GPIO_InitStruct.Pin = SERVO1_Pin|SERVO2_Pin|SERVO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM96_DIO0_Pin RES_RFM69_Pin CS_SD_Pin CS_RFM96_Pin */
  GPIO_InitStruct.Pin = RFM96_DIO0_Pin|RES_RFM69_Pin|CS_SD_Pin|CS_RFM96_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN_1_Pin BTN_2_Pin BTN_3_Pin */
  GPIO_InitStruct.Pin = BTN_1_Pin|BTN_2_Pin|BTN_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void send(const char *message) {
  RF98_send((uint8_t *)message);
}

void update_bme280_values(void) {
  BME280Calculation(&BME280);
  
  // float temperature = BME280.Temperature;
  // float pressure = BME280.Pressure;
  // float humidity = BME280.Humidity;
  // float altitudeTP = BME280.AltitudeTP;
  // float altitudeP = BME280.AltitudeP;
}

void update_mpu9250_values(void) {
  MPU_readProcessedData(&hi2c1);

  // int16_t raw_ax = MPURawData.ax;
  // int16_t raw_ay = MPURawData.ay;
  // int16_t raw_az = MPURawData.az;
  // int16_t raw_gx = MPURawData.gx;
  // int16_t raw_gy = MPURawData.gy;
  // int16_t raw_gz = MPURawData.gz;

  // float ax = MPUData.ax;
  // float ay = MPUData.ay;
  // float az = MPUData.az;

  // float gx = MPUData.gx;
  // float gy = MPUData.gy;
  // float gz = MPUData.gz;

  MPU_calcAttitude(&hi2c1);

  // float pitch = MPUattitude.p; // voor de pitch
  // float roll = MPUattitude.r; // voor de roll
  // float yaw = MPUattitude.y; // voor de yaw
}

void update_nmea_values(void) {
  nmea_parse(&GPSData, GPSBuffer);

  // double latitude = GPSData.latitude; // latitude in degrees with decimal places
  // char latside = GPSData.latSide; // N or S
  // double longitude = GPSData.longitude; // longitude in degrees with decimal places
  // char longside = GPSData.lonSide; // E or W
  // double altitudeGPS = GPSData.altitude; // altitude in meters
  // double hdop = GPSData.hdop; // horizontal dilution of precision
  // int satelliteCount = GPSData.satelliteCount; // number of satellites used in measurement
  // double fix = GPSData.fix; // 1 = fix, 0 = no fix
  // char lastMeasure[10];
  // strcpy(lastMeasure, GPSData.lastMeasure); // hhmmss.ss UTC of last successful measurement; time

}

// Helper function to convert int to string
static int int_to_str(char *str, int value) {
  if (value < 0) {
    str[0] = '-';
    return 1 + int_to_str(&str[1], -value);
  }
  if (value >= 10) {
    int len = int_to_str(str, value / 10);
    return len + int_to_str(&str[len], value % 10);
  }
  str[0] = value + '0';
  return 1;
}

const char* timestamp_to_string(void) {
  static char buffer[32];
  // Parse lastMeasure string (format: "hhmmss.ss")
  float timeValue = 0.0;
  for (int i = 0; GPSData.lastMeasure[i] && GPSData.lastMeasure[i] != '\0'; i++) {
    if (GPSData.lastMeasure[i] == '.') {
      timeValue = timeValue * 100 + (GPSData.lastMeasure[i+1] - '0') * 10 + (GPSData.lastMeasure[i+2] - '0');
      break;
    }
    timeValue = timeValue * 10 + (GPSData.lastMeasure[i] - '0');
  }
  
  int hours = (int)(timeValue / 10000);
  int minutes = (int)((timeValue - (hours * 10000)) / 100);
  float seconds = timeValue - (hours * 10000) - (minutes * 100);
  
  // Manual string building
  int pos = 0;
  const char *prefix = "2025-06-05T";
  for (int i = 0; prefix[i]; i++) buffer[pos++] = prefix[i];
  
  // Add hours (HH)
  buffer[pos++] = (hours / 10) + '0';
  buffer[pos++] = (hours % 10) + '0';
  buffer[pos++] = ':';
  
  // Add minutes (MM)
  buffer[pos++] = (minutes / 10) + '0';
  buffer[pos++] = (minutes % 10) + '0';
  buffer[pos++] = ':';
  
  // Add seconds (SS.SS)
  int sec_int = (int)seconds;
  buffer[pos++] = (sec_int / 10) + '0';
  buffer[pos++] = (sec_int % 10) + '0';
  buffer[pos++] = '.';
  int frac = (int)((seconds - sec_int) * 100);
  buffer[pos++] = (frac / 10) + '0';
  buffer[pos++] = (frac % 10) + '0';
  
  buffer[pos++] = 'Z';
  buffer[pos] = '\0';
  return buffer;
}

const char* json_from_values(void) {
  static char json_buffer[512];
  int pos = 0;
  
  // Helper to add string
  #define ADD_STR(s) do { const char *_s = (s); for(int _i=0; _s[_i]; _i++) json_buffer[pos++] = _s[_i]; } while(0)
  
  ADD_STR("{\"device_id\":\"glider-001\",\"timestamp\":\"");
  const char *ts = timestamp_to_string();
  for(int i=0; ts[i]; i++) json_buffer[pos++] = ts[i];
  ADD_STR("\",\"location\":{\"latitude\":");
  
  // Add latitude with 6 decimal places
  int lat_int = (int)GPSData.latitude;
  int lat_frac = (int)((GPSData.latitude - lat_int) * 1000000);
  if(lat_frac < 0) lat_frac = -lat_frac;
  pos += int_to_str(&json_buffer[pos], lat_int);
  json_buffer[pos++] = '.';
  for(int i = 100000; i > 0; i /= 10) json_buffer[pos++] = (lat_frac / i) % 10 + '0';
  
  ADD_STR(",\"longitude\":");
  int lon_int = (int)GPSData.longitude;
  int lon_frac = (int)((GPSData.longitude - lon_int) * 1000000);
  if(lon_frac < 0) lon_frac = -lon_frac;
  pos += int_to_str(&json_buffer[pos], lon_int);
  json_buffer[pos++] = '.';
  for(int i = 100000; i > 0; i /= 10) json_buffer[pos++] = (lon_frac / i) % 10 + '0';
  
  ADD_STR(",\"altitude\":");
  int alt_int = (int)GPSData.altitude;
  int alt_frac = (int)((GPSData.altitude - alt_int) * 100);
  if(alt_frac < 0) alt_frac = -alt_frac;
  pos += int_to_str(&json_buffer[pos], alt_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (alt_frac / 10) + '0';
  json_buffer[pos++] = (alt_frac % 10) + '0';
  
  ADD_STR("},\"readings\":{\"temperature_celsius\":");
  int temp_int = (int)BME280.Temperature;
  int temp_frac = (int)((BME280.Temperature - temp_int) * 100);
  if(temp_frac < 0) temp_frac = -temp_frac;
  pos += int_to_str(&json_buffer[pos], temp_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (temp_frac / 10) + '0';
  json_buffer[pos++] = (temp_frac % 10) + '0';
  
  ADD_STR(",\"humidity_percent\":");
  int hum_int = (int)BME280.Humidity;
  int hum_frac = (int)((BME280.Humidity - hum_int) * 100);
  if(hum_frac < 0) hum_frac = -hum_frac;
  pos += int_to_str(&json_buffer[pos], hum_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (hum_frac / 10) + '0';
  json_buffer[pos++] = (hum_frac % 10) + '0';
  
  ADD_STR(",\"air_pressure_hpa\":");
  int press_int = (int)BME280.Pressure;
  int press_frac = (int)((BME280.Pressure - press_int) * 100);
  if(press_frac < 0) press_frac = -press_frac;
  pos += int_to_str(&json_buffer[pos], press_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (press_frac / 10) + '0';
  json_buffer[pos++] = (press_frac % 10) + '0';
  
  ADD_STR(",\"pitch_deg\":");
  int pitch_int = (int)MPUattitude.p;
  int pitch_frac = (int)((MPUattitude.p - pitch_int) * 100);
  if(pitch_frac < 0) pitch_frac = -pitch_frac;
  pos += int_to_str(&json_buffer[pos], pitch_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (pitch_frac / 10) + '0';
  json_buffer[pos++] = (pitch_frac % 10) + '0';
  
  ADD_STR(",\"roll_deg\":");
  int roll_int = (int)MPUattitude.r;
  int roll_frac = (int)((MPUattitude.r - roll_int) * 100);
  if(roll_frac < 0) roll_frac = -roll_frac;
  pos += int_to_str(&json_buffer[pos], roll_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (roll_frac / 10) + '0';
  json_buffer[pos++] = (roll_frac % 10) + '0';
  
  ADD_STR(",\"yaw_deg\":");
  int yaw_int = (int)MPUattitude.y;
  int yaw_frac = (int)((MPUattitude.y - yaw_int) * 100);
  if(yaw_frac < 0) yaw_frac = -yaw_frac;
  pos += int_to_str(&json_buffer[pos], yaw_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (yaw_frac / 10) + '0';
  json_buffer[pos++] = (yaw_frac % 10) + '0';
  
  ADD_STR("},\"status\":{\"satellites\":");
  pos += int_to_str(&json_buffer[pos], GPSData.satelliteCount);
  
  ADD_STR(",\"fix\":");
  pos += int_to_str(&json_buffer[pos], (int)GPSData.fix);
  
  ADD_STR(",\"hdop\":");
  int hdop_int = (int)GPSData.hdop;
  int hdop_frac = (int)((GPSData.hdop - hdop_int) * 100);
  if(hdop_frac < 0) hdop_frac = -hdop_frac;
  pos += int_to_str(&json_buffer[pos], hdop_int);
  json_buffer[pos++] = '.';
  json_buffer[pos++] = (hdop_frac / 10) + '0';
  json_buffer[pos++] = (hdop_frac % 10) + '0';
  
  ADD_STR("}}");
  json_buffer[pos] = '\0';
  
  #undef ADD_STR
  return json_buffer;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
