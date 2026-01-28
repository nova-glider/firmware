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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "MPU9250.h"
#include "RF98.h"
#include "bme280.h"
#include "nmea_parse.h"
#include <stdint.h>
#include <limits.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
BME280_Data_t BME280;

/* Buffer size constants */
#define JSON_BUFFER_SIZE 512
#define TIMESTAMP_BUFFER_SIZE 32
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
#define RxBuffer_SIZE 64   // configure uart receive buffer size
#define GPSBuffer_SIZE 512 // gather a few rxBuffer frames before parsing

uint16_t oldPos = 0;
uint16_t newPos = 0;
uint8_t RxBuffer[RxBuffer_SIZE];
uint8_t GPSBuffer[GPSBuffer_SIZE];

GPS GPSData;
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
  /* USER CODE BEGIN 2 */
  RF98_Init();

  MPU_begin(&hi2c1, AD0_LOW, AFSR_16G, GFSR_2000DPS, 0.98, 0.004);
  MPU_calibrateGyro(&hi2c1, 1500);
  MPU_calcAttitude(&hi2c1);

  Reset_BME280();
  BME280Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    update_bme280_values();
    update_mpu9250_values();
    update_nmea_values();

    send(json_from_values());
    
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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

/**
 * @brief Helper function to convert int to string with bounds checking
 * @param str Output string buffer
 * @param buf_size Remaining buffer size
 * @param value Integer value to convert
 * @return Number of characters written, or -1 on error
 */
static int int_to_str_safe(char *str, int buf_size, int value) {
  if (buf_size <= 0) {
    return -1; // Buffer overflow
  }
  
  // Handle INT_MIN special case
  if (value == INT_MIN) {
    const char *min_str = "-2147483648";
    int len = 0;
    while (min_str[len] && len < buf_size - 1) {
      str[len] = min_str[len];
      len++;
    }
    if (len >= buf_size - 1) {
      return -1; // Buffer overflow
    }
    return len;
  }
  
  if (value < 0) {
    str[0] = '-';
    int result = int_to_str_safe(&str[1], buf_size - 1, -value);
    if (result < 0) {
      return -1;
    }
    return 1 + result;
  }
  
  if (value >= 10) {
    int len = int_to_str_safe(str, buf_size, value / 10);
    if (len < 0) {
      return -1;
    }
    int digit_result = int_to_str_safe(&str[len], buf_size - len, value % 10);
    if (digit_result < 0) {
      return -1;
    }
    return len + digit_result;
  }
  
  if (buf_size < 1) {
    return -1;
  }
  str[0] = value + '0';
  return 1;
}

/**
 * @brief Convert GPS timestamp to ISO 8601 format string
 * @return Pointer to static buffer containing timestamp, or NULL on error
 */
const char* timestamp_to_string(void) {
  static char buffer[TIMESTAMP_BUFFER_SIZE];
  
  // Validate GPS data
  if (!GPSData.lastMeasure[0]) {
    return NULL; // No GPS data available
  }
  
  // Parse lastMeasure string (format: "hhmmss.ss")
  float timeValue = 0.0;
  int i;
  for (i = 0; i < 10 && GPSData.lastMeasure[i] != '\0'; i++) {
    if (GPSData.lastMeasure[i] == '.') {
      if (GPSData.lastMeasure[i+1] >= '0' && GPSData.lastMeasure[i+1] <= '9' &&
          GPSData.lastMeasure[i+2] >= '0' && GPSData.lastMeasure[i+2] <= '9') {
        timeValue = timeValue * 100 + (GPSData.lastMeasure[i+1] - '0') * 10 + 
                    (GPSData.lastMeasure[i+2] - '0');
      }
      break;
    }
    if (GPSData.lastMeasure[i] >= '0' && GPSData.lastMeasure[i] <= '9') {
      timeValue = timeValue * 10 + (GPSData.lastMeasure[i] - '0');
    }
  }
  
  int hours = (int)(timeValue / 10000);
  int minutes = (int)((timeValue - (hours * 10000)) / 100);
  float seconds = timeValue - (hours * 10000) - (minutes * 100);
  
  // Validate time values
  if (hours > 23 || minutes > 59 || seconds >= 60.0) {
    return NULL; // Invalid time
  }
  
  // Manual string building with bounds checking
  int pos = 0;
  const char *prefix = "2025-06-05T";
  for (i = 0; prefix[i] && pos < TIMESTAMP_BUFFER_SIZE - 1; i++) {
    buffer[pos++] = prefix[i];
  }
  
  if (pos + 12 >= TIMESTAMP_BUFFER_SIZE) {
    return NULL; // Buffer too small
  }
  
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
  if (frac < 0) frac = 0;
  buffer[pos++] = (frac / 10) + '0';
  buffer[pos++] = (frac % 10) + '0';
  
  buffer[pos++] = 'Z';
  buffer[pos] = '\0';
  return buffer;
}

/**
 * @brief Add string to buffer with bounds checking
 * @param buffer Target buffer
 * @param pos Current position in buffer (updated)
 * @param max_size Maximum buffer size
 * @param str String to add
 * @return 0 on success, -1 on overflow
 */
static int add_str_safe(char *buffer, int *pos, int max_size, const char *str) {
  int i;
  for (i = 0; str[i] != '\0'; i++) {
    if (*pos >= max_size - 1) {
      return -1; // Buffer overflow
    }
    buffer[(*pos)++] = str[i];
  }
  return 0;
}

/**
 * @brief Add integer to buffer with bounds checking
 * @param buffer Target buffer
 * @param pos Current position in buffer (updated)
 * @param max_size Maximum buffer size
 * @param value Integer to add
 * @return 0 on success, -1 on overflow
 */
static int add_int_safe(char *buffer, int *pos, int max_size, int value) {
  int written = int_to_str_safe(&buffer[*pos], max_size - *pos, value);
  if (written < 0) {
    return -1;
  }
  *pos += written;
  return 0;
}

/**
 * @brief Add float to buffer with specified decimal places and bounds checking
 * @param buffer Target buffer
 * @param pos Current position in buffer (updated)
 * @param max_size Maximum buffer size
 * @param value Float value to add
 * @param decimal_places Number of decimal places
 * @return 0 on success, -1 on overflow
 */
static int add_float_safe(char *buffer, int *pos, int max_size, float value, int decimal_places) {
  // Calculate multiplier for decimal places
  int multiplier = 1;
  for (int i = 0; i < decimal_places; i++) {
    multiplier *= 10;
  }
  
  int int_part = (int)value;
  int frac_part = (int)((value - int_part) * multiplier);
  if (frac_part < 0) frac_part = -frac_part;
  
  // Add integer part
  if (add_int_safe(buffer, pos, max_size, int_part) < 0) {
    return -1;
  }
  
  // Add decimal point
  if (*pos >= max_size - 1) {
    return -1;
  }
  buffer[(*pos)++] = '.';
  
  // Add fractional part with leading zeros
  int divisor = multiplier / 10;
  for (int i = 0; i < decimal_places; i++) {
    if (*pos >= max_size - 1) {
      return -1;
    }
    buffer[(*pos)++] = (frac_part / divisor) % 10 + '0';
    divisor /= 10;
  }
  
  return 0;
}

/**
 * @brief Generate JSON string from sensor values with bounds checking
 * @return Pointer to static JSON buffer, or NULL on error
 */
const char* json_from_values(void) {
  static char json_buffer[JSON_BUFFER_SIZE];
  int pos = 0;
  
  // Get timestamp
  const char *ts = timestamp_to_string();
  if (!ts) {
    return NULL; // Invalid timestamp
  }
  
  // Build JSON with safety checks
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, "{\"device_id\":\"glider-001\",\"timestamp\":\"") < 0) return NULL;
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ts) < 0) return NULL;
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, "\",\"location\":{\"latitude\":") < 0) return NULL;
  
  // Add latitude with 6 decimal places
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, GPSData.latitude, 6) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"longitude\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, GPSData.longitude, 6) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"altitude\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, GPSData.altitude, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, "},\"readings\":{\"temperature_celsius\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, BME280.Temperature, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"humidity_percent\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, BME280.Humidity, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"air_pressure_hpa\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, BME280.Pressure, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"pitch_deg\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, MPUattitude.p, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"roll_deg\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, MPUattitude.r, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"yaw_deg\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, MPUattitude.y, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, "},\"status\":{\"satellites\":") < 0) return NULL;
  if (add_int_safe(json_buffer, &pos, JSON_BUFFER_SIZE, GPSData.satelliteCount) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"fix\":") < 0) return NULL;
  if (add_int_safe(json_buffer, &pos, JSON_BUFFER_SIZE, (int)GPSData.fix) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, ",\"hdop\":") < 0) return NULL;
  if (add_float_safe(json_buffer, &pos, JSON_BUFFER_SIZE, GPSData.hdop, 2) < 0) return NULL;
  
  if (add_str_safe(json_buffer, &pos, JSON_BUFFER_SIZE, "}}") < 0) return NULL;
  
  json_buffer[pos] = '\0';
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