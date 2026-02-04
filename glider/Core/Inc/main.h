/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define TX_LED_Pin GPIO_PIN_13
#define TX_LED_GPIO_Port GPIOC
#define RX_LED_Pin GPIO_PIN_14
#define RX_LED_GPIO_Port GPIOC
#define SERVO1_Pin GPIO_PIN_0
#define SERVO1_GPIO_Port GPIOA
#define SERVO2_Pin GPIO_PIN_1
#define SERVO2_GPIO_Port GPIOA
#define SERVO3_Pin GPIO_PIN_2
#define SERVO3_GPIO_Port GPIOA
#define RFM96_DIO0_Pin GPIO_PIN_0
#define RFM96_DIO0_GPIO_Port GPIOB
#define RES_RFM69_Pin GPIO_PIN_1
#define RES_RFM69_GPIO_Port GPIOB
#define BTN_1_Pin GPIO_PIN_2
#define BTN_1_GPIO_Port GPIOB
#define CS_SD_Pin GPIO_PIN_11
#define CS_SD_GPIO_Port GPIOB
#define CS_RFM96_Pin GPIO_PIN_12
#define CS_RFM96_GPIO_Port GPIOB
#define BTN_2_Pin GPIO_PIN_3
#define BTN_2_GPIO_Port GPIOB
#define BTN_3_Pin GPIO_PIN_4
#define BTN_3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
