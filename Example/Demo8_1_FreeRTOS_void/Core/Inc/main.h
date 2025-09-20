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
#include "stm32g4xx_hal.h"

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
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ESP32_IO9_Pin GPIO_PIN_14
#define ESP32_IO9_GPIO_Port GPIOC
#define ESP32_EN_Pin GPIO_PIN_15
#define ESP32_EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define USE_DEBUG
#define USE_INFO
#define USE_PRINT

#if defined(USE_DEBUG) || defined(USE_INFO) || defined(USE_PRINT)
extern uint8_t printfBuf[100];
#endif

#ifdef USE_DEBUG
#define LOG_DEBUG(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[DEBUG] %s:%d %s(): " format "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_DEBUG(format, ...)
#endif
#ifdef USE_INFO
#define LOG_INFO(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[INFO] %d %s(): " format "\n", __LINE__, __func__, ##__VA_ARGS__))
#else
#define LOG_INFO(format, ...)
#endif
#ifdef USE_PRINT
#define LOG_PRINT(format, ...) CDC_Transmit_FS((uint8_t *)printfBuf, sprintf((char*)printfBuf, "[PRINT] %s(): " format "\n", __func__, ##__VA_ARGS__))
#else
#define LOG_PRINT(format, ...)
#endif
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
