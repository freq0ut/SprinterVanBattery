/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32l1xx_hal.h"

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
#define HEATER_EN_Pin GPIO_PIN_13
#define HEATER_EN_GPIO_Port GPIOC
#define UV_INPUT_Pin GPIO_PIN_14
#define UV_INPUT_GPIO_Port GPIOC
#define OV_INPUT_Pin GPIO_PIN_15
#define OV_INPUT_GPIO_Port GPIOC
#define _30A_CKT6_SNS_Pin GPIO_PIN_0
#define _30A_CKT6_SNS_GPIO_Port GPIOA
#define _30A_CKT5_SNS_Pin GPIO_PIN_1
#define _30A_CKT5_SNS_GPIO_Port GPIOA
#define _30A_CKT4_SNS_Pin GPIO_PIN_2
#define _30A_CKT4_SNS_GPIO_Port GPIOA
#define _30A_CKT3_SNS_Pin GPIO_PIN_3
#define _30A_CKT3_SNS_GPIO_Port GPIOA
#define _30A_CKT2_SNS_Pin GPIO_PIN_4
#define _30A_CKT2_SNS_GPIO_Port GPIOA
#define _30A_CKT1_SNS_Pin GPIO_PIN_5
#define _30A_CKT1_SNS_GPIO_Port GPIOA
#define _60A_CKT2_SNS_Pin GPIO_PIN_6
#define _60A_CKT2_SNS_GPIO_Port GPIOA
#define _60A_CKT1_SNS_Pin GPIO_PIN_7
#define _60A_CKT1_SNS_GPIO_Port GPIOA
#define _60A_CKT1_LED_Pin GPIO_PIN_1
#define _60A_CKT1_LED_GPIO_Port GPIOB
#define _60A_CKT2_LED_Pin GPIO_PIN_2
#define _60A_CKT2_LED_GPIO_Port GPIOB
#define UV_TO_INV_Pin GPIO_PIN_10
#define UV_TO_INV_GPIO_Port GPIOB
#define OV_TO_CHGR_Pin GPIO_PIN_11
#define OV_TO_CHGR_GPIO_Port GPIOB
#define GV_CLOSE_PULSE_Pin GPIO_PIN_12
#define GV_CLOSE_PULSE_GPIO_Port GPIOB
#define GV_OPEN_PULSE_Pin GPIO_PIN_13
#define GV_OPEN_PULSE_GPIO_Port GPIOB
#define NTC_PCBA_Pin GPIO_PIN_14
#define NTC_PCBA_GPIO_Port GPIOB
#define NTC_SHUNTS_Pin GPIO_PIN_15
#define NTC_SHUNTS_GPIO_Port GPIOB
#define _30A_CKT1_LED_Pin GPIO_PIN_8
#define _30A_CKT1_LED_GPIO_Port GPIOA
#define _30A_CKT2_LED_Pin GPIO_PIN_9
#define _30A_CKT2_LED_GPIO_Port GPIOA
#define _30A_CKT3_LED_Pin GPIO_PIN_10
#define _30A_CKT3_LED_GPIO_Port GPIOA
#define _30A_CKT4_LED_Pin GPIO_PIN_11
#define _30A_CKT4_LED_GPIO_Port GPIOA
#define _30A_CKT5_LED_Pin GPIO_PIN_12
#define _30A_CKT5_LED_GPIO_Port GPIOA
#define _30A_CKT6_LED_Pin GPIO_PIN_15
#define _30A_CKT6_LED_GPIO_Port GPIOA
#define DEBUG_LED_Pin GPIO_PIN_5
#define DEBUG_LED_GPIO_Port GPIOB
#define UV_INDICATE_LED_Pin GPIO_PIN_8
#define UV_INDICATE_LED_GPIO_Port GPIOB
#define OV_INDICATE_LED_Pin GPIO_PIN_9
#define OV_INDICATE_LED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
