/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g0xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B_INPUT_Pin GPIO_PIN_0
#define B_INPUT_GPIO_Port GPIOA
#define A_INPUT_Pin GPIO_PIN_1
#define A_INPUT_GPIO_Port GPIOA
#define led_1_Pin GPIO_PIN_4
#define led_1_GPIO_Port GPIOA
#define led_2_Pin GPIO_PIN_5
#define led_2_GPIO_Port GPIOA
#define led_3_Pin GPIO_PIN_6
#define led_3_GPIO_Port GPIOA
#define led_4_Pin GPIO_PIN_7
#define led_4_GPIO_Port GPIOA
#define led_5_Pin GPIO_PIN_0
#define led_5_GPIO_Port GPIOB
#define led_7_Pin GPIO_PIN_1
#define led_7_GPIO_Port GPIOB
#define led_6_Pin GPIO_PIN_9
#define led_6_GPIO_Port GPIOA
#define TRIGGER_INPUT_Pin GPIO_PIN_10
#define TRIGGER_INPUT_GPIO_Port GPIOA
#define led_8_Pin GPIO_PIN_15
#define led_8_GPIO_Port GPIOA
#define MTR_RELAY_Pin GPIO_PIN_3
#define MTR_RELAY_GPIO_Port GPIOB
#define MTR_LED_STATS_Pin GPIO_PIN_4
#define MTR_LED_STATS_GPIO_Port GPIOB
#define led_10_Pin GPIO_PIN_6
#define led_10_GPIO_Port GPIOB
#define led_9_Pin GPIO_PIN_7
#define led_9_GPIO_Port GPIOB
#define FLT_SENSE_Pin GPIO_PIN_8
#define FLT_SENSE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
