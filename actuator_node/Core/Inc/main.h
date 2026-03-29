/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/*
 * SensorState_t — Consolidated view of all remote sensor data.
 * Currently unused (the implementation uses individual volatile globals
 * in freertos.c instead). Kept here for potential future refactoring
 * into a single mutex-protected struct.
 */
typedef struct {
	float temperature;
	float pressure;
	float humidity;
	uint8_t door_open;
	uint8_t pir_motion;
	uint32_t last_motion_tick;
	uint8_t system_armed;
} SensorState_t;
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
#define PUSH_BUTTON_Pin GPIO_PIN_1
#define PUSH_BUTTON_GPIO_Port GPIOA
#define PUSH_BUTTON_EXTI_IRQn EXTI1_IRQn
#define ARM_LED_Pin GPIO_PIN_4
#define ARM_LED_GPIO_Port GPIOA
#define green_led_Pin GPIO_PIN_7
#define green_led_GPIO_Port GPIOC
#define blue_led_Pin GPIO_PIN_9
#define blue_led_GPIO_Port GPIOA
#define ALARM_Pin GPIO_PIN_5
#define ALARM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
