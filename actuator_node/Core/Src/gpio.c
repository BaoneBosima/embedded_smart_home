/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/*
 * MX_GPIO_Init — Configures all GPIO pins used by the Actuator Node.
 *
 * Pin map:
 *   PA1  — PUSH_BUTTON   (input,  EXTI falling-edge interrupt, internal pull-up)
 *   PA4  — ARM_LED        (output, indicates armed/disarmed state)
 *   PA5  — Traffic LED    (output, toggles on each received CAN frame)
 *   PA9  — blue_led       (output, heater indicator in climateTask)
 *   PC7  — green_led      (output, fan/AC indicator in climateTask)
 *   PB5  — ALARM buzzer   (output, driven by Buzzer_On/Off in alarmTask)
 *
 * All outputs start LOW (GPIO_PIN_RESET) so LEDs are off and buzzer is silent.
 */
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* Enable the clock for each GPIO port before configuring any pin on it */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* Set all output pins to LOW (known state) before configuring them as outputs */
  HAL_GPIO_WritePin(GPIOA, ARM_LED_Pin|GPIO_PIN_5|blue_led_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);

  /*
   * Push button: configured for falling-edge interrupt (press = HIGH->LOW).
   * Internal pull-up keeps the pin HIGH when the button is not pressed.
   * When pressed, the button connects the pin to GND -> falling edge -> EXTI fires.
   */
  GPIO_InitStruct.Pin = PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /* LED outputs on Port A: push-pull, no pull resistor, low speed (toggling at human-visible rates) */
  GPIO_InitStruct.Pin = ARM_LED_Pin|GPIO_PIN_5|blue_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* Green LED on Port C */
  GPIO_InitStruct.Pin = green_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(green_led_GPIO_Port, &GPIO_InitStruct);

  /* Buzzer/alarm on Port B */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /*
   * EXTI1 interrupt for the push button — priority 5, sub-priority 0.
   * Priority 5 = configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, so the
   * HAL_GPIO_EXTI_Callback can safely call osSemaphoreRelease().
   */
  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */
