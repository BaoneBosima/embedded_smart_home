/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/*
 * MX_CAN_Init — Configures the bxCAN peripheral for 500 kbps communication.
 *
 * Baud rate calculation (must match the Sender Node exactly):
 *   APB1 clock = 36 MHz (HCLK 72 MHz / APB1 prescaler 2)
 *   CAN time quantum = 36 MHz / Prescaler(4) = 9 MHz
 *   Bits per CAN bit = Sync(1) + BS1(15) + BS2(2) = 18 TQ
 *   Baud = 9 MHz / 18 = 500 kbps
 *   Sampling point = (1 + 15) / 18 = 88.9%  (standard for high-speed CAN)
 */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;                         /* TQ clock = APB1(36MHz) / 4 = 9 MHz */
  hcan.Init.Mode = CAN_MODE_NORMAL;                /* Normal bus operation (not loopback or silent) */
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;           /* Resync jump width for clock drift tolerance */
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;               /* Propagation + Phase Segment 1 */
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;                /* Phase Segment 2 */
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;                   /* Auto-recover from Bus-Off state */
  hcan.Init.AutoWakeUp = ENABLE;                   /* Wake from sleep on bus activity */
  hcan.Init.AutoRetransmission = ENABLE;           /* Retry failed transmissions automatically */
  hcan.Init.ReceiveFifoLocked = DISABLE;           /* Overwrite oldest message if FIFO is full */
  hcan.Init.TransmitFifoPriority = DISABLE;        /* TX priority by message ID (not FIFO order) */
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/*
 * HAL_CAN_MspInit — Low-level hardware setup called automatically by HAL_CAN_Init().
 * "Msp" = MCU Support Package. This function handles:
 *   1. Enabling the CAN peripheral clock
 *   2. Configuring PA11 (CAN_RX) and PA12 (CAN_TX) as alternate-function pins
 *   3. Setting the CAN RX0 interrupt priority to 5 (safe for FreeRTOS API calls)
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspInit 0 */

  /* USER CODE END CAN_MspInit 0 */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;       /* Alternate Function Push-Pull (CAN peripheral drives the pin) */
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN;      /* AF9 maps these pins to the CAN controller */
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * CAN RX0 interrupt priority = 5: at the boundary of
     * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY. This is the minimum
     * priority (numerically) from which FreeRTOS API calls are allowed.
     * We don't call FreeRTOS APIs from the CAN IRQ handler, but keeping
     * it at 5 is safe and consistent with the EXTI button interrupt.
     */
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspInit 1 */

  /* USER CODE END CAN_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN)
  {
  /* USER CODE BEGIN CAN_MspDeInit 0 */

  /* USER CODE END CAN_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
  /* USER CODE BEGIN CAN_MspDeInit 1 */

  /* USER CODE END CAN_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
