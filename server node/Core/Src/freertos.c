/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "https_server.h"
#include "can.h"
#include <string.h>
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
/* USER CODE BEGIN Variables */
/* Shared sensor data -- written by canRxTask , read by httpServerTask */
volatile float dash_temp = 0.0f;
volatile float dash_press = 0.0f;
volatile float dash_hum = 0.0f;
volatile uint8_t dash_door_open = 0;
volatile uint8_t dash_pir_motion = 0;
volatile uint8_t dash_armed = 0;
volatile uint8_t dash_alarm = 0;
volatile float dash_target_temp = 22.0f;

CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];
uint32_t              RxMailbox;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for httpsServerTask */
osThreadId_t httpsServerTaskHandle;
const osThreadAttr_t httpsServerTask_attributes = {
  .name = "httpsServerTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for statusLedTask */
osThreadId_t statusLedTaskHandle;
const osThreadAttr_t statusLedTask_attributes = {
  .name = "statusLedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for dashDataMutex */
osMutexId_t dashDataMutexHandle;
const osMutexAttr_t dashDataMutex_attributes = {
  .name = "dashDataMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCanRxTask(void *argument);
void StartHttpsServerTask(void *argument);
void StartStatusLedTask(void *argument);

extern void MX_LWIP_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of dashDataMutex */
  dashDataMutexHandle = osMutexNew(&dashDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(StartCanRxTask, NULL, &canRxTask_attributes);

  /* creation of httpsServerTask */
  httpsServerTaskHandle = osThreadNew(StartHttpsServerTask, NULL, &httpsServerTask_attributes);

  /* creation of statusLedTask */
  statusLedTaskHandle = osThreadNew(StartStatusLedTask, NULL, &statusLedTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
* @brief Function implementing the canRxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void *argument)
{
  /* USER CODE BEGIN StartCanRxTask */
	for (;;)
	  {
	    /* 1. Drain the CAN FIFO:
	       We use a 'while' loop instead of an 'if' statement to ensure we process
	       ALL waiting messages before going back to sleep. */
	    while (HAL_CAN_GetRxFifoFillLevel(&hcan1, CAN_RX_FIFO0) > 0)
	    {
	      // Retrieve the next message from the hardware buffer into RxData
	      HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData);

	      /* 2. Lock the Mutex:
	         We lock the shared variables so the httpServerTask cannot read
	         them while we are in the middle of updating them.  */
	      osMutexAcquire(dashDataMutexHandle, osWaitForever);

	      /* 3. Decode based on the Message ID */
	      if (RxHeader.StdId == 0x103)
	      {
	        // Message 0x103 (From Sender): Door, Motion, and Temperature [cite: 173]
	        dash_door_open = (RxData[0] == 'D') ? 1 : 0;
	        dash_pir_motion = (RxData[1] != 0) ? 1 : 0;

	        // We use memcpy because a float is 4 bytes long, and CAN data is 1 byte per slot
	        memcpy((void *)&dash_temp, &RxData[2], 4);
	      }
	      else if (RxHeader.StdId == 0x104)
	      {
	        // Message 0x104 (From Sender): Pressure and Humidity
	        memcpy((void *)&dash_press, &RxData[0], 4);
	        memcpy((void *)&dash_hum, &RxData[4], 4);
	      }
	      else if (RxHeader.StdId == 0x105)
	      {
	        // Message 0x105 (From Actuator): Armed status, Alarm status, Target Temp
	        dash_armed = RxData[0];
	        dash_alarm = RxData[1];
	        memcpy((void *)&dash_target_temp, &RxData[2], 4);
	      }

	      /* 4. Unlock the Mutex:
	         Release the lock immediately so the web server can safely read the fresh data.  */
	      osMutexRelease(dashDataMutexHandle);
	    }

	    /* 5. Yield to Scheduler:
	       Sleep for 10ms. This uses zero CPU power and allows the web server
	       to handle incoming browser connections.  */
	    osDelay(10);
	  }
  /* USER CODE END StartCanRxTask */
}

/* USER CODE BEGIN Header_StartHttpsServerTask */
/**
* @brief Function implementing the httpsServerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartHttpsServerTask */
void StartHttpsServerTask(void *argument)
{
  /* USER CODE BEGIN StartHttpsServerTask */
	http_server_init(); // This starts the infinite Netconn loop!
  /* USER CODE END StartHttpsServerTask */
}

/* USER CODE BEGIN Header_StartStatusLedTask */
/**
* @brief Function implementing the statusLedTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStatusLedTask */
void StartStatusLedTask(void *argument)
{
  /* USER CODE BEGIN StartStatusLedTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartStatusLedTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

// Bring in the CAN1 handle from main.c so we can transmit
extern CAN_HandleTypeDef hcan1;

/* ------------------------------------------------------------------------ */
/* SEND ARM COMMAND: Transmits 0x200 with 0x01 (Arm) or 0x02 (Disarm)       */
/* ------------------------------------------------------------------------ */
void send_arm_command(uint8_t cmd) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[1];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x200;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 1;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = cmd; /* 0x01 = ARM, 0x02 = DISARM */

    // Check if the CAN hardware has room to send a message
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}

/* ------------------------------------------------------------------------ */
/* SEND TARGET TEMP: Transmits 0x201 with the new float temperature         */
/* ------------------------------------------------------------------------ */
void send_target_temp(float temp) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[4];
    uint32_t TxMailbox;

    TxHeader.StdId = 0x201;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 4;
    TxHeader.TransmitGlobalTime = DISABLE;

    // Safely copy the 4 bytes of the float into the 4 bytes of the CAN payload
    memcpy(&TxData[0], &temp, 4);

    // Check if the CAN hardware has room to send a message
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
        HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
    }
}

/* USER CODE END Application */

