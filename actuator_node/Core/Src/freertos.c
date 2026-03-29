/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS task definitions for the Actuator Node
  *
  * This file contains:
  *   - RTOS object definitions (tasks, mutex, semaphore)
  *   - MX_FREERTOS_Init() which creates all RTOS objects
  *   - Four task functions:
  *       1. StartDefaultTask  — Arm/disarm toggle via button semaphore
  *       2. StartCanRxTask    — Polls CAN FIFO and updates shared sensor data
  *       3. StartAlarmTask    — Drives buzzer when armed + intrusion detected
  *       4. StartClimateTask  — Blinks heater/fan LEDs based on temperature
  *
  * Shared data between tasks is protected by sensorDataMutex.
  * The ISR-to-task handoff for the arm button uses armButtonSem.
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
#include <string.h>
#include "can.h"
#include "gpio.h"
#include "buzzer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//#define PREFERRED_TEMP_C        22.0f   /* Target room temperature in Celsius for climate control */
#define NO_MOTION_TIMEOUT_MS    30000U  /* 30 seconds: if no PIR motion within this window, disable HVAC LEDs */
#define ACTUATOR_BLINK_MS       500U    /* LED toggle period for heater/fan visual indicator */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* CAN handle defined in can.c — needed here to call HAL_CAN_GetRxMessage */
extern CAN_HandleTypeDef hcan;

/* CAN receive buffer — filled by HAL_CAN_GetRxMessage in the canRxTask */
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

/*
 * system_armed: Global flag toggled by defaultTask on button press.
 *   0 = disarmed (alarm cannot sound)
 *   1 = armed (alarm will sound on door/PIR trigger)
 * Declared volatile because it is written by one task (defaultTask) and
 * read by another (alarmTask). On ARM Cortex-M, single-byte reads/writes
 * are atomic, so no mutex is strictly needed for this flag.
 */
volatile uint8_t  system_armed = 0;

/*
 * Shared sensor data — updated by canRxTask, read by alarmTask and climateTask.
 * All accesses MUST be wrapped with osMutexAcquire/Release(sensorDataMutexHandle)
 * to prevent torn reads (especially the 4-byte floats, which are NOT atomic).
 */
volatile float    remote_temp       = 0.0f;  /* Temperature from BME280 (Celsius) */
volatile float    remote_press      = 0.0f;  /* Barometric pressure (hPa) */
volatile float    remote_hum        = 0.0f;  /* Relative humidity (%) */
volatile uint8_t  remote_door_open  = 0;     /* 1 = door open, 0 = closed */
volatile uint8_t  remote_pir_motion = 0;     /* 1 = motion detected */
volatile uint32_t last_motion_tick  = 0;     /* HAL_GetTick() timestamp of last motion event */

/* Debug counters: track how many messages of each CAN ID have been received */
volatile uint32_t rx_count_103 = 0;
volatile uint32_t rx_count_104 = 0;

volatile float target_temp = 22.0f;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for canRxTask */
osThreadId_t canRxTaskHandle;
const osThreadAttr_t canRxTask_attributes = {
  .name = "canRxTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for alarmTask */
osThreadId_t alarmTaskHandle;
const osThreadAttr_t alarmTask_attributes = {
  .name = "alarmTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for climateTask */
osThreadId_t climateTaskHandle;
const osThreadAttr_t climateTask_attributes = {
  .name = "climateTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for statusTxTask */
osThreadId_t statusTxTaskHandle;
const osThreadAttr_t statusTxTask_attributes = {
  .name = "statusTxTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sensorDataMutex */
osMutexId_t sensorDataMutexHandle;
const osMutexAttr_t sensorDataMutex_attributes = {
  .name = "sensorDataMutex"
};
/* Definitions for armButtonSem */
osSemaphoreId_t armButtonSemHandle;
const osSemaphoreAttr_t armButtonSem_attributes = {
  .name = "armButtonSem"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartCanRxTask(void *argument);
void StartAlarmTask(void *argument);
void StartClimateTask(void *argument);
void StartStatusTxTask(void *argument);

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
  /* creation of sensorDataMutex */
  sensorDataMutexHandle = osMutexNew(&sensorDataMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of armButtonSem — initial count 0: wait for button ISR, not a spurious wake at boot */
  armButtonSemHandle = osSemaphoreNew(1, 0, &armButtonSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of canRxTask */
  canRxTaskHandle = osThreadNew(StartCanRxTask, NULL, &canRxTask_attributes);

  /* creation of alarmTask */
  alarmTaskHandle = osThreadNew(StartAlarmTask, NULL, &alarmTask_attributes);

  /* creation of climateTask */
  climateTaskHandle = osThreadNew(StartClimateTask, NULL, &climateTask_attributes);

  /* creation of statusTxTask */
  statusTxTaskHandle = osThreadNew(StartStatusTxTask, NULL, &statusTxTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  defaultTask — Arm/Disarm button handler (deferred from ISR).
  *
  * This task spends most of its life blocked on the armButtonSem semaphore.
  * When the user presses the physical push button:
  *   1. EXTI ISR fires -> HAL_GPIO_EXTI_Callback in main.c
  *   2. Callback releases armButtonSem (after debounce check)
  *   3. This task wakes up, toggles system_armed, and updates the ARM LED
  *   4. If disarming, immediately silences the buzzer
  *
  * Blocking on a semaphore (instead of polling) means this task consumes
  * zero CPU while waiting — the scheduler runs other tasks instead.
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  for(;;)
  {
    /* Block indefinitely until the button ISR releases the semaphore */
    if (osSemaphoreAcquire(armButtonSemHandle, osWaitForever) == osOK)
    {
        system_armed = !system_armed;

        if(system_armed) {
            HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_RESET);
            Buzzer_Off();
        }
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartCanRxTask */
/**
  * @brief  canRxTask — CAN bus receive handler (highest priority task).
  *
  * Polls the CAN RX FIFO0 every 10ms for pending messages from the
  * Sender Node. Two message types are expected:
  *
  *   ID 0x103 (DLC=6): [Door(1B)] [PIR(1B)] [Temperature(4B float)]
  *     - Door byte: 'D' = open, 'C' = closed
  *     - PIR byte:  non-zero = motion detected
  *     - Temperature: IEEE 754 float copied byte-by-byte via memcpy
  *
  *   ID 0x104 (DLC=8): [Pressure(4B float)] [Humidity(4B float)]
  *
  * All shared sensor variables are updated under mutex protection to
  * prevent alarmTask or climateTask from reading partially-written data.
  * The PA5 "traffic LED" toggles on every received frame for visual
  * confirmation that CAN communication is alive.
  */
/* USER CODE END Header_StartCanRxTask */
void StartCanRxTask(void *argument)
{
  /* USER CODE BEGIN StartCanRxTask */
  for(;;)
  {
    /* Drain all pending messages from the hardware FIFO before sleeping */
    while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
    {
      HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
      HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

      osMutexAcquire(sensorDataMutexHandle, osWaitForever);

      if (RxHeader.StdId == 0x103)
      {
          rx_count_103++;
          remote_door_open  = (RxData[0] == 'D') ? 1 : 0;
          remote_pir_motion = (RxData[1] != 0)   ? 1 : 0;

          if (remote_pir_motion) {
              last_motion_tick = HAL_GetTick();
          }
          /*
           * memcpy is used instead of a direct float cast because the bytes
           * in RxData may not be aligned to a 4-byte boundary. Casting a
           * misaligned pointer to float* can cause a HardFault on Cortex-M.
           */
          memcpy((void *)&remote_temp, &RxData[2], 4);
      }
      else if (RxHeader.StdId == 0x104)
      {
          rx_count_104++;
          memcpy((void *)&remote_press, &RxData[0], 4);
          memcpy((void *)&remote_hum,   &RxData[4], 4);
      }
      else if (RxHeader.StdId == 0x200)
      {
          if (RxData[0] == 0x01) {
              system_armed = 1;
              HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_SET);
          } else if (RxData[0] == 0x02) {
              system_armed = 0;
              HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_RESET);
              Buzzer_Off();
          }
      }
      else if (RxHeader.StdId == 0x201)
      {
          memcpy((void *)&target_temp, &RxData[0], 4);
      }

      osMutexRelease(sensorDataMutexHandle);
    }
    /* Yield to lower-priority tasks; 10ms polling interval is fast enough
     * given the sender transmits every 500ms */
    osDelay(10);
  }
  /* USER CODE END StartCanRxTask */
}

/* USER CODE BEGIN Header_StartAlarmTask */
/**
  * @brief  alarmTask — Security alarm logic.
  *
  * Checks every 50ms whether the system is armed AND an intrusion is
  * detected (door open OR PIR motion). If both conditions are true,
  * the buzzer sounds. When conditions clear (or system is disarmed),
  * the buzzer is turned off.
  *
  * The local_* copies are taken under mutex to get a consistent snapshot,
  * then all decision logic runs outside the mutex to keep the critical
  * section as short as possible.
  *
  * Note: system_armed is read WITHOUT the mutex — it is a single byte
  * (atomic on ARM) and is only written by one task (defaultTask).
  * The buzzer is also controlled by defaultTask (immediate off on disarm),
  * so there is a benign race: worst case, the buzzer stays on for one
  * extra 50ms cycle before this task turns it off.
  */
/* USER CODE END Header_StartAlarmTask */
void StartAlarmTask(void *argument)
{
  /* USER CODE BEGIN StartAlarmTask */
  uint8_t local_door_open;
  uint8_t local_pir_motion;

  for(;;)
  {
    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    local_door_open  = remote_door_open;
    local_pir_motion = remote_pir_motion;
    osMutexRelease(sensorDataMutexHandle);

    if (system_armed && (local_door_open || local_pir_motion)) {
        Buzzer_On();
    } else {
        Buzzer_Off();
    }

    osDelay(50);
  }
  /* USER CODE END StartAlarmTask */
}

/* USER CODE BEGIN Header_StartClimateTask */
/**
  * @brief  climateTask — HVAC simulation via LEDs (lowest priority).
  *
  * Simulates a smart thermostat with occupancy awareness:
  *   - If no motion has been detected for 30 seconds, both HVAC LEDs
  *     are turned off (energy saving — nobody is in the room).
  *   - If motion is recent AND temp < 22C: blue LED blinks (heater ON)
  *   - If motion is recent AND temp > 22C: green LED blinks (fan/AC ON)
  *   - If temp == 22C exactly: both LEDs off (comfortable)
  *
  * The 500ms blink is software-timed using HAL_GetTick() rather than
  * osDelay(), because we want the 50ms polling loop to keep checking
  * for state changes (e.g., motion timeout) at a faster rate than
  * the visual blink rate.
  */
/* USER CODE END Header_StartClimateTask */
void StartClimateTask(void *argument)
{
  /* USER CODE BEGIN StartClimateTask */
  uint32_t last_heater_fan_blink = 0;
  float local_temp;
  float local_target_temp;
  uint32_t local_motion_tick;

  for(;;)
  {
    osMutexAcquire(sensorDataMutexHandle, osWaitForever);
    local_temp        = remote_temp;
    local_target_temp = target_temp;
    local_motion_tick = last_motion_tick;
    osMutexRelease(sensorDataMutexHandle);

    uint32_t now = HAL_GetTick();

    /*
     * Occupancy check: if last_motion_tick is still 0 (no motion ever received)
     * or more than 30 seconds have elapsed since the last PIR trigger,
     * consider the room unoccupied and shut off climate LEDs.
     */
    uint8_t motion_active = (local_motion_tick != 0 && (now - local_motion_tick <= NO_MOTION_TIMEOUT_MS)) ? 1 : 0;

    if (!motion_active) {
        HAL_GPIO_WritePin(blue_led_GPIO_Port,  blue_led_Pin,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
    } else {
        if (local_temp < local_target_temp) {
            /* Too cold -> blink blue (heater indicator) */
            if (now - last_heater_fan_blink >= ACTUATOR_BLINK_MS) {
                HAL_GPIO_TogglePin(blue_led_GPIO_Port, blue_led_Pin);
                last_heater_fan_blink = now;
            }
            HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
        } else if (local_temp > local_target_temp) {
            /* Too hot -> blink green (fan/AC indicator) */
            if (now - last_heater_fan_blink >= ACTUATOR_BLINK_MS) {
                HAL_GPIO_TogglePin(green_led_GPIO_Port, green_led_Pin);
                last_heater_fan_blink = now;
            }
            HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_RESET);
        } else {
            /* At target temperature — both off */
            HAL_GPIO_WritePin(blue_led_GPIO_Port,  blue_led_Pin,  GPIO_PIN_RESET);
            HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
        }
    }

    osDelay(50);
  }
  /* USER CODE END StartClimateTask */
}

/* USER CODE BEGIN Header_StartStatusTxTask */
/**
* @brief Function implementing the statusTxTask thread.
*/
/* USER CODE END Header_StartStatusTxTask */
void StartStatusTxTask(void *argument)
{
  /* USER CODE BEGIN StartStatusTxTask */
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[6];
  uint32_t TxMailbox;

  TxHeader.StdId = 0x105;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 6;
  TxHeader.TransmitGlobalTime = DISABLE;

  for(;;)
  {
      // Byte 0: Armed Status
      TxData[0] = system_armed;

      // Byte 1: Alarm Status (Sounding or not)
      TxData[1] = (system_armed && (remote_door_open || remote_pir_motion)) ? 1 : 0;

      // Bytes 2-5: Current Target Temperature
      osMutexAcquire(sensorDataMutexHandle, osWaitForever);
      memcpy(&TxData[2], (void *)&target_temp, 4);
      osMutexRelease(sensorDataMutexHandle);

      // Transmit the message
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0) {
          HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
      }

      // Wait 1 second before broadcasting again
      osDelay(1000);
  }
  /* USER CODE END StartStatusTxTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

