/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Receiver Node (Board 2) - Blinks LED on CAN Message
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// User preferred temperature (°C). Below this = heater (blue), above = fan (green).
#define PREFERRED_TEMP_C        22.0f
// No motion for this many ms -> turn off heater/fan.
#define NO_MOTION_TIMEOUT_MS    30000U
// Blink period for heater/fan LED when active (ms).
#define ACTUATOR_BLINK_MS       500U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef   RxHeader;
uint8_t               RxData[8];

volatile uint8_t  system_armed = 0;           // 1 = armed, 0 = disarmed (arm button)
volatile uint32_t last_button_press = 0;      // Debounce for arm button

volatile float remote_temp = 0.0f;
volatile float remote_press = 0.0f;
volatile float remote_hum = 0.0f;
volatile uint8_t remote_door_open = 0;       // 1 = 'D' (open), 0 = 'C' (closed)
volatile uint8_t remote_pir_motion = 0;       // 1 = motion, 0 = no motion
volatile uint32_t last_motion_tick = 0;      // Last time we received PIR=1 (for timeout)

volatile uint32_t rx_count_103 = 0;
volatile uint32_t rx_count_104 = 0;

uint32_t last_heater_fan_blink = 0;         // For blinking blue/green when active
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  // --- 1. CONFIG FILTER (Accept All Messages) ---
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh = 0;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0;
  canfilterconfig.FilterMaskIdLow = 0;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 14;

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  // --- 2. START CAN MODULE ---
  HAL_CAN_Start(&hcan);

  // --- Initialize Buzzer State ---
    Buzzer_Off(); // Ensure silence on startup
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
      // Drain all pending CAN messages so we don't miss 0x104 when it follows 0x103
      while (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0) > 0)
      {
        HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);

        // Toggle Traffic LED
        HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);

        if (RxHeader.StdId == 0x103)
        {
            // --- DECODE MESSAGE 1 (Door + PIR + Temp), DLC 6 ---
            rx_count_103++;

            remote_door_open = (RxData[0] == 'D') ? 1 : 0;
            remote_pir_motion = (RxData[1] != 0) ? 1 : 0;
            if (remote_pir_motion) {
                last_motion_tick = HAL_GetTick();
            }
            memcpy((void *)&remote_temp, &RxData[2], 4);

            // Alarm: armed and (door open OR PIR motion) -> buzzer
            if (system_armed && (remote_door_open || remote_pir_motion)) {
                Buzzer_On();
            } else {
                Buzzer_Off();
            }
        }
        else if (RxHeader.StdId == 0x104)
        {
            // --- DECODE MESSAGE 2 (Pressure + Humidity) ---
            rx_count_104++;

            // 1. Pressure: sender sends in Pa (BMP280); convert to hPa for display
            memcpy((void *)&remote_press, &RxData[0], 4);
            remote_press /= 100.0f;   // Pa -> hPa (e.g. 146059 -> 1460.59)

            // 2. Handle Humidity (bytes 4-7)
            memcpy((void *)&remote_hum, &RxData[4], 4);
        }
      }

      // --- Heater/Fan actuators: only when motion recently; off after no-motion timeout ---
      uint32_t now = HAL_GetTick();
      // Require at least one PIR=1 ever (last_motion_tick != 0); else heater/fan stay off at boot
      uint8_t motion_active = (last_motion_tick != 0 && (now - last_motion_tick <= NO_MOTION_TIMEOUT_MS)) ? 1 : 0;

      if (!motion_active) {
          HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
      } else {
          if (remote_temp < PREFERRED_TEMP_C) {
              // Heater (blue): blink when cold
              if (now - last_heater_fan_blink >= ACTUATOR_BLINK_MS) {
                  HAL_GPIO_TogglePin(blue_led_GPIO_Port, blue_led_Pin);
                  last_heater_fan_blink = now;
              }
              HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
          } else if (remote_temp > PREFERRED_TEMP_C) {
              // Fan (green): blink when hot
              if (now - last_heater_fan_blink >= ACTUATOR_BLINK_MS) {
                  HAL_GPIO_TogglePin(green_led_GPIO_Port, green_led_Pin);
                  last_heater_fan_blink = now;
              }
              HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_RESET);
          } else {
              HAL_GPIO_WritePin(blue_led_GPIO_Port, blue_led_Pin, GPIO_PIN_RESET);
              HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);
          }
      }
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = ENABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARM_LED_Pin|GPIO_PIN_5|blue_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(green_led_GPIO_Port, green_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PUSH_BUTTON_Pin */
  GPIO_InitStruct.Pin = PUSH_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(PUSH_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARM_LED_Pin PA5 blue_led_Pin */
  GPIO_InitStruct.Pin = ARM_LED_Pin|GPIO_PIN_5|blue_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : green_led_Pin */
  GPIO_InitStruct.Pin = green_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(green_led_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ALARM_Pin */
  GPIO_InitStruct.Pin = ALARM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ALARM_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_1) // Verify it's OUR button (PA1)
    {
        // --- RTOS-Friendly Debounce ---
        uint32_t current_time = HAL_GetTick();

        // Only accept input if 200ms has passed since the last press
        if ((current_time - last_button_press) > 200)
        {
            // Toggle the Armed State
            system_armed = !system_armed;

            // Update the Red LED immediately
            if(system_armed) {
                HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_SET);
            } else {
                HAL_GPIO_WritePin(ARM_LED_GPIO_Port, ARM_LED_Pin, GPIO_PIN_RESET);
                // Also kill the buzzer immediately if we disarm
                Buzzer_Off();
            }

            last_button_press = current_time; // Reset timer
        }
    }
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
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
