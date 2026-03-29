/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Actuator Node (Board 2) - Environmental Monitor Receiver
  *
  * This is the entry point for the Actuator Node in a two-board environmental
  * monitoring system. This node:
  *   - Receives CAN messages from the Sender Node (BME280 + PIR + Door data)
  *   - Runs FreeRTOS with 4 tasks to handle button input, CAN reception,
  *     alarm control, and climate-based LED actuation
  *   - Uses TIM6 as the HAL timebase (instead of SysTick) because FreeRTOS
  *     takes ownership of SysTick for its own tick interrupt
  *
  * Startup sequence:
  *   HAL_Init() -> SystemClock_Config() -> GPIO/CAN peripheral init ->
  *   CAN filter + start -> osKernelInitialize() -> MX_FREERTOS_Init()
  *   (creates tasks/mutex/semaphore) -> osKernelStart() (never returns)
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "buzzer.h"
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

/* USER CODE BEGIN PV */
/*
 * last_button_press: Stores the tick count (ms) of the most recent valid
 * button press. Used for software debouncing — we reject any interrupt that
 * fires within 200ms of the previous one.
 * Declared volatile because it is written inside an ISR (HAL_GPIO_EXTI_Callback)
 * and could be read from task context.
 */
volatile uint32_t last_button_press = 0;

/*
 * armButtonSemHandle: A binary semaphore created in freertos.c.
 * We extern it here so the EXTI callback (ISR) can release it to wake
 * the defaultTask, which handles arming/disarming logic in task context.
 */
extern osSemaphoreId_t armButtonSemHandle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
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

  /*
   * --- CAN RECEIVE FILTER CONFIGURATION ---
   * The STM32 CAN peripheral has hardware filter banks that decide which
   * incoming message IDs get placed into the receive FIFO and which get
   * silently discarded. This is useful on a busy CAN bus with many nodes.
   *
   */
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 10;                       /* Which of the 14 filter banks to use (0-13) */
  canfilterconfig.FilterFIFOAssignment = CAN_RX_FIFO0;  /* Accepted messages go to FIFO0 */
  canfilterconfig.FilterIdHigh = 0;                      /* Upper 16 bits of the ID to match (0 = don't care) */
  canfilterconfig.FilterIdLow = 0;                       /* Lower 16 bits of the ID to match */
  canfilterconfig.FilterMaskIdHigh = 0;                  /* Upper 16 bits of the mask (0 = ignore all bits) */
  canfilterconfig.FilterMaskIdLow = 0;                   /* Lower 16 bits of the mask */
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;    /* Mask mode: (RxID & Mask) == (FilterID & Mask) */
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;   /* Use full 32-bit filter width */
  canfilterconfig.SlaveStartFilterBank = 14;             /* Not relevant for single-CAN devices */

  HAL_CAN_ConfigFilter(&hcan, &canfilterconfig);

  /* Start the CAN peripheral — transitions from Initialization to Normal mode. */
  HAL_CAN_Start(&hcan);

  /* Force the buzzer/alarm GPIO low so we start in a known silent state */
  Buzzer_Off();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {
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

/* USER CODE BEGIN 4 */
/*
 * HAL_GPIO_EXTI_Callback — Called by the HAL when any EXTI line fires.
 *
 * This is an Interrupt Service Routine (ISR) context.
 * Pattern used: "Deferred Interrupt Handling"
 *   ISR releases a binary semaphore -> FreeRTOS task wakes up and does
 *   the real work (toggling armed state, controlling LEDs/buzzer).
 *   This keeps the ISR fast and lets the scheduler manage priorities.
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == PUSH_BUTTON_Pin)
    {
        uint32_t current_time = HAL_GetTick();

        /*
         * Software debounce: ignore repeats within 200 ms of the last *accepted* press.
         * Do not compare against last_button_press == 0: (now - 0) > 200 would reject
         * every press until system time exceeds 200 ms, which feels like a dead button.
         */
        if (last_button_press != 0U && (current_time - last_button_press) <= 200U)
        {
            return;
        }

        (void)osSemaphoreRelease(armButtonSemHandle);
        last_button_press = current_time;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
