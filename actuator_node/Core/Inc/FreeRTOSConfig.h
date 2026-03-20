/* USER CODE BEGIN Header */
/*
 * FreeRTOS Kernel V10.0.1
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * These parameters and more are described within the 'configuration' section of the
 * FreeRTOS API documentation available on the FreeRTOS.org web site.
 *
 * See http://www.freertos.org/a00110.html
 *----------------------------------------------------------*/

/* USER CODE BEGIN Includes */
/* Section where include file can be added */
/* USER CODE END Includes */

/* Ensure definitions are only used by the compiler, and not by the assembler. */
#if defined(__ICCARM__) || defined(__CC_ARM) || defined(__GNUC__)
  #include <stdint.h>
  extern uint32_t SystemCoreClock;
#endif
/* --- Core scheduling behaviour --- */
#define configUSE_PREEMPTION                     1     /* 1 = preemptive scheduler (higher-priority task interrupts lower) */
#define configSUPPORT_STATIC_ALLOCATION          1     /* Allow statically-allocated RTOS objects (required by CMSIS-RTOS V2) */
#define configSUPPORT_DYNAMIC_ALLOCATION         1     /* Allow heap-based RTOS object creation (osThreadNew, osMutexNew, etc.) */
#define configUSE_IDLE_HOOK                      0     /* No idle hook callback needed */
#define configUSE_TICK_HOOK                      0     /* No tick hook callback needed */
#define configCPU_CLOCK_HZ                       ( SystemCoreClock )  /* 72 MHz (HSI x PLL MUL9 / PREDIV1) */
#define configTICK_RATE_HZ                       ((TickType_t)1000)   /* 1 ms tick = 1000 Hz; osDelay(50) = 50ms */
#define configMAX_PRIORITIES                     ( 56 )               /* CMSIS-RTOS V2 maps its priority enum into this range */
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)      /* Idle task stack: 128 words = 512 bytes */
#define configTOTAL_HEAP_SIZE                    ((size_t)8192)       /* 8 KB FreeRTOS heap (used by heap_4.c allocator) */
#define configMAX_TASK_NAME_LEN                  ( 16 )
#define configUSE_TRACE_FACILITY                 1     /* Enable vTaskList / vTaskGetRunTimeStats for debugging */
#define configUSE_16_BIT_TICKS                   0     /* 0 = 32-bit tick counter (won't overflow for ~49 days at 1 kHz) */
#define configUSE_MUTEXES                        1     /* Enable mutex API (osMutexNew, etc.) */
#define configQUEUE_REGISTRY_SIZE                8     /* Max queues visible in a debugger's RTOS-aware view */
#define configUSE_RECURSIVE_MUTEXES              1     /* Allow same task to acquire a mutex multiple times */
#define configUSE_COUNTING_SEMAPHORES            1     /* Enable counting semaphores (our binary semaphore uses this) */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0     /* Use generic (portable) task selection, not CLZ instruction */

/* Co-routine definitions. */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          ( 2 )

/* Software timer definitions — FreeRTOS creates a hidden "timer service" task */
#define configUSE_TIMERS                         1     /* Enable software timer API */
#define configTIMER_TASK_PRIORITY                ( 2 ) /* Timer task priority (low — we don't use SW timers heavily) */
#define configTIMER_QUEUE_LENGTH                 10    /* Max pending timer commands before queue blocks */
#define configTIMER_TASK_STACK_DEPTH             256   /* Timer task stack in words (1024 bytes) */

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet            1
#define INCLUDE_uxTaskPriorityGet           1
#define INCLUDE_vTaskDelete                 1
#define INCLUDE_vTaskCleanUpResources       0
#define INCLUDE_vTaskSuspend                1
#define INCLUDE_vTaskDelayUntil             1
#define INCLUDE_vTaskDelay                  1
#define INCLUDE_xTaskGetSchedulerState      1
#define INCLUDE_xTimerPendFunctionCall      1
#define INCLUDE_xQueueGetMutexHolder        1
#define INCLUDE_uxTaskGetStackHighWaterMark 1
#define INCLUDE_eTaskGetState               1

/*
 * The CMSIS-RTOS V2 FreeRTOS wrapper is dependent on the heap implementation used
 * by the application thus the correct define need to be enabled below
 */
#define USE_FreeRTOS_HEAP_4

/*
 * --- INTERRUPT PRIORITY CONFIGURATION (Critical for Cortex-M + FreeRTOS) ---
 *
 * STM32F303 has 4 NVIC priority bits (16 levels: 0=highest, 15=lowest).
 * FreeRTOS needs two boundaries:
 *
 *   configLIBRARY_LOWEST_INTERRUPT_PRIORITY (15):
 *     The lowest possible priority. Used for PendSV and SysTick (kernel).
 *
 *   configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5):
 *     Any ISR that calls a FreeRTOS API function (like osSemaphoreRelease)
 *     MUST have a priority number >= 5 (i.e., numerically 5..15).
 *     ISRs with priority 0..4 are "above" FreeRTOS and MUST NOT call any
 *     FreeRTOS functions — but they will never be masked by critical sections,
 *     making them ultra-low-latency.
 *
 *   Our EXTI1 (button) ISR uses priority 5 — exactly at the boundary,
 *   which is legal and allows the osSemaphoreRelease() call in the callback.
 *
 * The << (8 - configPRIO_BITS) shift converts from the "library" priority
 * (0-15) to the hardware register format (upper 4 bits of an 8-bit field).
 */
#ifdef __NVIC_PRIO_BITS
 #define configPRIO_BITS         __NVIC_PRIO_BITS
#else
 #define configPRIO_BITS         4
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY 		( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* Normal assert() semantics without relying on the provision of an assert.h
header file. */
/* USER CODE BEGIN 1 */
#define configASSERT( x ) if ((x) == 0) {taskDISABLE_INTERRUPTS(); for( ;; );}
/* USER CODE END 1 */

/*
 * --- HANDLER NAME MAPPING ---
 * FreeRTOS internally defines its own SVC, PendSV, and SysTick handlers
 * with "vPort..."/"xPort..." names. These #defines map them to the CMSIS
 * standard names so the linker connects them to the correct vector table
 * entries without needing duplicate definitions in stm32f3xx_it.c.
 *
 * Because we use TIM6 (not SysTick) as the HAL timebase, there is no
 * conflict — SysTick is fully owned by FreeRTOS for its tick interrupt.
 */
#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

/* USER CODE BEGIN Defines */
/* Section where parameter definitions can be added (for instance, to override default ones in FreeRTOS.h) */
/* USER CODE END Defines */

#endif /* FREERTOS_CONFIG_H */
