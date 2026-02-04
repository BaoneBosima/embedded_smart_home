/*
 * pir.h
 *
 * Created on: Feb 4, 2026
 * Author: baone bosima
 */

#ifndef INC_PIR_H_
#define INC_PIR_H_

#include "main.h" // Gives us access to HAL_GPIO functions

// Define which pin we used (Easier to change later)
#define PIR_PORT GPIOA
#define PIR_PIN  GPIO_PIN_8

// State tracker
typedef enum {
    PIR_NO_MOTION = 0,
    PIR_MOTION_DETECTED = 1
} PIR_Status_t;

// Function Prototypes
void PIR_Init(void);
PIR_Status_t PIR_GetStatus(void);

#endif /* INC_PIR_H_ */
