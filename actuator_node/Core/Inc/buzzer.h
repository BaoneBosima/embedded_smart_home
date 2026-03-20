/*
 * buzzer.h
 *
 * Created on: Feb 4, 2026
 * Author: bmbosima
 *
 * Thin abstraction over the ALARM GPIO pin (PB5). Using named functions
 * instead of raw HAL_GPIO_WritePin calls makes the intent clearer in
 * task code (Buzzer_On vs. GPIO_PIN_SET) and centralizes the pin
 * assignment so it only needs to change in one place.
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "main.h"

void Buzzer_On(void);     /* Drive ALARM_Pin HIGH  — buzzer sounds */
void Buzzer_Off(void);    /* Drive ALARM_Pin LOW   — buzzer silent */
void Buzzer_Toggle(void); /* Flip ALARM_Pin state  — unused currently */

#endif /* INC_BUZZER_H_ */
