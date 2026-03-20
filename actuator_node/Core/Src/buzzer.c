/*
 * buzzer.c
 *
 * Created on: Feb 4, 2026
 * Author: bmbosima
 *
 * Controls the piezo buzzer / alarm connected to PB5 (ALARM_Pin).
 * GPIO_PIN_SET = 3.3V on the pin = buzzer sounds.
 * GPIO_PIN_RESET = 0V = buzzer silent.
 *
 * Called from:
 *   - StartAlarmTask() in freertos.c  (Buzzer_On / Buzzer_Off)
 *   - StartDefaultTask() in freertos.c (Buzzer_Off on disarm)
 *   - main.c startup                   (Buzzer_Off for known initial state)
 */

#include "buzzer.h"

void Buzzer_On(void) {
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_SET);
}

void Buzzer_Off(void) {
    HAL_GPIO_WritePin(ALARM_GPIO_Port, ALARM_Pin, GPIO_PIN_RESET);
}

void Buzzer_Toggle(void) {
    HAL_GPIO_TogglePin(ALARM_GPIO_Port, ALARM_Pin);
}
