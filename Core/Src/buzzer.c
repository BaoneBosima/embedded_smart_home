/*
 * buzzer.c
 *
 * Created on: Feb 4, 2026
 * Author: bmbosima
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
