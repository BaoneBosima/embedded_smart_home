/*
 * pir.c
 *
 * Created on: Feb 4, 2026
 * Author: Baone Bosima
 */

#include "pir.h"

void PIR_Init(void) {
    // The GPIO Clock and Pin settings are already handled by main.c
    // because we set them up in the .ioc file.
    // We can add a startup delay here if the sensor needs "warmup" time (PIRs often take 30s).
}

PIR_Status_t PIR_GetStatus(void) {
    // Simply read the pin state
    if (HAL_GPIO_ReadPin(PIR_PORT, PIR_PIN) == GPIO_PIN_SET) {
        return PIR_MOTION_DETECTED;
    } else {
        return PIR_NO_MOTION;
    }
}
