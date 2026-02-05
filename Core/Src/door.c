/*
 * door.c
 *
 * Created on: Feb 4, 2026
 * Author: bmbosima
 */

#include "door.h"

Door_Status_t Door_GetStatus(void) {
    // If Pin is HIGH (1), the Magnet is DETECTED -> Door is CLOSED.
    // If Pin is LOW (0), the Magnet is GONE     -> Door is OPEN.

    if (HAL_GPIO_ReadPin(DOOR_SENSOR_GPIO_Port, DOOR_SENSOR_Pin) == GPIO_PIN_RESET) {
        return DOOR_OPEN;   // Pin is Low (0) -> Magnet is gone
    } else {
        return DOOR_CLOSED; // Pin is High (1) -> Magnet is near
    }
}
