/*
 * door.h
 *
 * Created on: Feb 4, 2026
 * Author: bmbosima
 */

#ifndef INC_DOOR_H_
#define INC_DOOR_H_

#include "main.h" // Gives access to HAL functions and pin names

// Define States for readability
typedef enum {
    DOOR_OPEN = 1,   // Logic 1 (High) = Magnet Away
    DOOR_CLOSED = 0  // Logic 0 (Low)  = Magnet Near
} Door_Status_t;

// Function Prototypes
Door_Status_t Door_GetStatus(void);

#endif /* INC_DOOR_H_ */
