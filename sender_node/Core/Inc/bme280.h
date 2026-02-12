/*
 * bme280.c
 *
 *  Created on: Feb 3, 2026
 *      Author: bmbosima
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "main.h"  // Access to HAL library

// Device Address (Left shifted for HAL)
#define BME280_ADDR (0x76 << 1) // SDO to GND

// Struct to hold the final readable data
typedef struct {
	float temperature_C;
	float pressure_hPa;
	float humidity_pct;
} BME280_Data;

// --- Function Prototypes ---
// Call this once before the while loop
void BME280_Init(I2C_HandleTypeDef *hi2c);

// Call this inside the loop to get fresh data
void BME280_ReadSensor(BME280_Data *data);

#endif /* INC_BME280_H_ */
