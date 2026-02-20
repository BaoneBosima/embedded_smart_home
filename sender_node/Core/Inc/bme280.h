/*
 * bme280.h
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

// --- Debug Variables (add to live expressions/watch window) ---
extern volatile uint8_t  bme280_debug_raw_data[8];  // Raw bytes from 0xF7-0xFE
extern volatile int32_t  bme280_debug_raw_temp;     // 20-bit raw temperature
extern volatile int32_t  bme280_debug_raw_press;    // 20-bit raw pressure
extern volatile int32_t  bme280_debug_raw_hum;      // 16-bit raw humidity
extern volatile float    bme280_debug_temp_C;        // Compensated temp (°C)
extern volatile float    bme280_debug_press_hPa;    // Compensated pressure (hPa)
extern volatile float    bme280_debug_hum_pct;      // Compensated humidity (%)

#endif /* INC_BME280_H_ */
