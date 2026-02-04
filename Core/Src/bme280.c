/*
 * bme280.c
 *
 * Created on: Feb 3, 2026
 * Author: Bmbosima
 * Email:bmbosima@uwaterloo.ca
 */

#include "bme280.h"

// --- Private Variables ---
static I2C_HandleTypeDef *hi2c_bme; // Local pointer to the I2C handle
static int32_t t_fine;              // Global temperature value for pressure comp

// Calibration Data (Factory constants)(uniquie to my bme sensor so use your own data sheet values.
static struct {
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
} calib_data;


// --- Private Helper Functions (Compensation Math) ---
// These are 'static' so they cannot be called from main.c

static void BME280_ReadCalibration(void) {
	uint8_t calib[26];
	uint8_t hum_calib[7];

	// Read Temp/Pressure calibration
	HAL_I2C_Mem_Read(hi2c_bme, BME280_ADDR, 0x88, 1, calib, 24, HAL_MAX_DELAY);
	calib_data.dig_T1 = (calib[1] << 8) | calib[0];
	calib_data.dig_T2 = (calib[3] << 8) | calib[2];
	calib_data.dig_T3 = (calib[5] << 8) | calib[4];
	calib_data.dig_P1 = (calib[7] << 8) | calib[6];
	calib_data.dig_P2 = (calib[9] << 8) | calib[8];
	calib_data.dig_P3 = (calib[11] << 8) | calib[10];
	calib_data.dig_P4 = (calib[13] << 8) | calib[12];
	calib_data.dig_P5 = (calib[15] << 8) | calib[14];
	calib_data.dig_P6 = (calib[17] << 8) | calib[16];
	calib_data.dig_P7 = (calib[19] << 8) | calib[18];
	calib_data.dig_P8 = (calib[21] << 8) | calib[20];
	calib_data.dig_P9 = (calib[23] << 8) | calib[22];

	// Read Humidity calibration
	HAL_I2C_Mem_Read(hi2c_bme, BME280_ADDR, 0xA1, 1, &calib_data.dig_H1, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(hi2c_bme, BME280_ADDR, 0xE1, 1, hum_calib, 7, HAL_MAX_DELAY);
	calib_data.dig_H2 = (hum_calib[1] << 8) | hum_calib[0];
	calib_data.dig_H3 = hum_calib[2];
	calib_data.dig_H4 = (hum_calib[3] << 4) | (hum_calib[4] & 0x0F);
	calib_data.dig_H5 = (hum_calib[5] << 4) | (hum_calib[4] >> 4);
	calib_data.dig_H6 = hum_calib[6];
}

static float BME280_Compensate_T(int32_t adc_T) {
	int32_t var1, var2;
	var1 = ((((adc_T >> 3) - ((int32_t)calib_data.dig_T1 << 1))) * ((int32_t)calib_data.dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)calib_data.dig_T1)) * ((adc_T >> 4) - ((int32_t)calib_data.dig_T1))) >> 12) * ((int32_t)calib_data.dig_T3)) >> 14;
	t_fine = var1 + var2;
	return (t_fine * 5 + 128) >> 8;
}

static uint32_t BME280_Compensate_P(int32_t adc_P) {
	int64_t var1, var2, p;
	var1 = (int64_t)t_fine - 128000;
	var2 = var1 * var1 * (int64_t)calib_data.dig_P6;
	var2 = var2 + ((var1 * (int64_t)calib_data.dig_P5) << 17);
	var2 = var2 + (((int64_t)calib_data.dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data.dig_P3) >> 8) + ((var1 * (int64_t)calib_data.dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data.dig_P1) >> 33;
	if (var1 == 0) return 0;
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data.dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)calib_data.dig_P7) << 4);
	return (uint32_t)p;
}

static uint32_t BME280_Compensate_H(int32_t adc_H) {
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)calib_data.dig_H4) << 20) - (((int32_t)calib_data.dig_H5) * v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)calib_data.dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)calib_data.dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)calib_data.dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)calib_data.dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}

// --- Public Functions ---

void BME280_Init(I2C_HandleTypeDef *hi2c) {
	hi2c_bme = hi2c; // Assign the I2C handle

	// 1. Read Calibration
	BME280_ReadCalibration();

	// 2. Configure Humidity (ctrl_hum 0xF2)
	uint8_t hum_conf = 0x01; // Oversampling x1
	HAL_I2C_Mem_Write(hi2c_bme, BME280_ADDR, 0xF2, 1, &hum_conf, 1, HAL_MAX_DELAY);

	// 3. Configure Measurement (ctrl_meas 0xF4) -> Normal Mode
	uint8_t meas_conf = 0x27;
	HAL_I2C_Mem_Write(hi2c_bme, BME280_ADDR, 0xF4, 1, &meas_conf, 1, HAL_MAX_DELAY);

	// 4. Configure Config (config 0xF5) -> Standby 1000ms
	uint8_t config_conf = 0xA0;
	HAL_I2C_Mem_Write(hi2c_bme, BME280_ADDR, 0xF5, 1, &config_conf, 1, HAL_MAX_DELAY);
}

void BME280_ReadSensor(BME280_Data *data) {
	uint8_t raw_data[8];
	int32_t raw_temp, raw_press, raw_hum;

	// Burst Read 0xF7 to 0xFE
	HAL_I2C_Mem_Read(hi2c_bme, BME280_ADDR, 0xF7, 1, raw_data, 8, HAL_MAX_DELAY);

	raw_press = (raw_data[0] << 12) | (raw_data[1] << 4) | (raw_data[2] >> 4);
	raw_temp  = (raw_data[3] << 12) | (raw_data[4] << 4) | (raw_data[5] >> 4);
	raw_hum   = (raw_data[6] << 8) | raw_data[7];

	// Convert and store in the struct
	data->temperature_C = BME280_Compensate_T(raw_temp) / 100.0f;
	data->pressure_hPa = BME280_Compensate_P(raw_press) / 256.0f;
	data->humidity_pct = BME280_Compensate_H(raw_hum) / 1024.0f;
}
