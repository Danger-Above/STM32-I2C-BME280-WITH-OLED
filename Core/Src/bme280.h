/*
 * bme280.h
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#ifndef SRC_BME280_H_
#define SRC_BME280_H_

#include "i2c_bus.h"

struct bme280
{
	uint8_t address;
	struct i2c_bus *bus;
};

//todo przeniesc do pliku .c
struct bme280_compensation_params
{
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
};

struct bme280_results
{
	uint32_t raw_press;
	uint32_t raw_temp;
	uint16_t raw_hum;

	int32_t temperature;
	uint32_t pressure;
	uint32_t humidity;
};

//todo static i const co trzeba

HAL_StatusTypeDef bme280_soft_reset(const struct bme280 *sensor);
void bme280_init(const struct bme280 *sensor, void (*delay_ms)(uint32_t), uint8_t *id);
void bme280_configure_ctrl_registers(const struct bme280 *sensor, void (*delay_ms)(uint32_t), uint8_t ctrl_hum, uint8_t ctrl_meas, uint8_t config);
void bme280_get_compensation_params(const struct bme280 *sensor, struct bme280_compensation_params *params);
void bme280_get_measurments(const struct bme280 *sensor, const struct bme280_compensation_params *params, struct bme280_results *results);
void bme280_results_to_string(const struct bme280_results *results, char *buff, uint16_t size);

#endif /* SRC_BME280_H_ */
