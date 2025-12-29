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

HAL_StatusTypeDef bme280_read_id(struct bme280 *sensor, uint8_t *id);
HAL_StatusTypeDef bme280_read_raw(struct bme280 *sensor, uint8_t *data);
HAL_StatusTypeDef bme280_read_reg(struct bme280 *sensor, uint8_t reg, uint8_t *data, uint8_t bytes);
HAL_StatusTypeDef bme280_write(struct bme280 *sensor, uint8_t reg, uint8_t *data);
HAL_StatusTypeDef bme280_soft_reset(struct bme280 *sensor);
int32_t bme280_calculate_temp(int32_t raw_temp, struct bme280_compensation_params *params, int32_t *t_fine);
void bme280_parse_compensation(uint8_t *raw_compensation, struct bme280_compensation_params *params);

#endif /* SRC_BME280_H_ */
