/*
 * bme280.c
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#include "bme280.h"


HAL_StatusTypeDef bme280_read_id(struct bme280 *sensor, uint8_t *id)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, 0xD0, id, 1);
	return status;
}

HAL_StatusTypeDef bme280_read_raw(struct bme280 *sensor, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, 0xFA, data, 3);
	return status;
}

HAL_StatusTypeDef bme280_read_reg(struct bme280 *sensor, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, reg, data, 1);
	return status;
}

HAL_StatusTypeDef bme280_write(struct bme280 *sensor, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, reg, data, 1);
	return status;
}

HAL_StatusTypeDef bme280_soft_reset(struct bme280 *sensor)
{
	uint8_t data = 0xB6;
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, 0xE0, &data, 1);
	return status;
}
