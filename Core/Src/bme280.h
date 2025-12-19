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

HAL_StatusTypeDef bme280_read_id(struct bme280 *sensor, uint8_t *id);
HAL_StatusTypeDef bme280_read_raw(struct bme280 *sensor, uint8_t *data);
HAL_StatusTypeDef bme280_write(struct bme280 *sensor, uint8_t reg, uint8_t *data);

#endif /* SRC_BME280_H_ */
