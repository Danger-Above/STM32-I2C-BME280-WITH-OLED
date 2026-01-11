/*
 * i2c_bus.h
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#ifndef SRC_I2C_BUS_H_
#define SRC_I2C_BUS_H_

#include "stm32l4xx_hal.h"

struct i2c_bus
{
	I2C_HandleTypeDef *handle;
	uint16_t timeout;
};

HAL_StatusTypeDef mem_read(struct i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size);
HAL_StatusTypeDef mem_write(struct i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size);
HAL_StatusTypeDef master_transmit(struct i2c_bus *bus, uint8_t address, uint8_t *data, uint16_t size);

#endif /* SRC_I2C_BUS_H_ */
