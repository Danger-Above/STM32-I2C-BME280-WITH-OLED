/*
 * i2c_bus.c
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#include "i2c_bus.h"

HAL_StatusTypeDef mem_read(struct i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Read(bus->handle, (address << 1), reg, I2C_MEMADD_SIZE_8BIT, data, size, bus->timeout);
	return status;
}

HAL_StatusTypeDef mem_write(struct i2c_bus *bus, uint8_t address, uint8_t reg, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_I2C_Mem_Write(bus->handle, (address << 1), reg, I2C_MEMADD_SIZE_8BIT, data, size, bus->timeout);
	return status;
}

HAL_StatusTypeDef master_transmit(struct i2c_bus *bus, uint8_t address, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(bus->handle, (address << 1), data, size, bus->timeout);
	return status;
}

HAL_StatusTypeDef is_device_ready(struct i2c_bus *bus, uint8_t address)
{
	HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(bus->handle, (address << 1), 3, bus->timeout);
	return status;
}
