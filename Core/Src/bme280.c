/*
 * bme280.c
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#include "bme280.h"

//macros for combining 8 bit registers into uint16_t and int16_t
#define U16(lsb, msb)  ((uint16_t)((msb << 8) | (lsb)))
#define S16(lsb, msb)  ((int16_t)((msb << 8) | (lsb)))


HAL_StatusTypeDef bme280_read_id(struct bme280 *sensor, uint8_t *id)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, 0xD0, id, 1);
	return status;
}

HAL_StatusTypeDef bme280_read_raw(struct bme280 *sensor, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, 0xF7, data, 8);
	return status;
}

HAL_StatusTypeDef bme280_read_reg(struct bme280 *sensor, uint8_t reg, uint8_t *data, uint8_t bytes)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, reg, data, bytes);
	return status;
}

HAL_StatusTypeDef bme280_write(struct bme280 *sensor, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, reg, data, 1);
	return status;
}

HAL_StatusTypeDef bme280_soft_reset(struct bme280 *sensor)
{
	uint8_t reset_value = 0xB6;
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, 0xE0, &reset_value, 1);
	return status;
}

int32_t bme280_calculate_temp(int32_t raw_temp, struct bme280_compensation_params *params, int32_t *t_fine)
{
	int32_t var1, var2, T;

	var1 = ((((raw_temp >> 3) - ((int32_t)params->dig_T1 << 1))) * ((int32_t)params->dig_T2)) >> 11;
	var2 = (((((raw_temp >> 4) - ((int32_t)params->dig_T1)) * ((raw_temp >> 4) - ((int32_t)params->dig_T1))) >> 12) *
			 ((int32_t)params->dig_T3)) >> 14;

	*t_fine = var1 + var2;

	T = (*t_fine * 5 + 128) >> 8;
	return T;
}

void bme280_parse_compensation(uint8_t *raw_compensation, struct bme280_compensation_params *params)
{
	params->dig_T1 = U16(raw_compensation[0], raw_compensation[1]);		//0x88, 0x89
	params->dig_T2 = S16(raw_compensation[2], raw_compensation[3]);		//0x8A, 0x8B
	params->dig_T3 = S16(raw_compensation[4], raw_compensation[5]);		//0x8C, 0x8D

	params->dig_P1 = U16(raw_compensation[6], raw_compensation[7]);		//0x8E, 0x8F
	params->dig_P2 = S16(raw_compensation[8], raw_compensation[9]);		//0x90, 0x91
	params->dig_P3 = S16(raw_compensation[10], raw_compensation[11]);	//0x92, 0x93
	params->dig_P4 = S16(raw_compensation[12], raw_compensation[13]);	//0x94, 0x95
	params->dig_P5 = S16(raw_compensation[14], raw_compensation[15]);	//0x96, 0x97
	params->dig_P6 = S16(raw_compensation[16], raw_compensation[17]);	//0x98, 0x99
	params->dig_P7 = S16(raw_compensation[18], raw_compensation[19]);	//0x9A, 0x9B
	params->dig_P8 = S16(raw_compensation[20], raw_compensation[21]);	//0x9C, 0x9D
	params->dig_P9 = S16(raw_compensation[22], raw_compensation[23]);	//0x9E, 0x9F

	//raw_compensation[24] -> 0xA0 reserved register
	params->dig_H1 = raw_compensation[25];								//0xA1
	params->dig_H2 = S16(raw_compensation[26], raw_compensation[27]);	//0xE1, 0xE2
	params->dig_H3 = raw_compensation[28];								//0xE3

	// 0xE4, 0xE5[3:0]
	params->dig_H4 = (int16_t)((raw_compensation[29] << 4) | (raw_compensation[30] & 0x0F));
	// 0xE5[7:4], 0xE6
	params->dig_H5 = (int16_t)((raw_compensation[31] << 4) | (raw_compensation[30] >> 4));

	params->dig_H6 = raw_compensation[32]; 								//0xE7
}
