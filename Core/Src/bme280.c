/*
 * bme280.c
 *
 *  Created on: Dec 19, 2025
 *      Author: Adam
 */

#include "bme280.h"
#include "stdio.h"

//macros for combining 8 bit registers into uint16_t and int16_t
#define U16(lsb, msb)  ((uint16_t)((msb << 8) | (lsb)))
#define S16(lsb, msb)  ((int16_t)((msb << 8) | (lsb)))

#define BME280_REG_RESET 0xE0
#define BME280_REG_MEAS_DATA 0xF7
#define BME280_REG_ID 0xD0
#define BME280_REG_CRTL_HUM 0xF2
#define BME280_REG_CRTL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_REG_CALIB_TP_START 0x88
#define BME280_REG_CALIB_H_START 0xE1


static HAL_StatusTypeDef bme280_read_raw(const struct bme280 *sensor, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, BME280_REG_MEAS_DATA, data, 8);
	return status;
}

static HAL_StatusTypeDef bme280_read_reg(const struct bme280 *sensor, uint8_t reg, uint8_t *data, uint8_t bytes)
{
	HAL_StatusTypeDef status = mem_read(sensor->bus, sensor->address, reg, data, bytes);
	return status;
}

static HAL_StatusTypeDef bme280_write(const struct bme280 *sensor, uint8_t reg, uint8_t *data)
{
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, reg, data, 1);
	return status;
}

static HAL_StatusTypeDef bme280_is_ready(const struct bme280 *sensor)
{
	HAL_StatusTypeDef status = is_device_ready(sensor->bus, sensor->address);
	return status;
}

//implementations of the BOSCH BME280 temperature, pressure, and humidity compensation formulas
static int32_t bme280_compensate_temp(int32_t raw_temp, const struct bme280_compensation_params *params, int32_t *t_fine)
{
	int32_t var1, var2, T;
	var1 = ((((raw_temp >> 3) - ((int32_t)params->dig_T1 << 1))) * ((int32_t)params->dig_T2)) >> 11;
	var2 = (((((raw_temp >> 4) - ((int32_t)params->dig_T1)) * ((raw_temp >> 4) - ((int32_t)params->dig_T1))) >> 12) *
			 ((int32_t)params->dig_T3)) >> 14;
	*t_fine = var1 + var2;
	T = (*t_fine * 5 + 128) >> 8;
	return T;
}

static uint32_t bme280_compensate_press(int32_t raw_press, const struct bme280_compensation_params *params, int32_t t_fine)
{
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)params->dig_P6;
	var2 = var2 + ((var1 * (int64_t)params->dig_P5) << 17);
	var2 = var2 + (((int64_t)params->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)params->dig_P3) >> 8) + ((var1 * (int64_t)params->dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)params->dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576 - raw_press;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)params->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)params->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)params->dig_P7) << 4);
	return (uint32_t)p;
}

static uint32_t bme280_compensate_hum(int32_t raw_hum, const struct bme280_compensation_params *params, int32_t t_fine)
{
	int32_t v_x1_u32r;
	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((raw_hum << 14) - (((int32_t)params->dig_H4) << 20) - (((int32_t)params->dig_H5) * v_x1_u32r)) +
					((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)params->dig_H6))  >> 10) * (((v_x1_u32r *
					((int32_t)params->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) *
					((int32_t)params->dig_H2) + 8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)params->dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r >> 12);
}

static void bme280_parse_compensation_params(uint8_t *raw_compensation, struct bme280_compensation_params *params)
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

static HAL_StatusTypeDef bme280_soft_reset(const struct bme280 *sensor)
{
	uint8_t reset_value = 0xB6;
	HAL_StatusTypeDef status = mem_write(sensor->bus, sensor->address, BME280_REG_RESET, &reset_value, 1);
	return status;
}

void bme280_init(const struct bme280 *sensor, void (*delay_ms)(uint32_t), HAL_StatusTypeDef *status)
{
	*status = bme280_is_ready(sensor);
	if (*status != HAL_OK)
	{
		return;
	}
	bme280_soft_reset(sensor);
	delay_ms(100);	//wait a few ms for the reset to complete
}

void bme280_configure_ctrl_registers(const struct bme280 *sensor, void (*delay_ms)(uint32_t), uint8_t ctrl_hum, uint8_t ctrl_meas, uint8_t config)
{
	  uint8_t new_ctrl_hum, new_config;
	  const uint8_t ctrl_hum_mask = 0b00000111;
	  const uint8_t config_mask = 0b00000010;

	  bme280_read_reg(sensor, BME280_REG_CRTL_HUM, &new_ctrl_hum, 1);
	  bme280_read_reg(sensor, BME280_REG_CONFIG, &new_config, 1);

	  //apply mask so the reserved bits are not modified
	  new_ctrl_hum = (new_ctrl_hum) | (ctrl_hum & ctrl_hum_mask);
	  new_config = (new_config) | (config  & config_mask);

	  bme280_write(sensor, BME280_REG_CRTL_HUM, &new_ctrl_hum);
	  bme280_write(sensor, BME280_REG_CRTL_MEAS, &ctrl_meas);
	  bme280_write(sensor, BME280_REG_CONFIG, &new_config);

	  delay_ms(100);	//wait a few ms for the first measurement to complete
}

void bme280_get_compensation_params(const struct bme280 *sensor, struct bme280_compensation_params *params)
{
	uint8_t raw_compensation[33] = {0};

	bme280_read_reg(sensor, BME280_REG_CALIB_TP_START, raw_compensation, 26);
	bme280_read_reg(sensor, BME280_REG_CALIB_H_START, &raw_compensation[26], 7);

	bme280_parse_compensation_params(raw_compensation, params);
}

void bme280_get_measurments(const struct bme280 *sensor, const struct bme280_compensation_params *params, struct bme280_results *results)
{
	  int32_t t_fine = 0;
	  uint8_t raw[8] = {0};

	  bme280_read_raw(sensor, raw);

	  results->raw_press = (raw[0] << 12) | (raw[1] << 4) | (raw[2] >> 4);	//0xF7, 0xF8, 0xF9[7:4]
	  results->raw_temp = (raw[3] << 12) | (raw[4] << 4) | (raw[5] >> 4);	//0xFA, 0xFB, 0xFC[7:4]
	  results->raw_hum = (raw[6] << 8) | raw[7]; 							//0xFD, 0xFE

	  results->temperature = bme280_compensate_temp(results->raw_temp, params, &t_fine);
	  results->pressure = bme280_compensate_press(results->raw_press, params, t_fine);
	  results->humidity = bme280_compensate_hum(results->raw_hum, params, t_fine);
}

void bme280_results_to_string(const struct bme280_results *results, char *buff, uint16_t size)
{
	int32_t temperature = results->temperature;
	uint32_t pressure = results->pressure;
	uint32_t humidity = results->humidity;

	char temperature_string[5];
	char pressure_string[5];
	char humidity_string[3];

	if (temperature < 0)
	{
		temperature = 0;
	}
	else if (temperature > 9999)
	{
		temperature = 9999;
	}
	snprintf(temperature_string, sizeof(temperature_string), "%02ld.%ld", (temperature / 100), ((temperature/10) % 10));

	pressure = pressure >> 8; //result in q24.8, turncating fractional part for simplicity
	if (pressure > 99999)
	{
		pressure = 99999;
	}
	snprintf(pressure_string, sizeof(pressure_string), "%04lu", (pressure / 100));

	humidity = humidity >> 10; //result in q22.10, turncating fractional part for simplicity
	if (humidity > 99)
	{
		humidity = 99;
	}
	snprintf(humidity_string, sizeof(humidity_string), "%02lu", humidity);

	snprintf(buff, size, "T:%s P:%s H:%s", temperature_string, pressure_string, humidity_string);
}
