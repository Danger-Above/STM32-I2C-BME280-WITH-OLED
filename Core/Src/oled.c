/*
 * oled.c
 *
 *  Created on: Jan 11, 2026
 *      Author: Adam
 */

#include "oled.h"

#define OLED_CMD_DISP_ON 0xAF
#define OLED_CMD_DISP_OFF 0xAE

HAL_StatusTypeDef oled_send_command(const struct oled *dev, uint8_t control_byte, uint8_t *commands,
									uint8_t number_of_commands)
{
	HAL_StatusTypeDef status = mem_write(dev->bus, dev->address, control_byte, commands, number_of_commands);
	return status;
}

void oled_clear(const struct oled *dev)
{
	uint8_t page = 0xB0; //page 0
	uint8_t column_low = 0x00;
	uint8_t column_high = 0x10; // (high[3:0] << 4) | low[3:0] = column 0
	uint8_t buff[133] = {0}; //CH1116 -> 132 bytes per page
	buff[0] = 0x40; //control byte

	for (uint8_t i = 0; i < 8; ++i)
	{
		oled_send_command(dev, 0x00, &page, 1); //set the current page number
		oled_send_command(dev, 0x00, &column_low, 1); //set lower nibble of column start addres
		oled_send_command(dev, 0x00, &column_high, 1); //set higher nibble of column start addres
		master_transmit(dev->bus, dev->address, buff, sizeof(buff));
		++page;
	}
}


//static HAL_StatusTypeDef oled_read_id(const struct oled *dev, uint8_t *id)
//{
//	HAL_StatusTypeDef status = mem_read(dev->bus, dev->address, BME280_REG_ID, id, 1);
//	return status;
//}

void oled_init(const struct i2c_bus *bus, const struct oled *dev)
{

}
