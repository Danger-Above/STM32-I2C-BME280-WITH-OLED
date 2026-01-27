/*
 * oled.c
 *
 *  Created on: Jan 11, 2026
 *      Author: Adam
 */

#include "oled.h"
#include "font6x8.h"

#define OLED_CMD_DISP_ON 0xAF
#define OLED_CMD_DISP_OFF 0xAE
#define OLED_ADDR_PAGE_0 0xB0

HAL_StatusTypeDef oled_send_command(const struct oled *dev, uint8_t control_byte, uint8_t *commands,
									uint8_t number_of_commands)
{
	HAL_StatusTypeDef status = mem_write(dev->bus, dev->address, control_byte, commands, number_of_commands);
	return status;
}

HAL_StatusTypeDef oled_send_data(const struct oled *dev, uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef status = master_transmit(dev->bus, dev->address, data, size);
	return status;
}

void oled_set_pointer_home(const struct oled *dev)
{
	uint8_t page = OLED_ADDR_PAGE_0; //page 0
	uint8_t column_low = 0x04; 	//offset because of 4 invisible bytes
	uint8_t column_high = 0x10; // (high[3:0] << 4) | low[3:0] = column 0
	oled_send_command(dev, 0x00, &page, 1); //set the current page number
	oled_send_command(dev, 0x00, &column_low, 1); //set lower nibble of column start addres
	oled_send_command(dev, 0x00, &column_high, 1); //set higher nibble of column start addres
}

void oled_clear(const struct oled *dev)
{
	uint8_t buff[133] = {0}; //CH1116 -> 132 bytes per page
	uint8_t page = OLED_ADDR_PAGE_0; //page 0
	uint8_t column_low = 0x00;	//no offset to clear bytes both visible and invisible on display
	uint8_t column_high = 0x10; // (high[3:0] << 4) | low[3:0] = column 0

	buff[0] = 0x40; //control byte as first buffer byte

	for (uint8_t i = 0; i < 8; ++i)
	{
		oled_send_command(dev, 0x00, &page, 1); //set the current page number
		oled_send_command(dev, 0x00, &column_low, 1); //set lower nibble of column start addres
		oled_send_command(dev, 0x00, &column_high, 1); //set higher nibble of column start addres
		oled_send_data(dev, buff, sizeof(buff));
		++page;
	}
}

//todo send text on any page
void oled_send_text(const struct oled *dev, const char *text, uint16_t size)
{
	uint8_t buff[129] = {0};
	uint16_t character_width = sizeof(font6x8_ascii[0]);
	uint16_t buffer_pointer = 1; //point to the first non control byte

	buff[0] = 0x40; //control byte as first buffer byte

	if (size > ((sizeof(buff) - 1) / character_width))
	{
		return;
	}

	oled_set_pointer_home(dev);

	for (uint16_t i = 0; i < size; ++i)
	{
		uint8_t ascii_value = (uint8_t)text[i];

		if (ascii_value >= 128)
		{
			ascii_value = '?';
		}

		for (uint8_t j = 0; j < character_width; ++j)
		{
			buff[buffer_pointer] = font6x8_ascii[ascii_value][j];
			++buffer_pointer;
		}
	}
	oled_send_data(dev, buff, buffer_pointer);
}

void oled_debug_text(const struct oled *dev)
{
	oled_set_pointer_home(dev);

	uint8_t buff[7] = {0x00, 0x01, 0x01, 0x06, 0x29, 0x29, 0x1E}; //CH1116 -> 132 bytes per page
	buff[0] = 0x40; //control byte

	oled_send_data(dev, buff, 7);
}

//static HAL_StatusTypeDef oled_read_id(const struct oled *dev, uint8_t *id)
//{
//	HAL_StatusTypeDef status = mem_read(dev->bus, dev->address, BME280_REG_ID, id, 1);
//	return status;
//}

void oled_init(const struct i2c_bus *bus, const struct oled *dev)
{

}
