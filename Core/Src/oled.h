/*
 * oled.h
 *
 *  Created on: Jan 11, 2026
 *      Author: Adam
 */

#ifndef SRC_OLED_H_
#define SRC_OLED_H_

#include "i2c_bus.h"

struct oled
{
	uint8_t address;
	struct i2c_bus *bus;
};

HAL_StatusTypeDef oled_send_command(const struct oled *dev, uint8_t control_byte, uint8_t *commands,
									uint8_t number_of_commands);
HAL_StatusTypeDef oled_send_data(const struct oled *dev, uint8_t *data, uint16_t size);

void oled_clear(const struct oled *dev);
void oled_init(const struct i2c_bus *bus, const struct oled *dev);
void oled_send_text(const struct oled *dev, const char *text, uint16_t size);
void oled_debug_text(const struct oled *dev);

#endif /* SRC_OLED_H_ */
