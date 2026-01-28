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

void oled_init(const struct oled *dev, HAL_StatusTypeDef *status);
void oled_send_text(const struct oled *dev, const char *text, uint16_t size);

#endif /* SRC_OLED_H_ */
