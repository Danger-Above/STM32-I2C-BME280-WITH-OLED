/*
 * cli.h
 *
 *  Created on: Nov 27, 2025
 *      Author: Adam
 */

#ifndef SRC_CLI_H_
#define SRC_CLI_H_

#include "stdint.h"
#include "stm32l4xx_hal.h"

void cli_sendln(const char *s);

void cli_init(UART_HandleTypeDef *huart);

void cli_on_rx_char(uint8_t c);

uint8_t event_cli_line_ready(void);

void cli_process_line(void);

#endif /* SRC_CLI_H_ */
