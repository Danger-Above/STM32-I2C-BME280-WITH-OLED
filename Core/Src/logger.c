/*
 * cli.c
 *
 *  Created on: Nov 27, 2025
 *      Author: Adam
 */

#include "string.h"

#include "logger.h"

static UART_HandleTypeDef *cli_huart = NULL;

static void cli_send(const char *s)
{
	if (cli_huart == NULL) return;

	HAL_UART_Transmit(cli_huart, (uint8_t *)s, strlen(s), 100);
}

void cli_sendln(const char *s)
{
	cli_send(s);
	cli_send("\r\n");
}

void cli_init(UART_HandleTypeDef *huart)
{
    cli_huart = huart;

    cli_sendln("");
    cli_sendln("=== Logger for BME280 sensor ===");
    cli_sendln("");
}
