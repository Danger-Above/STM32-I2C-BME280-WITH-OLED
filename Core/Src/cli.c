/*
 * cli.c
 *
 *  Created on: Nov 27, 2025
 *      Author: Adam
 */

#include "string.h"
#include "stdlib.h"
#include "cli.h"

#define CLI_LINE_MAX 64

static UART_HandleTypeDef *cli_huart = NULL;

static char cli_line_buffer[CLI_LINE_MAX];
static uint8_t cli_line_len = 0;

static volatile uint8_t rx_char;
static volatile uint8_t event_rx_char = 0;

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
    cli_sendln("=== Simple cli ===");
    cli_sendln("Type 'help' for command list");
    cli_sendln("");
}

void cli_on_rx_char(uint8_t c)
{
    rx_char = c;
    event_rx_char = 1;
}

uint8_t event_cli_line_ready(void)
{
	if (!event_rx_char)
	{
		return 0;
	}

	event_rx_char = 0;

	//eol handling
	if (rx_char == '\r' || rx_char == '\n')
	{
		cli_send("\r\n");
		//ignore empty lines
		if (cli_line_len == 0)
		{
			return 0;
		}
		//null terminate string
		cli_line_buffer[cli_line_len] = '\0';
		cli_line_len = 0;
		return 1;
	}
	//any other char
	if (cli_line_len < (CLI_LINE_MAX - 1))
	{
		cli_line_buffer[cli_line_len++] = (char)rx_char;
		char tmp[2] = {(char)rx_char, '\0'};
		cli_send(tmp);
	}
	return 0;
}

void cli_process_line(void)
{
	if (*cli_line_buffer == '\0')
	{
		cli_send("\r\n");
		return;
	}

	cli_send("Received: ");
	cli_sendln(cli_line_buffer);

	if (strcmp(cli_line_buffer, "help") == 0)
	{
		cli_sendln("Available commands:");
		cli_sendln(" help");
		cli_sendln(" ver");

	}
	else if (strcmp(cli_line_buffer, "ver") == 0)
	{
		cli_sendln("Firmware ver: 0.1");
	}
	else
	{
		cli_sendln("Unknown command!");
	}
}


