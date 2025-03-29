/*
 * app.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Mike
 */

#include <string.h>
#include "app.h"
#include "usart1.h"
#include "peer_exchange.h"
#include "host_command.h"

static uint8_t _reply[PACKET_CONTENT_MAX_LENGTH];

static void _on_version(const uint8_t * p_cmd, const uint16_t length)
{
	uint8_t i;

	_reply[0] = p_cmd[0];

	for(i = 0; i < (PACKET_CONTENT_MAX_LENGTH - 1) && i < strlen(APP_VERSION); i++)
	{
		_reply[i + 1] = APP_VERSION[i];
	}

	send_peer_message(_reply, i + 1);
}

static void _on_echo(const uint8_t * p_cmd, const uint16_t length)
{
	send_peer_message(p_cmd, length);
}

static void _on_get_gpios(const uint8_t * p_cmd, const uint16_t length)
{

}

static void _on_set_gpios(const uint8_t * p_cmd, const uint16_t length)
{

}

void on_host_command(const uint8_t * p_command, const uint16_t length)
{
	if(length == 0)
	{
		print_log("Error: empty host message in %s\r\n", __FILE__);
		return;
	}
	if(p_command == NULL)
	{
		print_log("Error: invalid buffer address in %s\r\n", __FILE__);
		return;
	}

	print_log("host cmd: %d, %d bytes\r\n", p_command[0], length);

	uint8_t host_command = p_command[0];
	switch(host_command)
	{
	case HOST_COMMAND_VERSION:
		_on_version(p_command, length);
		break;
	case HOST_COMMAND_ECHO:
		_on_echo(p_command, length);
		break;
	case HOST_COMMAND_GET_GPIOS:
		_on_get_gpios(p_command, length);
		break;
	case HOST_COMMAND_SET_GPIOS:
		_on_set_gpios(p_command, length);
		break;
	default:
		print_log("Error: unknown host command: %d in %s\r\n", host_command, __FILE__);
		break;
	}
}


void poll_app(void)
{

}
