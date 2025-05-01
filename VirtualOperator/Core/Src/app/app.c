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

static uint16_t _get_gpio_mode(const GPIO_TypeDef * pGpio)
{
	uint32_t moder = pGpio->MODER;
	uint16_t mask = 0;

	for(int i = 0; i < 16; i++)
	{
		if((moder & 0x3) == 0x1)
		{
			// set bit to 1 if gpio output mode.
			mask |= (1 << i);
		}
		moder >>= 2;
	}

	return mask;
}

static void _on_get_gpio_mode(const uint8_t * p_cmd, const uint16_t length)
{
	uint16_t mask;

	_reply[0] = p_cmd[0];

	// little endian
	mask = _get_gpio_mode(GPIOA);
	_reply[1] = mask & 0xff;
	_reply[2] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOB);
	_reply[3] = mask & 0xff;
	_reply[4] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOC);
	_reply[5] = mask & 0xff;
	_reply[6] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOD);
	_reply[7] = mask & 0xff;
	_reply[8] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOE);
	_reply[9] = mask & 0xff;
	_reply[10] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOF);
	_reply[11] = mask & 0xff;
	_reply[12] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOG);
	_reply[13] = mask & 0xff;
	_reply[14] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOH);
	_reply[15] = mask & 0xff;
	_reply[16] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOI);
	_reply[17] = mask & 0xff;
	_reply[18] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOJ);
	_reply[19] = mask & 0xff;
	_reply[20] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOK);
	_reply[21] = mask & 0xff;
	_reply[22] = (mask >> 8) & 0xff;

	send_peer_message(_reply, 23);
}

static void _on_read_gpios(const uint8_t * p_cmd, const uint16_t length)
{
	uint16_t value;

	_reply[0] = p_cmd[0];

	// little endian
	value = GPIOA->IDR;
	_reply[1] = value & 0xff;
	_reply[2] = (value >> 8) & 0xff;

	value = GPIOB->IDR;
	_reply[3] = value & 0xff;
	_reply[4] = (value >> 8) & 0xff;

	value = GPIOC->IDR;
	_reply[5] = value & 0xff;
	_reply[6] = (value >> 8) & 0xff;

	value = GPIOD->IDR;
	_reply[7] = value & 0xff;
	_reply[8] = (value >> 8) & 0xff;

	value = GPIOE->IDR;
	_reply[9] = value & 0xff;
	_reply[10] = (value >> 8) & 0xff;

	value = GPIOF->IDR;
	_reply[11] = value & 0xff;
	_reply[12] = (value >> 8) & 0xff;

	value = GPIOG->IDR;
	_reply[13] = value & 0xff;
	_reply[14] = (value >> 8) & 0xff;

	value = GPIOH->IDR;
	_reply[15] = value & 0xff;
	_reply[16] = (value >> 8) & 0xff;

	value = GPIOI->IDR;
	_reply[17] = value & 0xff;
	_reply[18] = (value >> 8) & 0xff;

	value = GPIOJ->IDR;
	_reply[19] = value & 0xff;
	_reply[20] = (value >> 8) & 0xff;

	value = GPIOK->IDR;
	_reply[21] = value & 0xff;
	_reply[22] = (value >> 8) & 0xff;

	send_peer_message(_reply, 23);
}

static void _on_set_gpio(const uint8_t * p_cmd, const uint16_t length)
{
	_reply[0] = p_cmd[0];

	if((length % 3) != 1)
	{
		print_log("Error: wrong length of HOST_COMMAND_SET_GPIO: %d\r\n", length);
		send_peer_message(_reply, 1);
		return;
	}

	uint8_t portCount = (length - 1) / 3;
	for(uint8_t i=0; i<portCount; i++)
	{
		uint8_t portIndex = p_cmd[i*3+1];
		uint8_t bitIndex = 	p_cmd[i*3+2];
		uint8_t level = 	p_cmd[i*3+3];

		GPIO_TypeDef * pPort = NULL;

		switch(portIndex)
		{
		case 0:
			pPort = GPIOA;
			break;
		case 1:
			pPort = GPIOB;
			break;
		case 2:
			pPort = GPIOC;
			break;
		case 3:
			pPort = GPIOD;
			break;
		case 4:
			pPort = GPIOE;
			break;
		case 5:
			pPort = GPIOF;
			break;
		case 6:
			pPort = GPIOG;
			break;
		case 7:
			pPort = GPIOH;
			break;
		case 8:
			pPort = GPIOI;
			break;
		case 9:
			pPort = GPIOJ;
			break;
		case 10:
			pPort = GPIOK;
			break;
		default:
			break;
		}

		if(pPort == NULL)
		{
			print_log("Error: wrong port index (%d) in HOST_COMMAND_SET_GPIO\r\n", portIndex);
			send_peer_message(_reply, 1);
			return;
		}
		if(bitIndex > 15)
		{
			print_log("Error: wrong bit index (%d) in HOST_COMMAND_SET_GPIO\r\n", bitIndex);
			send_peer_message(_reply, 1);
			return;
		}
		if(level > 1)
		{
			print_log("Error: wrong level (%d) in HOST_COMMAND_SET_GPIO\r\n", level);
			send_peer_message(_reply, 1);
			return;
		}

		uint16_t mode = _get_gpio_mode(pPort);
		if((mode & (1 << bitIndex)) == 0)
		{
			print_log("Error: write read-only GPIO (%d) in HOST_COMMAND_SET_GPIO\r\n", bitIndex);
			send_peer_message(_reply, 1);
			return;
		}

		if(level == 0)
		{
			HAL_GPIO_WritePin(pPort, 1 << bitIndex, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(pPort, 1 << bitIndex, GPIO_PIN_SET);
		}

		_reply[i*3+1] = portIndex;
		_reply[i*3+2] = bitIndex;
		_reply[i*3+3] = level;
	}

	send_peer_message(_reply, length);
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
	case HOST_COMMAND_GET_GPIO_MODE:
		_on_get_gpio_mode(p_command, length);
		break;
	case HOST_COMMAND_READ_GPIOS:
		_on_read_gpios(p_command, length);
		break;
	case HOST_COMMAND_SET_GPIO:
		_on_set_gpio(p_command, length);
		break;
	default:
		print_log("Error: unknown host command: %d in %s\r\n", host_command, __FILE__);
		break;
	}
}


void poll_app(void)
{

}
