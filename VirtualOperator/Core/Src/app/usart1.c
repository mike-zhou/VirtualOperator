/*
 * usart1.c
 *
 *  Created on: Feb 21, 2025
 *      Author: Mike
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <usart1.h>

#include "circular_buffer.h"

#define SENDING_BUFFER_LENGTH 	0x10000
#define RECEIVING_BUFFER_LENGTH 0x1000

extern UART_HandleTypeDef huart1;

static uint8_t  _sending_buffer[SENDING_BUFFER_LENGTH];
static volatile bool _sending_done = true;

static uint8_t  _receiving_buffer[SENDING_BUFFER_LENGTH];
static volatile bool _receiving_started = false;
static uint8_t _receiving_cache;

static CircularBuffer _cbuffer_sending;
static CircularBuffer _cbuffer_receiving;
static bool cbuffer_initialized = false;

// this function can be called in interrupt or non-interrupt
static bool start_sending()
{
	static uint8_t cache[1024];
	uint32_t transfer_size;

	// retrieve bytes to send
	for(transfer_size = 0; transfer_size < sizeof(cache); transfer_size++)
	{
		if(cbuf_get(&_cbuffer_sending, cache + transfer_size) == false)
			break;
	}

	if(transfer_size)
	{
		// there is something to send out
		HAL_StatusTypeDef status;

		_sending_done = false; // change in advance to avoid being preempted if ISR finishes too soon.

		status = HAL_UART_Transmit_IT(&huart1, cache, transfer_size);
		if(status == HAL_OK)
		{
			return true;
		}
		else
		{
			// failed to start transfer
			_sending_done = true;
		}
	}

	return false;
}

static void start_receiving()
{
	HAL_StatusTypeDef status;

	_receiving_started = true;
	status = HAL_UART_Receive_IT(&huart1, &_receiving_cache, 1);
	if(status != HAL_OK)
	{
		_receiving_started = false;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	cbuf_put(&_cbuffer_receiving, _receiving_cache);

	start_receiving();
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	_sending_done = true;

	start_sending();
}


bool prepare_uart1()
{
	if(cbuf_init(&_cbuffer_sending, _sending_buffer, sizeof(_sending_buffer)) == false)
	{
		return false;
	}

	if(cbuf_init(&_cbuffer_receiving, _receiving_buffer, sizeof(_receiving_buffer)) == false)
	{
		return false;
	}

	cbuffer_initialized = true;

	return true;
}


bool poll_usart1()
{
	if(!cbuffer_initialized)
	{
		return false;
	}

	if(_sending_done)
	{
		start_sending();
	}

	if(!_receiving_started)
	{
		start_receiving();
	}

	return true;
}


static inline uint8_t get_ascii(uint8_t value)
{
	value &= 0xF;

	switch(value)
	{
		case 0:
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
		case 7:
		case 8:
		case 9:
			return '0' + value;
		default:
			return value - 10 + 'A';
	}
}

void print_uint8_hex(uint8_t value)
{
	uint8_t c;

	for(int i=1; i>-1; i--)
	{
		c = get_ascii(value >> (4 * i));
		if(cbuf_put(&_cbuffer_sending, c) == false)
			break;
	}
}

void print_uint16_hex(uint16_t val)
{
	uint8_t c;

	for(int i=3; i>-1; i--)
	{
		c = get_ascii(val >> (4 * i));
		if(cbuf_put(&_cbuffer_sending, c) == false)
			break;
	}
}

void print_uint32_hex(uint32_t val)
{
	uint8_t c;

	for(int i=7; i>-1; i--)
	{
		c = get_ascii(val >> (4 * i));
		if(cbuf_put(&_cbuffer_sending, c) == false)
			break;
	}
}

void print_char(char c)
{
	cbuf_put(&_cbuffer_sending, c);
}

void print_string(char * pStr)
{
	for(int i=0; i<strlen(pStr); i++)
	{
		if(cbuf_put(&_cbuffer_sending, pStr[i]) == false)
			break;
	}
}

void print_log(const char *format, ...)
{
	// 1. We need somewhere to store the formatted output:
    static char buffer[1024];  // or dynamically allocated, or thread-local

    // 2. Initialize a va_list to capture the variable arguments:
    va_list args;
    va_start(args, format);

    // 3. Use vsnprintf (safer than vsprintf) to format into 'buffer':
    //    vsnprintf returns the number of characters (excluding the null terminator).
    //    It will also ensure no buffer overflow happens (it will truncate if necessary).
    vsnprintf(buffer, sizeof(buffer), format, args);

    // 4. End usage of 'args'
    va_end(args);

    for(int i=0; i<strlen(buffer); i++)
    {
		if(cbuf_put(&_cbuffer_sending, buffer[i]) == false)
			break;
    }
}

bool print_complete_log(const char *format, ...)
{
	// 1. We need somewhere to store the formatted output:
    static char buffer[1024];  // or dynamically allocated, or thread-local
    int result;

    // 2. Initialize a va_list to capture the variable arguments:
    va_list args;
    va_start(args, format);

    // 3. Use vsnprintf (safer than vsprintf) to format into 'buffer':
    //    vsnprintf returns the number of characters (excluding the null terminator).
    //    It will also ensure no buffer overflow happens (it will truncate if necessary).
    result = vsnprintf(buffer, sizeof(buffer), format, args);

    // 4. End usage of 'args'
    va_end(args);

    if(result < 0)
    {
    	return false;
    }
    if(result >= sizeof(buffer))
    {
    	return false;
    }
    if(result > (cbuf_capacity(&_cbuffer_sending) - cbuf_size(&_cbuffer_sending)))
    {
    	return false;
    }

    for(int i=0; i<strlen(buffer); i++)
    {
		if(cbuf_put(&_cbuffer_sending, buffer[i]) == false)
		{
			return false;
		}
    }

    return true;
}


bool get_uart_char(unsigned char * pBuf)
{
	return cbuf_get(&_cbuffer_receiving, pBuf);
}
