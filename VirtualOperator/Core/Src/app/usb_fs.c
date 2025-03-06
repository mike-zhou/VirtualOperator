/*
 * usb_fs.c
 *
 *  Created on: Feb 23, 2025
 *      Author: Mike
 */

#include <usb_fs.h>

#include "circular_buffer.h"
#include "usbd_cdc_if.h"
#include "usart1.h"


#define USB_FS_SENDING_BUFFER_SIZE 0x10000
#define USB_FS_RECEIVING_BUFFER_SIZE 0x10000

static uint8_t _sending_buffer[USB_FS_SENDING_BUFFER_SIZE];
static uint8_t _receiving_buffer[USB_FS_RECEIVING_BUFFER_SIZE];

static CircularBuffer _cb_sending;
static CircularBuffer _cb_receiving;

static bool _usb_fs_initialized = false;

static bool _cb_receiving_overflow = false;
static bool _is_sending = false;


static void _start_sending()
{
	static uint8_t cache[256];
	static bool transmit_again = false;
	static uint32_t byte_count = 0;

	if(_is_sending)
	{
		return;
	}

	if(!transmit_again)
	{
		for(byte_count = 0; byte_count < sizeof(cache); byte_count++)
		{
			if(!cbuf_get(&_cb_sending, cache + byte_count))
				break;
		}
	}

	if(byte_count > 0)
	{
		_is_sending = true; // set before launching transmit to avoid race condition with on_usb_fs_sending_complete

		int8_t result = CDC_Transmit_FS(cache, byte_count);
		if(result == USBD_OK)
		{
			transmit_again = false;
		}
		else
		{
			transmit_again = true;
			_is_sending = false;
		}
	}
}


void on_usb_fs_receiving_complete(const uint8_t * buffer, uint32_t length)
{
	int i;

	for(i=0; i<length; i++)
	{
		if(!cbuf_put(&_cb_receiving, buffer[i]))
			break;
	}
	if(i!=length)
	{
		_cb_receiving_overflow = true;
	}
}


void on_usb_fs_sending_complete(const uint8_t * buffer, uint32_t length)
{
	_is_sending = false;

	_start_sending();
}


void init_usb_fs()
{
	cbuf_init(&_cb_sending, _sending_buffer, sizeof(_sending_buffer));
	cbuf_init(&_cb_receiving, _receiving_buffer, sizeof(_receiving_buffer));

	_usb_fs_initialized = true;
}

void poll_usb_fs()
{
	if(!_is_sending)
	{
		_start_sending();
	}

	if(_cb_receiving_overflow)
	{
		print_string("USB FS receiving overflow\r\n");
		_cb_receiving_overflow = false;
	}
}

bool get_usb_fs_byte(uint8_t * p_buffer)
{
	return cbuf_get(&_cb_receiving, p_buffer);
}

bool send_usb_fs_bytes(const uint8_t * p_buffer, const uint32_t length)
{
	if(NULL == p_buffer)
	{
		return false;
	}

	size_t capacity = cbuf_capacity(&_cb_sending);
	size_t data_count = cbuf_size(&_cb_sending);
	size_t free_space = capacity - data_count;
	//print_log("h: %d, t: %d, f:%d\r\n", _cb_sending.head, _cb_sending.tail, free_space);
	if(length > free_space)
	{
		return false;
	}

	uint32_t i;
	for(i=0; i<length; i++)
	{
		cbuf_put(&_cb_sending, p_buffer[i]);
	}

	return true;
}


