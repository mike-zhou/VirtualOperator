/*
 * cmd_and_reply.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Mike
 */

#include <string.h>
#include <peer_exchange.h>
#include "usart1.h"


#define MAX_PACKET_LENGTH 256
#define OUTPUT_BUFFER_COUNT 16

// maximum milliseconds between 2 bytes in a packet in input channel
#define PACKET_RECEIVING_TIMEOUT 10
// maximum milliseconds that an output packet must be acknowledged in output channel
#define PACKET_ACK_TIMEOUT 20

/**
 * Packet is in the format:
 * struct
 * {
 * 		uint8_t tag,
 * 		uint8_t length,
 * 		uint8_t sequence_number,
 * 		uint8_t value[length - 1]
 * }
 *
 * Tag 0xDD is for data packet
 * Tag 0xAA is for ACK packet
 */

#define SEQUENCE_NUMBER_INITIAL 0
#define SEQUENCE_NUMBER_INVALID 255

enum ChannelState
{
	IDLE = 0,
	DATA_PACKET,
	ACK_PACKET,
	WAIT_ACK
};

static struct InputChannel
{
	enum ChannelState state;
	uint8_t buffer[MAX_PACKET_LENGTH];
	uint8_t data_count;
	uint16_t timestamp;
	uint8_t sequence_number_received;
	uint8_t ack_buffer[3];
} _input_channel;


static struct OutputChannel
{
	enum ChannelState state;
	uint8_t buffer_array[OUTPUT_BUFFER_COUNT][MAX_PACKET_LENGTH + 1];
	uint16_t head, tail;
	uint16_t timestamp;
	uint8_t sequence_number;
} _output_channel;

static bool (* _get_byte_from_peer)(uint8_t * const p_bype);
static bool (* _send_bytes_to_peer)(const uint8_t * p_buffer, const uint16_t length);
static uint16_t (* _get_timestamp_ms)(void);
static void (* _on_peer_message)(const uint8_t * p_buffer, const uint16_t length);

static inline uint8_t next_sequence_number(uint8_t current_sequence_number)
{
	current_sequence_number++;
	if(current_sequence_number == SEQUENCE_NUMBER_INVALID)
	{
		return 1; // skip SEQUENCE_NUMBER_INITIAL
	}
	return current_sequence_number;
}

static void _on_peer_reset()
{
	print_log("Warning: peer reset in %s\r\n", __FILE__);
	_output_channel.state = IDLE; // don't wait for ACK in case of WAIT_ACK state
}

static void _send_ack_to_peer(const uint8_t * p_ack, const uint8_t length)
{
	if(length != sizeof(_input_channel.ack_buffer))
	{
		print_log("Error: wrong ACK packet length: %d in %s\r\n", length, __FILE__);
		return;
	}

	_send_bytes_to_peer(p_ack, length);
}

static void _on_ack_from_peer(const uint8_t sequence_number)
{
	if(sequence_number == SEQUENCE_NUMBER_INVALID)
	{
		print_log("Warning: invalid sequence number in ACK packet in %s\r\n", __FILE__);
		return;
	}
	if(_output_channel.state != WAIT_ACK)
	{
		print_log("Warning: ACK is not expected, output channel state: %d in %s\r\n", _output_channel.state, __FILE__);
		return;
	}
	if(sequence_number != _output_channel.sequence_number)
	{
		print_log("Warning: wrong ACK sequence_number: %d, expected: %d in %s\r\n", sequence_number, _output_channel.sequence_number, __FILE__);
		return;
	}

	_output_channel.state = IDLE;
	_output_channel.head = (_output_channel.head + 1) % OUTPUT_BUFFER_COUNT;
	_output_channel.sequence_number = next_sequence_number(_output_channel.sequence_number);
}

static inline uint16_t timestamp_abs(uint16_t current_time, uint16_t previous_time)
{
	uint32_t time = current_time + 0x10000;
	return (time - previous_time) & 0xFFFF;
}

static inline void _process_input_channel()
{
	uint8_t byte;
	uint16_t current_timestamp = _get_timestamp_ms();
	bool update_timestamp = false; //update to true if any packet byte is received.

	if(_input_channel.state != IDLE)
	{
		if(timestamp_abs(current_timestamp, _input_channel.timestamp) >= PACKET_RECEIVING_TIMEOUT)
		{
			print_log("Warning: timeout in %s in %s\r\n", __FUNCTION__, __FILE__);
			_input_channel.state = IDLE;
		}
	}

	for(;;)
	{
		switch(_input_channel.state)
		{
			case IDLE:
			{
				if(!_get_byte_from_peer(&byte))
				{
					return; // no input in IDLE state
				}
				switch(byte)
				{
					case 0xDD:
					{
						_input_channel.buffer[0] = byte;
						_input_channel.data_count = 1;
						_input_channel.state = DATA_PACKET;
						update_timestamp = true;
					}
					break;
					case 0xAA:
					{
						_input_channel.buffer[0] = byte;
						_input_channel.data_count = 1;
						_input_channel.state = ACK_PACKET;
						update_timestamp = true;
					}
					break;
					default:
					{
						update_timestamp = false;
						// ignore byte which is not Tag.
					}
					break;
				}
			}
			break;
			case DATA_PACKET:
			{
				if(!_get_byte_from_peer(&byte))
				{
					if(update_timestamp)
					{
						_input_channel.timestamp = current_timestamp;
					}
					return;
				}

				// save the byte
				update_timestamp = true;
				_input_channel.buffer[_input_channel.data_count] = byte;
				_input_channel.data_count += 1;

				if(_input_channel.data_count == 2)
				{
					const uint8_t length = _input_channel.buffer[1];
					if(length == 0)
					{
						// content error, ignore current packet
						print_log("Error: data packet length is 0 in %s\r\n", __FILE__);
						_input_channel.state = IDLE;
					}
				}
				else if(_input_channel.data_count > 2)
				{
					// check if a complete packet is received
					const uint8_t length = _input_channel.buffer[1];
					const uint8_t sequence_number = _input_channel.buffer[2];

					if(SEQUENCE_NUMBER_INVALID == sequence_number)
					{
						// error in packet content, ignore current packet
						print_log("Error: invalid sequence number in data packet in %s\r\n", __FILE__);
						_input_channel.state = IDLE;
					}
					else if(length == (_input_channel.data_count - 2))
					{
						// a complete packet is received.
						_input_channel.ack_buffer[2] = sequence_number;
						_send_ack_to_peer(_input_channel.ack_buffer, sizeof(_input_channel.ack_buffer));

						if(sequence_number == _input_channel.sequence_number_received)
						{
							// this packet has just been received, then get ready to receive next packet
							_input_channel.state = IDLE;
						}
						else
						{
							if(SEQUENCE_NUMBER_INITIAL == sequence_number)
							{
								_on_peer_reset();
							}
							_on_peer_message(_input_channel.buffer + 3, length - 1);
							_input_channel.sequence_number_received = sequence_number;
							_input_channel.state = IDLE; // get ready to receive next packet
						}
					}
					else if(length > (_input_channel.data_count - 2))
					{
						// either length or data_count is wrongly changed, which shouldn't happen
						print_log("Error: either length (%d) or data_count (%d) is wrong in data packet receiving in %s\r\n", length, _input_channel.data_count, __FILE__);
						_input_channel.state = IDLE;
					}
				}
			}
			break;
			case ACK_PACKET:
			{
				if(!_get_byte_from_peer(&byte))
				{
					if(update_timestamp)
					{
						_input_channel.timestamp = current_timestamp;
					}
					return;
				}

				// save the byte
				update_timestamp = true;
				_input_channel.buffer[_input_channel.data_count] = byte;
				_input_channel.data_count += 1;

				if(_input_channel.data_count == 2)
				{
					const uint8_t length = _input_channel.buffer[1];
					if(length != 1)
					{
						// content error, ignore current packet
						print_log("Error: wrong length (%d) in ACK packet in %s\r\n", length, __FILE__);
						_input_channel.state = IDLE;
					}
				}
				else if(_input_channel.data_count == 3)
				{
					const uint8_t sequence_number = _input_channel.buffer[2];
					if(SEQUENCE_NUMBER_INVALID != sequence_number)
					{
						_on_ack_from_peer(sequence_number);
					}
					else
					{
						// program shouldn't get here
						print_log("Error: invalid sequence number in ACK packet in %s\r\n", __FILE__);
					}
					_input_channel.state = IDLE;
				}
				else
				{
					// program shouldn't get here
					print_log("Error: wrong data_count (%d) in ACP packet in %s\r\n", _input_channel.data_count, __FILE__);
					_input_channel.state = IDLE;
				}
			}
			break;
			default:
				print_log("Error: wrong input_channel state (%d) in %s\r\n", _input_channel.state, __FILE__);
				_input_channel.state = IDLE;
				return;
		}
	}
}

static inline void _process_output_channel()
{
	if(IDLE == _output_channel.state)
	{
		if((_output_channel.head >= OUTPUT_BUFFER_COUNT) ||
				(_output_channel.tail >= OUTPUT_BUFFER_COUNT))
		{
			print_log("Error: output buffer index wrong, head: %d, tail: %d, in %s\r\n", _output_channel.head, _output_channel.tail, __FILE__);
			_output_channel.state = IDLE;
			_output_channel.head = 0;
			_output_channel.tail = 0;
			_output_channel.sequence_number = SEQUENCE_NUMBER_INITIAL;
			return;
		}
		if(_output_channel.head == _output_channel.tail)
		{
			return; // no data to send out
		}

		uint8_t * p_buf = _output_channel.buffer_array[_output_channel.head] + 1;
		uint8_t length = _output_channel.buffer_array[_output_channel.head][0];
		if(!_send_bytes_to_peer(p_buf, length))
		{
			// fail to send data to peer
			return;
		}
		else
		{
			_output_channel.state = WAIT_ACK;
			_output_channel.timestamp = _get_timestamp_ms();
		}
	}
	else if(WAIT_ACK == _output_channel.state)
	{
		uint16_t current_timestamp = _get_timestamp_ms();

		if(timestamp_abs(current_timestamp, _output_channel.timestamp) < PACKET_ACK_TIMEOUT)
		{
			// keep waiting for ACK
			return;
		}

		uint8_t * p_buf = _output_channel.buffer_array[_output_channel.head] + 1;
		uint8_t length = _output_channel.buffer_array[_output_channel.head][0];
		if(!_send_bytes_to_peer(p_buf, length))
		{
			// fail to send data to peer again
			return;
		}
		else
		{
			// update time stamp
			_output_channel.timestamp = current_timestamp;
		}
	}
	else
	{
		print_log("Error: wrong output channel state (%d) in %s\r\n", __FILE__);
		_output_channel.state = IDLE;
	}
}

bool init_peer_exchange(
		bool (*get_byte_from_peer)(uint8_t * const p_bype), // receive a byte from the peer
		bool (*send_bytes_to_peer)(const uint8_t * p_buffer, const uint16_t length), // send bytes to the peer
		uint16_t (*get_timestamp_ms)(void), // get time stamp
		void (*on_peer_message)(const uint8_t * p_buffer, const uint16_t length) // function which processes a peer message
		)
{
	if((NULL == get_byte_from_peer) ||
			(NULL == send_bytes_to_peer) ||
			(NULL == get_timestamp_ms) ||
			(NULL == on_peer_message))
	{
		print_log("Error: invalid parameter in %s in %s\r\n", __FUNCTION__, __FILE__);
		return false;
	}

	_get_byte_from_peer = get_byte_from_peer;
	_send_bytes_to_peer =send_bytes_to_peer;
	_get_timestamp_ms = get_timestamp_ms;
	_on_peer_message = on_peer_message;

	_input_channel.state = IDLE;
	_input_channel.data_count = 0;
	_input_channel.sequence_number_received = SEQUENCE_NUMBER_INVALID;
	_input_channel.ack_buffer[0] = 0xAA;
	_input_channel.ack_buffer[1] = 1;

	_output_channel.state = IDLE;
	_output_channel.head = 0;
	_output_channel.tail = 0;
	_output_channel.sequence_number = SEQUENCE_NUMBER_INITIAL;

	return true;
}

bool send_peer_message(const uint8_t * p_buf, const uint16_t length)
{
	const uint16_t new_tail = (_output_channel.tail + 1) % OUTPUT_BUFFER_COUNT;
	if(new_tail == _output_channel.head)
	{
		print_log("Error: no available buffer for new message in %s\r\n", __FILE__);
		return false;
	}

	if(length > MAX_PACKET_LENGTH)
	{
		print_log("Error: message is too long: %d, in %s\r\n", length, __FILE__);
		return false;
	}

	_output_channel.buffer_array[_output_channel.tail][0] = (uint8_t)length;
	memcpy(_output_channel.buffer_array[_output_channel.tail] + 1, p_buf, length);
	_output_channel.tail = new_tail;

	return true;
}

void poll_peer_exchange()
{
	_process_input_channel();
	_process_output_channel();
}
