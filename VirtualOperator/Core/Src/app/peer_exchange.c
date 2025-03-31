/*
 * cmd_and_reply.c
 *
 *  Created on: Mar 4, 2025
 *      Author: Mike
 */

#include <string.h>
#include "peer_exchange.h"
#include "usart1.h"

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
 * 		struct
 * 		{
 * 			uint8_t sequence_number,
 * 			uint8_t value[length - 2]
 * 			uint8_t ending
 * 		} value
 * }
 *
 * Tag 0xDD is for data packet
 * Tag 0xAA is for ACK packet
 * Tag 0xEE is for packet end
 */

#define SEQUENCE_NUMBER_INITIAL 0
#define SEQUENCE_NUMBER_INVALID 255

#define ACK_TAG 	0xAA
#define DATA_TAG 	0xDD
#define END_TAG 	0xEE

enum InputChannelState
{
	TAG = 0,
	LENGTH,
	VALUE
};

enum OutputChannelState
{
	IDLE = 0,
	WAIT_ACK
};


static struct InputChannel
{
	enum InputChannelState state;
	uint8_t buffer[PACKET_MAX_LENGTH];
	uint8_t data_count;
	uint16_t timestamp;
	uint8_t sequence_number_received;
} _input_channel;


static struct OutputChannel
{
	enum OutputChannelState state;
	uint8_t buffer_array[OUTPUT_BUFFER_COUNT][PACKET_MAX_LENGTH];
	uint16_t head, tail;
	uint16_t timestamp;
	uint8_t sequence_number;
} _output_channel;

static bool (* _get_byte_from_peer)(uint8_t * const p_bype);
static bool (* _send_bytes_to_peer)(const uint8_t * p_buffer, const uint16_t length);
static uint16_t (* _get_timestamp_ms)(void);
static void (* _on_peer_message)(const uint8_t * p_buffer, const uint16_t length);

static inline uint8_t _next_sequence_number(uint8_t current_sequence_number)
{
	current_sequence_number++;
	if(current_sequence_number == SEQUENCE_NUMBER_INVALID)
	{
		return 1;
	}
	return current_sequence_number;
}

static void _ack_peer(uint8_t sequence_number)
{
	uint8_t buffer[4] = {ACK_TAG, 2, sequence_number, END_TAG};

	// print_log("PeerExchange: Send ACK to peer, sequence_number: %d\r\n", sequence_number);
	_send_bytes_to_peer(buffer, sizeof(buffer));
}

static void _on_ack(const uint8_t sequence_number)
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
	_output_channel.sequence_number = _next_sequence_number(_output_channel.sequence_number);
}

static inline uint16_t _timestamp_abs(uint16_t current_time, uint16_t previous_time)
{
	uint32_t time = current_time + 0x10000;
	return (time - previous_time) & 0xFFFF;
}

static inline bool _check_packet_validity(const uint8_t * p_packet, const uint8_t length)
{
	if(length < 4)
	{
		return false; // less than an empty packet
	}
	if((p_packet[0] != ACK_TAG) && (p_packet[0] != DATA_TAG))
	{
		return false;
	}
	if(p_packet[length - 1] != END_TAG)
	{
		return false;
	}
	if(p_packet[2] == SEQUENCE_NUMBER_INVALID)
	{
		return false;
	}
	if(p_packet[1] > (PACKET_MAX_LENGTH - 2))
	{
		return false;
	}
	if(p_packet[1] != (length - 2))
	{
		return false;
	}

	return true;
}

static void _on_data(const uint8_t * p_packet, const uint8_t length)
{
	// a complete packet is received.
	uint8_t sequence_number = p_packet[2];
	_ack_peer(sequence_number);

	if(sequence_number == _input_channel.sequence_number_received)
	{
		// this packet has just been received
		return;
	}

	_on_peer_message(_input_channel.buffer + 3, length - 4);
	_input_channel.sequence_number_received = sequence_number;
}

static void _process_input_channel()
{
	uint8_t byte;
	uint16_t current_timestamp = _get_timestamp_ms();
	bool valid_byte_received = false; //update to true if any packet byte is received.

	if(_input_channel.state != TAG)
	{
		// check if timeout in input
		if(_timestamp_abs(current_timestamp, _input_channel.timestamp) >= PACKET_RECEIVING_TIMEOUT)
		{
			print_log("Warning: PeerExchange: input timeout in %s in %s\r\n", __FUNCTION__, __FILE__);
			_input_channel.state = TAG;
		}
	}

	for(;;)
	{
		switch(_input_channel.state)
		{
			case TAG:
			{
				if(!_get_byte_from_peer(&byte))
				{
					return; // no input in TAG state
				}
				switch(byte)
				{
					case DATA_TAG:
					case ACK_TAG:
					{
						_input_channel.buffer[0] = byte;
						_input_channel.data_count = 1;
						_input_channel.state = LENGTH;
						valid_byte_received = true;
					}
					break;
					default:
					{
						valid_byte_received = false;
						// ignore byte which is not Tag.
					}
					break;
				}
			}
			break;

			case LENGTH:
			{
				if(!_get_byte_from_peer(&byte))
				{
					if(valid_byte_received)
					{
						_input_channel.timestamp = current_timestamp;
					}
					return;
				}

				if((byte == 0) || (byte > (PACKET_MAX_LENGTH - 2)))
				{
					// invalid length
					_input_channel.state = TAG;
					continue;
				}

				// save the byte
				valid_byte_received = true;
				_input_channel.buffer[1] = byte;
				_input_channel.data_count = 2;
				_input_channel.state = VALUE;
			}
			break;

			case VALUE:
			{
				if(!_get_byte_from_peer(&byte))
				{
					if(valid_byte_received)
					{
						_input_channel.timestamp = current_timestamp;
					}
					return;
				}

				valid_byte_received = true;
				_input_channel.buffer[_input_channel.data_count] = byte;
				_input_channel.data_count++;

				if(_input_channel.data_count < (_input_channel.buffer[1] + 2))
				{
					continue; // incomplete packet
				}

				// a complete packet is received
				uint8_t * p_packet = _input_channel.buffer;
				if(_check_packet_validity(p_packet, _input_channel.data_count))
				{
					if(p_packet[0] == ACK_TAG)
					{
						// print_log("PeerExchange: ACK packet arrives, sequence_number: %d\r\n", p_packet[2]);
						_on_ack(p_packet[2]);
					}
					else if(p_packet[0] == DATA_TAG)
					{
						// print_log("PeerExchange: DATA packet arrives, sequence_number: %d, %d bytes\r\n", p_packet[2], _input_channel.data_count);
						_on_data(p_packet, _input_channel.data_count);
					}
					else
					{
						print_log("Error: PeerExchange: unknown packet arrives, TAG: %d\r\n", p_packet[0]);
					}
				}
				else
				{
					print_log("Error: PeerExchange: invalid packet arrives\r\n");
				}

				_input_channel.state = TAG;
			}
			break;

			default:
				print_log("Error: PeerExchange: wrong input_channel state (%d) in %s\r\n", _input_channel.state, __FILE__);
				_input_channel.state = TAG;
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
			print_log("Error: PeerExchange: output buffer index wrong, head: %d, tail: %d, in %s\r\n", _output_channel.head, _output_channel.tail, __FILE__);
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

		uint8_t * p_buf = _output_channel.buffer_array[_output_channel.head];
		uint8_t packet_length = p_buf[1] + 2;
		p_buf[2] = _output_channel.sequence_number;
		// print_log("PeerExchange: Send packet to peer, sequence_number: %d, %d bytes\r\n", _output_channel.sequence_number, packet_length);
		if(!_send_bytes_to_peer(p_buf, packet_length))
		{
			// fail to send data to peer
			print_log("Error: PeerExchange: failed in sending packet to peer\r\n");
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

		if(_timestamp_abs(current_timestamp, _output_channel.timestamp) < PACKET_ACK_TIMEOUT)
		{
			// keep waiting for ACK
			return;
		}

		uint8_t * p_buf = _output_channel.buffer_array[_output_channel.head];
		uint8_t packet_length = p_buf[1] + 2;
		// print_log("PeerExchange: Re-send packet to peer, sequence_number: %d, %d bytes\r\n", _output_channel.sequence_number, packet_length);
		if(!_send_bytes_to_peer(p_buf, packet_length))
		{
			print_log("Error: PeerExchange: failed in re-sending packet\r\n");
		}
		_output_channel.timestamp = current_timestamp;
	}
	else
	{
		print_log("Error: PeerExchange: wrong output channel state (%d) in %s\r\n", _output_channel.state, __FILE__);
		_output_channel.state = IDLE;
	}
}

/**
 * Initialize the peer_exchange
 */
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

	_input_channel.state = TAG;
	_input_channel.data_count = 0;
	_input_channel.sequence_number_received = SEQUENCE_NUMBER_INVALID;

	_output_channel.state = IDLE;
	_output_channel.head = 0;
	_output_channel.tail = 0;
	_output_channel.sequence_number = SEQUENCE_NUMBER_INITIAL; // the first data packet

	return true;
}

/**
 * Put the message to output channel
 */
bool send_peer_message(const uint8_t * p_msg, const uint16_t msg_length)
{
	const uint16_t new_tail = (_output_channel.tail + 1) % OUTPUT_BUFFER_COUNT;
	if(new_tail == _output_channel.head)
	{
		print_log("Error: no available buffer for new message in %s\r\n", __FILE__);
		return false;
	}

	if(msg_length > PACKET_CONTENT_MAX_LENGTH)
	{
		print_log("Error: message is too long: %d, in %s\r\n", msg_length, __FILE__);
		return false;
	}

	uint8_t * p_packet = _output_channel.buffer_array[_output_channel.tail];
	p_packet[0] = DATA_TAG; // tag
	p_packet[1] = msg_length + 2; // length
	// p_packet[2] = sequence_number; // be assigned just before being sent
	memcpy(p_packet + 3, p_msg, msg_length);
	p_packet[msg_length + 4 - 1] = END_TAG;

	_output_channel.tail = new_tail;

	return true;
}

/**
 * This function needs to be called repeatedly to drive the peer_exchange,
 * the interval should be less than PACKET_RECEIVING_TIMEOUT
 */
void poll_peer_exchange()
{
	_process_input_channel();
	_process_output_channel();
}
