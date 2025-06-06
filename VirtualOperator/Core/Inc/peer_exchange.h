/*
 * packet_exchanger.h
 *
 *  Created on: Mar 4, 2025
 *      Author: Mike
 */

#ifndef INC_PEER_EXCHANGE_H_
#define INC_PEER_EXCHANGE_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PACKET_MAX_LENGTH 256
#define PACKET_CONTENT_MAX_LENGTH (PACKET_MAX_LENGTH - 4) // 4 bytes for Tag, Length, sequence number and ending


bool init_peer_exchange(
		bool (*get_byte_from_peer)(uint8_t * const p_bype), // receive a byte from the peer
		bool (*send_bytes_to_peer)(const uint8_t * p_buffer, const uint16_t length), // send bytes to the peer
		uint16_t (*get_timestamp_ms)(void), // get time stamp
		void (*on_peer_message)(const uint8_t * p_buffer, const uint16_t length) // function which processes a peer message
		);

bool send_peer_message(const uint8_t * p_msg, const uint16_t msg_length);

void poll_peer_exchange();

#ifdef __cplusplus
}
#endif

#endif /* INC_PEER_EXCHANGE_H_ */
