/*
 * usart1.h
 *
 *  Created on: Feb 21, 2025
 *      Author: Mike
 */

#ifndef SRC_APP_USART1_H_
#define SRC_APP_USART1_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"

bool prepare_uart1();
bool poll_usart1();

void print_uint8_hex(uint8_t val);
void print_uint16_hex(uint16_t val);
void print_uint32_hex(uint32_t val);
void print_char(char c);
void print_string(char * pStr);
void print_log(const char *format, ...);

// return true if the complete log is put to transfer buffer
// return false in case that truncation happens or no enough empty space in transfer buffer.
bool print_complete_log(const char *format, ...);

bool poll_char(unsigned char * pBuf);

#endif /* SRC_APP_USART1_H_ */
