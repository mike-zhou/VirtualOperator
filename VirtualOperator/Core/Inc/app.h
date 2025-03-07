/*
 * app.h
 *
 *  Created on: Mar 7, 2025
 *      Author: Mike
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdint.h>

#define APP_VERSION "0.0.0"

void on_host_command(const uint8_t * p_command, const uint16_t length);

void poll_app(void);

#endif /* INC_APP_H_ */
