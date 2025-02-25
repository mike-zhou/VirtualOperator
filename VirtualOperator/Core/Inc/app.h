/*
 * app.h
 *
 *  Created on: Feb 18, 2025
 *      Author: Mike
 */

#ifndef INC_APP_H_
#define INC_APP_H_

#include <stdbool.h>
#include "stm32h7xx_hal.h"

bool test_UART_output(UART_HandleTypeDef *huart);
bool test_UART_echo(UART_HandleTypeDef *huart);

#endif /* INC_APP_H_ */
