/*
 * app.c
 *
 *  Created on: Feb 18, 2025
 *      Author: Mike
 */
#include <string.h>
#include "app.h"


bool test_UART_output(UART_HandleTypeDef *huart)
{
	static int counter = 0;
	char content[80];
	HAL_StatusTypeDef status;

	sprintf(content, "Hello world! %d\r\n", counter++);

	status = HAL_UART_Transmit(huart, content, strlen(content), HAL_MAX_DELAY);

	if(status == HAL_OK)
		return true;
	else
		return false;
}

bool test_UART_echo(UART_HandleTypeDef *huart)
{
	static int counter = 0;
	char content[80];
	HAL_StatusTypeDef status;

	status = HAL_UART_Receive(huart, content, 1, HAL_MAX_DELAY);
	if(status != HAL_OK)
		return false;

	status = HAL_UART_Transmit(huart, content, 1, HAL_MAX_DELAY);

	if(status == HAL_OK)
		return true;
	else
		return false;
}
