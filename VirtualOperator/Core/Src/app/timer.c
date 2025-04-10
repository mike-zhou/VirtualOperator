/*
 * timer.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Mike
 */

#include "stm32h7xx_hal.h"
#include "usart1.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

static uint8_t overflow_counter = 0;
static uint8_t comparator_counter = 0;

/**
 * TIM Update event:
 * 	(htim->Instance->SR & 0x1) == 0x1 AND (htim->Instance->DIER & 0x1) == 0x1
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim17))
	{
		uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
		print_string("HAL_TIM_PeriodElapsedCallback: Timer 17 counter: 0x");
		print_uint32_hex(counter);
		print_string(", counter: 0x");
		print_uint8_hex(overflow_counter);
		print_string("\r\n");
	}
	else
	{
		uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
		print_string("HAL_TIM_PeriodElapsedCallback: unknown counter: 0x");
		print_uint32_hex(counter);
		print_string(", counter: 0x");
		print_uint8_hex(overflow_counter);
		print_string("\r\n");
	}

	overflow_counter++;
}


/**
 * Output compare event hander:
 * 	(htim->Instance->SR    & (TIM_FLAG_CC1)) == (TIM_FLAG_CC1)
 * 	(htim->Instance->DIER  & (TIM_IT_CC1))   == (TIM_IT_CC1)
 * 	(htim->Instance->CCMR1 & TIM_CCMR1_CC1S) == 0x00U
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim17))
	{
		uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
		print_string("HAL_TIM_OC_DelayElapsedCallback: Timer 17 counter: 0x");
		print_uint32_hex(counter);
		print_string(", counter: 0x");
		print_uint8_hex(comparator_counter);
		print_string("\r\n");
	}
	else
	{
		uint32_t counter = __HAL_TIM_GET_COUNTER(htim);
		print_string("HAL_TIM_OC_DelayElapsedCallback: unknown counter: 0x");
		print_uint32_hex(counter);
		print_string(", counter: 0x");
		print_uint8_hex(comparator_counter);
		print_string("\r\n");
	}

	comparator_counter++;
}

