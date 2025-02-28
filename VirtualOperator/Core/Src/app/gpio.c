/*
 * gpio.c
 *
 *  Created on: Feb 26, 2025
 *      Author: Mike
 */

#include "gpio.h"
#include "usart1.h"
#include "stm32h7xx_hal.h"

struct GPIO_Port_Pin
{
	GPIO_TypeDef * port;
	uint32_t pins;
} gpio_ports[11];


static void init_gpio_ports()
{
	gpio_ports[0].port = GPIOE;
	gpio_ports[0].pins = GPIO_PIN_2|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_10
            |GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
			|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_0
            |GPIO_PIN_1;

	gpio_ports[1].port = GPIOI;
	gpio_ports[1].pins = GPIO_PIN_8|GPIO_PIN_10|GPIO_PIN_12|GPIO_PIN_13
            |GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_1|GPIO_PIN_2
            |GPIO_PIN_5|GPIO_PIN_6
			|GPIO_PIN_9|GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4
            |GPIO_PIN_7;

	gpio_ports[2].port = GPIOC;
	gpio_ports[2].pins = GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_11
			|GPIO_PIN_14|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
            |GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
            |GPIO_PIN_12;

	gpio_ports[3].port = GPIOF;
	gpio_ports[3].pins = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8|GPIO_PIN_9
            |GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
            |GPIO_PIN_14|GPIO_PIN_15;

	gpio_ports[4].port = GPIOA;
	gpio_ports[4].pins = GPIO_PIN_2|GPIO_PIN_4;

	gpio_ports[5].port = GPIOH;
	gpio_ports[5].pins = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_13
            |GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_12|GPIO_PIN_15;

	gpio_ports[6].port = GPIOB;
	gpio_ports[6].pins = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;

	gpio_ports[7].port = GPIOJ;
	gpio_ports[7].pins = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
            |GPIO_PIN_4|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_5|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10
            |GPIO_PIN_11|GPIO_PIN_14;

	gpio_ports[8].port = GPIOG;
	gpio_ports[8].pins = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11
            |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
            |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;

	gpio_ports[9].port = GPIOD;
	gpio_ports[9].pins = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_14
            |GPIO_PIN_15|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
            |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;

	gpio_ports[10].port = GPIOK;
	gpio_ports[10].pins = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_4
            |GPIO_PIN_7|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_6;
}

static void set_all_gpio(uint32_t mode, uint32_t pull, uint32_t speed)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = pull;
	GPIO_InitStruct.Speed = speed;

	for(size_t index=0; index<11; index++)
	{
		GPIO_InitStruct.Pin = gpio_ports[index].pins;
		HAL_GPIO_Init(gpio_ports[index].port, &GPIO_InitStruct);
	}
}

static char * get_port_name(GPIO_TypeDef * port)
{
	if(port == GPIOE)
	{
		return "PE";
	}
	else if(port == GPIOI)
	{
		return "PI";
	}
	else if(port == GPIOC)
	{
		return "PC";
	}
	else if(port == GPIOF)
	{
		return "PF";
	}
	else if(port == GPIOA)
	{
		return "PA";
	}
	else if(port == GPIOH)
	{
		return "PH";
	}
	else if(port == GPIOB)
	{
		return "PB";
	}
	else if(port == GPIOJ)
	{
		return "PJ";
	}
	else if(port == GPIOG)
	{
		return "PG";
	}
	else if(port == GPIOD)
	{
		return "PD";
	}
	else if(port == GPIOK)
	{
		return "PK";
	}
	else
	{
		return "";
	}
}

static bool test_all_gpio(GPIO_PinState state)
{
	bool b_succeed = true;

	for(int port_index = 0; port_index < 11; port_index++)
	{
		uint32_t pins = gpio_ports[port_index].pins;
		uint32_t reg = gpio_ports[port_index].port->IDR;

		for(int pin_index = 0; pins > 0; pin_index++)
		{
			if(pins & 1)
			{
				if((reg & 1) != state)
				{
					b_succeed = false;
					print_log("        Error: %s%d is not %s\r\n", get_port_name(gpio_ports[port_index].port), pin_index, (state == GPIO_PIN_RESET)?"LOW":"HIGH");
				}
			}

			pins >>= 1;
			reg >>= 1;
		}

	}

	return b_succeed;
}

static bool test_all_gpio_pull_up_down()
{
	bool b_succeed = true;

	print_log("    All gpios should be in LOW state\r\n");
	set_all_gpio(GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW);
	if(test_all_gpio(GPIO_PIN_RESET))
	{
		print_log("        Passed\r\n");
	}
	else
	{
		print_log("        Failed\r\n");
		b_succeed = false;
	}

	print_log("    All gpios should be in HIGH state\r\n");
	set_all_gpio(GPIO_MODE_INPUT, GPIO_PULLUP, GPIO_SPEED_FREQ_LOW);
	if(test_all_gpio(GPIO_PIN_SET))
	{
		print_log("        Passed\r\n");
	}
	else
	{
		print_log("        Failed\r\n");
		b_succeed = false;
	}

	return b_succeed;
}

static bool check_all_gpio_with_exception(GPIO_TypeDef * excep_port, uint32_t excep_pin_index, GPIO_PinState excep_state)
{
	bool b_succeed = true;
	GPIO_PinState general_state = (excep_state == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;

	for(int port_index = 0; port_index < 11; port_index++)
	{
		uint32_t pins = gpio_ports[port_index].pins;
		uint32_t reg = gpio_ports[port_index].port->IDR;

		for(int pin_index = 0; pins > 0; pin_index++)
		{
			if(pins & 1)
			{
				if((reg & 1) != general_state)
				{
					if((gpio_ports[port_index].port != excep_port)
							|| (pin_index != excep_pin_index))
					{
						b_succeed = false;
						print_log("        Error: %s%d is not %s\r\n", get_port_name(gpio_ports[port_index].port), pin_index, (general_state == GPIO_PIN_RESET)?"LOW":"HIGH");
					}
				}
			}

			pins >>= 1;
			reg >>= 1;
		}
	}

	return b_succeed;
}

static bool test_pin_exception(GPIO_TypeDef * port, uint32_t pin_index, GPIO_PinState state)
{
	bool b_succeed = true;

	GPIO_InitTypeDef GPIO_InitStruct = {0};
	uint32_t pull = (state == GPIO_PIN_SET) ? GPIO_PULLDOWN : GPIO_PULLUP;
	uint32_t pin = 1;

	set_all_gpio(GPIO_MODE_INPUT, pull, GPIO_SPEED_FREQ_LOW);

	pin <<= pin_index;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Pin = pin;
	HAL_GPIO_Init(port, &GPIO_InitStruct);

	HAL_GPIO_WritePin(port, pin, state);

	b_succeed = check_all_gpio_with_exception(port, pin_index, state);

	return b_succeed;
}


static bool test_single_pin_exception()
{
	bool b_succeed = true;

	print_log("    GPIO pin's output shouldn't propagate to adjacent pins\r\n");
	for(int port_index = 0; port_index < 11; port_index++)
	{
		uint32_t pins = gpio_ports[port_index].pins;
		uint32_t reg = gpio_ports[port_index].port->IDR;

		for(uint32_t pin_index = 0; pins > 0; pin_index++)
		{
			if(pins & 1)
			{
				if(!test_pin_exception(gpio_ports[port_index].port, pin_index, GPIO_PIN_SET))
				{
					b_succeed = false;
				}
				if(!test_pin_exception(gpio_ports[port_index].port, pin_index, GPIO_PIN_RESET))
				{
					b_succeed = false;
				}
			}

			pins >>= 1;
			reg >>= 1;
		}
	}

	if(b_succeed)
	{
		print_log("    Passed\r\n");
	}
	else
	{
		print_log("    Failed");
	}

	return b_succeed;
}


void test_gpio()
{
	bool b_succeed = true;

	init_gpio_ports();

	if(b_succeed)
	{
		print_log("Test case: test_all_gpio_pull_up_down\r\n");
		b_succeed = test_all_gpio_pull_up_down();
	}
	if(b_succeed)
	{
		print_log("Test case: test_single_pin_exception\r\n");
		b_succeed = test_single_pin_exception();
	}
}

