/*
 * app.c
 *
 *  Created on: Mar 7, 2025
 *      Author: Mike
 */

#include <string.h>
#include "app.h"
#include "usart1.h"
#include "peer_exchange.h"
#include "host_command.h"
#include "encoder.h"
#include "timer.h"

extern uint32_t mainLoopCount;

static uint8_t _reply[PACKET_CONTENT_MAX_LENGTH];

static uint8_t _fill_version_data(uint8_t * p_buffer)
{
	uint8_t i;

	for(i = 0; i < (PACKET_CONTENT_MAX_LENGTH - 1) && i < strlen(APP_VERSION); i++)
	{
		p_buffer[i] = APP_VERSION[i];
	}

	return i;
}

static void _on_version(const uint8_t * p_cmd, const uint16_t length)
{
	uint8_t versionLength;

	_reply[0] = p_cmd[0];

	versionLength = _fill_version_data(_reply + 1);

	send_peer_message(_reply, 1 + versionLength);
}

static void _on_echo(const uint8_t * p_cmd, const uint16_t length)
{
	send_peer_message(p_cmd, length);
}

static uint16_t _get_gpio_mode(const GPIO_TypeDef * pGpio)
{
	uint32_t moder = pGpio->MODER;
	uint16_t mask = 0;

	for(int i = 0; i < 16; i++)
	{
		if((moder & 0x3) == 0x1)
		{
			// set bit to 1 if gpio output mode.
			mask |= (1 << i);
		}
		moder >>= 2;
	}

	return mask;
}

static void _on_get_gpio_mode(const uint8_t * p_cmd, const uint16_t length)
{
	uint16_t mask;

	_reply[0] = p_cmd[0];

	// little endian
	mask = _get_gpio_mode(GPIOA);
	_reply[1] = mask & 0xff;
	_reply[2] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOB);
	_reply[3] = mask & 0xff;
	_reply[4] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOC);
	_reply[5] = mask & 0xff;
	_reply[6] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOD);
	_reply[7] = mask & 0xff;
	_reply[8] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOE);
	_reply[9] = mask & 0xff;
	_reply[10] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOF);
	_reply[11] = mask & 0xff;
	_reply[12] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOG);
	_reply[13] = mask & 0xff;
	_reply[14] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOH);
	_reply[15] = mask & 0xff;
	_reply[16] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOI);
	_reply[17] = mask & 0xff;
	_reply[18] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOJ);
	_reply[19] = mask & 0xff;
	_reply[20] = (mask >> 8) & 0xff;

	mask = _get_gpio_mode(GPIOK);
	_reply[21] = mask & 0xff;
	_reply[22] = (mask >> 8) & 0xff;

	send_peer_message(_reply, 23);
}

static uint8_t _fill_gpio_data(uint8_t * p_buffer)
{
	uint16_t value;

	// little endian
	value = GPIOA->IDR;
	p_buffer[0] = value & 0xff;
	p_buffer[1] = (value >> 8) & 0xff;

	value = GPIOB->IDR;
	p_buffer[2] = value & 0xff;
	p_buffer[3] = (value >> 8) & 0xff;

	value = GPIOC->IDR;
	p_buffer[4] = value & 0xff;
	p_buffer[5] = (value >> 8) & 0xff;

	value = GPIOD->IDR;
	p_buffer[6] = value & 0xff;
	p_buffer[7] = (value >> 8) & 0xff;

	value = GPIOE->IDR;
	p_buffer[8] = value & 0xff;
	p_buffer[9] = (value >> 8) & 0xff;

	value = GPIOF->IDR;
	p_buffer[10] = value & 0xff;
	p_buffer[11] = (value >> 8) & 0xff;

	value = GPIOG->IDR;
	p_buffer[12] = value & 0xff;
	p_buffer[13] = (value >> 8) & 0xff;

	value = GPIOH->IDR;
	p_buffer[14] = value & 0xff;
	p_buffer[15] = (value >> 8) & 0xff;

	value = GPIOI->IDR;
	p_buffer[16] = value & 0xff;
	p_buffer[17] = (value >> 8) & 0xff;

	value = GPIOJ->IDR;
	p_buffer[18] = value & 0xff;
	p_buffer[19] = (value >> 8) & 0xff;

	value = GPIOK->IDR;
	p_buffer[20] = value & 0xff;
	p_buffer[21] = (value >> 8) & 0xff;

	return 22;
}

static void _on_read_gpios(const uint8_t * p_cmd, const uint16_t length)
{

	_reply[0] = p_cmd[0];

	uint8_t amount = _fill_gpio_data(_reply + 1);

	send_peer_message(_reply, 1 + amount);
}

static void _on_set_gpio(const uint8_t * p_cmd, const uint16_t length)
{
	_reply[0] = p_cmd[0];

	if((length % 3) != 1)
	{
		print_log("Error: wrong length of HOST_COMMAND_SET_GPIO: %d\r\n", length);
		send_peer_message(_reply, 1);
		return;
	}

	uint8_t portCount = (length - 1) / 3;
	for(uint8_t i=0; i<portCount; i++)
	{
		uint8_t cmdByteIndex = i * 3 + 1;
		uint8_t portIndex = p_cmd[cmdByteIndex];
		uint8_t bitIndex = 	p_cmd[cmdByteIndex + 1];
		uint8_t level = 	p_cmd[cmdByteIndex + 2];

		GPIO_TypeDef * pPort = NULL;

		switch(portIndex)
		{
		case 0:
			pPort = GPIOA;
			break;
		case 1:
			pPort = GPIOB;
			break;
		case 2:
			pPort = GPIOC;
			break;
		case 3:
			pPort = GPIOD;
			break;
		case 4:
			pPort = GPIOE;
			break;
		case 5:
			pPort = GPIOF;
			break;
		case 6:
			pPort = GPIOG;
			break;
		case 7:
			pPort = GPIOH;
			break;
		case 8:
			pPort = GPIOI;
			break;
		case 9:
			pPort = GPIOJ;
			break;
		case 10:
			pPort = GPIOK;
			break;
		default:
			break;
		}

		if(pPort == NULL)
		{
			print_log("Error: wrong port index (%d) in HOST_COMMAND_SET_GPIO\r\n", portIndex);
			send_peer_message(_reply, 1);
			return;
		}
		if(bitIndex > 15)
		{
			print_log("Error: wrong bit index (%d) in HOST_COMMAND_SET_GPIO\r\n", bitIndex);
			send_peer_message(_reply, 1);
			return;
		}
		if(level > 1)
		{
			print_log("Error: wrong level (%d) in HOST_COMMAND_SET_GPIO\r\n", level);
			send_peer_message(_reply, 1);
			return;
		}

		uint16_t mode = _get_gpio_mode(pPort);
		if((mode & (1 << bitIndex)) == 0)
		{
			print_log("Error: write read-only GPIO (%d) in HOST_COMMAND_SET_GPIO\r\n", bitIndex);
			send_peer_message(_reply, 1);
			return;
		}

		if(level == 0)
		{
			HAL_GPIO_WritePin(pPort, 1 << bitIndex, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(pPort, 1 << bitIndex, GPIO_PIN_SET);
		}

		_reply[cmdByteIndex] = portIndex;
		_reply[cmdByteIndex + 1] = bitIndex;
		_reply[cmdByteIndex + 2] = level;
	}

	send_peer_message(_reply, length);
}

static uint8_t _fill_encoder_data(uint8_t * p_buffer)
{
	uint16_t value;

	// little endian
	value = encoder_get_count(ENCODER_0);
	p_buffer[0] = value & 0xff;
	p_buffer[1] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_1);
	p_buffer[2] = value & 0xff;
	p_buffer[3] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_2);
	p_buffer[4] = value & 0xff;
	p_buffer[5] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_3);
	p_buffer[6] = value & 0xff;
	p_buffer[7] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_4);
	p_buffer[8] = value & 0xff;
	p_buffer[9] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_5);
	p_buffer[10] = value & 0xff;
	p_buffer[11] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_6);
	p_buffer[12] = value & 0xff;
	p_buffer[13] = (value >> 8) & 0xff;

	value = encoder_get_count(ENCODER_7);
	p_buffer[14] = value & 0xff;
	p_buffer[15] = (value >> 8) & 0xff;

	return 16;
}

static void _on_read_encoders(const uint8_t * p_cmd, const uint16_t length)
{
	_reply[0] = p_cmd[0];

	uint8_t amount = _fill_encoder_data(_reply + 1);

	send_peer_message(_reply, 1 + amount);
}

static void _on_get_status(const uint8_t * p_cmd, const uint16_t length)
{
	uint16_t amount;
	uint16_t consumed;
	uint16_t value;

	_reply[0] = p_cmd[0];
	amount = 1;
	// _reply = command tag
	// amount: 1

	consumed = _fill_gpio_data(_reply + amount);
	amount += consumed;
	if(consumed != 22)
	{
		print_log("Error: wrong amount of gpio values in %s\r\n", __FILE__);
		send_peer_message(_reply, amount);
		return;
	}
	// _reply += gpio values
	// amount: 23
	
	consumed = _fill_encoder_data(_reply + amount);
	amount += consumed;
	if(consumed != 16)
	{
		print_log("Error: wrong amount of encoder values in %s\r\n", __FILE__);
		send_peer_message(_reply, amount);
		return;
	}
	// _reply += gpio values
	// amount: 39

	_reply[amount] = mainLoopCount;
	amount++;
	_reply[amount] = mainLoopCount >> 8;
	amount++;
	_reply[amount] = mainLoopCount >> 16;
	amount++;
	_reply[amount] = mainLoopCount >> 24;
	amount++;
	// _reply += mainLoopCount
	// amount: 43

	value = timer_get_max_flex_isr_period();
	_reply[amount] = value & 0xff;
	amount++;
	_reply[amount] = value >> 8;
	amount++;
	// _reply += max_flex_isr_period
	// amount: 45

	value = timer_get_max_fix_isr_period();
	_reply[amount] = value & 0xff;
	amount++;
	_reply[amount] = value >> 8;
	amount++;
	// _reply += max_fix_isr_period
	// amount: 47

	send_peer_message(_reply, amount);
}

static void _on_set_stepper_active_rampup_pulse_width(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * cmd is in the format:
	 * 	byte:	command ID
	 * 	byte: 	stepper ID
	 * 	byte: 	batch index
	 * 	byte: 	total batches
	 * 	byte: 	pulse0 low byte
	 * 	byte:	pulse0 high byte
	 * 	byte:	...
	 * 	byte: 	pulseN high byte
	 */
	_reply[0] = p_cmd[0];
	
	if(length > 255)
	{
		_reply[1] = 1; 
		send_peer_message(_reply, 2);
		print_log("Error: too many active rampup pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}
	if(length <= 4)
	{
		_reply[1] = 2; 
		send_peer_message(_reply, 2);
		print_log("Error: too less active rampup pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}
	if(length & 0x1 != 0x0)
	{
		_reply[1] = 3; 
		send_peer_message(_reply, 2);
		print_log("Error: wrongly aligned rampup pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}

	StepperId stepperId = (StepperId) p_cmd[1];
	uint8_t batchIndex = p_cmd[2];
	uint8_t totalBatches = p_cmd[3];

	uint8_t * pPulseWidth = p_cmd + 4;
	uint8_t widthLength = length - 4;

	StepperReturnCode result = stepper_set_active_rampup_pulse_widths(stepperId, pPulseWidth, widthLength, batchIndex, totalBatches);
	if(result != STEPPER_OK)
	{
		_reply[1] = 4;
		print_log("Error: stepper_set_active_rampup_pulse_widths failure: %d in %s\r\n", result, __FILE__);
	}
	else
	{
		_reply[1] = 0;
	}
	send_peer_message(_reply, 2);
}

static void _on_set_stepper_active_cruise_pulse_width(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * cmd is in the format:
	 * 	byte:	command ID
	 * 	byte: 	stepper ID
	 * 	byte: 	pulse low byte
	 * 	byte:	pulse high byte
	 */
	_reply[0] = p_cmd[0];

	if(length != 4)
	{
		_reply[1] = 1;
		send_peer_message(_reply, 2);
		print_log("Error: invalid length in _on_set_stepper_active_cruise_pulse_width: %d in %s\r\n", length, __FILE__);
		return;
	}

	StepperId stepperId = (StepperId) p_cmd[1];

	uint16_t width = p_cmd[3];
	width <<= 8;
	width += p_cmd[2];

	StepperReturnCode result = stepper_set_active_cruise_pulse_width(stepperId, width);
	if(result != STEPPER_OK)
	{
		_reply[1] = 2;
		print_log("Error: stepper_set_active_cruise_pulse_width failure: %d in %s\r\n", result, __FILE__);
	}
	else
	{
		_reply[1] = 0;
	}
	send_peer_message(_reply, 2);
}

static void _on_set_stepper_active_rampdown_pulse_width(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * cmd is in the format:
	 * 	byte:	command ID
	 * 	byte: 	stepper ID
	 * 	byte: 	batch index
	 * 	byte: 	total batches
	 * 	byte: 	pulse0 low byte
	 * 	byte:	pulse0 high byte
	 * 	byte:	...
	 * 	byte: 	pulseN high byte
	 */
	_reply[0] = p_cmd[0];
	
	if(length > 255)
	{
		_reply[1] = 1; 
		send_peer_message(_reply, 2);
		print_log("Error: too many active rampdown pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}
	if(length <= 4)
	{
		_reply[1] = 2; 
		send_peer_message(_reply, 2);
		print_log("Error: too less active rampdown pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}
	if(length & 0x1 != 0x0)
	{
		_reply[1] = 3; 
		send_peer_message(_reply, 2);
		print_log("Error: wrongly aligned rampdown pulse width: %d in %s\r\n", length, __FILE__);
		return;
	}

	StepperId stepperId = (StepperId) p_cmd[1];
	uint8_t batchIndex = p_cmd[2];
	uint8_t totalBatches = p_cmd[3];

	uint8_t * pPulseWidth = p_cmd + 4;
	uint8_t widthLength = length - 4;

	StepperReturnCode result = stepper_set_active_rampdown_pulse_widths(stepperId, pPulseWidth, widthLength, batchIndex, totalBatches);
	if(result != STEPPER_OK)
	{
		_reply[1] = 4;
		print_log("Error: stepper_set_active_rampdown_pulse_widths failure: %d in %s\r\n", result, __FILE__);
	}
	else
	{
		_reply[1] = 0;
	}
	send_peer_message(_reply, 2);	
}

static GPIO_TypeDef * _get_gpio_port_ptr(uint8_t port_index)
{
	switch(port_index)
	{
		case 0: return GPIOA;
		case 1: return GPIOB;
		case 2: return GPIOC;
		case 4:	return GPIOD;
		case 5:	return GPIOE;
		case 6: return GPIOF;
		case 7: return GPIOG;
		case 8: return GPIOG;
		case 9: return GPIOH;
		case 10: return GPIOI;
		case 11: return GPIOJ;
		case 12: return GPIOK;
		default: 
			print_log("Error: invalid port index in _get_gpio_port_ptr() in %s\r\n", __FILE__);
			return NULL;
	}
}

static void _on_set_stepper_controls(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * cmd is in the format:
	 *  0: 	command id
	 * 	1: 	stepper id
	 * 	2: 	is rising edge driven
	 * 	3:	is forward high
	 * 	4:	is enable high
	 * 	5:	gpio port index of home boundary 
	 * 	6: 	gpio pin index of home boundary
	 * 	7:	gpio port index of end boundary
	 * 	8:	gpio pin index of end boundary
	 * 	9:	gpio port index of enable signal
	 * 	10:	gpio pin index of enable signal
	 * 	11:	gpio port index of forward signal
	 * 	12:	gpio pin index of forward signal
	 * 	13:	gpio port index of clock signal
	 * 	14:	gpio pin idnex of clock signal
	 * 	15:	1/2 of steps from home boundary to ready
	 * 	16:	2/2 of steps from home boundary to ready
	 * 	17: 1/4 of range
	 * 	18: 2/4 of range
	 * 	19:	3/4 of range
	 * 	20: 4/4 of range
	 * 	21: 1/2 of steps when the stepper turns a round
	 * 	22: 2/2 of steps when the stepper turns a round
	 * 	23: encoder id
	 * 	24: 1/2 of counts when the encoder turns a round
	 * 	25: 2/2 of counts when the encoder turns a round
	 * 	26: 1/2 of max allowable difference between actual position and ideal position
	 * 	27: 2/2 of max allowable difference between actual position and ideal position
	 */

	StepperId stepperId = (StepperId)p_cmd[1];
	bool isRisingEdgeDriven = p_cmd[2] > 0;
	bool isForwardHigh = p_cmd[3] > 0;
	bool isEnableHigh = p_cmd[4] > 0;
	uint8_t portIndexHomeBoundary = p_cmd[5];
	uint8_t pinIndexHomeBoundary = p_cmd[6];
	uint8_t portIndexEndBoundary = p_cmd[7];
	uint8_t pinIndexEndBoundary = p_cmd[8];
	uint8_t portIndexEnableSignal = p_cmd[9];
	uint8_t pinIndexEnableSignal = p_cmd[10];
	uint8_t portIndexForwardSignal = p_cmd[11];
	uint8_t pinIndexForwardSignal = p_cmd[12];
	uint8_t portIndexClockSignal = p_cmd[13];
	uint8_t pinIndexClockSignal = p_cmd[14];

	uint16_t homeToReadySteps = p_cmd[16];
	homeToReadySteps <<= 8;
	homeToReadySteps += p_cmd[15];

	uint32_t range = p_cmd[20];
	range <<= 8;
	range += p_cmd[19];
	range <<= 8;
	range += p_cmd[18];
	range <<=8;
	range += p_cmd[17];

	uint16_t stepsPerRound = p_cmd[22];
	stepsPerRound <<= 8;
	stepsPerRound += p_cmd[21];

	EncoderId encoderId = (EncoderId)p_cmd[23];

	uint16_t countsPerEncoderRound = p_cmd[25];
	countsPerEncoderRound <<= 8;
	countsPerEncoderRound += p_cmd[24];

	uint16_t maxPositionError = p_cmd[27];
	maxPositionError <<= 8;
	maxPositionError += p_cmd[26];

	_reply[0] = p_cmd[0];

	if(stepperId >= STEPPER_ID_COUNT)
	{
		_reply[1] = 1;
		send_peer_message(_reply, 2);
		return;
	}

	if(portIndexHomeBoundary > ('K' - 'A'))
	{
		_reply[1] = 2;
		send_peer_message(_reply, 2);
		return;
	}
	if(pinIndexHomeBoundary > 15)
	{
		_reply[1] = 3;
		send_peer_message(_reply, 2);
		return;
	}

	if(portIndexEndBoundary > ('K' - 'A'))
	{
		_reply[1] = 4;
		send_peer_message(_reply, 2);
		return;
	}
	if(pinIndexEndBoundary > 15)
	{
		_reply[1] = 5;
		send_peer_message(_reply, 2);
		return;
	}

	if(portIndexEnableSignal > ('K' - 'A'))
	{
		_reply[1] = 6;
		send_peer_message(_reply, 2);
		return;
	}
	if(pinIndexEnableSignal > 15)
	{
		_reply[1] = 7;
		send_peer_message(_reply, 2);
		return;
	}
	
	if(portIndexForwardSignal > ('K' - 'A'))
	{
		_reply[1] = 8;
		send_peer_message(_reply, 2);
		return;
	}
	if(pinIndexForwardSignal > 15)
	{
		_reply[1] = 9;
		send_peer_message(_reply, 2);
		return;
	}
	
	if(portIndexClockSignal > ('K' - 'A'))
	{
		_reply[1] = 10;
		send_peer_message(_reply, 2);
		return;
	}
	if(pinIndexClockSignal > 15)
	{
		_reply[1] = 11;
		send_peer_message(_reply, 2);
		return;
	}

	if(encoderId >= ENCODER_COUNT && encoderId != ENCODER_INVALID_ID)
	{
		_reply[1] = 12;
		send_peer_message(_reply, 2);
		return;
	}

	print_log("Info: stepperId: %d, risingEdgeDriven: %d, forwardHigh: %d, enableHigh: %d, portIndexHome: %d, pinIndexHome: %d, portIndexEnd: %d, pinIndexEnd: %d, portIndexEnable: %d, pinIndexEnable: %d, portIndexForward: %d, pinIndexForward: %d,	portIndexClock: %d, pinIndexClock: %d, homeToReadySteps: %d, range: %d, stepsPerRound: %d, encoderId: %d, countsPerEncoderRound: %d, maxPositionError: %d\r\n", 
		stepperId, 
		isRisingEdgeDriven,
		isForwardHigh,
		isEnableHigh,
		portIndexHomeBoundary, 
		pinIndexHomeBoundary,
		portIndexEndBoundary,
		pinIndexEndBoundary,
		portIndexEnableSignal,
		pinIndexEnableSignal,
		portIndexForwardSignal,
		pinIndexForwardSignal,
		portIndexClockSignal,
		pinIndexClockSignal,
		homeToReadySteps,
		range,
		stepsPerRound,
		encoderId,
		countsPerEncoderRound,
		maxPositionError);
	
	GPIO_TypeDef * pPortHomeBoundary = 	_get_gpio_port_ptr(portIndexHomeBoundary);
	GPIO_TypeDef * pPortEndBoundary = 	_get_gpio_port_ptr(portIndexEndBoundary);
	GPIO_TypeDef * pPortEnableSignal = 	_get_gpio_port_ptr(portIndexEnableSignal);
	GPIO_TypeDef * pPortForwardSignal = _get_gpio_port_ptr(portIndexForwardSignal);
	GPIO_TypeDef * pPortClockSignal = 	_get_gpio_port_ptr(portIndexClockSignal);

	StepperReturnCode result = stepper_set_controls(
		stepperId,
		isRisingEdgeDriven,
		isForwardHigh,
		isEnableHigh,
		pPortHomeBoundary,
		pinIndexHomeBoundary,
		pPortEndBoundary,
		pinIndexEndBoundary,
		pPortEnableSignal,
		pinIndexEnableSignal,
		pPortForwardSignal,
		pinIndexForwardSignal,
		pPortClockSignal,
		pinIndexClockSignal,
		homeToReadySteps,
		range,
		stepsPerRound,
		encoderId,
		countsPerEncoderRound,
		maxPositionError);

	if(result == STEPPER_OK)
	{
		_reply[1] = 0;
	}
	else
	{
		print_log("Error: stepper_set_controls() failure: %d in %s\r\n", result, __FILE__);
		_reply[1] = 13;
	}
	send_peer_message(_reply, 2);
}

static void _on_set_stepper_enable(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * command format:
	 * 0:	command id
	 * 1:	stepper id
	 * 2:	enable stepper
	 */

	_reply[0] = p_cmd[0];
	StepperId stepperId = (StepperId) p_cmd[1];
	bool enableStepper = p_cmd[2] > 0;

	StepperReturnCode result = stepper_set_enable(stepperId, enableStepper);
	if(result == STEPPER_OK)
	{
		_reply[1] = 0;
	}
	else
	{
		print_log("Error: stepper_set_enable() failure: %d in %s\r\n", result, __FILE__);
		_reply[1] = 1;
	}
	send_peer_message(_reply, 2);
}

static void _on_set_stepper_forward(const uint8_t * p_cmd, const uint16_t length)
{
	/**
	 * command format:
	 * 0:	command id
	 * 1:	stepper id
	 * 2:	isForward
	 */

	_reply[0] = p_cmd[0];
	StepperId stepperId = (StepperId) p_cmd[1];
	bool isForward = p_cmd[2] > 0;

	StepperReturnCode result = stepper_set_forward(stepperId, isForward);
	if(result == STEPPER_OK)
	{
		_reply[1] = 0;
	}
	else
	{
		print_log("Error: stepper_set_forward() failure: %d in %s\r\n", result, __FILE__);
		_reply[1] = 1;
	}
	send_peer_message(_reply, 2);
}

static void _on_start_stepper_home_positioning(const uint8_t * p_cmd, const uint16_t length)
{
	
}

static void _on_run_stepper(const uint8_t * p_cmd, const uint16_t length)
{
	
}

void on_host_command(const uint8_t * p_command, const uint16_t length)
{
	if(length == 0)
	{
		print_log("Error: empty host message in %s\r\n", __FILE__);
		return;
	}
	if(p_command == NULL)
	{
		print_log("Error: invalid buffer address in %s\r\n", __FILE__);
		return;
	}

	print_log("host cmd: %d, %d bytes\r\n", p_command[0], length);

	uint8_t host_command = p_command[0];
	switch(host_command)
	{
	case HOST_COMMAND_VERSION:
		_on_version(p_command, length);
		break;
	case HOST_COMMAND_ECHO:
		_on_echo(p_command, length);
		break;
	case HOST_COMMAND_GET_GPIO_MODE:
		_on_get_gpio_mode(p_command, length);
		break;
	case HOST_COMMAND_READ_GPIOS:
		_on_read_gpios(p_command, length);
		break;
	case HOST_COMMAND_SET_GPIO:
		_on_set_gpio(p_command, length);
		break;
	case HOST_COMMAND_READ_ENCODERS:
		_on_read_encoders(p_command, length);
		break;
	case HOST_COMMAND_GET_STATUS:
		_on_get_status(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_ACTIVE_RAMPUP_PULSE_WIDTH:
		_on_set_stepper_active_rampup_pulse_width(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_ACTIVE_CRUISE_PULSE_WIDTH:
		_on_set_stepper_active_cruise_pulse_width(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_ACTIVE_RAMPDOWN_PULSE_WIDTH:
		_on_set_stepper_active_rampdown_pulse_width(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_CONTROLS:
		_on_set_stepper_controls(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_ENABLE:
		_on_set_stepper_enable(p_command, length);
		break;
	case HOST_COMMAND_SET_STEPPER_FORWARD:
		_on_set_stepper_forward(p_command, length);
		break;
	case HOST_COMMAND_START_STEPPER_HOME_POSITIONING:
		_on_start_stepper_home_positioning(p_command, length);
		break;
	case HOST_COMMAND_RUN_STEPPER:
		_on_run_stepper(p_command, length);
		break;	
	default:
		print_log("Error: unknown host command: %d in %s\r\n", host_command, __FILE__);
		break;
	}
}


void poll_app(void)
{

}
