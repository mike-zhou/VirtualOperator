/*
 * timer.c
 *
 *  Created on: Apr 10, 2025
 *      Author: Mike
 */

#include "stm32h7xx_hal.h"
#include "usart1.h"
#include "timer.h"

extern HRTIM_HandleTypeDef hhrtim;

extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
extern TIM_HandleTypeDef htim14;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

#define FLEX_TIMER_COUNT (TIMER_ID_COUNT - 1)

typedef struct 
{
	TimerState state;
	TIM_HandleTypeDef * pTimerHandle;
	StepperId stepperId;

	// data for testing
	bool isTesting;
	uint16_t pulseIndexTesting;
	uint16_t totalPulseTesting;
	uint16_t logIntervalTesting;
} FlexTimer;

typedef struct
{
	TimerState state;
	HRTIM_HandleTypeDef * pTimerHandle;
	uint32_t fixPulseWidth_ns; // in nano second
	struct Stepper
	{
		StepperId stepperId;
		// when the stepper is added, the FixTimer may have already started 
		// the current pulse. If so, the stepper is triggered by a pulse which
		// is less than the expected length. To avoid this, the first pulse
		// is skipped.
		bool firstPulseSkipped; 
		uint16_t expectedPulseWidth;
		uint16_t remainingPulseWidth;
	} steppers[STEPPER_ID_COUNT];

	// data for testing
	bool isTesting;
	uint16_t pulseIndexTesting;
	uint16_t totalPulseTesting;
	uint16_t logIntervalTesting;
} FixTimer;

static FlexTimer _flexTimers[FLEX_TIMER_COUNT];
static FixTimer _fixTimer;

static uint16_t _maxFlexTimerIsrPeriod;
static uint16_t _maxFixTimerIsrPeriod;

static inline uint32_t _get_fix_timer_interval_ns(void)
{
    /* Step-1: kernel clock that feeds the whole HRTIM engine */
    // uint32_t f_hrtim = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_HRTIM1);
	uint32_t f_hrtim = 240 * 1024 * 1024;
	uint32_t CKPSC = __HAL_HRTIM_GETCLOCKPRESCALER(&hhrtim, HRTIM_TIMERINDEX_TIMER_A);
	uint32_t ratio;

	switch(CKPSC)
	{
		case 0b101:
			ratio = 1;
			break;
		
		case 0b110:
			ratio = 2;
			break;

		case 0b111:
			ratio = 4;
			break;
		
		default:
			print_log("Error: _get_fix_timer_interval_ns, unsupported CKPSK: 0x%x\r\n", CKPSC);
			ratio = 4;
			break;
	}

	uint32_t clockInterval = 1024 * 1024 * 1024 / (f_hrtim / ratio); // GHz / (f_hrtim / ratio)
	uint32_t period = __HAL_HRTIM_GETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_A);
	uint32_t interval_ns = clockInterval * period;

    return interval_ns;         
}

void timer_init_data_structure()
{
	FlexTimer * pTimer;
	
	// flex timers. A flex timer can drive only 1 stepper.
	pTimer = _flexTimers;
	pTimer->pTimerHandle = &htim12;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	pTimer = _flexTimers + 1;
	pTimer->pTimerHandle = &htim13;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	pTimer = _flexTimers + 2;
	pTimer->pTimerHandle = &htim14;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	pTimer = _flexTimers + 3;
	pTimer->pTimerHandle = &htim15;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	pTimer = _flexTimers + 4;
	pTimer->pTimerHandle = &htim16;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	pTimer = _flexTimers + 5;
	pTimer->pTimerHandle = &htim17;
	pTimer->stepperId = STEPPER_ID_INVALID;
	pTimer->state = TIMER_STATE_IDLE;
	pTimer->isTesting = false;

	// fix timer. A fix timer can drive multiple steppers.
	_fixTimer.pTimerHandle = &hhrtim;
	_fixTimer.fixPulseWidth_ns = _get_fix_timer_interval_ns();
	print_log("Info: fix timer interval is set to %d ns\r\n", _fixTimer.fixPulseWidth_ns);
	for(int i=0; i<STEPPER_ID_COUNT; i++)
	{
		_fixTimer.steppers[i].stepperId = STEPPER_ID_INVALID;
		_fixTimer.steppers[i].remainingPulseWidth = 0;
	}
	_fixTimer.state = TIMER_STATE_IDLE;
	_fixTimer.isTesting = false;

	_maxFlexTimerIsrPeriod = 0;
	_maxFixTimerIsrPeriod = 0;
}

TimerReturnCode timer_start(const TimerId timerId, const StepperId stepperId, const uint16_t pulseWidth)
{
	if(timerId >= TIMER_ID_COUNT)
	{
		return TIMER_ERROR_INVALID_ID;
	}
	if(stepperId >= STEPPER_ID_COUNT)
	{
		return TIMER_ERROR_INVALID_STEPPER_ID;
	}
	if(pulseWidth == 0)
	{
		return TIMER_ERROR_INVALID_PULSE_WIDTH;
	}
	if(timerId < FIX_TIMER_ID && _flexTimers[timerId].state != TIMER_STATE_IDLE)
	{
		return TIMER_ERROR_ALREADY_RUNNING;
	}

	bool stepperIsRunning = false;
	for(int i=0; i<(int)FIX_TIMER_ID; i++)
	{
		FlexTimer * pTimer = _flexTimers + i;
		if(pTimer->state != TIMER_STATE_BUSY)
		{
			continue;
		}
		if(pTimer->stepperId == stepperId)
		{
			stepperIsRunning = true; // stepper is being driven
			break;
		}
	}
	if(stepperIsRunning)
	{
		return TIMER_ERROR_STEPPER_DRIVEN_BY_OTHER;
	}

	if(_fixTimer.state == TIMER_STATE_BUSY)
	{
		for(int i=0; i<(int)STEPPER_ID_COUNT; i++)
		{
			if(_fixTimer.steppers[i].stepperId == stepperId)
			{
				stepperIsRunning = true; // stepper has already been driven
				break;
			}
		}
	}
	if(stepperIsRunning)
	{
		return TIMER_ERROR_STEPPER_DRIVEN_BY_OTHER;
	}

	if(timerId == FIX_TIMER_ID)
	{
		_fixTimer.steppers[stepperId].remainingPulseWidth = pulseWidth;
		_fixTimer.steppers[stepperId].firstPulseSkipped = false;
		_fixTimer.steppers[stepperId].stepperId = stepperId; // indicate the stepper is being clocked
		
		return TIMER_OK;
	}

	_flexTimers[timerId].stepperId = stepperId;
	__HAL_TIM_SET_AUTORELOAD(_flexTimers[timerId].pTimerHandle, pulseWidth);
	__HAL_TIM_SET_COUNTER(_flexTimers[timerId].pTimerHandle, 1);
	_flexTimers[timerId].state = TIMER_STATE_BUSY;
	HAL_StatusTypeDef rc = HAL_TIM_Base_Start_IT(_flexTimers[timerId].pTimerHandle);
	if(rc != HAL_OK)
	{
		_flexTimers[timerId].state = TIMER_STATE_IDLE;
		_flexTimers[timerId].stepperId = STEPPER_ID_INVALID;
		print_log("Error: timer_start(), failed to start timer: %d, rc: %d\r\n", timerId, rc);
		return TIMER_ERROR_INTERNAL_FAILURE;
	}

	return TIMER_OK;	
}

TimerReturnCode timer_start_fix_timer()
{
	if(_fixTimer.state == TIMER_STATE_IDLE)
	{
		_fixTimer.state = TIMER_STATE_BUSY;
		__HAL_HRTIM_SETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A, 1);
		HAL_StatusTypeDef rc = HAL_HRTIM_SimpleBaseStart_IT(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
		if(rc != HAL_OK)
		{
			_fixTimer.state = TIMER_STATE_IDLE;
			print_log("Error: timer_start(), failed to start fix timer, rc: %d\r\n", rc);
			return TIMER_ERROR_INTERNAL_FAILURE;
		}
	}

	return TIMER_OK;
}

/**
 * @brief 	Test the specified timer with designated pulse information. 
 * 			An filed in the timer structure is set to indicate that the test is on-going,
 * 			and the ISR will print something to indicate that the timer is running.
 * 			The timer is restored to normal state after the test finishes.
 * @param timerId Timer ID
 * @param pulseWidth The PERIOD value in the timer hardware
 * @param totalPulse Total count of timer events in this test
 * @param logInterval The count of timer events for one printing
 */
TimerReturnCode timer_test(const TimerId timerId, const uint16_t pulseWidth, const uint16_t totalPulse, const uint16_t logInterval)
{
	if(timerId >= TIMER_ID_COUNT)
	{
		return TIMER_ERROR_INVALID_ID;
	}
	if(pulseWidth == 0)
	{
		return TIMER_ERROR_INVALID_PULSE_WIDTH;
	}

	if(timerId == FIX_TIMER_ID)
	{
		__HAL_HRTIM_SETPERIOD(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A, pulseWidth);
		__HAL_HRTIM_SETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A, 1);

		_fixTimer.pulseIndexTesting = 0;
		_fixTimer.totalPulseTesting = totalPulse;
		_fixTimer.logIntervalTesting = logInterval;
		_fixTimer.isTesting = true;

		HAL_StatusTypeDef rc = HAL_HRTIM_SimpleBaseStart_IT(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
		if(rc != HAL_OK)
		{
			print_log("Error: timer_test(), failed to start fix timer, rc: %d\r\n", rc);
			return TIMER_ERROR_INTERNAL_FAILURE;
		}

		return TIMER_OK;
	}

	__HAL_TIM_SET_AUTORELOAD(_flexTimers[timerId].pTimerHandle, pulseWidth);
	__HAL_TIM_SET_COUNTER(_flexTimers[timerId].pTimerHandle, 1);

	_flexTimers[timerId].pulseIndexTesting = 0;
	_flexTimers[timerId].totalPulseTesting = totalPulse;
	_flexTimers[timerId].logIntervalTesting = logInterval;
	_flexTimers[timerId].isTesting = true;

	HAL_StatusTypeDef rc = HAL_TIM_Base_Start_IT(_flexTimers[timerId].pTimerHandle);
	if(rc != HAL_OK)
	{
		print_log("Error: timer_test(), failed to start timer: %d, rc: %d\r\n", timerId, rc);
		return TIMER_ERROR_INTERNAL_FAILURE;
	}

	return TIMER_OK;	
}

static void _stop_fix_timer()
{
	_fixTimer.state = TIMER_STATE_IDLE;
	HAL_HRTIM_SimpleBaseStop_IT(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
	
	for(int i=0; i<STEPPER_ID_COUNT; i++)
	{
		_fixTimer.steppers[i].stepperId = STEPPER_ID_INVALID;
	}
}

static void _stop_flex_timer(const TimerId timerId)
{
	if(timerId >= FLEX_TIMER_COUNT)
	{
		return;
	}
	_flexTimers[timerId].state = TIMER_STATE_IDLE;
	HAL_TIM_Base_Stop_IT(_flexTimers[timerId].pTimerHandle);
	_flexTimers[timerId].stepperId = STEPPER_ID_INVALID;
}

TimerReturnCode timer_stop(const TimerId timerId)
{
	if(timerId >= TIMER_ID_COUNT)
	{
		return TIMER_ERROR_INVALID_ID;
	}

	if(timerId == FIX_TIMER_ID)
	{
		if(_fixTimer.state != TIMER_STATE_BUSY)
		{
			return TIMER_ERROR_WRONG_STATE;
		}
		_stop_fix_timer();
		
		return TIMER_OK;
	}

	if(_flexTimers[timerId].state != TIMER_STATE_BUSY)
	{
		return TIMER_ERROR_WRONG_STATE;
	}
	_stop_flex_timer(timerId);

	return TIMER_OK;
}

TimerReturnCode timer_set_prescaler(const TimerId timerId, const uint16_t prescaler)
{
	switch(timerId)
	{
		case FLEX_TIMER_ID_0:
		case FLEX_TIMER_ID_1:
		case FLEX_TIMER_ID_2:
		case FLEX_TIMER_ID_3:
		case FLEX_TIMER_ID_4:
		case FLEX_TIMER_ID_5:
			__HAL_TIM_SET_PRESCALER(_flexTimers[timerId].pTimerHandle, prescaler);
			return TIMER_OK;

		case FIX_TIMER_ID:
			__HAL_HRTIM_SETPERIOD(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A, prescaler);
			return TIMER_OK;

		default:
			return TIMER_ERROR_INVALID_ID;
	}
}

TimerReturnCode timer_get_prescaler(const TimerId timerId, uint16_t * const pPrescaler)
{
	if(pPrescaler == NULL)
	{
		return TIMER_ERROR_NULL_PARAMETER;
	}

	switch(timerId)
	{
		case FLEX_TIMER_ID_0:
		case FLEX_TIMER_ID_1:
		case FLEX_TIMER_ID_2:
		case FLEX_TIMER_ID_3:
		case FLEX_TIMER_ID_4:
		case FLEX_TIMER_ID_5:
			*pPrescaler = _flexTimers[timerId].pTimerHandle->Instance->PSC;
			return TIMER_OK;

		case FIX_TIMER_ID:
			*pPrescaler = __HAL_HRTIM_GETPERIOD(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
			return TIMER_OK;

		default:
			return TIMER_ERROR_INVALID_ID;
	}
}

TimerReturnCode timer_get_state(const TimerId timerId, TimerState * const pState)
{
	if(timerId >= TIMER_ID_COUNT)
	{
		return TIMER_ERROR_INVALID_ID;
	}
	if(pState == NULL)
	{
		return TIMER_ERROR_NULL_PARAMETER;
	}

	if(timerId == FIX_TIMER_ID)
	{
		*pState = _fixTimer.state;
		return TIMER_OK;
	}

	*pState = _flexTimers[timerId].state;
	return TIMER_OK;
}

uint16_t timer_get_max_flex_isr_period()
{
	return _maxFlexTimerIsrPeriod;
}

uint16_t timer_get_max_fix_isr_period()
{
	return _maxFixTimerIsrPeriod;
}

void timer_on_emergency()
{
	_stop_flex_timer(FLEX_TIMER_ID_0);
	_stop_flex_timer(FLEX_TIMER_ID_1);
	_stop_flex_timer(FLEX_TIMER_ID_2);
	_stop_flex_timer(FLEX_TIMER_ID_3);
	_stop_flex_timer(FLEX_TIMER_ID_4);
	_stop_flex_timer(FLEX_TIMER_ID_5);
	_stop_fix_timer();
}

static void _on_flex_timer(const TimerId timerId)
{
	FlexTimer * pTimer = _flexTimers + timerId;

	if(pTimer->isTesting == false)
	{
		if(pTimer->state != TIMER_STATE_BUSY)
		{
			return;
		}
		if(pTimer->stepperId == STEPPER_ID_INVALID)
		{
			return;
		}

		uint16_t newPulseWidth;
		const StepperReturnCode rc = on_interupt_stepper_pulse_end(pTimer->stepperId, &newPulseWidth);
		if(rc == STEPPER_OK)	
		{
			if(newPulseWidth == 0)
			{
				_stop_flex_timer(timerId);
			}
			else
			{
				__HAL_TIM_SET_AUTORELOAD(pTimer->pTimerHandle, newPulseWidth);
			}
		}
		else
		{
			print_log("Error: on_interupt_stepper_pulse_end() returned %d for stepper %d\r\n", rc, pTimer->stepperId);
			_stop_flex_timer(timerId);
		}

		uint16_t count = __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle);
		if(count > _maxFlexTimerIsrPeriod)
		{
			_maxFlexTimerIsrPeriod = count;
		}
	}
	else
	{
		if(pTimer->pulseIndexTesting >= pTimer->totalPulseTesting)
		{
			print_log("ERROR: FlexTimer: %d, pulse: %d/%d, ISR time: %d\r\n", timerId, pTimer->pulseIndexTesting, pTimer->totalPulseTesting, __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle));
		}

		if(pTimer->pulseIndexTesting == 0)
		{
			print_log("TEST start, FlexTimer: %d, pulse: %d/%d, ISR time: %d\r\n", timerId, pTimer->pulseIndexTesting, pTimer->totalPulseTesting, __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle));
		}
		if((pTimer->pulseIndexTesting % pTimer->logIntervalTesting) == 0)
		{
			print_log("TEST: FlexTimer: %d, pulse: %d/%d, ISR time: %d\r\n", timerId, pTimer->pulseIndexTesting, pTimer->totalPulseTesting, __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle));
		}

		pTimer->pulseIndexTesting++;

		if(pTimer->pulseIndexTesting == pTimer->totalPulseTesting)
		{
			print_log("TEST end, FlexTimer: %d, pulse: %d/%d, ISR time: %d\r\n", timerId, pTimer->pulseIndexTesting, pTimer->totalPulseTesting, __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle));
			_stop_flex_timer(timerId);
			pTimer->isTesting = false;

			_maxFlexTimerIsrPeriod = 0;
		}
		else
		{
			uint16_t count = __HAL_TIM_GET_COUNTER(pTimer->pTimerHandle);
			if(count > _maxFlexTimerIsrPeriod)
			{
				_maxFlexTimerIsrPeriod = count;
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim17))
	{
		_on_flex_timer(FLEX_TIMER_ID_5);
	}
	else if(htim == (&htim16))
	{
		_on_flex_timer(FLEX_TIMER_ID_4);
	}
	else if(htim == (&htim15))
	{
		_on_flex_timer(FLEX_TIMER_ID_3);
	}
	else if(htim == (&htim14))
	{
		_on_flex_timer(FLEX_TIMER_ID_2);
	}
	else if(htim == (&htim13))
	{
		_on_flex_timer(FLEX_TIMER_ID_1);
	}
	else if(htim == (&htim12))
	{
		_on_flex_timer(FLEX_TIMER_ID_0);
	}
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim,
                                              uint32_t TimerIdx)
{
	if(TimerIdx != HRTIM_TIMERINDEX_TIMER_A)
	{
		print_log("ERROR: invalid fix timer index: %d\r\n", TimerIdx);
		return;
	}

	if(_fixTimer.isTesting == false)
	{
		if(_fixTimer.state != TIMER_STATE_BUSY)
		{
			return;
		}

		for(int i=0; i<STEPPER_ID_COUNT; i++)
		{
			struct Stepper * pStepper = _fixTimer.steppers + i;

			if(pStepper->stepperId == STEPPER_ID_INVALID)
			{
				continue;
			}
			if(pStepper->firstPulseSkipped == false)
			{
				pStepper->firstPulseSkipped = true;
				continue;
			}

			if(pStepper->remainingPulseWidth > 1)
			{
				pStepper->remainingPulseWidth -= 1;
			}
			else
			{
				uint16_t newPulseWidth;
				const StepperReturnCode rc = on_interupt_stepper_pulse_end(pStepper->stepperId, &newPulseWidth);

				if(rc == STEPPER_OK)
				{
					if(newPulseWidth == 0)
					{
						// stepper doesn't need to be driven any more
						pStepper->stepperId = STEPPER_ID_INVALID;
					}
					else
					{
						pStepper->remainingPulseWidth = newPulseWidth;
					}
				}
				else
				{
					print_log("Error: on_interupt_stepper_pulse_end() returned %d for stepper %d\r\n", rc, pStepper->stepperId);
					pStepper->stepperId = STEPPER_ID_INVALID;
				}
			}
		}

		uint16_t count = __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
		if(count > _maxFixTimerIsrPeriod)
		{
			_maxFixTimerIsrPeriod = count;
		}
	}
	else
	{
		if(_fixTimer.pulseIndexTesting >= _fixTimer.totalPulseTesting)
		{
			print_log("ERROR: FixTimer pulse: %d/%d, ISR time: %d\r\n", _fixTimer.pulseIndexTesting, _fixTimer.totalPulseTesting, __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A));
		}

		if(_fixTimer.pulseIndexTesting == 0)
		{
			print_log("TEST start, FixTimer pulse: %d/%d, ISR time: %d\r\n", _fixTimer.pulseIndexTesting, _fixTimer.totalPulseTesting, __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A));
		}
		if((_fixTimer.pulseIndexTesting % _fixTimer.logIntervalTesting) == 0)
		{
			print_log("TEST: FixTimer pulse: %d/%d, ISR time: %d\r\n", _fixTimer.pulseIndexTesting, _fixTimer.totalPulseTesting, __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A));
		}

		_fixTimer.pulseIndexTesting++;

		if(_fixTimer.pulseIndexTesting == _fixTimer.totalPulseTesting)
		{
			print_log("TEST end, FixTimer pulse: %d/%d, ISR time: %d\r\n", _fixTimer.pulseIndexTesting, _fixTimer.totalPulseTesting, __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A));
			_fixTimer.isTesting = false;

			_maxFixTimerIsrPeriod = 0;
		}
		else
		{
			uint16_t count = __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
			if(count > _maxFixTimerIsrPeriod)
			{
				_maxFixTimerIsrPeriod = count;
			}
		}
	}
}
