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

static const FLEX_TIMER_COUNT = TIMER_COUNT - 1;

typedef struct 
{
	TimerState state;
	TIM_HandleTypeDef * pTimerHandle;
	StepperId stepperId;
} FlexTimer;

typedef struct
{
	TimerState state;
	HRTIM_HandleTypeDef * pTimerHandle;
	uint16_t fixPulseWidth_ns; // in nano second
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
	} steppers[STEPPER_COUNT];
} FixTimer;

static volatile FlexTimer _flexTimers[FLEX_TIMER_COUNT];
static volatile FixTimer _fixTimer;

static uint16_t _maxFlexTimerIsrPeriod;
static uint16_t _maxFixTimerIsrPeriod;

static inline uint32_t _get_fix_timer_interval_ns(void)
{
    /* Step-1: kernel clock that feeds the whole HRTIM engine */
    uint32_t f_hrtim = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_HRTIM1);
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

	uint32_t clockInterval = 1024 * 1024 * 1024 * ratio / f_hrtim; // GHz / (f_hrtim / ratio)
	uint32_t period = __HAL_HRTIM_GETPERIOD(&hhrtim, HRTIM_TIMERINDEX_TIMER_A);
	uint32_t interval_ns = clockInterval * period;

    return interval_ns;         
}

void timer_init_data_structure()
{
	volatile FlexTimer * pTimer;
	
	// flex timers. A flex timer can drive only 1 stepper.
	pTimer = _flexTimers;
	pTimer->pTimerHandle = &htim12;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	pTimer = _flexTimers + 1;
	pTimer->pTimerHandle = &htim13;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	pTimer = _flexTimers + 2;
	pTimer->pTimerHandle = &htim14;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	pTimer = _flexTimers + 3;
	pTimer->pTimerHandle = &htim15;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	pTimer = _flexTimers + 4;
	pTimer->pTimerHandle = &htim16;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	pTimer = _flexTimers + 5;
	pTimer->pTimerHandle = &htim17;
	pTimer->stepperId = STEPPER_INVALID_ID;
	pTimer->state = TIMER_IDLE;

	// fix timer. A fix timer can drive multiple steppers.
	_fixTimer.pTimerHandle = &hhrtim;
	_fixTimer.fixPulseWidth_ns = _get_fix_timer_interval_ns();
	print_log("Info: fix timer interval is set to %d ns\r\n", _fixTimer.fixPulseWidth_ns);
	for(int i=0; i<STEPPER_COUNT; i++)
	{
		_fixTimer.steppers[i].stepperId = STEPPER_INVALID_ID;
		_fixTimer.steppers[i].remainingPulseWidth = 0;
	}
	_fixTimer.state = TIMER_IDLE;

	_maxFlexTimerIsrPeriod = 0;
	_maxFixTimerIsrPeriod = 0;
}

TimerReturnCode timer_start(const TimerId timerId, const StepperId stepperId, const uint16_t pulseWidth)
{
	if(timerId >= TIMER_COUNT)
	{
		return TIMER_INVALID_ID;
	}
	if(stepperId >= STEPPER_COUNT)
	{
		return TIMER_INVALID_STEPPER_ID;
	}
	if(pulseWidth == 0)
	{
		return TIMER_INVALID_PULSE_WIDTH;
	}
	if(timerId < FIX_TIMER && _flexTimers[timerId].state != TIMER_IDLE)
	{
		return TIMER_IS_RUNNING;
	}

	bool stepperIsRunning = false;
	for(int i=0; i<(int)FIX_TIMER; i++)
	{
		volatile FlexTimer * pTimer = _flexTimers + i;
		if(pTimer->state != TIMER_BUSY)
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
		return TIMER_STEPPER_DRIVEN_BY_OTHER;
	}

	if(_fixTimer.state == TIMER_BUSY)
	{
		for(int i=0; i<(int)STEPPER_COUNT; i++)
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
		return TIMER_STEPPER_DRIVEN_BY_OTHER;
	}

	if(timerId == FIX_TIMER)
	{
		_fixTimer.steppers[stepperId].remainingPulseWidth = pulseWidth;
		_fixTimer.steppers[stepperId].firstPulseSkipped = false;
		_fixTimer.steppers[stepperId].stepperId = stepperId; // indicate the stepper is being clocked
		
		if(_fixTimer.state == TIMER_IDLE)
		{
			_fixTimer.state = TIMER_BUSY;
			HAL_StatusTypeDef rc = HAL_HRTIM_SimpleBaseStart_IT(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
			if(rc != HAL_OK)
			{
				_fixTimer.state = TIMER_IDLE;
				print_log("Error: timer_start(), failed to start fix timer, rc: %d\r\n", rc);
				return TIMER_INTERNAL_FAILURE;
			}
		}

		return TIMER_OK;
	}

	_flexTimers[timerId].stepperId = stepperId;
	__HAL_TIM_SET_AUTORELOAD(_flexTimers[timerId].pTimerHandle, pulseWidth);
	__HAL_TIM_SET_COUNTER(_flexTimers[timerId].pTimerHandle, 1);
	_flexTimers[timerId].state = TIMER_BUSY;
	HAL_StatusTypeDef rc = HAL_TIM_Base_Start_IT(_flexTimers[timerId].pTimerHandle);
	if(rc != HAL_OK)
	{
		_flexTimers[timerId].state = TIMER_IDLE;
		_flexTimers[timerId].stepperId = STEPPER_INVALID_ID;
		print_log("Error: timer_start(), failed to start timer: %d, rc: %d\r\n", timerId, rc);
		return TIMER_INTERNAL_FAILURE;
	}

	return TIMER_OK;	
}

static void _stop_fix_timer()
{
	_fixTimer.state = TIMER_IDLE;
	HAL_HRTIM_SimpleBaseStop_IT(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
	
	for(int i=0; i<STEPPER_COUNT; i++)
	{
		_fixTimer.steppers[i].stepperId = STEPPER_INVALID_ID;
	}
}

static void _stop_flex_timer(const TimerId timerId)
{
	if(timerId >= FLEX_TIMER_COUNT)
	{
		return;
	}
	_flexTimers[timerId].state = TIMER_IDLE;
	HAL_TIM_Base_Stop_IT(_flexTimers[timerId].pTimerHandle);
	_flexTimers[timerId].stepperId = STEPPER_INVALID_ID;
}

TimerReturnCode timer_stop(const TimerId timerId)
{
	if(timerId >= TIMER_COUNT)
	{
		return TIMER_INVALID_ID;
	}

	if(timerId == FIX_TIMER)
	{
		if(_fixTimer.state != TIMER_BUSY)
		{
			return TIMER_WRONG_STATE;
		}
		_stop_fix_timer();
		
		return TIMER_OK;
	}

	if(_flexTimers[timerId].state != TIMER_BUSY)
	{
		return TIMER_WRONG_STATE;
	}
	_stop_flex_timer(timerId);

	return TIMER_OK;
}

TimerReturnCode timer_get_state(const TimerId timerId, TimerState * const pState)
{
	if(timerId >= TIMER_COUNT)
	{
		return TIMER_INVALID_ID;
	}
	if(pState == NULL)
	{
		return TIMER_NULL_PARAMETER;
	}

	if(timerId == FIX_TIMER)
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
	_stop_flex_timer(FLEX_TIMER_0);
	_stop_flex_timer(FLEX_TIMER_1);
	_stop_flex_timer(FLEX_TIMER_2);
	_stop_flex_timer(FLEX_TIMER_3);
	_stop_flex_timer(FLEX_TIMER_4);
	_stop_flex_timer(FLEX_TIMER_5);
	_stop_fix_timer();
}

static void _on_flex_timer(const TimerId timerId)
{
	FlexTimer * pTimer = _flexTimers + timerId;

	if(pTimer->state != TIMER_BUSY)
	{
		return;
	}
	if(pTimer->stepperId == STEPPER_INVALID_ID)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == (&htim17))
	{
		_on_flex_timer(FLEX_TIMER_5);
	}
	else if(htim == (&htim16))
	{
		_on_flex_timer(FLEX_TIMER_4);
	}
	else if(htim == (&htim15))
	{
		_on_flex_timer(FLEX_TIMER_3);
	}
	else if(htim == (&htim14))
	{
		_on_flex_timer(FLEX_TIMER_2);
	}
	else if(htim == (&htim13))
	{
		_on_flex_timer(FLEX_TIMER_1);
	}
	else if(htim == (&htim12))
	{
		_on_flex_timer(FLEX_TIMER_0);
	}
}

void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim,
                                              uint32_t TimerIdx)
{
	if(TimerIdx != HRTIM_TIMERINDEX_TIMER_A)
	{
		return;
	}
	if(_fixTimer.state != TIMER_BUSY)
	{
		return;
	}

	bool stopTimer = true;
	
	for(int i=0; i<STEPPER_COUNT; i++)
	{
		struct Stepper * pStepper = _fixTimer.steppers + i;

		if(pStepper->stepperId == STEPPER_INVALID_ID)
		{
			continue;
		}
		if(pStepper->firstPulseSkipped == false)
		{
			pStepper->firstPulseSkipped = true;
			stopTimer = false;
			continue;
		}

		if(pStepper->remainingPulseWidth > 1)
		{
			stopTimer = false;
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
					pStepper->stepperId == STEPPER_INVALID_ID;
				}
				else
				{
					stopTimer = false;
					pStepper->remainingPulseWidth = newPulseWidth;
				}
			}
			else
			{
				print_log("Error: on_interupt_stepper_pulse_end() returned %d for stepper %d\r\n", rc, pStepper->stepperId);
				pStepper->stepperId == STEPPER_INVALID_ID;
			}
		}
	}

	if(stopTimer)
	{
		// no stepper needs to be clocked
		_stop_fix_timer();
	}

	uint16_t count = __HAL_HRTIM_GETCOUNTER(_fixTimer.pTimerHandle, HRTIM_TIMERINDEX_TIMER_A);
	if(count > _maxFixTimerIsrPeriod)
	{
		_maxFixTimerIsrPeriod = count;
	}
}