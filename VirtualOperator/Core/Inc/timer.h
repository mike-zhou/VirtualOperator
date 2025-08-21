/*
 * timer.h
 *
 *  Created on: May 20, 2025
 *      Author: mike
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_

#include <stdint.h>
#include <stdbool.h>

#include "stepper.h"

typedef enum 
{
    FLEX_TIMER_ID_0 = 0,
    FLEX_TIMER_ID_1,
    FLEX_TIMER_ID_2,
    FLEX_TIMER_ID_3,
    FLEX_TIMER_ID_4,
    FLEX_TIMER_ID_5,
    FIX_TIMER_ID,
    TIMER_ID_COUNT,
    TIMER_ID_INVALID = 0xFF
} TimerId;

typedef enum
{
    TIMER_OK = 0,
    TIMER_ERROR_INVALID_ID,
    TIMER_ERROR_INVALID_STEPPER_ID,
    TIMER_ERROR_INVALID_PULSE_WIDTH,
    TIMER_ERROR_ALREADY_RUNNING,
    TIMER_ERROR_STEPPER_DRIVEN_BY_OTHER,
    TIMER_ERROR_NULL_PARAMETER,
    TIMER_ERROR_INTERNAL_FAILURE,
    TIMER_ERROR_WRONG_STATE
} TimerReturnCode;

typedef enum
{
    TIMER_STATE_UNINITIALIZED = 0,
    TIMER_STATE_IDLE,
    TIMER_STATE_BUSY
} TimerState;

void timer_init_data_structure();

TimerReturnCode timer_start(const TimerId timerId, const StepperId stepperId, const uint16_t pulseWidth);
TimerReturnCode timer_test(const TimerId timerId, const uint16_t pulseWidth, const uint16_t totalPulse, const uint16_t logInterval);
TimerReturnCode timer_stop(const TimerId timerId);
TimerReturnCode timer_set_prescaler(const TimerId timerId, const uint16_t prescaler);
TimerReturnCode timer_get_prescaler(const TimerId timerId, uint16_t * const pPrescaler);
TimerReturnCode timer_get_state(const TimerId timerId, TimerState * const pState);

uint16_t timer_get_max_flex_isr_period();
uint16_t timer_get_max_fix_isr_period();

void timer_on_emergency();

#endif /* INC_TIMER_H_ */
