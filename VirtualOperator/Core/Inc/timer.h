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
    FLEX_TIMER_0 = 0,
    FLEX_TIMER_1,
    FLEX_TIMER_2,
    FLEX_TIMER_3,
    FLEX_TIMER_4,
    FLEX_TIMER_5,
    FIX_TIMER,
    TIMER_COUNT,
    TIMER_INVALID_ID = 0xFF
} TimerId;

typedef enum
{
    TIMER_OK = 0,
    TIMER_INVALID_ID,
    TIMER_INVALID_STEPPER_ID,
    TIMER_STEPPER_DRIVEN_BY_OTHER,
    TIMER_NULL_PARAMETER
} TimerReturnCode;

typedef enum
{
    TIMER_UNINITIALIZED = 0,
    TIMER_IDLE,
    TIMER_BUSY
} TimerState;

void timer_init_data_structure();

TimerReturnCode timer_start(TimerId timerId, StepperId stepperId, uint16_t pulseWidth);
TimerReturnCode timer_stop(TimerId timerId);
TimerReturnCode timer_get_state(TimerId timerId, TimerState * pState);

void timer_on_emergency();

#endif /* INC_TIMER_H_ */
