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
    FLEX_PULSE_TIMER_0 = 0,
    FLEX_PULSE_TIMER_1,
    FLEX_PULSE_TIMER_2,
    FLEX_PULSE_TIMER_3,
    FLEX_PULSE_TIMER_4,
    FLEX_PULSE_TIMER_5,
    FIX_PULSE_TIMER,
    TIMER_COUNT,
} TimerId;

typedef enum
{
    TIMER_OK = 0,
    TIMER_INVALID_ID,
    TIMER_INVALID_STEPPER_ID,
} TimerReturnCode;


void timer_init_data_structure();

TimerReturnCode timer_start(TimerId timerId, StepperId stepperId, uint16_t pulseWidth);
TimerReturnCode timer_stop(TimerId timerId);
bool is_timer_running(TimerId timerId);

#endif /* INC_TIMER_H_ */
