/*
 * stepper.h
 *
 *  Created on: May 4, 2025
 *      Author: mike
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include <stdint.h>
#include <stdbool.h>

#include "encoder.h"

/**
 * 10 steppers at most can be driven.
 * Steppers can run in one of 3 mode:
 *   - active
 *   - passive
 *   - forced
 * 
 * In active mode, the stepper is driven by a timer. The clock signal
 * of the stepper has 2 phase, the first and second phase, in different
 * voltage level. The phase width is actually pulse width of the clock, 
 * identified by the value of the auto-reload register in the timer.  
 * The timer starts with the first phase, calls on_pulse_end()
 * at the end of the phase, then starts with the second phase, then 
 * calls on_pulse_end(), then starts with the first phase of next step,
 * and so on. The active stepper can drive its passive steppers when it
 * runs to the pre-set steps. The active stepper notifies passive steppers
 * of its current step index and pulse state. When the step index matches
 * one of the pre-set indexes, the passive stepper sets its clock signal 
 * accordingly.
 * 
 * In passive mode, the stepper is driven by its active stepper, through
 * checking active stepper's step index and pulse state. The combination
 * of active and passives can achieve coordinated movement among more than
 * one axises.
 * 
 * In forced mode, stepper can only run a small range. The purpose of 
 * this mode is to move steppers manually when something goes wrong, 
 * for example, an emergency accident. Some wormgears are driven by
 * steppers, they cannot be moved by pushing or pulling.
 * 
 * The first function to call is stepper_init_data_structure().
 * 
 * To make a stepper run in active mode:
 *      - set_stepper_controls()
 *      - set_active_stepper_cruise_pulse_width
 *      - set_active_stepper_rampup_pulse_widths
 *      - set_active_stepper_rampdown_pulse_widths
 *      - start_stepper_home_positioning
 *      - set_stepper_forward
 *      - set_stepper_enable
 *      - run_active_stepper
 *      - get_current_pulse_width
 *      - start timer
 * 
 * To make a stepper run in passive mode:
 *      - set_stepper_controls()
 *      - set_active_stepper_cruise_pulse_width
 *      - set_active_stepper_rampup_pulse_widths
 *      - set_active_stepper_rampdown_pulse_widths
 *      - start_stepper_home_positioning
 *      - set_stepper_forward
 *      - set_stepper_enable
 *      - set_passive_stepper_step_indexes
 *      - couple_passive_stepper
 * 
 * To make a passive stepper back to active:
 *      - decouple_passive_stepper
 *      - set_active_stepper_cruise_pulse_width
 *      - set_active_stepper_rampup_pulse_widths
 *      - set_active_stepper_rampdown_pulse_widths
 *      - run_active_stepper
 *      - get_current_pulse_width
 *      - start timer
 * 
 * To run stepper in force mode:
 *      - run_force_stepper
 *      - get_current_pulse_width
 *      - start timer     
*/


typedef enum 
{
    STEPPER_0 = 0,
    STEPPER_1,
    STEPPER_2,
    STEPPER_3,
    STEPPER_4,
    STEPPER_5,
    STEPPER_6,
    STEPPER_7,
    STEPPER_8,
    STEPPER_9,
    STEPPER_COUNT,
    STEPPER_INVALID_ID = 0xFF
} StepperId;

typedef enum 
{
    STEPPER_OK = 0,
    STEPPER_INVALID_ID,
    STEPPER_INVALID_GPIO_PORT,
    STEPPER_INVALID_GPIO_PIN_INDEX,
    STEPPER_INVALID_READY_STEPS,
    STEPPER_INVALID_RANGE,
    STEPPER_INVALID_ENCODER_ID,
    STEPPER_INVALID_CONTROL_PARAMETER,
    STEPPER_INVALID_PASSIVE_ID,
    STEPPER_NO_PULSE,
    STEPPER_WRONG_BATCH_INDEX,
    STEPPER_WRONG_PULSE_WIDTH,
    STEPPER_WRONG_PULSE_ORDER,
    STEPPER_INTERNAL_DATA_ERROR,
    STEPPER_TOO_MANY_PULSE_WIDTHS,
    STEPPER_TOO_MANY_PASSIVE_INDEXES,
    STEPPER_WRONG_INIT_ORDER,
    STEPPER_WRONG_STATE,
    STEPPER_INVALID_ACTIVE_PULSES,
    STEPPER_INVALID_PASSIVE_INDEXES,
    STEPPER_WILL_OUT_OF_RANGE,
    SETPPER_ALREADY_COUPLED,
    STEPPER_NOT_COUPLED,
    STEPPER_NULL_PARAMETER,
    STEPPER_NOT_SUPPORT_SYNC,
    STEPPER_OUT_OF_STEPS,
    STEPPER_UN_SYNC,
    STEPPER_OUT_OF_RANGE,
    STEPPER_MISSED_ACTIVE_PULSE
} StepperReturnCode;

typedef enum 
{
    STEPPER_UNINITIALIZED = 0,  // data about stepper hasn't been initialized
    STEPPER_INITIALIZED,        // all data about stepper has been initialized
    STEPPER_RETURN_TO_HOME_BOUNDARY,
    STEPPER_HOME_BOUNDARY_TO_READY,
    STEPPER_READY,              // the stepper is ready to be clocked
    STEPPER_RUNNING_ACTIVE,
    STEPPER_RUNNING_PASSIVE,
    STEPPER_RUNNING_FORCED,
    STEPPER_OUT_OF_SYNC,        
    STEPPER_OUT_OF_BOUNDARY,    // boundary detector is triggered
    STEPPER_DRIVER_ALARM,       // the stepper driver activates alarm signal
    STEPPER_STATE_COUNT
} StepperState;

void stepper_init_data_structure();

/**
 * If there are several batches of pulses, then the pulse count in 
 * each batch must be same except for the last batch.
 * For example, if there are 5 batches, then pulse count could be
 * 200, 200, 200, 200, 150.
 */
StepperReturnCode stepper_set_active_rampup_pulse_widths(
    const StepperId id, 
    const uint16_t * pWidths, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches);

StepperReturnCode stepper_set_active_cruise_pulse_width(
    const StepperId id, const uint16_t width);

StepperReturnCode stepper_set_active_rampdown_pulse_widths(
    const StepperId id, 
    const uint16_t * pWidths, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches);

StepperReturnCode stepper_set_passive_step_indexes(
    const StepperId id, 
    const uint16_t * pIndexes, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches);

StepperReturnCode stepper_set_controls(
    const StepperId id,
    const bool isRisingEdgeDriven,
    const bool isForwardHigh,
    const bool isEnableHigh,
    const GPIO_TypeDef * pGpioPortHomeBoundary,
    const uint8_t gpioPinIndexHomeBoundary,
    const GPIO_TypeDef * pGpioPortEndBoundary,
    const uint8_t gpioPinIndexEndBoundary,
    const GPIO_TypeDef * pGpioPortEnable,
    const uint8_t gpioPinIndexEnable,
    const GPIO_TypeDef * pGpioPortForward,
    const uint8_t gpioPinIndexForward,
    const GPIO_TypeDef * pGpioPortClock,
    const uint8_t gpioPinIndexClock,
    const uint16_t homeBoundaryToReadySteps,
    const uint32_t range,
    const uint16_t stepsPerRevolution,
    const EncoderId encoderId,
    const uint16_t countsPerRevolution,
    const uint16_t maxPositionError
);

StepperReturnCode stepper_set_forward(const StepperId id, const bool isForward);
StepperReturnCode stepper_set_enable(const StepperId id, const bool isEnable);

StepperReturnCode stepper_start_home_positioning(const StepperId id);

StepperReturnCode stepper_run_active(const StepperId id, const uint32_t steps);

StepperReturnCode stepper_couple_passive(const StepperId activeStepperId, const StepperId passiveStepperId);
StepperReturnCode stepper_decouple_passive(const StepperId activeStepperId, const StepperId passiveStepperId);

StepperReturnCode stepper_run_force(const StepperId id, const uint16_t pulseWidth, const uint8_t steps);

StepperReturnCode stepper_get_state(const StepperId id, StepperState * const pState);

/**
 * check if stepper gets out of sync when it is NOT moving.
 */
StepperReturnCode stepper_check_sync(const StepperId id, bool * const pInSync);

/**
 *  interface with timer
*/
StepperReturnCode stepper_get_startup_pulse_width(const StepperId id, uint16_t * const pPulseWidth);

// this function is called when the clock pulse finishes, pNextPulseWidth returns the width of next pulse.
// if this function doesn't return STEPPER_OK or a pulse width is zero, then this stepper shouldn't be driven any more.
StepperReturnCode on_interupt_stepper_pulse_end(const StepperId id, uint16_t * const pNextPulseWidth);

#endif /* INC_STEPPER_H_ */
