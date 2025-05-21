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


/**
 * 10 steppers at most can be driven.
 * Steppers can run in one of 3 mode:
 *   - master
 *   - slave
 *   - forced
 * 
 * In master mode, the stepper is driven by a timer. The clock signal
 * of the stepper has 2 phase, the first and second phase, in different
 * voltage level. The phase width is actually pulse width of the clock, 
 * identified by the value of the auto-reload register in the timer.  
 * The timer starts with the first phase, calls on_pulse_end()
 * at the end of the phase, then starts with the second phase, then 
 * calls on_pulse_end(), then starts with the first phase of next step,
 * and so on. The master stepper can drive its slave steppers when it
 * runs to the pre-set steps. The master stepper notifies slave steppers
 * of its current step index and pulse state. When the step index matches
 * one of the pre-set indexes, the slave stepper sets its clock signal 
 * accordingly.
 * 
 * In slave mode, the stepper is driven by its master stepper, through
 * checking master stepper's step index and pulse state. The combination
 * of master and slaves can achieve coordinated movement among more than
 * one axises.
 * 
 * In forced mode, stepper can only run a small range. The purpose of 
 * this mode is to move steppers manually when something goes wrong, 
 * for example, an emergency accident. Some wormgears are driven by
 * steppers, they cannot be moved by pushing or pulling.
 * 
 * The first function to call is stepper_init_data_structure().
 * 
 * To make a stepper run in master mode:
 *      - set_stepper_controls()
 *      - set_master_stepper_cruise_pulse_width
 *      - set_master_stepper_rampup_pulse_widths
 *      - set_master_stepper_rampdown_pulse_widths
 *      - start_stepper_home_positioning
 *      - set_stepper_forward
 *      - set_stepper_enable
 *      - run_master_stepper
 *      - get_current_pulse_width
 *      - start timer
 * 
 * To make a stepper run in slave mode:
 *      - set_stepper_controls()
 *      - set_master_stepper_cruise_pulse_width
 *      - set_master_stepper_rampup_pulse_widths
 *      - set_master_stepper_rampdown_pulse_widths
 *      - start_stepper_home_positioning
 *      - set_stepper_forward
 *      - set_stepper_enable
 *      - set_slave_stepper_step_indexes
 *      - couple_slave_stepper
 * 
 * To make a slave stepper back to master:
 *      - decouple_slave_stepper
 *      - set_master_stepper_cruise_pulse_width
 *      - set_master_stepper_rampup_pulse_widths
 *      - set_master_stepper_rampdown_pulse_widths
 *      - run_master_stepper
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
    STEPPER_COUNT
} StepperId;

typedef enum 
{
    ENCODER_0 = 0,
    ENCODER_1,
    ENCODER_2,
    ENCODER_3,
    ENCODER_4,
    ENCODER_5,
    ENCODER_6,
    ENCODER_7
} EncoderId;

typedef enum 
{
    STEPPER_OK = 0,
    STEPPER_INVALID_ID,
    STEPPER_WRONG_PULSE_WIDTH,
    STEPPER_TOO_MANY_PULSE_WIDTHS,
    STEPPER_TOO_MANY_SLAVE_INDEXES,
    STEPPER_WRONG_STATE,
    STEPPER_STEPS_OUT_OF_RANGE,
    STEPPER_INVALID_SLAVE_ID,
    STEPPER_INVALID_ENCODER_ID,
    STEPPER_NULL_PARAMETER
} StepperReturnCode;

typedef enum 
{
    UNINITIALIZED = 0,  // data about stepper hasn't been initialized
    INITIALIZED,        // all data about stepper has been initialized
    RETURN_TO_HOME,
    HOME_TO_READY,
    READY,              // the stepper is ready to be clocked
    RUNNING_MASTER,
    RUNNING_SLAVE,
    RUNNING_FORCED,
    OUT_OF_BOUND,       // the stepper has touched its boundary
    DRIVER_ALARM        // the stepper driver activates alarm signal
} StepperState;

void stepper_init_data_structure();

StepperReturnCode stepper_set_controls(
    StepperId _id,
    bool _isRisingEdgeDriven,
    GPIO_TypeDef * _pGpioPortHomeBoundary,
    uint8_t _gpioPinIndexHomeBoundary,
    GPIO_TypeDef * _pGpioPortEndBoundary,
    uint8_t _gpioPinIndexEndBoundary,
    GPIO_TypeDef * _pGpioPortEnable,
    uint8_t _gpioPinIndexEnable,
    GPIO_TypeDef * _pGpioPortForward,
    uint8_t _gpioPinIndexForward,
    GPIO_TypeDef * _pGpioPortClock,
    uint8_t _gpioPinIndexClock,
    uint32_t _maxSteps
);

StepperReturnCode stepper_set_forward(StepperId id, bool isForward);
StepperReturnCode stepper_set_enable(StepperId id, bool isEnable);

StepperReturnCode stepper_set_master_cruise_pulse_width(StepperId id, uint16_t width);
StepperReturnCode stepper_set_master_rampup_pulse_widths(StepperId id, uint16_t * pWidths, uint8_t count, uint8_t batchIndex, uint8_t totalBatches);
StepperReturnCode stepper_set_master_rampdown_pulse_widths(StepperId id, uint16_t * pWidths, uint8_t count, uint8_t batchIndex, uint8_t totalBatches);
StepperReturnCode stepper_set_slave_step_indexes(StepperId id, uint16_t * pIndexes, uint8_t count, uint8_t batchIndex, uint8_t totalBatches);

StepperReturnCode stepper_start_home_positioning(StepperId id);

StepperReturnCode stepper_run_master(StepperId id, uint32_t steps);

StepperReturnCode stepper_couple_slave(StepperId masterStepperId, StepperId slaveStepperId);
StepperReturnCode stepper_decouple_slave(StepperId masterStepperId, StepperId slaveStepperId);

StepperReturnCode stepper_run_force(StepperId id, uint16_t pulseWidth, uint8_t steps);

StepperReturnCode stepper_couple_encoder(StepperId stepperId, uint16_t stepsPerREvolution, EncoderId encoderId, uint16_t countsPerRevolution);
StepperReturnCode stepper_decouple_encoder(StepperId stepperId, EncoderId encoderId);

StepperReturnCode stepper_get_state(StepperId id, StepperState * pState);

// interface with timer
StepperReturnCode stepper_get_current_pulse_width(StepperId id, uint16_t * pPulseWidth);
StepperReturnCode on_stepper_pulse_end(StepperId id, uint16_t * pNextPulseWidth);

#endif /* INC_STEPPER_H_ */
