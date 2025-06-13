/*
 * stepper.c
 *
 *  Created on: May 4, 2025
 *      Author: mike
 */

#include "stepper.h"
#include "usart1.h"

#include "stm32h7xx_hal.h"


#define MAX_UINT16_ARRAY_LENGTH 4096
#define MAX_RAMP_CLOCKS (MAX_UINT16_ARRAY_LENGTH >> 1)

typedef enum _PulseState
{
    FIRST_HALF,          
    SECOND_HALF         
} PulseState;

 typedef struct _Stepper
{
    /**
     * static data
     */
    // stepper controls
    bool isRisingEdgeDriven;
    bool isForwardHigh;
    bool isEnableHigh;
    GPIO_TypeDef * pGpioPortHomeBoundary;
    uint8_t gpioPinIndexHomeBoundary;
    GPIO_TypeDef * pGpioPortEndBoundary;
    uint8_t gpioPinIndexEndBoundary;
    GPIO_TypeDef * pGpioPortEnable;
    uint8_t gpioPinIndexEnable;
    GPIO_TypeDef * pGpioPortForward;
    uint8_t gpioPinIndexForward;
    GPIO_TypeDef * pGpioPortClock;
    uint8_t gpioPinIndexClock;
    uint16_t homeBoundaryToReadySteps; 
    uint32_t range;
    uint16_t stepsPerRevolution;
    EncoderId encoderId;
    uint16_t countsPerRevolution;
    uint16_t maxPositionError;
    bool isStepperControlInitialized;

    // uint16_t data array
    uint16_t uint16Array[MAX_UINT16_ARRAY_LENGTH];

    // array of pulse width for the stepper to speed up from still to cruising
    uint16_t * pRampupPulseWidths;
    // length of widthRampUp
    uint32_t rampupPulses;
    bool isRampupPuleseWidthsPopulated;

    // array of pulse width for the stepper to slow down from curising to still
    uint16_t * pRampdownPulseWidths;
    // length of widthRampDown
    uint32_t rampdownPulses;
    bool isRampdownPulseWidthsPopulated;

    // pulse width when the stepper is cruising
    uint16_t cruisePulseWidth;
    bool isCruisePulseWidthPopulated;

    // indexes for passive steppers coupled 
    uint16_t * pPassiveStepArray;
    uint32_t passiveStepsCount;
    uint32_t passiveStepIndex;
    bool isPassiveStepsInitialized;

    /**
     * dynamic data
     */

    StepperState state;

    bool isEnabled;
    bool isForward;
    uint32_t offset;

    int32_t actualPosition;
    uint16_t encoderCount;

    uint32_t stepsToRun;
    uint32_t currentStep;
    uint16_t currentPulseWidth;

    PulseState pulseState;

    uint16_t forcePulseWidth;
    
    StepperId passiveStepperIds[STEPPER_COUNT];
    bool passiveCoupled;
} StepperData;

static volatile StepperData _steppers[STEPPER_COUNT];

static void _set_clock_pulse_level_first(StepperData * const pStepper)
{
    GPIO_PinState pinState = GPIO_PIN_SET;

    if(pStepper->isRisingEdgeDriven == true)
    {
        pinState = GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortClock, 1 << pStepper->gpioPinIndexClock, pinState);
}

static void _set_clock_pulse_level_second(StepperData * const pStepper)
{
    GPIO_PinState pinState = GPIO_PIN_RESET;

    if(pStepper->isRisingEdgeDriven == true)
    {
        pinState = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortClock, 1 << pStepper->gpioPinIndexClock, pinState);
}

/**
 * This function is called by the active stepper to notify the passive stepper 
 * of its current stepIndex.
 * If stepIndex is what the passive expected, the passive needs to drive its
 * clock pin according to pulseState.
 */
static StepperReturnCode _on_active_stepper_pulse_end(
    const StepperId passiveStepperId, 
    const uint32_t activeStepIndex, 
    const PulseState activePulseState)
{
    if(passiveStepperId >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    
    StepperData * pPassive = _steppers + (int)passiveStepperId;

    if(pPassive->state == STEPPER_RUNNING_PASSIVE)
    {
        if(pPassive->passiveStepIndex >= pPassive->passiveStepsCount)
        {
            return STEPPER_INTERNAL_DATA_ERROR;
        }

        uint32_t expectedActiveStepIndex = pPassive->pPassiveStepArray[pPassive->passiveStepIndex];

        if(activeStepIndex < expectedActiveStepIndex)
        {
            return STEPPER_OK;
        }
        if(activeStepIndex > expectedActiveStepIndex)
        {
            return STEPPER_MISSED_ACTIVE_PULSE;
        }

        if(activePulseState == FIRST_HALF)
        {
            _set_clock_pulse_level_second(pPassive);
        }
        else
        {
            _set_clock_pulse_level_first(pPassive);

            pPassive->passiveStepIndex++;
            if(pPassive->passiveStepIndex == pPassive->passiveStepsCount)
            {
                pPassive->state = STEPPER_READY;
            }
        }

        return STEPPER_OK;
    }
    else if(pPassive->state == STEPPER_READY)
    {
        if(pPassive->passiveStepIndex != pPassive->passiveStepsCount)
        {
            return STEPPER_INTERNAL_DATA_ERROR;
        }

        return STEPPER_OK;
    }

    return STEPPER_WRONG_STATE;
}

static StepperReturnCode _notify_passive_steppers(StepperData * const pStepper)
{
    if(pStepper->passiveCoupled == false)
    {
        return STEPPER_OK;
    }

    StepperReturnCode rc;

    for(int i = 0; i < STEPPER_COUNT; i++)
    {
        if(pStepper->passiveStepperIds[i] == STEPPER_INVALID_ID)
        {
            continue;
        }

        rc = _on_active_stepper_pulse_end(pStepper->passiveStepperIds[i], pStepper->currentStep, pStepper->pulseState);
        if(rc != STEPPER_OK)
        {
            return rc;
        }
    }

    return STEPPER_OK;
}

static uint16_t _is_static_data_initialized(StepperId id)
{
    StepperData * pStepper = _steppers + (int)id;

    if(!pStepper->isStepperControlInitialized)
    {
        return false;
    }
    
    if(pStepper->isPassiveStepsInitialized)
    {
        return true;
    }
    
    if(pStepper->isRampupPuleseWidthsPopulated &&
        pStepper->isRampdownPulseWidthsPopulated &&
        pStepper->isCruisePulseWidthPopulated)
    {
        return true;
    }

    return false;
}

void stepper_init_data_structure()
{
    for(uint8_t stepperIndex=0; stepperIndex<STEPPER_COUNT; stepperIndex++)
    {
        StepperData * pStepper = _steppers + stepperIndex;

        // static data
        pStepper->isStepperControlInitialized = false;
        pStepper->isRampupPuleseWidthsPopulated = false;
        pStepper->isRampdownPulseWidthsPopulated = false;
        pStepper->isCruisePulseWidthPopulated = false;
        pStepper->isPassiveStepsInitialized = false;

        // dynamic data
        pStepper->state = STEPPER_UNINITIALIZED;
        pStepper->isEnabled = false;
        pStepper->offset = 0;
        pStepper->actualPosition = 0;
        pStepper->encoderCount = 0;
        pStepper->stepsToRun = 0;
        pStepper->currentStep = 0;
        pStepper->pulseState = FIRST_HALF;
        pStepper->forcePulseWidth = 0xFFFF;
        for(uint8_t passiveIndex=0; passiveIndex<STEPPER_COUNT; passiveIndex++)
        {
            pStepper->passiveStepperIds[passiveIndex] = STEPPER_INVALID_ID;
        }
        pStepper->passiveCoupled = false;
    }
}

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
    const uint16_t maxPositionError)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(pGpioPortHomeBoundary == NULL ||
        pGpioPortEndBoundary == NULL ||
        pGpioPortEnable == NULL ||
        pGpioPortForward == NULL ||
        pGpioPortClock == NULL)
    {
        return STEPPER_INVALID_GPIO_PORT;
    }
    if(gpioPinIndexHomeBoundary > 15 ||
        gpioPinIndexEndBoundary > 15 ||
        gpioPinIndexEnable > 15 ||
        gpioPinIndexForward > 15 ||
        gpioPinIndexClock > 15)
    {
        return STEPPER_INVALID_GPIO_PIN_INDEX;
    }
    if(homeBoundaryToReadySteps == 0)
    {
        return STEPPER_INVALID_READY_STEPS;
    }
    if(range == 0)
    {
        return STEPPER_INVALID_RANGE;
    }
    if(encoderId >= ENCODER_COUNT && encoderId != ENCODER_INVALID_ID)
    {
        return STEPPER_INVALID_ENCODER_ID;
    }
    if(stepsPerRevolution == 0 || countsPerRevolution == 0)
    {
        return STEPPER_INVALID_CONTROL_PARAMETER;
    }
    if(maxPositionError < 1)
    {
        return STEPPER_INVALID_CONTROL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_UNINITIALIZED)
    {
        return STEPPER_WRONG_STATE;
    }

    pStepper->isRisingEdgeDriven = isRisingEdgeDriven;
    pStepper->isForwardHigh = isForwardHigh;
    pStepper->isEnableHigh = isEnableHigh;
    pStepper->pGpioPortHomeBoundary = pGpioPortHomeBoundary;
    pStepper->gpioPinIndexHomeBoundary = gpioPinIndexHomeBoundary;
    pStepper->gpioPinIndexEndBoundary = gpioPinIndexEndBoundary;
    pStepper->pGpioPortEnable = pGpioPortEnable;
    pStepper->gpioPinIndexEnable = gpioPinIndexEnable;
    pStepper->pGpioPortForward = pGpioPortForward;
    pStepper->gpioPinIndexForward = gpioPinIndexForward;
    pStepper->pGpioPortClock = pGpioPortClock;
    pStepper->gpioPinIndexClock = gpioPinIndexClock;
    pStepper->homeBoundaryToReadySteps = homeBoundaryToReadySteps;
    pStepper->range = range;
    pStepper->stepsPerRevolution = stepsPerRevolution;
    pStepper->encoderId = encoderId;
    pStepper->countsPerRevolution = countsPerRevolution;
    pStepper->maxPositionError = maxPositionError;
    pStepper->isStepperControlInitialized = true;

    if(!_is_static_data_initialized(id))
    {
        return STEPPER_WRONG_INIT_ORDER;
    }
    pStepper->state = STEPPER_INITIALIZED;

    return STEPPER_OK;
}

static void _stepper_set_forward(StepperData * const pStepper, const bool isForward)
{
    GPIO_PinState pinState;

    if(isForward)
    {
        if(pStepper->isForwardHigh)
            pinState = GPIO_PIN_SET;
        else
            pinState = GPIO_PIN_RESET;
    }
    else
    {
        if(pStepper->isForwardHigh)
            pinState = GPIO_PIN_RESET;
        else
            pinState = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortForward, 1 << pStepper->gpioPinIndexForward, pinState);
    pStepper->isForward = isForward;
}

StepperReturnCode stepper_set_forward(const StepperId id, const bool isForward)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_INITIALIZED &&
        pStepper->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }

    if(isForward)
    {
        if(pStepper->offset + pStepper->stepsToRun > pStepper->range)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }
    else
    {
        if(pStepper->offset < pStepper->stepsToRun)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }

    _stepper_set_forward(pStepper, isForward);

    return STEPPER_OK;
}

StepperReturnCode stepper_set_enable(const StepperId id, const bool isEnable)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_INITIALIZED &&
        pStepper->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(pStepper->state == STEPPER_READY)
    {
        pStepper->state = STEPPER_INITIALIZED; // stepper lose its position if enabled or disabled
    }

    GPIO_PinState pinState;

    if(isEnable)
    {
        if(pStepper->isEnableHigh)
            pinState = GPIO_PIN_SET;
        else
            pinState = GPIO_PIN_RESET;
    }
    else
    {
        if(pStepper->isEnableHigh)
            pinState = GPIO_PIN_RESET;
        else
            pinState = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortEnable, 1 << pStepper->gpioPinIndexEnable, pinState);
    pStepper->isEnabled = isEnable;

    return STEPPER_OK;
}

static void _reset_active_stepper_pulses(const StepperId id)
{
    StepperData * pStepper = _steppers + (int)id;

    pStepper->pRampdownPulseWidths = pStepper->uint16Array;
    pStepper->rampupPulses = 0;
    pStepper->isRampupPuleseWidthsPopulated = false;

    pStepper->isCruisePulseWidthPopulated = false;

    pStepper->pRampdownPulseWidths = pStepper->uint16Array + MAX_RAMP_CLOCKS;
    pStepper->rampdownPulses = 0;
    pStepper->isRampdownPulseWidthsPopulated = false;

    pStepper->pPassiveStepArray = NULL;
    pStepper->passiveStepsCount = 0;
    pStepper->passiveStepIndex = 0;
    pStepper->isPassiveStepsInitialized = false;
}

StepperReturnCode stepper_set_active_rampup_pulse_widths(
    const StepperId id, 
    const uint16_t * pWidths, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(NULL == pWidths)
    {
        return STEPPER_NULL_PARAMETER;
    }
    if(count < 1)
    {
        return STEPPER_NO_PULSE;
    }
    if(totalBatches < 1)
    {
        return STEPPER_NO_PULSE;
    }

    const uint8_t lastBatchIndex = totalBatches - 1;
    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_HOME_BOUNDARY_TO_READY:
        case STEPPER_RUNNING_ACTIVE:
        case STEPPER_RUNNING_PASSIVE:
        case STEPPER_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_WRONG_STATE; 
        default:
            break;
    }

    if(0 == batchIndex)
    {
        _reset_active_stepper_pulses(id);
    }

    if(pStepper->rampupPulses != count * batchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }
    if(pStepper->pRampdownPulseWidths == NULL)
    {
        return STEPPER_INTERNAL_DATA_ERROR;
    }
    if((pStepper->rampupPulses + count) > MAX_RAMP_CLOCKS)
    {
        return STEPPER_TOO_MANY_PULSE_WIDTHS;
    }

    for(uint8_t i=0; i<count; i++)
    {
        pStepper->pRampupPulseWidths[pStepper->rampupPulses + i] = pWidths[i];
    }
    pStepper->rampupPulses +=  count;

    if(batchIndex == (totalBatches - 1))
    {
        pStepper->isRampupPuleseWidthsPopulated = true;
    }

    return STEPPER_OK;
}

StepperReturnCode stepper_set_active_cruise_pulse_width(
    const StepperId id, const uint16_t width)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(width < 1)
    {
        return STEPPER_WRONG_PULSE_WIDTH;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_HOME_BOUNDARY_TO_READY:
        case STEPPER_RUNNING_ACTIVE:
        case STEPPER_RUNNING_PASSIVE:
        case STEPPER_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_WRONG_STATE; 
        default:
            break;
    }

    if(pStepper->isRampupPuleseWidthsPopulated == false)
    {
        return STEPPER_WRONG_PULSE_ORDER;
    }

    pStepper->cruisePulseWidth = width;
    pStepper->isCruisePulseWidthPopulated = true;

    return STEPPER_OK;
}

StepperReturnCode stepper_set_active_rampdown_pulse_widths(
    const StepperId id, 
    const uint16_t * pWidths, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(NULL == pWidths)
    {
        return STEPPER_NULL_PARAMETER;
    }
    if(count < 1)
    {
        return STEPPER_NO_PULSE;
    }
    if(totalBatches < 1)
    {
        return STEPPER_NO_PULSE;
    }

    const uint8_t lastBatchIndex = totalBatches - 1;
    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_HOME_BOUNDARY_TO_READY:
        case STEPPER_RUNNING_ACTIVE:
        case STEPPER_RUNNING_PASSIVE:
        case STEPPER_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_WRONG_STATE; 
        default:
            break;
    }

    if(pStepper->isRampupPuleseWidthsPopulated == false ||
        pStepper->isCruisePulseWidthPopulated == false)
    {
        return STEPPER_WRONG_PULSE_ORDER;
    }

    if(pStepper->rampdownPulses != count * batchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }
    if(pStepper->pRampdownPulseWidths == NULL)
    {
        return STEPPER_INTERNAL_DATA_ERROR;
    }
    if((pStepper->rampdownPulses + count) > MAX_RAMP_CLOCKS)
    {
        return STEPPER_TOO_MANY_PULSE_WIDTHS;
    }

    for(uint8_t i=0; i<count; i++)
    {
        pStepper->pRampdownPulseWidths[pStepper->rampdownPulses + i] = pWidths[i];
    }
    pStepper->rampdownPulses +=  count;

    if(batchIndex == lastBatchIndex)
    {
        pStepper->isRampdownPulseWidthsPopulated = true;
    }

    return STEPPER_OK;
}

static void _reset_passive_stepper_pulses(const StepperId id)
{
    StepperData * pStepper = _steppers + (int)id;

    pStepper->pRampdownPulseWidths = NULL;
    pStepper->rampupPulses = 0;
    pStepper->isRampupPuleseWidthsPopulated = false;

    pStepper->isCruisePulseWidthPopulated = false;

    pStepper->pRampdownPulseWidths = NULL;
    pStepper->rampdownPulses = 0;
    pStepper->isRampdownPulseWidthsPopulated = false;

    pStepper->pPassiveStepArray = pStepper->uint16Array;
    pStepper->passiveStepsCount = 0;
    pStepper->passiveStepIndex = 0;
    pStepper->isPassiveStepsInitialized = false;
}

StepperReturnCode stepper_set_passive_step_indexes(
    const StepperId id, 
    const uint16_t * pIndexes, 
    const uint8_t count, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(pIndexes == NULL)
    {
        return STEPPER_NULL_PARAMETER;
    }
    if(count < 1)
    {
        return STEPPER_NO_PULSE;
    }
    if(totalBatches < 1)
    {
        return STEPPER_NO_PULSE;
    }
    const uint8_t lastBatchIndex = totalBatches - 1;

    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_HOME_BOUNDARY_TO_READY:
        case STEPPER_RUNNING_ACTIVE:
        case STEPPER_RUNNING_PASSIVE:
        case STEPPER_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_WRONG_STATE; 
        default:
            break;
    }

    if(batchIndex == 0)
    {
        _reset_passive_stepper_pulses(id);
    }

    if(pStepper->pPassiveStepArray == NULL)
    {
        return STEPPER_INTERNAL_DATA_ERROR;
    }
    if(pStepper->passiveStepsCount != count * batchIndex)
    {
        return STEPPER_WRONG_BATCH_INDEX;
    }
    if(pStepper->passiveStepsCount + count > MAX_UINT16_ARRAY_LENGTH)
    {
        return STEPPER_TOO_MANY_PASSIVE_INDEXES;
    }
    if(pStepper->isForward)
    {
        if(pStepper->offset + pStepper->passiveStepsCount + count > pStepper->range)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }
    else
    {
        if(pStepper->passiveStepsCount + count > pStepper->offset)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }

    for(uint8_t i=0; i<count; i++)
    {
        pStepper->pPassiveStepArray[pStepper->passiveStepsCount + i] = pIndexes[i];
    }
    pStepper->passiveStepsCount += count;

    if(batchIndex == lastBatchIndex)
    {
        pStepper->isPassiveStepsInitialized = true;
    }

    return STEPPER_OK;
}

StepperReturnCode stepper_start_home_positioning(const StepperId id)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_INITIALIZED)
    {
        return STEPPER_WRONG_STATE;
    }
    if(!pStepper->isRampupPuleseWidthsPopulated)
    {
        return STEPPER_WRONG_STATE;
    }

    StepperReturnCode rc;

    rc = stepper_set_enable(id, true);
    if(STEPPER_OK != rc)
    {
        print_log("Error: stepper_start_home_positioning: failed to enable stepper %d, return code: %d\r\n", id, rc);
        return rc;
    }
    rc = stepper_set_forward(id, false);
    if(STEPPER_OK != rc)
    {
        print_log("Error: stepper_start_home_positioning: failed to reverse stepper %d, return code: %d\r\n", id, rc);
        return rc;
    }

    pStepper->currentStep = 0;
    pStepper->state = STEPPER_RETURN_TO_HOME_BOUNDARY;
    _set_clock_pulse_level_first(pStepper);

    return STEPPER_OK;
}

StepperReturnCode stepper_run_active(const StepperId id, const uint32_t steps)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(!pStepper->isRampupPuleseWidthsPopulated ||
        !pStepper->isCruisePulseWidthPopulated ||
        !pStepper->isRampdownPulseWidthsPopulated)
    {
        return STEPPER_INVALID_ACTIVE_PULSES;
    }
    if(pStepper->isForward)
    {
        if(pStepper->offset + steps > pStepper->range)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }
    else
    {
        if(pStepper->offset < steps)
        {
            return STEPPER_WILL_OUT_OF_RANGE;
        }
    }

    pStepper->stepsToRun = steps;
    pStepper->currentStep = 0;
    pStepper->currentPulseWidth = pStepper->pRampupPulseWidths[0];
    pStepper->pulseState = FIRST_HALF;
    _set_clock_pulse_level_first(pStepper);

    pStepper->state = STEPPER_RUNNING_ACTIVE;

    return STEPPER_OK;
}

StepperReturnCode stepper_couple_passive(const StepperId activeStepperId, const StepperId passiveStepperId)
{
    if(activeStepperId >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(passiveStepperId >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(activeStepperId == passiveStepperId)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pActive = _steppers + (int)activeStepperId;
    StepperData * pPassive = _steppers + (int)passiveStepperId;

    if(pActive->state != STEPPER_RUNNING_ACTIVE)
    {
        return STEPPER_WRONG_STATE;
    }
    if(pPassive->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(pPassive->isPassiveStepsInitialized == false)
    {
        return STEPPER_WRONG_STATE;
    }

    // check if passive stepper has been coupled with any stepper
    bool coupled = false;
    for(int i = 0; i < (int)STEPPER_COUNT; i++)
    {
        StepperData * pStepper = _steppers + i;
        if(pStepper->passiveCoupled)
        {
            for(int j = 0; j < (int)STEPPER_COUNT; j++)
            {
                if(pStepper->passiveStepperIds[j] == passiveStepperId)
                {
                    coupled = true;
                    break;
                }
            }
        }
        if(coupled)
        {
            break;
        }
    }
    if(coupled)
    {
        return SETPPER_ALREADY_COUPLED;
    }

    pActive->passiveStepperIds[passiveStepperId] = passiveStepperId;
    pActive->passiveCoupled = true;

    _set_clock_pulse_level_first(pPassive);
    pPassive->passiveStepIndex = 0;
    pPassive->currentPulseWidth = 0;
    pPassive->state = STEPPER_RUNNING_PASSIVE;

    return STEPPER_OK;
}

StepperReturnCode stepper_decouple_passive(const StepperId activeStepperId, const StepperId passiveStepperId)
{
    if(activeStepperId >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(passiveStepperId >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(activeStepperId == passiveStepperId)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pActive = _steppers + (int)activeStepperId;
    StepperData * pPassive = _steppers + (int)passiveStepperId;

    if(pActive->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(pPassive->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(!pActive->passiveCoupled)
    {
        return STEPPER_NOT_COUPLED;
    }

    bool found = false;
    for(int i=0; i < (int)STEPPER_COUNT; i++)
    {
        if(pActive->passiveStepperIds[i] == passiveStepperId)
        {
            found = true;
            pActive->passiveStepperIds[i] = ENCODER_INVALID_ID;
        }
    }
    if(!found)
    {
        return STEPPER_NOT_COUPLED;
    }

    found = false;
    for(int i=0; i < (int)STEPPER_COUNT; i++)
    {
        if(pActive->passiveStepperIds[i] != ENCODER_INVALID_ID)
        {
            found = true;
            break;
        }
    }
    if(!found)
    {
        pActive->passiveCoupled = false;
    }

    return STEPPER_OK;
}

StepperReturnCode stepper_run_force(const StepperId id, const uint16_t pulseWidth, const uint8_t steps)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_INITIALIZED)
    {
        return STEPPER_WRONG_STATE;
    }

    pStepper->forcePulseWidth = pulseWidth;
    pStepper->stepsToRun = steps;
    pStepper->currentStep = 0;
    pStepper->state = STEPPER_RUNNING_FORCED;

    return STEPPER_OK;
}

StepperReturnCode stepper_get_state(const StepperId id, StepperState * const pState)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(pState == NULL)
    {
        return STEPPER_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    *pState = pStepper->state;

    return STEPPER_OK;
}

static void _update_stepper_position(StepperData * const pStepper)
{
    const uint16_t curr = encoder_get_count(pStepper->encoderId);
    const uint16_t prev = pStepper->encoderCount;

    // this function is called in high frequency, so the difference between
    // curr and prev should not be too large.
    // forword:
    //   case 0:   0x0 -------------------prev-----curr------------------------- 0xFFFF
    //   case 1:   0x0 --curr-------------------------------------------prev---- 0xFFFF
    // backword:
    //   case 2:   0x0 ----------------------------curr-----------prev---------- 0xFFFF
    //   case 3:   0x0 ----prev-----------------------------------------curr---- 0xFFFF

    if(curr > prev)
    {
        uint16_t delta = curr - prev;
        if(delta < 0x7FFF)
        {
            pStepper->actualPosition += delta; // case 0
        }
        else
        {
            pStepper->actualPosition -= 0x10000 - delta; // case 3
        }
    }
    else
    {
        uint16_t delta = prev - curr;
        if(delta < 0x7FFF)
        {
            pStepper->actualPosition -= delta; // case 2
        }
        else
        {
            pStepper->actualPosition += 0x10000 - delta; // case 1
        }
    }

    pStepper->encoderCount = curr;
}

static bool _is_stepper_in_sync(const StepperData * const pStepper)
{
    // expectedPosition = offset * (countsPerRevolution / stepsPerRevolution)
    int64_t expectedPosition = pStepper->offset;
    expectedPosition *= pStepper->countsPerRevolution;
    expectedPosition /= pStepper->stepsPerRevolution;

    if(pStepper->actualPosition >= (expectedPosition - pStepper->maxPositionError) &&
        pStepper->actualPosition <= (expectedPosition + pStepper->maxPositionError))
    {
        return true;
    }
    else
    {
        return false;
    }
}

StepperReturnCode stepper_check_sync(const StepperId id, bool * const pInSync)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_READY)
    {
        return STEPPER_WRONG_STATE;
    }
    if(pStepper->encoderId == ENCODER_INVALID_ID)
    {
        return STEPPER_INVALID_ENCODER_ID;
    }

    _update_stepper_position(pStepper);
    *pInSync = _is_stepper_in_sync(pStepper);

    return STEPPER_OK;
}

StepperReturnCode stepper_get_startup_pulse_width(const StepperId id, uint16_t * const pPulseWidth)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(pPulseWidth == NULL)
    {
        return STEPPER_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_RUNNING_ACTIVE:
            *pPulseWidth = pStepper->pRampupPulseWidths[0];
            return STEPPER_OK;
        
        case STEPPER_RUNNING_FORCED:
            *pPulseWidth = pStepper->forcePulseWidth;
            return STEPPER_OK;

        default:
            return STEPPER_WRONG_STATE;
    }
}

static StepperReturnCode _on_stepper_pulse_end_force(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        // first clock pulse has finished
        _set_clock_pulse_level_second(pStepper);
        *pNextPulseWidth = pStepper->forcePulseWidth;
        pStepper->pulseState = SECOND_HALF;

        return STEPPER_OK;
    }

    // second clock pulse has finished
    _set_clock_pulse_level_first(pStepper);

    pStepper->currentStep += 1;
    if(pStepper->currentStep < pStepper->stepsToRun)
    {
        *pNextPulseWidth = pStepper->forcePulseWidth;
        pStepper->pulseState = FIRST_HALF;

        return STEPPER_OK;
    }

    *pNextPulseWidth = 0;

    pStepper->state = STEPPER_READY;

    return STEPPER_OK;
}

static StepperReturnCode _on_stepper_pulse_end_active(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        // first clock pulse has finished, check if stepper has arrived at the expected position
        if(pStepper->encoderId != ENCODER_INVALID_ID)
        {
            _update_stepper_position(pStepper);
            bool inSync = _is_stepper_in_sync(pStepper);
            if(!inSync)
            {
                *pNextPulseWidth = 0;
                pStepper->state = STEPPER_OUT_OF_SYNC;
                return STEPPER_UN_SYNC;
            }
        }
        
        // start the second clock pulse to make the stepper move further
        _set_clock_pulse_level_second(pStepper);
        const StepperReturnCode rc = _notify_passive_steppers(pStepper);
        if(rc != STEPPER_OK)
        {
            *pNextPulseWidth = 0;
            return rc;
        }

        *pNextPulseWidth = pStepper->currentPulseWidth;
        pStepper->pulseState = SECOND_HALF;

        return STEPPER_OK;
    }

    // second clock pulse has finished
    _set_clock_pulse_level_first(pStepper);
    const StepperReturnCode rc = _notify_passive_steppers(pStepper);
    if(rc != STEPPER_OK)
    {
        *pNextPulseWidth = 0;
        return rc;
    }

    pStepper->currentStep += 1;
    if(pStepper->currentStep == pStepper->stepsToRun)
    {
        pStepper->state = STEPPER_READY;
        *pNextPulseWidth = 0;
        return STEPPER_OK;
    }

    // calculate the width of next pulse
    uint16_t width = pStepper->cruisePulseWidth; // the default
    bool inFirstHalf = pStepper->currentStep < (pStepper->stepsToRun >> 1);
    if(inFirstHalf)
    {
        if(pStepper->currentStep < pStepper->rampupPulses)
        {
            // accelerating
            width = pStepper->pRampupPulseWidths[pStepper->currentStep];
        }
    }
    else
    {
        uint32_t remainingSteps = pStepper->stepsToRun - pStepper->currentStep;
        if(pStepper->rampdownPulses > remainingSteps)
        {
            // deaccelerating
            uint32_t index = pStepper->rampdownPulses - remainingSteps - 1;
            width = pStepper->pRampdownPulseWidths[index];
        }
    }
    *pNextPulseWidth = width;
    
    pStepper->currentPulseWidth = width;
    pStepper->pulseState = FIRST_HALF;

    return STEPPER_OK;
}

static bool _is_stepper_at_home_boundary(StepperData * const pStepper)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(pStepper->pGpioPortHomeBoundary, 0x1 << pStepper->gpioPinIndexHomeBoundary);

    if(state == GPIO_PIN_SET)
        return true;
    else
        return false;
}

static bool _is_stepper_at_end_boundary(StepperData * const pStepper)
{
    GPIO_PinState state = HAL_GPIO_ReadPin(pStepper->pGpioPortEndBoundary, 0x1 << pStepper->gpioPinIndexEndBoundary);

    if(state == GPIO_PIN_SET)
        return true;
    else
        return false;
}


static StepperReturnCode _on_stepper_pulse_end_to_home(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        _set_clock_pulse_level_second(pStepper);
        pStepper->pulseState = SECOND_HALF;
        *pNextPulseWidth = pStepper->currentPulseWidth;

        return STEPPER_OK;
    }

    // second half of clock pulse has finished.
    _set_clock_pulse_level_first(pStepper);
    pStepper->pulseState = FIRST_HALF;
    *pNextPulseWidth = pStepper->pRampupPulseWidths[0];
    pStepper->currentPulseWidth = pStepper->pRampupPulseWidths[0];

    bool atHomeBoundary = _is_stepper_at_home_boundary(pStepper);
    if(!atHomeBoundary)
    {
        return STEPPER_OK;
    }

    // stepper arrives at home boundary
    _stepper_set_forward(pStepper, true);
    pStepper->state = STEPPER_HOME_BOUNDARY_TO_READY;

    return STEPPER_OK;
}

static StepperReturnCode _on_stepper_pulse_end_home_to_ready(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        _set_clock_pulse_level_second(pStepper);
        pStepper->pulseState = SECOND_HALF;
        *pNextPulseWidth = pStepper->currentPulseWidth;

        return STEPPER_OK;
    }

    // second half of clock pulse has finished.
    _set_clock_pulse_level_first(pStepper);
    pStepper->pulseState = FIRST_HALF;
    *pNextPulseWidth = pStepper->pRampupPulseWidths[0];
    pStepper->currentPulseWidth = pStepper->pRampupPulseWidths[0];

    bool atHomeBoundary = _is_stepper_at_home_boundary(pStepper);
    if(atHomeBoundary)
    {
        pStepper->currentStep = 0;
    }
    else
    {
        pStepper->currentStep++;
    }

    if(pStepper->currentStep == pStepper->homeBoundaryToReadySteps)
    {
        *pNextPulseWidth = 0;
        pStepper->state = STEPPER_READY;
    }

    return STEPPER_OK;
}

StepperReturnCode on_interupt_stepper_pulse_end(const StepperId id, uint16_t * const pNextPulseWidth)
{
    if(id >= STEPPER_COUNT)
    {
        return STEPPER_INVALID_ID;
    }
    if(pNextPulseWidth == NULL)
    {
        return STEPPER_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state == STEPPER_RUNNING_ACTIVE)
    {
        if(pStepper->currentStep >= pStepper->stepsToRun)
        {
            return STEPPER_OUT_OF_STEPS;
        }
        if(_is_stepper_at_home_boundary(pStepper) ||
            _is_stepper_at_end_boundary(pStepper))
        {
            *pNextPulseWidth = 0;
            return STEPPER_OUT_OF_RANGE;
        }

        StepperReturnCode returnCode = _on_stepper_pulse_end_active(pStepper, pNextPulseWidth);
        return returnCode;
    }   
    
    if(pStepper->state == STEPPER_RUNNING_FORCED)
    {
        if(pStepper->currentStep >= pStepper->stepsToRun)
        {
            return STEPPER_OUT_OF_STEPS;
        }

        StepperReturnCode returnCode = _on_stepper_pulse_end_force(pStepper, pNextPulseWidth);
        return returnCode;
    }

    if(pStepper->state == STEPPER_RETURN_TO_HOME_BOUNDARY)
    {
        StepperReturnCode returnCode = _on_stepper_pulse_end_to_home(pStepper, pNextPulseWidth);
        return returnCode;
    }

    if(pStepper->state == STEPPER_HOME_BOUNDARY_TO_READY)
    {
        StepperReturnCode returnCode = _on_stepper_pulse_end_home_to_ready(pStepper, pNextPulseWidth);
        return returnCode;
    }
    
    return STEPPER_WRONG_STATE;
}
