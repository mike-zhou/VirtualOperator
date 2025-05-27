/*
 * stepper.c
 *
 *  Created on: May 4, 2025
 *      Author: mike
 */

#include "stepper.h"

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
    uint32_t range;
    uint16_t stepsPerREvolution;
    EncoderId encoderId;
    uint16_t countsPerRevolution;
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

    uint32_t stepsToRun;
    uint32_t currentStep;

    PulseState pulseState;

    StepperId passiveIds[STEPPER_COUNT];
    bool passiveCoupled;
} StepperData;

static StepperData _steppers[STEPPER_COUNT];

/**
 * This function is called by the master stepper to notify the slave stepper 
 * of its current stepIndex.
 * If stepIndex is what the slave expected, the slave needs to drive its
 * clock pin according to pulseState.
 */
static bool _on_master_step(StepperId slaveId, uint32_t stepIndex, PulseState pulseState)
{

}

static uint16_t _get_encoder(EncoderId encoderId)
{
    
}

static uint16_t _is_static_data_initialized(StepperId id)
{
    StepperData * pStepper = _steppers + (int)id;

    if(!pStepper->isStepperControlInitialized)
    {
        return false;
    }
    
    if(pStepper->isRampupPuleseWidthsPopulated)
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
        pStepper->stepsToRun = 0;
        pStepper->currentStep = 0;
        pStepper->pulseState = FIRST_HALF;
        for(uint8_t passiveIndex=0; passiveIndex<STEPPER_COUNT; passiveIndex++)
        {
            pStepper->passiveIds[passiveIndex] = STEPPER_INVALID_ID;
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
    const uint32_t range,
    const uint16_t stepsPerRevolution,
    const EncoderId encoderId,
    const uint16_t countsPerRevolution)
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
    pStepper->range = range;
    pStepper->stepsPerREvolution = stepsPerRevolution;
    pStepper->encoderId = encoderId;
    pStepper->countsPerRevolution = countsPerRevolution;
    pStepper->isStepperControlInitialized = true;

    if(_is_static_data_initialized(id))
    {
        pStepper->state = STEPPER_INITIALIZED;
    }
    else
    {
        pStepper->state = STEPPER_UNINITIALIZED;
    }

    return STEPPER_OK;
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

static void _reset_active_stepper_pulses(StepperId id)
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
        case STEPPER_RETURN_TO_HOME:
        case STEPPER_HOME_TO_READY:
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
        case STEPPER_RETURN_TO_HOME:
        case STEPPER_HOME_TO_READY:
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
        case STEPPER_RETURN_TO_HOME:
        case STEPPER_HOME_TO_READY:
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

