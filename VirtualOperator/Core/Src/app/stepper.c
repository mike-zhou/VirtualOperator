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

extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

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

    PulseState pulseState;

    uint16_t forcePulseWidth;
    
    StepperId passiveStepperIds[STEPPER_COUNT];
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
    pStepper->pulseState = FIRST_HALF;

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

static uint16_t _get_encoder_count(const EncoderId encoderId)
{
    uint16_t value = 0;

    switch(encoderId)
    {
        case ENCODER_0:
            value = HAL_LPTIM_ReadCounter(&hlptim1);
            break;

        case ENCODER_1:
            value = HAL_LPTIM_ReadCounter(&hlptim2);
            break;

        case ENCODER_2:
            value = __HAL_TIM_GET_COUNTER(&htim1);
            break;

        case ENCODER_3:
            value = __HAL_TIM_GET_COUNTER(&htim2);
            break;

        case ENCODER_4:
            value = __HAL_TIM_GET_COUNTER(&htim3);
            break;

        case ENCODER_5:
            value = __HAL_TIM_GET_COUNTER(&htim4);  
            break;

        case ENCODER_6:
            value = __HAL_TIM_GET_COUNTER(&htim5);
            break;

        case ENCODER_7:
            value = __HAL_TIM_GET_COUNTER(&htim8);
            break;

        default:
            break;
    }

    return value;
}

static void _update_stepper_position(StepperData * const pStepper)
{
    const uint16_t curr = _get_encoder_count(pStepper->encoderId);
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

