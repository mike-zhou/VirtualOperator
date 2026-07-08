/*
 * stepper.c
 *
 *  Created on: May 4, 2025
 *      Author: mike
 */

#include <stdlib.h>

#include "app.h"
#include "stepper.h"
#include "usart1.h"

#define MAX_UINT16_ARRAY_LENGTH 4096
#define MAX_RAMP_CLOCKS (MAX_UINT16_ARRAY_LENGTH >> 1)
#define MAX_AMOUNT_OF_PULSE_IN_BATCH 100 // There is a correspondance in SetActivePeriods()
#define CROSS_BOUDNARY_MAX_ITEMS 4


typedef enum _PulseState
{
    FIRST_HALF,          
    SECOND_HALF         
} PulseState;

typedef struct _CrossBoundary
{
	bool enabled;
	int32_t negativeRange;

	struct Boundary
	{
		bool enabled;
		int32_t offset;
		int16_t error;
	} boundaries[CROSS_BOUDNARY_MAX_ITEMS];
} CrossBoundary;

typedef enum _activeSubState
{
    ACCELERATING,
    CRUISING,
    DEACCELERATING
} StepperActiveSubState;

typedef struct _Stepper
{
    /**
     * static data
     */
    StepperId stepperId;
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
    int32_t range;
    uint16_t stepsPerRotation;
    EncoderId encoderId;
    uint16_t encoderCountsPerRotation;
    uint16_t encoderOffsetErrorThreshold;
    float encoderStepperRatio;
    bool isStepperControlInitialized;

    // uint16_t data array for active or passive pulses
    uint16_t uint16Array[MAX_UINT16_ARRAY_LENGTH];

    /**
     * active pules
     */
    // array of pulse width for the stepper to speed up from still to cruising
    uint16_t * pRampupPulseWidths;
    uint32_t rampupPulseCount;
    bool isRampupPuleseWidthsPopulated;
    // array of pulse width for the stepper to slow down from curising to still
    uint16_t * pRampdownPulseWidths;
    uint32_t rampdownPulseCount;
    bool isRampdownPulseWidthsPopulated;
    // pulse width when the stepper is cruising
    uint16_t cruisePulseWidth;
    bool isCruisePulseWidthPopulated;

    /**
     * passive pules
     */
    uint16_t * pPassiveStepArray;
    uint32_t passiveStepsCount;
    uint32_t passiveStepIndex;
    bool isPassiveStepsPopulated;

    /**
     * forced pulse
     */
    uint16_t forcePulseWidth;

    /**
     * cross boundary
     */
    CrossBoundary crossBoundary;

    /**
     * dynamic data
     */

    StepperState state;
    StepperActiveSubState activeSubState;

    bool isEnabled;
    bool isForward;
    int32_t offset;

    int32_t encoderHomePosition;
    int32_t encoderOffset;
    uint8_t maxEncoderOffsetError;

    int32_t stepsToRun;
    int32_t currentStep;
    uint16_t currentPulseWidth;

    PulseState pulseState;

    StepperId passiveStepperIds[STEPPER_ID_COUNT];
    bool passiveCoupled;
} StepperData;

static StepperData _steppers[STEPPER_ID_COUNT];

/**
 * Set the voltage level at the first half of the clock signal
 */
static void _set_clock_first_half(StepperData * const pStepper)
{
    GPIO_PinState pinState = GPIO_PIN_SET;

    if(pStepper->isRisingEdgeDriven == true)
    {
        pinState = GPIO_PIN_RESET;
    }
    HAL_GPIO_WritePin(pStepper->pGpioPortClock, 1 << pStepper->gpioPinIndexClock, pinState);
    pStepper->pulseState = FIRST_HALF;
}

/**
 * Set the voltage level at the second half of the clock signal
 */
static void _set_clock_second_half(StepperData * const pStepper)
{
    GPIO_PinState pinState = GPIO_PIN_RESET;

    if(pStepper->isRisingEdgeDriven == true)
    {
        pinState = GPIO_PIN_SET;
    }
    HAL_GPIO_WritePin(pStepper->pGpioPortClock, 1 << pStepper->gpioPinIndexClock, pinState);
    pStepper->pulseState = SECOND_HALF;
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

static bool _boundary_detector_flips(StepperData * const pStepper)
{
	static bool prevHomeDetectorStatus = false;
	static bool prevEndDetectorStatus = false;

	bool stateChanged;

	bool currentHomeDetectorStatus = _is_stepper_at_home_boundary(pStepper);
	bool currentEndDetectorStatus = _is_stepper_at_end_boundary(pStepper);

	if(prevHomeDetectorStatus != currentHomeDetectorStatus ||
	   prevEndDetectorStatus != currentEndDetectorStatus) {
		stateChanged = true;
	}
	else {
		stateChanged = false;
	}

	prevHomeDetectorStatus = currentHomeDetectorStatus;
	prevEndDetectorStatus = currentEndDetectorStatus;

	return stateChanged;
}

static bool _is_stepper_out_of_range(StepperData * const pStepper)
{
	if(pStepper->crossBoundary.enabled)
	{
		if(_boundary_detector_flips(pStepper))
		{
    		bool boundaryFailure = false;

    		for(int i = 0; i < CROSS_BOUDNARY_MAX_ITEMS; i++)
    		{
    			struct Boundary * pBoundary = pStepper->crossBoundary.boundaries + i;

    			if(!pBoundary->enabled) {
    				continue;
    			}

    			// a boundary checking is enabled
    			boundaryFailure = true;
    			if(pStepper->offset >= (pBoundary->offset - pBoundary->error) &&
    			   pStepper->offset <= (pBoundary->offset + pBoundary->error)) {
    				// in one expected scope
    				boundaryFailure = false;
    				break;
    			}
    		}

    		if(boundaryFailure)
    		{
				return true;
     		}
		}
	}
	else if(_is_stepper_at_home_boundary(pStepper) ||
            _is_stepper_at_end_boundary(pStepper))
	{
		return true;
	}

	return false;
}

static bool _is_stepper_in_sync(StepperData * const pStepper)
{
    encoder_poll(pStepper->encoderId);

    int32_t encoderOffset = encoder_get_offset(pStepper->encoderId) - pStepper->encoderHomePosition;
    int32_t expectedEncoderPosition = pStepper->offset * pStepper->encoderStepperRatio;

    int32_t error = abs(abs(encoderOffset) - abs(expectedEncoderPosition));

    if(error > pStepper->maxEncoderOffsetError)
    {
    	if(error > UINT8_MAX) {
    		pStepper->maxEncoderOffsetError = UINT8_MAX;
    	}
    	else {
    		pStepper->maxEncoderOffsetError = (uint8_t)error;
    	}
    }

    pStepper->encoderOffset = encoderOffset;

    if(error >= pStepper->encoderOffsetErrorThreshold)
    {
        return false;
    }
    else
    {
        return true;
    }
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
    if(passiveStepperId >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    
    StepperData * pPassive = _steppers + (int)passiveStepperId;

    if(pPassive->state != STEPPER_STATE_RUNNING_PASSIVE) {
        return STEPPER_ERROR_WRONG_STATE;
    }

	if(pPassive->passiveStepIndex >= pPassive->passiveStepsCount)
	{
		return STEPPER_ERROR_INTERNAL_DATA_ERROR;
	}

	uint32_t expectedActiveStepIndex = pPassive->pPassiveStepArray[pPassive->passiveStepIndex];

	if(activeStepIndex < expectedActiveStepIndex)
	{
		return STEPPER_OK;
	}
	if(activeStepIndex > expectedActiveStepIndex)
	{
		return STEPPER_ERROR_MISSED_ACTIVE_PULSE;
	}


	if(_is_stepper_out_of_range(pPassive))
	{
		on_stepper_out_of_scope_interrupt(passiveStepperId);
		return STEPPER_ERROR_OUT_OF_RANGE;
	}

	if(activePulseState == FIRST_HALF)
	{
		if(pPassive->encoderId != ENCODER_ID_INVALID)
		{
			bool inSync = _is_stepper_in_sync(pPassive);
			if(!inSync)
			{
				on_stepper_out_of_sync_interrupt(passiveStepperId);
                print_log("Error: passive stepper %d out of sync, stepper offset: %d, encoder offset: %d\r\n", (int)passiveStepperId, pPassive->offset, pPassive->encoderOffset);
				pPassive->state = STEPPER_STATE_OUT_OF_SYNC;
				return STEPPER_ERROR_OUT_OF_SYNC;
			}
		}
		_set_clock_second_half(pPassive);
	}
	else
	{
		_set_clock_first_half(pPassive);

		pPassive->passiveStepIndex++;
		if(pPassive->passiveStepIndex == pPassive->passiveStepsCount)
		{
			pPassive->state = STEPPER_STATE_READY;
		}

		if(pPassive->isForward)
		{
			pPassive->offset++;
			if(pPassive->offset >= pPassive->range)
			{
				return STEPPER_ERROR_OUT_OF_RANGE;
			}
		}
		else
		{
			pPassive->offset--;
			if(pPassive->offset < 0)
			{
	        	if(pPassive->crossBoundary.enabled)
	        	{
	        		if(pPassive->offset <= pPassive->crossBoundary.negativeRange)
	        		{
	            		return STEPPER_ERROR_OUT_OF_RANGE;
	        		}
	        	}
	        	else
	        	{
	        		return STEPPER_ERROR_OUT_OF_RANGE;
	        	}
			}
		}
	}

	return STEPPER_OK;
}

static StepperReturnCode _notify_passive_steppers(StepperData * const pActiveStepper)
{
    if(pActiveStepper->passiveCoupled == false)
    {
        return STEPPER_OK;
    }

    StepperReturnCode rc;

    for(int i = 0; i < STEPPER_ID_COUNT; i++)
    {
        const StepperId passiveStepperId = pActiveStepper->passiveStepperIds[i];
        if(passiveStepperId == STEPPER_ID_INVALID)
        {
            continue;
        }

        rc = _on_active_stepper_pulse_end(passiveStepperId, pActiveStepper->currentStep, pActiveStepper->pulseState);
        if(rc != STEPPER_OK)
        {
            return rc;
        }

        StepperData * pPassive = _steppers + (int)passiveStepperId;
        if(pPassive->state == STEPPER_STATE_READY)
        {
            // passive stepper moves to designated position, decouple it
            pActiveStepper->passiveStepperIds[i] = STEPPER_ID_INVALID; 
        }
    }

    // check if any passive stepper needs to be driven
    pActiveStepper->passiveCoupled = false;
    for(int j=0; j<STEPPER_ID_COUNT; j++)
    {
        if(pActiveStepper->passiveStepperIds[j] != STEPPER_ID_INVALID)
        {
            pActiveStepper->passiveCoupled = true;
            break;
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
    
    if(pStepper->isPassiveStepsPopulated)
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
    for(uint8_t stepperIndex=0; stepperIndex<STEPPER_ID_COUNT; stepperIndex++)
    {
        StepperData * pStepper = _steppers + stepperIndex;

        // static data
        pStepper->stepperId = (StepperId)stepperIndex;
        pStepper->isStepperControlInitialized = false;
        pStepper->isRampupPuleseWidthsPopulated = false;
        pStepper->isRampdownPulseWidthsPopulated = false;
        pStepper->isCruisePulseWidthPopulated = false;
        pStepper->isPassiveStepsPopulated = false;
        pStepper->crossBoundary.enabled = false;

        // dynamic data
        pStepper->state = STEPPER_STATE_UNINITIALIZED;
        pStepper->isEnabled = false;
        pStepper->offset = 0;
        pStepper->maxEncoderOffsetError = 0;
        pStepper->stepsToRun = 0;
        pStepper->currentStep = 0;
        pStepper->pulseState = FIRST_HALF;
        pStepper->forcePulseWidth = 0xFFFF;
        for(uint8_t passiveIndex=0; passiveIndex<STEPPER_ID_COUNT; passiveIndex++)
        {
            pStepper->passiveStepperIds[passiveIndex] = STEPPER_ID_INVALID;
        }
        pStepper->passiveCoupled = false;
    }
}

StepperReturnCode stepper_set_controls(
    const StepperId id,
    const bool isRisingEdgeDriven,
    const bool isForwardHigh,
    const bool isEnableHigh,
    GPIO_TypeDef * const pGpioPortHomeBoundary,
    const uint8_t gpioPinIndexHomeBoundary,
    GPIO_TypeDef * const pGpioPortEndBoundary,
    const uint8_t gpioPinIndexEndBoundary,
    GPIO_TypeDef * const pGpioPortEnable,
    const uint8_t gpioPinIndexEnable,
    GPIO_TypeDef * const pGpioPortForward,
    const uint8_t gpioPinIndexForward,
    GPIO_TypeDef * const pGpioPortClock,
    const uint8_t gpioPinIndexClock,
    const uint16_t homeBoundaryToReadySteps,
    const uint32_t range,
    const uint16_t stepsPerRotation,
    const EncoderId encoderId,
    const uint16_t encoderCountsPerRotation,
    const uint16_t encoderOffsetErrorThreshold)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(pGpioPortHomeBoundary == NULL ||
        pGpioPortEndBoundary == NULL ||
        pGpioPortEnable == NULL ||
        pGpioPortForward == NULL ||
        pGpioPortClock == NULL)
    {
        return STEPPER_ERROR_INVALID_GPIO_PORT;
    }
    if(gpioPinIndexHomeBoundary > 15 ||
        gpioPinIndexEndBoundary > 15 ||
        gpioPinIndexEnable > 15 ||
        gpioPinIndexForward > 15 ||
        gpioPinIndexClock > 15)
    {
        return STEPPER_ERROR_INVALID_GPIO_PIN_INDEX;
    }
    if(homeBoundaryToReadySteps == 0)
    {
        return STEPPER_ERROR_INVALID_READY_STEPS;
    }
    if(range == 0 || range >= INT32_MAX)
    {
        return STEPPER_ERROR_INVALID_RANGE;
    }
    if(encoderId >= ENCODER_ID_COUNT && encoderId != ENCODER_ID_INVALID)
    {
        return STEPPER_ERROR_INVALID_ENCODER_ID;
    }
    if(stepsPerRotation == 0 || encoderCountsPerRotation == 0)
    {
        return STEPPER_ERROR_INVALID_CONTROL_PARAMETER;
    }
    if(encoderOffsetErrorThreshold < 1 || encoderOffsetErrorThreshold >= INT16_MAX)
    {
        return STEPPER_ERROR_INVALID_CONTROL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_STATE_UNINITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    pStepper->isRisingEdgeDriven = isRisingEdgeDriven;
    pStepper->isForwardHigh = isForwardHigh;
    pStepper->isEnableHigh = isEnableHigh;
    pStepper->pGpioPortHomeBoundary = pGpioPortHomeBoundary;
    pStepper->gpioPinIndexHomeBoundary = gpioPinIndexHomeBoundary;
    pStepper->pGpioPortEndBoundary = pGpioPortEndBoundary;
    pStepper->gpioPinIndexEndBoundary = gpioPinIndexEndBoundary;
    pStepper->pGpioPortEnable = pGpioPortEnable;
    pStepper->gpioPinIndexEnable = gpioPinIndexEnable;
    pStepper->pGpioPortForward = pGpioPortForward;
    pStepper->gpioPinIndexForward = gpioPinIndexForward;
    pStepper->pGpioPortClock = pGpioPortClock;
    pStepper->gpioPinIndexClock = gpioPinIndexClock;
    pStepper->homeBoundaryToReadySteps = homeBoundaryToReadySteps;
    pStepper->range = range;
    pStepper->stepsPerRotation = stepsPerRotation;
    pStepper->encoderId = encoderId;
    pStepper->encoderCountsPerRotation = encoderCountsPerRotation;
    pStepper->encoderOffsetErrorThreshold = encoderOffsetErrorThreshold;
    pStepper->encoderStepperRatio = (float)encoderCountsPerRotation / (float)stepsPerRotation;
    pStepper->isStepperControlInitialized = true;

    if(!_is_static_data_initialized(id))
    {
        return STEPPER_ERROR_WRONG_INIT_ORDER;
    }

    GPIO_PinState pinState;
    
    // check stepper's direction
    pinState = HAL_GPIO_ReadPin(pStepper->pGpioPortForward, 0x1 << pStepper->gpioPinIndexForward);
    if(pStepper->isForwardHigh)
    {
        if(pinState == GPIO_PIN_SET)
        {
            pStepper->isForward = true;
        }
        else
        {
            pStepper->isForward = false;
        }
    }
    else
    {
        if(pinState == GPIO_PIN_SET)
        {
            pStepper->isForward = false;
        }
        else
        {
            pStepper->isForward = true;
        }
    }

    // check if stepper is enabled
    pinState = HAL_GPIO_ReadPin(pStepper->pGpioPortEnable, 0x1 << pStepper->gpioPinIndexEnable);
    if(pStepper->isEnableHigh)
    {
        if(pinState == GPIO_PIN_SET)
        {
            pStepper->isEnabled = true;
        }
        else
        {
            pStepper->isEnabled = false;
        }
    }
    else
    {
        if(pinState == GPIO_PIN_SET)
        {
            pStepper->isEnabled = false;
        }
        else
        {
            pStepper->isEnabled = true;
        }
    }

    pStepper->state = STEPPER_STATE_INITIALIZED;

    _set_clock_first_half(pStepper);

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
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_STATE_INITIALIZED &&
        pStepper->state != STEPPER_STATE_READY)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    if(pStepper->state == STEPPER_STATE_READY)
    {
        if(isForward)
        {
            if((pStepper->offset + pStepper->stepsToRun) > pStepper->range)
            {
                return STEPPER_ERROR_WILL_OUT_OF_RANGE;
            }
        }
        else
        {
            if(pStepper->crossBoundary.enabled)
            {
                if((pStepper->offset - pStepper->stepsToRun) < pStepper->crossBoundary.negativeRange)
                {
                    return STEPPER_ERROR_WILL_OUT_OF_RANGE;
                }
            }
            else 
            {
                if(pStepper->offset < pStepper->stepsToRun)
                {
                    return STEPPER_ERROR_WILL_OUT_OF_RANGE;
                }
            }
        }
    }

    _stepper_set_forward(pStepper, isForward);

    return STEPPER_OK;
}

StepperReturnCode stepper_set_enable(const StepperId id, const bool isEnable)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    GPIO_PinState pinState;

    if(isEnable)
    {
        if(pStepper->state != STEPPER_STATE_INITIALIZED)
        {
            return STEPPER_ERROR_WRONG_STATE;
        }

        if(pStepper->isEnableHigh)
            pinState = GPIO_PIN_SET;
        else
            pinState = GPIO_PIN_RESET;
    }
    else
    {
        if(pStepper->state == STEPPER_STATE_UNINITIALIZED)
        {
            return STEPPER_ERROR_WRONG_STATE;
        }

        pStepper->state = STEPPER_STATE_INITIALIZED; // stepper loses position info if disabled.

        if(pStepper->isEnableHigh)
            pinState = GPIO_PIN_RESET;
        else
            pinState = GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortEnable, 1 << pStepper->gpioPinIndexEnable, pinState);
    pStepper->isEnabled = isEnable;

    if(!isEnable)
    {
        _set_clock_first_half(pStepper);
    }

    return STEPPER_OK;
}

static void _reset_active_stepper_pulses(const StepperId id)
{
    StepperData * pStepper = _steppers + (int)id;

    pStepper->pRampdownPulseWidths = pStepper->uint16Array;
    pStepper->rampupPulseCount = 0;
    pStepper->isRampupPuleseWidthsPopulated = false;

    pStepper->isCruisePulseWidthPopulated = false;

    pStepper->pRampdownPulseWidths = pStepper->uint16Array + MAX_RAMP_CLOCKS;
    pStepper->rampdownPulseCount = 0;
    pStepper->isRampdownPulseWidthsPopulated = false;

    pStepper->pPassiveStepArray = NULL;
    pStepper->passiveStepsCount = 0;
    pStepper->passiveStepIndex = 0;
    pStepper->isPassiveStepsPopulated = false;
}

StepperReturnCode stepper_set_active_rampup_pulse_widths(
    const StepperId id, 
    const uint8_t * const pWidths, 
    const uint8_t length, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(NULL == pWidths)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }
    if(length < 2)
    {
        return STEPPER_ERROR_NO_PULSE;
    }
    if(length & 0x1)
    {
        return STEPPER_ERROR_INVALID_PULSE_LENGTH;
    }
    if(totalBatches < 1)
    {
        return STEPPER_ERROR_NO_PULSE;
    }

    const uint8_t count = length >> 1;
    const uint8_t lastBatchIndex = totalBatches - 1;
    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_HOME_BOUNDARY_TO_READY:
        case STEPPER_STATE_RUNNING_ACTIVE:
        case STEPPER_STATE_RUNNING_PASSIVE:
        case STEPPER_STATE_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_ERROR_WRONG_STATE; 
        default:
            break;
    }

    if(0 == batchIndex)
    {
        _reset_active_stepper_pulses(id);
    }

    if(pStepper->rampupPulseCount != MAX_AMOUNT_OF_PULSE_IN_BATCH * batchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }
    if(pStepper->pRampdownPulseWidths == NULL)
    {
        return STEPPER_ERROR_INTERNAL_DATA_ERROR;
    }
    if((pStepper->rampupPulseCount + count) > MAX_RAMP_CLOCKS)
    {
        return STEPPER_ERROR_TOO_MANY_PULSE_WIDTHS;
    }

    for(uint8_t i=0; i<count; i++)
    {
        uint16_t width = pWidths[i * 2 + 1];
        width <<= 8;
        width += pWidths[i * 2];

        pStepper->pRampupPulseWidths[pStepper->rampupPulseCount + i] = width;
    }
    pStepper->rampupPulseCount +=  count;

    // all pulses are to be set up.
    pStepper->isRampupPuleseWidthsPopulated = false;
    pStepper->isCruisePulseWidthPopulated = false;
    pStepper->isRampdownPulseWidthsPopulated = false;

    if(batchIndex == (totalBatches - 1))
    {
        pStepper->isRampupPuleseWidthsPopulated = true;
    }

    return STEPPER_OK;
}

StepperReturnCode stepper_set_active_cruise_pulse_width(
    const StepperId id, const uint16_t width)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(width < 1)
    {
        return STEPPER_ERROR_WRONG_PULSE_WIDTH;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_HOME_BOUNDARY_TO_READY:
        case STEPPER_STATE_RUNNING_ACTIVE:
        case STEPPER_STATE_RUNNING_PASSIVE:
        case STEPPER_STATE_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_ERROR_WRONG_STATE; 
        default:
            break;
    }

    if(pStepper->isRampupPuleseWidthsPopulated == false)
    {
        return STEPPER_ERROR_WRONG_PULSE_ORDER;
    }

    pStepper->cruisePulseWidth = width;
    pStepper->isCruisePulseWidthPopulated = true;

    return STEPPER_OK;
}

StepperReturnCode stepper_set_active_rampdown_pulse_widths(
    const StepperId id, 
    const uint8_t * const pWidths, 
    const uint8_t length, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(NULL == pWidths)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }
    if(length < 2)
    {
        return STEPPER_ERROR_NO_PULSE;
    }
    if(length & 0x1)
    {
        return STEPPER_ERROR_INVALID_PULSE_LENGTH;
    }
    if(totalBatches < 1)
    {
        return STEPPER_ERROR_NO_PULSE;
    }

    const uint8_t count = length >> 1;
    const uint8_t lastBatchIndex = totalBatches - 1;
    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_HOME_BOUNDARY_TO_READY:
        case STEPPER_STATE_RUNNING_ACTIVE:
        case STEPPER_STATE_RUNNING_PASSIVE:
        case STEPPER_STATE_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_ERROR_WRONG_STATE; 
        default:
            break;
    }

    if(pStepper->isRampupPuleseWidthsPopulated == false ||
        pStepper->isCruisePulseWidthPopulated == false)
    {
        return STEPPER_ERROR_WRONG_PULSE_ORDER;
    }

    if(pStepper->rampdownPulseCount != MAX_AMOUNT_OF_PULSE_IN_BATCH * batchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }
    if(pStepper->pRampdownPulseWidths == NULL)
    {
        return STEPPER_ERROR_INTERNAL_DATA_ERROR;
    }
    if((pStepper->rampdownPulseCount + count) > MAX_RAMP_CLOCKS)
    {
        return STEPPER_ERROR_TOO_MANY_PULSE_WIDTHS;
    }

    for(uint8_t i=0; i<count; i++)
    {
        uint16_t width = pWidths[i * 2 + 1];
        width <<= 8;
        width += pWidths[i * 2];

        pStepper->pRampdownPulseWidths[pStepper->rampdownPulseCount + i] = width;
    }
    pStepper->rampdownPulseCount +=  count;

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
    pStepper->rampupPulseCount = 0;
    pStepper->isRampupPuleseWidthsPopulated = false;

    pStepper->isCruisePulseWidthPopulated = false;

    pStepper->pRampdownPulseWidths = NULL;
    pStepper->rampdownPulseCount = 0;
    pStepper->isRampdownPulseWidthsPopulated = false;

    pStepper->pPassiveStepArray = pStepper->uint16Array;
    pStepper->passiveStepsCount = 0;
    pStepper->passiveStepIndex = 0;
    pStepper->isPassiveStepsPopulated = false;
}

StepperReturnCode stepper_set_passive_step_indexes(
    const StepperId id, 
    const uint8_t * const pIndexes, 
    const uint8_t length, 
    const uint8_t batchIndex, 
    const uint8_t totalBatches)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(pIndexes == NULL)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }
    if(length < 2)
    {
        return STEPPER_ERROR_NO_INDEX;
    }
    if(length & 0x1)
    {
        return STEPPER_ERROR_INVALID_INDEX_LENGTH;
    }
    if(totalBatches < 1)
    {
        return STEPPER_ERROR_NO_INDEX;
    }
    const uint8_t lastBatchIndex = totalBatches - 1;

    if(batchIndex > lastBatchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }

    const uint8_t count = length >> 1;
    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_HOME_BOUNDARY_TO_READY:
        case STEPPER_STATE_RUNNING_ACTIVE:
        case STEPPER_STATE_RUNNING_PASSIVE:
        case STEPPER_STATE_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_ERROR_WRONG_STATE; 
        default:
            break;
    }

    if(batchIndex == 0)
    {
        _reset_passive_stepper_pulses(id);
    }

    if(pStepper->pPassiveStepArray == NULL)
    {
        return STEPPER_ERROR_INTERNAL_DATA_ERROR;
    }
    if(pStepper->passiveStepsCount != count * batchIndex)
    {
        return STEPPER_ERROR_WRONG_BATCH_INDEX;
    }
    if(pStepper->passiveStepsCount + count > MAX_UINT16_ARRAY_LENGTH)
    {
        return STEPPER_ERROR_TOO_MANY_PASSIVE_INDEXES;
    }
    if(pStepper->isForward)
    {
        if(pStepper->offset + pStepper->passiveStepsCount + count > pStepper->range)
        {
            return STEPPER_ERROR_WILL_OUT_OF_RANGE;
        }
    }
    else
    {
        if(pStepper->passiveStepsCount + count > pStepper->offset)
        {
            return STEPPER_ERROR_WILL_OUT_OF_RANGE;
        }
    }

    for(uint8_t i=0; i<count; i++)
    {
        uint16_t index = pIndexes[i * 2 + 1];
        index <<= 8;
        index += pIndexes[i * 2];

        pStepper->pPassiveStepArray[pStepper->passiveStepsCount + i] = index;
    }
    pStepper->passiveStepsCount += count;

    if(batchIndex == lastBatchIndex)
    {
        pStepper->isPassiveStepsPopulated = true;
    }

    return STEPPER_OK;
}

StepperReturnCode stepper_set_cross_boundary(
	const StepperId id,
	const bool crossBoundaryEnabled,
	const int32_t negativeRange,
	const bool boundary0Enabled,
	const int32_t boundary0Offset,
	const int16_t boundary0Error,
	const bool boundary1Enabled,
	const int32_t boundary1Offset,
	const int16_t boundary1Error,
	const bool boundary2Enabled,
	const int32_t boundary2Offset,
	const int16_t boundary2Error,
	const bool boundary3Enabled,
	const int32_t boundary3Offset,
	const int16_t boundary3Error)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(negativeRange > 0 ||
        boundary0Error < 0 ||
        boundary1Error < 0 ||
        boundary2Error < 0 ||
        boundary3Error < 0)
    {
        return STEPPER_ERROR_INVALID_PARAMETER;    
    }

    StepperData * pStepper = _steppers + (int)id;
    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_HOME_BOUNDARY_TO_READY:
        case STEPPER_STATE_RUNNING_ACTIVE:
        case STEPPER_STATE_RUNNING_PASSIVE:
        case STEPPER_STATE_RUNNING_FORCED:
            // cannot set pulses while stepper is moving
            return STEPPER_ERROR_WRONG_STATE;
        default:
            break;
    }

    pStepper->crossBoundary.enabled = crossBoundaryEnabled;
    pStepper->crossBoundary.negativeRange = negativeRange;

    pStepper->crossBoundary.boundaries[0].enabled = boundary0Enabled;
    pStepper->crossBoundary.boundaries[0].offset = boundary0Offset;
    pStepper->crossBoundary.boundaries[0].error = boundary0Error;

    pStepper->crossBoundary.boundaries[1].enabled = boundary1Enabled;
    pStepper->crossBoundary.boundaries[1].offset = boundary1Offset;
    pStepper->crossBoundary.boundaries[1].error = boundary1Error;

    pStepper->crossBoundary.boundaries[2].enabled = boundary2Enabled;
    pStepper->crossBoundary.boundaries[2].offset = boundary2Offset;
    pStepper->crossBoundary.boundaries[2].error = boundary2Error;

    pStepper->crossBoundary.boundaries[3].enabled = boundary3Enabled;
    pStepper->crossBoundary.boundaries[3].offset = boundary3Offset;
    pStepper->crossBoundary.boundaries[3].error = boundary3Error;

    return STEPPER_OK;
}

StepperReturnCode stepper_start_home_positioning(const StepperId id)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }
    if(!pStepper->isRampupPuleseWidthsPopulated)
    {
        return STEPPER_ERROR_WRONG_STATE;
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
    pStepper->state = STEPPER_STATE_RETURN_TO_HOME_BOUNDARY;
    pStepper->currentPulseWidth = pStepper->pRampupPulseWidths[0];

    return STEPPER_OK;
}

StepperReturnCode stepper_run_active(const StepperId id, const int32_t steps)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(steps <= 0)
    {
        return STEPPER_ERROR_INVALID_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_STATE_READY)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }
    if(!pStepper->isRampupPuleseWidthsPopulated ||
        !pStepper->isCruisePulseWidthPopulated ||
        !pStepper->isRampdownPulseWidthsPopulated)
    {
        return STEPPER_ERROR_INVALID_ACTIVE_PULSES;
    }
    if(pStepper->isForward)
    {
        if(pStepper->offset + steps > pStepper->range)
        {
            return STEPPER_ERROR_WILL_OUT_OF_RANGE;
        }
    }
    else
    {
        if(pStepper->crossBoundary.enabled)
        {
            if((pStepper->offset - steps) < pStepper->crossBoundary.negativeRange) 
            {
                return STEPPER_ERROR_WILL_OUT_OF_RANGE;
            }
        }
        else
        {
            if(pStepper->offset < steps)
            {
                return STEPPER_ERROR_WILL_OUT_OF_RANGE;
            }
        }
    }

    pStepper->stepsToRun = steps;
    pStepper->currentStep = 0;
    pStepper->currentPulseWidth = pStepper->pRampupPulseWidths[0];

    pStepper->state = STEPPER_STATE_RUNNING_ACTIVE;
    pStepper->activeSubState = ACCELERATING;

    return STEPPER_OK;
}

StepperReturnCode stepper_couple_passive(const StepperId activeStepperId, const StepperId passiveStepperId)
{
    if(activeStepperId >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(passiveStepperId >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(activeStepperId == passiveStepperId)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pActive = _steppers + (int)activeStepperId;
    StepperData * pPassive = _steppers + (int)passiveStepperId;

    if(pActive->state != STEPPER_STATE_RUNNING_ACTIVE)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }
    if(pPassive->state != STEPPER_STATE_READY)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }
    if(pPassive->isPassiveStepsPopulated == false)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    // check if passive stepper has been coupled with any stepper
    bool coupled = false;
    for(int i = 0; i < (int)STEPPER_ID_COUNT; i++)
    {
        StepperData * pStepper = _steppers + i;
        if(pStepper->passiveCoupled)
        {
            for(int j = 0; j < (int)STEPPER_ID_COUNT; j++)
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
        return SETPPER_ERROR_ALREADY_COUPLED;
    }

    pActive->passiveStepperIds[passiveStepperId] = passiveStepperId;
    pActive->passiveCoupled = true;

    pPassive->passiveStepIndex = 0;
    pPassive->currentPulseWidth = 0;
    pPassive->state = STEPPER_STATE_RUNNING_PASSIVE;

    return STEPPER_OK;
}

StepperReturnCode stepper_run_force(const StepperId id, const uint16_t pulseWidth, const uint16_t steps)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ID_INVALID;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    pStepper->forcePulseWidth = pulseWidth;
    pStepper->stepsToRun = steps;
    pStepper->currentStep = 0;
    pStepper->state = STEPPER_STATE_RUNNING_FORCED;

    return STEPPER_OK;
}

StepperReturnCode stepper_get_state(const StepperId id, StepperState * const pState)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ID_INVALID;
    }
    if(pState == NULL)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    *pState = pStepper->state;

    return STEPPER_OK;
}

StepperReturnCode stepper_get_status(const StepperId id, uint8_t * const p_buffer, uint8_t * p_status_length)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(p_buffer == NULL || p_status_length == NULL)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }

    uint8_t tmpByte;
    uint32_t tmpInt;
    StepperData * pStepper = _steppers + (int)id;
    
    p_buffer[0] = (uint8_t)pStepper->state;

    tmpByte = 0;
    if(pStepper->isForward)
        tmpByte |= 0x01;
    if(pStepper->isEnabled)
        tmpByte |= 0x02;
    if(_is_stepper_at_end_boundary(pStepper))
        tmpByte |= 0x04;
    if(_is_stepper_at_home_boundary(pStepper))
        tmpByte |= 0x08;
    if(pStepper->isEnableHigh)
        tmpByte |= 0x10;
    if(pStepper->isForwardHigh)
        tmpByte |= 0x20;
    if(pStepper->isRisingEdgeDriven)
        tmpByte |= 0x40;
    p_buffer[1] = tmpByte;

    tmpByte = 0;
    if(pStepper->isPassiveStepsPopulated)
        tmpByte |= 0x01;
    if(pStepper->isRampdownPulseWidthsPopulated)
        tmpByte |= 0x02;
    if(pStepper->isCruisePulseWidthPopulated)
        tmpByte |= 0x04;
    if(pStepper->isRampupPuleseWidthsPopulated)
        tmpByte |= 0x08;
    p_buffer[2] = tmpByte;

    tmpInt = pStepper->offset;
    p_buffer[3] = tmpInt;
    tmpInt >>= 8;
    p_buffer[4] = tmpInt;
    tmpInt >>=8;
    p_buffer[5] = tmpInt;
    tmpInt >>= 8;
    p_buffer[6] = tmpInt;

    tmpInt = pStepper->encoderOffset;
    p_buffer[7] = tmpInt;
    tmpInt >>= 8;
    p_buffer[8] = tmpInt;
    tmpInt >>=8;
    p_buffer[9] = tmpInt;
    tmpInt >>= 8;
    p_buffer[10] = tmpInt;

    p_buffer[11] = pStepper->maxEncoderOffsetError;
    
    *p_status_length = 12;
    
    return STEPPER_OK;
}

StepperReturnCode stepper_get_startup_pulse_width(const StepperId id, uint16_t * const pPulseWidth)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(pPulseWidth == NULL)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    switch(pStepper->state)
    {
        case STEPPER_STATE_RETURN_TO_HOME_BOUNDARY:
        case STEPPER_STATE_RUNNING_ACTIVE:
            *pPulseWidth = pStepper->pRampupPulseWidths[0];
            return STEPPER_OK;
        
        case STEPPER_STATE_RUNNING_FORCED:
            *pPulseWidth = pStepper->forcePulseWidth;
            return STEPPER_OK;

        default:
            return STEPPER_ERROR_WRONG_STATE;
    }
}

static StepperReturnCode _on_stepper_pulse_end_force(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        // first clock pulse has finished
        _set_clock_second_half(pStepper);
        *pNextPulseWidth = pStepper->forcePulseWidth;

        return STEPPER_OK;
    }

    // second clock pulse has finished
    _set_clock_first_half(pStepper);

    pStepper->currentStep += 1;
    if(pStepper->currentStep < pStepper->stepsToRun)
    {
        *pNextPulseWidth = pStepper->forcePulseWidth;

        return STEPPER_OK;
    }

    // has run designated steps
    pStepper->currentStep = 0;
    pStepper->stepsToRun = 0;
    pStepper->state = STEPPER_STATE_INITIALIZED;
    *pNextPulseWidth = 0;

    return STEPPER_OK;
}

static StepperReturnCode _on_stepper_pulse_end_active(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        // first clock pulse has finished, check if stepper has arrived at the expected position
        if(pStepper->encoderId != ENCODER_ID_INVALID)
        {
            bool inSync = _is_stepper_in_sync(pStepper);
            if(!inSync)
            {
                on_stepper_out_of_sync_interrupt(pStepper->stepperId);
                print_log("Error: stepper %d out of sync, stepper offset: %d, encoder offset: %d\r\n", (int)pStepper->stepperId, pStepper->offset, pStepper->encoderOffset);
                *pNextPulseWidth = 0;
                pStepper->state = STEPPER_STATE_OUT_OF_SYNC;
                return STEPPER_ERROR_OUT_OF_SYNC;
            }
        }
        
        // start the second clock pulse to make the stepper move further
        const StepperReturnCode rc = _notify_passive_steppers(pStepper);
        _set_clock_second_half(pStepper);
        if(rc != STEPPER_OK)
        {
            *pNextPulseWidth = 0;
            return rc;
        }

        *pNextPulseWidth = pStepper->currentPulseWidth;

        return STEPPER_OK;
    }

    // second half of clock pulse has finished
    const StepperReturnCode rc = _notify_passive_steppers(pStepper);
    _set_clock_first_half(pStepper);
    if(rc != STEPPER_OK)
    {
        *pNextPulseWidth = 0;
        return rc;
    }

    // check if stepper is out of range
    if(pStepper->isForward)
    {
        pStepper->offset++;
        if(pStepper->offset > pStepper->range)
        {
            *pNextPulseWidth = 0;
            return STEPPER_ERROR_OUT_OF_RANGE;
        }
    }
    else
    {
        pStepper->offset--;
        if(pStepper->offset < 0)
        {
        	if(pStepper->crossBoundary.enabled)
        	{
        		if(pStepper->offset < pStepper->crossBoundary.negativeRange)
        		{
            		*pNextPulseWidth = 0;
            		return STEPPER_ERROR_OUT_OF_RANGE;
        		}
        	}
        	else
        	{
        		*pNextPulseWidth = 0;
        		return STEPPER_ERROR_OUT_OF_RANGE;
        	}
        }
    }

    if(pStepper->currentStep == 0)
    {
        // print_string("Stepper ");
        // print_uint8_hex(pStepper->stepperId);
        // print_string(" rampUp, stepIndex: ");
        // print_uint32_hex(pStepper->currentStep);
        // print_string(", width: ");
        // print_uint16_hex(pStepper->currentPulseWidth);
        // print_string("\r\n");
    }

    pStepper->currentStep += 1;
    if(pStepper->currentStep == pStepper->stepsToRun)
    {
        // print_string("Stepper ");
        // print_uint8_hex(pStepper->stepperId);
        // print_string(" stop, stepIndex: ");
        // print_uint32_hex(pStepper->currentStep);
        // print_string(", width: ");
        // print_uint16_hex(pStepper->currentPulseWidth);
        // print_string("\r\n");

        // has run the designated steps
        pStepper->currentStep = 0;
        pStepper->stepsToRun = 0;
        pStepper->state = STEPPER_STATE_READY;
        *pNextPulseWidth = 0;
        return STEPPER_OK;
    }

    // calculate the width of next pulse
    uint16_t width = pStepper->cruisePulseWidth; // the default
    bool beforeMidPoint = pStepper->currentStep < (pStepper->stepsToRun >> 1);
    if(beforeMidPoint)
    {
        if(pStepper->currentStep < pStepper->rampupPulseCount)
        {
            // accelerating
            width = pStepper->pRampupPulseWidths[pStepper->currentStep];
        }
        else
        {
            // cruising
            if(pStepper->activeSubState != CRUISING)
            {
                // print_string("Stepper ");
                // print_uint8_hex(pStepper->stepperId);
                // print_string(" cruise, stepIndex: ");
                // print_uint32_hex(pStepper->currentStep);
                // print_string(", width: ");
                // print_uint16_hex(width);
                // print_string("\r\n");

                pStepper->activeSubState = CRUISING;
            }
        }
    }
    else
    {
        uint32_t remainingSteps = pStepper->stepsToRun - pStepper->currentStep;
        if(pStepper->rampdownPulseCount >= remainingSteps)
        {
            // deaccelerating
            uint32_t index = pStepper->rampdownPulseCount - remainingSteps;
            width = pStepper->pRampdownPulseWidths[index];

            if(pStepper->activeSubState != DEACCELERATING)
            {
                // print_string("Stepper ");
                // print_uint8_hex(pStepper->stepperId);
                // print_string(" rampDown, stepIndex: ");
                // print_uint32_hex(pStepper->currentStep);
                // print_string(", width: ");
                // print_uint16_hex(width);
                // print_string("\r\n");

                pStepper->activeSubState = DEACCELERATING;
            }
        }
    }
    *pNextPulseWidth = width;
    
    pStepper->currentPulseWidth = width;

    return STEPPER_OK;
}

static StepperReturnCode _on_stepper_pulse_end_to_home(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        _set_clock_second_half(pStepper);
        *pNextPulseWidth = pStepper->currentPulseWidth;

        return STEPPER_OK;
    }

    // second half of clock pulse has finished.
    _set_clock_first_half(pStepper);
    *pNextPulseWidth = pStepper->currentPulseWidth;

    bool atHomeBoundary = _is_stepper_at_home_boundary(pStepper);
    if(!atHomeBoundary)
    {
        return STEPPER_OK;
    }

    // stepper arrives at home boundary
    _stepper_set_forward(pStepper, true);
    pStepper->state = STEPPER_STATE_HOME_BOUNDARY_TO_READY;

    return STEPPER_OK;
}

static StepperReturnCode _on_stepper_pulse_end_home_to_ready(StepperData * const pStepper, uint16_t * const pNextPulseWidth)
{
    if(pStepper->pulseState == FIRST_HALF)
    {
        _set_clock_second_half(pStepper);
        *pNextPulseWidth = pStepper->currentPulseWidth;

        return STEPPER_OK;
    }

    // second half of clock pulse has finished.
    _set_clock_first_half(pStepper);
    *pNextPulseWidth = pStepper->currentPulseWidth;

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
        
        pStepper->currentStep = 0;
        pStepper->offset = 0;
        pStepper->state = STEPPER_STATE_READY;
        if(pStepper->encoderId < ENCODER_ID_COUNT)
        {
            encoder_reset(pStepper->encoderId);
            pStepper->encoderHomePosition = encoder_get_offset(pStepper->encoderId);
            pStepper->encoderOffset = 0;
        }

        _boundary_detector_flips(pStepper); // init static data in function.
    }

    return STEPPER_OK;
}

StepperReturnCode on_interupt_stepper_pulse_end(const StepperId id, uint16_t * const pNextPulseWidth)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    if(pNextPulseWidth == NULL)
    {
        return STEPPER_ERROR_NULL_PARAMETER;
    }

    StepperData * pStepper = _steppers + (int)id;

    if(pStepper->state == STEPPER_STATE_RUNNING_ACTIVE)
    {
        if(pStepper->currentStep >= pStepper->stepsToRun)
        {
            return STEPPER_ERROR_OUT_OF_STEPS;
        }

        if(_is_stepper_out_of_range(pStepper))
        {
        	if(pStepper->crossBoundary.enabled)
        	{
        		pStepper->state = STEPPER_STATE_OUT_OF_SYNC;
        		on_stepper_out_of_sync_interrupt(id);
        	}
        	else
        	{
        		pStepper->state = STEPPER_STATE_OUT_OF_BOUNDARY;
        		on_stepper_out_of_scope_interrupt(id);
        	}
        	*pNextPulseWidth = 0;

        	return STEPPER_ERROR_OUT_OF_RANGE;
        }

        StepperReturnCode returnCode = _on_stepper_pulse_end_active(pStepper, pNextPulseWidth);
        return returnCode;
    }   
    
    if(pStepper->state == STEPPER_STATE_RUNNING_FORCED)
    {
        if(pStepper->currentStep >= pStepper->stepsToRun)
        {
            return STEPPER_ERROR_OUT_OF_STEPS;
        }

        StepperReturnCode returnCode = _on_stepper_pulse_end_force(pStepper, pNextPulseWidth);
        return returnCode;
    }

    if(pStepper->state == STEPPER_STATE_RETURN_TO_HOME_BOUNDARY)
    {
        StepperReturnCode returnCode = _on_stepper_pulse_end_to_home(pStepper, pNextPulseWidth);
        return returnCode;
    }

    if(pStepper->state == STEPPER_STATE_HOME_BOUNDARY_TO_READY)
    {
        StepperReturnCode returnCode = _on_stepper_pulse_end_home_to_ready(pStepper, pNextPulseWidth);
        return returnCode;
    }
    
    return STEPPER_ERROR_WRONG_STATE;
}

StepperReturnCode stepper_test_signal_enable(const StepperId id, const bool isEnable)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }
    
    StepperData * pStepper = _steppers + (int)id;
    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    GPIO_PinState pinState;
    if(pStepper->isEnableHigh)
    {
        pinState = isEnable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    else
    {
        pinState = isEnable ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortEnable, 0x1 << pStepper->gpioPinIndexEnable, pinState);
    if(pinState != HAL_GPIO_ReadPin(pStepper->pGpioPortEnable, 0x1 << pStepper->gpioPinIndexEnable))
    {
        return STEPPER_ERROR_GPIO_RW_FAILURE;
    }

    pStepper->isEnabled = isEnable;

    return STEPPER_OK;
}

StepperReturnCode stepper_test_signal_forward(const StepperId id, const bool isForward)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;
    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    GPIO_PinState pinState;
    if(pStepper->isForwardHigh)
    {
        pinState = isForward ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }
    else
    {
        pinState = isForward ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortForward, 0x1 << pStepper->gpioPinIndexForward, pinState);
    if(pinState != HAL_GPIO_ReadPin(pStepper->pGpioPortForward, 0x1 << pStepper->gpioPinIndexForward))
    {
        return STEPPER_ERROR_GPIO_RW_FAILURE;
    }
    
    pStepper->isForward = isForward;

    return STEPPER_OK;
}

StepperReturnCode stepper_test_signal_clock(const StepperId id, const bool isFirstHalf)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;
    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    GPIO_PinState pinState;
    if(pStepper->isRisingEdgeDriven)
    {
        pinState = isFirstHalf ? GPIO_PIN_RESET : GPIO_PIN_SET;
    }
    else
    {
        pinState = isFirstHalf ? GPIO_PIN_SET : GPIO_PIN_RESET;
    }

    HAL_GPIO_WritePin(pStepper->pGpioPortClock, 0x1 << pStepper->gpioPinIndexClock, pinState);
    if(pinState != HAL_GPIO_ReadPin(pStepper->pGpioPortClock, 0x1 << pStepper->gpioPinIndexClock))
    {
        return STEPPER_ERROR_GPIO_RW_FAILURE;
    }
    
    pStepper->pulseState = isFirstHalf ? FIRST_HALF : SECOND_HALF;

    return STEPPER_OK;
}

StepperReturnCode stepper_test_state_ready(const StepperId id)
{
    if(id >= STEPPER_ID_COUNT)
    {
        return STEPPER_ERROR_INVALID_ID;
    }

    StepperData * pStepper = _steppers + (int)id;
    if(pStepper->state != STEPPER_STATE_INITIALIZED)
    {
        return STEPPER_ERROR_WRONG_STATE;
    }

    pStepper->isForward = true;
    pStepper->offset = 0;
    if(pStepper->encoderId != ENCODER_ID_INVALID)
    {
        pStepper->encoderOffset = 0;
        pStepper->encoderHomePosition = encoder_get_offset(pStepper->encoderId);
        pStepper->maxEncoderOffsetError = 0;
    }

    pStepper->stepsToRun = 0;
    pStepper->currentStep = 0;
    pStepper->currentPulseWidth = 0;
    pStepper->pulseState = FIRST_HALF;
    for(int i=0; i < STEPPER_ID_COUNT; i++)
    {
        pStepper->passiveStepperIds[i] = STEPPER_ID_INVALID;
    }
    pStepper->passiveCoupled = false;

    pStepper->state = STEPPER_STATE_READY;

    return STEPPER_OK;
}

