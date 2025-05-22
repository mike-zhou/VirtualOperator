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
    uint32_t maxSteps;
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

    // indexes for slave 
    uint16_t * pSlaveStepArray;
    uint32_t slaveStepsCount;
    uint32_t slaveStepIndex;
    bool isSlaveStepsInitialized;

    /**
     * dynamic data
     */

    StepperState stepperState;

    bool isEnabled;
    bool isForward;

    uint32_t currentPosition;

    uint32_t stepsToRun;
    uint32_t currentStep;

    PulseState pulseState;

    StepperId slaveIds[STEPPER_COUNT];
    bool hasSlaves;
} StepperData;

static StepperData steppers[STEPPER_COUNT];

/**
 * This function is called by the master stepper to notify the slave stepper 
 * of its current stepIndex.
 * If stepIndex is what the slave expected, the slave needs to drive its
 * clock pin according to pulseState.
 */
static bool on_master_step(StepperId slaveId, uint32_t stepIndex, PulseState pulseState)
{

}

static uint16_t get_encoder(EncoderId encoderId)
{
    
}

void stepper_init_data_structure()
{
    for(uint8_t i=0; i<STEPPER_COUNT; i++)
    {
        steppers[i].stepperState = UNINITIALIZED;
        steppers[i].isStepperControlInitialized = false;
        steppers[i].isRampupPuleseWidthsPopulated = false;
        steppers[i].isRampdownPulseWidthsPopulated = false;
        steppers[i].isCruisePulseWidthPopulated = false;
    }
}

