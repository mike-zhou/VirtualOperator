/*
 * position_detector.c
 *
 *  Created on: Jul 13, 2025
 *      Author: mike
 */

#include "stm32h753xx.h"

#include "app.h"
#include "position_detector.h"

#define GPIO_PORT_COUNT 11
static uint16_t _gpio_position_detector_masks[GPIO_PORT_COUNT];
static uint16_t _gpio_position_detector_critical_masks[GPIO_PORT_COUNT];

void position_detector_init_data()
{
    for(int i=0; i<GPIO_PORT_COUNT; i++)
    {
        _gpio_position_detector_masks[i] = 0;
        _gpio_position_detector_critical_masks[i] = 0;
    }
}

bool position_detector_set_masks(const uint16_t * const pMasks, const uint16_t * const pCriticalMasks, const uint8_t length)
{
    if(length != GPIO_PORT_COUNT)
    {
        return false;
    }

    for(int i=0; i<GPIO_PORT_COUNT; i++)
    {
        _gpio_position_detector_masks[i] = pMasks[i];
        _gpio_position_detector_critical_masks[i] = pCriticalMasks[i];
    }

    return true;
}

void poll_position_detector()
{
    static GPIO_TypeDef * ports[GPIO_PORT_COUNT] = {
        GPIOA,
        GPIOB,
        GPIOC,
        GPIOD,
        GPIOE,
        GPIOF,
        GPIOG,
        GPIOH,
        GPIOI,
        GPIOJ,
        GPIOK
    };

    for(int i=0; i<GPIO_PORT_COUNT; i++)
    {
        if(ports[i]->IDR & _gpio_position_detector_critical_masks[i])
        {
            on_position_detector_critical_mask();
        }
    }
}