#include "encoder.h"
#include "stm32h7xx_hal.h"

extern LPTIM_HandleTypeDef hlptim1;
extern LPTIM_HandleTypeDef hlptim2;
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

uint16_t encoder_get_count(const EncoderId encoderId)
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


