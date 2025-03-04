/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GP0_in_Pin GPIO_PIN_2
#define GP0_in_GPIO_Port GPIOE
#define GP1_out_Pin GPIO_PIN_3
#define GP1_out_GPIO_Port GPIOE
#define GP2_out_Pin GPIO_PIN_4
#define GP2_out_GPIO_Port GPIOE
#define CLK_STP0_Pin GPIO_PIN_5
#define CLK_STP0_GPIO_Port GPIOE
#define GP3_out_Pin GPIO_PIN_6
#define GP3_out_GPIO_Port GPIOE
#define GP4_in_Pin GPIO_PIN_8
#define GP4_in_GPIO_Port GPIOI
#define GP5_out_Pin GPIO_PIN_13
#define GP5_out_GPIO_Port GPIOC
#define GP6_in_Pin GPIO_PIN_14
#define GP6_in_GPIO_Port GPIOC
#define GP7_out_Pin GPIO_PIN_15
#define GP7_out_GPIO_Port GPIOC
#define GP8_out_Pin GPIO_PIN_9
#define GP8_out_GPIO_Port GPIOI
#define GP9_in_Pin GPIO_PIN_10
#define GP9_in_GPIO_Port GPIOI
#define GP10_in_Pin GPIO_PIN_0
#define GP10_in_GPIO_Port GPIOF
#define GP11_in_Pin GPIO_PIN_1
#define GP11_in_GPIO_Port GPIOF
#define GP12_in_Pin GPIO_PIN_2
#define GP12_in_GPIO_Port GPIOF
#define GP13_in_Pin GPIO_PIN_12
#define GP13_in_GPIO_Port GPIOI
#define GP14_in_Pin GPIO_PIN_13
#define GP14_in_GPIO_Port GPIOI
#define GP15_in_Pin GPIO_PIN_14
#define GP15_in_GPIO_Port GPIOI
#define GP16_in_Pin GPIO_PIN_3
#define GP16_in_GPIO_Port GPIOF
#define GP71_in_Pin GPIO_PIN_4
#define GP71_in_GPIO_Port GPIOF
#define GP18_in_Pin GPIO_PIN_5
#define GP18_in_GPIO_Port GPIOF
#define CLK_STP1_Pin GPIO_PIN_6
#define CLK_STP1_GPIO_Port GPIOF
#define CLK_STP2_Pin GPIO_PIN_7
#define CLK_STP2_GPIO_Port GPIOF
#define GP19_in_Pin GPIO_PIN_8
#define GP19_in_GPIO_Port GPIOF
#define GP20_in_Pin GPIO_PIN_9
#define GP20_in_GPIO_Port GPIOF
#define GP21_in_Pin GPIO_PIN_10
#define GP21_in_GPIO_Port GPIOF
#define GP22_in_Pin GPIO_PIN_1
#define GP22_in_GPIO_Port GPIOC
#define GP23_in_Pin GPIO_PIN_2
#define GP23_in_GPIO_Port GPIOC
#define EA_0_Pin GPIO_PIN_0
#define EA_0_GPIO_Port GPIOA
#define EB_0_Pin GPIO_PIN_1
#define EB_0_GPIO_Port GPIOA
#define GP24_in_Pin GPIO_PIN_2
#define GP24_in_GPIO_Port GPIOA
#define EA_1_Pin GPIO_PIN_2
#define EA_1_GPIO_Port GPIOH
#define GP25_in_Pin GPIO_PIN_3
#define GP25_in_GPIO_Port GPIOH
#define GP26_in_Pin GPIO_PIN_4
#define GP26_in_GPIO_Port GPIOH
#define GP27_in_Pin GPIO_PIN_5
#define GP27_in_GPIO_Port GPIOH
#define GP28_in_Pin GPIO_PIN_4
#define GP28_in_GPIO_Port GPIOA
#define CLK_STP3_Pin GPIO_PIN_6
#define CLK_STP3_GPIO_Port GPIOA
#define CLK_STP4_Pin GPIO_PIN_7
#define CLK_STP4_GPIO_Port GPIOA
#define GP29_in_Pin GPIO_PIN_4
#define GP29_in_GPIO_Port GPIOC
#define GP30_in_Pin GPIO_PIN_5
#define GP30_in_GPIO_Port GPIOC
#define GP32_in_Pin GPIO_PIN_2
#define GP32_in_GPIO_Port GPIOB
#define GP32_inI15_Pin GPIO_PIN_15
#define GP32_inI15_GPIO_Port GPIOI
#define GP33_in_Pin GPIO_PIN_0
#define GP33_in_GPIO_Port GPIOJ
#define GP34_in_Pin GPIO_PIN_1
#define GP34_in_GPIO_Port GPIOJ
#define GP35_in_Pin GPIO_PIN_2
#define GP35_in_GPIO_Port GPIOJ
#define GP36_in_Pin GPIO_PIN_3
#define GP36_in_GPIO_Port GPIOJ
#define GP37_in_Pin GPIO_PIN_4
#define GP37_in_GPIO_Port GPIOJ
#define GP38_in_Pin GPIO_PIN_11
#define GP38_in_GPIO_Port GPIOF
#define GP39_in_Pin GPIO_PIN_12
#define GP39_in_GPIO_Port GPIOF
#define GP40_in_Pin GPIO_PIN_13
#define GP40_in_GPIO_Port GPIOF
#define GP41_in_Pin GPIO_PIN_14
#define GP41_in_GPIO_Port GPIOF
#define GP42_in_Pin GPIO_PIN_15
#define GP42_in_GPIO_Port GPIOF
#define GP43_in_Pin GPIO_PIN_0
#define GP43_in_GPIO_Port GPIOG
#define GP44_in_Pin GPIO_PIN_1
#define GP44_in_GPIO_Port GPIOG
#define GP45_in_Pin GPIO_PIN_7
#define GP45_in_GPIO_Port GPIOE
#define GP46_in_Pin GPIO_PIN_8
#define GP46_in_GPIO_Port GPIOE
#define EA_2_Pin GPIO_PIN_9
#define EA_2_GPIO_Port GPIOE
#define GP47_in_Pin GPIO_PIN_10
#define GP47_in_GPIO_Port GPIOE
#define EB_2_Pin GPIO_PIN_11
#define EB_2_GPIO_Port GPIOE
#define GP48_in_Pin GPIO_PIN_12
#define GP48_in_GPIO_Port GPIOE
#define GP49_in_Pin GPIO_PIN_13
#define GP49_in_GPIO_Port GPIOE
#define GP50_in_Pin GPIO_PIN_14
#define GP50_in_GPIO_Port GPIOE
#define GP51_in_Pin GPIO_PIN_15
#define GP51_in_GPIO_Port GPIOE
#define GP52_out_Pin GPIO_PIN_5
#define GP52_out_GPIO_Port GPIOJ
#define CLK_STP5_Pin GPIO_PIN_6
#define CLK_STP5_GPIO_Port GPIOH
#define GP53_out_Pin GPIO_PIN_7
#define GP53_out_GPIO_Port GPIOH
#define GP54_out_Pin GPIO_PIN_8
#define GP54_out_GPIO_Port GPIOH
#define GP55_out_Pin GPIO_PIN_9
#define GP55_out_GPIO_Port GPIOH
#define EA_3_Pin GPIO_PIN_10
#define EA_3_GPIO_Port GPIOH
#define EB_3_Pin GPIO_PIN_11
#define EB_3_GPIO_Port GPIOH
#define GP56_out_Pin GPIO_PIN_12
#define GP56_out_GPIO_Port GPIOH
#define USB_DEC_1_out_Pin GPIO_PIN_8
#define USB_DEC_1_out_GPIO_Port GPIOD
#define GP57_out_Pin GPIO_PIN_9
#define GP57_out_GPIO_Port GPIOD
#define GP58_out_Pin GPIO_PIN_10
#define GP58_out_GPIO_Port GPIOD
#define EB_4_Pin GPIO_PIN_11
#define EB_4_GPIO_Port GPIOD
#define EA_4_Pin GPIO_PIN_12
#define EA_4_GPIO_Port GPIOD
#define EB_5_Pin GPIO_PIN_13
#define EB_5_GPIO_Port GPIOD
#define GP59_out_Pin GPIO_PIN_14
#define GP59_out_GPIO_Port GPIOD
#define GP60_out_Pin GPIO_PIN_15
#define GP60_out_GPIO_Port GPIOD
#define EB_6_Pin GPIO_PIN_6
#define EB_6_GPIO_Port GPIOJ
#define GP61_out_Pin GPIO_PIN_7
#define GP61_out_GPIO_Port GPIOJ
#define EA_6_Pin GPIO_PIN_8
#define EA_6_GPIO_Port GPIOJ
#define GP62_out_Pin GPIO_PIN_9
#define GP62_out_GPIO_Port GPIOJ
#define GP63_out_Pin GPIO_PIN_10
#define GP63_out_GPIO_Port GPIOJ
#define GP64_out_Pin GPIO_PIN_11
#define GP64_out_GPIO_Port GPIOJ
#define GP65_out_Pin GPIO_PIN_0
#define GP65_out_GPIO_Port GPIOK
#define GP66_out_Pin GPIO_PIN_1
#define GP66_out_GPIO_Port GPIOK
#define GP67_out_Pin GPIO_PIN_2
#define GP67_out_GPIO_Port GPIOK
#define GP68_out_Pin GPIO_PIN_2
#define GP68_out_GPIO_Port GPIOG
#define GP69_out_Pin GPIO_PIN_3
#define GP69_out_GPIO_Port GPIOG
#define GP70_out_Pin GPIO_PIN_4
#define GP70_out_GPIO_Port GPIOG
#define GP71_out_Pin GPIO_PIN_5
#define GP71_out_GPIO_Port GPIOG
#define GP72_out_Pin GPIO_PIN_6
#define GP72_out_GPIO_Port GPIOG
#define GP73_out_Pin GPIO_PIN_7
#define GP73_out_GPIO_Port GPIOG
#define GP74_out_Pin GPIO_PIN_8
#define GP74_out_GPIO_Port GPIOG
#define EA_7_Pin GPIO_PIN_6
#define EA_7_GPIO_Port GPIOC
#define EB_7_Pin GPIO_PIN_7
#define EB_7_GPIO_Port GPIOC
#define GP75_in_Pin GPIO_PIN_8
#define GP75_in_GPIO_Port GPIOC
#define GP76_in_Pin GPIO_PIN_9
#define GP76_in_GPIO_Port GPIOC
#define USB_DEC_0_in_Pin GPIO_PIN_13
#define USB_DEC_0_in_GPIO_Port GPIOH
#define GP77_in_Pin GPIO_PIN_14
#define GP77_in_GPIO_Port GPIOH
#define GP78_out_Pin GPIO_PIN_15
#define GP78_out_GPIO_Port GPIOH
#define GP79_out_Pin GPIO_PIN_0
#define GP79_out_GPIO_Port GPIOI
#define GP80_in_Pin GPIO_PIN_1
#define GP80_in_GPIO_Port GPIOI
#define GP81_in_Pin GPIO_PIN_2
#define GP81_in_GPIO_Port GPIOI
#define GP82_out_Pin GPIO_PIN_3
#define GP82_out_GPIO_Port GPIOI
#define GP83_in_Pin GPIO_PIN_10
#define GP83_in_GPIO_Port GPIOC
#define GP84_out_Pin GPIO_PIN_11
#define GP84_out_GPIO_Port GPIOC
#define GP85_in_Pin GPIO_PIN_12
#define GP85_in_GPIO_Port GPIOC
#define GP86_in_Pin GPIO_PIN_0
#define GP86_in_GPIO_Port GPIOD
#define GP87_in_Pin GPIO_PIN_1
#define GP87_in_GPIO_Port GPIOD
#define GP88_in_Pin GPIO_PIN_2
#define GP88_in_GPIO_Port GPIOD
#define GP89_in_Pin GPIO_PIN_3
#define GP89_in_GPIO_Port GPIOD
#define GP90_in_Pin GPIO_PIN_4
#define GP90_in_GPIO_Port GPIOD
#define GP91_in_Pin GPIO_PIN_5
#define GP91_in_GPIO_Port GPIOD
#define GP92_in_Pin GPIO_PIN_6
#define GP92_in_GPIO_Port GPIOD
#define GP93_in_Pin GPIO_PIN_7
#define GP93_in_GPIO_Port GPIOD
#define GP94_in_Pin GPIO_PIN_12
#define GP94_in_GPIO_Port GPIOJ
#define GP95_in_Pin GPIO_PIN_13
#define GP95_in_GPIO_Port GPIOJ
#define GP96_out_Pin GPIO_PIN_14
#define GP96_out_GPIO_Port GPIOJ
#define GP97_in_Pin GPIO_PIN_15
#define GP97_in_GPIO_Port GPIOJ
#define GP98_out_Pin GPIO_PIN_9
#define GP98_out_GPIO_Port GPIOG
#define GP99_in_Pin GPIO_PIN_10
#define GP99_in_GPIO_Port GPIOG
#define GP100_in_Pin GPIO_PIN_11
#define GP100_in_GPIO_Port GPIOG
#define EB_1_Pin GPIO_PIN_12
#define EB_1_GPIO_Port GPIOG
#define GP101_in_Pin GPIO_PIN_13
#define GP101_in_GPIO_Port GPIOG
#define GP102_in_Pin GPIO_PIN_14
#define GP102_in_GPIO_Port GPIOG
#define GP103_in_Pin GPIO_PIN_3
#define GP103_in_GPIO_Port GPIOK
#define GP104_out_Pin GPIO_PIN_4
#define GP104_out_GPIO_Port GPIOK
#define GP105_in_Pin GPIO_PIN_5
#define GP105_in_GPIO_Port GPIOK
#define GP106_in_Pin GPIO_PIN_6
#define GP106_in_GPIO_Port GPIOK
#define GP107_out_Pin GPIO_PIN_7
#define GP107_out_GPIO_Port GPIOK
#define GP108_in_Pin GPIO_PIN_15
#define GP108_in_GPIO_Port GPIOG
#define GP109_in_Pin GPIO_PIN_4
#define GP109_in_GPIO_Port GPIOB
#define EA_5_Pin GPIO_PIN_6
#define EA_5_GPIO_Port GPIOB
#define GP110_in_Pin GPIO_PIN_7
#define GP110_in_GPIO_Port GPIOB
#define GP111_in_Pin GPIO_PIN_8
#define GP111_in_GPIO_Port GPIOB
#define GP112_out_Pin GPIO_PIN_9
#define GP112_out_GPIO_Port GPIOB
#define GP113_out_Pin GPIO_PIN_0
#define GP113_out_GPIO_Port GPIOE
#define GP114_out_Pin GPIO_PIN_1
#define GP114_out_GPIO_Port GPIOE
#define GP115_out_Pin GPIO_PIN_4
#define GP115_out_GPIO_Port GPIOI
#define GP116_in_Pin GPIO_PIN_5
#define GP116_in_GPIO_Port GPIOI
#define GP117_in_Pin GPIO_PIN_6
#define GP117_in_GPIO_Port GPIOI
#define GP118_out_Pin GPIO_PIN_7
#define GP118_out_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
