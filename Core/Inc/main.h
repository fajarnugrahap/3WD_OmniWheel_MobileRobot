/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32g4xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define TIM1_CH1_Pin GPIO_PIN_0
#define TIM1_CH1_GPIO_Port GPIOC
#define TIM1_CH2_Pin GPIO_PIN_1
#define TIM1_CH2_GPIO_Port GPIOC
#define TIM20_CH2_Pin GPIO_PIN_2
#define TIM20_CH2_GPIO_Port GPIOC
#define TIM2_CH1_Pin GPIO_PIN_0
#define TIM2_CH1_GPIO_Port GPIOA
#define TIM2_CH2_Pin GPIO_PIN_1
#define TIM2_CH2_GPIO_Port GPIOA
#define LPUART1_TX_Pin GPIO_PIN_2
#define LPUART1_TX_GPIO_Port GPIOA
#define LPUART1_RX_Pin GPIO_PIN_3
#define LPUART1_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define MLEFT_A_Pin GPIO_PIN_6
#define MLEFT_A_GPIO_Port GPIOA
#define MLEFT_B_Pin GPIO_PIN_7
#define MLEFT_B_GPIO_Port GPIOA
#define MBACK_A_Pin GPIO_PIN_4
#define MBACK_A_GPIO_Port GPIOC
#define MBACK_B_Pin GPIO_PIN_5
#define MBACK_B_GPIO_Port GPIOC
#define MRIGHT_A_Pin GPIO_PIN_0
#define MRIGHT_A_GPIO_Port GPIOB
#define MRIGHT_B_Pin GPIO_PIN_1
#define MRIGHT_B_GPIO_Port GPIOB
#define TIM20_CH1_Pin GPIO_PIN_2
#define TIM20_CH1_GPIO_Port GPIOB
#define PBRight_Pin GPIO_PIN_10
#define PBRight_GPIO_Port GPIOB
#define TIM8_CH1_Pin GPIO_PIN_6
#define TIM8_CH1_GPIO_Port GPIOC
#define TIM8_CH2_Pin GPIO_PIN_7
#define TIM8_CH2_GPIO_Port GPIOC
#define TIM20_CH3_Pin GPIO_PIN_8
#define TIM20_CH3_GPIO_Port GPIOC
#define PBDown_Pin GPIO_PIN_8
#define PBDown_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define T_SWO_Pin GPIO_PIN_3
#define T_SWO_GPIO_Port GPIOB
#define PBLeft_Pin GPIO_PIN_4
#define PBLeft_GPIO_Port GPIOB
#define PBUp_Pin GPIO_PIN_5
#define PBUp_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
