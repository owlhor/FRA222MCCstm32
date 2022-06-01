/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define TIM2_CH1_VINCp_Pin GPIO_PIN_0
#define TIM2_CH1_VINCp_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define Pwr_Sense_Pin GPIO_PIN_1
#define Pwr_Sense_GPIO_Port GPIOB
#define Mot_dir_Pin GPIO_PIN_2
#define Mot_dir_GPIO_Port GPIOB
#define PLamp_Green_Pin GPIO_PIN_13
#define PLamp_Green_GPIO_Port GPIOB
#define PLamp_Blue_Pin GPIO_PIN_14
#define PLamp_Blue_GPIO_Port GPIOB
#define PLamp_Yellow_Pin GPIO_PIN_15
#define PLamp_Yellow_GPIO_Port GPIOB
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define EXTI10_Stop_Pin GPIO_PIN_10
#define EXTI10_Stop_GPIO_Port GPIOC
#define EXTI10_Stop_EXTI_IRQn EXTI15_10_IRQn
#define EXTI11_EMER_Pin GPIO_PIN_11
#define EXTI11_EMER_GPIO_Port GPIOC
#define EXTI11_EMER_EXTI_IRQn EXTI15_10_IRQn
#define Stop_Sense_Pin GPIO_PIN_12
#define Stop_Sense_GPIO_Port GPIOC
#define EXTI2_SetZero_Pin GPIO_PIN_2
#define EXTI2_SetZero_GPIO_Port GPIOD
#define EXTI2_SetZero_EXTI_IRQn EXTI2_IRQn
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define TIM4_CH1_PWMMOT_Pin GPIO_PIN_6
#define TIM4_CH1_PWMMOT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
