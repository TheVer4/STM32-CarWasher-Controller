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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Marquee_CS_Pin GPIO_PIN_4
#define Marquee_CS_GPIO_Port GPIOA
#define Marquee_CLS_Pin GPIO_PIN_5
#define Marquee_CLS_GPIO_Port GPIOA
#define Marquee_DIN_Pin GPIO_PIN_7
#define Marquee_DIN_GPIO_Port GPIOA
#define Button_1_Pin GPIO_PIN_7
#define Button_1_GPIO_Port GPIOE
#define Button_1_EXTI_IRQn EXTI9_5_IRQn
#define Button_5_Pin GPIO_PIN_8
#define Button_5_GPIO_Port GPIOE
#define Button_5_EXTI_IRQn EXTI9_5_IRQn
#define Button_2_Pin GPIO_PIN_9
#define Button_2_GPIO_Port GPIOE
#define Button_2_EXTI_IRQn EXTI9_5_IRQn
#define Button_6_Pin GPIO_PIN_10
#define Button_6_GPIO_Port GPIOE
#define Button_6_EXTI_IRQn EXTI15_10_IRQn
#define Button_3_Pin GPIO_PIN_11
#define Button_3_GPIO_Port GPIOE
#define Button_3_EXTI_IRQn EXTI15_10_IRQn
#define Button_7_Pin GPIO_PIN_12
#define Button_7_GPIO_Port GPIOE
#define Button_7_EXTI_IRQn EXTI15_10_IRQn
#define Button_4_Pin GPIO_PIN_13
#define Button_4_GPIO_Port GPIOE
#define Button_4_EXTI_IRQn EXTI15_10_IRQn
#define Button_8_Pin GPIO_PIN_14
#define Button_8_GPIO_Port GPIOE
#define Button_8_EXTI_IRQn EXTI15_10_IRQn
#define Relay_8_Pin GPIO_PIN_15
#define Relay_8_GPIO_Port GPIOA
#define Relay_4_Pin GPIO_PIN_10
#define Relay_4_GPIO_Port GPIOC
#define Relay_7_Pin GPIO_PIN_11
#define Relay_7_GPIO_Port GPIOC
#define Relay_3_Pin GPIO_PIN_12
#define Relay_3_GPIO_Port GPIOC
#define Relay_6_Pin GPIO_PIN_0
#define Relay_6_GPIO_Port GPIOD
#define Relay_2_Pin GPIO_PIN_1
#define Relay_2_GPIO_Port GPIOD
#define Relay_5_Pin GPIO_PIN_2
#define Relay_5_GPIO_Port GPIOD
#define Relay_1_Pin GPIO_PIN_3
#define Relay_1_GPIO_Port GPIOD
#define LED_Tim_Pin GPIO_PIN_0
#define LED_Tim_GPIO_Port GPIOE
#define LED_Balance_Pin GPIO_PIN_1
#define LED_Balance_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
