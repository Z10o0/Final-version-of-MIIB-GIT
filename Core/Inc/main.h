/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_ll_bdma.h"
#include "stm32h7xx_ll_dma.h"
#include "stm32h7xx_ll_rcc.h"
#include "stm32h7xx_ll_crs.h"
#include "stm32h7xx_ll_bus.h"
#include "stm32h7xx_ll_system.h"
#include "stm32h7xx_ll_exti.h"
#include "stm32h7xx_ll_cortex.h"
#include "stm32h7xx_ll_utils.h"
#include "stm32h7xx_ll_pwr.h"
#include "stm32h7xx_ll_spi.h"
#include "stm32h7xx_ll_tim.h"
#include "stm32h7xx_ll_usart.h"
#include "stm32h7xx_ll_gpio.h"

#if defined(USE_FULL_ASSERT)
#include "stm32_assert.h"
#endif /* USE_FULL_ASSERT */

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
#define CS19_Pin LL_GPIO_PIN_4
#define CS19_GPIO_Port GPIOC
#define CS20_Pin LL_GPIO_PIN_5
#define CS20_GPIO_Port GPIOC
#define CS25_Pin LL_GPIO_PIN_0
#define CS25_GPIO_Port GPIOB
#define CS26_Pin LL_GPIO_PIN_1
#define CS26_GPIO_Port GPIOB
#define CS31_Pin LL_GPIO_PIN_13
#define CS31_GPIO_Port GPIOF
#define CS32_Pin LL_GPIO_PIN_14
#define CS32_GPIO_Port GPIOF
#define CS22_Pin LL_GPIO_PIN_15
#define CS22_GPIO_Port GPIOF
#define CS21_Pin LL_GPIO_PIN_0
#define CS21_GPIO_Port GPIOG
#define CS28_Pin LL_GPIO_PIN_1
#define CS28_GPIO_Port GPIOG
#define CS27_Pin LL_GPIO_PIN_7
#define CS27_GPIO_Port GPIOE
#define CS33_Pin LL_GPIO_PIN_8
#define CS33_GPIO_Port GPIOE
#define CS34_Pin LL_GPIO_PIN_9
#define CS34_GPIO_Port GPIOE
#define CS23_Pin LL_GPIO_PIN_10
#define CS23_GPIO_Port GPIOE
#define CS24_Pin LL_GPIO_PIN_11
#define CS24_GPIO_Port GPIOE
#define CS29_Pin LL_GPIO_PIN_14
#define CS29_GPIO_Port GPIOE
#define CS30_Pin LL_GPIO_PIN_15
#define CS30_GPIO_Port GPIOE
#define CS36_Pin LL_GPIO_PIN_12
#define CS36_GPIO_Port GPIOB
#define CS35_Pin LL_GPIO_PIN_13
#define CS35_GPIO_Port GPIOB
#define CS_IMU1_Pin LL_GPIO_PIN_9
#define CS_IMU1_GPIO_Port GPIOC
#define CS_IMU2_Pin LL_GPIO_PIN_8
#define CS_IMU2_GPIO_Port GPIOA
#define CS_IMU3_Pin LL_GPIO_PIN_2
#define CS_IMU3_GPIO_Port GPIOD
#define CS_IMU4_Pin LL_GPIO_PIN_3
#define CS_IMU4_GPIO_Port GPIOD
#define CS_IMU5_Pin LL_GPIO_PIN_9
#define CS_IMU5_GPIO_Port GPIOG
#define CS_IMU6_Pin LL_GPIO_PIN_15
#define CS_IMU6_GPIO_Port GPIOG
#ifndef NVIC_PRIORITYGROUP_0
#define NVIC_PRIORITYGROUP_0         ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority,
                                                                 4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1         ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority,
                                                                 3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2         ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority,
                                                                 2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3         ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority,
                                                                 1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4         ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority,
                                                                 0 bit  for subpriority */
#endif

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
