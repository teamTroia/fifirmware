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
#define LED0_Pin GPIO_PIN_0
#define LED0_GPIO_Port GPIOC
#define LED1_Pin GPIO_PIN_1
#define LED1_GPIO_Port GPIOC
#define M1ADC_Pin GPIO_PIN_3
#define M1ADC_GPIO_Port GPIOA
#define M1NSLEEP_Pin GPIO_PIN_4
#define M1NSLEEP_GPIO_Port GPIOA
#define M1DIR_Pin GPIO_PIN_5
#define M1DIR_GPIO_Port GPIOA
#define M2ADC_Pin GPIO_PIN_7
#define M2ADC_GPIO_Port GPIOA
#define M2NSLEEP_Pin GPIO_PIN_2
#define M2NSLEEP_GPIO_Port GPIOB
#define M2DIR_Pin GPIO_PIN_10
#define M2DIR_GPIO_Port GPIOB
#define Encoder2_csn_Pin GPIO_PIN_12
#define Encoder2_csn_GPIO_Port GPIOB
#define ID_BIT0_Pin GPIO_PIN_6
#define ID_BIT0_GPIO_Port GPIOC
#define ID_BIT1_Pin GPIO_PIN_7
#define ID_BIT1_GPIO_Port GPIOC
#define ID_BIT2_Pin GPIO_PIN_8
#define ID_BIT2_GPIO_Port GPIOC
#define LED2_Pin GPIO_PIN_9
#define LED2_GPIO_Port GPIOC
#define NRF_CE_Pin GPIO_PIN_9
#define NRF_CE_GPIO_Port GPIOA
#define NRF_IRQ_Pin GPIO_PIN_10
#define NRF_IRQ_GPIO_Port GPIOA
#define MPU_INT_Pin GPIO_PIN_5
#define MPU_INT_GPIO_Port GPIOB
#define Encoder1_csn_Pin GPIO_PIN_9
#define Encoder1_csn_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
