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
#include "stm32f1xx_hal.h"

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
#define Wakeup_Pin GPIO_PIN_0
#define Wakeup_GPIO_Port GPIOA
#define X_RIGHT_Pin GPIO_PIN_2
#define X_RIGHT_GPIO_Port GPIOA
#define Y_RIGHT_Pin GPIO_PIN_3
#define Y_RIGHT_GPIO_Port GPIOA
#define NRF_CSN_Pin GPIO_PIN_4
#define NRF_CSN_GPIO_Port GPIOA
#define X_LEFT_Pin GPIO_PIN_0
#define X_LEFT_GPIO_Port GPIOB
#define Y_LFFT_Pin GPIO_PIN_1
#define Y_LFFT_GPIO_Port GPIOB
#define NRF_IRQ_Pin GPIO_PIN_10
#define NRF_IRQ_GPIO_Port GPIOB
#define NRF_CE_Pin GPIO_PIN_11
#define NRF_CE_GPIO_Port GPIOB
#define BUZZER_Pin GPIO_PIN_12
#define BUZZER_GPIO_Port GPIOB
#define BTN6_Pin GPIO_PIN_13
#define BTN6_GPIO_Port GPIOB
#define BTN5_Pin GPIO_PIN_14
#define BTN5_GPIO_Port GPIOB
#define BTN4_Pin GPIO_PIN_15
#define BTN4_GPIO_Port GPIOB
#define BTN3_Pin GPIO_PIN_8
#define BTN3_GPIO_Port GPIOA
#define BTN2_Pin GPIO_PIN_9
#define BTN2_GPIO_Port GPIOA
#define BTN1_Pin GPIO_PIN_10
#define BTN1_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_11
#define LED2_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOA
#define BTN_RIGHT_Pin GPIO_PIN_15
#define BTN_RIGHT_GPIO_Port GPIOA
#define BTN_LFFT_Pin GPIO_PIN_5
#define BTN_LFFT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
