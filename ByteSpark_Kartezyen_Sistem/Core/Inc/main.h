/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#define X_ON_OFF_Pin GPIO_PIN_13
#define X_ON_OFF_GPIO_Port GPIOC
#define LimitSwitch_X2_Pin GPIO_PIN_14
#define LimitSwitch_X2_GPIO_Port GPIOC
#define POT1_Pin GPIO_PIN_0
#define POT1_GPIO_Port GPIOA
#define POT2_Pin GPIO_PIN_1
#define POT2_GPIO_Port GPIOA
#define POT3_Pin GPIO_PIN_2
#define POT3_GPIO_Port GPIOA
#define LimitSwitch_Y1_Pin GPIO_PIN_3
#define LimitSwitch_Y1_GPIO_Port GPIOA
#define LimitSwitch_Y2_Pin GPIO_PIN_4
#define LimitSwitch_Y2_GPIO_Port GPIOA
#define LED4_Pin GPIO_PIN_5
#define LED4_GPIO_Port GPIOA
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOA
#define LED3_Pin GPIO_PIN_7
#define LED3_GPIO_Port GPIOA
#define LED2_Pin GPIO_PIN_0
#define LED2_GPIO_Port GPIOB
#define HC05_TX3_Pin GPIO_PIN_10
#define HC05_TX3_GPIO_Port GPIOB
#define HC05_RX3_Pin GPIO_PIN_11
#define HC05_RX3_GPIO_Port GPIOB
#define STEP_X_Pin GPIO_PIN_13
#define STEP_X_GPIO_Port GPIOB
#define EN_X_Pin GPIO_PIN_14
#define EN_X_GPIO_Port GPIOB
#define DIR_X_Pin GPIO_PIN_15
#define DIR_X_GPIO_Port GPIOB
#define HC05_TX1_Pin GPIO_PIN_9
#define HC05_TX1_GPIO_Port GPIOA
#define HC05_RX1_Pin GPIO_PIN_10
#define HC05_RX1_GPIO_Port GPIOA
#define Y_YON_Pin GPIO_PIN_11
#define Y_YON_GPIO_Port GPIOA
#define Y_ON_OFF_Pin GPIO_PIN_15
#define Y_ON_OFF_GPIO_Port GPIOA
#define STEP_Y_Pin GPIO_PIN_3
#define STEP_Y_GPIO_Port GPIOB
#define EN_Y_Pin GPIO_PIN_4
#define EN_Y_GPIO_Port GPIOB
#define DIR_Y_Pin GPIO_PIN_5
#define DIR_Y_GPIO_Port GPIOB
#define X_YON_Pin GPIO_PIN_7
#define X_YON_GPIO_Port GPIOB
#define LimitSwitch_X1_Pin GPIO_PIN_9
#define LimitSwitch_X1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
