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
#define INVERTOR_VOLTAGE_Pin GPIO_PIN_3
#define INVERTOR_VOLTAGE_GPIO_Port GPIOC
#define VOLTAGE_CONTROL_Pin GPIO_PIN_4
#define VOLTAGE_CONTROL_GPIO_Port GPIOA
#define INVERTOR_CURRENT_Pin GPIO_PIN_5
#define INVERTOR_CURRENT_GPIO_Port GPIOA
#define TEST_Pin GPIO_PIN_12
#define TEST_GPIO_Port GPIOB
#define CAN1_S_Pin GPIO_PIN_8
#define CAN1_S_GPIO_Port GPIOA
#define CAN2_S_Pin GPIO_PIN_9
#define CAN2_S_GPIO_Port GPIOA
#define CAN1_SHTDN_Pin GPIO_PIN_10
#define CAN1_SHTDN_GPIO_Port GPIOA
#define CAN_EN_Pin GPIO_PIN_15
#define CAN_EN_GPIO_Port GPIOA
#define PG_Pin GPIO_PIN_2
#define PG_GPIO_Port GPIOD
#define PON_Pin GPIO_PIN_3
#define PON_GPIO_Port GPIOB
#define CAN2_SHTDN_Pin GPIO_PIN_4
#define CAN2_SHTDN_GPIO_Port GPIOB
#define MOT_EN_Pin GPIO_PIN_7
#define MOT_EN_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
