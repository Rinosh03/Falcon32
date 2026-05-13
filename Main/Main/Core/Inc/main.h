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
#define LP_S_22HH_CS_Pin GPIO_PIN_13
#define LP_S_22HH_CS_GPIO_Port GPIOC
#define LP_S_22HH_SCK_Pin GPIO_PIN_13
#define LP_S_22HH_SCK_GPIO_Port GPIOB
#define LP_S_22HH_MISO_Pin GPIO_PIN_14
#define LP_S_22HH_MISO_GPIO_Port GPIOB
#define LP_S_22HH_MOSI_Pin GPIO_PIN_15
#define LP_S_22HH_MOSI_GPIO_Port GPIOB
#define LSM6DSR_CS_Pin GPIO_PIN_8
#define LSM6DSR_CS_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_4
#define LD2_GPIO_Port GPIOB
#define LD1_Pin GPIO_PIN_5
#define LD1_GPIO_Port GPIOB
#define M1_PWM_Pin GPIO_PIN_6
#define M1_PWM_GPIO_Port GPIOB
#define M2_PWM_Pin GPIO_PIN_7
#define M2_PWM_GPIO_Port GPIOB
#define M3_PWM_Pin GPIO_PIN_8
#define M3_PWM_GPIO_Port GPIOB
#define M4_PWM_Pin GPIO_PIN_9
#define M4_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
