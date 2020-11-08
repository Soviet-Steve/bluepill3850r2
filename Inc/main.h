/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#define V_BAT_Pin GPIO_PIN_4
#define V_BAT_GPIO_Port GPIOA
#define HEADLIGHTS_Pin GPIO_PIN_5
#define HEADLIGHTS_GPIO_Port GPIOA
#define LEFT_INDICATOR_Pin GPIO_PIN_6
#define LEFT_INDICATOR_GPIO_Port GPIOA
#define RIGHT_INDICATOR_Pin GPIO_PIN_7
#define RIGHT_INDICATOR_GPIO_Port GPIOA
#define WC_EN_Pin GPIO_PIN_0
#define WC_EN_GPIO_Port GPIOB
#define WC_ADC_Pin GPIO_PIN_1
#define WC_ADC_GPIO_Port GPIOB
#define MOTOR_PWM_Pin GPIO_PIN_10
#define MOTOR_PWM_GPIO_Port GPIOB
#define SERVO_PWM_Pin GPIO_PIN_11
#define SERVO_PWM_GPIO_Port GPIOB
#define SONAR_ECHO_Pin GPIO_PIN_12
#define SONAR_ECHO_GPIO_Port GPIOB
#define SONAR_ECHO_EXTI_IRQn EXTI15_10_IRQn
#define SONAR_TRIG_Pin GPIO_PIN_13
#define SONAR_TRIG_GPIO_Port GPIOB
#define HALL_EFFECT_Pin GPIO_PIN_4
#define HALL_EFFECT_GPIO_Port GPIOB
#define HALL_EFFECT_EXTI_IRQn EXTI4_IRQn
#define MOTOR_INA_Pin GPIO_PIN_8
#define MOTOR_INA_GPIO_Port GPIOB
#define MOTOR_INB_Pin GPIO_PIN_9
#define MOTOR_INB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
