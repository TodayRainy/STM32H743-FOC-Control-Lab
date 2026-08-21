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
#include "stm32h7xx_hal.h"

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
#define SPI4_CS_Pin GPIO_PIN_4
#define SPI4_CS_GPIO_Port GPIOE
#define ADC_POT_Pin GPIO_PIN_0
#define ADC_POT_GPIO_Port GPIOC
#define AMP_IW_Pin GPIO_PIN_1
#define AMP_IW_GPIO_Port GPIOC
#define ADC_POWER_Pin GPIO_PIN_2
#define ADC_POWER_GPIO_Port GPIOC
#define KEY0_Pin GPIO_PIN_1
#define KEY0_GPIO_Port GPIOA
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOA
#define KEY2_Pin GPIO_PIN_3
#define KEY2_GPIO_Port GPIOA
#define LED0_Pin GPIO_PIN_4
#define LED0_GPIO_Port GPIOA
#define AMP_IV_Pin GPIO_PIN_0
#define AMP_IV_GPIO_Port GPIOB
#define AMP_IU_Pin GPIO_PIN_1
#define AMP_IU_GPIO_Port GPIOB
#define MT6835_PWM_Pin GPIO_PIN_11
#define MT6835_PWM_GPIO_Port GPIOB
#define SPI2_CS_Pin GPIO_PIN_12
#define SPI2_CS_GPIO_Port GPIOB
#define MT6835_CAL_EN_Pin GPIO_PIN_8
#define MT6835_CAL_EN_GPIO_Port GPIOD
#define MT6835_RST_Pin GPIO_PIN_9
#define MT6835_RST_GPIO_Port GPIOD
#define HALL_W_Pin GPIO_PIN_12
#define HALL_W_GPIO_Port GPIOD
#define HALL_V_Pin GPIO_PIN_13
#define HALL_V_GPIO_Port GPIOD
#define HALL_U_Pin GPIO_PIN_14
#define HALL_U_GPIO_Port GPIOD
#define ENC_Z_Pin GPIO_PIN_15
#define ENC_Z_GPIO_Port GPIOD
#define ENC_B_Pin GPIO_PIN_6
#define ENC_B_GPIO_Port GPIOC
#define ENC_A_Pin GPIO_PIN_7
#define ENC_A_GPIO_Port GPIOC
#define LCD_BLK_Pin GPIO_PIN_3
#define LCD_BLK_GPIO_Port GPIOD
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOD
#define LCD_DC_Pin GPIO_PIN_5
#define LCD_DC_GPIO_Port GPIOD
#define LCD_REST_Pin GPIO_PIN_6
#define LCD_REST_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
