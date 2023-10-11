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
#define KIT_LED_Pin GPIO_PIN_13
#define KIT_LED_GPIO_Port GPIOC
#define LEFT_SENSOR_LED_Pin GPIO_PIN_12
#define LEFT_SENSOR_LED_GPIO_Port GPIOB
#define RIGHT_SENSOR_LED_Pin GPIO_PIN_13
#define RIGHT_SENSOR_LED_GPIO_Port GPIOB
#define BUTTON_Pin GPIO_PIN_7
#define BUTTON_GPIO_Port GPIOB
#define LCD_CLK_Pin GPIO_PIN_8
#define LCD_CLK_GPIO_Port GPIOB
#define LCD_DIO_Pin GPIO_PIN_9
#define LCD_DIO_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef enum
{
  WARMING_LEDS_OFF,
  WARMING_UP,  
  SAMPLING_LEDS_OFF,
  SAMPLING  
} SAMPLING_STATE;



#define BUTTON_PRESS_FLAG         (0x0001)
#define SENSOR_TRIGGER_FLAG       (0x0002)
#define TIMER_OVERFLOW_FLAG       (0x0004)
#define TIMED_EVENT_FLAG          (0x0008)
#define FLAGS_ALL                 (0x000F)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
