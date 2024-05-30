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
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "st7789.h"
#include "lcd.h"
#include "sdram.h"
#include "sdctr.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern SPI_HandleTypeDef hspi6;
extern TIM_HandleTypeDef htim6;
extern SDRAM_HandleTypeDef hsdram1;

struct send_data{
	float DATA[1];
	uint8_t tail[4];


};


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
#define LCD_PWM_Pin GPIO_PIN_8
#define LCD_PWM_GPIO_Port GPIOI
#define LCD_DC_Pin GPIO_PIN_13
#define LCD_DC_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_14
#define LCD_CS_GPIO_Port GPIOC
#define LCD_RST_Pin GPIO_PIN_15
#define LCD_RST_GPIO_Port GPIOC
#define RGB3_Pin GPIO_PIN_11
#define RGB3_GPIO_Port GPIOD
#define RGB2_Pin GPIO_PIN_3
#define RGB2_GPIO_Port GPIOG
#define RGB1_Pin GPIO_PIN_6
#define RGB1_GPIO_Port GPIOG
#define KEY1_Pin GPIO_PIN_15
#define KEY1_GPIO_Port GPIOH
#define KEY2_Pin GPIO_PIN_0
#define KEY2_GPIO_Port GPIOI
#define KEY3_Pin GPIO_PIN_1
#define KEY3_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */
#define LCD_CS_H HAL_GPIO_WritePin(GPIOC, LCD_CS_Pin, 1);
#define LCD_CS_L HAL_GPIO_WritePin(GPIOC, LCD_CS_Pin, 0);

#define LCD_DC_H HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin, 1);//Êý¾Ý
#define LCD_DC_L HAL_GPIO_WritePin(GPIOC, LCD_DC_Pin, 0);//ÃüÁî

#define LCD_RST_H HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, 1);
#define LCD_RST_L HAL_GPIO_WritePin(GPIOC, LCD_RST_Pin, 0);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
