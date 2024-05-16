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
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct PID_Param{

	float P;
	float I;
	float error;
	float error_sum;
	float out;
	float target;

}PID;

extern ADC_HandleTypeDef hadc1;
extern uint16_t SPI_recive_data,SPI_send_data,Angel_begin;
#define DRV_EN_H HAL_GPIO_WritePin(GPIOC, DRV_EN_Pin, 1);
#define DRV_EN_L HAL_GPIO_WritePin(GPIOC, DRV_EN_Pin, 0);
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
#define I_A_Pin GPIO_PIN_0
#define I_A_GPIO_Port GPIOA
#define I_B_Pin GPIO_PIN_1
#define I_B_GPIO_Port GPIOA
#define I_C_Pin GPIO_PIN_2
#define I_C_GPIO_Port GPIOA
#define V_bus_Pin GPIO_PIN_3
#define V_bus_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define RGB1_Pin GPIO_PIN_4
#define RGB1_GPIO_Port GPIOC
#define RGB3_Pin GPIO_PIN_0
#define RGB3_GPIO_Port GPIOB
#define RGB2_Pin GPIO_PIN_1
#define RGB2_GPIO_Port GPIOB
#define AS_CS_Pin GPIO_PIN_12
#define AS_CS_GPIO_Port GPIOB
#define AS_SCK_Pin GPIO_PIN_13
#define AS_SCK_GPIO_Port GPIOB
#define AS_MISO_Pin GPIO_PIN_14
#define AS_MISO_GPIO_Port GPIOB
#define AS_MOSI_Pin GPIO_PIN_15
#define AS_MOSI_GPIO_Port GPIOB
#define DRV_EN_Pin GPIO_PIN_6
#define DRV_EN_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
