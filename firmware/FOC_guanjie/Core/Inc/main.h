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
#define EN_12V_Pin GPIO_PIN_13
#define EN_12V_GPIO_Port GPIOC
#define I_A_Pin GPIO_PIN_0
#define I_A_GPIO_Port GPIOA
#define I_B_Pin GPIO_PIN_1
#define I_B_GPIO_Port GPIOA
#define I_C_Pin GPIO_PIN_2
#define I_C_GPIO_Port GPIOA
#define I_BUS_Pin GPIO_PIN_3
#define I_BUS_GPIO_Port GPIOA
#define RGB2_Pin GPIO_PIN_4
#define RGB2_GPIO_Port GPIOA
#define RGB1_Pin GPIO_PIN_5
#define RGB1_GPIO_Port GPIOA
#define A_L_Pin GPIO_PIN_7
#define A_L_GPIO_Port GPIOA
#define RGB3_Pin GPIO_PIN_4
#define RGB3_GPIO_Port GPIOC
#define B_L_Pin GPIO_PIN_0
#define B_L_GPIO_Port GPIOB
#define C_L_Pin GPIO_PIN_1
#define C_L_GPIO_Port GPIOB
#define AS_CS_Pin GPIO_PIN_12
#define AS_CS_GPIO_Port GPIOB
#define AS_SCK_Pin GPIO_PIN_13
#define AS_SCK_GPIO_Port GPIOB
#define AS_MISO_Pin GPIO_PIN_14
#define AS_MISO_GPIO_Port GPIOB
#define AS_MOSI_Pin GPIO_PIN_15
#define AS_MOSI_GPIO_Port GPIOB
#define A_H_Pin GPIO_PIN_8
#define A_H_GPIO_Port GPIOA
#define B_H_Pin GPIO_PIN_9
#define B_H_GPIO_Port GPIOA
#define C_H_Pin GPIO_PIN_10
#define C_H_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */
struct send_data{
	float DATA[9];
	uint8_t tail[4];


};

typedef struct PID_Param{

	float P;
	float I;
	float error;
	float error_sum;
	float out;

}PID;

extern struct send_data my_data;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
