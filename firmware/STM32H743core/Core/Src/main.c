/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dcmi.h"
#include "dma.h"
#include "fatfs.h"
#include "fdcan.h"
#include "i2c.h"
#include "sai.h"
#include "sdmmc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
uint16_t LCD_RAM[28900]__attribute__((section(".bss.ARM.__at_0XC0000000")));


//uint32_t jpeg_data_buf[30][30] __attribute__((section(".bss.ARM.__at_0XC0000000")));//SDRAM中的数据
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);
struct send_data my_data;
 bool usb_data_flag=0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
bool rgb=0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)// 
{   
	if(htim == &htim3)  //判断中断是否来自于定时器1
   {
			//__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);

		
		 if(rgb==1)
		 {
		//	LCD_Fill(50,50,100,100,31);
			// LCD_DrawRectangle(0,0,172,320,31);
		 }
		 else
		 {
		 // LCD_Fill(50,50,50,50,0XF800);
		 // LCD_DrawRectangle(50,50,100,100,31);
		 }
		__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
		 
   }
//	 else if(htim == &htim6)  //判断中断是否来自于定时器1
//   {
//		rgb=!rgb;
//		HAL_GPIO_WritePin(RGB3_GPIO_Port,RGB3_Pin,rgb);
//		HAL_GPIO_WritePin(GPIOG, RGB2_Pin, rgb);
//		HAL_GPIO_WritePin(GPIOG, RGB1_Pin, 0);
//	 }
}

void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{

		LCD_ShowPicture(0,0,170,170,LCD_RAM);

		__HAL_DCMI_ENABLE_IT(hdcmi,DCMI_IT_FRAME);

}
void DMA1_Stream1_IRQHandler(void)
{
//	for(int i=0;i<8;i++)
//	{
//		my_data.DATA[i]=LCD_RAM[i];
//	
//	}
//	CDC_Transmit_FS((uint8_t*)&my_data, sizeof(my_data));

}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	uint16_t cnmd[8]={0x01,0x02,0x03,0x04,0x1234,0x4567,0x7777,0x1235};
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
    my_data.tail[0]=0x00;
		my_data.tail[1]=0x00;
		my_data.tail[2]=0x80;
		my_data.tail[3]=0x7F;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DCMI_Init();
  MX_FMC_Init();
  MX_FDCAN1_Init();
  MX_I2C2_Init();
  MX_SAI1_Init();
  MX_SDMMC2_SD_Init();
  MX_SPI5_Init();
  MX_UART5_Init();
  MX_UART4_Init();
  MX_SPI6_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_USB_DEVICE_Init();
  MX_FATFS_Init();
  MX_I2C4_Init();
  /* USER CODE BEGIN 2 */
	
  __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim6);
	RCC->CFGR|=2<<22;//开启时钟引脚PA7输出HSE外部高速时钟 25MHZ
	LCD_Init();
	LCD_Fill(0,0,172,320,WHITE);//0XF800,0X7E0
	ov5640_Init();
	
	SDRAM_initialization_sequence();//SDRAM初始化序列
	HAL_SDRAM_ProgramRefreshRate(&hsdram1,1500);//隔1880个计数值进行刷新SDRAM，防止SDRAM数据丢失
	
	//LCD_ShowPicture(0,0,270,270,(uint8_t *)LCD_RAM);

	__HAL_DCMI_ENABLE_IT(&hdcmi, DCMI_IT_FRAME);
	 DCMI->CR |=DCMI_CR_CAPTURE;
//	uint16_t sdram_data;		
//	*(__IO uint16_t *)(SDRAM_BASE_ADDR + 0x100) = 399;
//	sdram_data = *(uint16_t *)(SDRAM_BASE_ADDR + 0x100) ;
	
	//LCD_ShowIntNum(10,10,(uint16_t)dcmi_line_buf[0][0],6,WHITE,BLACK,32);

	uint8_t isopen=f_open(&SDFile,"cnmd.txt",FA_READ);
	if(isopen==0)
	{
			char buf[10];
			
			uint32_t br;
			if(f_read(&SDFile,buf,10,&br)==0)
			{
			//	LCD_ShowIntNum(10,80,buf[0] ,6,WHITE,BLACK,32);
			}
	}
	
			
   HAL_DCMI_Start_DMA(&hdcmi,DCMI_MODE_CONTINUOUS,(uint32_t)LCD_RAM,14450);
		// LCD_ShowPicture(0,0,80,40,(uint8_t *)LCD_RAM);//172,320,172,320
	

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		
    /* USER CODE BEGIN 3 */

		
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  __HAL_RCC_SYSCFG_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 4;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_RCC_MCOConfig(RCC_MCO1, RCC_MCO1SOURCE_HSI, RCC_MCODIV_1);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_TIM;
  PeriphClkInitStruct.TIMPresSelection = RCC_TIMPRES_ACTIVATED;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
	
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
