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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"

#define AS_CS_H HAL_GPIO_WritePin(GPIOB, AS_CS_Pin, 1);
#define AS_CS_L HAL_GPIO_WritePin(GPIOB, AS_CS_Pin, 0);
#define squrt3 1.73205f
#define PWM_Max 4200
#define K1 0.0003834952f
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

struct send_data my_data;
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

uint8_t isready=0;
void Get_Aagel();
float Ksp;
float Vbus=16;

float cos_value,sin_value;//һ��PWM����ֻ��һ��

float Ualpha,Ubata;
float Iq_Target=0.01,Id_Target;
float Id_ref ,Iq_ref;
PID Iq_current;
float Uq=0,Ud=0.8; 

float Ia,Ib,Ic;

float Angel_Now=0,Angle_Last=0;
int angel_chazhi,laps;

float speed,speed_last;
uint16_t SPI_recive_data,SPI_send_data,SPI_recive_data_last,Angel_begin;

float Iq_current_loop()
{
		Iq_current.error=Iq_Target-Iq_ref;
		Iq_current.error_sum+=Iq_current.error;
		if(Iq_current.error_sum>5)
		{
				Iq_current.error_sum=5;
		}
		else if(Iq_current.error_sum<-5)
		{
				Iq_current.error_sum=-5;
		}
	  Iq_current.out = Iq_current.error * Iq_current.P + Iq_current.error_sum * Iq_current.I;
		return Iq_current.out;
}


void Park_change_Contrary()//Park��任
{

	if(isready==1)
	{
	Angel_Now=-K1*(SPI_recive_data-Angel_begin)*14.0f-87.9645943f*laps;
	}
	speed=Angel_Now-Angle_Last;
	speed=0.7f*speed+0.3*speed_last;
	if(speed>10||speed<-10)
	{
		speed=speed_last;
	}
	speed_last=speed;
	Angle_Last=Angel_Now;
	
	cos_value=arm_cos_f32(Angel_Now);
	sin_value=arm_sin_f32(Angel_Now);
	Ksp=7274.613f/Vbus;
	
	Ualpha=Ud*cos_value-Uq*sin_value;
	Ubata= Uq*cos_value+Ud*sin_value;
	
}


void Park_change(float I_alpha,float I_beta)//Park�任
{
	
		Id_ref= I_alpha*cos_value  + I_beta*sin_value;
		Iq_ref= -I_alpha*sin_value + I_beta*cos_value;
		my_data.DATA[1] = Id_ref;
		my_data.DATA[2] = Iq_ref;

	
}


float I_ref_ahp,I_ref_beta;
void Klark_change()//�����˱任
{
	I_ref_ahp=Ia-0.5f*(Ib+Ic);
	I_ref_beta=0.5774f*(Ib - Ic);	
	

	
	Park_change(I_ref_ahp,I_ref_beta);

}


float ta,tb,tc;
float X,Y,Z;
uint8_t sector;
void sector_judg()
{
	float u1,u2,u3;
	float t0,t1,t2,t3,t4,t5,t6,t7;
	u1=Ubata;
	u2=squrt3*Ualpha/2.f-Ubata/2.f;
	u3=-squrt3*Ualpha/2.f-Ubata/2.f;
		
	sector=(u1>0.0)+((u2>0.0)<<1)+((u3>0.0)<<2);	
	 X=Ksp*Ubata;
	 Y=Ksp*(-(squrt3/2.0f)*Ualpha-0.5f*Ubata);
	 Z=Ksp*((squrt3/2.0f)*Ualpha-0.5f*Ubata);
		

	if(sector==3)//��1����
	{
			t4=Z;
			t6=X;		
			t7=(PWM_Max-t4-t6)/2;
			ta=t4+t6+t7;
			tb=t6+t7;
			tc=t7;
	}
	else if(sector==1)//��2����
	{	
			t2=-Z;
			t6=-Y;
			t7=(PWM_Max-t2-t6)/2;
			ta=t6+t7;
			tb=t2+t6+t7;
			tc=t7;
	}
	else if(sector==5)//��3����
	{	
			t2=X;
			t3=Y;		
			t7=(PWM_Max-t2-t3)/2;
			ta=t7;
			tb=t2+t3+t7;
			tc=t3+t7;
	}
	else if(sector==4)//��4����
	{	
			t1=-X;
			t3=-Z;	
			t7=(PWM_Max-t1-t3)/2;
			ta=t7;
			tb=t3+t7;
			tc=t1+t3+t7;
	}
	else if(sector==6)//��5����
	{	
			t1=Y;
			t5=Z;	
			t7=(PWM_Max-t1-t5)/2;
			ta=t5+t7;
			tb=t7;
			tc=t1+t5+t7;
	}
	else if(sector==2)//��6����
	{	
			t4=-Y;
			t5=-X;	
			t7=(PWM_Max-t4-t5)/2;
			ta=t4+t5+t7;
			tb=t7;
			tc=t5+t7;
	}
	else 
	{
		ta=0;
		tb=0;
		tc=0;
	
	}
//	my_data.DATA[3]   =ta;
//	my_data.DATA[4]   =tb;
//	my_data.DATA[5]   =tc;

}
void SVPWM()
{
	Get_Aagel();
	Klark_change();
//	Angel_Now+=0.05f;
//	if(Angel_Now>6.28)
//	{
//		Angel_Now=0;
//	}
	Park_change_Contrary();
	sector_judg();


	TIM1->CCR1 =  (uint16_t)ta;
	TIM1->CCR2 =  (uint16_t)tb;
	TIM1->CCR3 =  (uint16_t)tc;

}

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */
uint16_t SPI_recive_data,SPI_send_data;
void Get_Aagel()	
{
	SPI_send_data|=0x3fff;	
	AS_CS_L
	HAL_SPI_TransmitReceive(&hspi2,(uint8_t*)&SPI_send_data,(uint8_t*)&SPI_recive_data,1,10);
	SPI_recive_data &= 0x3fff;
	AS_CS_H
	
	if(isready==0)
	{
		Angel_begin=SPI_recive_data;
		
	}
	angel_chazhi=SPI_recive_data-SPI_recive_data_last;
	
	if(abs(angel_chazhi)>13107) laps += (angel_chazhi>0)?-1:1;
	SPI_recive_data_last=SPI_recive_data;

}

 void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
 {

		if(hadc == &hadc1)
		{
				my_data.DATA[5]   =hadc1.Instance->JDR4;
				my_data.DATA[6]   =hadc1.Instance->JDR1;
	      my_data.DATA[7]   =hadc1.Instance->JDR2;
				my_data.DATA[8]   =hadc1.Instance->JDR3;
			
			 __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
			Ia=(hadc1.Instance->JDR1*0.0008f-1.65f)*0.227f;
			Ib=(hadc1.Instance->JDR2*0.0008f-1.65f)*0.227f;
			Ic=(hadc1.Instance->JDR3*0.0008f-1.65f)*0.227f;
			Klark_change();
		}
   
 
 }
 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
 uint8_t times200us;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//10KHz 0.0001ms
{   
	if(htim == &htim6)  //�ж��ж��Ƿ������ڶ�ʱ��1
   {
			SVPWM();
		 if(isready==1)
		 {
				 times200us++;
				 if(times200us==2)
				 {
						Uq=Iq_current_loop();
						my_data.DATA[3]   =Uq;
						
						times200us=0;
				 }
		 }
		 my_data.DATA[4]   = Iq_current.error;
//			__HAL_TIM_CLEAR_FLAG(&htim6,TIM_FLAG_UPDATE);
   }
}


void Pid_Param_init()
{
	Iq_current.P=0.194;
	Iq_current.I=0.366;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
		my_data.tail[0]=0x00;
		my_data.tail[1]=0x00;
		my_data.tail[2]=0x80;
		my_data.tail[3]=0x7F;
	
	Pid_Param_init();
	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_USB_Device_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
 HAL_GPIO_WritePin(GPIOC, EN_12V_Pin, 1);
 HAL_TIM_Base_Start_IT(&htim6);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
 HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
 HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
 HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
 HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
 HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);
 TIM1->CCR4=1;
 
 HAL_ADCEx_InjectedStart(&hadc1);
 __HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);
 Get_Aagel();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    /* USER CODE END WHILE */
	if(isready==0)
	{
		
		HAL_Delay(1000);
		isready=1;
		Ud=0 ;
		//Uq=0;
		
	}
    /* USER CODE BEGIN 3 */
	 CDC_Transmit_FS((uint8_t*)&my_data, sizeof(my_data));
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOA, RGB2_Pin|RGB1_Pin, GPIO_PIN_RESET);
//		HAL_Delay(500);
//		HAL_GPIO_WritePin(GPIOA, RGB2_Pin|RGB1_Pin, 1);
		//HAL_UART_Transmit(&hlpuart1,(uint8_t*)&my_data,sizeof(my_data),0xff);

	
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV6;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_InjectionConfTypeDef sConfigInjected = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_1;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
  sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
  sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
  sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
  sConfigInjected.InjectedOffset = 0;
  sConfigInjected.InjectedNbrOfConversion = 4;
  sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
  sConfigInjected.AutoInjectedConv = DISABLE;
  sConfigInjected.QueueInjectedContext = DISABLE;
  sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
  sConfigInjected.ExternalTrigInjecConvEdge = ADC_EXTERNALTRIGINJECCONV_EDGE_RISINGFALLING;
  sConfigInjected.InjecOversamplingMode = DISABLE;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_2;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Injected Channel
  */
  sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
  sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
  if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  htim1.Init.Period = 4200;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 55;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */
 
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 16800;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, EN_12V_Pin|RGB3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, RGB2_Pin|RGB1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(AS_CS_GPIO_Port, AS_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_12V_Pin RGB3_Pin */
  GPIO_InitStruct.Pin = EN_12V_Pin|RGB3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RGB2_Pin RGB1_Pin */
  GPIO_InitStruct.Pin = RGB2_Pin|RGB1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : AS_CS_Pin */
  GPIO_InitStruct.Pin = AS_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(AS_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
