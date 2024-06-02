#ifndef __OV5640_H
#define __OV5640_H
#include "main.h"

#define SCCB_SDA(x) HAL_GPIO_WritePin(GPIOB, SCCB_SDA_Pin, x)    
#define SCCB_SCL(x) HAL_GPIO_WritePin(GPIOB, SCCB_SCL_Pin, x)

//IO方向设置
#define SCCB_SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	 //SDA PB7输入模式
#define SCCB_SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}    //SDA PB7输出模式

#define SCCB_READ_SDA    HAL_GPIO_ReadPin(GPIOB,SCCB_SDA_Pin)     //输入SDA

#define OV5640_PWDN(n)  (n?HAL_GPIO_WritePin(GPIOH,DCMI_PWDN_Pin,1) :   HAL_GPIO_WritePin(GPIOH,DCMI_PWDN_Pin,0))//POWER DOWN控制信号,由PCF8574_P2控制 
#define OV5640_RST(n)  	(n?HAL_GPIO_WritePin(GPIOH,DCMI_RST_Pin,1)  :   HAL_GPIO_WritePin(GPIOH,DCMI_RST_Pin,0))//复位控制信号 

#define SCCB_ID         0X60
#define OV5640_ADDR        		0X78		//OV5640的IIC地址

//OV5640相关寄存器定义  
#define OV5640_CHIPIDH          0X300A  	//OV5640芯片ID高字节
#define OV5640_CHIPIDL          0X300B  	//OV5640芯片ID低字节 void ov5640_Init();
void OV5640_RGB565_Mode(void) ;
void ov5640_Init();

uint8_t	OV5640_Focus_Init();
void OV5640_Light_Mode(uint8_t mode);	//自动模式
void OV5640_Color_Saturation(uint8_t sat);//色彩饱和度0
void OV5640_Brightness(uint8_t bright);	//亮度0
void OV5640_Contrast(uint8_t contrast);		//对比度0
void OV5640_Sharpness(uint8_t sharp);//自动锐度
void ov5640_get_output_size(void);
uint8_t	OV5640_Focus_Constant();//启动持续对焦
uint8_t OV5640_OutSize_Set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);

#endif