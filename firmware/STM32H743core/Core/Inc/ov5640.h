#ifndef __OV5640_H
#define __OV5640_H
#include "main.h"

#define SCCB_SDA(x) HAL_GPIO_WritePin(GPIOB, SCCB_SDA_Pin, x)    
#define SCCB_SCL(x) HAL_GPIO_WritePin(GPIOB, SCCB_SCL_Pin, x)

//IO��������
#define SCCB_SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}	 //SDA PB7����ģʽ
#define SCCB_SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}    //SDA PB7���ģʽ

#define SCCB_READ_SDA    HAL_GPIO_ReadPin(GPIOB,SCCB_SDA_Pin)     //����SDA

#define OV5640_PWDN(n)  (n?HAL_GPIO_WritePin(GPIOH,DCMI_PWDN_Pin,1) :   HAL_GPIO_WritePin(GPIOH,DCMI_PWDN_Pin,0))//POWER DOWN�����ź�,��PCF8574_P2���� 
#define OV5640_RST(n)  	(n?HAL_GPIO_WritePin(GPIOH,DCMI_RST_Pin,1)  :   HAL_GPIO_WritePin(GPIOH,DCMI_RST_Pin,0))//��λ�����ź� 

#define SCCB_ID         0X60
#define OV5640_ADDR        		0X78		//OV5640��IIC��ַ

//OV5640��ؼĴ�������  
#define OV5640_CHIPIDH          0X300A  	//OV5640оƬID���ֽ�
#define OV5640_CHIPIDL          0X300B  	//OV5640оƬID���ֽ� void ov5640_Init();
void OV5640_RGB565_Mode(void) ;
void ov5640_Init();

uint8_t	OV5640_Focus_Init();
void OV5640_Light_Mode(uint8_t mode);	//�Զ�ģʽ
void OV5640_Color_Saturation(uint8_t sat);//ɫ�ʱ��Ͷ�0
void OV5640_Brightness(uint8_t bright);	//����0
void OV5640_Contrast(uint8_t contrast);		//�Աȶ�0
void OV5640_Sharpness(uint8_t sharp);//�Զ����
void ov5640_get_output_size(void);
uint8_t	OV5640_Focus_Constant();//���������Խ�
uint8_t OV5640_OutSize_Set(uint16_t offx,uint16_t offy,uint16_t width,uint16_t height);

#endif