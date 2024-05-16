#include "st7789.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE           	 0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
#define LIGHTGREEN     	 0X841F //浅绿色
#define LGRAY 			     0XC618 //浅灰色(PANNEL),窗体背景色
#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
//134400/100=1344
#define LCD_Buf_Size 1344


static uint16_t g_width;
static uint16_t g_height;
uint8_t byte,SPI_recive_data;
uint8_t lcd_buf[LCD_Buf_Size];

void LCD_Writ_Bus(uint8_t *dat,uint16_t size) 
{	
	 LCD_CS_L;
	if(HAL_SPI_Transmit(&hspi6,dat,size,1000)!= HAL_OK)
	{
		Error_Handler();
	}
	LCD_CS_H;

}
void LCD_WR_DATA8(uint8_t dat)//发送数据
{
	LCD_DC_H
	LCD_Writ_Bus(&dat,1);
}
void LCD_WR_REG(uint8_t dat)//发送命令
{
	LCD_DC_L;
	LCD_Writ_Bus(&dat,1);
}

void LCD_WR_DATA(uint16_t dat)//发送16位数据
{
	LCD_WR_DATA8(dat>>8);
	LCD_WR_DATA8(dat&0xFF);
}

void LCD_Address_Set(uint16_t x1,uint16_t y1,uint16_t x2,uint16_t y2)//设置屏幕显示方向
{
	
		LCD_WR_REG(0x2a);//列地址设置
		LCD_WR_DATA(x1+34);
		LCD_WR_DATA(x2+34);
		LCD_WR_REG(0x2b);//行地址设置
		LCD_WR_DATA(y1);
		LCD_WR_DATA(y2);
		LCD_WR_REG(0x2C);//储存器写
	
	
}

void LCD_Clear(uint16_t color)
{
		uint16_t i, j;
		uint16_t k;
    uint8_t data[2] = {0};  //color是16bit的，每个像素点需要两个字节的显存
 
    /* 将16bit的color值分开为两个单独的字节 */
    data[0] = color >> 8;
    data[1] = color&0xff;
 
    /* 显存的值需要逐字节写入 */
    for(j = 0;j < LCD_Buf_Size/2; j++)
    {
        lcd_buf[j * 2] =  data[0];
        lcd_buf[j * 2 + 1] =  data[1];
    }
    /* 指定显存操作地址为全屏幕 */
    LCD_Address_Set(0,0,210-1,320-1);
    /* 指定接下来的数据为数据 */
    LCD_DC_H
    /* 将显存缓冲区的数据全部写入缓冲区 */
    for(i = 0;i <(67200/LCD_Buf_Size); i++)
    {
				//HAL_Delay(1);
			for(k=0;k<10;k++){}
        LCD_Writ_Bus(lcd_buf, (uint16_t)LCD_Buf_Size);
    }


}
void LCD_Fill(uint16_t xsta,uint16_t ysta,uint16_t xend,uint16_t yend,uint16_t color)
{          
	uint16_t i,j; 
	LCD_Address_Set(xsta,ysta,xend-1,yend-1);//设置显示范围
	for(i=ysta;i<yend;i++)
	{													   	 	
		for(j=xsta;j<xend;j++)
		{
			LCD_WR_DATA(color);
		}
	} 					  	    
}





void LCD_Init(void)
{

	

	
//	LCD_RST_L
//	HAL_Delay(300);
	//RST拉低移到GPIOinit中了，所以这里只需要拉高就行
	LCD_RST_H;
	HAL_Delay(500);

	
	//************* Start Initial Sequence **********//
	LCD_WR_REG(0x11); //Sleep out 
	HAL_Delay(120);              //Delay 120ms 
	//************* Start Initial Sequence **********// 
	LCD_WR_REG(0x36);
  LCD_WR_DATA8(0x00);
	
	LCD_WR_REG(0x3A);     
	LCD_WR_DATA8(0x05);   
 
	LCD_WR_REG(0xB2);     
	LCD_WR_DATA8(0x0c);   
	LCD_WR_DATA8(0x0c);   
	LCD_WR_DATA8(0x00);   
	LCD_WR_DATA8(0x33);   
	LCD_WR_DATA8(0x33);   
 
	LCD_WR_REG(0xB7);     
	LCD_WR_DATA8(0x72);   
 
	LCD_WR_REG(0xBB);     
	LCD_WR_DATA8(0x3d);   //2b
 
	LCD_WR_REG(0xC0);     
	LCD_WR_DATA8(0x2C);   
 
	LCD_WR_REG(0xC2);     
	LCD_WR_DATA8(0x01); 
  LCD_WR_DATA8(0xFF);   
 
	LCD_WR_REG(0xC3);     
	LCD_WR_DATA8(0x19);   
 
	LCD_WR_REG(0xC4);     
	LCD_WR_DATA8(0x20);   //VDV, 0x20:0v
 
	LCD_WR_REG(0xC6);     
	LCD_WR_DATA8(0x0f);   //0x13:60Hz   
 
	LCD_WR_REG(0xD0);     
	LCD_WR_DATA8(0xA4);   
	LCD_WR_DATA8(0xA1);   
 
 
 
//	LCD_WR_REG(0xD6);     
//	LCD_WR_DATA8(0xA1);   //sleep in后，gate输出为GND
 
	 LCD_WR_REG(0xE0);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2B);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x54);
    LCD_WR_DATA8(0x4C);
    LCD_WR_DATA8(0x18);
    LCD_WR_DATA8(0x0D);
    LCD_WR_DATA8(0x0B);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x23);
    /* 电压设置 */
    LCD_WR_REG(0xE1);
    LCD_WR_DATA8(0xD0);
    LCD_WR_DATA8(0x04);
    LCD_WR_DATA8(0x0C);
    LCD_WR_DATA8(0x11);
    LCD_WR_DATA8(0x13);
    LCD_WR_DATA8(0x2C);
    LCD_WR_DATA8(0x3F);
    LCD_WR_DATA8(0x44);
    LCD_WR_DATA8(0x51);
    LCD_WR_DATA8(0x2F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x1F);
    LCD_WR_DATA8(0x20);
    LCD_WR_DATA8(0x23);
    /* 显示开 */
    LCD_WR_REG(0x21);
    LCD_WR_REG(0x29);
}

